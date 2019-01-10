//
//	messaging.h  --  message support for QNX
//
//	John Nagle
//	Team Overbot
//	August, 2003
//
//	A message-passing library for use with the "watchdog" program.
//	The "watchdog" program knows the PIDs of all its children, and
//	provides a server via which they can be found by name.
//	The name used is the "ID" field used in the watchdog file,
//	and is the program name by default. So, to establish communications
//	with another process, you only need to know its ID.
//
//	Inline implementations
//	To be moved to a library if they get too big.
//
//	Typical usage is
//
//		MsgPort actuatorport("ACTUATORS",  0.2);								 // set up the connection
//		...
//		actuatorport.MsgSend(cmd,sizeof(cmd), reply,sizeof(reply));	// send a command to the server
//
#ifndef MESSAGING_IMPL_H
#define MESSAGING_IMPL_H
#include <sys/neutrino.h>
#include <sys/netmgr.h>
#include <sys/utsname.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <errno.h>
#include <assert.h>
#include <process.h>
#include <pthread.h>
//
//	Inline implementations
//
//	(Move to a library if they get too big)
//
//	These functions all return -1 on failure, with an error in errno.
//	This is for consistency with QNX MsgSend, etc.
//
//
//
//	getnodend  -- get usable node descriptor given node identifier string
//
//	This works even if the calling process is spawned from a remote node.
//
//	When a process is spawned by a remote root node, calls to the "netmgr_"
//	functions behave as if the process is on the remote root node.  But
//	calls to ConnectAttach require the node number as valid for the
//	local node. Thus, some conversions are required.
//
inline int getnodend(const char* remotename)
{	assert(remotename);														// must be valid remote node name
	//	Get local node name
	struct utsname thisnodeinfo;											// local info
	if (uname(&thisnodeinfo) < 0) return(-1);							// get local node info
	int localrootnd = netmgr_strtond(thisnodeinfo.nodename,0);	// get local node node descriptor in root node space
	if (localrootnd < 0) { assert(errno); return(-1); }				// handle errror
	int remoterootnd = netmgr_strtond(remotename,0);			// get remote node node descriptor in root node space
	if (remoterootnd < 0) { assert(errno); return(-1); }			// handle errror
	//	Now we have all the node descriptors.
	//	Translate the remote node descriptor into local node space.
	int remotelocalnd = netmgr_remote_nd(localrootnd, remoterootnd);	// translate
	if (remotelocalnd < 0) { assert(errno); return(-1); }			// handle error
	return(remotelocalnd);														// success, return valid local node descriptor for remote node
}
//
//	Constructor
//
inline MsgPort::MsgPort(const char* name, double timeout)		
	: m_name(name),  m_timeoutns(uint64_t(timeout*1.0e9)), m_watchdogcoid(-1), m_verbose(false)
{	int stat = setpriority();														// set priority in constructor
	if (stat) perror("Unable to set thread priority");
}
//
//	Destructor
//
inline MsgPort::~MsgPort()
{	disconnectwatchdogcoid();												// disconnect if necessary
}
//	
//	setpriority -- get priority from "PRI" environment variable
//
inline int MsgPort::setpriority()
{	return(setpriority(0));														// set to "PRI" value
}
//
//	Set priority relative to "PRI" environment variable
//
//	Applies to current thread and later children of it
//	
inline int MsgPort::setpriority(int relval)
{	const char* prival = getenv("PRI");									// get environment priority value
	if (!prival) return(0);															// no priority, no action
	int pri = atoi(prival);															// convert to integer
	if (pri < 1 || pri > 62)														// priority out of range
	{	errno = EINVAL; return(-1);	}										// fails
	pri += relval;																	// apply adjustment
	if (pri < 1 || pri > 62)														// priority out of range
	{	errno = EINVAL; return(-1);	}										// fails
	const char* maxprival = getenv("MAXPRI");						// get maximum priority value
	if (maxprival) 
	{	int maxpri = atoi(maxprival);										// get max allowed priority
		if (pri > maxpri)
		{	pri = maxpri;	 } 														// limit priority
	}
	if (pri < 1 || pri > 62)														// priority out of range
	{	errno = EINVAL; return(-1);	}										// fails
	//	We have a priority value; set it.
	struct sched_param sched;												// scheduling structure
	sched.sched_priority = pri;												// set the priority
	int policy = (pri > 12) ? SCHED_FIFO : SCHED_RR;				// greater than 12 is real time
	return(pthread_setschedparam(pthread_self(), policy, &sched));	// set thread priority	
}
//
//	getwatchdogportid  -- get watchdog process port ID
//
//	This information comes from environment variables.
//	The watchdog may be on a different node, so we return the
//	local node ID valid for this node, to allow internode communication.
//
inline int MsgPort::getwatchdogportid(MsgPortID& watchport)
{
	//	***IS GETENV THREAD SAFE?*** POSIX says yes, QNX docs are ambiguous
	const char* watchdogpid = getenv("WATCHDOG_PID");	// get watchdog PID
	if (!watchdogpid) {  errno = EDESTADDRREQ;	return(-1); }		// fails with "destination address required"
	watchport.m_pid = atoi(watchdogpid);							// get watchdog PID

	const char* watchdogchid = getenv("WATCHDOG_CHID");	// get watchdog channel ID
	if (!watchdogchid)  {  errno = EDESTADDRREQ;	return(-1); }		// fails with "destination address required"
	watchport.m_chid = atoi(watchdogchid);						// get watchdog channel ID

	const char* watchdognodename = getenv("WATCHDOG_ND");	// get watchdog node name
	if (!watchdognodename)  {  errno = EDESTADDRREQ;	return(-1); }		// fails with "destination address required"
	int nd = getnodend(watchdognodename);					// look up node name
	if (nd < 0)																		// if fail
	{	 if (errno == ENOTSUP)												// if not supported
		{	nd = 0;	}																// QNET networking not running, assume non-networked operation
		else  {	assert(errno);  return(-1);	}							// fails
	}
	watchport.m_nodeid = nd;											// save node ID usable locally
	return(0);																		// success
}
//
//	getwatchdogcoid -- get connection ID for watchdog
//
//	Connects if necessary.
//
inline int MsgPort::getwatchdogcoid()
{	if (m_watchdogcoid >0 ) return(m_watchdogcoid);		// valid, return it
	MsgPortID watchport;													// port ID info for watchdog
	int stat = getwatchdogportid(watchport);						// look up the watchdog
	if (stat) {	assert(errno); return(-1);	}							// handle error
	m_watchdogcoid = ConnectAttach(watchport.m_nodeid, watchport.m_pid, watchport.m_chid, _NTO_SIDE_CHANNEL, 0);	// get channel to watchdog
	if (m_watchdogcoid < 0)
	{	if (verbose()) { perror("MsgPort cannot communicate with watchdog"); }
		assert(errno);
		return(-1);																	// fails
	}
	return(m_watchdogcoid);												// success, return watchdog connection ID
}
//
//	disconnectwatchdogcoid  -- disconnect watchdog connection
//
//	This is called whenever a message to the watchdog fails, so we can reconnect next time.
//
inline void MsgPort::disconnectwatchdogcoid()
{	if (m_watchdogcoid >= 0) return;									// nothing to do
	::ConnectDetach(m_watchdogcoid);								// detach, ignoring status
	m_watchdogcoid = -1;													// now detached
}
//
//	watchdogreset  -- reset the watchdog
//
inline int MsgPort::watchdogreset()
{
	//	Build transaction to talk to watchdog
	WatchdogMsgOK okmsg;
	okmsg.m_msgtype = WatchdogMsgOK::k_msgtype;		// set message type (no constructor so unions will work)
	int watchdogcoid = getwatchdogcoid();						// get watchdog channel ID
	if (watchdogcoid < 0)													// if unable to get watchdog ID
	{	assert(errno);	return(-1);	}									// report error
	int stat = MsgSend(watchdogcoid, okmsg, okmsg);		// tell server we are OK
	if (stat < 0) { assert(errno); return(-1); }						// if watchdog unhappy, it is an error to this caller
	return(0);																		// success
}
//
//	Dump  -- dump to standard output
//
inline void MsgPort::Dump()
{													
	printf("  MsgPort: m_name=\"%s\"  m_watchdogcoid=0x%08x\n",m_name? m_name : "???",m_watchdogcoid);						// dump self
}
//
//	registerserverwithwatchdog  -- set a channel ID in the watchdog
//
//	Node and process ID are determined by who is calling.
//
inline int MsgServerPort::registerserverwithwatchdog(int chid)
{
	//	Build transaction to talk to watchdog
	WatchdogMsgSV svmsg;
	svmsg.m_msgtype = WatchdogMsgSV::k_msgtype;
	svmsg.m_chid = chid;
	//	khianhao
	//	added to copy name into msg packet
	const char * name = getname();
	if (!name)																		// if no server ID known
	{	errno = EFAULT ; return(-1);	}									// avoid reference to null
	strncpy(svmsg.m_name, name, sizeof(svmsg.m_name));	// copy name of server

    // set channel ID
	int watchdogcoid = getwatchdogcoid();						// get watchdog channel ID
	if (watchdogcoid < 0)													// if unable to get watchdog ID
	{	assert(errno); return(-1);	}										// report error
	int stat = MsgSend(watchdogcoid, svmsg, svmsg);		// ask server for help
	if (stat < 0) { disconnectwatchdogcoid();	 }					// disconnect on comm error
	if (stat) { assert(errno); return(-1); }							// if watchdog unhappy, it is an error to this caller
	return(0);																		// success
}
//
//	ChannelCreate  -- create a channel and tell the watchdog about it
//
//	Only servers can do this.
//
inline int MsgServerPort::ChannelCreate(int flags)
{
	ChannelDestroy();														// get rid of channel if any
	int chid = ::ChannelCreate(flags);									// create a channel
	if (chid < 0) return(-1);													// fails
	int stat = registerserverwithwatchdog(chid);				// register with server
	if (stat)																		// if register fails
	{
		ChannelDestroy();													// drop channel
		return(-1);
	}
	m_chid = chid;																// save channel ID locally
	return(0);																		// success
}
//
//	ChannelDestroy -- destroy channel, if any
//
//	Only servers should do this.
//
inline void MsgServerPort::ChannelDestroy()
{	 if (m_chid < 0) return;																// nothing to do
	::ChannelDestroy(m_chid);														// destroy channel
	m_chid = -1;																				// clear
}
//
//	MsgReceive -- receive a message, setting up the connection if necessary
//
inline int MsgServerPort::MsgReceive(void* msg, int msgbytes, struct _msg_info* info)
{
	//	Set timeout for next kernel call. Call will return ETIMEDOUT status if timeout.
	const uint64_t timeoutns = gettimeoutns();					// timeout in nanoseconds
	if (timeoutns != 0)														// if timeout
	{	TimerTimeout( CLOCK_REALTIME,  _NTO_TIMEOUT_RECEIVE, NULL, &timeoutns, NULL );	}
	return(::MsgReceive(m_chid, msg, msgbytes, info));	// send the message
}
//
//	Dump  -- dump to standard output
//
inline void MsgServerPort::Dump()
{	MsgPort::Dump();																		// dump parent first
	printf("  MsgServerPort: \n");														// dump self
}
//
//	Class MsgClientPort implementation
//
//
//	ConnectDetach  -- detach from a channel
//
//	Only clients do this
//
inline void MsgClientPort::ConnectDetach()
{	if (m_coid >= 0)																		// if has channel
	{	::ConnectDetach(m_coid);														// detach
		m_coid = -1;																			// done with it
	}
}
//
//	updatecoid -- make the connection if necessary
//
inline int MsgClientPort::updatecoid()
{
        // attached, no problem
        if (m_coid >= 0) return(0);
        //Get current msg port ID from watchdog.
	if (verbose()) {
            printf("MsgClientPort trying to attach to server %s\n", getname());
            fflush(stderr);
            fflush(stdout);
        }
        // debug info
	//	Port ID info is transient; we never save it long. A network reset will change node numbers.
	MsgPortID portid;
	if (getportidfromwatchdog(portid)) {  //if fail
            if (verbose()) { perror("MsgClient could not get server port from watchdog"); }
            assert(errno);
            return(-1);
	}
        // create a new connection to existing channel
	m_coid = ConnectAttach(portid.m_nodeid, portid.m_pid, portid. m_chid, _NTO_SIDE_CHANNEL,0);
        // if unable to connect to server, but did find watchdog
	if (m_coid < 0)	{
            if (verbose()) {
                perror("MsgClient got server port, but cannot ConnectAttach to it");
            }
            assert(errno);
            return(-1);
	}
	return(0);																		// success
}
//
//	getportidfromwatchdog  -- look up a port ID by asking the watchdog
//
inline int MsgClientPort::getportidfromwatchdog(MsgPortID& portid)
{
	//	Find watchdog
	int wcoid = getwatchdogcoid();									// get watchdog channel ID
	if (wcoid < 0)																// if unable to get watchdog ID
	{	assert(errno); return(-1);	}										// report error
	//	Build transaction to talk to watchdog
	WatchdogMsgID idmsg;
	idmsg.m_msgtype = WatchdogMsgID::k_msgtype;
	strncpy(idmsg.m_progid, getname(), sizeof(idmsg.m_progid));	// copy name to msg
	int stat = ::MsgSend(wcoid, &idmsg, sizeof(idmsg), &idmsg, sizeof(idmsg));	// ask server for help
	if (stat < 0)																	// if fail
	{	if (verbose()) { perror("MsgClientPort unable to communicate with watchdog"); }
		disconnectwatchdogcoid();										// lost connection to watchdog
		return(-1);																	// fails
	}
	portid = idmsg.m_portid;												// return port info
	return(0);																		// success
}
//
//	MsgSend  -- send a message, setting up the connection if necessary
//
inline int MsgClientPort::MsgSend(const void* msg, int msgbytes, void* rmsg, int rmsgbytes)
{	if (updatecoid())															// update connection info
	{	if (verbose()) { perror("MsgClientPort cannot locate server"); }
		assert(errno); return(-1);											// fails
	}
	//	Set timeout for next kernel call. Call will return ETIMEDOUT status if timeout.
	const uint64_t timeoutns = gettimeoutns();					// timeout in nanoseconds
	if (timeoutns != 0)														// if timeout
	{	TimerTimeout( CLOCK_REALTIME,  _NTO_TIMEOUT_SEND | _NTO_TIMEOUT_REPLY, NULL, &timeoutns, NULL );	}
	int stat = ::MsgSend(m_coid, msg, msgbytes, rmsg, rmsgbytes);	// send the message
	if (stat < 0)																	// if communications trouble
	{	if (verbose()) {	perror("MsgSend failed in client"); }	// report the problem
		if (errno == ETIMEDOUT) { return(stat); }					// timeout is normal; we're still attached
		ConnectDetach();														// break connection. Next call will remake it
	}
	return(stat);																	// return status
}
//
//	Dump  -- dump to standard output
//
inline void MsgClientPort::Dump()
{	MsgPort::Dump();															// dump parent first
	printf("  MsgClientPort: m_coid = 0x%08x\n", m_coid);		// dump self
}
#endif // MESSAGING__IMPL_H
