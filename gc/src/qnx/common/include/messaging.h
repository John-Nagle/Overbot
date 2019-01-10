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
//	Typical usage is
//
//		MsgPort actuatorport("ACTUATORS",  0.2);								 // set up the connection
//		...
//		actuatorport.MsgSend(cmd,sizeof(cmd), reply,sizeof(reply));	// send a command to the server
//
#ifndef MESSAGING_H
#define MESSAGING_H
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
//
//	Template forms for standard messaging primitives.
//	These are for convenience and safety; they don't have to be used.
//	These take a fixed-length object for input and output, and compute the size automatically
//	to avoid mistakes.
//
template<class TIN, class TOUT> int MsgSend(int coid, const TIN& msg, TOUT& rmsg)
	{	return(::MsgSend(coid, &msg, sizeof(msg), &rmsg, sizeof(rmsg))); }
template<class TIN, class TOUT> int MsgSend(int coid, const TIN& msg)
	{	return(::MsgSend(coid, &msg, sizeof(msg), 0,0)); }
//	In our usage, the status into MsgReply is always the reply size, which the
//	caller might need.  Size < 0 indicates an error, indicated using MsgError.
template<class T> int MsgReply(int rcvid, const T& msg)
	{	return(::MsgReply(rcvid, sizeof(msg), &msg, sizeof(msg))); }
//
//	struct MsgPortID -- one message port, identified by a node ID/processID/channel ID triple
//
//	These are transient data. Any network change changes node IDs, so don't save these.
//
struct MsgPortID {
	int32_t m_nodeid;															// node ID
	pid_t m_pid;																	// process ID
	int32_t m_chid;																// channel ID
};
const MsgPortID emptyMsgPortID = {0,0,0};						// default empty value
//
//	General message base type
//
//	Structures based on this header will travel between processes and CPUs, and
//	thus may not contain virtual functions or pointers.  Our convention is that
//	messages are derived from type MsgBase, so that they all have a
//	4 char code at the beginning to identify them. This is useful for debugging
//	and logging.
//
struct MsgBase {
	uint32_t	m_msgtype;												// the message type, 4 chars, as 'TYPE', not "TYPE", so switch will work.
};
//
//	char4  -- combine 4 chars into an integer
//
//	Has to be a define or C++ won't accept the result of this as a true constant usable at compile time.
//
#define char4(a,b,c,d) ((a)<<24 | (b)<<16 | (c) << 8 | (d))
//
//	Messages the watchdog process understands.
//
struct WatchdogMsgOK: public MsgBase {					//	OK: program OK, reset watchdog timer
	static const uint32_t k_msgtype = char4('W','D','O','K');		// The message type for this msg
};

struct WatchdogMsgID: public MsgBase {						// D: look up MsgPortID by name
	static const uint32_t k_msgtype = char4('W','D','I','D');		// The message type for this msg
	char m_progid[32];															// program ID to look up
	MsgPortID m_portid;													// filled in reply, meaningless in query
};
struct WatchdogMsgSV: public MsgBase {					// SV: this program has a server, register it
	static const uint32_t k_msgtype = char4('W','D','S','V');		// The message type for this msg
	int		m_chid;																// channel ID to register

        //khianhao
        //the name of the server specified with env var ID=
        char m_name[32];
};
//
//	WatchdogMsg -- all watchdog messages as a union
//
//	Used as argument to MsgReceive. Size of union is size of largest acceptable message.
//
union WatchdogMsg {
	WatchdogMsgOK	m_ok;
	WatchdogMsgID	m_id;
	WatchdogMsgSV	m_sv;
};
//
//	class MsgPort  --  encapsulates a message port and provides useful operations on it.
//
//	Handles communications with the watchdog.
//
class MsgPort {
private:
	const char* m_name;													// name of server
	uint64_t	m_timeoutns;												// timeout in nanoseconds
	int m_watchdogcoid;														// connection ID to watchdog (-1 if invalid)
	bool m_verbose;															// turns on debug messages
protected:
	static int getwatchdogportid(MsgPortID& watchport);	// find the watchdog
	int getwatchdogcoid();													// get watchdog connection ID, connecting if necessary
	void disconnectwatchdogcoid();									// release the watchdog connection
public:
	MsgPort(const char* name, double timeout = 0.0);		// constructor
	virtual ~MsgPort();														// destructor
	//	Utility functions
	void setverbose(bool verbose = true) { m_verbose = verbose; }
	bool verbose() { return(m_verbose); }						// if verbose mode
	virtual void Dump();														// dump to standard output
	void SetTimeout(double timeout)									// set timeout for later operations
	{	m_timeoutns = uint64_t(timeout*1.0e9);	}				// convert to nanoseconds
	uint64_t gettimeoutns() const { return(m_timeoutns); }// get timeout in nanoseconds
	const char* getname() const { return(m_name);		}	// access
	//	Watchdog access
	int watchdogreset();														// reset watchdog timer, get system status
	//	Priority setting
	static int setpriority();													// set priority based on "PRI" environment variable
	static int setpriority(int relval);										// set priority relative to "PRI" environment variable
};
//
//	MsgServerPort  --  port for use by a server
//
class MsgServerPort: public MsgPort {
private:
	int m_chid;																	// server channel ID
private:
	int registerserverwithwatchdog(int chid);						// register with the watchdog
public:
	//	Standard messaging functions
	int MsgReceive(void* rmsg, int nbytes, struct _msg_info* info);
	//	Template forms for ease of use and safety
	template<class T> int MsgReceive(T& msg, struct _msg_info* info = 0)
	{	return(MsgReceive(&msg, sizeof(msg), info)); }
	int ChannelCreate(int flags = 0);									// create the channel
	void ChannelDestroy();													// destroy the channel if any
	MsgServerPort(double timeout = 0.0)							// usual constructor
	: MsgPort(getenv("ID"), timeout), m_chid(-1) {}
	MsgServerPort(const char* name, double timeout = 0.0)	// constructor with explit name (unusual)
	: MsgPort(name,timeout), m_chid(-1) {}
	virtual void Dump();														// dump to standard output
	virtual ~MsgServerPort()												// destructor
	{	ChannelDestroy(); }													// destroy on exit
};
//
//	MsgClientPort  -- port for use by a client
//
class MsgClientPort: public MsgPort {
private:
	int	 m_coid;																	// connection ID (-1 if invalid)
protected:
	int updatemsgportid();													// update connection info by asking watchdog
	int getportidfromwatchdog(MsgPortID& portid);			// look up server port via watchdog
	int updatecoid();															// update connection id when needed
public:
	MsgClientPort(const char* name, double timeout = 0.0)
	: MsgPort(name,timeout), m_coid(-1) {}
	//	Standard QNX form
	int MsgSend(const void* msg, int msgbytes, void* rmsg, int rmsgbytes);
	//	Safer, more convenient template form
	template<class TIN, class TOUT> int MsgSend(const TIN& msg, TOUT& rmsg)
	{	return(MsgSend(&msg,sizeof(msg),&rmsg, sizeof(rmsg))); }
	template<class TIN> int MsgSend(const TIN& msg)	// no reply expected form
	{	return(MsgSend(&msg,sizeof(msg),0,0)); }
	void ConnectDetach();
	virtual void Dump();														// dump to standard output
	virtual ~MsgClientPort()
	{	ConnectDetach();  }
};

//
//	Inline implementations
//
//	(Move to a library if they get too big)
//
#include "messaging_impl.h"

#endif // MESSAGING_H
