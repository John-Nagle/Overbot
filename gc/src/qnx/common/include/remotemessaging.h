//
//	remotemessaging.h  -- messaging from non-watchdog to watchdog programs
//
//	To use this, there must be a copy of "remotemsgserver" running under the
//	watchdog on the server machine. This program talks to the specified
//	"remotemsgserver" program, which then talks to the proper program
//	under the watchdog.  
//
//	For test use, to allow external access to programs on the Overbot vehicle.
//
//	John Nagle
//	January, 2005
//
#ifndef REMOTEMESSAGING_H
#define REMOTEMESSAGING_H
#include <stdio.h>
#include <stdlib.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
//
//	name_open_remote  -- do a name_open on a remote machine
//
//	This is a bit hokey, but valid on QNX 6.2x.
//
inline int name_open_remote(const char* node, const char* path)
{	if (!node) return(name_open(path,0));													// local form	
	char s[511];											
	snprintf(s,sizeof(s),"../../../net/%s/dev/name/local/%s",node,path);		// construct name of node on remote machine
	return(name_open(s,0));
}
//
//	RemoteMsgClientPort  -- port for use by a remote client
//
class RemoteMsgClientPort {
private:
	const char* m_node;													// server's node name
	const char* m_name;													// connection name on server
	int	 m_coid;																	// connection ID (-1 if invalid)
	uint64_t  m_timeout;														// message/connection timeout
protected:
	int updatecoid();															// update connection id when needed
public:
	RemoteMsgClientPort(const char* node, const char* name, double timeout = 0.0)
	: m_node(node), m_name(name), m_coid(-1),m_timeout(uint64_t(timeout*1.0e9))
	{}
	//	Standard QNX form
	int MsgSend(const void* msg, int msgbytes, void* rmsg, int rmsgbytes);
	//	Safer, more convenient template form
	template<class TIN, class TOUT> int MsgSend(const TIN& msg, TOUT& rmsg)
	{	return(MsgSend(&msg,sizeof(msg),&rmsg, sizeof(rmsg))); }
	template<class TIN> int MsgSend(const TIN& msg)	// no reply expected form
	{	return(MsgSend(&msg,sizeof(msg),0,0)); }
	void ConnectDetach();
	virtual ~RemoteMsgClientPort()
	{	ConnectDetach();  }
};
//
//	Implementation
//
//
//	MsgSend  -- send a message, setting up the connection if necessary
//
inline int RemoteMsgClientPort::MsgSend(const void* msg, int msgbytes, void* rmsg, int rmsgbytes)
{	if (updatecoid())															// update connection info
	{	 return(-1);																// fails, error in errno.
	}
	//	Set timeout for next kernel call. Call will return ETIMEDOUT status if timeout.
	const uint64_t timeoutns = m_timeout;							// timeout in nanoseconds
	if (timeoutns != 0)														// if timeout
	{	TimerTimeout( CLOCK_REALTIME,  _NTO_TIMEOUT_SEND | _NTO_TIMEOUT_REPLY, NULL, &timeoutns, NULL );	}
	int stat = ::MsgSend(m_coid, msg, msgbytes, rmsg, rmsgbytes);	// send the message
	if (stat < 0)																	// if communications trouble
	{	if (errno == ETIMEDOUT) { return(stat); }					// timeout is normal; we're still attached
		ConnectDetach();														// break connection. Next call will remake it
	}
	return(stat);																	// return status
}
//
//	updatecoid -- update connection ID
//
//	Connects if necessary.  This allows us to recover from disconnection.
//
inline int RemoteMsgClientPort::updatecoid()
{	if (m_coid >= 0) return(0);											// success
	m_coid = name_open_remote(m_node, m_name);		// try to open
	if (m_coid < 0) return(-1);												// failed
	return(0);																		// success
}
//
//	ConnectDetach  -- detaches a connection
//
inline void RemoteMsgClientPort::ConnectDetach()
{	if (m_coid < 0) return;													// if not open, done
	name_close(m_coid);													// close it
	m_coid = -1;																	// mark as closed
}
#endif // REMOTEMESSAGING_H