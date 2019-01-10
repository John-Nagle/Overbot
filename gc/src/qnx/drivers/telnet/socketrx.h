// SocketRx.h: interface for the CSocketRx class.
//
//////////////////////////////////////////////////////////////////////

#ifndef SOCKETRX_H
#define SOCKETRX_H

#include "mutexlock.h"
#include "protocolrx.h"
#include "threadobject.h"
class TelnetComPort;
//
//	class CSocketRx  -- receive thread of Telnet client
//
class CSocketRx: ost::Pthread									// we are a thread
{
private:
	ost::Mutex m_lock;													// lock for access to this object's data from outside the thread
	TelnetComPort&	m_owner;									// owning object
    CProtocolRx m_protocol;										// protocol handler
    bool m_forceexit;													// set to true to force exit
public:
    CSocketRx(TelnetComPort& owner)
    :	m_owner(owner), m_forceexit(false) {}			// initialization
    virtual ~CSocketRx();
    TelnetComPort& getowner() { return(m_owner); }		// get owning object
    int start();																// start up receiver thread
    int stop();																// stop receiver
    uint32_t getbaudrate();											// get baud rate last reported
    void clearbaudrate();											// get baud rate last reported
    void setverbose(bool mode);								// set verbose mode
protected:
    void run();
private:
   	void deliver(const uint8_t msg[], uint32_t len);		// deliver to owner
 };

#endif
