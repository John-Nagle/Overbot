//
//	TelnetComPort.h  -- interface for talking to a serial port over a TCP connection
//
//	RFC 2217 compliant
//
//	John Nagle
//	Team Overbot
//	December, 2003
//
#ifndef TELNETCOMPORT_H
#define  TELNETCOMPORT_H
#include <inttypes.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "socketrx.h"

//
//	TelnetComPort  --  one socket, with limited Telnet option support
//
//	Automatically sets raw mode. Sets baud rate if requested.
//
//	Abstract class.  Must subclass and provide "deliver" function to get incoming data.
//	Don't block for long in "deliver", or data may be lost due to overrun.
//
class TelnetComPort
{
private:
    sockaddr_in m_sockaddr_in;							// the Internet address to which the socket is connected
    SOCKET m_hSocket;										// the socket
    CSocketRx m_rx;											// the receive thread and its code 
public:
    TelnetComPort();
    virtual ~TelnetComPort();
    int open(const char* hostname, int portid);	// open requested port
   	void close();													// close port if open
   	SOCKET getsocket() { return(m_hSocket); }	// return socket 
   	void setverbose(int verbose);						// set verbose mode
   	virtual void deliver(const uint8_t msg[], uint32_t len) = 0;	// override to get data delivered
   	int write(const uint8_t msg[], uint32_t len);	// write to net
   	int setbaudrate(uint32_t baud);					// set desired baud rate at remote end
   	int getbaudrate(uint32_t& baud);					// returns current baud rate
	int setDTR(bool on);										// set/clear DTR value								
	int purgedata();											// purge queued data, if any								
};

#endif //  TELNETCOMPORT_H
