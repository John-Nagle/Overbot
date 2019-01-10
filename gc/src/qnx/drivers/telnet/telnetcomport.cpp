//
//	TelnetComPort.cpp  -- interface for talking to a serial port over a TCP connection
//
//	RFC 2217 compliant
//
//	John Nagle
//	Team Overbot
//	December, 2003
//

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <netdb.h>
#include <errno.h>
#include "telnetcomport.h"

//
//	Constructor
//
TelnetComPort::TelnetComPort()
: m_hSocket(INVALID_SOCKET),											// no socket yet
 m_rx(*this)
{}
//
//	open  -- open a connection
//
//	Returns nonzero if fail.
//	Error in errno.
//
int TelnetComPort::open(const char* strIP, int nPort)
{	close();																			// close any existing socket
    unsigned long ip;

    if((*strIP <= '9') && (*strIP >= '0'))
    {
        if((ip = inet_addr(strIP)) == INADDR_NONE)
        {  printf("Invalid host ip address: %s.\n",strIP);
        	errno = EFAULT;													// bad param
        	return(1);																// fails
      	}
    }
    else
    {
        hostent* ent = gethostbyname(strIP);
        if(!ent)
        {	printf("Host name not found: %s\n", strIP);			// report problem
        	errno = EHOSTDOWN;											// no such host
        	return(1);																// fails
        }
        ip = *(unsigned long*)(ent->h_addr);
    }

    m_sockaddr_in.sin_family = AF_INET;
    m_sockaddr_in.sin_port = htons(nPort);
    m_sockaddr_in.sin_addr = *(in_addr*)&ip;
    //	We have an IP address. Get a socket.
   	m_hSocket = socket(PF_INET,SOCK_STREAM,IPPROTO_TCP);	// make a socket
   	if (m_hSocket < 0)														// if fail
   	{	perror("Unable to create socket"); return(1); } 		// fails
   	//	We have a socket.  Attempt connection.
   	int stat = connect(m_hSocket,(sockaddr*)&m_sockaddr_in,sizeof(sockaddr));
	if (stat)																			// if connection fails
	{	perror("Connect failed");											// connection failed
		return(1);																	// fails
	}
	//	Start receive thread
	stat = m_rx.start();														// start the receive thread	
	return(stat);																	// success	
}
TelnetComPort::~TelnetComPort()
{	close();																			// close any existing socket
}
//
//	close -- close socket, stop receive thread
//
void TelnetComPort::close()
{	if (m_hSocket < 0) return;												// nothing to do
	::close(m_hSocket);														// normal close - may wait
	m_hSocket = -1;															// done with socket
	//	Closing the socket first will cause the receive thread to fail at the next "recv" call.
	//	The thread should then exit, and the thread join will work. So when we return
	//	from close, the receive operation should be properly shut down.
	m_rx.stop();																	// shut down receive thread
}
//
//	write -- write to connection
//
//	Translates IAC to IAC IAC, as an escape
//
int TelnetComPort::write(const uint8_t data[], size_t datalen)
{	if (!data || datalen == 0) return(0);									// nothing to do
	uint8_t  buf[1024];															// output buf
	unsigned int pos = 0;														// position in output buf
	for (unsigned int i=0; i<datalen; i++)								// for all chars to write
	{	if (pos >= sizeof(buf)-1)												// if buffer full
		{	int stat = send(m_hSocket,buf,pos,0);						// send
			if (stat < 0) return(stat);											// if fail
			pos = 0;
		}
		uint8_t ch = data[i];
		if (ch == TELNET_IAC) buf[pos++] = TELNET_IAC;			// escape byte
		buf[pos++] = ch;															// regular byte
	}
	if (pos == 0) return(0);														// nothing more to do
	////printf("Writing %d bytes\n",pos);										// ***TEMP***
	return(send(m_hSocket,buf,pos,0));									// send escaped data
}
//
//	setverbose -- set verbose mode
//
void TelnetComPort::setverbose(int mode)
{	m_rx.setverbose(mode); }												// makes protocol engine verbose
//
//	purgedata -- purge data at remote end
//
//	Otherwise, old junk data comes in
//
int TelnetComPort::purgedata()									
{	
	const uint8_t cmd[] = { TELNET_IAC, TELNET_SB, TOPT_COMPORTCONTROL,
		COM_PORT_PURGE_DATA, TELNET_IAC, TELNET_SE };
	return(send(m_hSocket, cmd, sizeof(cmd), 0));					// send command 	
}
//
//	setbaudrate -- set baud rate at remote end
//
//	Builds appropriate telnet option and sends it.
//	Waits for other end to send an option indicating the new baud rate.
//
//	Requesting zero will poll for the baud rate.
//
int TelnetComPort::setbaudrate(uint32_t baud)									
{	m_rx.clearbaudrate();														// clear what the receive side thinks is the baud rate
	const uint8_t b1 = (baud >> 0) & 0xff;
	const uint8_t b2 = (baud >> 8) & 0xff;
	const uint8_t b3 = (baud >> 16) & 0xff;
	const uint8_t b4 = (baud >> 24) & 0xff;
	const uint8_t cmd[] = { TELNET_IAC, TELNET_SB, TOPT_COMPORTCONTROL,
		COM_PORT_SET_BAUDRATE, b4, b3, b2, b1, TELNET_IAC, TELNET_SE };
	int stat = send(m_hSocket, cmd, sizeof(cmd), 0);				// send command 	
	if (stat < 0) return(stat);													// if fail, error
	for (int i=0; i<50; i++)														// wait up to 5 seconds for baud rate to change
	{	baud = m_rx.getbaudrate();											// get last known baud rate from received options
		if (baud != 0) return(0);												// success
		usleep(100000);															// wait 100ms
	}
	errno = ETIMEDOUT;															// fails, can't get baud rate
	return(-1);																			// failed
}
//
//	getbaudrate -- get baud rate at remote end
//
//	Gets current baud rate.
//
int TelnetComPort::getbaudrate(uint32_t& baud)									
{	
	baud = m_rx.getbaudrate();												// get the last known baud rate
	if (baud != 0) return(0);													// success
	//	No baud rate known. Set rate to 0, which will get us more info but will not change the baud rate
	int stat = setbaudrate(0);													// this waits for a baud rate change
	if (stat < 0) return(stat);													// if fail, error
	baud = m_rx.getbaudrate();												// get the last known baud rate
	return(0);																			// success
}
//
//	setDTR -- set/clear Data Terminal Ready signal. 
//
//	Operates relay in the Sealink unit
//
int TelnetComPort::setDTR(bool on)									
{	const uint8_t dtrval = on ? 8 : 9;										// per RFC 2217
	const uint8_t cmd[] = { TELNET_IAC, TELNET_SB, TOPT_COMPORTCONTROL,
		COM_PORT_SET_CONTROL, dtrval, TELNET_IAC, TELNET_SE };
	return(send(m_hSocket, cmd, sizeof(cmd), 0));					// send command 
}

