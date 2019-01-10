/////////////////////////////////////////////////////////////////////////////
//
//    File: ethernet.h
//
//    Usage:
//        #include "ethernet.h"
//
//        Ethernet e("myhostname", PORTNUM, TIMEOUT);
//
//    Description:
//        A uniform interface for communicating with TCP/IP sockets over
//        Ethernet.
//
//        Granted other protocols can be used to communicate over Ethernet
//        but this configuration is the most common and robust.
// 
//        "Galil" Controller functions are for communicating with the Galil 
//        motion controllers and servo amplifiers.
//
//        FIX - may need to added variable length timeouts for Galil
//        controllers.  Some of the commands block.  My preference would
//        be to not use any of those commands.
//
//    Written By:
//        Celia Oakley
//            (modeled after the LidarServer_ code written by Eric Seidel
//             and the Serial code written by me)
//        Team Overbot
//        January, 2004
//
/////////////////////////////////////////////////////////////////////////////

#ifndef ETHERNET_H
#define ETHERNET_H

#include <string.h>
#include <termios.h>
#include <netdb.h>		// for gethostname()
#include <netinet/in.h>	// for sockaddr_in
#include <sys/types.h>
#include <sys/socket.h>	// for socket()
#include <unistd.h>		// for close(), read(), write()
#include <sys/select.h>	// for select()
#include "mutexlock.h"

const size_t ETHERNET_HOSTNAME_LEN	= 82;
const size_t ETHERNET_IPADDRESS_LEN = 82;
const size_t ETHERNET_BUF_LEN = 1500;								// input buffer size
const int ETHERNET_PORTNUM_DEF = 23; 							// default telnet port number

class Ethernet {
public:
    Ethernet(const char *hostName, int portNum, int usecTimeout);
    virtual ~Ethernet();																// destructor
	int Connect(double timeout = 0.0);
	int Shutdown();
	int SendBuf(const char* buf, size_t nbytes);
	int SendController(const char *buf, size_t strLen);
	int RecvBuf(char* buf, size_t nbytes);									// DEPRECATED - will be removed
	int RecvController(char *buf, size_t strLen, int usec = 0);
	int Flush();							// reads all data from socket
	
	// get values of private variables
	const char *Hostname()  const { return hostname; }
	const char *IPAddress() const { return ipaddr; }
	bool Connected() const { return(connected); }	// true if connected
	
	// verbose mode
	bool Verbose();
	void Verbose(bool iVerbose);
	//	datagram (UDP) mode
	bool DatagramMode() const { return(datagrammode); }
	void DatagramMode(bool iDatagram);
private:
	ost::Mutex m_lock;													// concurrency protection
    char hostname[ETHERNET_HOSTNAME_LEN];
    char ipaddr[ETHERNET_IPADDRESS_LEN];
    char inbuf[ETHERNET_BUF_LEN];							// input buffer
    int inbuflen;															// length of data in input buffer
    int inbufpos;															// position in input buffer
	int s;				// socket file descriptor
	sockaddr_in sa;		// TCP/IP network address structure
	bool connected;		// true if connected
	int usec;			// timeout in microseconds
	int selectData(struct timeval* tv);	
	int recvChar(struct timeval* tv, char& inbyte);
	bool verbose;		// verbose printing on/off
	bool datagrammode;											// true if datagram (UDP) mode
};

#endif // ETHERNET_H
