/////////////////////////////////////////////////////////////////////////////
//
//    File: ethernet.cc
//
//    Usage:
//        see ethernet.h
//
//    Description:
//        see ethernet.h
//
//    Written By:
//        Celia Oakley
//            (modeled after the LidarServer_ code written by Eric Seidel,
//             a bit of the telnet code written by John Nagle,
//             and the Serial code written by me)
//        Team Overbot
//        January, 2004
//
/////////////////////////////////////////////////////////////////////////////

#include <assert.h>
#include <sys/neutrino.h>
#include <arpa/inet.h>
#include "ethernet.h"
#include "mutexlock.h"
#include "logprint.h"

//
//    Ethernet::Ethernet - default class constructor
//
//	Acquires a socket, but does not connect. 
//	Can fail only for local reasons.
//
//	Errors throw string exceptions. 
//	Errors should not occur unless system is misconfigured or badly broken.
//
Ethernet::Ethernet(const char* hostName, int portNum, int usecTimeout)
: inbuflen(0), inbufpos(0), s(-1),connected(false), verbose(false)	,datagrammode(false)			// initial state
{
   	
   	// set up server address
    memset(&sa, 0, sizeof(sa));
    sa.sin_family = AF_INET;
    if ( portNum == 0 ) {
    	// have protocol assign unused, nonprivileged value
    	sa.sin_port = htons(ETHERNET_PORTNUM_DEF);
    } else {
		// use portnum given
    	sa.sin_port = htons(portNum);
    }
    if ( (hostName[0] >= '0') && (hostName[0] <= '9') ) {
    	// hostName is IP address
    	in_addr_t ip;
    	
    	// convert string into numeric IP address
    	if ( (ip = inet_addr(hostName)) == INADDR_NONE ) {
    		logprintf("Ethernet::Ethernet - error parsing IP address %s\n",
    		        hostName);
    		throw("Ethernet::Ethernet - error parsing IP address");
    	}
    	sa.sin_addr = *(in_addr *) &ip;

    	// set hostname and IP address
    	snprintf(hostname, ETHERNET_HOSTNAME_LEN, "<only IP address given>");
	    snprintf(ipaddr, ETHERNET_IPADDRESS_LEN, "%s", hostName);
    } else {
    	// hostName should be in /etc/hosts
    	static ost::Mutex gethostlock;											// avoid reentering non-reentrant gethostbyname
    	ost::MutexLock lok(gethostlock);										// lock
 		hostent *hp;		// internet host structure
	
		// look up hostname and initialize TCP/IP address structure
    	if ( (hp = gethostbyname(hostName)) == NULL ) {
    		logprintf("Ethernet::Ethernet - host '%s' not found.\n",
    		        hostName);
    		throw("Ethernet::Ethernet - host name not found");
   		}
    
   		// make sure address filled in structure
    	if ( ((in_addr *) hp->h_addr) == NULL ) {\
    		logprintf("Ethernet::Ethernet - no address for host '%s'\n",
    	    	    hostname);
    		throw("Ethernet::Ethernet - no address for host");
    	}
    	
    	memcpy(&sa.sin_addr, hp->h_addr, hp->h_length);
    
    	// set hostname and IP address
    	snprintf(hostname, ETHERNET_HOSTNAME_LEN, "%s", hostName);
	    snprintf(ipaddr, ETHERNET_IPADDRESS_LEN, "%s", inet_ntoa(sa.sin_addr));
    }
    
	// set timeout time
	usec = usecTimeout;

}
//
//	Destructor
//
Ethernet::~Ethernet()
{
	if (s >= 0) close(s);															// close socket if open
}

//
//    Ethernet::Connect - initiate a connection on the socket
//
//    Connect is not included in class constructor so that errors can be
//    returned and deal with by the caller.
//
//    Returns 0 if successful, -1 if an error occurred (errno is set)
//
int Ethernet::Connect(double timeout)
{	ost::MutexLock lok(m_lock);											// single user
	if (s >= 0) Shutdown();													// get rid of any old connection
   	// set up socket (3rd argument to socket() can be 0 or IPPROTO_TCP; both ok)
   	int proto = datagrammode ? IPPROTO_UDP : IPPROTO_TCP;	// select protocol
   	int ptype = datagrammode ? SOCK_DGRAM : SOCK_STREAM; // select type to match
    if ( (s = socket(AF_INET, ptype, proto)) == -1 ) {			// create socket
    	logperror("Ethernet::Connect - unable to create socket");
    	return(-1);																	// cannot connect
    }
    //	Arrange for timeout if other end is down.
    //	This prevents long delays when trying to connect to a dead controller.
    if (timeout > 0)																// if timeout requested
    {	uint64_t ntime = uint64_t(timeout*1.0e9);					// wait this long
    	int stat = TimerTimeout(CLOCK_REALTIME,  (_NTO_TIMEOUT_SEND | _NTO_TIMEOUT_REPLY),
    		0,&ntime, 0);
    	if (stat)
    	{	logperror("Ethernet::Connect - TimerTimeout failed");
    		return(-1);
    	}
    }
    //	Try to connect
    if ( connect(s, (struct sockaddr *)&sa, sizeof sa) == -1 ) {
    	if (verbose) { logperror("Ethernet::Connect - unable to connect socket"); }
        return -1;
    }

    // Record that connection is alive.
    connected = true;
    return 0;
}

//
//    Ethernet::Shutdown - closes connection forcibly
//
//    Returns 0 if successful, -1 if an error occurred (errno is set)
//
int Ethernet::Shutdown()
{
	ost::MutexLock lok(m_lock);			// single user
	// remember that connection is not active
	connected = false;   					 // end connection
	if (s >= 0) close(s);						// close
	s = -1;											// no socket
    return 0;
}

//
//    Ethernet::SendBuf - write buffer data to a socket
//
//    Call after establishing socket connection with Connect()
//
//    Returns number of bytes written if successful, 
//            -1 if an error occurred (errno is set)
//            -2 if an error occurred (errno not set)
//
//
ssize_t Ethernet::SendBuf(const char* buf, size_t nbytes)
{
	ost::MutexLock lok(m_lock);									// single user
	if (!connected) {	errno = ENOTCONN; return(-1);	}	// not connected, handle
	ssize_t cnt = send(s,buf,nbytes,0);						// send everything all at once
	if (cnt < 0) 															// if error
	{	Shutdown();														// close the connection to force a reopen
		return(cnt);														// report error
	}
	if ( verbose ) {
			logprintf("Ethernet::SendBuf - sent %d bytes\n", cnt);
	}
	//	Check byte count sent
	if (cnt != int(nbytes))
	{	logprintf("Ethernet::SendBuf: tried to send %d bytes, sent %d bytes.\n", nbytes, cnt);	
		errno = EIO; return(-1);											// fails
	}
    return cnt;
}
//
//    Ethernet::SendController - write Galil Controller data to the socket
//
//    Call after establishing socket connection with Connect()
//
//		Converts string to Galil Controller format by appending a
//		\r, then calls SendBuf.  
//		Used by ethernetmenu only, not the actual control library.
//		The actual control library appends its own CR LF and can
//		send longer messages.
//
//    Returns number of bytes written if successful, 
//            -1 if an error occurred (errno is set)
//            -2 if an error occurred (errno not set)
//
ssize_t Ethernet::SendController(const char *str, size_t strLen)
{
	ost::MutexLock lok(m_lock);											// single user
	if ( verbose ) {
    	logprintf("Ethernet::SendController - sending '%s' to %s\n", str, hostname);
    }
    size_t len = strlen(str);													// length of msg
    if (len >= ETHERNET_BUF_LEN-2)									// if overside
    {	logprintf("Ethernet::SendController - oversize message: %s\n",str);
    	return(-2);																	// fails
    }
	//	Send little strings as one TCP transaction, to avoid delays.
    char strController[ETHERNET_BUF_LEN];
    // append CR to string
    snprintf(strController, sizeof(strController),"%s\r", str);
    return Ethernet::SendBuf(strController, strlen(strController));
}
//
//    Ethernet::RecvBuf - read buffer data from the socket, within timeout time
//
//    Call after establishing socket connection with Connect()
//
//    Returns 0 if successful, -1 if an error occurred (errno is set)
//                             -2 if an error occurred (errno not set)
//
//	This is only for debug program use.  It doesn't parse the input, and it always waits
//	one timeout interval for a timeout
//
int Ethernet::RecvBuf(char* buf, size_t buflen)
{
	ost::MutexLock lok(m_lock);							// single user
	if (!connected) 											// if not connected
	{	errno = ENOTCONN; return(-1);	}				// fails
    struct timeval tv;
    // set the timeout
    tv.tv_sec = 0;
    tv.tv_usec = usec;										// timeout is in microseconds
    memset(buf,0,buflen);									// clear buffer
    for (unsigned int i=0; i<buflen-1; i++)			// stop one short of end, to insure null-termination
    {	
		char inchar;
		int stat = recvChar(&tv,inchar);				// get a byte
		if (stat == -2)											// normal end of input - timeout
		{	 break; }
		if (stat < 0)												// error case
		{	logperror("Ethernet::RecvBuf - error reading from socket");
			return(-1);
		}
		buf[i] = inchar;											// normal case - save char
	}
	//	Output dump
	if (verbose)
	{	logprintf("Received: %s\n",buf);
	}
	return(0);
}
//
//    Ethernet::RecvController - read Galil Controller data from the socket, 
//                               within timeout time
//
//    Call after establishing socket connection with Connect()
//
//	Returns one line from the Galil controller, stopping at the next ":" or "?" at the
//	end or beginning of a line.
//
//    Returns 0 if successful, -1 if an error occurred (errno is set)
//                             -2 if an error occurred (errno not set)
//
//	Two processes shouldn't be calling this at the same time, but
//	we lock anyway.
//
int Ethernet::RecvController(char *buf, size_t buflen, int timeoutus)
{
	ost::MutexLock lok(m_lock);									// single user
	if (!connected)
	{	errno = ENOTCONN; return(-1);	}						// fails
    struct timeval tv;
    unsigned int numChar;

    // set the timeout
    if (timeoutus)														// if timeout specified, use it
    {	tv.tv_sec = timeoutus / 1000000;
    	tv.tv_usec = timeoutus % 1000000;
    } else {
	    tv.tv_sec = 0;													// otherwise use default
	    tv.tv_usec = usec;											
	 }

    // read characters and fill buf appropriately
    numChar = 0;
    buf[0] = '\0';
    
    if ( verbose ) {
    	logprintf("Ethernet::RecvController - reading from %s\n", hostname);
    }

	//	Get and process next char. Level below does buffering.
	//	Timeouts should NOT occur. 
	//	The next ":" or "?" at the begnning or end of a line ends processing.
	//
	char prevchar = '\0';													// no previous char yet   
   for (;;)
   {	char inchar;
		// read character from socket when ready
		int stat = recvChar(&tv, inchar);							// get a char
		if (stat == -2)														// if timeout
		{	logprintf("Ethernet::RecvController: Timeout while reading reply from controller.\n");	
			return(-2);															// report error	
		}
		if (stat < 0)
		{	logperror("Ethernet::RecvController: read error from network");
			return(-1);
		}
		// deal with character
		// display character if verbose mode
		if ( verbose ) {
			logprintf(".%c|0x%02x.", inchar, int(inchar));
		}
		if (numChar >= buflen-1)										// if incoming line too long
		{	logprintf("Ethernet::SendController - received oversize line from controller..\n");
			errno = EMSGSIZE;											// set reasonable error code
			return(-1);															// fails
		}
		// not too long, append character to buffer
		buf[numChar++] = inchar;   	
		// check if last char in reply
		// : = valid command, ? = invalid command
		//	Meaningful only at beginning of line.
       	if ((inchar == ':') || (inchar == '?'))					// if prompt/reply char
       	{	if (prevchar == '\0')	break;							// the reply is ":' or '?" alone.  D	one.	
       	}
       	if (inchar == ':')												// if ':'
       {	if (prevchar == '\r' || prevchar == '\n' )		// following end of line or beginning of line
       		{	numChar--;												// remove trailing ':'
       			//	Remove any trailing CR LF
       			while (numChar > 0 && (buf[numChar] == '\r' || buf[numChar] == '\n')) numChar--;
       		 	break;														// normal end of line 
       		}
       	}
      	prevchar = inchar;											// save new as previous char  	
	} 
	assert(numChar < buflen);
	buf[numChar++] = '\0';										// make sure there is a terminating null
    return 0;
}

//
//    Ethernet::Flush - flush the input buffer of the socket
//
//    Returns flush count if successful, -1 if an error occurred (errno is set)
//
int Ethernet::Flush()
{
	ost::MutexLock lok(m_lock);													// single user
	if (!connected) 																	// if not connected, fails
	{	errno = ENOTCONN; return(-1); }
    struct timeval tv;
    // set the timeout to 0.  This is a poll, and will not block.
    tv.tv_sec = 0;
    tv.tv_usec = 0;
        
	// wait until read times out (no more chars)
	int flushed = 0;
	for (;;)
	{  char inchar = '\0';																// incoming char, initialized to keep compiler happy
		int stat = recvChar(&tv,inchar);										// wait for data or timeout
		if (stat == -2) break;															// timeout, we are happy
		if (stat < 0) return(-1);														// error, return
		flushed++;
	}
	if (flushed > 0) { logprintf("Ethernet::Flush: %d chars flushed.\n", flushed); }
	return(flushed);																	// success
}

//
//    Ethernet::Verbose - get the value of verbose
//
//    Returns true if verbose mode is on, false otherwise
//
bool Ethernet::Verbose()
{
	return verbose;
}

//
//    Ethernet::Verbose - set the value of verbose
//
//    Returns nothing
//
void Ethernet::Verbose(bool iVerbose)
{
	verbose = iVerbose;
}
//
//	Private functions
//
//
//    Ethernet::selectData  -- wait for input on selected file descriptor.
//
//	Time-limited.
//
//    Returns 0 if successful, -1 if an error occurred (errno is set)
//                             -2 if timeout
//
int Ethernet::selectData(struct timeval* tv)
{
    // Must be connected.
    // File descriptor must be open.
    // Check for race condition where other process has closed 
    // and set s to -1
    int fd = s;													// file descriptor
    if (!connected  || (fd < 0))
    {	errno = ENOTCONN;
    	 return(-1);											// not connected, fails
   	}
	// clear the set of file descriptors and add socket
    fd_set rfd1;
	FD_ZERO(&rfd1);
	FD_SET(fd, &rfd1);
    // wait until port is ready for reading, or has an error
	switch ( select(fd+1, &rfd1, 0, 0, tv) ) {
	case -1:	// errno set
		logperror("Ethernet::selectData - problem with select call");
		return -1;

	case 0:		// timeout expired
		//	Zero timeout is just a poll. That's OK.
		if (tv->tv_sec != 0 || tv->tv_usec != 0)		// if nonzero timeout, this is a problem
		{	double waittime = tv->tv_sec + (tv->tv_usec * 0.000001);	
			logprintf("Ethernet::selectData - read select timed out after %f sec.\n",waittime);
		}
		return -2;

	default: 	// socket ready for reading
		return 0;
	}
}

//
//  Ethernet::recvChar - generic get byte from socket.
//
//	Buffered
//
//    Called from Read routines.
//
//    Returns 0 if successful, -1 if an error occurred (errno is set)
//                             -2 if timeout
//
int Ethernet::recvChar(struct timeval* tv, char& inbyte)
{
	if (inbuflen <= inbufpos)										// if need to read
	{	inbuflen = 0; inbufpos = 0;								// set to empty buffer state
		int stat = selectData(tv);									// wait for input
		if (stat < 0) return(stat);									// fails, timeout or connection lost
		int recvBytes = recv(s,inbuf,sizeof(inbuf),0);	// receive data
		if (recvBytes == -2) return(-2);						// handle timeout, which is normal
		if (recvBytes == 0)											// Zero-length UDP datagram
		{	logprintf("Ethernet::recvChar: Zero-byte message from \"%s\"\n",hostname);
			return(-2);														// handle as timeout
		}
		if (recvBytes < 0)												// handle error
		{	logperror("Ethernet::recvChar - problem with recv call");
			if (errno == ENOTCONN)								// if connection has been lost
			{	Shutdown();												// officially disconnect it
				logprintf("Ethernet::recvChar: Lost connection to \"%s\"\n",hostname);
			}
			return -1;
		}
		//	Normal case, we have received data
		inbuflen = recvBytes;
	}
	//	Data is now available to return.
	assert(inbuflen > inbufpos);								// must have at least one character
	inbyte = inbuf[inbufpos++];								// return next available byte
	return(0);															// success
}
//
//	DatagramMode -- change to/from datagram mode
//
void Ethernet::DatagramMode(bool isDatagramMode)
{	ost::MutexLock lok(m_lock);								// single user
	Shutdown();														// kill outstanding connection if any
	datagrammode = isDatagramMode;					// set mode
}
