//
//	lidarsocket.h  -- LIDAR socket I/O
//
//	Usable from multiple threads
//
//	Designed not to generate SIGPIPE signals
//
//	J.	Nagle
//	Team Overbot
//	August, 2005
//
#include <stdio.h>
#include <netinet/in.h> /* for sockadd_in */
#include <sys/types.h>
#include <sys/socket.h> /* for socket() */
#include <netdb.h> /*for gethostbyname() */
#include <unistd.h> /* close(), read(), write() */
#include <sys/select.h> /* for select() */
#include <sys/neutrino.h>	/* for timers */
#include "lidarsocket.h"
#include "logprint.h"
//
//	Destructor
//
LidarSocket::~LidarSocket()
{
	if (m_sock >= 0) ::close(m_sock);													// final close
}
//
//	isopen  -- true if open
//
bool LidarSocket::isopen() const
{	return(m_sock >= 0);	}
//
//	open -- open a connection, and save its port
//
int LidarSocket::open(const char *hostname, unsigned short portnum)
{
    struct sockaddr_in sa;
    struct hostent     *hp;
    int s;

    if ((hp = gethostbyname(hostname)) == NULL)
    {	/* do we know the host's */
        errno = ENONET;                       /* address? */
        return(-1);                                /* no */
    }
    ost::MutexLock lokr(m_reading);											// need both locks to open or close
    ost::MutexLock lokw(m_writing);											// must lock reading first to prevent deadlock
    if (m_sock >= 0)																	// if already open
    {	::close(m_sock);																// close it
        m_sock = -1;																		// closed
    }
    memset(&sa,0,sizeof(sa));
    memcpy((char *)&sa.sin_addr,hp->h_addr,hp->h_length);     /* set address */
    sa.sin_family= hp->h_addrtype;
    sa.sin_port= htons((u_short)portnum);

    if ((s = socket(hp->h_addrtype,SOCK_STREAM,0)) < 0)     /* get socket */
        return(-1);
    if (connect(s,(struct sockaddr *)&sa,sizeof sa) < 0)
    {
        logprintf("Failed to connect to host: \"%s\" on port: %i: %s\n", hostname, portnum, strerror(errno));
        ::close(s);																			// close socket
        return(-1);
    }

    logprintf("Connected to host: \"%s\" on port: %i\n", hostname, portnum);
    m_sock = s;																			// now opened
    m_closing = false;																	// not closing now
    return(0);																				// success
}
//
//	close -- close socket if open
//
void LidarSocket::close()
{	m_closing = true;																	// prevent further blocking
    ost::MutexLock lokr(m_reading);											// need both locks to open or close
    ost::MutexLock lokw(m_writing);											// must lock reading first to prevent deadlock
    if (m_sock >= 0)
    {
        logprintf("Closing TCP connection.\n");
        int stat = ::close(m_sock);													// close TCP connection
        if (stat < 0)
        {
            logprintf("Error closing socket: %s\n",strerror(errno));	// report trouble
        }
        m_sock = -1;																		// closed
    }
    m_closing = false;																	// no longer closing, done
}
//
//	read_data  -- read with timeout
//
int LidarSocket::read_data(
    char *buf, /* pointer to the buffer */
    int n      /* number of characters (bytes) we want */
)
{
    //printf("Called read_data with socket: %i, buf: %p, size: %i\n", s, buf, n);
	if (m_closing) 																				// if close in progress
	{	usleep(2000);																			// let it happen
		return(-1);																					// closing, fails
	}
	ost::MutexLock lok(m_reading);														// read lock
    int bcount; /* counts bytes read */
    int br;     /* bytes read this pass */

    struct timeval timeout;
    //	timeout after a second.
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;

    bcount= 0;
    br= 0;
    while (bcount < n)
    {
        /* loop until full buffer */

        int s = m_sock;																			// socket port
        if (s < 0)
        {
            usleep(100000);
            return(-1);																				// not open, fails
        }
        fd_set rfd;
        FD_ZERO(&rfd);
        FD_SET(s, &rfd);

        // block until data or time out
        switch(select(s+1, &rfd, 0, 0, &timeout))
        {
        case -1:
            return -1;
            break;
        case 0:
            if (m_verbose)
                logprintf("read_data: select timed out.\n");
            return (-2);
            break;
        default:																				// normal read
            break;
        }
        //	Actually read
        if ((br = read(s,buf,n-bcount)) > 0)
        {
            bcount += br;                /* increment byte counter */
            //for (int x = 0; x < br; x++) {
            //	printf("read: %02x (c:%c) (i:%i)\n", *buf, *buf, *buf);
            // 	buf++;
            //}
            buf += br;                   /* move buffer ptr for next read */
            //printf("read through byte %i\n", bcount);
        }
        else if (br < 0)               /* signal an error to the caller */
        {
            logprintf("read_data: Failed, reading byte: %i (of %i) from socket: %i \n", bcount, n, s);
            close();												// must close to prevent read of EOF socket and pipe fault
            return(-1);
        }
        else if (br == 0)
        {
            logprintf("read_data: Got EOF when asked to read (%i bytes) from socket: %i.\n", n, s);
            close();												// must close to prevent read of EOF socket and pipe fault
            return (-1);
        }
    }
    return(bcount);
}
//
//	write_data  -- write data to connected socket
//
//	Used for sending commands to LMS.
//
//	We don't actually send much. Once it's up, we just receive.
//
//	The delay between bytes is required by the LMS, which requires a 55us delay between bytes.
//
int LidarSocket::write_data(
    const char *buf, /* pointer to the buffer */
    int n      /* number of characters (bytes) we want */
)
{
	if (m_closing) 																				// if close in progress
	{	usleep(2000);																			// let it happen
		return(-1);																					// closing, fails
	}
	int stat  = 0;																					// write status																		
	{	ost::MutexLock lok(m_writing);													// write lock
	    int s = m_sock;																			// get socket
	    if (s < 0)																					// not open, try again later
	    {	logprintf("write_data - not connected to LMS.\n");
 	       return(-1);
	    }
	    //	Write, with timeout
	    const uint64_t timeout = 2*1000000000;									// 2 seconds in nanoseconds
	    for (int i=0; i<n; i++)																// write slowly, per LMS spec
	    {	TimerTimeout(CLOCK_REALTIME, _NTO_TIMEOUT_SEND | _NTO_TIMEOUT_REPLY, NULL, &timeout, NULL);
			int cnt = write(s,&buf[i],1);													// write requested data, one byte at a time
			if (cnt != 1)
	       {
				logprintf("Error writing to TCP socket: %s.\n", strerror(errno));
				stat = -1;
				break;
			}
	        usleep(1000);																		// wait 1ms between bytes
		}
	}																									// end locked section
	//	Must not close in locked section. Locking order problem causes deadlock.  Must lock read, then write.
	if (stat < 0)																					// if trouble
	{	close();																						// close and force reopen
		return(stat);																				// fails
	}
    if (m_verbose)
    {
        logprintf("write_data: sent %i bytes\n", n);
    }
    return(0);																						// return status
}



