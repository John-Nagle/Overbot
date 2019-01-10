//
//	Remote serial port access utility program
//
//	Connects a serial port on one machine to a serial port on another.
//
//	Used to help diagnose Novatel GPS problems.
//
//	John Nagle
//	Team Overbot
//	September, 2005
//
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <pthread.h>
#include <sys/select.h>

//
//	class RemoteSerial  -- one per remote serial port pair
//
class RemoteSerial {
private:
	int m_fd1;						// file descriptor for one end
	int m_fd2;						// file descriptor for other end
	bool m_verbose;			// true if verbose
public:
	RemoteSerial()
	: m_fd1(-1), m_fd2(-1)
	{}
	int init(const char* fname1, const char* fname2, int baudrate);
	void run();					// do it
	void setverbose(bool verbose) { m_verbose = verbose; }
private:
    static void* thread1start(void* arg)					// start the thread
	{ return(reinterpret_cast<RemoteSerial*>(arg)->thread1()); }
	void* thread1();			// read from 1, write to 2
	void *thread2();			// read from 2, write to 1
	void docopy(int fd1, int fd2);		// does the copy for both threads
	int setbaud(int fd, int baudrate);
};
//
//	Implementation
//
//
//	run -- actually does the work
//
//	Does not return
//
//	Files must already be open
void RemoteSerial::run()
{
	pthread_create(0, 0, thread1start, this);				// start thread
	thread2();																// just do other thread
}
//
//	thread1  -- copy one way
//
void* RemoteSerial::thread1()
{	docopy(m_fd1, m_fd2);	
	return(0);
}
//
//	thread2  -- copy other way
//
void* RemoteSerial::thread2()
{	docopy(m_fd2, m_fd1);	
	return(0);
}
//	
//	docopy -- actually do the copy
//
void RemoteSerial::docopy(int fd1, int fd2)
{
	const size_t k_bufsize = 8192;
	char* buf[k_bufsize];
	for (;;)
	{	fd_set fdset;													// file descriptor set
		FD_ZERO(&fdset);											// clear file descriptor set
		FD_SET(fd1, &fdset);										// set FD
		int stat = select(fd1+1, &fdset, 0, 0, 0);		// wait for input
		if (stat < 0)
		{	perror("Serial port select error"); exit(1); }
		if (stat == 0)													// 
		{	printf("Unexpected select timeout.\n"); exit(1); }
		int cnt = read(fd1, buf, k_bufsize);				// read
		if (cnt < 0)
		{	perror("Serial port read error");						// fails
			exit(1);
		}
		if (cnt == 0) 
		{	printf("End of file on serial port.\n");	
			exit(1);
		}
		if (m_verbose)													// debug
		{	printf("Read %d bytes from #%d\n", cnt, fd1);	}
		cnt = write(fd2, buf, cnt);									// write - will fail with SIGPIPE if closed
		if (cnt < 0)
		{	perror("Serial port write error");						// fails
			exit(1);
		}
	}
}
//
//	init -- initialize
//
int RemoteSerial::init(const char* fname1, const char* fname2, int baudrate)
{	//	Open serial ports
	m_fd1 = open(fname1, O_RDWR);						// open
	if (m_fd1 < 0)
	{	printf("Unable to open %s: %s\n", fname1, strerror(errno));
		exit(1);
	}
	m_fd2 = open(fname2, O_RDWR);						// open
	if (m_fd2 < 0)
	{	printf("Unable to open %s: %s\n", fname2, strerror(errno));
		exit(1);
	}
	// Set baud rate
	int stat = setbaud(m_fd1, baudrate);
	if (stat < 0)
	{	perror("Unable to set baud rate");	exit(1); }
	stat = setbaud(m_fd2, baudrate);
	if (stat < 0)
	{	perror("Unable to set baud rate");	exit(1); }
	printf("Connecting \"%s\" to \"%s\" at %d baud.\n", fname1, fname2, baudrate); fflush(stdout);
	return(0);	// success
}
//
//	setbaud -- set baud rate
//
int RemoteSerial::setbaud(int fd, int baud)
{
    speed_t speed;
	struct termios termios_p;
    if ( tcgetattr(fd, &termios_p) == -1 ) {
        return -1;
    }
    // configure raw mode
    cfmakeraw(&termios_p);			// set to raw mode
    // set baud rate
    switch ( baud ) {
        case   1200:
            speed =   B1200;
            break;
        case   2400:
            speed =   B2400;
            break;
        case   4800:
            speed =   B4800;
            break;
        case   9600:
            speed =   B9600;
            break;
        case  19200:
            speed =  B19200;
            break;
        case  38400:
            speed =  B38400;
            break;
        case  57600:
            speed =  B57600;
            break;
        case  76800:
            speed =  B76800;
            break;
        case 115200:
            speed = B115200;
            break;
        default:
            speed = B9600; // default
            fprintf(stderr, "Serial::ConfigSet - baud rate = %i not valid\n", 
                    (int) baud);
            break;
    }
    if ( cfsetispeed(&termios_p, speed) == -1 ) {
        return -1;
    }
	if ( cfsetospeed(&termios_p, speed) == -1 ) {
        return -1;
    }
	// configure modes - control characters
	////termios_p.c_cc[VMIN]  = 0;           // read() as much data as possible
	////termios_p.c_cc[VTIME] = 0;           // without waiting

	// set port attributes
	if ( tcsetattr(fd, TCSANOW, &termios_p) == -1 ) {
		return -1;
	}
	return 0;
}
//
//	Usage -- usual usage
//
void usage()
{	printf("Usage: remoteserial [-b baudrate -v verbose] port1 port2\n");
	exit(1);
}
//
//	Main program
//
int main(int argc, const char* argv[])
{
	const char* fname1 = 0;						// serial port names
	const char* fname2 = 0;
	bool verbose = false;
	int baudrate = 57600;
	for (int i=1; i < argc; i++)						// for all args
	{	const char* arg = argv[i];				// this arg
		if (arg[0] == '-')								// if flag
		{
			switch (arg[1]) {							// fan out on arg
			case 'b':										// baud rate
				i++;											// go to next arg
				if (i >= argc) usage();				// no next arg
				baudrate = atoi(argv[i]);			// get arg
				break;
				
			case 'v':										// verbose
				verbose = true;						// verbose mode
				break;
				
			default:	usage();
			}
		} else {											// filename arg
			if (fname1 == 0)
			{	fname1 = arg; }
			else if (fname2 == 0)
			{	fname2 = arg; }
			else usage();
		}
	}
	if (fname2 == 0) usage();						// no args
	RemoteSerial ser;									// object to work on
	ser.setverbose(verbose);						// set verbosity
	int stat = ser.init(fname1, fname2, baudrate);		// do it
	if (stat < 0) exit(1);								// fails
	ser.run();												// run it
}