//
//	telnetclient  --  dummy test telnet client for telnet option library
//
//	Opens socket, attempts to negotiate options, prints option negotiation.
//
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <ctype.h>
#include <time.h>
#include "socketrx.h"
#include "telnetcomport.h"
//
//	constants
//
const uint32_t k_printinterval = 50000;						// print every N bytes

using namespace std;
//
//	usage -- print usage
//
static void usage()
{	printf("Usage:	telnetclient [options] hostname [portnumber]\n");
	printf("  Options: -v  verbose\n");
	printf("				  -l   loopback test (use loopback plug)\n");
	exit(1);
}
//
//	getnow  -- get time seconds, as a double
//
double getnow()
{	struct timespec now;
	clock_gettime(CLOCK_REALTIME,&now); 				// get current time
	return(double(timespec2nsec(&now))*1e-9);		// in seconds, as a double
}

//
//	TestTelnetSocket - Derived class with "deliver" function, for testing
//
class TestTelnetSocket: public TelnetComPort
{
public:
	bool m_loopback;													// true if loopback test
	uint8_t m_expected;												// the byte expected
	uint32_t m_count;													// count of bytes sent
	TestTelnetSocket() 
	:	m_loopback(false), m_expected(0),m_count(0) {}
	void deliver(const uint8_t msg[], uint32_t len);		// called to deliver data for us to print
	void reset() 
	{	m_expected = 0; m_count = 0; }
};
//
//	deliver -- display data delivered from Telnet connection
//
//	Dump routine, for debug.
//
//	The loopback test just sends bytes 0..255 over and over, checking the result.
//	Any error stops the test.
//
void TestTelnetSocket::deliver(const uint8_t msg[], uint32_t len)
{	if (len == 0)
	{	printf("\nTimeout.\n");	return;	}						// no data means a timeout
	if (m_loopback)														// if loopback test
	{	static double starttime = 0;								// get time now in seconds
		if (m_count  == 0) starttime = getnow();			// get time first time through
		for (uint32_t i=0; i<len; i++)				
		{	uint8_t ch = msg[i];										// get char
			if (ch == m_expected)									// if expected, fine
			{	m_expected++; 
				m_count++;												// continue count
				if (m_count % k_printinterval == 0)			// every megabyte
				{	double now = getnow();						// time now
					double elapsed = now - starttime;		// elapsed time
					starttime =  now;									// reset for next interval
					double byterate = k_printinterval / elapsed;	// bytes/second
					double baudrate = byterate * 11;		// 8 data bits, 1 start, 2 stop
					printf("Loopback received %d bytes OK at %1.1f bytes/sec (%1.1f async baud).\n", m_count, byterate, baudrate); 
				}
				continue;
			}
			if (m_count < 2)											// 
			{	static bool junkprint = false;		
				if (!junkprint) printf("NOTE: Junk bytes before good data - ignored\n");
				junkprint = true;
				continue; 													// ignore leading junk
			}
			printf("Loopback error after %d bytes received. Expected %d, received %d\n",
				m_count, m_expected, ch);
			m_loopback = false;											// terminate loopback test		
			return;
		}
		return;
	}
	//	Not loopback, just dump
	for (uint32_t i=0; i<len; i++)
	{	uint8_t ch = msg[i];											// get char
		if (isgraph(ch))													// if printable
		{	putchar(ch);	}												// print
		else
		{	printf("\\0x%02x ",ch);	}								// otherwise print as hex
	}
}
//
//	loopbacktest -- run loop back test
//
static int loopbacktest(TestTelnetSocket& telnetsocket, const char* hostname, int portid, uint32_t baudrate)
{
	telnetsocket.m_loopback = true;							// set loopback test mode
	telnetsocket.reset();												// reset loopback counters
	int stat = telnetsocket.open(hostname,portid);		// create net object
	if (stat)																	// if fail
	{	perror("Open of socket failed");						// report
		return(-1);															// fails
	}
	uint32_t rdbaud;													// baud rate read
	stat = telnetsocket.purgedata();							// get rid of any remaining data
	if (stat < 0)															// if fail
	{	perror("Could not send purge data");				// report
		return(-1);															// fails
	}
	stat = telnetsocket.getbaudrate(rdbaud);				// get the old baud rate
	if (stat < 0)															// if fail
	{	perror("Could not get baud rate");					// report
		return(-1);															// fails
	}
	printf("Old baud rate = %u\n", rdbaud);				// get previous baud rate
	telnetsocket.setbaudrate(baudrate);					// set new baud rate
	stat = telnetsocket.getbaudrate(rdbaud);				// get the old baud rate
	if (stat < 0)															// if fail
	{	perror("Could not get baud rate");					// report
		return(-1);															// fails
	}
	printf("New baud rate = %d\n", rdbaud);				// get previous baud rate
	uint8_t buf[256];												// test data
	printf("Loopback test in progress.\n");
	for (unsigned int i=0; i<sizeof(buf); i++) buf[i] = i;			// make test data, 0..255
	for (unsigned int i=0; i<k_printinterval/256*3; i++)								// many blocks
	{	int stat = telnetsocket.write(buf,sizeof(buf));// write to port
		if (stat < 0)
		{	perror("Write to network failed");
			exit(1);														// fails
		}
	}
	telnetsocket.close();												// close, which shuts everything down
	return(0);
}
//
//	main program
//
int main(int argc, char* argv[])
{
	const char* hostname = 0;									// no hostname yet
	const char* portnumber = 0;								// no port number yet
	int verbose =0;														// not verbose
	bool loopback = false;											// not loopback
	for (int i=1; i< argc; i++)										// for all args
	{	const char* arg = argv[i];								// this arg
		if (arg[0] == '-')												// if flag
		{	switch (arg[1])	{											// fan out on flag arg
			case 'v': verbose++; break;							// set verbose mode, more is more verbose
			case 'l':	loopback = true; break;					// set loopback mode
			default:															// not valid
				usage();
			}
		} else {															// if non-flag arg
			if (!hostname)												// want hostname
			{	hostname = arg; }
			else if (!portnumber)									// then portnumber
			{	portnumber = arg; }
			else usage();
		}
	}
	if (!hostname) usage();											// must have host name
	if (!portnumber) portnumber = "4680";					// default port is Sealevel simulated com port
	int portid = atoi(portnumber);								// get port number as integer
	printf("Connecting to \"%s\", port %d\n", hostname, portid);	
	TestTelnetSocket	telnetsocket;								// create net object
	telnetsocket.setverbose(verbose);						// if verbose mode wanted
	telnetsocket.m_loopback = loopback;					// set loopback test mode 
	if (loopback)
	{	for (int i=0; i<10; i++)
		{	uint32_t baudrate = (i%4 == 0) ? 57600 : 500000;	// toggle baud rate
			loopbacktest(telnetsocket, hostname, portid, baudrate);			// do the loopback test
		}
		printf("Tests passed.\n");
	}
	printf("Exiting.\n");													
	return(0);																// done
}
