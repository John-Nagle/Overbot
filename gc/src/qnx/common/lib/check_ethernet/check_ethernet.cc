/////////////////////////////////////////////////////////////////////////////
//
//    File: check_ethernet.cc
//
//    Usage:
//        check_ethernet
//
//    Description:
//        A test file for ethernet.cc and ethernetmenu.cc.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        January 2003
//
////////////////////////////////////////////////////////////////////////////

#include "ethernet.h"
#include "ethernetmenu.h"
#include "ask.h"
//
//	usage  -- print usage and exit
//
static void usage()
{	printf("Usage: check_ethernet [flags]  hostname\n"); 
	printf("  -u    Use UDP (defaults is TCP)\n");
	exit(1);
}

int main(int argc, const char* argv[])
{
	if (argc < 2)
	{	usage(); }
	const char* hostname = 0;
	bool datagrammode = false;							// not datagram mode
	for (int i=1; i<argc; i++)
	{	const char* arg = argv[i];							// this arg
		if (arg[0] == '-')											// if flag arg
		{	switch(arg[1]) {										// fan out on flag arg
			case 'u':	datagrammode = true; break;	// -u datagram mode
			default: usage(); break;
			}
		} else {														// non-flag arg
			if (hostname) usage();								// already have hostname
			hostname = arg;
		}	
	}
	if (!hostname) usage();										// no host name yet
	printf("Using %s protocol.\n", datagrammode ? "UDP" : "TCP"); 
	try {
		Ethernet e(hostname,0,100000);					// get name from arg 0
		e.DatagramMode(datagrammode);				// set mode
		char msg[100];
		snprintf(msg,sizeof(msg), "Ethernet TCP/IP connection (%s)",hostname);
 	   	EthernetMenu em(&e, msg, 'e', 'g');
	    MenuHandler mh;
  		mh.Install(&em);
  		mh.Start();
  	}
  	catch (const char* msg)
  	{	fprintf(stderr,"Exception: %s.\n",msg);
  		exit(1);																	// fails
  	}
  	catch (...)
  	{	fprintf(stderr,"Exception (unknown type).\n");
  		exit(1);
  	}
  	
  	
}