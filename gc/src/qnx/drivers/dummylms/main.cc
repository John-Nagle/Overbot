////////////////////////////////////////////////////////////////////////////
//
//    File: main.cc
//
//    Usage:
//        See lidarserver.h.
//
//    Description:
//        See lidarserver.h.
//
//        The Lidar Server consists of the following threads:
//            - the main thread, which accepts messages, sends to SICK unit
//            - a timed loop thread, which collects data on a regular basis
//
//    Written By:
//        Eric Seidel
//		  (Modeled after code from Celia Oakley)
//        Team Overbot
//        December, 2003
//
/////////////////////////////////////////////////////////////////////////////

#include <unistd.h>
#include <string> 
#include "logprint.h"
#include "lidarserver_.h"



// static, necessary for cleanup:
static LidarServer_	*ls = 0;
bool exiting = false;

void handleEXITSIGNAL(int signo)
{
	if (exiting) {
		logprintf("\n\nReceived a second exit signal, exiting immediately!  Socket may get improperly shutdown...\n");
		exit(1);
	}
	exiting = true;
	
	logprintf("Shutting down...\n");
	
	if (ls)
		delete ls;
	exit(0);
}



//
//  Usage - print usage message and exit
//
static void Usage()
{
	logprintf("Usage: lidar  [-v] [-c <clientID>]");
	exit(1);
}

//
// Main - the guts of the program
// 
int main(int argc, char ** argv)
{
	/* Before we do anything else, set up signal handlers */
	struct sigaction act, oact;
	act. sa_handler = &handleEXITSIGNAL;
	sigemptyset(&act.sa_mask);
	act.sa_flags = 0;
	if (sigaction(SIGTERM, &act, &oact) < 0) 
	{	logprintf("LidarServer: an error occured while try to set up SIGTERM handler.\n");
		exit(1);
	}
	if (sigaction(SIGINT, &act, &oact) < 0)
	{	logprintf("LidarServer: an error occured while try to set up SIGINT handler.\n");
		exit(1);
	}
	////signal(SIGPIPE,SIG_IGN);										// ignore problems with write to closed sockets
	
	ls = new LidarServer_();											// allocate forever; object contains threads
	
	std::string clientID =  LIDARSERVER_CLIENTID;			// we're the client, this is the default server name
	
	// parse command line arguments
	for ( int i = 1 ; i < argc; i++ ) {
		const char* arg=argv[i];
		if ( arg[0] == '-' ) {
			switch( arg[1] ) {

				case 'c':														// specify target for output messages
					if ( i < (argc-1) ) {
						clientID = argv[++i];								// copy target name
					} else {
						Usage();
					}
					break;
										
				case 'v':														// verbose
					ls->SetVerbose(true);								// set verbose mode
					logprintf("Verbose mode.\n");		
					break;
					
				default:
					Usage();
					break;
			}
			continue;
		}
		Usage();
	}
	
	
	// FIX - could allow for changing timeout.
	ls->SetupClientConnection(clientID.c_str(), 0.0);
		
	// start collecting messages forever
	ls->MessageServer();
	//	Not reached
}