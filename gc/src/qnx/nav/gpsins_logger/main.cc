//
//	gpsins_logger  --  logger for GPS/INS data
//
//	Creates log files for later evaluation.
//
//	Client of the move server.
//	Runs under the watchdog
//
//	Team Overbot
//	J.	Nagle
//	January, 2005
//
#include <stdio.h>
#include <stdlib.h>
#include "logger.h"
//
//	Constants
//
const char* k_default_log_directory = "/tmp/logs";		// where to put logs if not otherwise specified
//
//	usage  -- usual usage
//
static void usage()
{	printf("Usage: gpsins_logger [-d logdir] [-i]\n");
	printf("  -i	accept non-Omnistar GPS data (low quality)\n");
	printf("Runs under watchdog.\n");
	exit(1);
}
//
//	Main program
//
int main(int argc, const char* argv[])
{	const char* logdir = 0;												// need a log directory
	const float interval = 0.10;										// query GPS server this often
	bool acceptbad = false;
	////bool waypointlog = true;											// assume we are doing a waypoint log
	for (int i=1; i<argc; i++)
	{	const char* arg = argv[i];
		if (arg[0] == '-')
		{	switch(arg[1]) {
			case 'd':															// -d logdirectory
				i++;																// advance arg
				if (i>=argc) usage();
				logdir = argv[i];											// save logs in this directory
				continue;
				
			case 'i':																// -i	accept non-Omnistar data
				acceptbad = true;
				continue;
				
			default: usage();
			}
		} else {																// file arg
			usage();															// there are no file args
		}
	}
	if (!logdir) logdir = k_default_log_directory;				// use default if none
	Logger waypointlogger;											// the logger object
	waypointlogger.client(logdir,interval, acceptbad);	// make waypoint logs
	return(0);
}