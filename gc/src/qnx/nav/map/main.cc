//
//	main.cc -- map server -- main program
//
//	Normally run under watchdog as part of the real-time Overbot system, but can also be run
//	as a standalone program with GPS and LIDAR files as input.
//
//	An early version was by Achut Reddy, but the code was completely rewritten.
//	John Nagle
//	Team Overbot
//	December, 2004
//
//
//	Copyright 2005 by John Nagle
//	999 Woodland Avenue
//	Menlo Park, CA  94025
//
//	This program is free software; you can redistribute it and/or modify
//	it under the terms of the GNU General Public License as published by
//	the Free Software Foundation; either version 2 of the License, or
//	(at your option) any later version.

//	This program is distributed in the hope that it will be useful,
//	but WITHOUT ANY WARRANTY; without even the implied warranty of
//	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//	GNU General Public License for more details.

//	You should have received a copy of the GNU General Public License
//	along with this program; if not, write to the Free Software
//	Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//
#include <exception>
#include "mapserver.h"

//
//  Usage - print usage message and exit
//
static void
usage()
{
    printf("usage: map  [-v] [-s scannerlogfilein] [-g gpslogfilein] [-w waypointfile] [-d logdir]\n");
    exit(1);
}
//
//	The main program
//
int main(int argc, const char* argv[])
{
    static MapServer ms;
	const char* dummylidarin = 0;									// dummy LIDAR file for input
	const char* dummygpsin = 0;									// dummy GPSINS file for input
	const char* logdir = 0;												// log dir, if desired
	const char* waypointin = 0;										// input waypoint file
	int verboselevel = 0;												// no verbose level yet
    // parse command line arguments
    for (int i=1; i < argc; i++) {
    	const char *arg = argv[i];
        if (arg[0] == '-') {
	    switch(arg[1] ) {

	    case 'v':	// verbose mode
	    	ms.setVerboseLevel(++verboselevel);			// each -v increases the verbosity level
	    	break;
	    	
	    case 's'	:																// replay mode -- take scanner input
			i++;
			if (i >= argc) usage();										// must have another arg
			dummylidarin = argv[i];									// -r filename
			break;
			
	    case 'g'	:																// replay mode -- take gps input
			i++;
			if (i >= argc) usage();										// must have another arg
			dummygpsin = argv[i];										// -g filename
			break;
			
		case 'd':																// logging directory
			i++;
			if (i >= argc) usage();										// must have another arg
			logdir = argv[i];												// -d filename
			break;
			
		case 'w':																// waypoint file
			i++;
			if (i >= argc) usage();										// must have another arg
			waypointin = argv[i];										// -w filename
			break;
			

	    default:																	// unknown flag
			usage();
			break;
            }
            continue;
        } 
        usage();
    }
    try {
	    if (!dummylidarin)
	    {	// start collecting messages forever
	    	//	***NEEDS WORK for real operation***
	    	//	***NO WAY TO EXERT CONTROL ONCE RUNNING***
	    	bool good = ms.executeMission(waypointin, logdir);	// load up a mission
	    	if (!good)
	    	{	throw("Unable to start mission.");	}
		    ms.messageThread();											// run as a server to get LIDAR data
		} else {																	// reading dummy data files
			ms.playbackTest(dummylidarin, dummygpsin, waypointin, logdir);		// read dummy data files
		}
		return(0);																	// success
	}
	catch (const char* msg)
	{	printf("Exception: %s\n",msg);	fflush(stdout); sleep(1); exit(1);	}
	catch(std::exception& exc)
	{	printf("Exception: %s\n",exc.what()); 	fflush(stdout); sleep(1); exit(1);	}
	catch(...)
	{	printf("Unknown exception.\n"); 	fflush(stdout); sleep(1); exit(1); }
}
