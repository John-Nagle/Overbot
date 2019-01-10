////////////////////////////////////////////////////////////////////////////
//
//    File: main.cc
//
//    Usage:
//	  roadserver [-v] [-d directory]
//
//    Description:
//        See roadserver.h.
//
//    Written By:
//        Tim Nicholson
//        Team Overbot
//        January 2004
//
/////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdlib.h>

#include "roadserver.h"

//
//  Usage - print usage message and exit
//
static void
usage()
{
    printf("usage: roadserver [-v] [-d directory]\n");
    exit(1);
}

int
main(int argc, char **argv)
{
    RoadServer rs;
    char *filedirectory = NULL;

    // parse command line arguments
    for (int i=1; i < argc; i++) {
		const char *arg = argv[i];
		if (arg[0] == '-') {
		    switch (arg[1]) {
		    case 'd':		// waypoint file
		        if (++i >= argc)
			    	usage();
				filedirectory = argv[i];
			break;
			    
		    case 'v':
		    	rs.setVerbose(true);
			break;
			    
		    default:
				usage();
			break;
		    }
		}
    }

    // read road database
    rs.readRoadFile (filedirectory);
    
    // get waypoints from waypoint server
    rs.get_waypoints();
    rs.waypoint_find_roads(); // find roads in waypoint corridor

    // start collecting messages forever
    rs.messageThread();
}

