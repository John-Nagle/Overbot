////////////////////////////////////////////////////////////////////////////
//
//    File: main.cc
//
//    Usage:
//	 waypt [-v] [-f waypointfile]
//
//    Description:
//        See wayptserver.h.
//
//	  Reads in a list of waypoints from waypointfile, or from stdin
//	  if no file is specified.
//
//        The Waypt Server consists of the following threads:
//            - a main thread, which accepts messages
//
//    Written By:
//        Achut Reddy
//        Team Overbot
//        January 2004
//
/////////////////////////////////////////////////////////////////////////////

#include <stdio.h>

#include "wayptserver.h"

//
//  Usage - print usage message and exit
//
static void
usage()
{
    printf("usage: waypt [-v] [-f waypointfile]\n");
    exit(1);
}

int
main(int argc, char **argv)
{
    WayptServer ws;
    char *fileName = NULL;

    // parse command line arguments
    for (int i=1; i < argc; i++)
    {
        const char *arg = argv[i];
        if (arg[0] == '-')
        {
            switch (arg[1])
            {
            case 'f':		// waypoint file
                if (++i >= argc)
                    usage();
                fileName = argv[i];
                break;

            case 'v':
                ws.setVerbose(true);
                break;

            default:
                usage();
                break;
            }
        }
    }

    // read Waypoints
    int err = ws.readWaypts(fileName);
    if (err != 0)
    {
        fprintf(stderr, "wayptserver: errors reading waypoint file; quitting\n");
        exit(1);
    }

    // start collecting messages forever
    ws.messageThread();
}

