////////////////////////////////////////////////////////////////////////////
//
//    File: main.cc
//
//    Usage:
//        See speedserver.h.
//
//    Description:
//        See speedserver.h.
//
//        The Speed server consists of the following threads:
//            - a main thread, which accepts messages
//
//    Written By:
//        Achut Reddy
//        Team Overbot
//        January 2004
//
/////////////////////////////////////////////////////////////////////////////

#include "speedserver.h"
#include <stdio.h>
//
//  Usage - print usage message and exit
//
static void usage()
{
    printf("usage: speed [-v] [-s] [-t targets]\n");
    printf("  -v    verbose\n");
    printf("  -s    dummy simulation mode for testing\n");
    printf("  -t		dummy VORAD target location file for testing only.\n");
    exit(1);
}
//
//	Main program
//
int main(int argc, const char** argv)
{	bool verbose = false;
	bool dummysim = false;
	const char* dummytargetfile = 0;		// no file arg yet
    // parse command line arguments
    for (int i=1; i < argc; i++)
    {	
        const char *arg = argv[i];
        if (arg[0] == '-')
        {
            switch (arg[1])
            {
            case 's':									// simulation mode
                dummysim = true;
                break;
                
           case 'v':										// verbose mode
           		verbose = true;
           		break;
           		
         	case 't':										// dummy target file
         		if (dummytargetfile) usage();
         		i++;										// to next arg
         		if (i >= argc) usage();			// too many args
         		dummytargetfile = argv[i];	// save file
         		break;

            default:
                usage();
                break;
            }
        }  else {									// non-flag arg
        	usage();
        }
    }
    //	Start the thread.
    //	Exceptions are caught and produce messages
    try {
    	if (dummysim)
    	{	//	Dummy mode - no actual vehicle required
    		printf("DUMMY TEST MODE - no connection to vehicle.\n"); fflush(stdout);
			SpeedServerDummy ms;		
			if (dummytargetfile)				// if there is a dummy target file
			{	int stat = ms.loaddummyobstacles(dummytargetfile);	// read it in
				if (stat < 0) exit(1);			// fails if bad
			}
			//	Start dummy GPS/INS server
			ms.startdummygpsinsserver();												
			// start collecting messages forever
			ms.messageThread(verbose);
    	} else {
			SpeedServerChassis ms;														
			// start collecting messages forever
			ms.messageThread(verbose);
		}
	}
	catch(const char* msg)
	{	fflush(stdout);
		printf("EXCEPTION: %s\n",msg);
		fflush(stdout);
		abort();
	}
	catch(const std::exception& except)
	{	fflush(stdout);
		printf("EXCEPTION: %s\n",except.what());
		fflush(stdout);
		abort();
	}
	abort();																							// should never be reached
}
