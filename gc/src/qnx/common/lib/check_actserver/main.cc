#include "check_actserver_.h"
#include "check_actservermenu.h"

//
//  Usage - print usage message and exit
//
static void Usage()
{
	printf("Usage: check_actserver  [-m]\n");
    exit(1);
}

int main(int argc, char ** argv)
{
	MyServer_ ms;

	// parse command line arguments
    for ( int i = 1 ; i < argc; i++ ) {
    	const char* arg=argv[i];
        if ( arg[0] == '-' ) {
        	switch( arg[1] ) {
            	case 'm':
					// start thread to make menus available
					ms.MenuThread();
            		break;
            	default:
            		Usage();
					break;
            }
            continue;
        }
        Usage();
    }
    
	// start thread to issue target commands to MVP Server
	ms.TargetThread();
	
	// start timed-loop thread to collect data from MVP Server
	// start paused for illustration
	ms.DataLoopPause();
	ms.DataLoopStart();

	// start timed-loop control loop
	// start paused for illustration
	ms.ControlLoopPeriod(0.1);
	ms.ControlLoopTimeout(2.0);
	ms.ControlLoopPause();
	ms.ControlLoopStart();

	// start collecting messages forever
	ms.MessageThread();
}