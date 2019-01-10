////////////////////////////////////////////////////////////////////////////
//
//    File: main.cc
//
//    Usage:
//        See moveserver.h.
//
//    Description:
//        See moveserver.h.
//
//        The Move Server consists of the following threads:
//            - a main thread, which accepts messages
//            - a command thread
//            - a menu thread
//
//    Written By:
//        Achut Reddy
//        Team Overbot
//        December 2003
//
/////////////////////////////////////////////////////////////////////////////

#include "moveserver.h"

//
//  Usage - print usage message and exit
//
static void usage()
{
    printf("usage: move [-v]\n");
    exit(1);
}
//
//	Main program
//
int main(int argc, const char **argv)
{
    MoveServer ms;
    bool verbose = false;
    // parse command line arguments
    for (int i=1; i < argc; i++)
    {
        const char *arg = argv[i];
        if (arg[0] == '-')
        {
            switch (arg[1])
            {
            case 'v':
            	verbose = true; continue;

            default:
                usage();
                break;
            }
        }
        usage();
    }
    // start collecting messages forever
    ms.messageThread(verbose);
    return(0);
}
