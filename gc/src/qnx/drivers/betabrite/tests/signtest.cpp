//
//	signtest  -- test for sign
//	J.	Nagle
//	November, 2003
//
//	Exercises sign serer  program
//
//	Usage: sign [options] msg
//
//	Options:	
//		-p																			// "priority" mesage
//		-u																			// "urgent" message
//	
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>
#include "messaging.h"
#include "signserver.h"
//
//	Statics
//
static bool verbose = false;											// verbose mode
//
//	usage -- print usage message and exit
//
static void usage()
{	printf("Usage: signtest [-u] [-p]  msg\n");	// print message
	exit(1);
}


//
//	Main program
//
int main(int argc, const char* argv[])
{	const char* msg = 0;												// no msg yet
	SignServerMsgSNTX::SignPriority priority = SignServerMsgSNTX::Routine;	// assume routine priority
	//	Parse input arguments
	for (int i=1; i<argc; i++)											// for all args
	{	const char* arg= argv[i];										// this arg
		if (arg[0] == '-')													// if flag argument
		{	switch(arg[1])	{												// interpret flags
			case 'v': verbose = true;	 break;						// set verbose mode
			case 'p': priority = SignServerMsgSNTX::Priority; break;
			case 'u': priority = SignServerMsgSNTX::Urgent; break;
			default: usage();												// bad call, fails
			}
			continue;															// next arg
		}
		//	Not flag, must be the message to display
		if (msg) { usage(); }
		msg = arg;															// save as arg
	}
	if (!msg) usage();
	printf("Sign: %s\n",msg);											// show displayed message
	for (int i=0; i<5; i++)												// try several times if busy
	{	int stat = SignDisplay(msg,priority);						// display message
		if (stat >= 0) return(0);											// success
		if (errno == EBUSY)
		{	printf("Sign busy, will retry later.\n");
			sleep(1);
			continue;
		}
		printf("SignDisplay error: %s\n",strerror(errno));	// trouble
		break;
	}
	exit(1);																	// fails
}
