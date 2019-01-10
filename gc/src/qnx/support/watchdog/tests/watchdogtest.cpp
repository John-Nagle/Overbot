//
//	Watchdogtest  -- test for watchdog
//	J.	Nagle
//	September, 2003
//
//	Exercises watchdog program
//
//	Usage: watchdogtest [options]
//
//	Options:	
//		-s																			// act as server - just echo any message received
//		-c ID																		// act as client - send to ID, check for but ignore replies.
//		-h NNN																	// hang for 10 seconds after idling for NNN seconds
//	
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>
#include "messaging.h"
//
//	Statics
//
static bool verbose = false;											// verbose mode
static bool server = false;												// true if running a server
static bool slowserver = false;										// if slow server test
static char* servername ="testserver";						// name of server to talk to if client
const int maxmsgcount = 10000;									// print this often
const size_t msgsize = 1000;										// message size
////static int hangtime = 0;												// no hang to test CPU hang
//
//	usage -- print usage message and exit
//
static void usage()
{	printf("Usage: watchdogtest [-v -s -c ID -w] \n");	// print message
	exit(1);
}
//
//	testmsg -- the server and client exchange these
struct testmsg {
	char ch[msgsize];
	int n;
};
//
//	runserver -- run as a server
//
//	This server accepts request messages, does some computation, and returns the result.
//
void runserver()															
{
	//	The server normally has the name given the program by the watchdog file. This is in the env. variable "ID"
	MsgServerPort serverport(0.0);								// define message port, no timeout
	if (verbose) serverport.setverbose();						// more talkative
	int stat = serverport.ChannelCreate();						// create a channel, tell watchdog about it
	if (stat) 
	{	perror("ChannelCreate failed in server"); 
		serverport.Dump();
		abort();																	// fails
	}
	for (;;)																		// forever
	{	testmsg msgin;														// area for incoming msg
		int rcvid = serverport.MsgReceive(msgin);	// get msg
		if (rcvid < 0)															// if error
		{	fflush(stdout); perror("MsgReceive failed in server");				// fails
			sleep(1);															// avoid tight loop if repeated trouble
			continue;															// fails
		}
		if (rcvid == 0)														// pulse
		{	printf("Server received a pulse.\n");					// pulses don't require a reply
			continue;
		}
		//	We have received a message
		if (verbose) printf("Server received a message.\n");				// note msg
		if (slowserver)														// if running in slow mode
		{	static int cycle = 0;											// cycle count
			 if (cycle++ > 1000)											// every 1000 cycles, stall for a second
			{	sleep(1);														// Slow message timeout test
				cycle = 0;														// reset counter
			}
		}
		int err = MsgReply(rcvid, msgin);							// reply to msg by sending it back.
		if (err) perror("MsgReply failed in server");
		////serverport.watchdogreset();								// tell the watchdog we are still alive
	}
}
//
//	runclient -- run as a client
//
//	This server accepts request messages, does some computation, and returns the result.
//
void runclient()															
{	int msgcount = 0;														// messages sent
	uint64_t starttime;
	ClockTime(CLOCK_REALTIME,0,&starttime);				// get starting time
	//	The client talks to a server whose name it knows. 
	MsgClientPort clientport(servername,0.2);					// define client port, 0.2 sec timeout
	if (verbose) clientport.setverbose();							// more talkative
	for (;;)																		// forever
	{	testmsg msgin, msgout;
		//	Send message and wait for reply
		if (verbose) printf("Sending message.\n"); 
		int err = clientport.MsgSend(msgin, msgout);
		if (err < 0)															// if fail
		{	perror("Client MsgSend failed");						// trouble
			sleep(1);															// avoid loop while things get better
			continue;
		}
		if (verbose) printf("Got reply.\n");							// got a reply
		if (++msgcount >= maxmsgcount)						// if enough messages to print stats
		{	uint64_t endtime;												// new time
			ClockTime(CLOCK_REALTIME,0,&endtime);			// end time
			double elapsed = (endtime - starttime)*1.0e-9;	// elapsed time
			double msgssec = msgcount / elapsed;				// time per msg
			printf("Sent/received %d messages in %1.2f secs. %1.1f msgs/sec.\n",msgcount, elapsed, msgssec); fflush(stdout);
			msgcount = 0;													// reset for next time
			ClockTime(CLOCK_REALTIME,0,&starttime);		// get new start time 
		}
		////clientport.watchdogreset();									// tell the watchdog we are still alive
	}
}
//
//	Main program
//
int main(int argc, const char* argv[])
{	//	Parse input arguments
	for (int i=1; i<argc; i++)											// for all args
	{	const char* arg= argv[i];										// this arg
		if (arg[0] == '-')													// if flag argument
		{	switch(arg[1])	{												// interpret flags
			case 'v': verbose = true;	 break;						// set verbose mode
			case 's': server = true; break;							// server mode
			case 'w': slowserver = true; break;					// run server very slowly, for timeout test
			default: usage();												// bad call, fails
			}
			continue;															// next arg
		}
		//	Not flag, must be file arg
		usage();																// no filename arg
	}
	if (server)
	{	runserver();	}														// run dummy server
	else 
	{	runclient(); }															// run client
	return(0);																	// success
}
