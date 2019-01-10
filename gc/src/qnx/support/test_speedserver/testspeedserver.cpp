//
//	Speed server testing server
//
//	Allows remote nodes to send speed commands to server, for manual
//	driving.
//
//	TEST USE ONLY
//
//	John Nagle
//	Team Overbot
//	November, 2004
//
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <errno.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <logprint.h>
#include "messaging.h"
#include "remotemessaging.h"
#include "speedservermsg.h"
#include "logprint.h"
//
//	gettimenow  -- Get time in floating point, seconds
//
//	Move elsewhere.
//
inline double gettimenow()
{	struct timespec tm;
	clock_gettime(CLOCK_REALTIME,&tm);								// get time now
	double now = timespec2nsec(&tm);									// convert to ns
	return (now * (1.0e-9));													// nanoseconds to seconds (double)
}
//
//	runcannedtest -- connection established, run the test
//
//	Just something to get us going.
//
//
int runtest(RemoteMsgClientPort& port)
{	MsgSpeedSet request;
	//	Create a simple request
	request.m_msgtype = MsgSpeedSet::k_msgtype;
	request.m_state = MsgSpeedSet::state_idle;
	request.m_gear = MsgSpeedSet::gear_low;
	request.m_speed = 0.0;
	request.m_acceleration = 0.0;
	request.m_curvature = 0.0;												// straight ahead
	MsgSpeedSetReply reply;													// the reply
	while (1)
	{	
		int stat = port.MsgSend(request, reply);	
		if (stat < 0)																		// messaging error
		{	if (errno == EBUSY)														// try again if busy
			{	usleep(50000);
				continue;
			}
			perror("MsgSend failed");
			sleep(2);																		// wait for client, try again
			continue;
		}
		if ((reply.m_state == request.m_state)
		&& (reply.m_gear == request.m_gear)) break;					// idle and in gear
		logprintf("In %s and %s, waiting for idle state and low gear.\n",ErrMsg(reply.m_state), ErrMsg(reply.m_gear));
		sleep(1);																			// not yet, keep trying
	}
	//	Now in idle and low gear. Change to run, then set a speed.
	request.m_state = MsgSpeedSet::state_run;
	logprintf("In idle state, requesting change to run state.\n");
	while (1)
	{	double before = gettimenow();											// time now
		int stat = port.MsgSend(request, reply);	
		if (stat < 0)																		// messaging error
		{	if (errno == EBUSY)														// try again if busy
			{	usleep(50000);
				continue;
			}
			perror("MsgSend failed in idle state.");
			return(-1);																		// fails
		}
		double after = gettimenow();
		double elapsed = after - before;
		if (elapsed > 0.110)															// if too slow
		{	logprintf("MsgSend to test speed server too slow: took %6.4f seconds.\n",elapsed);	}
		if (reply.m_state == request.m_state) break;					// success
		/////logprintf("In %s state, waiting  for run state.\n", ErrMsg(reply.m_state));
		usleep(50000);																	// not yet, keep trying
	}
	logprintf("In run mode. Starting movement. Kill program to stop.\n");
	//	In run mode.
	request.m_speed = 2.0;														// meters per second
	while (1)
	{	double before = gettimenow();											// time now
		int stat = port.MsgSend(request, reply);	
		if (stat < 0)																		// messaging error
		{	perror("MsgSend failed");
			return(-1);																		// fails
		}
		double after = gettimenow();
		double elapsed = after - before;
		if (elapsed > 0.110)															// if too slow
		{	logprintf("MsgSend to test speed server too slow: took %6.4f seconds.\n",elapsed);	}
		if (reply.m_state != request.m_state)
		{	logprintf("Dropped RUN mode.\nTest FAILED.\n");
			return(1);
		}
		////usleep(100000);														// wait 100ms and send again
		usleep(50000);																// wait 100ms and send again
	}	
	logprintf("Test complete.\n");
	return(0);																			// success			
}
//
//	testclient  -- our test client
//
void testclient(const char* node, const char* server)
{	RemoteMsgClientPort port(node,server);
    runtest(port);																	// run the canned test for now
}
//
//	usage -- usual usage 
//
static void usage()
{	printf("Usage: test_speedserver [-v] [NODE] SERVER\n");				
	exit(1);
}
//
//	main program
//
int main(int argc, const char* argv[])
{	bool verbose = false;														// verbose flag
	const char* arg1 = 0;														// target node
	const char* arg2 = 0;														// target server
	for (int i=1; i<argc; i++)													// parse args
	{	const char* arg = argv[i];											// this arg
		if (arg[0] == '-')
		{	switch(arg[1]) {
			case 'v': 	verbose = true; break;
			default:	usage();
			}
		} else {
			if (!arg1) {	arg1 = arg; continue; };
			if (!arg2) {	arg2 = arg; continue; }
			usage();
		}		
	}
	if (!arg1) usage();																// 1 or 2 args
	if (arg2)
	{	testclient(arg1, arg2);		}											// run the test remotely
	else
	{	testclient(0,arg1);		}													// local
	return(0);																			// done
}