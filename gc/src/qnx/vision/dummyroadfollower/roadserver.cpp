//
//	roadserver.cpp  --  dummy road follower
//
//	John Nagle
//	Team Overbot
//	November, 2003
//
#include <assert.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include "messaging.h"
#include "roadserver.h"
#include "logprint.h"
//
const double k_cycleperiod = 30*5;									// road wanders every 30 seconds
const float k_maxcurvature = 1.0 / 14;							// max curvature from follower
//
//	createDummyRoad -- simulate a "dummy road" that wanders aimlessly
//
static void	 createDummyRoad(RoadServerMsgReply& replymsg)
{
	usleep(200000);														// only one frame every 200ms
	static unsigned int cycle = 0;									// cycle number
	double cyclefract = fmod((M_PI*2)* cycle / k_cycleperiod, (M_PI*2));	// cycle at 2PI
	double weave = sin(cyclefract);								// cycles through -1..1, smoothly
	weave *= k_maxcurvature*1.5;
	if (fabs(weave) > k_maxcurvature)
	{	replymsg.m_confidence = -1; }
	else
	{	replymsg.m_confidence = 1; }
	replymsg.m_curvature = weave;								// desired steering angle
	cycle++;
}
//
//	runserver -- run as a server
//
//	This server accepts request messages, does some computation, and returns the result.
//
void runserver()															
{	MsgServerPort serverport(1.0);
	int stat = serverport.ChannelCreate();						// attach
	if (stat < 0)
	{	perror("Unable to create server channel.\n"); exit(1); }
	for (;;)																		// forever
	{	logprintf("\n");														// periodically log a blank line to keep the watchdog happy
		RoadServerMsg msgin;											// area for incoming msg
		_msg_info msginfo;												// aux info
		int rcvid = serverport.MsgReceive(msgin,&msginfo);	// get msg
		if (rcvid < 0)															// if error
		{	if (errno == ETIMEDOUT)									// if timeout
			{	
				continue;
			}
			fflush(stdout); perror("MsgReceive failed in server");				// fails
			sleep(1);															// avoid tight loop if repeated trouble
			continue;															// fails
		}
		if (rcvid == 0)														// pulse
		{	logprintf("Server received a pulse.\n");					// pulses don't require a reply
			continue;
		}
		//	We have received a message
		switch (msgin.m_rdpc.m_msgtype) {					// fan out on type
		case RoadServerMsgRDPC::k_msgtype:				// request for picture
		{	MsgError(rcvid, EBADRPC);								// no pictures in dummy version
			break;
		}

		case RoadServerMsgRDDR::k_msgtype:				// request for road direction
		{	RoadServerMsgReply replymsg;
			createDummyRoad(replymsg);							// simulate a "dummy road" that wanders aimlessly
			MsgReply(rcvid, replymsg);
			break;
		}
			
		default:																	// unknown, fails
			MsgError(rcvid,EBADRPC);									// reply with result code only
			break;
		}
	}
}
//
//	usage  -- print usage and exit
//
static void usage()
{	printf("Usage: dummyroadserver [options]\n");
	exit(1);																		// fails
}
//
//	Main program
//
//	Usage: roadserver [options] 
//
int main(int argc, const char* argv[])
{	//	Parse input arguments
	for (int i=1; i<argc; i++)											// for all args
	{	const char* arg= argv[i];										// this arg
		if (arg[0] == '-')													// if flag argument
		{	switch(arg[1])	{												// interpret flags
			default: usage();												// bad call, fails
			}
			continue;															// next arg
		}
		//	Not flag, must be file arg
		usage();
	}
	runserver();				// run road follower server
	return(0);																	// success
}
