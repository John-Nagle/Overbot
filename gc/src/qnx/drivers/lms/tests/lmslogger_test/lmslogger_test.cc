//
//	LMS logging test.
//	J.	Nagle
//	December, 2004
//
//	Receives 
//
//	Usage: watchdogtest [options]
//
//	Options:	
//		-d	directory															// log files go into this directory
//		-n NNN																	// maximum number of records to save
//		-v 																		// verbose
//	
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>
#include "math.h"
#include "mutexlock.h"
#include "messaging.h"
#include "lidarserver.h"
#include "logfile.h"
#include "logprint.h"
#include "controller.h"
#include "tuneable.h"
//
//	Constants
//
const char k_lmsserverid[] = "LMS";								// watchdog ID of server
//
const float k_tiltrateslow = (M_PI/180)*15;					// for imaging
const float k_tiltratefast = (M_PI/180)*90;						// for moving
const float k_tiltinitialpos = (M_PI/180)*40;					// 40 degrees forward from straight down.
const float k_tilthighpos = (M_PI/180)*100;					// 100 degrees from straight down
const float k_tiltposincr = (M_PI/180)*0.05;					// increment unit

Tuneable k_goal_dist_m("LIDARGOALDIST", 4, 30, 5, "LIDAR pointing goal distance (m)");
const uint16_t k_goal_dist = int(k_goal_dist_m*100);	// goal distance in cm.
const uint16_t k_deadband = 25;									// 25cm deadband

const int k_initialscanrecs = 500;									// first 500 scans are the initial tilt sequence 
//
//	Statics
//
static bool verbose = false;											// verbose mode
static ost::BoundedBuffer<float,10>	tiltcommands;		// tilt commands
static float gtiltpos = 0;													// last tilt position
//
//	printsizeinfo  -- sizes of items
//
//	We need this info to read the data file from Python.
//
static void printsizeinfo()
{
	//	Check that size is consistent with structure definition
	LidarScanLine scanline;												// area for incoming msg
	//	Size info
	size_t sfileitemsize = sizeof(scanline);				// total size
	size_t sheadersize = sizeof(scanline.m_header);	// header size
	size_t sdataitemsize = sizeof(scanline.m_range[0]);	// size of one data item
	size_t sdatasize = sdataitemsize * LMS_MAX_DATA_POINTS; // size of data
	int offset0 = (char*)(&scanline.m_header.m_timestamp) - (char*)&scanline;				// offset of first field
	int offsetn = (char*)(&scanline.m_header.m_valueCount) - (char*)&scanline;				// offset of last field
	int offsetdata = (char*)(&scanline.m_range[0]) - (char*)&scanline;								// offset of first data item
	logprintf("File record size as plain struct %d bytes (%d header, %d range data (%d data items of %d bytes).\n",
		sfileitemsize, sheadersize, sdatasize, LMS_MAX_DATA_POINTS,sdataitemsize);
	logprintf("Offsets: m_timestamp = %d, m_valueCount = %d, m_range = %d\n",
		offset0, offsetn, offsetdata);																// check offsets for filler
}
//
//	usage -- print usage message and exit
//
static void usage()
{	logprintf("Usage: lmslogger_test [-v] [-n maxrecs] [-t] directory] \n");	// print message
	printsizeinfo();
	exit(1);
}
//
//	runserver -- run as a server
//
//	This server accepts request messages, does some computation, and returns the result.
//
static void runserver(const char* logfile, int maxrecs)															
{
	//	The server normally has the name given the program by the watchdog file. This is in the env. variable "ID"
	bool trackground = false;											// track ground if set
	MsgServerPort serverport(0.0);								// define message port, no timeout
	if (verbose) serverport.setverbose();						// more talkative
	int stat = serverport.ChannelCreate();						// create a channel, tell watchdog about it
	if (stat) 
	{	perror("ChannelCreate failed in server"); 
		serverport.Dump();
		abort();																	// fails
	}
	//
	//	Open the log file
	//
	FILE* fd = fopen(logfile,"w");										// open for writing
	{	if (!fd)
		{	logprintf("Unable to create file \"%s\" for writing.\n", logfile);
			exit(1);
		}
	}
	int reccnt = 0;															// record count written
	//
	//	Run the server
	//
	for (;;)																		// forever
	{	LidarServerMsgLISN msgin;									// area for incoming msg
		int rcvid = serverport.MsgReceive(msgin);			// get msg
		if (rcvid < 0)															// if error
		{	fflush(stdout); perror("MsgReceive failed in server");				// fails
			sleep(1);															// avoid tight loop if repeated trouble
			continue;															// fails
		}
		if (rcvid == 0)														// pulse
		{	logprintf("Server received a pulse.\n");					// pulses don't require a reply
			continue;
		}
		//	We have received a message
		switch (msgin.m_msgtype) {								// fan out on message type
		case LidarServerMsgLISN::k_msgtype:					// the one we want
		{	//	Save range from center pixel of scan.
			if (trackground) 												// if done with initial capture
			{	//	Try to maintain tilt head with a center pixel at specified distance.
				uint16_t samplerange = msgin.m_data.m_range[msgin.m_data.m_header.m_valueCount / 2];	// get middle pixel of scan
				float tiltpos =  gtiltpos;									// most recent actual pos
				if (samplerange > k_goal_dist)						// if looking too far out
				{	tiltpos -= k_tiltposincr;	}							// look closer
				else if (samplerange < (k_goal_dist - k_deadband))
				{	tiltpos += k_tiltposincr;	}							// look further out
				tiltpos = std::min(std::max(tiltpos, k_tiltinitialpos), k_tilthighpos);	// keep in range
				tiltcommands.put(tiltpos);								// go to indicated pos
			}
			//	Logging to file
			if (!finite(msgin.m_data.m_header.m_tilt)) break;				// tilt not up yet
			int cnt = fwrite(&msgin.m_data,sizeof(msgin.m_data),1,fd);	// write one record to file
			if (cnt < 0)
			{	perror("Error writing log file.");	exit(1);	}		// fails
			}
			if (reccnt == 0)													// if first record written
			{	tiltcommands.put((M_PI/2)*1.2);						// point forward and up
			}
			if (reccnt >= k_initialscanrecs)							// if done with initial scan
			{	if (!trackground)
				{	trackground = true;									// set track mode
					logprintf("Beginning ground tracking.\n");
				}
			}
			if (++reccnt >= maxrecs)								// if wrote enough
			{	fclose(fd); fd = 0;
				logprintf("Log file complete. %d records written.\n", reccnt);
				tiltcommands.put(0.0);									// back to straight down
			}
			break;

		default:
			logprintf("Unexpected message type received: 0x%08x.\n",msgin.m_msgtype);
			break;
		}
		int err = MsgError(rcvid, EOK);								// Reply with empty
		if (err) perror("MsgReply failed in server");
		////serverport.watchdogreset();								// tell the watchdog we are still alive
	}
}
//
//	runtilttest -- run the tilt head test
//
//	A thread.
//
static void* runtilttest(void* arg)
{
	MsgClientPort tiltport(k_lmsserverid,5.0);					// for talking to client
	//	Run actual tilt test
	LidarServerMsgTILT tiltmsg, tiltreply;							// tilt message
	tiltmsg.m_msgtype = LidarServerMsgTILT::k_msgtype;	// set header
	logprintf("Running tilt test\n");		
	float lasttilt = -9999999;											// bogus last value
	//	Reset to straight down
	tiltcommands.put(k_tiltinitialpos);								// send to self
	for (;;)																		// get tilt commands from queue and execute them.
	{	
		tiltcommands.get(tiltmsg.m_tilt);								// get next tilt command
		if (tiltmsg.m_tilt == lasttilt) continue;						// unchanged
		tiltmsg.m_tiltrate = k_tiltrateslow;							// do at slow rate
		int stat = tiltport.MsgSend(tiltmsg, tiltreply);			// ask for a tilt
		if (stat < 0)															// send problem
		{	logprintf("Can't send tilt command: %s.\n",strerror(errno));
			lasttilt = 99999999;											// so we will retry
			sleep(3);
			continue;
		}
		if (tiltreply.m_err != Controller::ERR_OK)				// if controller not ready
		{	logprintf("Tilt controller problem: %s.\n",Controller::ErrMsg(tiltreply.m_err));
			lasttilt = 99999999;											// so we will retry
			sleep(3);
			continue;
		}
		if (tiltreply.m_washing)											// if wash cycle in progress
		{	logprintf("Washing, wait.\n");
			lasttilt = 99999999;											// so we will retry
			sleep(3);
			continue;
		}
		////logprintf("Tilt command to %f executed.\n", tiltmsg.m_tilt);		// success
		lasttilt = tiltmsg.m_tilt;											// save last command completed
		gtiltpos = lasttilt;													// save actual last tilt position
	}
	return(0);
}
//
//	Main program
//
int main(int argc, const char* argv[])
{	//	Parse input arguments
	const char* dirarg = 0;												// directory for log files
	bool tilttest = false;
	int maxrecs = 100;													// default records to copy
	for (int i=1; i<argc; i++)											// for all args
	{	const char* arg= argv[i];										// this arg
		if (arg[0] == '-')													// if flag argument
		{	switch(arg[1])	{												// interpret flags
			case 'v': verbose = true;	 break;						// set verbose mode
			case 't': tilttest = true;	break;							// requesting tilt test
			case 'n': 															// need count
			{	i++;																// advance one arg
				if (i<argc) 													// if additional arg
				{	maxrecs = atoi(argv[i]);							// get record count
					break;
				} else usage();												// otherwise fail
			}
			default: usage();												// bad call, fails
			}
			continue;															// next arg
		} 
		//	Not flag, must be file arg
		if (dirarg) usage();												// two dir args
		dirarg = arg;															// save filename arg
	}
	if (!dirarg) usage();													// no filename arg
	//	Construct output file name
	char fname[511];														// max length
	const char* k_prefix = "lidarscans";
	const char* k_suffix = "log";
	buildlogfilename(fname, sizeof(fname), dirarg,  k_prefix, k_suffix);
	logprintf("Logging to \"%s\"\n",fname); fflush(stdout);	// log
	printsizeinfo();
	if (tilttest) pthread_create(0,0,runtilttest,0);				// run tilt test if requested.
	runserver(fname,maxrecs);										// run the server
	return(0);																	// success
}
