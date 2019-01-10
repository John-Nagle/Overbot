//
//	logger.cc  --  logger for GPS/INS data
//
//	Creates log files for later evaluation.
//
//	Client of the move server.
//	Runs under the watchdog
//
//	Team Overbot
//	J.	Nagle
//	January, 2005
//
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <math.h>
#include "messaging.h"
#include "logfile.h"
#include "logprint.h"
#include "gpsins_messaging.h"
#include "logger.h"
//
//
const float k_min_move = 1.0;						// must move 2 meters to get a new waypoint
const float k_meters_per_mile = 1609.344;
const float k_seconds_per_hour = 3600;
const float k_halfwidth = 5;							// halfwidth of path between waypoints to place in output.
const char* k_server = "GPSINS";
//
//	Class Logger -- logging class
//
//
//	Constructor
//
Logger::Logger()
: 	m_gpsinsport(k_server),
	m_waypointlog(NULL),
	m_gpslog(NULL),
	m_waypointnumber(0),
	m_logdir("/tmp")											// safe, but never used
{	bzero(&m_lastfix, sizeof(m_lastfix));				// clear original fix
}
//
//	Destructor
//
Logger::~Logger()
{	if (m_waypointlog) fclose(m_waypointlog);						// final close
	if (m_gpslog) fclose(m_gpslog);
}
//
//	writegpslog  ---  write one GPS message in binary format
//
int Logger::writegpslog(FILE* gpslog, const GPSINSMsgRep& gpsmsg)
{	int stat = fwrite(&gpsmsg,sizeof(gpsmsg),1,gpslog);					// write to log
	fflush(gpslog);																				// flush every time
	return(stat);
}
//
//	writewaypoint  -- write one GPS message in waypoint file format
//
int Logger::writewaypoint(FILE* waypt, const GPSINSMsgRep& gpsmsg)
{	float vx = gpsmsg.vel[0];
	float vy = gpsmsg.vel[1];
	float v = sqrt(vx*vx+vy*vy);						// travel speed, meters per second
	float mph = v * (k_seconds_per_hour / k_meters_per_mile);		// convert to mph and log
	int num = ++m_waypointnumber;				// waypoint number
	logprintf("#%d: Lat: %12.8f Long: %12.8f\n", num,gpsmsg.llh[0],gpsmsg.llh[1]);	// ***TEMP***
	int stat = fprintf(waypt, "%d,%12.8f, %12.8f, %6.1f, %6.1f\n",num,gpsmsg.llh[0],gpsmsg.llh[1], k_halfwidth,mph);
	fflush(waypt);												// so we get a good file after killing the program
	return(stat);													// return status
}
//
//	handlemsg   --  handle an incoming GPS message
//
void Logger::handlemsg(const GPSINSMsgRep& gpsmsg, bool acceptbad)
{	//	Must have valid solution from GPS/INS
#ifdef OBSOLETE
	if (gpsmsg.posStat != GPSINS_MSG::SOL_COMPUTED && gpsmsg.posStat != GPSINS_MSG::INSUFFICIENT_OBS)

	{	logprintf("GPS status (%s) invalid, fix ignored.\n", decodePosStat(gpsmsg.posStat));
		if (m_gpslog)
		{	fclose(m_gpslog);
			m_gpslog = NULL;
			logprintf("Lost GPS, ending GPS log file\n");
		}
		if (m_waypointlog)													// if we were logging
		{	logprintf("Lost GPS, ending waypoint log  file.\n");					// end log file on GPS loss
			fclose(m_waypointlog);
			m_waypointlog = NULL;
		}
		sleep(2);																	// don't frantically repeat after comm loss
		return;
	}
#endif // OBSOLETE
	float dx = gpsmsg.pos[0] - m_lastfix.pos[0];
	float dy = gpsmsg.pos[1] - m_lastfix.pos[1];
	float distmoved = sqrt(dx*dx+dy*dy);							// distance since last output
	if (distmoved < k_min_move) 										// haven't moved enough yet
	{
		return;																		// ignore
	}
	//	Do output log. This has all GPS entries, good or bad.
	if (!m_gpslog)
	{	char filename[511];													// create a file name
		buildlogfilename(filename, sizeof(filename), m_logdir,  "lgps", "gpslog");
		m_gpslog = fopen(filename,"w");								// create new log file
	}
	if (m_gpslog)																	// if have an output file
	{	writegpslog(m_gpslog, gpsmsg);								// log the waypoint
	}
	m_lastfix = gpsmsg;														// save for next time
	//	Do waypoint log. Omnistar HP entries only, so waypoints are always good.
	if ((!acceptbad) && gpsmsg.posType != GPSINS_MSG::OMNISTAR_HP) 							// not enough precision
	{	////logprintf("No Omnistar, fix ignored.\n");					// ***TEMP***
		return;
	}	
	if (!m_waypointlog)														// if no output file
	{	char filename[511];													// create a file name
		buildlogfilename(filename, sizeof(filename), m_logdir,  "waypoints", "txt");
		m_waypointlog = fopen(filename,"w");						// create new log file
		m_waypointnumber = 0;											// start waypoint numbering 
	}
	if (m_waypointlog)																	// if have an output file
	{	writewaypoint(m_waypointlog, gpsmsg);							// log the waypoint
	}
}
//
//	client -- the client task
//
//	Runs forever.
//
void Logger::client(const char* logdir, float interval, bool acceptbad)
{
	if (interval <= 0 || interval > 100) interval = 1.0;					// reasonable poll interval
	bool first = true;
	m_logdir = logdir;																	// set logging dir
	for (;;)
	{
		GPSINSMsgReq gpsreq;														// request
		gpsreq.m_msgtype = GPSINSMsgReq::k_msgtype;			// request type
		GPSINSMsgRep gpsmsg;													// reply
		int stat = m_gpsinsport.MsgSend(gpsreq, gpsmsg);			// request a GPS fix
		if (stat < 0)
		{	logprintf("Unable to reach GPS server: %s\n",strerror(errno));	// fails
			sleep(2);																		// try again
			continue;
		}
		//	Have GPS fix.
		if (first)
		{	m_lastfix = gpsmsg;	first = false; }								// first time through
		handlemsg(gpsmsg, acceptbad);										// handle this message
		usleep(int(interval*1000000.0));										// wait appropriate period
	}
}
