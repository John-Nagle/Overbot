#ifndef LOGGER_H
#define LOGGER_H
//
//	logger.h  --  logger for GPS/INS data
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
#include "messaging.h"
#include "gpsins_messaging.h"
//

//	Class Logger -- logging class
//
class Logger {
private:
	MsgClientPort	m_gpsinsport;						// port for talking to GPS/INS
	GPSINSMsgRep m_lastfix;								// last fix received
	FILE*	 m_waypointlog;									// waypoint log
	FILE* m_gpslog;											// GPS log
	int m_waypointnumber;								// waypoint number
	const char* m_logdir;									// logging directory
public:
	Logger();
	virtual ~Logger();
	void client(const char* logdir, float interval, bool acceptbad);	// our client
private:
	int writewaypoint(FILE* waypt, const GPSINSMsgRep& gpsmsg);
	int writegpslog(FILE* gpslog, const GPSINSMsgRep& gpsmsg);
	void handlemsg(const GPSINSMsgRep& gpsmsg, bool acceptbad);
};
#endif // LOGGER_H