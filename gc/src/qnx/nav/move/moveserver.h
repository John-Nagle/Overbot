////////////////////////////////////////////////////////////////////////////////
//
//    File: moveserver.h
//
//    Description:
//	Move Server main class
//
//    See also:
//        moveserver.h
//
//    Written By:
//       Achut Reddy
//       Team Overbot
//       December, 2003
//
/////////////////////////////////////////////////////////////////////////////////

#ifndef MOVESERVER_H
#define MOVESERVER_H

#include <stdio.h>
#include <pthread.h>

#include "messaging.h"
#include "mutexlock.h"
#include "gpsins_messaging.h"
#include "moveservermsg.h"
#include "speedservermsg.h"
#include "faultcodes.h"
//
//	class MoveServer  -- the entire move server
//
class MoveServer
{
private:
    // Message handling
    MsgServerPort	m_serverport;				// receiving port for incoming move commands
    MsgClientPort 	m_speedserverport;		// client port for messages to speed server
    MsgClientPort	m_gpsinsport;				// client port for messages to GPS/INS server
    bool		m_verbose;								// true if verbose
	MsgSpeedSetReply m_lastspeedreply;	// last reply from speed server
	GPSINSMsgRep		m_lastgpsinsreply;	// last GPS/INS message
	float	m_lastdistance;								// last distance to move
	Fault::Faultcode m_lastfault;					// last fault code
	bool	m_timedout;									// true if we have timed out
	uint64_t	m_lastlogtime;						// time of last log message
	float	m_avgvibration;
public:
    MoveServer();										// constructor
    ~MoveServer();									// destructor
	void messageThread(bool verbose);	// main thread, call to start
private:
    void handleMessage(int rcvid, const MoveServerMsg::MsgUnion& msg);
    void handleMove(int rcvid, const MoveServerMsg::MsgMove& msg);
	void handleStop(int rcvid, const MoveServerMsg::MsgMoveStop& msg);
	void handleQuery(int rcvid);
	void handleTimeout();
	int setFault(Fault::Faultcode faultid);		// set a fault situation
	float safeSpeedLimit() const;				// canned speed limit
	float safeSpeedForDistance(float dist, float pitchradians);			// apply stopping distance check
	float safeSpeedForCurvature(float curvature, float rollradians);	// apply skid load check
	float safeSpeedForRoughness(float lastspeed, float vibratoin);	// apply vibration check
	float getPitch();										// get current pitch info (radians)
	float getRoll();										// get current roll info (radians)
	float getVibration();								// total accel, all axes (m/s^2)
	bool getINSValid();								// true if INS data above is valid
	int sendSpeedMessage(const MoveServerMsg::MsgMove& msg, MsgSpeedSetReply& speedreply);
	int sendGPSINSMessage();					// get positional update from GPS
	void log();												// log if needed

};

#endif // MOVESERVER_H
