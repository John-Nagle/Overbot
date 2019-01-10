////////////////////////////////////////////////////////////////////////////////
//
//    File: speedserver.h
//
//    Description:
//	Speed Server main class
//
//    See also:
//
//    Written By:
//       Achut Reddy
//       Team Overbot
//       January, 2004
//
/////////////////////////////////////////////////////////////////////////////////

#ifndef SPEEDSERVER_H
#define SPEEDSERVER_H

#include <stdio.h>
#include <pthread.h>

#include "messaging.h"
#include "mutexlock.h"
#include "gpsins_messaging.h"
#include "simplecontroller.h"
#include "speedservermsg.h"
#include "chassis.h"
#include "algebra3.h"

//
//	Class SpeedServer
//
//	A single-threaded server. Except in dummy mode.
//
class SpeedServer {
	// Message handling
    MsgServerPort	m_serverport;		// receiving port for msgs
protected:
	bool m_verbose;							// verbose flag
public:
    SpeedServer();			// constructor
    virtual ~SpeedServer();			// destructor
	void messageThread(bool verbose);	// does the work
    
    // constants
    // XXX move to common vehicle specs file
    float topSpeedHighGear;
    float topSpeedLowGear;
    float topSpeedReverse;

private:
	void handleMessage(int rcvid, const SpeedServerMsg& msg);
	void handleSpeedSet(int rcvid, const MsgSpeedSet& msg);
	void handleSpeedStop(int rcvid, const MsgSpeedStop& msg);
	void handleTimeout();
	//	Subclassed functions		
	virtual void update() = 0;
	virtual bool handleSpeedChange(float accel, float speed, float steering) = 0;
	virtual bool handleStateChange(MsgSpeedSet::State newstate, bool& busy) = 0;
	virtual bool handleGearChange(MsgSpeedSet::Gear newgear, bool& busy) = 0;
	virtual void handleFault(Fault::Faultcode faultid) = 0;
	virtual void faultReported() = 0;
	virtual void buildMsgReply(MsgSpeedSetReply& replymsg) = 0;
	virtual MsgSpeedSet::State GetState() = 0;
	virtual 	MsgSpeedSet::Gear GetGear() = 0;
    ////void getCurrentSpeed();
};
//
//	Class SpeedServerChassis  -- the one that actually controls the vehicle
//
class SpeedServerChassis: public SpeedServer {
private:
    //	The chassis we are controlling
    Chassis m_chassis;						// appropriate chassis object
    // Message handling for things we call
    MsgClientPort 	*clientPort;	// client port for sending msgs 
    ////GPSINSMsgRep	gpsMsg;		// GPS/INS msg

protected:
	bool handleSpeedChange(float accel, float speed, float steering);
	bool handleStateChange(MsgSpeedSet::State newstate, bool& busy);
	bool handleGearChange(MsgSpeedSet::Gear newgear, bool& busy);
	void handleFault(Fault::Faultcode faultid);
	void faultReported();
	void update();
	void buildMsgReply(MsgSpeedSetReply& replymsg);
	MsgSpeedSet::State GetState();
	MsgSpeedSet::Gear GetGear();
};

//
//	Class SpeedServerDummy  -- dummy version used for off-line debugging
//
class SpeedServerDummy: public SpeedServer {
private:
	MsgSpeedSet::State m_dummystate;									// dummy state
	MsgSpeedSet::Gear m_dummygear;									// dummy gear
	Fault::Faultcode m_dummyfault;										// dummy fault code
	float m_dummyspeed;														// dummy speed
	double m_dummyodometer;												// dummy odometer
	float m_dummycurvature;													// dummy curvature
	int m_runwaitdelay;															// timer for RUNWAIT state
	double m_dummyx;															// location in XY space
	double m_dummyy;
	double m_dummyheading;												// heading (radians)
	double m_dummyllhbasepoint[3];										// base point (lat (rad) , long (rad), elev (m))
public:
	SpeedServerDummy();
	void startdummygpsinsserver();										// start the dummy GPS/INS server
private:
	//	Dummy GPS/INS server.  Reports simulated positions.
	void calc_lat_long(double llh[3]);
	void 	handle_request(int rcvid);
	void 	handle_basepoint(int rcvid, const GPSINSBasepoint& msg);
	void handle_getbasepoint(int rcvid, const GPSINSGetBasepoint& msg);
	void* server_thread();														// server thread
    static void* serverthreadstart(void* arg)							// passed to pthread_create
	{ return(reinterpret_cast<SpeedServerDummy*>(arg)->server_thread()); }
private:
	//	Dummy VORAD server
	MsgClientPort m_dummymapclientport;								// dummy map client port for sending VORAD data
	vector<vec2> m_dummyobstacles;									// dummy obstacles
	void simulatevorad();														// send VORAD hits for dummy obtacles
public:
	int loaddummyobstacles(const char* filename);				// load dummy obstacles from file
protected:
	bool handleSpeedChange(float accel, float speed, float steering);
	bool handleStateChange(MsgSpeedSet::State newstate, bool& busy);
	bool handleGearChange(MsgSpeedSet::Gear newgear, bool& busy);
	void handleFault(Fault::Faultcode faultid);
	void faultReported();
	void update();
	void buildMsgReply(MsgSpeedSetReply& replymsg);
	MsgSpeedSet::State GetState();
	MsgSpeedSet::Gear GetGear();
	void Dump();

};

#endif // SPEEDSERVER_H
