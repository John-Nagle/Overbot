	//
//  speedserverdummy.cc  --  speed server interface to dummy test version
//
//	This isn't even a "simulator".  It's just enough to allow testing the move server.
//
//	John Nagle
//	Team Overbot
//	November, 2004
#include <stdlib.h>
#include <unistd.h>

#include "logprint.h"
#include "speedserver.h"
#include "tuneable.h"
//
//	Constants
//
const double k_speed_change_rate = 0.05;									// slow speed adjustment
const double k_max_accel = 4.9;													// half-G deceleration limit
const int k_runwait_ticks = 10*5;													// 5 second delay for RUN mode
//
//	Dummy understeer -- used to simulate understeering, to exercise correction for it
//
const Tuneable k_dummy_understeer("DUMMYUNDERSTEER", -0.3, 0.3, 0, "Simulated understeer (ratio)");	// 
const Tuneable k_dummy_heading("DUMMYHEADING", 0, 360, 0, "Simulated initial heading (degrees)");	// 
const Tuneable k_dummy_x("DUMMYX", -1000000, 1000000, 0, "Simulated initial position rel to 1st waypoint (m)");	// 
const Tuneable k_dummy_y("DUMMYY", -1000000, 1000000, 0, "Simulated initial position rel to 1st waypoint (m)");	// 
//
//	NOTE -- this steering angle info is also in the move/speed servers and must match.
const Tuneable k_max_steering_angle_deg("STEERINGANGLELIM",25,60,33.0,"Maximum steering angle, degrees");
const float k_max_steering_angle = k_max_steering_angle_deg*(M_PI/180.0);		// convert to radians
const Tuneable k_wheelbase("WHEELBASE",1,3,1.7,"Effective wheelbase, m");	// measured to point between rear axles
const float k_invturnradius = sin(k_max_steering_angle) / k_wheelbase;				// max curvature
//
const Tuneable k_lock_to_lock_time("LOCKTOLOCKTIME", 3.0, 7.0, 4.0, "Steering time, lock to lock (secs)");
const float k_max_curvature_rate = (2.0*k_invturnradius) / k_lock_to_lock_time;		// max speed at which curvature can change
//
//	constructor
//
SpeedServerDummy::SpeedServerDummy()
:	m_dummystate(MsgSpeedSet::state_parked),
	m_dummygear(MsgSpeedSet::gear_unknown),
	m_dummyfault(Fault::none),
	m_dummyspeed(0),
	m_dummyodometer(0),
	m_dummycurvature(0),
	m_runwaitdelay(0),
	m_dummyx(0),
	m_dummyy(0),
	m_dummyheading(0),
	m_dummymapclientport("MAP",0.2)
{
	m_dummyllhbasepoint[0] = m_dummyllhbasepoint[1] = m_dummyllhbasepoint[2] = 0;	// initial dummy GPS/INS zero
	m_dummyheading = (M_PI / 180) * k_dummy_heading;		// set initial dummy heading
	m_dummyx = k_dummy_x;
	m_dummyy = k_dummy_y;
}
//
//	handleStateChange  -- request for a state change
//
//	Returns true if success
//
bool SpeedServerDummy::handleStateChange(MsgSpeedSet::State newstate, bool& busy)
{	busy = false;
	if ((m_dummyspeed > 0) && (newstate == MsgSpeedSet::state_run)) // can't change to run until stop completed
	{	logprintf("ERROR: Tried to change state while vehicle is moving.\n");	// can't do that
		newstate = MsgSpeedSet::state_idle;										// force down to iidle
		busy = true;
		return(true);
	}
	if (newstate == MsgSpeedSet::state_run)										// if requesting RUN
	{	newstate = MsgSpeedSet::state_runwait;									// go to RUNWAIT instead
		busy = true;																				// still waiting
		if (m_dummystate != MsgSpeedSet::state_runwait)					// if change
		{	m_runwaitdelay = k_runwait_ticks;	}									// set delay timer
	}
	if (m_dummystate == newstate) return(true);								// no change
	logprintf("State change: %s to %s\n", ErrMsg(m_dummystate),ErrMsg(newstate)); 
	m_dummystate = newstate;															// just change state right now
	return(true);																					// success
}
//
//	handleSpeedChange  -- request for a speed change
//	
//	Adjusts speed.  Acceleration will slowly approach the goal speed, regardless of accel.
//	Deceleration follows the specified accel, up to a limit.
//
bool SpeedServerDummy::handleSpeedChange(float accel, float speed, float steering)
{	if (speed < 0) return(false);															// validate args
	if (accel < 0) return(false);
	const double dt = 0.1;																	// 100ms per step
	if (m_dummystate != MsgSpeedSet::state_run) return(true);			// ignore request
	double diff = speed - m_dummyspeed;										// desired change
	if (diff > 0)																					// if accelerating
	{	m_dummyspeed += diff*k_speed_change_rate;	}					// adjust, slowly
	else																								// if decelerating
	{	if (accel > k_max_accel) accel = k_max_accel;							// limit accel
		if (accel <= 0) accel = k_speed_change_rate;							// slow accel if not specified
		double ds = accel *  dt;															// change in speed
		if (ds > m_dummyspeed) ds = m_dummyspeed;						// no negative speeds
		m_dummyspeed -= ds;																// new, slower speed
	}
	//	New steering calculation.  Understands finite steering change rate and limits
	const float maxsteeringchange = k_max_curvature_rate*dt;		// max steering change per unit time
	float steeringchange = steering - m_dummycurvature;				// desired steering change
	steeringchange = std::min(std::max(-maxsteeringchange, steeringchange), maxsteeringchange);	// bound change per cycle
	float newsteering = m_dummycurvature + steeringchange;		// new steering, bounded
	m_dummycurvature = std::min(std::max(-k_invturnradius, newsteering), k_invturnradius);	// bound curvature
	return(true);																					// success
}
//
//	handleGearChange  -- request for a gear change
//
//	Returns true if request acceptable. 
//
//	Must be in correct state before shifting.
//
bool SpeedServerDummy::handleGearChange(MsgSpeedSet::Gear newgear, bool& busy)
{	busy = false;
	if (newgear == GetGear()) return(true);										// nothing to do
	switch (GetState()) {																	// what to do based on state
	case MsgSpeedSet::state_parked:												// OK to shift
	case MsgSpeedSet::state_idle:														// OK to shift
	case MsgSpeedSet::state_run:														// OK to shift
		if (m_dummyspeed > 0)															// if not stopped
		{	busy = true;
			return(true);
		}
		break;																						// OK
		
	case MsgSpeedSet::state_shifting:												// shift in progress
		break;																						// OK to shift
		
	case MsgSpeedSet::state_paused:												// paused
		if (newgear != MsgSpeedSet::gear_neutral)								// only allowed to shift to neutral
		{	logprintf("ERROR: Can't shift into gear while paused.\n");					
			return(false);																		// caller error
		}
		break;															
	
	case MsgSpeedSet::state_runwait:												// runwait
		busy = true;																				// not yet
		return(true);																				// you can ask, but nothing happens yet.
		break;
		
	default:
		logprintf("ERROR: Can't shift in this state.\n");
		return(false);																			// caller error
	}
	m_dummygear = newgear;															// change gear
	return(true);																					// success
}

//
//	handleFault  --  handle a fault
//
void SpeedServerDummy::handleFault(Fault::Faultcode faultid)
{	m_dummyfault = faultid;																// note fault
	logprintf("FAULT set: %s\n",Fault::ErrMsg(faultid));							// debug print
	MsgSpeedSet::State newstate = MsgSpeedSet::state_parked;		// go to parked state by default
	if (m_dummystate >=  MsgSpeedSet::state_idle)							// if state above idle
	{	newstate = MsgSpeedSet::state_idle;	}									// only drop to idle if above idle
	bool busy;
	handleStateChange(newstate, busy);											// force a state change
}
//
//	faultReported -- the fault has been reported.
//	
//	We will reset it if the vehicle has stopped.
//
void SpeedServerDummy::faultReported()
{	if (m_dummyfault == Fault::none) return;										// nothing to do
	if ((m_dummystate <= MsgSpeedSet::state_idle)							// if stopped
	&& (m_dummyspeed < 0.01))														// and speed is zero
	{	m_dummyfault = Fault::none;													// clear the fault
		logprintf("Fault cleared.\n");														// clear the fault
	} else {																						// uncleared fault
		logprintf("Pending fault uncleared: %s\n", Fault::ErrMsg(m_dummyfault));	// we still have an uncleared fault.
	}
}
//
//	Update  -- update the chassis object
//
//	Should happen every 100ms, never more than 120ms between updates.
//
void SpeedServerDummy::update()
{	
	if (m_dummystate != MsgSpeedSet::state_run)								// zero speed if not in run state
	{	
		m_dummyspeed -= k_max_accel*0.1;										// decel at max rate
		if (m_dummyspeed < 0.01)														// if very small or negative
		{	m_dummyspeed = 0;	}														// make zero
	}
	double dir = 0;																				// direction of travel
	switch (m_dummygear) {
	case MsgSpeedSet:: gear_reverse: 	dir = -1; break;						// going backwards
	case MsgSpeedSet:: gear_low:
	case MsgSpeedSet:: gear_high:	dir = 1; break;							// going forwards
	default: break;
	}
	//	Update position
	double dt = 0.1;																			// assume 100ms per call
	double movedist = m_dummyspeed*dt*dir;									// distance moved
	m_dummyodometer += movedist;												// update odometer
	float dirx = sin(m_dummyheading);												// get unit vector in heading dir
	float diry = cos(m_dummyheading);												// heading is zero when north
	m_dummyx += dirx	* movedist;													// update position
	m_dummyy += diry * movedist;
	//	Update heading based on curvature and move distance
	double oldheading = m_dummyheading;				// ***TEMP***
	double curv = m_dummycurvature*(1.0 - k_dummy_understeer);	// simulate understeer/oversteer		
	m_dummyheading = fmod(curv*movedist + m_dummyheading, M_PI*2);		// update heading
	logprintf("Heading update: %4.3f -> %4.3f  curv=%4.3f dist = %4.3f\n",
		oldheading, m_dummyheading, curv, movedist);	// ***TEMP***
	//	Handle runwait state
	if ((m_dummystate == MsgSpeedSet::state_runwait)
	& (m_runwaitdelay-- < 0))															// and timer ran out
	{	logprintf("Run wait complete.\n");		
		logprintf("State change: %s to %s\n", ErrMsg(m_dummystate),ErrMsg(MsgSpeedSet::state_run)); 
		m_dummystate = MsgSpeedSet::state_run;								// go to run state			
	}
	simulatevorad();																			// simulate VORAD targets
	bool verbose = true;
	if (verbose)																					// if verbose mode
	{	static int msgcnt = 0;																// dump every tenth time
		msgcnt++;
		if (msgcnt > 9)
		{	Dump();	msgcnt = 0; }
	}

}
//
//	buildMsgReply -- fill in reply data about what chassis is doing
//
void
SpeedServerDummy::buildMsgReply(MsgSpeedSetReply& replymsg)
{
	//	Build reply data from dummy data
	replymsg.m_odometer = m_dummyodometer;
	replymsg.m_speed = m_dummyspeed;
	replymsg.m_curvature = m_dummycurvature;
	replymsg.m_state = m_dummystate;
	replymsg.m_gear = m_dummygear;
	replymsg.m_lastfault = m_dummyfault;
	replymsg.m_hint  = MsgSpeedSetReply::hint_none;						// no hints yet
}
//
//	GetState --  access to chassis state
//
MsgSpeedSet::State SpeedServerDummy::GetState()
{	return(m_dummystate); }
//
//	GetGear --  access to chassis state
//
MsgSpeedSet::Gear SpeedServerDummy::GetGear()
{	return(m_dummygear); }
//
//	Dump  --  print info
//	
//	In verbose mode, we get a crude log this way.
//
void SpeedServerDummy::Dump()
{	
	const char* gearnames[5] = {"unknown gear","reverse","neutral","low","high"};
	const char* gearmsg = gearnames[m_dummygear];								// name of current gear
	const double headingdeg = m_dummyheading*(180/M_PI);						// heading in degrees
	logprintf("> %s (%s) speed %6.2f  odom: %8.2f  1/r: %6.2f  hdg=%1.0f  (%1.2f,%1.2f)\n",
		ErrMsg(m_dummystate), gearmsg, m_dummyspeed, m_dummyodometer, m_dummycurvature,
			headingdeg, m_dummyx, m_dummyy);
}

