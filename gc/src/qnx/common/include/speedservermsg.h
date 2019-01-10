////////////////////////////////////////////////////////////////////////////////
//
//    File:
//	speedservermsg.h
//
//    Usage:
//
//	In the watchdog startup file (startfile.txt):
//		ID=DIR speedserver
//
//    Description:
//	The Speed server takes care of managing the speed of the vehicle:
//	acceleration and deceleration.  Acceleration is accomplished by
//	exerting more pressure on the Throttle.  For deceleration, this may
//	be accomplished by easing off the throttle, or by braking, depending
//	on the desired rate of deceleration.
//
//
//    Messages received:
//
//	SpeedServerMsg::MsgSpeedSet
//
//    Messages sent:
//
//
//    Written By:
//
//      Achut Reddy
//      Team Overbot
//      January 2004
//
//	Revised by J. Nagle, October, 2004.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef SPEEDSERVERMSG_H
#define SPEEDSERVERMSG_H

#include <math.h>

#include "messaging.h"
#include "faultcodes.h"

#define SPEEDSERVER_ID	"SPEED"


//
// Speed Server Message Interface
//

//
// Messages Implemented by Speed Server
//

// Set vehicle speed
//
// If "speed" is _higher_ than the current speed, vehicle will
//	   accelerate at "rate" until its current speed matches "speed".
// If "speed" is _lower_ than the current speed, vehicle will
//	   decelerate at "rate" until its current speed matches "speed".
//	   For speed enough values of "rate", the deceleration can be
//	   accomplished merely by reducing the pressure on the accelerator.
//	   For higher values, the deceleration must be accomplished by
//	   braking.
// If "speed" is the _same_ as the current speed, the vehicle continues at the same speed.
//
//	A speed message must be sent every 10ms or timeout will occur.
//

// Maximum value for acceleration .  Setting acceleration to this
// means "
static const float MAX_ACCELERATION = MAXFLOAT;

struct MsgSpeedSet: public MsgBase
{
    static const uint32_t k_msgtype = char4('S','P','S','S');
    enum Gear { gear_unknown = 0, gear_reverse=1, gear_neutral=2, gear_low = 3, gear_high = 4 };	// gear number
	enum State { state_parked, state_reset, state_starting, state_startwait,
		state_paused, state_runwait, state_idle, state_shifting, state_run };
    float m_speed;        		// New speed (in meters/sec).
	float	m_curvature;		// 1/r, r = radius of curvature (in meters^-1) >0  means right turn
    // Always nonnegative.
    float m_acceleration; 		// Acceleration (or deceleration) in
    // meters/sec^2.  The rate at which the
    // vehicle should change its current
    // speed to match the new speed.
    // Always positive.
    Gear m_gear;					// desired gear
    State m_state;					// desired state
};

//
//	Reply from MsgSpeedSet request
//	Not derived from MsgBase, because we don't really need a header on a reply.
//	The client  knows what they're getting back.
//
struct MsgSpeedSetReply
{
	double m_odometer;					// actual odometer (m)
    float m_speed;							// actual speed (m/s)
	float	m_curvature;					// actual 1/r, r = radius of curvature (in meters^-1) >0  means right turn
    enum Hint { hint_none, hint_stuck, hint_slipping, hint_downshift, hint_toofast, hint_busy };
    MsgSpeedSet::Gear m_gear;		// actual gear
    MsgSpeedSet::State m_state;	// actual state
    Fault::Faultcode m_lastfault;		// last fault, if nonzero.	Cleared by requesting "parked"
    Hint m_hint;								// hint
};
//
//	MsgSpeedStop
//
//	Anybody can send this, which will force a recoverable emergency stop.
//	When in doubt, send this.
//
struct MsgSpeedStop: public MsgBase
{
    static const uint32_t k_msgtype = char4('S','P','S','T');
    Fault::Faultcode m_fault;			// the fault code
};

//
//  SpeedServerMsg  - all Speed Server messages as a union
//
//  Used as argument to MsgReceive. Size of union is size of largest
//  acceptable message.
//
union SpeedServerMsg {
    MsgSpeedSet	m_speedset;
    MsgSpeedStop	m_speedstop;
};

//
//	Diagnostic functions
//
//	We provide an overloaded ErrMsg function for each enumeration type.
//
//	ErrMsg for state
//
inline const char* ErrMsg(MsgSpeedSet::State stateid)
{	
	switch (stateid) {
	case MsgSpeedSet::state_parked:	return("parked");
	case MsgSpeedSet::state_reset:		return("reset");
	case MsgSpeedSet::state_starting:	return("starting");
	case MsgSpeedSet::state_startwait:	return("startwait");
	case MsgSpeedSet::state_shifting:	return("shifting");
	case MsgSpeedSet::state_run:			return("run");
	case MsgSpeedSet::state_idle:			return("idle");
	case MsgSpeedSet::state_paused:	return("paused");
	case MsgSpeedSet::state_runwait:	return("runwait");
	default: return("out of range state");
	};
}
//
//	ErrMsg for gear
//
inline const char* ErrMsg(MsgSpeedSet::Gear gearid)
{	switch (gearid) {
	case MsgSpeedSet::gear_unknown:	return("unknown gear");
	case MsgSpeedSet::gear_reverse:	return("reverse");
	case MsgSpeedSet::gear_neutral:		return("neutral");
	case MsgSpeedSet::gear_low:			return("low");
	case MsgSpeedSet::gear_high:			return("high");
	default: return("out of range gear");
	};
}
//
//	ErrMsg for hint
//
inline const char* ErrMsg(MsgSpeedSetReply::Hint hintid)
{	switch (hintid) {
	case MsgSpeedSetReply::hint_none:				return("");
	case MsgSpeedSetReply::hint_stuck:			return("Stuck");
	case MsgSpeedSetReply::hint_downshift:		return("Shift down");
	case MsgSpeedSetReply::hint_toofast:			return("Slow down");
	case MsgSpeedSetReply::hint_busy:				return("Busy");
	default: return("out of range hint");
	};
}

#endif // SPEEDSERVERMSG_H
