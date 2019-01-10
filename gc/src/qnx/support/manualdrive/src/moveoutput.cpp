//
//	moveoutput.cpp  --  manual driving via the move server
//
//	John Nagle
//	Team Overbot
//	January. 2004
//
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include "drivingcontrolmode.h"
#include "manualdrive.h"
#include "logprint.h"
#include "moveservermsg.h"
#include "remotemessaging.h"
#include "tuneable.h"
#include "proto.h"													// so we can raise dialog boxes
#include "ablibs.h"
#include "abimport.h"

//
//	Constants
//
const char* k_move_server_node = "gcrear0"	;		// node for server	
const char* k_move_server_name = "TESTMOVE";	// watchdog ID of dummy access to move server
const double k_move_server_timeout = 0.5;				// if doesn't respond, complain
//
//	Vehicle constants
//	
////const float k_turning_radius = 3.81;							// from Polaris Ranger 6x6 specs
const float k_max_speed = 4.5;								// max speed for manual mode, approx 10MPH
const float k_braking_decel = (0.25*9.8);					// 1/4G, in m/sec.
//	These must match the values in the move server for good driving.
const Tuneable k_max_steering_angle_deg("STEERINGANGLELIM",25,60,33.0,"Maximum steering angle, degrees");
const float k_max_steering_angle = k_max_steering_angle_deg*(M_PI/180.0);		// convert to radians
const Tuneable k_wheelbase("WHEELBASE",1,3,2.29,"Effective wheelbase, m");	// per Polaris manual
//
//	
//	calcSteeringRadius  -- convert steering goal to 1/r
//
//	From move server.
//
//	Inverse of above. Input in range -1..1
//
static float calcSteeringRadius(float goal)
{
	if (isnan(goal)) return(goal);															// handle invalid data
	float steerangle = goal * k_max_steering_angle;							// steering angle, radians
	return(sin(steerangle)/k_wheelbase);											// calc 1/radius
}
//
//	Constructor
//
SemiAutoModes::SemiAutoModes(Manualdrive& owner, const char* nodename, const char* servername) 
	: DrivingControlMode(owner),
	m_runmode(false),
	m_moveservernode(nodename),
	m_moveservername(servername),
	m_moveserverport(m_moveservernode, m_moveservername,k_move_server_timeout),
	m_watchdog(*this,k_watchdog_interval, k_watchdog_priority)
{	clearlastmove();
	m_lastmove.m_gear = MsgSpeedSet::gear_neutral;	
}

//
//	SemiAutoModes -- manual driving, directly to the actuators
//
bool SemiAutoModes::takecontrol()
{	ost::MutexLock lok(m_lock);							// lock
	setrunmode(false);																// not in run mode
	m_coasting = false;																// not coasting
	m_requestedgear = MsgSpeedSet::gear_neutral;					// start in neutral
	clearlastmove();
	m_lastmove.m_gear = MsgSpeedSet::gear_neutral;	
	getowner().getmoveinfodisplay().enable(true);					// enable
	return(true);																			// take control
}
//
//	releasecontrol  -- mode change coming
//
//	Implies an emergency stop
//
void SemiAutoModes::releasecontrol()
{	ost::MutexLock lok(m_lock);													// lock
	setrunmode(false);																// not in run mode
	emergencystopactivated("Mode change","");						// send a final stop message
	m_moveserverport.ConnectDetach();									// break connection to move servers
	getowner().getmoveinfodisplay().enable(false);					// disable
}
//
//	clearlastmove  -- clear last move data
//
//	Implies we want to be stopped.
//
void SemiAutoModes::clearlastmove()
{
	m_lastmove.m_msgtype = MoveServerMsg::MsgMove::k_msgtype;	// build command
	m_lastmove.m_distance = 0;
	m_lastmove.m_speed = 0;
	m_lastmove.m_curvature = 0;
	////m_lastmove.m_gear = MsgSpeedSet::gear_neutral;	
}
//
//	emergencystopactivated -- an emergency stop has occured
//
//	Actually do the stop
//
void SemiAutoModes::emergencystopactivated(const char* who, const char* why)
{	ost::MutexLock lok(m_lock);													// lock
	logprintf("Emergency stop activated: %s  %s\n", who, why);	// ***TEMP***
	if (!m_runmode) return;															// only in run mode
	setrunmode(false);																// not in run mode
	clearlastmove();																	// clear move in progress if any
	//	Build and send emergency stop message
	MoveServerMsg::MsgMoveStop stopmsg;
	MoveServerMsg::MsgMoveReply replymsg;
	stopmsg.m_msgtype = MoveServerMsg::MsgMoveStop::k_msgtype;	// set type
	stopmsg.m_fault = Fault::manualkill;										// reason for stop
	int stat = m_moveserverport.MsgSend(stopmsg, replymsg);// send the message
	if (stat < 0)																			// if trouble
	{	perror("Unable to send emergency stop command");		// not good, but move server will time out
		return;
	}
}
//
//	getrunmode  -- true if in run mode
//
bool SemiAutoModes::getrunmode()
{	
	return(m_runmode);																		// ***TEMP***
}
//
//	updatedrive  -- called periodically if caller is happy
//
void SemiAutoModes::updatedrive()
{
	ost::MutexLock lok(m_lock);																// lock
	m_watchdog.keepalive();																	// keep the watchdog alive
	updatemovedisplay(m_lastmove, m_lastreply);								// update display
}
//
//	resendlastmove  --  resend the last move to keep things going
//
//	Called from non-Photon task.  Must not make Photon calls without locking.
//
void SemiAutoModes::resendlastmove()
{	int stat = 0;																								// status of comm
	MoveServerMsg::MsgMoveReply replymsg;
	{	ost::MutexLock lok(m_lock);																	// lock
		if (!m_runmode) return;																			// not running, no command
		if (m_coasting) 																						// coasting mode, don't issue new command
		{
			//	***MORE***	(But how do we get logging data?) 
			return;
		}
		//	Resend last message to keep things going.
		int stat = m_moveserverport.MsgSend(m_lastmove, replymsg);		// send the message
		if (stat >= 0)																						// if good
		{	m_lastreply = replymsg;																// save last reply for display
		}
	}																											// end locked section
	//	May now lock Photon, having unlocked other lock.
	if (stat < 0)																							// if trouble
	{	PtEnter(0);																						// enter Photon block
		getowner().emergencystop("Lost connection to move server");
		PtLeave(0);																						// leave Photon block
		return;
	}
	if (replymsg.m_speedreply.m_lastfault != Fault::none)							// if fault problem
	{	PtEnter(0);																						// enter Photon block
		if (m_runmode)																					// if still in run mode
		{	getowner().emergencystop(Fault::ErrMsg(replymsg.m_speedreply.m_lastfault));	// only once, or we loop
		}
		PtLeave(0);																						// leave Photon block
		return;
	}
}
//
//	joystick_move  -- joystick/pedals have moved
//
//	Generates plausible move commands based on requested pedal positions
//	Input range is 0..1 
//	0.5 is the neutral position.
//
//	General concept:
//
//		"Throttle" mode:
//			Calculate desired move distance from pedal position.
//			If distance is greater than distance remaining to be covered in this move,
//			send another move command.
//			Otherwise, enter "coast" mode, and let current move command run to completion.
//
//		"Brake" mode:
//			Full brake: send zero length move and E-stop command.
//			Partial brake: cut distance to go by brake fraction, and send move.
//
void SemiAutoModes::joystick_move(float x, float y)
{	//	Joystick has moved.  Update controllers.
	ost::MutexLock lok(m_lock);							// lock
	MoveServerMsg::MsgMove movemsg;
	movemsg.m_msgtype = MoveServerMsg::MsgMove::k_msgtype;	// set type
	//	Compute steering.  "curvature" is 1/turning radius.
	float steer = (x-0.5)*2;									// steering in range -1..1
	movemsg.m_curvature = calcSteeringRadius(steer);	// calc 1/turning radius
	movemsg.m_distance = 0;							// assume null move
	movemsg.m_speed = 0;
	movemsg.m_gear = m_requestedgear;		// gear change
	if (!m_runmode)											// if not in run mode
	{	return;	}													// do nothing
	//	Compute speed
	if (y > 0.5)													// brake pedal pushed
	{	m_coasting = false;									// not coasting
		coastingLog(false);									// coasting did not run to completion
		float brake =  (y - 0.5)*2;							// upper half of range is brake
		//	Brake is now in range 0..1
		if (brake > 0.80)										// if big brake
		{	getowner().emergencystop("Brake pedal pushed hard.");	// trip emergency stop
			return;													
		}
		if (brake < 0.05) return;							// ignore tiny brake, let move run out.
		//	Normal braking. Cut down last move. 
		float speed = (0.5 - brake)*k_max_speed;	// zero speed if more than half brake
		speed = std::min(speed, m_lastmove.m_speed); // never more than last speed during braking
		if (speed < 0) speed = 0;							// never negative
		movemsg.m_speed = speed;					// set max speed
		//	Calc move distance based on speed. But always less than last time, so you can't accel while braking.
		movemsg.m_distance = std::min(calcStoppingDistance(speed),m_lastmove.m_distance);	// calculate allowed move distance
	} else {														// accel pedal pushed
		float domove = (0.5-y)*2;							// lower half of range is "go"
		float speed = domove * k_max_speed;	// calc desired speed
		movemsg.m_speed = speed;
		movemsg.m_distance = calcStoppingDistance(speed);					// calculate allowed move distance
#ifdef NOTYET // no coasting support yet
		if (m_coasting) 											// if coasting out the last move
		{	if ((movemsg.m_distance < m_lastmove.m_distance)					// and let up on the accelerator
			&& (movemsg.m_speed < m_lastmove.m_speed))
			{	return;	}																					// continue coasting
		}
		m_coasting = true;																			// begin coasting
		coastingLog(true);																			// begin coasting log
#endif // NOTYET
	}
	MoveServerMsg::MsgMoveReply replymsg;
	int stat = m_moveserverport.MsgSend(movemsg, replymsg);				// send the message
	if (stat < 0)																							// if trouble
	{	getowner().emergencystop("Unable to send move message");		// not good, but move server will time out
		return;
	}
	if (replymsg.m_speedreply.m_lastfault != Fault::none)							// if fault problem
	{	if (m_runmode)																					// if in run mode
		{	getowner().emergencystop(Fault::ErrMsg(replymsg.m_speedreply.m_lastfault));			// report
			return;
		}
	}
	////coastingLog(movemsg, replymsg);													// log coasting event
	m_lastmove = movemsg;																		// save move for next time
	m_lastreply = replymsg;																		// save last reply for display
	updatemovedisplay(movemsg, replymsg);											// update display
}
//
//	setrunmode -- change to run mode
//
bool SemiAutoModes::setrunmode(bool runmodeon)
{	
	if (!runmodeon)																			// if turning off
	{	m_runmode = false;	}																// stop immediately, even if locking problem
	ost::MutexLock lok(m_lock);															// lock
	if (runmodeon == getrunmode()) return(true);								// no change
	clearlastmove();																			// clear old info
	m_coasting = false;																		// not coasting
	m_runmode = runmodeon;															// set run mode
	if (m_runmode)																				// enable or disable watchdog
	{	m_watchdog.enable(); }
	else 
	{	m_watchdog.disable(); }
	return(true);
}
//
//	Estimator of stopping distance
//
//	This is just for talking to the move server, not a serious calculation.
//
//	v = a*t;
//	d = 0.5*a*t*t
//	so
//	t = v/a
//	t*t = 2*d/a
//	v*v/(a*a) = 2*d/a
//	v*v/a = 2*d
//	d = 0.5*(v*v)/a
//
float SemiAutoModes::calcStoppingDistance(float speed)
{	if (speed < 0) return(0);
	return(0.5* (speed*speed)/	k_braking_decel);
}
//
//	shift_button_pressed  -- manual shift request
//
//	Request low, high, neutral, or reverse.
//
void SemiAutoModes::shift_button_pressed(MsgSpeedSet::Gear newgear)
{	
	ost::MutexLock lok(m_lock);							// lock
	if (m_requestedgear == newgear) return;
	m_requestedgear = newgear;
}
//
//	updatemovedisplay  --  Process a reply from the move server.
//
//	Updates the display.
//
void SemiAutoModes::updatemovedisplay(const MoveServerMsg::MsgMove& msg, const MoveServerMsg::MsgMoveReply& reply)
{
	MoveInfoDisplay& display = getowner().getmoveinfodisplay();	// object to update
	display.setstate(ErrMsg(reply.m_speedreply.m_state));				// update state on screen
	display.sethint(ErrMsg(reply.m_speedreply.m_hint));					// update hint
	display.setmovedistance(msg.m_distance);									// update distance
}
//
//	coastingLog  -- support for logging coast-down events
//
//	This is used to calibrate the move server
//
void SemiAutoModes::coastingLog(bool coasting)
{
}
//
//	getmynodename  -- get name of this node
//
static const char* getmynodename()
{
	static char buf[511];													// my node name
	int stat = netmgr_ndtostr(ND2S_LOCAL_STR | ND2S_QOS_HIDE, 0, buf, sizeof(buf));	// node 0 is always oneself
	if (stat < 0) return("QNX NATIVE NETWORKING NOT RUNNING");				// native networking not running
	return(buf);																// return node name, short form
}
//
//	Submodes
//
SemiAutoControlMode::SemiAutoControlMode(Manualdrive& owner) 
	: SemiAutoModes(owner, (getenv("TESTMOVENODE") ? getenv("TESTMOVENODE") : k_move_server_node), k_move_server_name)
{
}
//
//	getmodename  -- get name of this mode
//
const char*  SemiAutoControlMode::getmodename() const
{	return("Manual driving via move server");	}						// at least for now

//	Offline mode - always look on local node for node
OfflineSemiAutoControlMode::OfflineSemiAutoControlMode(Manualdrive& owner) 
	: SemiAutoModes(owner, getmynodename(), k_move_server_name)
{
}
//
//	getmodename  -- get name of this mode
//
const char*  OfflineSemiAutoControlMode::getmodename() const
{	return("OFFLINE TEST of manual driving via move server");	}						// at least for now
//	
//	Software watchdog  -- resends last move to keep move server going
//
SoftwareWatchdog::SoftwareWatchdog(SemiAutoModes& owner, double interval, int priority)
: TimedLoop(interval, priority), m_ticks(0), m_owner(owner)
{
}
//
//	keepalive -- a good window update has occured
//
//	We reset the watchdog counter.  if it goes to zero, everything stops.
//
void SoftwareWatchdog::keepalive()
{	
	if (Looping())													// if in run mode
	{	m_ticks = k_reset_ticks; }
}
//
//	code -- called on each timer tick
//
//	THIS CODE RUNS AT REAL TIME PRIORITY.
//
void SoftwareWatchdog::code()
{	
	if (m_ticks == 0)												// if trouble
	{	Stop();															// stop this thread. GUI will notice
		logprintf("HardwareWatchdog::code -- timeout, emergency stop\n");
		return;
	}
	m_ticks--;															// count down ticks
	//	OK to reset watchdog
	m_owner.resendlastmove();								// do it											
}
//
//	enable -- start the watchdog
//
void SoftwareWatchdog::enable()
{	
	if (Looping()) return;
	m_ticks = k_reset_ticks;
	Start();
}
//
//	disable -- stop the watchdog
//
void SoftwareWatchdog::disable()
{	Stop();
}
