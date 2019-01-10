//
//	manualdrivingcontrolmode.cpp  --  manual driving via controllers
//
//	John Nagle
//	Team Overbot
//	January. 2004
//
#include "drivingcontrolmode.h"
#include "manualdrive.h"
#include "logprint.h"


//
//	Base class function
//
//	We must have at least one non-inline or the code won't be generated.
//
DrivingControlMode::	DrivingControlMode(Manualdrive& owner) 
	: m_owner(owner)
{}

//
//	ObserveControlMode  -- watch only, no control
//
//
//	getmodename  -- get name of this mode
//
const char* ObserveControlMode::getmodename() const
{	return("Observe mode (read only)");	}
//
bool ObserveControlMode::takecontrol()
{	return(true);	}																	// not that we actually do anything
//
//	releasecontrol  -- mode change coming
//
void ObserveControlMode::releasecontrol()
{
}
//
//	emergencystopactivated -- an emergency stop has occured
//
void ObserveControlMode::emergencystopactivated(const char* who, const char* why)
{
}
void ObserveControlMode::joystick_move(float x, float y)
{
}
bool ObserveControlMode::setrunmode(bool runmodeon)
{	if (runmodeon) return(false);											// can't actually request run mode this way
	return(true);
}
//
//	Constructor
//
ManualDrivingControlMode::ManualDrivingControlMode(Manualdrive& owner) 
	: DrivingControlMode(owner),
	m_hardwarewatchdog(&owner.m_brake, k_watchdog_interval, k_watchdog_priority)
	{}

//
//	ManualDrivingControlMode -- manual driving, directly to the actuators
//
bool ManualDrivingControlMode::takecontrol()
{	m_hardwarewatchdog.disable();											// stop watchdog
	getowner().m_brake.SetReadOnly(false);								// enable direct access to controllers
	getowner().m_throttle.SetReadOnly(false);
	getowner().m_transmission.SetReadOnly(false);
	getowner().m_steer.SetReadOnly(false);
	Controller::Err stat = getowner().m_steer.SetAuto(true);		// set auto so we control steering
	stat = stat;																			// ignore status, make compiler happy
	return(true);																			// take control
}
//
//	releasecontrol  -- mode change coming
//
//	Implies an emergency stop
//
void ManualDrivingControlMode::releasecontrol()
{	m_hardwarewatchdog.disable();											// stop watchdog
	Controller::Err stat = getowner().m_steer.SetAuto(false);	// clear auto
	stat = stat;																			// ignore status, make compiler happy
	getowner().m_brake.SetReadOnly(true);								// enable direct access to controllers
	getowner().m_throttle.SetReadOnly(true);
	getowner().m_transmission.SetReadOnly(true);
	getowner().m_steer.SetReadOnly(true);
}
//
//	emergencystopactivated -- an emergency stop has occured
//
//	Actually do the stop
//
void ManualDrivingControlMode::emergencystopactivated(const char* who, const char* why)
{	m_hardwarewatchdog.disable();											// disable hardware watchdog immediately
}
//
//	getmodename  -- get name of this mode
//
const char*  ManualDrivingControlMode::getmodename() const
{	return("Manual driving");	}

//	getrunmode  -- true if in run mode
//
bool ManualDrivingControlMode::getrunmode()
{	return(m_hardwarewatchdog.Looping());								// Run mode means watchdog activated
}
//
//	runbuttonallowed  -- true if allowed to press run button
//
//	All ontrollers must be happy.
//
bool ManualDrivingControlMode::runbuttonallowed()
{
	//	Check status of each controller
	bool steerready = getowner().m_steer.GetLocalAuto();
	bool transmissionready = getowner().m_transmission.GetLocalAuto();
	bool brakeready = getowner().m_brake.GetLocalAuto();
	bool throttleready = getowner().m_throttle.GetLocalAuto();
	return(steerready && transmissionready && brakeready && throttleready);
}
//
//	updatedrive  -- called periodically if caller is happy
//
void ManualDrivingControlMode::updatedrive()
{	if (getrunmode())
	{	bool steerready = getowner().m_steer.GetLocalAuto();
		if (!steerready) {	getowner().emergencystop("Steering controller not ready."); return; }
		bool transmissionready = getowner().m_transmission.GetLocalAuto();
		if (!transmissionready) {	getowner().emergencystop("Transmission controller not ready."); return; }
		bool brakeready = getowner().m_brake.GetLocalAuto();
		if (!brakeready) {	getowner().emergencystop("Brake controller not ready."); return; }
		bool throttleready = getowner().m_throttle.GetLocalAuto();
		if (!throttleready) {	getowner().emergencystop("Throttle controller not ready."); return; }
		//	Check transmission situation
		float actualgearnum = getowner().m_transmission.GetLocalActual();
		float goalgearnum = getowner().m_transmission.GetLocalGoal();
		MsgSpeedSet::Gear actualgear = MsgSpeedSet::Gear(actualgearnum);	// get gear number as enum
		MsgSpeedSet::Gear goalgear = MsgSpeedSet::Gear(goalgearnum);	// get gear number as enum
		if (actualgear != goalgear)											// if shift in progress
		{	getowner().emergencystop("Shifting"); return; }							// fails
		//	Good to go; reset watchdog.
		m_hardwarewatchdog.keepalive(); 								// reset watchdog
	}
}
//
//	joystick_move  -- joystick has moved
//
//	In this mode, we command the controllers directly.
//	Input range is 0..1 
//	0.5 is the neutral position.
//
void ManualDrivingControlMode::joystick_move(float x, float y)
{	//	Joystick has moved.  Update controllers.
	//	Update brake
	{	float brake = 0.0;
		if (y > 0.5) brake = (y - 0.5)*2;			// upper half of range is brake
		//	Issue commands to controllers.  GUI will pick it up from there.
		const float k_minbrakepress = getowner().m_brake.GetLocalGoalMin();
		const float k_maxbrakepress = getowner().m_brake.GetLocalGoalMax();
		float	 goal = k_minbrakepress + (k_maxbrakepress - k_minbrakepress) * brake;			// into controller range
		//	Actually issue the command
		Controller::Err stat = getowner().m_brake.SetGoal(goal);			// ISSUES MOTOR CONTROL COMMAND
		if (stat != EOK) 
		{	getowner().emergencystop("Unable to send braking command");
		}
	}
	//	Update throttle
	{	float throttle = 0.0;
		if (y < 0.5) throttle = (0.5-y)*2;			// lower half of range is throttle
		//	Issue commands to controllers.  GUI will pick it up from there.
		const float k_minthrottle = getowner().m_throttle.GetLocalGoalMin();
		const float k_maxthrottle = getowner().m_throttle.GetLocalGoalMax();
		float goal = k_minthrottle + (k_maxthrottle - k_minthrottle) * throttle;			// into controller range
		 ////logprintf("Throttle: y=%5.2f  throttle=%5.2f  goal = %5.2f\n",m_joyy, throttle, goal);	// ***TEMP***
		//	Actually issue the command
		Controller::Err stat = getowner().m_throttle.SetGoal(goal);			// ISSUES MOTOR CONTROL COMMAND
		if (stat != EOK) 
		{	getowner().emergencystop("Unable to send throttle command"); }
	}
	//	Update steering
	{
		{	Controller::Err stat = getowner().m_steer.SetAuto(true);		// set auto so we control steering
			stat = stat;																			// ignore status, make compiler happy
		}
		float goal = x;																		// range 0..1
		//	Issue commands to controllers.  GUI will pick it up from there.
		const float k_minsteerval = getowner().m_steer.GetLocalGoalMin();
		const float k_maxsteerval = getowner().m_steer.GetLocalGoalMax();
		goal = k_minsteerval + (k_maxsteerval - k_minsteerval) * goal;			// into controller range
		////goal *= 0.7;															// ***NEED PROPER CALIBRATION
		//	Actually issue the command
		Controller::Err stat = getowner().m_steer.SetGoal(goal);				// ISSUES MOTOR CONTROL COMMAND
		if (stat != EOK) 
		{	getowner().emergencystop("Unable to send steering command");
		}
	}
}
bool ManualDrivingControlMode::setrunmode(bool runmodeon)
{	if (runmodeon == getrunmode()) return(true);							// no change
	if (runmodeon)
	{	m_hardwarewatchdog.enable();	}										// start watchdog
	else
	{	m_hardwarewatchdog.disable(); }										// stop watchdog
	return(true);
}
//
//	steering_goal_meter_move  -- steering goal meter has been moved by user using the mouse
//
//	Input range is actual goal
//
void ManualDrivingControlMode::steering_goal_meter_move(float goal)
{	
	{	Controller::Err stat = getowner().m_steer.SetAuto(true);		// set auto so we control steering
		stat = stat;																			// ignore status, make compiler happy
	}
	Controller::Err stat = getowner().m_steer.SetGoal(goal);									// command the controller
	stat = stat;															// ignore status, make compiler happy
}
//
//	brake_pressure_goal_meter_move  -- brake goal meter has moved.
//
//	Input range is actual goal
//
void ManualDrivingControlMode::brake_pressure_goal_meter_move(float goal)
{
	Controller::Err stat = getowner().m_brake.SetGoal(goal);									// command the controller
	stat = stat;															// ignore status, make compiler happy
}
//
//	throttle_goal_meter_move  -- throttle goal meter has moved.
//
//	Input range is actual goal
//
void ManualDrivingControlMode::throttle_goal_meter_move(float goal)
{
	Controller::Err stat = getowner().m_throttle.SetGoal(goal);									// command the controller
	stat = stat;															// ignore status, make compiler happy
}
//
//	shift_button_pressed  -- manual shift request
//
void ManualDrivingControlMode::shift_button_pressed(MsgSpeedSet::Gear newgear)
{	getowner().emergencystop("Shift");							// starting a shift, stop everything
	//	***SHOULD CHECK FOR LOW RPM, BRAKES ON***
	float goal = float(newgear);										// convert gear number for controller
	Controller::Err stat = getowner().m_transmission.SetGoal(goal);	// set "gear number"
	if (stat != EOK) 
	{	getowner().emergencystop("Unable to send shift command");
		logprintf("Transmission controller error: %s\n",Controller::ErrMsg(stat));				// ***TEMP***
		return;
	}
}
//
//	
//	Hardware watchdog
//
HardwareWatchdog::HardwareWatchdog(BrakeController* controller, double interval, int priority)
: TimedLoop(interval, priority), m_ticks(0), m_controller(controller)
{
}
//
//	keepalive -- a good window update has occured
//
//	We reset the watchdog counter.  if it goes to zero, everything stops.
//
void HardwareWatchdog::keepalive()
{	
	if (Looping())													// if in run mode
	{	m_ticks = k_reset_ticks; }
}
//
//	code -- called on each timer tick
//
//	THIS CODE RUNS AT REAL TIME PRIORITY.
//
void HardwareWatchdog::code()
{	
	if (m_ticks == 0)												// if trouble
	{	Stop();															// stop this thread. GUI will notice
		logprintf("HardwareWatchdog::code -- timeout, emergency stop\n");
		return;
	}
	m_ticks--;															// count down ticks
	//	OK to reset watchdog
	if (!m_controller->Connected()) 							// not our job to make the connection
	{	return;
	}
	Controller::Err err = m_controller->ResetWatchdog();
	if (err != Controller::ERR_OK)								// if trouble
	{	return;
	}
}
//
//	enable -- start the watchdog
//
void HardwareWatchdog::enable()
{	
	if (Looping()) return;
	m_ticks = k_reset_ticks;
	Start();
}
//
//	disable -- stop the watchdog
//
void HardwareWatchdog::disable()
{	Stop();
}