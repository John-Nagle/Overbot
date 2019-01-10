//
//	 File: chassis.cpp  --  interface to the Overbot Polaris Ranger with Galil controllers
//
//
//	John Nagle
//	Team Overbot
//	November, 2004
//
#include "logprint.h"
#include "tuneable.h"
#include "timeutil.h"
#include "chassis.h"
//
//	Hard constants
//
float k_connect_timeout = 0.050;										// 50ms to connect UDP
//
//	Tuneable calibration constants.
//
//	These can be overridden via environment parameters if necessary.
//
//	Fields: Name, min, max, default, description.
//	Put "NAME=nnn.nn"  in the environment to override.
//
//	Controller tuning
//
const Tuneable k_brake_p_gain("BRAKEPGAIN", 0, 1000, 1000, "Brake P gain");	// ***TEMP*** value
const Tuneable k_max_slipback_speed("SLIPBACKSPEEDLIM", 0, 2.0, 0.6, "Slipback speed limit, m/sec");	// ***TEMP*** value
const Tuneable k_max_speed_overshoot("OVERSHOOTSPEEDLIM", 0, 3.0, 1.0,"Speed overshoot limit, m/s");
const Tuneable k_lock_brakes_speed("LOCKBRAKESSPEED", 0, 4.0, 3.0,"Below this speed and stopping, lock brakes, m/s");
const Tuneable k_min_idle_rpm("RPMIDLEMIN",-1, 2500, 250, "Idle speed minimum, RPM");	// engine running if > this
const Tuneable k_max_idle_rpm("RPMIDLELIM", 1000, 2500, 1750, "Idle speed limit, RPM");	// engine above idle if > this
const Tuneable k_operating_rpm("RPMOPERATING", 3000, 6000, 5250, "Normal operating RPM");	// clutch definitely engaged if > this
const Tuneable k_max_steering_angle_deg("STEERINGANGLELIM",25,60,33.0,"Maximum steering angle, degrees");
const Tuneable k_hill_hold_brake_fraction("HILLHOLDBRAKE", 0.0 , 1.0 , 0.5, "Hill hold braking level (fract)");
const Tuneable k_hill_hold_rpm_release("HILLHOLDRPM", 1750, 5250, 2500,"Hill hold brake release point (RPM)");


const float k_max_steering_angle = k_max_steering_angle_deg*(M_PI/180.0);		// convert to radians
//	Simple Kalman filters on controller inputs. 
const Tuneable k_speed_filter_constant("SPEEDFILTERCONST", 0.05, 1, 0.5,"Speed filter constant 1=no filter");
const Tuneable k_accel_filter_constant("ACCELFILTERCONST", 0.05, 1, 0.5,"Acceleration filter constant 1=no filter");
//
//	Steering model
//
//	From experimental results:
//		goalfraction = P1*CURV+P2*CURV*CURV;		(then apply sign)
//
//	The default values here were measured on April 14, 2005.
//
const Tuneable k_steerp1("STEERP1",5, 10, 8.17,"Curvature to steering, linear term");	// per Polaris manual
const Tuneable k_steerp2("STEERP2",- 30, 30, -13.82 ,"Curvature to steering, quadratic term");	// per Polaris manual

//	Debug support
const Tuneable k_dump_interval("DEBUGINTERVAL", 0, 60, 1.0, "Debug dump interval, seconds.");	
const float k_max_valid_throttle = 0.95;											// saturated throttle setting
const float k_stationary_speed = 0.02;												// below this means stopped														

//
//	Speed controller tuning
//
//	PID controller.
//
//		Control law:
//
//		ctl = k_throttle_gain * (err + k_throttle_i_term*integratederr + k_throttle_d_gain*differr)
//
//
//	Model: delay followed by a first order lag.
//	This is simplistic, but roughly realistic.
//
//	Zeigler-Nichols method.
//
//	Initial guesses: 
//		Assume a step change in throttle comand from 0 to 0.5
// 		deltaU = 0.5
//		Td (time before any movement) = 2s
//		Ts (time to near-steady-state speed) = 4s
//		deltaY (change to steady state speed in low gear) = 10m/s
//		Steady state speed for half throttle = 5m/sec
//
//	Per Zeigler-Nichols method:
//
//		k_throttle_gain = 1.2*Ts*deltaU / (Td*deltaY)		
//		k_throttle_i_term = 1.0/(2.0*Td)
//		k_throttle_d_term = 0.5*Td
//
//
//			P = 1.2*4*0.5 / (2 * 10) = 2.4/20 = 0.12
//			I = 0.25
//			D = 1
//
//	Things to worry about:
//
//		Oscillation when creeping at low speed may be a problem.
//		Slipback when starting up on a steep hill is detected, but causes a fault.
//		We may need to keep brakes applied lightly until engine RPM exceeds some value.
//
//		Field testing on 6 FEB 2005 showed these values to be tolerable, if not optimal.
//
const Tuneable k_throttle_gain("THROTTLEGAIN", 0, 20, 0.12	, "Throttle overall gain");			// ***TEMP*** value
const Tuneable k_throttle_i_term("THROTTLEITERM", 0, 10, 0.25, "Throttle I term");				// ***TEMP*** value
const Tuneable k_throttle_d_term("THROTTLEDTERM", 0, 10, 1.0, "Throttle D term");			// ***TEMP*** value

const Tuneable k_max_throttle_rate("THROTTLERATE", 0.01, 2.0, 0.25, "Throttle max change rate, fract/sec");	// ***TEMP*** value
//
//	State machine support
//
class ChassisState {
public:
	const MsgSpeedSet::State m_state;															// state being described
	const char* m_name;																				// name of state
	const bool m_enablewatchdog;																// watchdog enable on? (also enables horn)
	const bool m_enginerun;																			// engine run relay on?
	const bool m_crank;																					// cranking engine relay on?
	const float m_timelimit;																				// max time allowed in this state
	const MsgSpeedSet::State m_timeoutstate;												// state to transition to if timeout
public:
	//	Constructor for a table row
	ChassisState(MsgSpeedSet::State thisstate, const char* name,
		bool enablewatchdog, bool enginerun, bool crank,
		float timelimit = 0.0,
		MsgSpeedSet::State timeoutstate = MsgSpeedSet::state_parked)
		: m_state(thisstate), m_name(name), m_enablewatchdog(enablewatchdog), m_enginerun(enginerun), m_crank(crank), 
		m_timelimit(timelimit), m_timeoutstate(timeoutstate)
		{}
	void Update();																							// take appropriate update action
};
//
//	State timeouts
//
const float k_reset_timeout = 5.0;																// time to reset controllers
const float k_cranking_timeout = 10.0;														// crank for this long, then wait.
const float k_recrank_timeout = 20.0;														// wait for this long for next start attempt
const float k_paused_timeout = 300.0;														// after this long in pause, stop engine
const float k_shift_timeout = 30;																// if can't shift in this time, restart
const float k_runwait_timeout = 5.0;															// DARPA-specified wait
//
//	The table of states.  Must be in order, and this is checked.
//
//	Order is
//		    enum State { state_parked, state_reset, state_starting, state_startwait,
//   	state_paused, state_runwait, state_idle, state_shifting, state_run };
//
static const ChassisState k_state_table[] = 
	//						State															watch	eng		crank	
	{	ChassisState(MsgSpeedSet::state_parked, 	"parked",	false, 	false, 	false),
		ChassisState(MsgSpeedSet::state_reset,		"reset",		false, 	false, 	false, 	k_reset_timeout, MsgSpeedSet::state_parked),
		ChassisState(MsgSpeedSet::state_starting,	"starting",	false, 	true, 	true,		k_cranking_timeout, MsgSpeedSet::state_startwait),
		ChassisState(MsgSpeedSet::state_startwait, "startwait", false, 	false, 	false,	k_recrank_timeout, MsgSpeedSet::state_starting),
		ChassisState(MsgSpeedSet::state_paused,	"paused", false,	true, 	false, 	k_paused_timeout, MsgSpeedSet::state_parked),
		ChassisState(MsgSpeedSet::state_runwait,	"runwait", true	,	true, 	false,	k_runwait_timeout, MsgSpeedSet::state_idle),
		ChassisState(MsgSpeedSet::state_idle, 		"idle", 		false,	true, 	false),
		ChassisState(MsgSpeedSet::state_shifting, 	"shifting",	true,		true, 	false,	k_shift_timeout, MsgSpeedSet::state_reset),
		ChassisState(MsgSpeedSet::state_run,			"run",		true,		true, 	false)
	};
//
//	Actual chassis control
//
//	Constructor
//
Chassis::Chassis(bool readonly)
:	m_brake("gcbrake",readonly),
	m_throttle("gcthrottle",readonly),
	m_transmission("gctransmission",readonly),
	m_steer("gcsteer",readonly),
	m_state(MsgSpeedSet::state_parked),
	m_speed(k_speed_filter_constant),
	m_accel(k_accel_filter_constant),
	m_prevodometer(0),
	m_prevspeed(0),
	m_accumspeederror(0),
	m_statetimestamp(0),
	m_updatetimestamp(0),
	m_dumptimestamp(0),
	m_hillholder(false),
	m_controllersready(false),
	m_fault(Fault::none)
{
	const bool useudp = true;											// Use UDP. not TCP
	m_transmission.DatagramMode(useudp);
	m_brake.DatagramMode(useudp);
	m_throttle.DatagramMode(useudp);
	m_steer.DatagramMode(useudp);

}
//
//	Destructor
//
Chassis::~Chassis()
{
}
//
//	UpdateControllersTry  -- update local copy of controller state
//
//	Retryable, to get around transient problems.
//
Controller::Err Chassis::UpdateControllersTry(Fault::Faultcode& faultid)
{
 	Controller::Err err = Controller::ERR_OK;
	//	Reestablish connections to controllers if necessary.
	//	This is UDP, so "connect" doesn't mean much.
	//	Controller must respond to ARP
	m_throttle.Connect(k_connect_timeout);
	m_brake.Connect(k_connect_timeout);
	m_transmission.Connect(k_connect_timeout);
	m_steer.Connect(k_connect_timeout);
	err = m_throttle.UpdateState();																// update throttle
	if (err != Controller::ERR_OK)																	// if problem
	{	faultid = Fault::throttle;																		// which controller
		return(err);																						// fails
	}
	err = m_brake.UpdateState();																// update brake
	if (err != Controller::ERR_OK)																	// if problem
	{	faultid = Fault::brake;
		return(err);
	}
	err = m_transmission.UpdateState();													// update transmission
	if (err != Controller::ERR_OK)																	// if problem
	{	faultid = Fault::trans;
		return(err);
	}
	err = m_steer.UpdateState();																// update steering
	if (err != Controller::ERR_OK)																	// if problem
	{	if (m_steer.GetLocalAuto() == 0)														// if in manual
		{	faultid = Fault::manualmode;															// we are in manual mode
		} else {
			faultid = Fault::steer;																		// otherwise steering trouble
		}
		return(err);
	}
	return(Controller::ERR_OK);																	// no trouble found
}
//
//	UpdateControllers  -- update local copy of controller state
//
//	Retryable, to get around transient problems.
//
Controller::Err Chassis::UpdateControllers(Fault::Faultcode& faultid)
{
 	const int k_controller_startup_retries = 5;												// try 5 times if not in run mode
	const int k_controller_run_retries = 1;													// try once in run mode.
	int retries = (GetState() == MsgSpeedSet::state_run) ? k_controller_run_retries : k_controller_startup_retries;
	Controller::Err err = Controller::ERR_OK;													// never used
	for (int i=0; i < retries; i++)																	// retry some number of times
	{	err = UpdateControllersTry(faultid);													// try to bring up controllers
		if (err == Controller::ERR_OK) return(err);											// success
		logprintf("Controller fault: %s %s -- attempting retry.\n", Fault::ErrMsg(faultid), Controller::ErrMsg(err));
	}
	//	Failed
	assert(err != Controller::ERR_OK);															// cannot be OK, we ran out of retries
	return(err);																							// fails
}
//
//	Update -- update by getting info from the controllers
//
//	Any trouble will cause an emergency stop.
//
void Chassis::Update()
{	Fault::Faultcode  faultid;																		// controller fault, if any
	Controller::Err err = UpdateControllers(faultid);										// try to update controllers
	if (err != Controller::ERR_OK)																	// if problem
	{	m_controllersready = false;																// fault
		SetFault(faultid, err);																			// trouble - will cause E-stop
		return;
	}
	m_controllersready = true;																	// we are talking to the controllers
	//	Speed calculation, obtained by differerencing the odometer values.
	double now = gettimenow();																// get time now
	double dt = now - m_updatetimestamp;												// get change in time
	////logprintf("Update interval: %4.3fs.\n",dt);												// ***TEMP***
	m_updatetimestamp = now;																	// update time stamp
	double odom = m_throttle.getLocalOdometer();									// get current odometer value
	double dp = odom - m_prevodometer;													// get change in position
	m_prevodometer = odom;																	// update current position
	if (dt <= 0.0 || dt > 1.0 || fabs(dp) > 100)											// sanity check on dt and dp
	{																											// speed calculation is bogus
		if (m_state == MsgSpeedSet::state_run)											// if we are moving
		{	SetFault(Fault::speedtimeout, "Speed estimation values are out of range");	}/// E-stop
		//	First time through, or after a controller reset, we may get bogus values once.
		dt = 0.1;																							// assume zero speed
		dp = 0;
		m_prevspeed = 0;
	}
	float speed = dp / dt;																			// calc raw speed
	////float accel = (speed - m_prevspeed) / dt;												// calc raw acceleration
	m_prevspeed = speed;																		// save previous raw speed
	float prevfilteredspeed = m_speed.GetOutput();									// get output filtered speed
	m_speed.PutInput(speed);																	// update filtered speed
	float accel = (m_speed.GetOutput() - prevfilteredspeed) / dt;				// calc accel from filtered speed
	m_accel.PutInput(accel);																		// update filtered accel
	////logprintf("Filters: Speed %6.2f->%6.2f  Accel %6.2f->%6.2f\n",				// ***TEMP***
	////	speed, m_speed.GetOutput(), accel, m_accel.GetOutput());				// ***TEMP***
	//	Update state of chassis. Does slow timeouts.
	UpdateState();																						// update actual state
	//	System state now updated. Take any needed control actions.
	//	Final sanity check on actions.  May trigger faults.
	ControllerSanityCheck();																		// perform sanity check on controller tracking
	//	In run state, set speed commands will set goals
	if (GetState() != MsgSpeedSet::state_run)												// if not in run state
	{	SetGoalsStopped();	}																		// apply brakes.
	//	Debug output
	if (m_verbose)																						// if verbose mode
	{	if (now > m_dumptimestamp + k_dump_interval)								// if time for dump
		{	Dump();																						// dump
			m_dumptimestamp = now;
		}
	}
}
//
//	SetSpeed  -- update controller goals
//
//	Given desired speed and accel, issue appropriate commands.
//
//	Must be called every 100ms in run mode, or vehicle will stop.
//
//	Strategy:
//	--	If desired speed is 0, always use brake.
//	-- If desired speed is slower than current speed by more than allowed overshoot, use brake.
//	-- Otherwise use throttle.
//
//	When the error is zero, the throttle is used.  The throttle servo has an I term, so once
//	the accumulated speed error builds up, the throttle should remain constant when there
//	is no speed error.  
//
void Chassis::SetSpeed(float desiredaccel, float desiredspeed, float desiredsteer)
{
	float brakesetting = 1;																			// default is full brake
	float throttlesetting = 0;
	float steersetting = 0;																			// default is straight ahread
	if (GetState() == MsgSpeedSet::state_run)											// only in RUN state
	{	float speederr = desiredspeed-GetSpeed();										// speed error
		if (desiredspeed <= k_stationary_speed)											// if stopping
		{	throttlesetting = 0;
			////m_accumspeederror = 0;															// clear throttle integrator
			float throttlesettingsink = calcThrottle(desiredspeed);					// calc throttle to update integrator, but do not use value
			throttlesettingsink = throttlesettingsink;										// make compiler happy
			brakesetting = calcBrake(desiredaccel);										// calculate brake
		} else if (speederr < -k_max_speed_overshoot) {							// if decelerating enough that we need to brake
			throttlesetting = 0;
			//// m_accumspeederror = 0;														// clear throttle integrator
			float throttlesettingsink = calcThrottle(desiredspeed);					// calc throttle to update integrator, but do not use value
			throttlesettingsink = throttlesettingsink;										// make compiler happy
			brakesetting = calcBrake(desiredaccel);										// calculate brake
		} else {																							// if accelerating
			throttlesetting = calcThrottle(desiredspeed);								// calc throttle
			brakesetting = 0;																			// release brake
			//	Hill holder. Lock brakes until throttled up.
			if (m_hillholder && (!engineabovehillholdrelease()))					// if hill holding, and throttle not up yet
			{	brakesetting = k_hill_hold_brake_fraction;	}							// lock brakes until revved up
		}
	}
	//	Continue to steer if IDLE or above, so that we steer during fault recovery.
		//	Do steering
	if (GetState() >= MsgSpeedSet::state_idle)											// IDLE state or above
	{
		steersetting = calcSteering(desiredsteer);										// steering angle from 1/radius
	}
	SetGoals(throttlesetting, brakesetting, steersetting);								// take action to update
}
//
//	calcBrake  -- calculate brake setting for desired accel
//
//	Initial dumb version
//
float Chassis::calcBrake(float desiredaccel)
{
	float accel = GetAccel();																		// get actual acceleration from odometer
	float speed = GetSpeed();																	// get speed
	float dir = GetDir();																				// get desired direction of motion	
	//	A little slip-back is OK; that may happen on a hill. But if the speed is too large, we have a problem.
	if (((speed > 0) && (dir < 0))																// if moving in wrong direction
	|| ((speed < 0) && (dir > 0)))
	{	if (fabs(speed) > k_max_slipback_speed)										// if slipping back too fast
		{	SetFault(Fault::slipback,"Moving in wrong direction");					// sliding backwards?  Trouble.
			m_hillholder = true;																		// set hill-holder mode
			logprintf("Begin hill holding mode, speed %1.2f m/s.\n",speed);	// enable hill holder
			return(1.0);																					// slam on brakes
		}
	}
	if (fabs(speed) < k_lock_brakes_speed)												// if close enough to stopped to lock brakes
	{	return(1.0);	}																					// lock brakes
	//	Moving, must plan decel.																						
	accel = accel*dir;																					// adjust accel direction
	//	Simple proportional controller
	float desiredbrake = accel*k_brake_p_gain;											// calculate P gain
	return(desiredbrake);
}
//
//	calcThrottle  -- calculate throttle setting for desired speed
//
//	PID controller.
//	Initial dumb version.
//
//	Need sanity checks:
//		1.  Throttle setting must not be above limit for current RPM.
//		2.	RPM must not be above limit for current speed.
//
//	Output is scaled to range 0 .. 1
//
float Chassis::calcThrottle(float desiredspeed)
{
	float accel = GetAccel();																		// get actual acceleration from odometer
	float speed = GetSpeed();																	// get speed
	float dir = GetDir();																				// get desired direction of motion
	//	A little slip-back is OK; that may happen on a hill. But if the speed is too large, we have a problem.
	if (((speed > 0) && (dir < 0))																// if moving in wrong direction
	|| ((speed < 0) && (dir > 0)))
	{	if (fabs(speed) > k_max_slipback_speed)										// if slipping back too fast
		{	SetFault(Fault::slipback,"Moving in wrong direction");					// sliding backwards?  Trouble.
			m_hillholder = true;																		// set hill-holder mode
			logprintf("Begin hill holding mode, speed %1.2f m/s.\n",speed);										// enable hill holder
			return(0.0);																					// zero throttle
		}
	}
	if (m_hillholder && fabs(speed) > 	k_max_slipback_speed)				// if moving in right direction
	{	m_hillholder = false;																			// done end hill holding
		logprintf("End hill holding mode.\n");
	}																					
	accel = accel*dir;																					// adjust accel direction
	speed = speed*dir;
	float speederr = desiredspeed-speed;													// speed error
	float accelerr = -accel;																			// acceleration error (oversimplified)
	//	Simple proportional controller
	//	Simple PID controller
	if (accelerr > 0) accelerr = 0;																// one-sided accel limit
	float desiredthrottle = k_throttle_gain * (speederr + m_accumspeederror*k_throttle_i_term + accelerr*k_throttle_d_term);
	float basedesiredthrottle = desiredthrottle;											// for debug
	//	Update I term for next time
	const float dt = 0.100;																			// 100ms integration interval
	//	Antiwindup control
	if ((desiredthrottle > 0 && desiredthrottle < 1)										// if unsaturated,  update (antiwindup check)
	|| (m_accumspeederror * speederr < 0))												// if saturated in wrong direction, update
	{	m_accumspeederror += (speederr * dt);		}									// update integrator
	//	Limit rate of throttle change.  But don't limit rate until engine is above idle speed.
	float throttleactual = m_throttle.GetLocalActual() / m_throttle.GetLocalGoalMax();		// scale into 0..1
	if (engineaboveidle())																// if engine above idle speed
	{	if (desiredthrottle > throttleactual+dt*k_max_throttle_rate)	// if increasing too fast
		{	desiredthrottle = throttleactual+dt*k_max_throttle_rate;	}	// hold to rate limit
		if (desiredthrottle < throttleactual-dt*k_max_throttle_rate)	// if decreasing too fast
		{	desiredthrottle = throttleactual-dt*k_max_throttle_rate;	}	// hold to rate limit
	}
	//	If desired speed is 0, always cut throttle
	if (desiredspeed <= 0) desiredthrottle = 0;											// throttle to 0
	//	Limit output range to 0..1
	if (desiredthrottle < 0) desiredthrottle = 0;
	if (desiredthrottle > 1) desiredthrottle = 1;
	static int cntr = 0;																					// ***TEMP***
	if (cntr++ % 10 == 0)
	{	logprintf("CalcThrottle: err %2.2f  accumerr %2.2f cmd %2.2f limited to %2.2f\n",
		speederr, m_accumspeederror, basedesiredthrottle, desiredthrottle);	 // ***TEMP***
	}
	return(desiredthrottle);
}
//
//	SetGoals  -- set desired throttle and brake pressure
//
//	THIS ISSUES COMMANDS TO THE ACTUATORS
//
//	Must be performed every 100ms in RUN mode or E-stop will occur.
//
//	Do not call unless previous Update was error-free.
//
//	Inputs range from 0 to 1, -1 to 1 for steersetting
//
bool Chassis:: SetGoals(float throttlesetting, float brakesetting, float steersetting)
{
	if (m_state != MsgSpeedSet::state_run)												// if not in RUN state
	{	brakesetting = 1; throttlesetting = 0;	}											// force a stop
#ifdef OBSOLETE
	if (brakesetting > 0) {	throttlesetting = 0;		}									// must drop throttle if braking
	if (!brakesreleased()) { throttlesetting = 0; }										// must drop throttle if brakes applied (for whatever reason)
#endif // OBSOLETE  // allow both throttle and brake so hill hold will work
	//	Check READY status for both controllers before commanding either one.
	if (!m_brake.GetLocalAuto())
	{	SetFault(Fault::brake,"Brake controller not ready");							// E-stop
		return(false);
	}
	if (!m_throttle.GetLocalAuto())
	{	SetFault(Fault::throttle,"Throttle controller not ready");						// E-stop
		return(false);
	}
	//	Scale into controller parameter space
	float throttlegoal = throttlesetting * m_throttle.GetLocalGoalMax();		// scale
	float brakegoal = brakesetting * m_brake.GetLocalGoalMax();				// scale
	float steergoal = 	steersetting * m_steer.GetLocalGoalMax(); 				// scale
	if ((!finite(throttlegoal)) || (!finite(brakegoal)) || (!finite(steergoal)))		// will be NAN if problem
	{	SetFault(Fault::internalerror,"Value for controller goal is a NaN");		// can happen if controllers not initialized properly
		logprintf("Controller settings: throttle %1.2f  brake %1.2f  steer %1.2f\n", throttlesetting, brakesetting, steersetting);
		logprintf("Controller goals: throttle %1.2f  brake %1.2f  steer %1.2f\n", throttlegoal, brakegoal, steergoal);
		return(false);
	}
	//	Set brake
	Controller::Err err = m_brake.SetGoal(brakegoal);									// COMMAND ACTUATOR
	if (err != Controller::ERR_OK)
	{	SetFault(Fault::brake, "Error sending command");	
		return(false);
	}
	//	Set throttle.
	err = m_throttle.SetGoal(throttlegoal);													// COMMAND ACTUATOR
	if (err != Controller::ERR_OK)
	{	SetFault(Fault::throttle, "Error sending command");	
		return(false);
	}
	//	Set steering
	err = m_steer.SetGoal(steergoal);															// COMMAND ACTUATOR
	////logprintf("Set steering goal: %2.1f -> %2.1f\n", steersetting, steergoal);	// ***TEMP***
	if (err != Controller::ERR_OK)
	{	SetFault(Fault::steer, "Error sending command");	
		return(false);
	}
	//	Reset watchdog, but only in run state.
	if (k_state_table[m_state].m_enablewatchdog)										// if watchdog should enable
	{	err = m_brake.ResetWatchdog();														// reset the hardware watchdog
		if (err != Controller::ERR_OK)
		{	SetFault(Fault::throttle, "Error resetting watchdog");	
			return(false);
		}
	}	
	return(true);																							// success
}
//
//	SetGoalsStopped -- set goals for any stopped state
//
bool Chassis::SetGoalsStopped()
{	float steersetting = m_steer.GetLocalGoal()/m_steer.GetLocalGoalMax();				// get last steering goal
	if (!finite(steersetting)) steersetting = 0;																// if not valid, zero steering
	return(SetGoals(0,1,steersetting));																		// idle throttle, full brakes, maintain steering
}
//
//	SetGear -- command a gear change
//
//	This just starts the gear change.
//	During a gear change, the current gear is reported as "gear_unknown".
//	When the transmission is in the new gear, that changes to the desired gear
//
bool Chassis::SetGear(MsgSpeedSet::Gear newgear, bool busy)
{	busy = false;
	if (!stopped()) 																						// if not stopped, 
	{	busy = true;																						// busy, try later
		return(true);																						// sequencing error
	}
	if (m_transmission.GetLocalGoal() == newgear) return(true);				// already doing it
	Controller::Err err = m_transmission.SetGoal(newgear);						// set new gear
	if (err != Controller::ERR_OK)																	// if unable to command
	{	SetFault(Fault::trans, "Error requesting gear change");						// trouble
		return(false);
	}
	return(true);																							// success
}
//
//	GetGear -- get current gear
//
//	As of last Update
//
MsgSpeedSet::Gear Chassis::GetGear()
{	return(m_transmission.GetLocalGear());	}
//
//	GetCurvature -- get turning curvature  
//
//	1/radius in meters, > 0 means right turn)
//
float Chassis::GetCurvature()
{
	float steeringfract = m_steer.GetLocalActual() / m_steer.GetLocalGoalMax();	// get current steering position (-1 .. 1)
	return(calcSteeringRadius(steeringfract));											// convert to 1/r format
}											
//
//	StateChange  --  a state change is occuring. 
//	
//	Internal, private
//
void Chassis::StateChange(MsgSpeedSet::State newstate, const char* msg)
{
	if (newstate == m_state) return;															// no action
	logprintf("CHASSIS STATE CHANGE: %s to %s: %s\n", ErrMsg(m_state), ErrMsg(newstate), msg);	// log
	m_statetimestamp = gettimenow();														// update time stamp
	m_state = newstate;																				// do the state change
}
//
//	UpdateState -- called during update, updates state if needed
//
//	May report faults.
//
void Chassis::UpdateState()
{	assert(m_state <= MsgSpeedSet::state_run);										// must be in range
	////logprintf("UpdateState: in %s\n", k_state_table[m_state].m_name);		// ***TEMP***
	const ChassisState& currentstate = k_state_table[m_state];					// current state
	if (currentstate.m_timelimit > 0 &&
		currentstate.m_timelimit + m_statetimestamp < gettimenow())			// if state has timed out
	{	StateChange(currentstate.m_timeoutstate, "Timeout");						// go to new state
		return;
	}
	//	Updates based on current chassis situation
	bool notestop, notpaused, inauto;
	bool enabled = unpaused(notestop, notpaused, inauto);						// E-stop control status
	switch (m_state) {																				// fan out on state
	case MsgSpeedSet::state_idle:
		if (!enginerunning())																			// engine has stalled
		{	logprintf("Engine has stalled.\n");
			SetFault(Fault::engine);																	// engine has stalled
		}
		if (!enabled)																						// paused, go to pause state
		{	if (stopped())																				// wait for vehicle to fully stop
			{	StateChange(MsgSpeedSet::state_paused, "E-stop not enabled");	}		// then go to paused state
		}
		break;

	case MsgSpeedSet::state_run:																// engine should be running in these states
	case MsgSpeedSet::state_runwait:
		if (!enginerunning())																			// engine has stalled
		{	logprintf("Engine has stalled.\n");
			SetFault(Fault::engine);																	// engine has stalled
		}
		if (!enabled)																						// paused, go to idle state, then paused
		{	StateChange(MsgSpeedSet::state_paused, "E-stop not enabled");	}
		break;
		
	case MsgSpeedSet::state_shifting:														
		if (!enginerunning())																			// engine has stalled
		{	logprintf("Engine has stalled.\n");
			SetFault(Fault::engine);																	// engine has stalled
		}
		if (!enabled)																						// paused, go to idle state
		{	StateChange(MsgSpeedSet::state_idle, "E-stop not enabled");	 break; }
		if (indesiredgear())
		{	StateChange(MsgSpeedSet::state_shifting, "Shift completed");	}	// done shifting
		break;

	case MsgSpeedSet::state_paused:														// paused, see if we need to stop
		if (!notestop || !inauto)																		// if in E-stop or manual
		{	StateChange(MsgSpeedSet::state_parked, "E-stop not enabled");	// go all the way to parked
			break;
		}
		if (!enginerunning())																			// engine has stalled
		{	logprintf("Engine has stalled.\n");
			SetFault(Fault::engine);																	// engine has stalled
		}
		break;
		
	case MsgSpeedSet::state_starting:														// are we cranking?
		if (enginerunning())
		{	logprintf("Engine has started.\n");													// end cranking
			StateChange(MsgSpeedSet::state_paused, "Engine has started"); 								// engine has started
		}
		break;

	default:																									// should not happen
		StateChange(MsgSpeedSet::state_parked, "Invalid state");				// go all the way to parked
		break;		
	}
}
//
//	SetState -- causes a state change based on a request from the outside.
//
//	Returns true if the request is permitted.
//	The state might not change immediately, but if accepted, it should get there
//	eventually. provided that the command is repeated.
//
//	Only state_run, state_parked, state_paused and state_idle can be commanded.
//
bool Chassis::SetState(MsgSpeedSet::State newstate, bool& busy)
{	busy = false;																						// did not report busy
	if (newstate == m_state) return(true);													// no action needed
	bool notestop, notpaused, inauto;
	bool enabled = unpaused(notestop, notpaused, inauto);						// E-stop control status
	switch (newstate) {
	case  MsgSpeedSet::state_parked:														// request for parked is always legal, although drastic
		break;
		
	case MsgSpeedSet::state_paused:
		if (m_state > MsgSpeedSet::state_paused) break;							// can always go down
		{	switch (m_state) {																		// fan out on previous state					
				case MsgSpeedSet::state_reset:												// resetting
				case MsgSpeedSet::state_starting:											// cranking
				case MsgSpeedSet::state_startwait:											// waiting for starter to cool
				case MsgSpeedSet::state_runwait:											// in DARPA-required wait
					busy = true;																			// busy, try later
					return(true);																			// but OK to ask
										
				case MsgSpeedSet::state_parked:											// we are parked, start cranking
					if (!enabled)																			// not enabled, can't come out of parked
					{	busy = true; 																	// busy
						return(true);																		// can't do it now
					}
					newstate = MsgSpeedSet::state_starting;								// begin startup
					busy = true;
					break;
														
				default:
					return(false);																		// illegal request
			}
		}		
		break;
		
	case MsgSpeedSet::state_idle:
		if (m_state > MsgSpeedSet::state_idle) break;									// can always drop to this state
		{	switch (m_state) {																		// fan out on previous state
				case MsgSpeedSet::state_idle:													// in IDLE state
					if (!enabled)																			// not enabled, can't come out of parked
					{	busy = true; 																	// busy
						return(true);																		// can't do it now
					}
					break;																					// OK to go to RUN state
					
				case MsgSpeedSet::state_runwait:											// waiting to start
					if (!enabled)																			// not enabled, can't come out of parked
					{	busy = true; 																	// busy
						break;																				// can't do it.
					}
					busy = true;																			// busy, try later
					return(true);																			// but OK to ask
					
				case MsgSpeedSet::state_paused:											// if in paused
					if (!enabled)																			// not enabled, can't come out of parked
					{	busy = true; 																	// busy
						return(true);																		// can't do it.
					}
					busy = true;
					newstate = MsgSpeedSet::state_runwait;								// enter RUNWAIT
					break;
					
				default:																						// anything lower
					SetState(MsgSpeedSet::state_paused, busy);						// try to go to paused first
					busy = true;
					return(true);
			}
		}	
		break;				
		
	case MsgSpeedSet::state_run:
		{	switch (m_state) {																		// fan out on previous state
				case MsgSpeedSet::state_idle:													// in IDLE state
					if (!stopped())																		// must be fully stopped, brakes locked, throttle cut, to go to run
					{	busy = true;																		// busy, try later
						return(true);
					}
					if (!indesiredgear())																// if not in desired gear, can't advance yet
					{	busy = true;
						return(true);
					}
					break;																					// OK to go to RUN state
					
				case MsgSpeedSet::state_shifting:											// shift in progress
					if (indesiredgear()) break;													// done shifting
					busy = true;																			// busy, try later
					return(true);																			// but OK to ask
					
				default:																						// anything lower
					SetState(MsgSpeedSet::state_idle, busy);								// try to go to idle first
					busy = true;
					return(true);
			}					
		}
		break;
		
	case MsgSpeedSet::state_shifting:														// requesting shifting state
		{	switch (m_state) {
				case MsgSpeedSet::state_run:													// in run state, this is legal
				case MsgSpeedSet::state_idle:													// also legal from idle state
					if (!enabled)																			// not enabled, can't go to shifting
					{	busy = true; 																	// busy
						return(true);																		// can't do it now
					}
					if (!stopped())																		// must be fully stopped, brakes locked, throttle cut, to go to run
					{	busy = true;																		// busy, try later
						return(true);
					}
					break;
				
				default:																						// anything lower
					SetState(MsgSpeedSet::state_idle, busy);								// try to go to idle first
					busy = true;
					return(true);
			}
		}
		break;	
				
	default: return(false);																			// not allowed to change to other states
	}
	StateChange(newstate, "as requested");												// do the state change
	return(true);
}
//
//	SetFault  -- causes an emergency stop
//
void Chassis::SetFault(Fault::Faultcode faultid, Controller::Err err)
{	SetFault(faultid, Controller::ErrMsg(err));												// use edited message for error
}
//
//	SetFault  -- causes an emergency stop.
//
//	Drops state from RUN to IDLE, or if already in IDLE, all the way to PARKED.
//
void Chassis::SetFault(Fault::Faultcode faultid, const char* msg)
{	if (m_fault == Fault::none)																	// if no fault
	{	m_fault = faultid;																				// set fault code if any
		const char* faultmsg = Fault::ErrMsg(faultid);									// convert to msg
		if (msg)																								// if additional mesage
		{	logprintf("FAULT: %s - %s\n", faultmsg, msg);	}
		else 
		{	logprintf("FAULT: %s \n", faultmsg);	}	
	}
	MsgSpeedSet::State newstate = MsgSpeedSet::state_parked;				// go to parked state by default
	if (GetState() >=  MsgSpeedSet::state_idle)											// if state above idle
	{	newstate = MsgSpeedSet::state_idle;	}											// only drop to idle if above idle
	else if ((GetState() == MsgSpeedSet::state_runwait) && stopped())		// if in runwait and safely stopped, stay in runwait with watchdog enabled
	{	newstate = MsgSpeedSet::state_runwait; }
	else if (GetState() == MsgSpeedSet::state_paused)								// if paused, stay paused
	{	newstate = MsgSpeedSet::state_paused; }
	StateChange(newstate, msg ? msg : "Fault");										// force a state change
	if (GetState() == MsgSpeedSet::state_run)											// NO - must NEVER be in run state here
	{	throw("INTERNAL ERROR: unable to get out of run state");	}			// abort, hardware timer will stop
	//	No longer in RUN state, so watchdog timer will stop.
	//	Next update will also force controllers to zero.
}
//
//	ResetFault  -- reset a fault condition
//
//	Only valid when stopped
//
bool Chassis::ResetFault()
{	
	if (GetFault() == Fault::none) return(true);											// there is no fault
	if (!stopped()) return(false);																	// not stopped, can't clear
	m_fault = Fault::none;																			// fault cleared
	logprintf("Chassis fault cleared.\n");														// report cleared
	return(true);
}
//
//	GetSpeed  --  get current driveshaft-derived speed.
//
//	This is signed; > 0 means forward.
//
float Chassis::GetSpeed()
{
	return(m_speed.GetOutput());																	// get filtered output
}
//
//	GetAccel  --  get current driveshaft-derived acceleration
//
//	This is signed; > 0 means forward.
//
float Chassis::GetAccel()
{
	return(m_accel.GetOutput());																		// get filtered output
}
//
//	GetDir  -- direction of travel
//
int Chassis::GetDir()
{
	if (GetGear() == MsgSpeedSet::gear_reverse) return(-1);						// only -1 if in reverse
	return(1);
}
//
//	GetOdometer -- get odometer value
//
double Chassis::GetOdometer()
{	return(m_throttle.getLocalOdometer());	}													// get latest odometer value
//
//	GetHint  --  get current driveshaft-derived speed
//
MsgSpeedSetReply::Hint Chassis::GetHint()
{
	return(MsgSpeedSetReply::hint_none);														// ***TEMP***
}
//
//	brakeslocked -- true if we sense brake pressure is applied,
//
bool Chassis::brakeslocked()
{	if (!GetControllersReady()) return(false);							// controllers not ready, can't read
	float brakefract = m_brake.GetLocalActual() / m_brake.GetLocalGoalMax();	// get brake fraction
	if (!finite(brakefract)) return(false);									// NaN indicates invalid values
	if (brakefract < 0.5) return(false);										// insufficient brake prssure
	return(true);																		// OK, brakes locked
}
//
//	brakesreleased -- true if we sense brake pressure is applied,
//
bool Chassis::brakesreleased()
{	if (!GetControllersReady()) return(false);							// controllers not ready, can't read
	float brakefract = m_brake.GetLocalActual() / m_brake.GetLocalGoalMax();	// get brake fraction
	if (!finite(brakefract)) return(false);									// NaN indicates invalid values
	if (brakefract > 0.1) return(false);										// some brake pressure
	return(true);																		// OK, brakes released
}
//
//	notmoving -- true if we sense driveshaft is not moving
//
bool Chassis::notmoving()
{	if (!GetControllersReady()) return(false);							// controllers not ready, can't read
	if (fabs(GetSpeed()) < k_stationary_speed) return(true);			// not moving, OK
	return(false);																	// fails
}
//
//	stopped  -- true if safely stopped and brakes locked
//
bool Chassis::stopped()
{	if (!GetControllersReady()) return(false);							// controllers not ready, can't read
	if (!notmoving()) return(false);											// moving, trouble
	if (!brakeslocked()) return(false);										// brakes not locked, trouble
	if (!throttlezero()) return(false);										// throttle not at zero, trouble
	return(true);																		// we are stopped
}
//
//	enginerunning  -- true if engine is turning above cranking speed
//
bool Chassis::enginerunning()
{	if (!GetControllersReady()) return(false);							// controllers not ready, can't read
	return(m_throttle.getLocalRPM() > k_min_idle_rpm);			// check engine RPM
}
//
//	engineaboveidle -- engine speed is above idle; motion should start
//
bool Chassis::engineaboveidle()
{	if (!GetControllersReady()) return(false);							// controllers not ready, can't read
	return(m_throttle.getLocalRPM() > k_max_idle_rpm);			// check engine RPM
}
//
//	engineabovehillholdrelease -- engine speed high enough to release brakes going uphill
//
bool Chassis::engineabovehillholdrelease()
{	if (!GetControllersReady()) return(false);							// controllers not ready, can't read
	return(m_throttle.getLocalRPM() > k_hill_hold_rpm_release);		// check engine RPM
}
//
//	throttlezero -- throttle is at zero
//
bool Chassis::throttlezero()
{	if (!GetControllersReady()) return(false);							// controllers not ready, can't read
	float throttlefract = m_throttle.GetLocalActual() / m_throttle.GetLocalGoalMax();	// get throttle fraction
	return(throttlefract < 0.05);												// throttle actually reached zero
}
//
//	indesiredgear -- true if in desired gear
//
bool Chassis::indesiredgear()
{	if (!GetControllersReady()) return(false);							// controllers not ready, can't read
	////if (GetGear() == MsgSpeedSet::gear_unknown) return(false);	// in bogus gear
	return(GetGear() == m_transmission.GetLocalGoal());		// true if goal and actual are the same
}
//
//	unpaused -- true if ready to go, E-stop enabled and unpaused
//
//	Steering digital inputs are:
//		1:	E-stop not disabled
//		2: E-stop unpaused
//		3:	In AUTO mode
//
bool Chassis::unpaused(bool& notestop, bool& notpaused, bool& inauto)
{	int digitalinputs;
	Controller::Err err = m_steer.DigitalInputGetAll(&digitalinputs);		// TI | int (0 to 127)
	if (err != Controller::ERR_OK) return(false);						// not ready
	notestop = digitalinputs & 1;											// E-stop not disabled, digital in 1
	notpaused = digitalinputs & 2;											// not paused, digital in 2
	inauto = digitalinputs & 4;												// AUTO mode, digital in 3
	bool enabled = notestop && notpaused && inauto;			// true if all good
	return(enabled);																// if true, good to go
}
//
//	unpaused - short form
//
bool Chassis::unpaused()
{	bool notestop, notpaused, inauto;
	return(unpaused(notestop, notpaused, inauto)); 				// short form
}
//
//	ControllerSanityCheck  -- checks that controllers are behaving sanely.
//
void Chassis::ControllerSanityCheck()
{
	if (!GetControllersReady()) return;									// ignore if controllers not active
	//	Check for engine above idle but driveshaft not turning.
	//	We have to use a relatively high RPM threshold here, or we will be unable to push our way
	//	over an obstacle. This may result in a momentary overspeed, but the speed servoloop
	//	will catch that and apply the brakes.
	const float k_min_stationary_speed = 0.05;						// at least 5cm/sec is moving
	if ((m_throttle.getLocalRPM() > k_operating_rpm)
	&& (fabs(GetSpeed()) < k_min_stationary_speed))
	{	SetFault(Fault::stuck, "Engine well above idle but driveshaft not turning");
		return;
	}
	//	Check for substantial throttle setting but engine RPM too low.
	//	This may need a timer to prevent false alarms.
	//	But it's worth having because it recovers from dropouts of the throttle electromagnet and engine stalls.
	if (m_throttle.getLocalRPM() < k_max_idle_rpm)				// if still in near-idle
	{	float throttlefract = m_throttle.GetLocalActual() / m_throttle.GetLocalGoalMax();	// get throttle fraction
		if (throttlefract > k_max_valid_throttle)							// if throttle way up, but engine not going faster
		{	SetFault(Fault::throttle,"Engine did not speed up on throttle up");
			return;
		}
	}
	//	***MORE***
}
//
//	Steering control
//
//	calcSteering -- 
//
//	calcSteering -- convert 1/r to steering goal
//
//	Output is in range -1..1
//
//	From experimental results:
//		goalfraction = P1*CURV+P2*CURV*CURV;		(then apply sign)
//
float Chassis::calcSteering(float curv)
{	const float abscurv = fabs(curv);									// curvature as absolute value
	float absgoalfract = k_steerp1*abscurv + k_steerp2*abscurv*abscurv;	// quadratic approximation
	absgoalfract = std::min(absgoalfract,1.0f);								// never exceed 1
	const float goalfract = curv > 0 ? absgoalfract : - absgoalfract;	// apply original sign
	return(goalfract);
}
//	
//	calcSteeringRadius  -- convert steering goal to 1/r
//
//	Inverse of above. Input in range -1..1
//
float Chassis::calcSteeringRadius(float goalfract)
{
	if (isnan(goalfract)) return(goalfract);										// handle invalid data
	float absgoalfract = fabs(goalfract);											// goal fraction as absolute value
	//	Must now solve formula above for abscurv
	//	float absgoalfract = k_steerp1*abscurv + k_steerp2*abscurv*abscurv;	// quadratic approximation
	//	Solution by quadratic formula
	const float a = k_steerp2;															// constants from tuned parameters
	const float b = k_steerp1;
	const float c = -absgoalfract;
	const float abscurv = (-b + sqrt(b*b - 4*a*c))/(2*a);				// quadratic solution
	const float curv = goalfract > 0 ? abscurv : -abscurv;				// apply sign
	//	Check result
	const float checkgoalfract = calcSteering(curv);						// inverse function, as check
	const float err = fabs(checkgoalfract - goalfract);					// compute error
	if (err > 0.02)																			// if error exceeds 2%
	{	logprintf("ERROR: calcSteeringRadius result incorrect: goalfract %1.4f  checkgoalfract %1.4f  curv %1.4f\n",
			goalfract, checkgoalfract, curv);
	}
	return(curv);																				// done
}

//
//	Dump  --  print info
//	
//	In verbose mode, we get a crude log this way.
void Chassis::Dump()
{	
	if (!GetControllersReady())
	{	logprintf("Controllers not ready.\n"); return; }
	const char* gearnames[5] = {"unknown gear","reverse","neutral","low","high"};
	const char* statemsg = k_state_table[m_state].m_name;				// name of current state
	const char* gearmsg = gearnames[GetGear()];							// name of current gear
	double throttle = m_throttle.GetLocalActual()*100 / m_throttle.GetLocalGoalMax();
	double brake = m_brake.GetLocalActual()*100 / m_brake.GetLocalGoalMax();
	double steer = m_steer.GetLocalActual()*100 / m_steer.GetLocalGoalMax();
	logprintf(">%s (%s) speed %4.1f accel %4.1f  rpm: %4.0f   odom: %8.2f  throttle: %2.0f%%  brake %2.0f%%  steer %1.0f%%\n",
		statemsg, gearmsg, 
		GetSpeed(),
		m_accel.GetOutput(),
		m_throttle.getLocalRPM(), m_throttle.getLocalOdometer(), throttle, brake, steer);
}

