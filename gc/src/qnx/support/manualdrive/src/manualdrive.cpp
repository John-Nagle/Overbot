//
//	manualdrive.cpp  --  support for displaying the navigation system results
//
//	John Nagle
//	Team Overbot
//	October, 2003
//
#include <assert.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/hidut.h>

#include "manualdrive.h"
#include "logprint.h"
#include "proto.h"
#include "ablibs.h"
#include "abimport.h"
//
//	Constants
//
const double k_connectwait = 0.5;										// timeout for TCP connnections to controllers (secs)
const bool k_useudp = true;													// use UDP instead of TCP
//
//	Constants
//
//	These are not limits; they're just used to scale the display
//

//
const int  k_maxsteerval = 300000;				// +90 degrees
const int  k_minsteerval = -300000;				// -90 degrees
const float k_minbrakepress = 0.0;				// voltage from pressure sensor
const float k_maxbrakepress = 2.5;				// 2.5V = 1000psi - actually is a limit for program
const float k_minthrottle = 0;
const float k_maxthrottle = 125000;				// max encoder counts (with new gear)
const float k_minrpm = 0.35;							// min volts from converter
const float k_maxrpm =  6.2;						// max volts from converter (1V = 10HZ)
const float k_minspeed = 0.35;						// min volts from converter
const float k_maxspeed = 6.2;						// max value freq. to volt. can generate (1V = ???)
const double k_odomscale = 0.0044;			// meters per encoder count (approx)

//
//	Class Manualdrive
//
Manualdrive::Manualdrive()
	: m_steer("gcsteer",true), 
		m_brake("gcbrake", true), 
		m_throttle("gcthrottle", true),
		m_transmission("gctransmission", true),
		m_steering(ABW_steering_goal_meter, ABW_steering_position_meter, ABW_steering_trend,
			k_minsteerval, k_maxsteerval),
		m_braking(ABW_brake_pressure_goal_meter, ABW_brake_pressure_meter, ABW_brake_pressure_trend,
			k_minbrakepress, k_maxbrakepress),
		m_throttling(ABW_throttle_goal_meter, ABW_throttle_meter, ABW_throttle_trend, 0, k_maxthrottle),
		m_rpmmeasuring(ABW_rpm_meter, k_minrpm, k_maxrpm),
		m_speedmeasuring(ABW_speed_meter,k_minspeed,k_maxspeed),
		m_runbutton(ABW_run_button),
		m_stopbutton(ABW_stop_button),
		m_reversebutton(ABW_reverse_button), 
		m_neutralbutton(ABW_neutral_button),
		m_lowbutton(ABW_low_button),
		m_highbutton(ABW_high_button),
		m_joystick(*this),
		m_joystickactive(false),
		m_holdmessage(false),
		m_drivemode(ObserveMode),
		m_observecontrolmode(*this),
		m_manualdrivingcontrolmode(*this),
		m_semiautocontrolmode(*this),
		m_offlinesemiautocontrolmode(*this),
		m_autocontrolmode(*this),
		m_offlineautocontrolmode(*this)
{
	//	Set datagram mode for connections to controllers
	m_steer.DatagramMode(k_useudp);
	m_brake.DatagramMode(k_useudp);
	m_throttle.DatagramMode(k_useudp);
	m_transmission.DatagramMode(k_useudp);
	//	Joystick
	m_joystick.Connect();												// wake up joystick, if possible
	//	Set colors for trend graphs
	const size_t k_trendcolorcount = 3;
	const PgColor_t k_trendcolors[k_trendcolorcount] = { Pg_GREEN, Pg_DGREEN, Pg_RED };
	m_steering.m_trend.SetColors(k_trendcolors, k_trendcolorcount);
	m_braking.m_trend.SetColors(k_trendcolors, k_trendcolorcount);
	m_throttling.m_trend.SetColors(k_trendcolors, k_trendcolorcount);
	////m_rpm.m_trend.SetColors(k_trendcolors, k_trendcolorcount);
	////m_speed.m_trend.SetColors(k_trendcolors, k_trendcolorcount);
	//	Set filtering for error estimator - TEMP values
	const float k_steergoalfilter = 0.1;
	const float k_steererrfilter = 1.0;
	m_steer.SetFilterConstants(k_steergoalfilter, k_steererrfilter);	// set error estimator
	const float k_brakegoalfilter = 0.2;
	const float k_brakeerrfilter = 1.0;
	m_brake.SetFilterConstants(k_brakegoalfilter, k_brakeerrfilter);	// set error estimator
	const float k_throttlegoalfilter = 0.2;
	const float k_throttleerrfilter = 1.0;
	m_throttle.SetFilterConstants(k_throttlegoalfilter, k_throttleerrfilter);	// set error estimator
	//	Set initial mode
	ApModifyItemState (&mode_menu, AB_ITEM_SET, ABN_observe_menuitem, NULL);
	PtSetResource(ABW_base, Pt_ARG_WINDOW_TITLE, getcontrolmode().getmodename(), 0 );	
}
Manualdrive::~Manualdrive()
{}
//
//	setrunmode -- run button pushed
//
void Manualdrive::setrunmode()
{	if (!getcontrolmode().runstopbuttonsmeaningful()) return;	// ignore if read-only mode
	//	***NEED TO RESET CONTROLS TO NEUTRAL POSITIONS
	m_holdmessage = false;												// clear hold on displayed message
	setmessage("");															// clear displayed error message
	bool good = getcontrolmode().setrunmode(true);		// enable watchdog
	if (!good)
	{	emergencystop("Unable to enter run mode.");			// weak message
		return;
	}
	//	Update buttons
	m_stopbutton.SetPressed(true);
	m_runbutton.SetPressed(false);
	logprintf("Run mode enabled.\n");
}
//
//	runmode  -- true if in run mode
//
bool Manualdrive::runmode()
{	return(getcontrolmode().getrunmode());	}					// true if watchdog active
//
//	emergencystop  -- stops vehicle
//
void Manualdrive::emergencystop(const char*msg)
{	bool wasrunning = runmode();										// true if was in run mode	
	getcontrolmode().setrunmode(false);							// tell driving controller to stop
	getcontrolmode().emergencystopactivated(msg,"");		// take emergency-stop action at control level
	//	***NEED TO RESET CONTROLS TO NEUTRAL POSITIONS
	//	Message display.
	//	Display latest message, but hold actual emergency stop message until reset.
	if (wasrunning)																// if was running
	{	logprintf("EMERGENCY STOP: %s\n", msg);					// display msg
	}									
	setmessage(msg,wasrunning);										// display message. Important if it stopped us
		
}
//
//	emergencystop  --  two-arg form
//
void Manualdrive::emergencystop(const char* who, const char* why)
{	char s[100];
	snprintf(s,sizeof(s),"\"%s\": %s",who, why);					// what component is unhappy
	emergencystop(s);
}
//
//	updatemeterset  -- update meters for a single controller
//
bool Manualdrive::updatemeterset(SimpleController& ctl, PhotonMeterGroup& meters)
{	//	Try to connect.  But call UpdateState anyway if fail, so meters turn red.
	if (!ctl.Connected())													// if not connected
	{	int stat = ctl.Connect(k_connectwait);					// try to connect
		stat = stat;															// ignore status
	}
	//	Begin update cycle
	Controller::Err stat = ctl.UpdateState();						// update local state from controller
	//	Local goal and actual are updated if possible. Otherwise, they are NAN values.
	//	So we always update the display.
	meters.m_actualmeter.SetValue(ctl.GetLocalActual());	
	meters.m_goalmeter.SetValue(ctl.GetLocalGoal());
	float trendvals[3] = {ctl.GetLocalGoal(), ctl.GetLocalActual(), ctl.GetFilteredError()};	// values for trend
	meters.m_trend.SetValue(trendvals,3);
	//	If there was a problem, we have to report that, and can't command.
	if (stat != Controller::ERR_OK)									// if problem
	{	////emergencystop(ctl.Hostname(),Controller::ErrMsg(stat));				// report error
		return(false);
	}
	//	Check that controller is READY and AUTO.
	if (!ctl.GetLocalAuto()) 												// if not AUTO mode, no commanding
	{	////emergencystop(ctl.Hostname(),"controller not ready");
		return(false);	}													// if not AUTO mode, no commanding
	//	Success. We can command the controller.
	return(true);
}
//
//	updatesteer -- an update cycle for the meters
//
bool Manualdrive::updatesteer()
{	bool ok = updatemeterset(m_steer, m_steering);		// update meters and state for this controller
	if (!ok) return(false);													// trouble, can't control
	return(true);
}
//
//	updatebrake -- update brake meters
//
bool Manualdrive::updatebrake()
{
	bool ok = updatemeterset(m_brake, m_braking);		// update meters and state for this controller
	if (!ok) return(false);													// trouble, can't control
	return(true);
}
//
//	updatetransmission -- update transmission position
//
bool Manualdrive::updatetransmission()
{	
	if (!m_transmission.Connected())											// if not connected
	{	int stat = m_transmission.Connect(k_connectwait);			// try to connect
		stat = stat;																		// ignore status
	}
	Controller::Err stat = m_transmission.UpdateState();				// get controller state
	if (stat != Controller::ERR_OK)												// if can't talk to controller
	{	m_reversebutton.SetColor(Pg_RED);									// turn all buttons red
		m_neutralbutton.SetColor(Pg_RED);
		m_lowbutton.SetColor(Pg_RED);	
		m_highbutton.SetColor(Pg_RED);		
		return(false);														// fails
	}
	float actualgearnum = m_transmission.GetLocalActual();
	float goalgearnum = m_transmission.GetLocalGoal();
	//	We can talk to the transmission.
	MsgSpeedSet::Gear actualgear = MsgSpeedSet::Gear(actualgearnum);	// get gear number as enum
	MsgSpeedSet::Gear goalgear = MsgSpeedSet::Gear(goalgearnum);	// get gear number as enum
	updateGearButtons(actualgear, goalgear);				// update the gear buttons
	return(true);																// OK
}
//
//	updateGearButtons -- update the actual buttons on screen.
//
void Manualdrive::updateGearButtons(MsgSpeedSet::Gear actualgear, MsgSpeedSet::Gear goalgear)
{
	updategearbutton(m_reversebutton, MsgSpeedSet::gear_reverse, actualgear, goalgear);
	updategearbutton(m_neutralbutton, MsgSpeedSet::gear_neutral, actualgear, goalgear);
	updategearbutton(m_lowbutton, MsgSpeedSet::gear_low, actualgear, goalgear);
	updategearbutton(m_highbutton, MsgSpeedSet::gear_high, actualgear, goalgear);		
}
//
//	updategearbutton  -- update gear info for each button
//
void Manualdrive::updategearbutton(PhotonButton& button, MsgSpeedSet::Gear buttongear, 
	MsgSpeedSet::Gear actualgear, MsgSpeedSet::Gear goalgear)
{
	bool runstopmeaningful = getcontrolmode().runstopbuttonsmeaningful(); // should RUN and STOP buttons show?
	button.SetPressed(false,!runstopmeaningful);			// set unpressed, ghost if appropriate
	if (goalgear != buttongear)										// if not goal gear
	{	button.ResetColor();												// go to normal color
		return;
	}
	if (goalgear == actualgear)										// if in gear
	{	button.SetColor(Pg_GREEN); }								// green if in gear
	else 
	{	button.SetColor(Pg_YELLOW); }								// yellow if changing
}
//
//	updatejoystick  -- update on every cycle if joystick active
//
void Manualdrive::updatejoystick()
{
	if (m_joystickactive)													// if joystick working
	{ 	float x,y;
		{	ost::MutexLock lok(m_joysticklock);					// joystick data updates asychronously
			x = m_joyx;														// get a stable value
			y = m_joyy;
		}
		getcontrolmode().joystick_move(x,y);						// apply joystick action
	}
}
//
//	updatemisc -- update misc. meters
//
//	RPM, speedometer, and odometer
//
bool Manualdrive::updatemisc()
{	const float k_NaN = _FNan._Float;									// quiet NaN value for floats					
	//	Update RPM
	float rpm = k_NaN;
	Controller::Err stat = m_throttle.AnalogInputGet(1,&rpm);	// get RPM from Analog 1
	if (stat != Controller::ERR_OK)
	{	rpm = k_NaN;	}													// force meter to turn red
	m_rpmmeasuring.SetValue(rpm);								// set as value
	//	Update speedometer
	float speed = k_NaN;
	stat = m_throttle.AnalogInputGet(2,&speed);			// get speed from Analog 2
	if (stat != Controller::ERR_OK)
	{	speed = k_NaN;	}												// force meter to turn red
	m_speedmeasuring.SetValue(speed);						// set value
	//	Update odometer
	int odometerint;														// odometer as 24-bit integer
	stat = m_throttle.ActualPosnAuxGetUnscaled(&odometerint);			// get odometer
	if (stat != Controller::ERR_OK)										// unable to get encoder
	{	return(false);	}													// fails
	double odometer = double(odometerint)*k_odomscale;		// calc and set value
	PtSetResource (ABW_odometer_value, Pt_ARG_NUMERIC_VALUE, &odometer, 0 );
	return(true);																// success
}
//
//	updatethrottle -- update throttle info
//
bool Manualdrive::updatethrottle()
{
	bool ok = updatemeterset(m_throttle, m_throttling);		// update meters and state for this controller
	if (!ok) return(false);														// trouble, can't control
	return(true);
}
//
//	Steering action from GUI
//
//	This is only called when the user selects and drags the meter pointer.
//
//	Ineffective if controller does not support it.
//
void Manualdrive::steering_goal_meter_move(short meterval)
{	
	if (m_joystickactive) return;											// ignore if joystick has control
	float	goal = m_steering.m_goalmeter.GetValue();			// get current value from meter
	getcontrolmode().steering_goal_meter_move(goal);		// scaled meter value is goal
}
//
//	Brake action from GUI
//
//	This is only called when the user selects and drags the meter pointer.
//
void Manualdrive::brake_pressure_goal_meter_move(short meterval)
{	if (m_joystickactive) return;						// ignore if joystick has control
	float	goal = m_braking.m_goalmeter.GetValue();			// get current value from meter
	getcontrolmode().brake_pressure_goal_meter_move(goal);		// scaled meter value is goal
}

//	
//	throttle meter moved - actuate throttle
//
//	NOT FOR DRIVING USE - belongs on joystick
//
void Manualdrive::throttle_goal_meter_move(short meterval)
{	if (m_joystickactive) return;						// ignore if joystick has control
	float	goal = m_throttling.m_goalmeter.GetValue();			// get current value from meter
	getcontrolmode().throttle_goal_meter_move(goal);		// scaled meter value is goal
}
//
//	stop_button_pressed -- Emergency stop - stop vehicle now
//
void Manualdrive::stop_button_pressed()
{
	emergencystop("On-screen STOP button pressed");
}
//
//	run_button_pressed  -- re-enable run mode.
//
//	But zero throttle and set brakes first
//
void Manualdrive::run_button_pressed()
{
	setrunmode();
}
//
//	shift_button_pressed -- a shift button has been pressed.  Shift gears.
//
void Manualdrive::shift_button_pressed(MsgSpeedSet::Gear newgear)
{	getcontrolmode().shift_button_pressed(newgear);	// pass to control level
}
//
//	File menu items
//
int Manualdrive::openwaypoints(const char* filename)
{
		PtNotice(ABW_base,0,"Menu item",0,"Unimplemented feature",0,"OK",0,Pt_MODAL|Pt_RELATIVE);
}
void Manualdrive::quit_menuitem_selected()
{
	//	***MORE***
}

//
//	Action menu items
//
//	***NEEDS WORK***
//
//	start_mission_action -- start the current mission
//
//	Only meaningful in auto mode.
//
void Manualdrive::start_mission_action()
{
	PtNotice(0,0,"Menu item",0,"Unimplemented feature",0,"OK",0,Pt_MODAL|Pt_RELATIVE);	// ***TEMP***
}
//
//	reset_controllers -- reset the chassis controllers
//
void Manualdrive::reset_controllers_action()
{
	if (m_throttle.ReadOnly()) 										// ignore if read-only mode
	{	PtNotice(ABW_base,0,"Reset controllers request",0,"Action not allowed in this mode.",0,"OK",0,Pt_MODAL|Pt_RELATIVE);	
		return;
	}
	emergencystop("Resetting controllers.");
	//	Forcibly reset all controllers.	  This is rather drastic. We should consider a confirm dialog.
	m_throttle.Reset();
	m_brake.Reset();
	m_steer.Reset();
	m_transmission.Reset();
}
//
//	reboot_computers  --  reboot the computers on the vehicle
//
void Manualdrive::reboot_computers_action()
{
	emergencystop("Rebooting computers.");
	PtNotice(ABW_base,0,"Menu item",0,"Unimplemented feature",0,"OK",0,Pt_MODAL|Pt_RELATIVE);	// ***TEMP***
}
//
//	Menu item update
//
static void setmenucheckbox(ApMenuLink_t& widget, int item, bool selected)
{	ApModifyItemState (&mode_menu, selected ? AB_ITEM_SET : 0, item, NULL);	}
//
//	Mode menu items
//
void Manualdrive::mode_change_selected(DriveMode newmode)
{
	if (newmode == m_drivemode) return;							// no change
	emergencystop("Mode change requested");				// stop before changing mode
	getcontrolmode().releasecontrol();								// tell old mode to let go
	DriveMode oldmode = m_drivemode;							// save old mode
	m_drivemode = newmode;											// set new mode
	bool taken = getcontrolmode().takecontrol();				// go to new mode
	if (!taken)
	{	m_drivemode = oldmode;											// go back to the old mode
		getcontrolmode().takecontrol();								// tell it to take control
	}
	//	Set appropriate buttons in menu
	setmenucheckbox(mode_menu, ABN_observe_menuitem, m_drivemode == ObserveMode);
	setmenucheckbox(mode_menu, ABN_remote_manual_menuitem, m_drivemode == RemoteManualMode);
	setmenucheckbox(mode_menu, ABN_remote_semiauto_menuitem, m_drivemode == RemoteSemiautoMode);
	setmenucheckbox(mode_menu, ABN_auto_menuitem, m_drivemode == AutoMode);
	setmenucheckbox(mode_menu, ABN_offline_semiauto_menuitem, m_drivemode == OfflineSemiautoMode);
	setmenucheckbox(mode_menu, ABN_offline_auto_menuitem, m_drivemode == OfflineAutoMode);
	//	Update mode in title bar
	PtSetResource(ABW_base, Pt_ARG_WINDOW_TITLE, getcontrolmode().getmodename(), 0 );	
}
//
//	base_lost_focus -- if window loses focus, treat as e-stop
//
//	This is because the red button on screen won't work in that situation.
//	Doesn't actually work.
//
void Manualdrive::base_lost_focus()
{
}
//
//	updatebuttons  -- update on-screen button status
//
//	Run button is green and pressed if running, blank if stopped, greyed out if system not ready.
//
void Manualdrive::updatebuttons(bool isrunmode, bool isreadymode)
{	//	Are these buttons even meaningful in this mode?
	bool runstopmeaningful = getcontrolmode().runstopbuttonsmeaningful(); // should RUN and STOP buttons show?	
	if (!runstopmeaningful)															// if not meaningful
	{	m_runbutton.SetColor(Pg_GREY);
		m_runbutton.SetPressed(false, true);								// not pressed, dim out
		m_stopbutton.SetColor(Pg_GREY);
		m_stopbutton.SetPressed(false, true);								// not pressed, dim out
		return;
	}
	m_stopbutton.ResetColor();													// back to red
	if (isrunmode)
	{	m_runbutton.SetColor(Pg_GREEN);									// run button lights up green when running
	} else {
		m_runbutton.ResetColor();
	}
	//	Set stop/run buttons.  Run button not pressable if not ready.
	m_runbutton.SetPressed(isrunmode, !isreadymode);			// button blocked if not ready
	m_stopbutton.SetPressed(!isrunmode);
}
//
//	updateallcontrollers -- update all controllers by direct access
//
//	This is read only; commanding is separate.
//
bool Manualdrive::updateallcontrollers()
{
	bool steerok = updatesteer();											// update steering meters
	bool brakeok = updatebrake();
	bool throttleok = updatethrottle();
	bool transok = updatetransmission();
	bool miscok = updatemisc();											// read misc, meters
	bool ready = steerok && brakeok && throttleok && transok && miscok;		// true if ready to go
	return(ready);																	// good to go
}
//
//	refresh_timer  -- refresh meters
//
void Manualdrive::refresh_timer()
{
	if (getcontrolmode().controlleraccessallowed())				// if allowed to read controllers
	{	updateallcontrollers();	}												// ***TEMP***
	updatejoystick();																// update joystick info
	getcontrolmode().updatedrive();										// update, may be good or not
	updatebuttons(runmode(), getcontrolmode().runbuttonallowed());		// update button status
}
//
//	forceredraw -- force a redraw cycle
//
void Manualdrive::forceredraw()
{	////PtDamageWidget(ABW_video_port);							// force a redraw
}
//
//	setmessage -- set message on screen
//
void Manualdrive::setmessage(const char* msg, bool important)
{	assert(msg);
	if ((!important) && m_holdmessage) return;										// ignore if holding previous message
	PtSetResource(ABW_message_text, Pt_ARG_TEXT_STRING, msg, 0); 	// set text
	m_holdmessage = important;															// keep importance
}
//
//	hasmessage -- true if message currently displayed
//
bool Manualdrive::hasmessage()
{	char* msg = 0;
	PtGetResource(ABW_message_text, Pt_ARG_TEXT_STRING, &msg, 0);	// get address of message
	return(msg[0] != '\0');											// true if string is non-null
}
//
//	Joystick support
//
//	joystick_move  -- joystick has moved
//
//	X is steering 
//	Y is throttle/brake.  
//
//	This is called from the HID thread, and has a small stack. So we can't do much here.
//
void Manualdrive::joystick_move(float x, float y)
{////	logprintf("X=%5.3f  Y=%5.3f   ",x,y);						// ***TEMP***
	ost::MutexLock lok(m_joysticklock);						// locking
	m_joyx = x;															// save for next GUI update
	m_joyy = y;							
}
//
//	joystick_valid  -- a joystick has appeared
//
void Manualdrive::joystick_valid(bool valid, const char* msg)
{
	PhotonLock lok;													// automatic locking
	if (m_joystickactive && valid)								// if second joystick
	{	emergencystop("Multiple joysticks - fix"); 
		m_joystickactive = false;
		return;
	}
	emergencystop(msg);											// always stop on joystick change
	m_joyx = m_joyy = 0.5;										// center values
	m_joystickactive = valid;										// set validity
}
//
//	joystick_button -- a joystick button has been pressed
//
void Manualdrive::joystick_button(bool stopbutton, bool runbutton)
{	PhotonLock lok;													// automatic locking
	if (stopbutton)
	{	emergencystop("Joystick stop button pressed.");
		return;
	}
	if (runbutton)
	{	run_button_pressed();	}
}
//
//	getcontrolmode -- what mode are we in?
//
//	This selects the subclass which currently controls the vehicle.
//
DrivingControlMode& Manualdrive::getcontrolmode()
{
	switch(m_drivemode) {
	case RemoteManualMode: return(m_manualdrivingcontrolmode);	// manual mode via controllers directly
	case RemoteSemiautoMode: return(m_semiautocontrolmode);			// indirect manual, using speed and move servers.
	case OfflineSemiautoMode: return(m_offlinesemiautocontrolmode);	// like semiauto, but offline
	case AutoMode: return(m_autocontrolmode);									// true auto
	case OfflineAutoMode: return(m_offlineautocontrolmode);				// like auto, but offline
	default:	return(m_observecontrolmode);											// when in doubt, read only
	}
}
//
//	class MoveInfoDisplay  -- updates some on-screen fields
//
MoveInfoDisplay::MoveInfoDisplay()
:	m_hinttext(0), m_statetext(0), m_movedistance(0), m_waypointnumber(0)
{}
void MoveInfoDisplay::enable(bool enabled)
{
	if (!enabled)																						// clear when not in use
	{	sethint("");
		setstate("");
		setmovedistance(0);
		setwaypointnumber(0);
	}
}
void MoveInfoDisplay::sethint(const char* msg)
{	if (msg == m_hinttext) return;															// avoid unnecessary updates
	m_hinttext = msg;
	PtSetResource(ABW_move_hint_text, Pt_ARG_TEXT_STRING, msg, 0); 	// set text
}
void MoveInfoDisplay::setstate(const char* msg)
{	if (msg == m_statetext) return;															// avoid unnecessary updates
	m_statetext = msg;
	PtSetResource(ABW_move_state_text, Pt_ARG_TEXT_STRING, msg, 0); 	// set text
}
void MoveInfoDisplay::setmovedistance(double movedistance)
{	if (movedistance == m_movedistance) return;
	m_movedistance = movedistance;
	PtSetResource (ABW_move_distance_value, Pt_ARG_NUMERIC_VALUE, &movedistance, 0 );
}
void MoveInfoDisplay::setwaypointnumber(int waypointnumber)
{	if (waypointnumber == m_waypointnumber) return;
	m_waypointnumber = waypointnumber;
	PtSetResource (ABW_move_waypoint_number_value, Pt_ARG_NUMERIC_VALUE, waypointnumber, 0 );
}
//
//	Static functions
//
//
//	init -- called at application startup
//
void Manualdrive::init(int argc, char* argv[])
{	for (int i=1; i<argc; i++)
	{	const char* arg = argv[i];
		if (arg[0] == '-')
		{	switch(arg[1]) {
			default: break;
			}
		}
	}
}				



