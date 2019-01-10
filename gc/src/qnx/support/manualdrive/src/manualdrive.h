//
//	manualdrive.h  --  manual driving GUI
//
//	John Nagle
//	Team Overbot
//	October, 2003
//
#ifndef MANUALDRIVE_H
#define MANUALDRIVE_H

//	Need to get phAb to search more directories

#include "ablibs.h"
#include <memory>
#include <vector>
#include "simplecontroller.h"
#include "photonwidget.h"
#include "timedloop.h"
#include "hiddconnection.hpp"
#include "drivingcontrolmode.h"
//
//	Joystick support
//
//
//	JoystickConnection - generic connection to all HID devices.
//
class JoystickConnection: public HIDDconnection
{
private: 
	Manualdrive& m_owner;														// owning object
protected:
	void onInsertion(hidd_device_instance_t* instance);			// reports a new device has appeared
public:
	JoystickConnection(Manualdrive& owner)							// link to owning object
	: m_owner(owner)
	{}
	Manualdrive& GetOwner() { return(m_owner); }					// return owner
};
//
//	JoystickDevice -- the device of interest, in this case a joystick
//
class JoystickDevice: public HIDDdevice
{
private:
	hidd_report_props_t m_xprop, m_yprop;								// X and Y property sets
public:
	JoystickDevice(HIDDconnection& conn, hidd_device_instance_t *instance)
		: HIDDdevice(conn, instance)
{}
	void onInsertion();																/// new joystick
	void onHidReport(struct hidd_report* handle, void* report_data,
                            _uint32 report_len, _uint32 flags);
	void onRemoval();																// joystick disconnected
	Manualdrive& GetManualDrive();											// return owner
	void trouble(const char* msg);												// trouble
};
//
//	MoveInfoDisplay  -- move info display
//
class MoveInfoDisplay {
private:
	const char* m_hinttext;															// old values, for optimization
	const char* m_statetext;
	double m_movedistance;
	int m_waypointnumber;
public:
	MoveInfoDisplay();
	void enable(bool enabled);
	void sethint(const char* hint);												// display hint
	void setstate(const char* state);											// display state
	void setmovedistance(double movedistance);						// display movedistance
	void setwaypointnumber(int waypointnumber);					// display waypoint number
};
//
//	class Manualdrive  --  Manual driving support
//
class Manualdrive {
public:
	enum DriveMode { ObserveMode, RemoteManualMode, RemoteSemiautoMode, AutoMode, OfflineSemiautoMode, 
		OfflineAutoMode };	// system mode
public:
	//	Controllers
	SteeringController m_steer;
	BrakeController m_brake;
	ThrottleController m_throttle;
	TransmissionController m_transmission;
private:
	//	Meters
	PhotonMeterGroup m_steering; 
	PhotonMeterGroup m_braking;	
	PhotonMeterGroup m_throttling;
	PhotonMeter m_rpmmeasuring;
	PhotonMeter m_speedmeasuring;
	//	Buttons
	PhotonButton m_runbutton;
	PhotonButton m_stopbutton;
	PhotonButton m_reversebutton;
	PhotonButton m_neutralbutton;
	PhotonButton m_lowbutton;
	PhotonButton m_highbutton;
	//	Panels
	MoveInfoDisplay m_moveinfodisplay;				// move info display
	//	Joystick
	JoystickConnection m_joystick;
	ost::Mutex m_joysticklock;									// lock on joystick data
	bool m_joystickactive;										// true if joystick active
	float m_joyx, m_joyy;										// joystick positions
	//	Message window
	bool m_holdmessage;										// keep this message up until reset
	//	Mode
	DriveMode m_drivemode;									// current mode
	ObserveControlMode	m_observecontrolmode;
	ManualDrivingControlMode m_manualdrivingcontrolmode;
	SemiAutoControlMode	m_semiautocontrolmode;
	OfflineSemiAutoControlMode m_offlinesemiautocontrolmode;
	AutoControlMode	m_autocontrolmode;
	OfflineAutoControlMode m_offlineautocontrolmode;
	//	more to come
private:
	void forceredraw();
	bool updatetransmission();
	bool updatemisc();
	void updatebuttons();
	void updatebuttons(bool isrunmode, bool isreadymode);
	void updatejoystick();
	void setrunmode();
	void setmessage(const char* msg, bool important = false);
	bool hasmessage();
	bool runmode();												// true if running
	bool joystickactive() const { return(m_joystickactive);	} // true if we have a joystick
	bool updatemeterset(SimpleController& ctl, PhotonMeterGroup& meters);
	void updategearbutton(PhotonButton& button, MsgSpeedSet::Gear buttongear, 
	MsgSpeedSet::Gear actualgear, MsgSpeedSet::Gear goalgear);
	DrivingControlMode& getcontrolmode();			// get current control mode
public:
	bool updatesteer();
	bool updatebrake();
	bool updatethrottle();
	bool updateallcontrollers();								// update controllers by direct access
	void emergencystop(const char* why);
	void emergencystop(const char* who, const char* why);
	MoveInfoDisplay& getmoveinfodisplay() { return(m_moveinfodisplay); }// move info display, for driving modes that use it
	PhotonMeter& getspeedmeasuring() {return(m_speedmeasuring); } //	similary for speed display
	void updateGearButtons(MsgSpeedSet::Gear actualgear, MsgSpeedSet::Gear goalgear);
public:
	//	GUI callbacks from Photon
	void refresh_timer();
	void steering_auto_button_new_value(bool ison);
	void steering_goal_meter_move(short meterval);
	void brake_pressure_goal_meter_move(short meterval);
	void throttle_goal_meter_move(short meterval);
	void stop_button_pressed();
	void run_button_pressed();
	void shift_button_pressed(MsgSpeedSet::Gear newgear);
	void base_lost_focus();
	void start_mission_action();
	void reset_controllers_action();
	void reboot_computers_action();
	void mode_change_selected(DriveMode newmode);
	int openwaypoints(const char* filename);
	void quit_menuitem_selected();
	//	Joystick callbacks (concurrent, must lock Photon)
	void joystick_move(float x, float y);						// joystick has moved
	void joystick_valid(bool, const char* msg);			// joystick present and working, or not
	void joystick_button(bool stopbutton, bool runbutton);		// may be extended later with more buttons
	int reset();
	Manualdrive();														// constructor
	~Manualdrive();													// destructor
public:
	static void init(int argc, char* argv[]);					// global initialization
};
#endif // MANUALDRIVE_H