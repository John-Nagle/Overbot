//
//	drivingcontrolmode.h  --  manual driving GUI
//
//	John Nagle
//	Team Overbot
//	October, 2003
//
#ifndef DRIVINGCONTROLMODE_H
#define DRIVINGCONTROLMODE_H

//	Need to get phAb to search more directories

#include "ablibs.h"
#include <memory>
#include <vector>
#include "simplecontroller.h"
#include "photonwidget.h"
#include "timedloop.h"
#include "mapservermsg.h"
#include "remotemessaging.h"
#include "throttlecontroller.h"
#include "transmissioncontroller.h"
#include "brakecontroller.h"
#include "steeringcontroller.h"
//
//	Constants
//
//
//	Constants for watchdogs
//
const float k_watchdog_interval = 0.100;					// must toggle watchdog this often
const int k_watchdog_priority = 14;							// Value for test program only.
const unsigned char k_reset_ticks = 10;					// this many missed ticks and we stop	
//
//	HardwareWatchdog -- resets hardware watchdog timer periodically.
//
//	Used in manual driving mode.
//
class Manualdrive;
class HardwareWatchdog: protected TimedLoop
{	unsigned char m_ticks;											// count down, when zero, watchdog halts
	BrakeController* m_controller;								// controller that owns watchdog device
public:
	HardwareWatchdog(BrakeController* control, double interval, int priority);
	void keepalive();												// screen updated and alive, keep going
	void enable();
	void disable();
	bool Looping()  { return(TimedLoop::Looping()); }	// export Looping primitive
protected:
	void code();															// called by timed loop
};	
//
//	SoftwareWatchdog -- resets move server timer periodically.
//
//	Used in manual driving mode via move server
//
class SemiAutoModes;
class SoftwareWatchdog: protected TimedLoop
{	unsigned char m_ticks;											// count down, when zero, watchdog halts
	SemiAutoModes& m_owner;									// owning object
public:
	SoftwareWatchdog(SemiAutoModes& owner, double interval, int priority);
	void keepalive();													// screen updated and alive, keep going
	void enable();
	void disable();
	bool Looping()  { return(TimedLoop::Looping()); }	// export Looping primitive
protected:
	void code();															// called by timed loop
};	
//
//	class DrivingControlMode  -- who is in control
//
//	Abstract class - subclass for desired subsystem
//
class Manualdrive;																// forward
class DrivingControlMode {
private:
	Manualdrive& m_owner;													// owning object
public:
	DrivingControlMode(Manualdrive& owner);						// constructor
public:
	//	To be implemented in child object
	virtual const char* getmodename() const = 0;					// name of this mode
	virtual bool takecontrol() = 0;											// take control if possible
	virtual void releasecontrol() = 0;										// release control (mandatory)
	virtual void emergencystopactivated(const char* who, const char* why) = 0;	// incoming emergency stop
	virtual void joystick_move(float x, float y) = 0;					// joystick has moved
	virtual bool setrunmode(bool runmodeon) = 0;				// request run mode, may be refused
	virtual bool getrunmode() = 0;											// true if in run mode
	virtual bool runstopbuttonsmeaningful() { return(false); }	// true if run button should be visible
	virtual bool runbuttonallowed() { return(false); }				// true if run button is allowed to work
	virtual bool controlleraccessallowed() { return(false); }	// true if allowed to touch controllers directly
	virtual void updatedrive() {}											// called to keep this alive
	virtual void steering_goal_meter_move(float goal) {}
	virtual void brake_pressure_goal_meter_move(float goal) {}
	virtual void throttle_goal_meter_move(float goal) {}
	virtual void shift_button_pressed(MsgSpeedSet::Gear newgear) {}
public:
	Manualdrive& getowner() { return(m_owner); }
};
//
//	ObserveControlMode -- watch only mode
//
//	Doesn't do anything
//
class ObserveControlMode: public DrivingControlMode
{
public:
	ObserveControlMode(Manualdrive& owner) 
	: DrivingControlMode(owner)
	{}
protected:
	const char* getmodename() const;									// name of this mode
	bool takecontrol();															// take control if possible
	void releasecontrol();														// release control (mandatory)
	void emergencystopactivated(const char* who, const char* why);	// incoming emergency stop
	void joystick_move(float x, float y);									// joystick has moved
	bool setrunmode(bool runmodeon);									// request run mode, may be refused
	bool getrunmode() { return(false); }								// never in run mode
	bool controlleraccessallowed() { return(true); }				// true if allowed to touch controllers directly
};
//
//	ManualDrivingControlMode -- manual driving, directly to the actuators
//
class ManualDrivingControlMode: public DrivingControlMode
{
private:
	HardwareWatchdog m_hardwarewatchdog;
public:
	ManualDrivingControlMode(Manualdrive& owner);
protected:
	const char* getmodename() const;									// name of this mode
	bool takecontrol();															// take control if possible
	void releasecontrol();														// release control (mandatory)
	void emergencystopactivated(const char* who, const char* why);	// incoming emergency stop
	void joystick_move(float x, float y);									// joystick has moved
	bool setrunmode(bool runmodeon);									// request run mode, may be refused
	bool getrunmode();															// true if in run mode
	bool runstopbuttonsmeaningful() { return(true); }			// true if run button should be visible
	bool runbuttonallowed();													// true if run button is allowed to work
	bool controlleraccessallowed() { return(true); }				// true if allowed to touch controllers directly
	void updatedrive();															// called to keep alive
	void steering_goal_meter_move(float goal);
	void brake_pressure_goal_meter_move(float goal);
	void throttle_goal_meter_move(float goal);
	void shift_button_pressed(MsgSpeedSet::Gear newgear);
};
//
//	SemiAutoModes -- common base for RemoteSemiautoMode and OfflineSemiautoMode
//
class SemiAutoModes: public DrivingControlMode
{
private:
	ost::Mutex m_lock;																// protect data and port
	bool	m_runmode;																// true if in run mode
	const char* m_moveservernode;										// node on which to look for move server
	const char* m_moveservername;										// name to look for it under
	RemoteMsgClientPort	m_moveserverport;						// port for talking to move server
	MsgSpeedSet::Gear m_requestedgear;								// gear requested by user
	MoveServerMsg::MsgMove m_lastmove;							// last move sent
	MoveServerMsg::MsgMoveReply m_lastreply;					// last reply received
	bool m_coasting;																// coasting, run last move to completion.
	SoftwareWatchdog m_watchdog;										// watchdog
public:
	SemiAutoModes(Manualdrive& owner, const char* moveservernode, const char* moveservername);
	void resendlastmove();													// called from watchdog task
protected:
	virtual const char* getmodename() const = 0;					// name of this mode
	bool takecontrol();															// take control if possible
	void releasecontrol();														// release control (mandatory)
	void emergencystopactivated(const char* who, const char* why);	// incoming emergency stop
	void joystick_move(float x, float y);									// joystick has moved
	bool setrunmode(bool runmodeon);									// request run mode, may be refused
	bool getrunmode();															// true if in run mode
	bool runstopbuttonsmeaningful() { return(true); }			// true if run button should be visible
	bool runbuttonallowed() { return(true); }							// you can always ask
	virtual bool controlleraccessallowed() = 0; 						// true if allowed to touch controllers directly
	void updatedrive();															// called to keep alive
	void shift_button_pressed(MsgSpeedSet::Gear newgear);
private:
	void clearlastmove();														// clear last move data
	void updatemovedisplay(const MoveServerMsg::MsgMove& msg, const MoveServerMsg::MsgMoveReply& reply);	// process a message
	float calcStoppingDistance(float speed);							// calculate stopping distance for this speed
	void coastingLog(bool coasting);										// coasting log, for stopping distance testing.
};
//
//	AutoModes -- common base for AutoMode and OfflineAutoMode
//
class AutoModes: public DrivingControlMode
{
private:
	ost::Mutex m_lock;																// protect data and port
	bool	m_runmode;																// true if in run mode
	const char* m_mapservernode;										// node on which to look for move server
	const char* m_mapservername;										// name to look for it under
	RemoteMsgClientPort	m_mapserverport;						// port for talking to move server
	MapServerMsg::MsgMapQueryReply m_lastreply;				// last reply received
public:
	AutoModes(Manualdrive& owner, const char* moveservernode, const char* moveservername);
protected:
	virtual const char* getmodename() const = 0;					// name of this mode
	bool takecontrol();															// take control if possible
	void releasecontrol();														// release control (mandatory)
	void emergencystopactivated(const char* who, const char* why);	// incoming emergency stop
	void joystick_move(float x, float y);									// joystick has moved
	bool setrunmode(bool runmodeon);									// request run mode, may be refused
	bool getrunmode();															// true if in run mode
	bool runstopbuttonsmeaningful() { return(true); }			// true if run button should be visible
	bool runbuttonallowed() { return(true); }							// you can always ask
	virtual bool controlleraccessallowed() = 0; 						// true if allowed to touch controllers directly
	void updatedrive();															// called to keep alive
	void shift_button_pressed(MsgSpeedSet::Gear newgear);
protected:
	virtual void updatemovedisplay(const MapServerMsg::MsgMapQueryReply& reply);	// process a message
};
//
//	SemiAutoControlMode -- manual driving, but via the move server
//
class SemiAutoControlMode: public SemiAutoModes
{
public:
	SemiAutoControlMode(Manualdrive& owner);
protected:
	bool controlleraccessallowed() { return(true); }				// true if allowed to touch controllers directly
	const char* getmodename() const;									// name of this mode
};
//
//	AutoControlMode -- true auto mode
//
class AutoControlMode: public AutoModes
{
public:
	AutoControlMode(Manualdrive& owner);
protected:
	bool controlleraccessallowed() { return(true); }				// true if allowed to touch controllers directly
	const char* getmodename() const;									// name of this mode
};
//
//	OfflineSemiAutoControlMode -- manual driving, but via the dummy move and speed servers
//
class OfflineSemiAutoControlMode: public SemiAutoModes
{
public:
	OfflineSemiAutoControlMode(Manualdrive& owner);
protected:
	bool controlleraccessallowed() { return(false); }				// true if allowed to touch controllers directly
	const char* getmodename() const;									// name of this mode
};
//
//	OfflineAutoControlMode -- true auto mode
//
class OfflineAutoControlMode: public AutoModes
{
public:
	OfflineAutoControlMode(Manualdrive& owner);
protected:
	bool controlleraccessallowed() { return(false); }				// true if allowed to touch controllers directly
	const char* getmodename() const;									// name of this mode
	void updatemovedisplay(const MapServerMsg::MsgMapQueryReply& reply);	// process a message
};
#endif // DRIVINGCONTROLMODE_H