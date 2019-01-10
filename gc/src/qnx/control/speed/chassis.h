//
//	 File: chassis.h  --  interface to the Overbot Polaris Ranger with Galil controllers
//
//
//	John Nagle
//	Team Overbot
//	November, 2004
//
//	
#ifndef CHASSIS_H
#define CHASSIS_H
#include <time.h>
#include "simplecontroller.h"
#include "throttlecontroller.h"
#include "transmissioncontroller.h"
#include "brakecontroller.h"
#include "steeringcontroller.h"
#include "speedservermsg.h"
#include "timedloop.h"

//
//	class Chassis -- all the engine-related controllers
//
class Chassis {
private:
	//	Controllers for hardware
	BrakeController m_brake;													// brake controlller (GOAL is pressure)
	ThrottleController m_throttle;												// throttle controller (GOAL is position)
	TransmissionController m_transmission;							// transmission controller (GOAL is gear)
	SteeringController m_steer;												// steering controller (GOAL is position)
	//	State info
	MsgSpeedSet::State m_state;											// what state we are in
	SimpleLowPassFilter m_speed;											// filtered speed
	SimpleLowPassFilter m_accel;											// filtered acceleration
	double m_prevodometer;													// previous odometer value
	double m_prevspeed;														// previous speed value
	double m_accumspeederror;											// accumulated speed error, for I term
	double m_statetimestamp;												// time of last state change
	double m_updatetimestamp;												// time of last update
	double m_dumptimestamp;												// time of last dump to log
	bool m_hillholder;																// hill holder mode; lock brakes until RPM is up
	bool m_controllersready;													// controllers ready at last update?
	bool m_verbose;																// true if verbose mode
	Fault::Faultcode m_fault;													// current fault code if any
public:
	Chassis(bool readonly=false);											// constructor
	~Chassis();																		// destructor
	void SetVerbose(bool verbose) { m_verbose = verbose; }
	void Update();																	// update incoming info from controllers.
	bool SetState(MsgSpeedSet::State newstate, bool& busy);	// change to new state
	void SetSpeed(float desiredaccel, float desiredspeed, float desiredsteer);	// actually set the speed and steering
	void SetFault(Fault::Faultcode faultid, const char* msg = 0);	// report a fault - causes an emergency stop
	void SetFault(Fault::Faultcode faultid, Controller::Err err);	// report a fault - causes an emergency stop
	bool ResetFault();																// reset fault, returns true if reset allowed.
	bool SetGear(MsgSpeedSet::Gear newgear, bool busy);	// request gear change
	//	Access functions
	MsgSpeedSet::State GetState() const {	return(m_state); }
	MsgSpeedSet::Gear GetGear();
	float GetSpeed();																// get speed in m/sec
	float GetAccel();																// get acceleration in m^2/sec
	float GetCurvature();															// get turning curvature (1/radius in meters, > 0 means right turn)
	int GetDir();																		// -1 reverse, +1 forward, 0 neutral.
	Fault::Faultcode GetFault() const { return(m_fault); }
	MsgSpeedSetReply::Hint GetHint();									// not yet
	double GetUpdateTmestamp() const { return(m_updatetimestamp); }
	bool GetControllersReady() const { return(m_controllersready); }
	bool GetStopped() { return(stopped()); }							// export Stopped
	double GetOdometer();														// get odometer value
	void Dump();																		// dump state (to logprintf)
private:
	void UpdateState();															// update state, called on each update cycle
	Controller::Err UpdateControllersTry(Fault::Faultcode& faultid);	// update local copy of controller state, make sure all are up
	Controller::Err UpdateControllers(Fault::Faultcode& faultid);	// update local copy of controller state, make sure all are up
	void StateChange(MsgSpeedSet::State newstate, const char* msg); // note a state change
	void ControllerSanityCheck();											// check state of recently updated controllers
	float calcThrottle(float desiredspeed);								// calculate throttle setting
	float calcBrake(float desiredaccel);									// calculate brake setting
	float calcSteering(float desiredinvrad);								// convert 1/r to steering goal
	float calcSteeringRadius(float goal);									// convert steering goal to 1/r
	bool SetGoals(float throttlesetting, float brakesetting, float steersetting);		// set moving throttle, brakes, steering
	bool SetGoalsStopped();													// set throttle, brakes for stopped state
	bool controllersready();													// true if controllers are ready
	bool brakeslocked();															// true if brake pressure is above high threshold
	bool brakesreleased();														// true if brake pressure is below low threshold
	bool engineabovehillholdrelease();									// true if engine above hill hold release point
	bool engineaboveidle();													// true if engine RPM is above movement threshold
	bool enginerunning();														// true if engine RPM is above cranking threshold
	bool throttlezero();															// true if throttle is at zero 
	bool notmoving();																// true if not moving according to encoder
	bool stopped();																	// true if brakes locked and engine idle and not moving
	bool indesiredgear();														// true if in desired gear
	bool unpaused();																// true if not estop, not paused, not manual
	bool unpaused(bool& notestop, bool& unpaused, bool& inauto);	// long form
};
#endif // CHASSIS_H
