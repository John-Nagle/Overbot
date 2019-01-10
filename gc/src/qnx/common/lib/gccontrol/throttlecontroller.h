//
//	 File: throttlecontroller.h  --  interface to the throttle controller
//
//
//	John Nagle
//	Team Overbot
//	November, 2004
//
//	
#ifndef THROTTLECONTROLLER_H
#define THROTTLECONTROLLER_H
#include <time.h>
#include "simplecontroller.h"

//
//	ThrottleController -- controller for throttle
//
//	Throttle control is just positional, but there are some other devices tied to that controller.
//	Engine start, engine run, and 6WD are on this controller.
//
class ThrottleController: public SimpleController {
private:
	int m_lastencoder;															// zero-value of odometer, for handling wrap
	double m_encoderwrap;													// adjustment factor for wrap
	float m_rpm;																		// last RPM value
	double m_odometer;															// last odometer value
public:
	ThrottleController(const char* name, bool readonly)		// constructor
	: SimpleController(name, readonly),
		m_lastencoder(0),															// initial value
		m_encoderwrap(0)														// adjustment value for wrap
	{}
	virtual ~ThrottleController() {}											// destructor
	Controller::Err UpdateState();											// update the local info
	//	Access functions
	double getLocalOdometer() const
	{	return(m_odometer); }
	double getLocalRPM() const
	{	return(m_rpm); }
	//
protected:
	Controller::Err getOdometer(double& odometer);				// get odometer, units of meters
	Controller::Err getTachometer(float& rpm);						// get tachometer, units of RPM
	Controller::Err getSpeedometer(float& speed);					// get speed, units of meters/second.
	Controller::Err getEngineRun(bool& on);							// get run relay state
	Controller::Err getEngineStart(bool& on);							// get start relay state
	Controller::Err get6WD(bool& on);										// get 6WD relay state
	Controller::Err setEngineRun(bool on);								// get run relay state
	Controller::Err setEngineStart(bool on);								// get start relay state
	Controller::Err set6WD(bool on);										// get 6WD relay state
	
};
//
//	SimpleOdometer --  just get odometer value, for standalone apps
//
class SimpleOdometer: public ThrottleController {
public:
	SimpleOdometer()
	: ThrottleController("gcthrottle",true)															// read only access
	{	DatagramMode(true);	}																		// use UDP
	double getOdometer();																				// get current odometer value
};
//
//	Implementation
//
//
//	getOdometer -- call to get odometer reading
//
//	Returns units of meters.
//	Returns a NaN if controller does not respond.
//
//	Check for NaN using "finite(value)" from math.h
//	
inline double SimpleOdometer::getOdometer()
{	const float k_NaN = _FNan._Float;																// returns NaN if can't get odometer
	Connect();																									// connect if needed
	double odom = k_NaN;
	Controller::Err err = ThrottleController::getOdometer(odom);						// get odometer value
	if (err != Controller::ERR_OK) return(k_NaN);												// fails
	return(odom);																							// return odometer reading
}
#endif // THROTTLECONTROLLER_H
