//
//	 File: brakecontroller.h  --  interface to the brake controller
//
//
//	John Nagle
//	Team Overbot
//	November, 2004
//
//	
#ifndef BRAKECONTROLLER_H
#define BRAKECONTROLLER_H
#include "simplecontroller.h"
//
//	BrakeController -- controller for brake
//
//	Brake controls pressure, not position, so we need to subclass to read pressure, not position,
//
class BrakeController: public SimpleController {
private:
	bool m_watchdog_toggle;													// used for resetting watchdog
public:
	BrakeController(const char* name, bool readonly = false)
	: SimpleController(name, readonly),
	m_watchdog_toggle(false)
	{}
	Controller::Err ResetWatchdog();										// reset the watchdog
	virtual ~BrakeController() {}
	Controller::Err GetActual(float& goalactual);						// must be subclassed if goal is not a position
};
//
//	Implementation  -- very simple
//
//	BrakeController
//
inline Controller::Err BrakeController::GetActual(float& goalactual)
{
	return(AnalogInputGet(1,&goalactual));		//	Actual value for brake is pressure
}
//
//	ResetWatchdog  -- reset hardware watchdog
//
//	Must do this every 100ms or the throttle will drop and the brakes will lock.
//
inline Controller::Err BrakeController::ResetWatchdog()
{
	const int k_watchdogDigitalOut = 1;					// watchdog is on channel 1
	m_watchdog_toggle = !m_watchdog_toggle;	// invert toggle
	return(DigitalOutputSet(k_watchdogDigitalOut, m_watchdog_toggle ? 1 : 0));	// set toggle
}
#endif // BRAKECONTROLLER_H
