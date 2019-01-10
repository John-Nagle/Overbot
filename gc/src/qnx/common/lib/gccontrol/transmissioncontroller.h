//
//	 File: transmissioncontroller.h  --  interface to the throttle controller
//
//
//	John Nagle
//	Team Overbot
//	November, 2004
//
//	
#ifndef TRANSMISSIONCONTROLLER_H
#define TRANSMISSIONCONTROLLER_H
#include "simplecontroller.h"
#include "speedservermsg.h"
//
//	TransmissionController -- controller for transmission
//
//	Goal controls gear number, not position, so we need to subclass to read gear number, not position,
//
class TransmissionController: public SimpleController {
public:
	TransmissionController(const char* name, bool readonly)	// constructor
	: SimpleController(name, readonly)
	{}
	virtual ~TransmissionController() {}								// destructor
	Controller::Err GetActual(float& goalactual);						// must be subclassed if goal is not a position
	MsgSpeedSet::Gear GetLocalGear();									// get current gear. 
};
//
//	Implementation -- very simple
//
//	TransmissionController
//
inline Controller::Err TransmissionController::GetActual(float& goalactual)
{
	return(VariableGet("ACTUAL",&goalactual));		//	Actual value for transmission is gear
}
//
//	GetLocalGear  -- get what gear we are in.  Returns gear_unknown if any problem.
//
inline MsgSpeedSet::Gear  TransmissionController::GetLocalGear()
{	int actual = int(GetLocalActual());						// get actual value 	
	if ((!GetLocalAuto())											// if not good to go
		|| (actual != GetLocalGoal())							// or shift in progress
		|| (actual > MsgSpeedSet::gear_high) 			// or out of range
		|| (actual < MsgSpeedSet::gear_unknown))
	{	return(MsgSpeedSet::gear_unknown);		}	// fails
	return(MsgSpeedSet::Gear(actual));				// coerce into enum type for gear
}
#endif // TRANSMISSIONCONTROLLER_H
