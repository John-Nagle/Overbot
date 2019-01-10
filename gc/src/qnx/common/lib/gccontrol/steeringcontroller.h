//
//	 File: steeringcontroller.h  --  interface to the steering controller
//
//
//	John Nagle
//	Team Overbot
//	November, 2004
//
//	
#ifndef STEERINGCONTROLLER_H
#define STEERINGCONTROLLER_H
#include "simplecontroller.h"
//
//	SteeringController -- controller for steering
//
//	No need to do much.
//
class SteeringController: public SimpleController {
public:
	SteeringController(const char* name, bool readonly)	// constructor
	: SimpleController(name, readonly)
	{}
	virtual ~SteeringController() {}								// destructor
};
#endif // STEERINGCONTROLLER_H
