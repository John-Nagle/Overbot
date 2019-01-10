//
//	tiltcontroller.h  -- LIDAR server tilt control, DUMMY OFFLINE TEST VERSION
//
//	John Nagle
//	Team Overbot
//	December, 2004
//
#ifndef TILTCONTROLLER_H
#define TILTCONTROLLER_H
//
#include "mutexlock.h"
#include "simplecontroller.h"
//
//	class TiltController  -- controller for tilt servo
//
class TiltController {
	ost::Mutex m_lock;													// only one thread at a time can access
public:
	TiltController(const char* name, bool readonly);	// constructor
public:
	float getTilt();														// get tilt angle from controller (radians)
	float updateTilt();													// get and update tilt; called for each scan line
	Controller::Err setTilt(float tilt, float tiltrate);			// set tilt angle (radians, 0 is down, pi/2 is forward)
	Controller::Err setLidarPower(bool on);					// turn LIDAR power on/off
	Controller::Err setCameraWash(bool wash, bool dry);	// wash control
private:
	float	m_dummytilt;													// dummy tilt position
	float m_dummyspeed;											// tilt controller speed
	float 	m_dummygoal;												// tilt controller dummy goal
};
#endif // TILTCONTROLLER_H