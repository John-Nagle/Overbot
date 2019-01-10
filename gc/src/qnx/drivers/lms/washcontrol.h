//
//	washcontrol.h  --  runs a LIDAR and camera wash cycle
//
//	John Nagle
//	Team Overbot
//	December, 2004
//
#ifndef WASHCONTROL_H
#define WASHCONTROL_H
#include "mutexlock.h"

class TiltController;
//
//	class WashControl  -- controls washing
//
class WashControl
{
private:
	TiltController&	m_tilt;										// associated tilt controller
	ost::Mutex m_lock;												// lock
	ost::Semaphore m_sem;									// post to initiate washing
	bool m_init;														// initialized thread
	bool m_washing;												// true if in wash cycle
public:
	WashControl(TiltController& tilt);
	bool washing() const;										// true if wash cycle in progress
	bool requestWash();											// request a wash cycle if not already in progress
private:
	bool doWash();													// do one wash
	bool doWiggle(int cycles);									// wiggle and wait
	bool doTilt(float tilt, float tiltspeed = 99);			// tilt and wait
    void* washThread();										// actually does the washing
    static void* washThreadStart(void* arg)			// idiom for starting a thread within an object
	{ return(reinterpret_cast<WashControl*>(arg)->washThread()); }
};
#endif // #define WASHCONTROL_H
