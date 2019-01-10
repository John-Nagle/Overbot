//
//	washcontrol.cc  --  runs a LIDAR and camera wash cycle
//
//	John Nagle
//	Team Overbot
//	December, 2004
//
#include "tiltcontroller.h"
#include "washcontrol.h"
#include "logprint.h"
#include "tuneable.h"
//
//	Configuration 
//
const Tuneable k_wash_cycles("WASHCYCLES", 1, 5, 1, "Length of wash cycle, cycles");
const Tuneable k_dry_cycles("DRYCYCLES", 1, 5, 4, "Length of dry cycle, cycles");
//
//	Constructor
//
WashControl::WashControl(TiltController& tilt)
	: m_tilt(tilt), m_init(false), m_washing(false)
{
}
//
//	washing -- true if washing
//
bool WashControl::washing() const
{	
	return(m_washing);															// true if washing
}
//
//	requestWash  -- request a wash cycle
//
bool WashControl::requestWash()
{	ost::MutexLock lok(m_lock);												// lock
	if (!m_init)
	{	pthread_create(NULL, NULL, &washThreadStart, this);	// start the static that starts the thread in the object
		m_init = true;																// now initialized
	}
	if (washing()) return(false);												// already washing, ignore
	m_washing = true;															// set washing
	m_sem.post();																	// and wake up wash system
	return(true);																		// wash started
}
//
//	washThread -- actually does the washing
//
//	Posting to the semaphore starts a wash cycle.
//
void* WashControl::washThread()
{
	for (;;)
	{	m_tilt.setCameraWash(false,false)	;								// turn off washer, just in case
		m_sem.wait();																// wait for a wash request
		m_washing = true;														// wash in progress
		logprintf("Beginning camera/LIDAR wash cycle.\n");
		doWash();																		// do it
		logprintf("Finished with camera/LIDAR wash cycle.\n");
		while (m_sem.trywait());												// consume any extra wash requests
		m_washing = false;
	}
}
//
//	doWash  -- do one wash cycle
//
//	No one else should use the tilt head while washing is in progress
//
bool WashControl::doWash()
{
	const int k_washwigglecycles = 1;									// number of wiggle cycles
	const int k_drywigglecycles = 2;										// number of wiggle cycles
	//	Move LIDAR to wash position
	bool good = doTilt(0);														// move to wash position
	if (!good) return(false);														// fails
	//	Prewash air blow to remove dirt
	Controller::Err err = m_tilt.setCameraWash(false,true);		// turn on air blow
	if (err != Controller::ERR_OK) return(false);						// trouble
	good = doWiggle(k_dry_cycles);										// wiggle head
	if (!good) return(false);														// fails
	err = m_tilt.setCameraWash(false,false);							// turn off air
	if (err != Controller::ERR_OK) return(false);						// trouble
	sleep(1);																			// allow dirt to fall off
	//	Wash cycle
	err = m_tilt.setCameraWash(true,false);							// air off, washer on
	if (err != Controller::ERR_OK) return(false);						// trouble
	good = doWiggle(k_wash_cycles);										// wiggle head
	if (!good) return(false);														// fails
	err = m_tilt.setCameraWash(false,false);							// end wash cycle
	if (err != Controller::ERR_OK) return(false);						// trouble
	sleep(1);																			// allow pump to stop
	//	Dry cycle
	err = m_tilt.setCameraWash(false,true);							// washer off, air on
	if (err != Controller::ERR_OK) return(false);						// trouble
	good = doWiggle(k_dry_cycles);										// wiggle head
	if (!good) return(false);														// fails
	err = m_tilt.setCameraWash(false,false);							// turn off air
	if (err != Controller::ERR_OK) return(false);						// trouble
	return(true);																		// success
}
//
//	doTilt  -- go to desired tilt, waiting.
//
bool WashControl::doTilt(float tilt, float tiltspeed)
{
	Controller::Err err = m_tilt.setTilt(tilt, tiltspeed);					// move to indicated position
	if (err != Controller::ERR_OK)												// if problem
	{	logprintf("Tilt error during wash cycle: %s\n", Controller::ErrMsg(err));	// trouble
		return(false);																// fails
	}
	for (int i=0; i<20; i++)														// allow max time for tilt
	{	sleep(1);
		float newtilt = m_tilt.getTilt();											// get current position
		if (fabs(newtilt - tilt) < 0.01)											// if arrived
		{	return(true); }
	}
	logprintf("LIDAR tilt failed during wash cycle.\n");
	return(false);																	// failed
}
//
//	doWiggle -- wiggle LIDAR head near wash position
//
//	Allows wash head to cover entire area
//
bool WashControl::doWiggle(int wigglecycles)
{
	const float k_wiggle_pos = (M_PI/180.0)*5;						// +- this many degrees for wiggle
	const float k_wiggle_speed = 0.2;										// speed for wiggle (rad/sec)
	for (int i=0; i< wigglecycles; i++)										// do wiggles
	{	bool good = doTilt(- k_wiggle_pos, k_wiggle_speed);	// one way
		if (!good) return(false);													// other way
		good = doTilt(k_wiggle_pos, k_wiggle_speed);
		if (!good) return(false);
	}
	bool good = doTilt(0, k_wiggle_speed);							// return to parked position
	if (!good) return(false);
	return(true);
}
