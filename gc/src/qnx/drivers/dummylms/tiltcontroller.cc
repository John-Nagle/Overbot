//
//	tiltcontroller.cc  -- LIDAR server tilt control
//
//	John Nagle
//	Team Overbot
//	December, 2004
//
#include "tiltcontroller.h"
#include "tuneable.h"
//
//	For the tilt head, zero is straight down, and PI/2 is straight ahead.
//
//	Constants
//
const float k_NaN = _FNan._Float;									// quiet NaN value for floats					

const int k_encoder_counts_per_rev = 288000;											// one full circle
const double k_encoder_scale_factor = (M_PI*2)/k_encoder_counts_per_rev;	// multiply encoder by this to get radians
const Tuneable k_forward_limit("TILTFLIMIT",0.00, 90000*M_PI, 90000*M_PI, "Tilt forward limit, radians.");
const Tuneable k_reverse_limit("TILTRLIMIT",-90000*M_PI, 0, -90000*M_PI, "Tilt reverse limit, radians.");
const Tuneable k_move_tolerance("TILTTOL",0, 100, 10, "Tilt tolerance, encoder counts.");
//
const int k_lidar_chan = 1;														// LIDAR power is on channel 1
const int k_wash_chan = 2;														// Wash power is on channel 2
const int k_dry_chan = 3;															// Dry power is on channel 3
//
//	dummy stuff
//
const float k_max_tiltrate = (40.0)*(M_PI/180);							// max possible tilt rate, radians/sec
//
//	constructor
//
TiltController::TiltController(const char* name, bool readonly)	// constructor
: m_dummytilt(0), m_dummyspeed(0), m_dummygoal(0)
{	
}

//
//	getTilt  -- get tilt angle from controller
//
//	Returns NaN if cannot get position
//
float TiltController::getTilt()
{	ost::MutexLock lok(m_lock);												// lock
	return(m_dummytilt);															// return dummy value
}
//
//	updateTilt  -- update tilt angle in controller
//
//	Returns NaN if cannot get position
//
//	Called for each scan line, and updates the angle
//	
//	We assume that getTilt is called at 75Hz.
//
float TiltController::updateTilt()
{	ost::MutexLock lok(m_lock);												// lock
	float err = m_dummygoal - m_dummytilt;							// desired error
	float incr = m_dummyspeed / 75.0;									// move dist in one line time
	if (fabs(err) <= incr*1.5)													// if very near goal
	{	m_dummytilt = m_dummygoal;	}									// set to goal
	else
	{	float chg = (err > 0) ? incr  : -incr;								// which way to change
		m_dummytilt += chg;													// change simulated position
	}
	////logprintf("Tilt: %1.1f deg, speed %1.2f deg/sec.\n", m_dummytilt*(180/M_PI), m_dummyspeed*(180/M_PI)); 
	return(m_dummytilt);															// return dummy value
}
//
//	setTilt  -- command controller to tilt angle
//
Controller::Err TiltController::setTilt(float tilt, float tiltrate)
{	if (tilt > k_forward_limit) tilt = k_forward_limit;					// apply bounds
	if (tilt < k_reverse_limit) tilt = k_reverse_limit;
	if (tiltrate < 0.0001) tiltrate = 0.0001;								// apply bounds
	if (tiltrate > k_max_tiltrate) tiltrate = k_max_tiltrate;			// the controller also enforces a speed limit
	ost::MutexLock lok(m_lock);												// lock
	m_dummygoal = tilt;
	m_dummyspeed = tiltrate;
	return(Controller::ERR_OK);												// and return error status
}
//
//	setLidarPower  -- turn LIDAR power on/off
//
//	LIDAR power is controlled by digital output 1
//
Controller::Err TiltController::setLidarPower(bool on)
{	ost::MutexLock lok(m_lock);												// lock
	return(Controller::ERR_OK);												// and return error status
}