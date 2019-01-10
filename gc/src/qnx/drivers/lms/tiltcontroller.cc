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
const int k_dry_chan = 2;															// Dry power is on channel 2
const int k_wash_chan = 3;														// Wash power is on channel 3
//
//	constructor
//
TiltController::TiltController(const char* name, bool readonly)	// constructor
	: SimpleController(name, readonly)
{	DatagramMode(true);													// use UDP
}

//
//	getTilt  -- get tilt angle from controller
//
//	Returns NaN if cannot get position
//
float TiltController::getTilt()
{	float actual = k_NaN;														// no value yet
	ost::MutexLock lok(m_lock);												// lock
	Connect(1.0);																	// connect if necessary
	Controller::Err stat = GetActual(actual);								// get position in counts
	if (stat != Controller::ERR_OK)											// if can't read
	{	return(k_NaN);		}														// fails
	return(actual*k_encoder_scale_factor);							// return tilt as radians
}
//
//	setTilt  -- command controller to tilt angle
//
Controller::Err TiltController::setTilt(float tilt, float tiltrate)
{	if (tilt > k_forward_limit) tilt = k_forward_limit;					// apply bounds
	if (tilt < k_reverse_limit) tilt = k_reverse_limit;
	if (tiltrate < 0.0001) tiltrate = 0.0001;								// apply bounds
	if (tiltrate > 10) tiltrate = 10;												// the controller also enforces a speed limit
	float goal = tilt / k_encoder_scale_factor;							// get goal in encoder counts
	float speed = tiltrate / k_encoder_scale_factor;					// get speed in encoder counts
	ost::MutexLock lok(m_lock);												// lock
	Connect(1.0);																	// connect if necessary
	Controller::Err err  = SimpleController::UpdateState();		// do parent first
	if (err != Controller::ERR_OK) return(err);							// fails, controller not ready
	if (fabs(goal-GetLocalGoal()) < k_move_tolerance)			// if close enough
	{	return(Controller::ERR_OK);	}										// success
	err = SetSpeed(speed);													// COMMAND CONTROLLER
	if (err != Controller::ERR_OK) return(err);							// fails, controller not ready
	err = SetGoal(goal);															// COMMAND CONTROLLER
	return(err);																		// and return error status
}
//
//	setLidarPower  -- turn LIDAR power on/off
//
//	LIDAR power is controlled by digital output 1
//
Controller::Err TiltController::setLidarPower(bool on)
{	ost::MutexLock lok(m_lock);												// lock
	Connect(1.0);																	// connect if necessary
	Controller::Err err  = SimpleController::UpdateState();		// do parent first
	if (err != Controller::ERR_OK) return(err);							// fails, controller not ready
    err = DigitalOutputSet(k_lidar_chan, on);							// COMMAND CONTROLLER
	if (err != Controller::ERR_OK) return(err);							// fails, controller not ready
	return(err);																		// and return error status
}
//
//	setCameraWash  -- set wash/dry modes
//
//	Controls digital outputs 2 and 3
//
Controller::Err TiltController::setCameraWash(bool wash, bool dry)
{	if (dry) wash = false;														// never both at once
	ost::MutexLock lok(m_lock);												// lock
	Connect(1.0);																	// connect if necessary
	Controller::Err err  = SimpleController::UpdateState();		// do parent first
	if (err != Controller::ERR_OK) return(err);							// fails, controller not ready
    err = DigitalOutputSet(k_wash_chan, wash);					// COMMAND CONTROLLER
	if (err != Controller::ERR_OK) return(err);							// fails, controller not ready
    err = DigitalOutputSet(k_dry_chan, dry);							// COMMAND CONTROLLER
	return(err);																		// and return error status
}