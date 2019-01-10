//
//	 File: throttlecontroller.cpp  --  interface to the throttle controller
//
//
//	John Nagle
//	Team Overbot
//	November, 2004
//
#include "throttlecontroller.h"
#include "logprint.h"
#include "tuneable.h"
//
//	Hard constants
//
const float k_connect_timeout = 0.050;										// 50ms to connect UDP
//
//	Tuneable calibration constants.
//
//	These can be overridden via environment parameters if necessary.
//
//	Fields: Name, min, max, default, description.
//	Put "NAME=nnn.nn"  in the environment to override.
const Tuneable k_encoder_scale_factor("ODOMSCALE",0.001, 0.01, 0.0044, "Odometer encoder scale, meters per count");
const Tuneable k_minrpm("RPMOFFSETVOLTS", 0,2, 0.35,"RPM sensor offset, volts");
const Tuneable k_maxrpm("RPMMAXVOLTS", 0, 10, 6.2,"RPM sensor max output, volts");
const Tuneable k_scalerpm("RPMSCALE", 500, 2000, (10*60*2), "RPM sensor, RPM/volt");
const Tuneable k_scalespeed("SPEEDSCALE",1, 10, 5, "Speed sensor, m/sec/volt");		// ***TEMP*** bad calibration
const float k_minspeed = 0.35;						// min volts from converter
////const float k_maxspeed = 6.2;						// max value freq. to volt. can generate (1V = ???)


//
//	ThrottleController -- controller for throttle
//
//	Throttle control is just positional, but there are some other devices tied to that controller.
//	Engine start, engine run, and 6WD are on this controller.
//
//
//	GetOdometer  -- get value for odometer
//
//	Must handle wrap and scaling
//
Controller::Err ThrottleController::getOdometer(double& odometer)
{	int auxencoder;
	Controller::Err err = ActualPosnAuxGetUnscaled(&auxencoder);				// get aux encoder reading
	if (err != Controller::ERR_OK) return(err);													// if fail
	//	Handle encoder value 24-bit wrap around. 
	const int k_max_encoder_range = (1<<24);											// range of encoder - where it wraps
	int encchange = auxencoder - m_lastencoder;										// how much did it change by?
	if (abs(encchange) > (k_max_encoder_range/2))									// if big change, must be wrap
	{	if (encchange > 0)																				// if change from - to +
		{	m_encoderwrap -= k_max_encoder_range; }									// just wrapped going down
		else																										// otherwise
		{	m_encoderwrap += k_max_encoder_range; }									// just wrapped going up
	}
	double encoder = m_encoderwrap + auxencoder;									// corrected encoder value
	m_lastencoder = auxencoder;																	// update for next time
	odometer = k_encoder_scale_factor * encoder;										// convert to meters
	return(Controller::ERR_OK);																		// good encoder reading
}
//
//	getTachometer -- get current tachometer value
//	
//	Returns RPM value.
//
//	Not really very accurate. Value good to maybe 5-10%.  Controller should have a dead band.
//
Controller::Err ThrottleController::getTachometer(float& rpm)	
{	float tachvolts = 0;
	Controller::Err err = AnalogInputGet(1,&tachvolts);									// get analog tachometer voltage
	if (err != Controller::ERR_OK) return(err);													// fails
	tachvolts -= k_minrpm;																				// adjust for offset
	if (tachvolts < 0) tachvolts = 0;																// avoid negative
	rpm = tachvolts * k_scalerpm;																	// scale RPM
	return(Controller::ERR_OK);																		// success
}
//
//	getSpeedometer  -- get radar speedometer
//
//	Returns m/sec. unsigned.
//
//	Not very accurate.  Value good to no better than 10%, plus a constant error.  Subject to
//	interference from flourescent lamps.
//
Controller::Err ThrottleController::getSpeedometer(float& speed)
{
	Controller::Err err = AnalogInputGet(2,&speed);										// get speed from Analog 2
	if (err != Controller::ERR_OK) return(err);													// fails
	//	Rescale speed
	speed -= k_minspeed;																				// remove offset from A/D
	speed = std::max(0.0f,speed);																	// avoid negative value
	speed *= k_scalerpm;																				// scale speed 
	return(Controller::ERR_OK);																		// success
}

//
//	UpdateState -- update state for throttle controller
//
Controller::Err ThrottleController::UpdateState()
{	Controller::Err err  = SimpleController::UpdateState();								// do parent first
	if (err != Controller::ERR_OK) return(err);													// fails
	err = getOdometer(m_odometer);																// update odometer
	if (err != Controller::ERR_OK) return(err);													// fails
	err = getTachometer(m_rpm);																	// update tachometer
	return(err);
}

