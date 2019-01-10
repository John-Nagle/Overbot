//
//	odospeedometer.h  --  Access to odometer and speedometer for FusedNav
//
//	Frank Zhang
//	March, 2005
//
//
#ifndef ODOSPEEDOMETER_H
#define ODOSPEEDOMETER_H
#include "throttlecontroller.h"
#include "timeutil.h"
//
//	TravelData -- returned by access to speedometer and odometer
//
struct TravelData{
    double		odometer;
    float		speedometer;
    uint64_t		time;				// nanoseconds from epoch
}; // end of struct TravelData

//
//  OdoSpeedometer --  just get current odometer and speedometer readings, for standalone apps
//  modified from 'class SimpleOdometer'
//  Frank Zhang, 3/6/2005
//
class OdoSpeedometer: public ThrottleController {
public:
    OdoSpeedometer(): ThrottleController("gcthrottle",true)	// read only access
    {	DatagramMode(true);	}	   	// use UDP
    TravelData getTravelData();			// get current odometer value, speedometer value, and a timestamp (not available yet)
};
//
//	Implementation
//
//
//	getTravelData -- call to get current odometer, speedometer and timestamp reading
//
//	Returns units of meters for odometer reading, meters/sec for speedometer reading,
//	Returns NaNs if controller does not respond.
//
//	Check for NaN using "finite(value)" from math.h
//
inline TravelData OdoSpeedometer::getTravelData()
{	const float k_NaN = _FNan._Float;	// returns NaN if can't get odometer
    Connect();			       		// connect if needed
    TravelData data = {k_NaN, k_NaN, 0};	// default values
    getOdometer(data.odometer);	            	// get odometer value
    getSpeedometer(data.speedometer);		// get speedometer value
    data.time = gettimenowns();			// units of 1ns, update rate 1ms on x86
    return(data);				// return reading in TravelData
}

#endif // ODOSPEEDOMETER_H
