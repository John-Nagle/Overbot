//
//	generatespeed -- generate speed command, given move command
//
//	This does the real work of the move server
//
//	John Nagle
//	Team Overbot
//	January, 2004
//
#include <stdlib.h>
#include <unistd.h>
#include <algorithm>
#include "logprint.h"
#include "moveserver.h"
#include "tuneable.h"
#include "timeutil.h"
//
//    Description: (by Achut Reddy)
//
//	Takes high-level move commands from mapserver and implements them
//	by translating them into low-level commands to the vehicle actuator
//	servers (steering, throttle, brake, trasmission).
//
//      Performs sanity checks on the move commands.  It contains a simple
//      dynamics model and will refuse to move in any manner which threatens
//      vehicle safety.  If necessary it will reduce the specified speed until
//      it is deemed safe.
//
//      Also checks for potential collisions by querying the vorad server;
//	if a collision appears imminent, it will halt the vehicle in spite
//	of any move commands.
//
inline float deg2radians(float x) { return(x*(M_PI/180.0)); }								// degrees to radians
//
//	Keeps track of predicted position vs. actual position error so the
//	algorithm can adapt and adjust for it.
//
//	Tuneable constants
//
const Tuneable k_friction_coefficient("FRICTION", 0.1, 0.7, 0.25, "Friction coefficient for slip prevention (fraction)");
const Tuneable k_reaction_time("REACTIONTIME", 0.1, 1.0, 0.4, "Braking reaction time (seconds)");
const Tuneable k_max_safe_vibration("VIBRATIONLIM", 0.05, 0.6, 0.25, "Allowed maximum vibration (g)");
const Tuneable k_max_speed("SPEEDLIM",0.01, 18, 4.5, "Speed limit (m/s)");	// default is rather slow
const Tuneable k_min_speed("SPEEDMIN",0.01, 2, 0.25, "Speed minimum (m/s)");	// drop to this speed in real trouble
const Tuneable k_max_roll_deg("ROLLLIM",5,30,15,"Maximum allowed roll (deg)");	// 15 degrees per Polaris manual
const Tuneable k_vibration_filter_constant("VIBRATIONFILTER",0.005, 1.0, 0.02, "Filter constant for vibration (fract)");	// 15 degrees per Polaris manual
const float k_max_roll = deg2radians(k_max_roll_deg);								// in radians
//
//	Constants
//
const float k_g = 9.80;															// acceleration of gravity, m/s.
//
//	safeSpeedLimit  -- configured speed limit
//
float MoveServer::safeSpeedLimit() const
{	return(k_max_speed);	}

//
// compute maximum safe speed given the distance to target
//
// stopping distance is defined as:
//
//	d = v*t + v^2/2*a
//
//	where,
//		v = speed,
//		t = reaction time
//		a = g * sin(theta) * f
//			g = braking deceleration
//			theta = tilt angle
//			f = coefficient of friction
//
// Solving for v yields:
//
//	v = sqrt((a*a*t*t) + 2*a*d) - a*t
//
//
//	safeSpeedForDistance -- stopping distance check
//
//	Understands about going uphill and downhill, but NOT about curvature.
//
//	Pitch is positive going uphill.
//
//	Derivation:
//
//	dist = 0.5*v*v/a + v*r								// stopping distance given velocity, accel, reaction time.
//	Solve for v:
//	(0.5/a)*v*v + r*v - dist = 0						// quadratic
//	Solution by quadratic formula:
//	v = (-r += sqrt(r*r - 4*(0.5/a)(-dist)) / 2*(0.5/a))
//	v = (-r += sqrt(r*r  +(2/a)(dist)) / (1/a))
//	v = a*(-r +- sqrt(r*r  +(2*dist/a)))
//
float MoveServer::safeSpeedForDistance(float dist, float pitchradians)
{	const float r = k_reaction_time;					// 400ms reaction time (hopefully conservative)
	const float f = k_friction_coefficient;			// conservative coeff of friction
	if (pitchradians > 0) pitchradians = 0;		// derate going downhilll; don't speed up going uphill
	float a = k_g*(sin(pitchradians) + f);		// braking deceleration available
	////float v = sqrt((a*a*r*r) + 2*a*dist) - a*r;	// allowed maximum velocity
	float v = a*(-r + sqrt(r*r  +(2*dist/a)));	// quadratic solution
	v = std::max(v,0.1f);									// allow some forward velocity, no matter what
	return(v);
}
//
//	safeSpeedForCurvature  -- allowed safe speed based on curvature
//
//	Based on friction coefficient
//
//	Roll is positive rolling to the right. So, when making a right turn, positive roll is good,
//	and negative roll is bad.
//
//	Centrifugal acceleration is v*v/r
//	So 
//		Ac = v*v/r
//		Ag = g*sin(rollradians);
//	We want v such that
//		Ac+Ag = g*k_friction_coefficient
//	So
//		v*v/r + g*sin(rollradians) = g*k_friction_coefficient
//		v*v  = r*g*(k_friction_coefficient - sin(rollradians))
//		v = sqrt(r*g*(k_friction_coefficient - sin(rollradians)));
//
//	***NEEDS WORK***
//
//	1. We can tell if we are near to rollover, but do not, yet, steer out of it.
//	2. Excessive roll can be fatal, because when we restart, we will get it again.
//	3. We need a roll rate check.
//
float MoveServer::safeSpeedForCurvature(float curvature, float rollradians)
{
	if (abs(rollradians) > k_max_roll)								// if excessive roll
	{	setFault(Fault::roll);												// note roll fault
		logprintf("Excessive roll (%3.2f deg.), near to rolling over.\n", rollradians*180/M_PI); 	
		return(k_min_speed);
	}
	if (abs(curvature) < 0.001) return(k_max_speed);	// max speed in a straight line
	float r = 1/curvature;												// get radius
	float vsq = abs(r*k_g*(k_friction_coefficient));			// max velocity, squared
	if (vsq < 0)																// if below zero,we are tilted too much
	{	logprintf("Excessive roll (%3.2f deg.), possibility of skid.\n", rollradians*180/M_PI); 	
		setFault(Fault::roll);												// note roll fault
		return(k_min_speed);											// near to rollover
	}
	return(sqrt(vsq));														// return max allowed vel.
}
//
//	safeSpeedForRoughness  -- if too rough, a little slower than we are going
//
float MoveServer::safeSpeedForRoughness(float lastspeed, float vibration)
{	const float maxvib = (k_max_safe_vibration*k_g);	// vibration acceleration limit
	if (vibration < maxvib) return(k_max_speed);			// OK, low vibration
	return(lastspeed*0.8);												// too much vibration, slow down
}
//
//	Get pitch, roll, and vibration from last GPS/INS reading
//
float MoveServer::getPitch()
{
	return(deg2radians(m_lastgpsinsreply.rpy[1]));			// get pitch in radians
}
float MoveServer::getRoll()
{
	return(deg2radians(m_lastgpsinsreply.rpy[0]));			// get roll in radians
}
inline float sqr(float x) { return(x*x); }									// square
float MoveServer::getVibration()
{	// acceleration
	double acc = sqrt(sqr(m_lastgpsinsreply.acc[0]) + sqr(m_lastgpsinsreply.acc[1]) + sqr(m_lastgpsinsreply.acc[2]));
	//	Apply low-pass filter to vibration
	m_avgvibration = k_vibration_filter_constant * acc + (1 - k_vibration_filter_constant)*m_avgvibration;
	return(m_avgvibration);													// acceleration, m/sec^2
}
//	
//	getINSValid  -- is INS data valid?
//
bool MoveServer::getINSValid()
{	return(m_lastgpsinsreply.err == GPSINS_MSG::OK);
}
//
//	log -- create log entry
//
void MoveServer::log()	
{	const double k_log_interval = 1.0;							// log once per second
	uint64_t now = gettimenowns();								// time now
	if (now - m_lastlogtime < uint64_t(k_log_interval * 1000000000)) return;		// not time to log yet
	m_lastlogtime = now;												// time to log
	if (!getINSValid())														// note not initialized
	{	logprintf("GPS/INS data not valid.\n");	
		return;
	}
	//	Log useful dynamics info.
	logprintf("Pitch %1.0f deg, roll %1.0f deg, vibration %1.2f m/secsq.\n",
		getPitch(), getRoll(), getVibration());
}


