//
//	dynamicsutil  -- dynamics utililty functions  
//
//	SAFETY CRITICAL
//
//	John Nagle
//	Team Overbot
//	March, 2005.
//
//
//
//	Copyright 2005 by John Nagle
//	999 Woodland Avenue
//	Menlo Park, CA  94025
//
//	This program is free software; you can redistribute it and/or modify
//	it under the terms of the GNU General Public License as published by
//	the Free Software Foundation; either version 2 of the License, or
//	(at your option) any later version.

//	This program is distributed in the hope that it will be useful,
//	but WITHOUT ANY WARRANTY; without even the implied warranty of
//	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//	GNU General Public License for more details.

//	You should have received a copy of the GNU General Public License
//	along with this program; if not, write to the Free Software
//	Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//
#include "geomutil.h"
#include "algebra3.h"
#include "tuneable.h"
#include "logprint.h"
#include "dynamicsutil.h"
#include "curvedwedge.h"
#include "curvedpath.h"
//
//	Constants
//
const Tuneable k_braking_friction_coefficient("BRAKINGFRICTION", 0.1, 0.75, 0.2, "Friction coefficient for braking (fraction)");
const Tuneable k_lateral_friction_coefficient("LATERALFRICTION", 0.001, 0.2, 0.2, "Friction coefficient for roll prevention (fraction)");
const Tuneable k_reaction_time("REACTIONTIME", 0.1, 1.0, 0.4, "Braking reaction time (seconds)");
const float k_g = 9.80;														// gravity (m/s)
//
//	calcSafeCurvatureLimits  -- calculate what we can command steering to do
//
//	This is where we must handle vehicle dynamics protection.
//
//	DOES NOT USE INFORMATION ABOUT CURRENT VEHICLE ROLL AND NEEDS TO
//
void calcSafeCurvatureLimits(float speed, float pitchradians, float rollradians, float abscurvlimit, float& mincurv, float& maxcurv)
{
	//	First, narrow limits based on speed of vehicle.
	float v = fabs(speed);													// vehicle speed
	if (v > 0.001)																// if nonzero velocity
	{	float rollcurvlimit = (k_g*k_lateral_friction_coefficient) / (v*v);	 // limit to prevent rollover
		abscurvlimit = std::min(rollcurvlimit, abscurvlimit);	// apply limit
	}
	mincurv = -abscurvlimit;												// set base curvature limits
	maxcurv = abscurvlimit;
}
//
//	calcSafeSpeedForCurvature  -- same calculation as above, but with curvature in and speed out
//
//	DOES NOT USE INFORMATION ABOUT CURRENT VEHICLE ROLL AND NEEDS TO
//
float calcSafeSpeedForCurvature(float pitchradians, float rollradians, float curvature)
{
	const float k_big_speed = 100;										// 100 m/sec, huge speed limit if no turn problems
	if (fabs(curvature) < 0.0001)										// if tiny curvature, avoid divide by zero
	{	return(k_big_speed);	}												// no limit here
	float speed = sqrt(k_g*k_lateral_friction_coefficient / fabs(curvature));	// speed limit
	////logprintf("calcSafeSpeedForCurvature: curv %1.4f  speed %1.2f\n", curvature,speed);	// ***TEMP***
	return(std::min(speed, k_big_speed));							// return speed limit based on curvature
}

//
//	calcStoppingDistance -- calculate how close the goal distance can be, given current speed
//
//	This is a minimum distance.
//
void calcStoppingDistance(float speed, float pitchradians, float rollradians, float &dist)
{	const float r = k_reaction_time;					// 400ms reaction time (hopefully conservative)
	const float f = k_braking_friction_coefficient;// conservative coeff of friction
	const float v = fabs(speed);						// input speed
	if (pitchradians > 0) pitchradians = 0;		// derate going downhilll; don't speed up going uphill
	float a = k_g*(sin(pitchradians) + f);		// braking deceleration available
	dist = r*v + (v*v)*0.5/a;							// stopping distance
}
//
//	calcSafeSpeedForDistance  --  how fast can we go given this stopping distance
//
//	The move server also does this calculation.
//
float calcSafeSpeedForDistance(float pitchradians, float dist)
{
	const float r = k_reaction_time;					// 400ms reaction time (hopefully conservative)
	const float f = k_braking_friction_coefficient;	// conservative coeff of friction
	if (pitchradians > 0) pitchradians = 0;		// derate going downhilll; don't speed up going uphill
	float a = k_g*(sin(pitchradians) + f);		// braking deceleration available
	float v = a*(-r + sqrt(r*r  +(2*dist/a)));	// quadratic solution
	v = std::max(v,0.1f);									// allow some forward velocity, no matter what
	return(v);
}
//
//	intightspot -- true if an impingement on both sides, or on either side in a turn
//
bool intightspot(const CurvedPath& path, float prevcurvature, const ImpingementGroup& impingements)
{	const float k_max_straight_curvature = (1.0/50.0);						// less than 50m radius is straight
	bool left = impingements.m_left.m_valid && !impingements.m_left.m_unknown;
	bool right = impingements.m_right.m_valid && !impingements.m_right.m_unknown;
	bool center = impingements.m_center.m_valid && !impingements.m_center.m_unknown;
	bool anyobstacle = left || center || right;										// true if any obstacles at all
	if (!anyobstacle) return(false);														// no problem
	bool tightspot = (center && left) || (center && right) || (left && right);	// true if obstacles on both sides
	if (tightspot) return(true);
	//	Trouble on at least one side.  Are we planning a turn, or in one?
	if (fabs(prevcurvature) > k_max_straight_curvature) return(true);	// was turning
	if (fabs(path.getsteeringcurvature()) > k_max_straight_curvature) return(true); // plan to turn
	return(false);																				// in a narrow place, but going straight. Don't overreact.
}