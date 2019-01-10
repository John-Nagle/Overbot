//
//	scurvepath.cpp  -- two-pass process over code in splinepath.h to approximately account for finite steering speed
//
//
//
//	Copyright 2005 by Ron Avitzur
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
#include "scurvepath.h"													// used in implementation
#include "logprint.h"

//
//	class SCurvesPath  -- three circular arcs where the initial one is chosen to account for
//   steering motor latency, steering motor finite speed, and initial steering, and following
//   two arcs are chosen to reach the goal point.


// Implementation
//
// The only difference is in setting up the inputs to SplinePath here
//
//	setstart  -- set start of path
//
void SCurvePath::setstart(
	const vec2& pos,
	const vec2& forward,
	const float speed,
	const float curvature, 
	const float initialcurvaturedist, 
	const float mincurvature, 
	const float maxcurvature, 
	const float maxcurvaturerate,
	const float lookaheaddist) 
	{
	// Save these inputs for the second pass done in set end.
	
	this->start								= pos;
	this->start_heading					= forward;
	this->start_speed						= speed;
	this->start_curvature				= curvature;
	this->start_curvature_distance	= initialcurvaturedist;
	this->start_min_curvature			= mincurvature;
	this->start_max_curvature			= maxcurvature;
	this->start_max_curvature_rate	= maxcurvaturerate;
	this->start_lookahead_distance = lookaheaddist;

	path.setstart(pos, forward, speed, curvature, initialcurvaturedist, mincurvature, maxcurvature, maxcurvaturerate, lookaheaddist);
	}
//
//	setend  -- set end of path
//
bool SCurvePath::setend(const vec2& pos, const vec2& forward)
	{
/*
0. Reset stored path per our own setstart parameters.
1. Run the existing algorithm
2. Compute the desired end curvature as above.
3. Compute how long it will take to get to that curvature based on the travel time of the steering motor and the difference in steering
4. Compute a new initial arc with a curvature between the initial and final and a length representing the travel time
5. Repeat the S-Curve computation with this new initial arc representing the slow steering movement
*/
// Step 0: get path back to initial state as of previous setstart
	path.setstart(start, start_heading, start_speed, start_curvature, start_curvature_distance,
		start_min_curvature, start_max_curvature,  start_max_curvature_rate, start_lookahead_distance);

// Step 1. Proceed as before calling SplinePath::setend
	bool works = path.setend(pos, forward);
	if (!works) return(false);															// must fail if can't even set basic path


// Step 2. See what initial curvature this expects
	float desired_curvature;
	bool good = path.steertopath(desired_curvature, start_lookahead_distance);
	if (!good) return(false);																// bogus steer to path - unlikely
	// need to duplicate logic in NewSteer::steerToPath(const CurvedPath& path, float& curvature), but that information is not available here
	
// Step 3.
	// if max_curvature_rate is the steering change per unit distance, this should give the distance needed to change the curvature that amount
	double curvchange = abs(desired_curvature - start_curvature);	// desired curvature change
	double new_curvature_distance = curvchange * start_speed / start_max_curvature_rate;	 // distance required to make this change
	////double new_curvature_distance = abs((desired_curvature - start_curvature) / start_max_curvature_rate);

// Step 4.
	const double kInitialCurvatureWeight = 0.8;
	double second_pass_initial_curvature = start_curvature * kInitialCurvatureWeight + desired_curvature * (1-kInitialCurvatureWeight);

// Step 5.
	path.setstart(start, start_heading, start_speed, second_pass_initial_curvature, new_curvature_distance,
		start_min_curvature, start_max_curvature,  start_max_curvature_rate, start_lookahead_distance);

	return path.setend(pos, forward);
	}
	
//	
//	dump -- dump key params to log file, with message
//
void SCurvePath::dump(const char* msg) const
{	path.dump(msg);																// splinepath does all the work
}
