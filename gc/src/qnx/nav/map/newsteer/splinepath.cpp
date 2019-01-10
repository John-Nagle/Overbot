//
//	splinepath.cpp  -- not spline paths at all, but curved paths composed of several arcs
//
//	Copyright 2005 by Ron Avitzur
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
#include "splinepath.h"

static double sgn(double x)
	{
	if (x > 0) return 1;
	else if (x < 0) return -1;
	else return x; // this will either return 0, or propagate a NaN
	}

inline double angle(vec2 a)
	{
	return atan2(a[1], a[0]);
	}

bool SplinePath::setend(const vec2& pos, const vec2& forward)
	{
	m_valid = false;					// assume not valid
	goal = pos;
	goal_heading = forward;
	goal_heading.normalize();

// The path equations are simpler in a simplified coordinate system
// where transformed_start = (0,0) and transformed_heading = {1,0)
// so we'll transform the inputs into the simplified coordinate system

	RotationMatrix m(start_heading[0], start_heading[1]);
	vec2 transformed_goal = m * (goal - start);
	vec2 transformed_goal_heading = m * goal_heading;

// k is the angle from the goal to the center of the arc ending at the goal
// so that the goal is reached in the desired direction goal_heading.
// Since the direction at the goal is tangent to the circle, and the 
// center of the circle is directed normal to the circle, k is pi/2,
// 90 degrees off of the goal_heading

	double k = angle(transformed_goal_heading) + M_PI_2;
	double cos_k = cos(k);
	double sin_k = sin(k);

// We now solve the geometric constraint equations from two circles of equal radius, R.
// One is centered at (0,R) and passes through (0,0) with a heading of (1,0). 
// R may be negative, so the initial circle has either its maximum or its
// minimum at transformed_start = (0,0) in the simplified coordinate system.
// The second arc passes through transformed_goal in the direction of 
// transformed_goal_heading. It is centered at transformed_goal - R*(cos(k), sin(k)).
// The radius of the circles is obtained by noting that the distance between
// their centers will be 2*R if they just touch. The expressions for their
// centers also depend on R, resulting in a quadratic for R unless the
// goal heading is in the same direction as the initial heading

	if (transformed_goal_heading[0] > 0.9999)
		arc_radius = transformed_goal.length2() / (4 * transformed_goal[1]);
	else {
		double a = 1 - sin_k;
		double b = cos_k * transformed_goal[0] + (sin_k + 1) * transformed_goal[1];
		double c = - 0.5 * transformed_goal.length2();
		double d = sqrt(b*b - 4*a*c);
		double r1 = (-b + d) / (2*a);
		double r2 = (-b - d) / (2*a);
		
		// choose the two-arc solution with smaller (unsigned) radius
		arc_radius = (abs(r1) < abs(r2)) ? r1 : r2;
		}

	double R = arc_radius;

	transformed_center_1 = vec2(0, R);
	transformed_center_2 = transformed_goal - R * vec2(cos_k, sin_k);

	double A = angle(transformed_center_2 - vec2(0,R)) + M_PI_2 * sgn(R);

	angular_distance_1 = A + M_PI * sgn(R) * (1 - sgn(A*R));
	transformed_arc2_start = angular_distance_1 + M_PI_2;
	angular_distance_2 = k - transformed_arc2_start;

	// The angular distances are scalar, the same in both coordinate systems.
	// transformed_arc2_start is a direction and is only useful in the simplified coordinate system
	// It is only used when constructing the path (in the simplified system). After we have the 
	// point in the simplified system, that point is transformed to the real coordinate system.
	// transformed_arc1_start = - pi/2 * sgn(R), by construction. 

	// Although the two-arc path this constructs may be valid, there are cases where the
	// shape of the path is a question-mark or figure eight, and iterating this solution
	// while following that shape results in oscillations between two different paths
	// such that the vehicle would behave like the proverbial donkey midway between 
	// two bales of hay. In these cases, we will just fail.
	
	if ((abs(angular_distance_1) > M_PI || abs(angular_distance_2) > M_PI) && finite(R) && abs(R) <= BIG_RADIUS)
		return false;
	
	// the maxcurvaturerate constraint is almost always violated, so we ignore that one.
	// otherwise just check that the radius of curvature of the arc is withing the current turning radius
	m_valid = abs(1 / R) <= abs(max_curvature);					// final validity check
	return(m_valid);
	}


bool SplinePath::pointalongpath(const float distance_along_path, vec2& pos, vec2& forward) const	// get point on path, measuring from start
	{
	if (!m_valid) return(false);															// not valid path, fails
	double dist = distance_along_path; // we'll need to adjust this later

	// Why does this return a boolean? Should this assert or return false for an invalid distance?
	if (! (0 <= dist && dist <= 1.1 * this->getlength()))
		return false;
	
	// Handle the 0th arc which accounts for steering servo latency
	if (dist < path_length_0) {
		if (!finite(arc_radius_0) || abs(arc_radius_0) > BIG_RADIUS) {
			// straight line case for initial steering-servo-latency arc
			pos = actual_start + dist * actual_start_heading;
			forward = actual_start_heading;
			}
		else {
			double theta = curvature_0 * dist;
			double cos_theta = cos(theta);
			double sin_theta = sin(theta);
			RotationMatrix m(actual_start_heading[0], -actual_start_heading[1]);
			pos = center_0 - arc_radius_0 * vec2(m * vec2(-sin_theta, cos_theta));
			forward = m * vec2(cos_theta, sin_theta);
			}
		return true;
		}
	else
		dist -= path_length_0;
	

	// The Graphing Calculator file demonstrating this construction uses 0 <= t <= 1 for each arc segment.
	// To use those equations, we determine from dist on which segment and how far along it we are.
	
	double R = arc_radius;
	
	if (!finite(R) || abs(R) > BIG_RADIUS) { // straight line path
		// We should reach here only if the initial heading is nearly the goal heading
		// and the initial heading points to the goal.  This deals with the R = INF case,
		// and should also be avoid numeric overflows when R is finite but ridiculously large
		// if the input data is very close to the R = INF straight line case, but not quite exactly there
		// Since start_heading is already normalized the position dist along a straight line is:
		pos = start + dist * start_heading;
		forward = start_heading;
		}
	else {
		double distance_along_arc_1 = abs(R * angular_distance_1);
		double distance_along_arc_2 = abs(R * angular_distance_2);

		vec2 transformed_p, transformed_direction;

		if (dist < distance_along_arc_1 || angular_distance_2 == 0) {
			double t = dist / distance_along_arc_1;
			double theta = angular_distance_1 * t - M_PI_2;
			double cos_theta = cos(theta);
			double sin_theta = sin(theta);
			transformed_p = transformed_center_1 + R * vec2(cos_theta, sin_theta);
			transformed_direction = sgn(R) * sgn(angular_distance_1) * vec2(- sin_theta, cos_theta);
			}
		else {
			double dist_2 = dist - distance_along_arc_1;
			double t = dist_2 / distance_along_arc_2;
			double theta = angular_distance_2 * t + transformed_arc2_start;
			double cos_theta = cos(theta);
			double sin_theta = sin(theta);
			transformed_p = transformed_center_2 + R * vec2(cos_theta, sin_theta);
			transformed_direction = sgn(R) * sgn(angular_distance_2) * vec2(- sin_theta, cos_theta);
			}

		RotationMatrix r(start_heading[0], - start_heading[1]);
		pos = r * transformed_p + start;
		forward = r * transformed_direction;
		}
	
	return true;
	}


//
//	Additional functions by J. Nagle
//
//	getcurvatureat  -- get curvature at specified distance from start.
//	
//	This is used to generate steering commands

float SplinePath::getcurvatureat(const float distance_along_path) const						
{	if (!m_valid) return(0.0);					// straight ahead if not valid
	double dist = distance_along_path; // we'll need to adjust this later

	// First check if dist is along the initial 0th arc used to account for steering servo latency
	if (dist < path_length_0)
		return curvature_0;
	else
		dist -= path_length_0;

	if (!finite(arc_radius) || abs(arc_radius) > BIG_RADIUS) 		// straight line path
	{	return(0.0);	}							// curvature is zero

	double distance_along_arc_1 = abs(arc_radius * angular_distance_1);
	if (dist < distance_along_arc_1) 			// if in first part of arc
	{	return(-1.0 / arc_radius); }				// use first arc radius
	else
	{ return(1.0 / arc_radius); }

}
//
//	getsteeringcurvature  -- get suggested steering curvature
//	
//	This is used to generate steering commands.
//
//	It's always the first arc curvature, after the steering latency section.
//
float SplinePath::getsteeringcurvature() const						
{	if (!m_valid) return(0.0);				// straight ahead if not valid
	if (!finite(arc_radius) || abs(arc_radius) > BIG_RADIUS) 		// straight line path
	{	return(0.0);	}							// curvature is zero
	return(-1.0 / arc_radius); 				// use first arc radius
}
//	
//	dump -- dump key params to log file, with message
//
void SplinePath::dump(const char* msg) const
{	
	logprintf("%s: path from (%1.2f, %1.2f) to (%1.2f, %1.2f), end dir (%1.2f, %1.2f) curv0. %1.4f for %1.2f m. curv %1.4f\n",
		msg, getstartpos()[0], getstartpos()[1], getendpos()[0], getendpos()[1],  getendforward()[0], getendforward()[1],
		curvature_0, path_length_0, getsteeringcurvature());	// dump path info
}



