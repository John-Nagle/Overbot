//
//	splinepath.h  -- spline path support
//
//	Ron Avitur
//	Team Overbot
//	June, 2005
//
//	Arcs are not enough. Sometimes we need to exit a turn going in a specific direction.
//	This requires a more complex curve.
//
//	Actually, this is an implementation that generates two arcs, not splines.
//
//	Splines with four control points are sufficient. We need to specify position and
//	direction at turn entry and turn exit.
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
#ifndef SPLINEPATH_H
#define SPLINEPATH_H

#include "curvedpath.h"													// the base class
#include <algorithm>  													// for std::min

class RotationMatrix
	{
	public:

	double cos_a,sin_a;
	
	inline RotationMatrix(double a, double b) { cos_a = a; sin_a = b;}
	};

inline vec2 operator* (const RotationMatrix& r, const vec2& v)
	{
	return vec2(
		 r.cos_a * v[VX] + r.sin_a * v[VY],
		-r.sin_a * v[VX] + r.cos_a * v[VY]
		);
	}

//
//	class SplinePath  -- one path determined by a spline
//
//	Positions are 2D vectors.
//	Directions are 2D unit vectors.
//
//	Notes:
//
//		"setstart" specifies two constraints - the tightest curvature, and
//		the maximum rate of change of curvature.  Maximum rate of change is
//		in units of curvature change per unit distance. (The caller must compute
//		this based on the steering rate and forward speed.)
//
//		"setend" returns true if successful, and false if the required constraints cannot be
//		maintained.
//
//		"setstart" must be called before "setend".
//		"setstart" and "setend" must be called before any of the other functions.
//		"setend" can be called multiple times.
//
//		"getlength" can be an approximation, but the approximation must agree with that
//		used by "pointalongpath", so that when "pointalongpath" is called with "dist"
//		equal to "getlength()", the output "pos" is consistent with "setend".
//
//		When "setend" is called with the one-argument form, the path generated should
//		be roughly an arc.
//
//		"pointalongpath" will be called most often, and must be fast.  It will be called
//		7 to 10 times for each call to "setend".
//
class SplinePath: public CurvedPath
{	
private:
	// An initial arc determined by inertia preceeds the construction
	vec2 actual_start;
	vec2 actual_start_heading;

	// The two-arc construction begins after the initial arc caused by the steering servo latency
	vec2 start;
	vec2 start_heading;	// the stored value is normalized

	vec2 goal;
	vec2 goal_heading;	// the stored value is normalized
	
	double max_curvature;
	
	// maxcurvaturerate is ignore. The two-arc path is guaranteed has a curvature discontuity where
	// the arcs join, except in the degenerate case where one of the arc-lengths is zero.
	
	// The path is constructed of two circular arcs with the same radius. That makes the solution unique.
	double arc_radius;

	// An initial arc determined by inertia preceeds the construction
	vec2   center_0;
	double curvature_0;
	double arc_radius_0;
	double path_length_0;
	double angular_distance_0; // fraction of circle in radians travelled on 0th arc
	double angular_distance_1; // fraction of circle in radians travelled on first arc
	double angular_distance_2; // fraction of circle in radians travelled on second arc
	
	// in the simplified coordinate system transformed_start = {0,0} and transformed_start_heading = {1,0}
	// the solution is formulated in that coordinate system to make the equations shorter
	
	double transformed_arc2_start;	// initial angle	for start of second arc in the simplified coordinate system
	
	vec2 transformed_center_1;			// center of the first arc in the simplified coordinate system

	vec2 transformed_center_2;			// center of the second arc in the simplified coordinate system
	bool	m_valid;								// true if valid path

public:
	SplinePath();																		// constraints on how tight the curve can be
	void setstart(const vec2& pos, const vec2& forward, const float speed, 
				const float curvature, const float initialcurvaturedist, 
				const float mincurvature, const float maxcurvature, const float maxcurvaturerate, const float lookaheaddist);

	bool setend(const vec2& pos);													// specify end of path, direction is don't care
	bool setend(const vec2& pos, const vec2& forward);					// specify end of path, direction is forward
	double getlength() const;																// get length of path
	bool pointalongpath(const float dist, vec2& pos, vec2& forward) const;	// get point on path, measuring from start
	const vec2& getstartpos() const;													// get starting point
	const vec2& getstartforward() const;											// get starting direction
	const vec2 getendpos() const;														// get ending position
	const vec2 getendforward() const;												// get ending direction (null vector if no constraint)
	float getcurvatureat(const float dist) const;									// get curvature at specified distance from start
	////double distanceto(const vec2& pt) const;								// distance to this point from path start (approx)
	float getsteeringcurvature() const;												// get best curvature for steering use					
	void clear();																					// clear path, invalidate it
	bool getvalid() const;																	// true if valid path
	float getmincurv() const;																// curvature limits
	float getmaxcurv() const;
	void dump(const char* msg) const;											// dump, with message

#ifdef DEBUGOUTPUT
	void debugoutput();
#endif
};
//
//	Implementation
//
//	Misc. access functions - return basic parameters of the path.
//
inline const vec2& SplinePath::getstartpos() const { return(actual_start); };								
inline const vec2& SplinePath::getstartforward() const { return(actual_start_heading); };		
inline const vec2 SplinePath::getendpos() const { return(goal); };							
inline const vec2 SplinePath::getendforward() const { return(goal_heading); };		
inline float SplinePath::getmincurv() const { return(-max_curvature); }	// doesn't do signed curvature
inline float SplinePath::getmaxcurv() const { return(max_curvature); }
inline bool SplinePath::getvalid() const { return(m_valid); }


// if goal heading isn't specified, arrive at goal with initial heading

inline SplinePath::SplinePath()
: m_valid(false)
{}

inline bool SplinePath::setend(const vec2& pos) 
	{
	return SplinePath::setend(pos, start_heading);
	} 


// The equations return arc_radius == INF for straight line solutions
// To avoid possible overflow for near-straight line solutions which create ridiculously large arc_radius
// we treat anything bigger than BIG_RADIUS as a straight line.
#define BIG_RADIUS 10000

inline double SplinePath::getlength() const
	{
	if (!m_valid) return(0.0);												// zero if not valid
	return
		path_length_0 +
			(
				(!finite(arc_radius) || abs(arc_radius) > BIG_RADIUS) ?  // straight line path
					(goal - start).length()
				:
					abs(arc_radius * (abs(angular_distance_1) + abs(angular_distance_2))) // any of those values may be negative
			);
	}


inline void SplinePath::setstart(
	const vec2& pos,
	const vec2& forward,
	const float /*speed*/ ,
	const float curvature, 
	const float initialcurvaturedist, 
	const float mincurvature, 
	const float maxcurvature, 
	const float /*maxcurvaturerate*/,
	const float /*lookaheaddist*/ )
	{
	m_valid = false;																	// not valid until successful setend.
	// maxcurvaturerate is ignored. These paths have a curvature discontinuity in all but the degenerate one-arc case
	actual_start = pos;
	actual_start_heading = forward;
	actual_start_heading.normalize();

	curvature_0   = curvature;
	arc_radius_0  = 1 / curvature;
	path_length_0 = initialcurvaturedist;
	
	if (!finite(arc_radius_0) || abs(arc_radius_0) > BIG_RADIUS) {
		// straight line case for initial steering-servo-latency arc
		start = actual_start + path_length_0 * actual_start_heading;
		start_heading = actual_start_heading;
		}
	else
		{
		center_0 = actual_start + arc_radius_0 * vec2(-actual_start_heading[1], actual_start_heading[0]);	
		angular_distance_0 = curvature * path_length_0;
		start_heading = RotationMatrix(cos(angular_distance_0), -sin(angular_distance_0)) * actual_start_heading;
		start = center_0 - arc_radius_0 * vec2(-start_heading[1], start_heading[0]);
		}

	max_curvature = std::min(fabs(maxcurvature), fabs(mincurvature));	// ***TEMP*** use most restrictive curvature range
}
//
//	clear -- invalidate path
//
inline void SplinePath::clear()
{	m_valid = false;	}
//
//	Implementation
//

#endif // SPLINEPATH_H