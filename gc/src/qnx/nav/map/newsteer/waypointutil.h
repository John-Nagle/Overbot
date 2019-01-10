//
//	waypointutil.h  -- math functions related to waypoints
//
//	Part of NewSteer
//
//	John Nagle
//	Team Overbot
//	February, 2005
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
#ifndef WAYPOINTUTIL_H
#define WAYPOINTUTIL_H
#include "waypoints.h"
#include "algebra3.h"
#include "geomutil.h"
#include "logprint.h"	// ***TEMP***
//
//	The null waypoint
//
const Waypoint k_null_waypoint =
    {
        -1,0,0,0,0
    }
    ;				// used when we don't have a usable waypoint
//
inline bool nullwaypoint(const Waypoint& wpt) { return(wpt.m_serial < 0); }	// test for null waypoint.
//
//	WaypointTriple --  a constant group of three waypoints
//
//	We pass groups of three around enough that this is useful. It keeps down the number of parameters to functions.
//
class WaypointTriple {
private:
	Waypoint	m_wpts[3];												// the waypoints
public:
	WaypointTriple(const Waypoint& wpt0, const Waypoint& wpt1, const Waypoint& wpt2)		// constructor
	{	m_wpts[0] = wpt0; m_wpts[1] = wpt1; m_wpts[2] = wpt2; }
	WaypointTriple()														// constructor
	{	m_wpts[0] = k_null_waypoint; m_wpts[1] = k_null_waypoint; m_wpts[2] = k_null_waypoint; }
	const Waypoint& operator[](size_t i) const				// const access
	{	assert(i<3); return(m_wpts[i]); }
	Waypoint& operator[](size_t i) 								// access
	{	assert(i<3); return(m_wpts[i]); }
};

//
//	distanceoutsidewaypoint  -- distance outside waypoint segment.
//
//	negative values mean inside.
//
inline double distanceoutsidewaypoint(const vec2& pt, const Waypoint& wp0, const Waypoint& wp1, double& disttostart, double& disttoend, int& side, vec2& intersect)
{	double U;																			// fraction along waypoint
	vec2 p0(wp0.m_x, wp0.m_y);
	vec2 p1(wp1.m_x, wp1.m_y);
    double dist = pointtolinesegmentdistance(pt, p0, p1, U, intersect);	// calc signed distance
    side = (dist > 0) ? 1 : -1;													// +1 if on right side
    dist = fabs(dist);																// use signed distance
	double length = (p1-p0).length();										// length of segment
    double width = wp0.m_width;											// take width from FIRST waypoint
    disttostart = U*length;														// distance to start of waypoint. Negative in endcap
    disttoend = (1-U)*length;													// distance to end.
    return(dist - (width*0.5));													// outside dist
}
//
//	distanceoutsidewaypoint  -- short form
//
//	Called very frequently.
//
inline double distanceoutsidewaypoint(const vec2& pt, const Waypoint& wp0, const Waypoint& wp1)
{	double U;																			// fraction along waypoint
	vec2 p0(wp0.m_x, wp0.m_y);
	vec2 p1(wp1.m_x, wp1.m_y);
	vec2 intersect;																	// sink
    double dist = pointtolinesegmentdistance(pt, p0, p1, U, intersect);	// calc signed distance
    dist = fabs(dist);																// use signed distance
    double width = wp0.m_width;											// take width from FIRST waypoint
    return(dist - (width*0.5));													// outside dist
}
//
//	distanceoutsidewaypoints -- 3-waypoint form
//
inline double distanceoutsidewaypoints(const vec2& p, const WaypointTriple& wp)
{
	double distance = distanceoutsidewaypoint(p, wp[0], wp[1]);		// check first segment
	if (nullwaypoint(wp[2])) return(distance);									// if no third waypoint, done
	distance = std::min(distance, distanceoutsidewaypoint(p, wp[1], wp[2]));	// most-inside distance wins
	return(distance);																		// done
}
//
//	getturningcenter  -- get center point of a waypoint turn
//
//	The center is relative to the middle waypoint
//
//	Returns true if result meaningful.
//	Returns false for straight line case, single segment case.
//
inline bool getturningcenter(const WaypointTriple& wp, vec2& center, int& turndir)
{
	if (nullwaypoint(wp[2])) return(false);										// not meaningful for two waypoint case
    const float width = std::min(wp[0].m_width,wp[1].m_width);		// smallest width (not halfwidth) involved in turn
    vec2 p0(wp[0].m_x, wp[0].m_y);												// first point
	vec2 p1(wp[1].m_x, wp[1].m_y);												// second point
	vec2 p2(wp[2].m_x, wp[2].m_y);												// third point
	vec2 fwd0(p1-p0);																	// segment forward vector
	vec2 fwd1(p2-p1);																	// segment forward vector
	fwd0.normalize();
	fwd1.normalize();
	vec2 right0(fwd0[1], -fwd0[0]);												// right unit vector, first segment
	vec2 right1(fwd1[1], -fwd1[0]);												// right unit vector, second segment
	const float sinturnangle = -fwd0*right1;									// sine of turn angle (>0 means right turn)
	if (fabs(sinturnangle) < 0.01) return(false);								// effectively straight
	turndir = (sinturnangle > 0) ? 1 : -1;											// output turn direction, 1 = right
	//	Now compute center of the turning circle
	vec2 ent0(p1 - fwd0*(width*0.5));											// turn entrance point
	vec2 ent1(p1 + fwd1*(width*0.5));											// turn exit point
	////logprintf("getturningcenter: (%1.2f, %1.2f) +  (%1.2f, %1.2f)   (%1.2f, %1.2f) + (%1.2f, %1.2f) dir %d\n",
	////	ent0[0], ent0[1], right0[0], right0[1], ent1[0], ent1[1], right1[0], right1[1], turndir);	// ***TEMP***
	return(lineintersection(ent0, ent0+right0, ent1, ent1+right1, center));	// center is intersection of radial lines
}
//
//	distanceoutsidewaypoints -- 3-waypoint form, with additional information
//
//	"side" > 0 means that p is on the right of the waypoint centerline. 
//	Negative values of the return value mean inside the waypoint
//	"centerlinept" is the nearest point on the centerline to the test point.
//	"forward" is the forward direction at the center point.
//
//	if return value is < 0, there is no impingement.
//
inline double distanceoutsidewaypoints(const vec2& p, const WaypointTriple& wp, int& side, 
	vec2& centerlinept, vec2& forward, vec2& impingement)
{
	double disttostart0, disttoend0;										// distances from starting, ending waypoints
	vec2 intersect0;																// point on centerline
	int side0;																			// -1 on left, +1 on right
	double distance = 0;														// output distance
	double width = 0;																// width of segment (not halfwidth)
	bool inturn = false;															// true if in the turn area between two waypoints
	double distance0 = distanceoutsidewaypoint(p, wp[0], wp[1], disttostart0, disttoend0, side0, intersect0);		// check first segment
	if (nullwaypoint(wp[2])) 
	{	centerlinept = intersect0;												// closest point on centerline to p
		side = side0;
		distance = distance0;													// only one waypoint
		forward = vec2(wp[1].m_x, wp[1].m_y) - vec2(wp[0].m_x, wp[0].m_y);		// forward direction, unnormalized
		width = wp[0].m_width;												// width
	} else {
		double disttostart1, disttoend1;									// distances from starting, ending waypoints
		vec2 intersect1;															// point on centerline
		int side1;																		// -1 on left, +1 on right
		double distance1 = distanceoutsidewaypoint(p, wp[1], wp[2], disttostart1, disttoend1, side1, intersect1);
		float turncircleradius = std::max(wp[0].m_width, wp[1].m_width)*0.5;	// waypoint radius of turn (halfwidth)
		inturn =  (disttostart1 < turncircleradius && disttoend0 < turncircleradius);	// if center line point is in the turn		
		if (distance1 < distance0	)											// if distance1 wins	
		{	side = side1;															// use onright from distance 1
			centerlinept = intersect1;											// closest point on centerline
			distance = distance1;
			forward = vec2(wp[2].m_x, wp[2].m_y) - vec2(wp[1].m_x, wp[1].m_y);		// forward direction, unnormalized
			width = wp[1].m_width;											// half
		} else {																		// distance 0 wins (more negative)
			side = side0;
			centerlinept = intersect0;
			distance = distance0;
			forward = vec2(wp[1].m_x, wp[1].m_y) - vec2(wp[0].m_x, wp[0].m_y);		// forward direction, unnormalized
			width = wp[0].m_width;											// width
		}
	}
	//	Common final computation of forward direction.
	//	"forward" is along the waypoint centerline, except in turns, where it is tangent to
	//	the turn centerline path.
	int turndir;																				// turning direction 1 right -1 left
	if (inturn)																				// in circular part of turn, must use turn angle
	{	vec2 center;																		// turning center of turn
		inturn = getturningcenter(wp, center, turndir);					// find turning center of waypoint turn - this is normally outside the boundaries
		if (inturn)
		{	vec2 sidevec(center - p);												// vector from point to turning center
			if (sidevec.length2() > 0.01)											// if not on top of turning center (which should not happen)
			{	vec2 right(sidevec*turndir);										// vector to right
				forward = vec2(-right[1], right[0]);							// perpen. to left of vector
				////logprintf("In turn: turn center (%1.2f, %1.2f), forward (%1.2f, %1.2f)\n", center[0], center[1], forward[0], forward[1]);	// ***TEMP***
			}
		}
	}
	forward.normalize();															// normalize forward vector
	//	Compute impingement point. This is exactly on the boundary.
	impingement = vec2(0,0);												// assume no impingement
	if (distance >= 0)																// if point outside waypoints
	{	
		if (inturn)																		// if in turn region, need point on circle
		{	float halfwidth = std::max(wp[0].m_width, wp[1].m_width)*0.5;	// larger turning circle
			vec2 wp1(wp[1].m_x, wp[1].m_y);							// waypoint 1, the center of the turn region (not turning center)
			vec2 radial = (p - wp1);											// from center point to 
			radial.normalize();														// unit radial from center point towards p
			impingement = wp1 + radial * halfwidth;					// impingement point, on circle
		} else {																		// if not in a turn			
			vec2 rightvec(forward[1], -forward[0]);					// right unit vector
			impingement = centerlinept + rightvec*(side*width*0.5);	// point at which line from p1 to centerline crosses boundary
		}
	}
	return(distance);																// finally return distance
}

#endif // WAYPOINTUTIL_H