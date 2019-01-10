//
//	goalsetting.cpp  -- setting of the goal point
//
//	Part of NewSteer
//
//	This contains the functions that work best in the absence of obstacles. There's some duplication of 
//	functionality between this and goalsetting2, but this code is retained to compute the "ideal goal point"
//	because it has been field tested.
//
//	John Nagle
//	Team Overbot
//	March, 2005
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
#include "newsteer.h"
#include "tuneable.h"
#include "waypoints.h"
#include "waypointutil.h"
#include "curvedwedge.h"
#include "dynamicsutil.h"
#include "arcpath.h"																
#include "logprint.h"

//
//	getMinGoalDist  -- get minimum lookahead distance for goal point on centerline
//
float NewSteer::getMinGoalDist(float dist)
{
	const float k_quarter_turn_dist = (1.0/m_maxcurvature)*M_PI*0.5;	// quarter turn at tightest radius
	dist = std::max(dist, k_quarter_turn_dist);								// prevent excessively short lookahead
	return(dist);																			// dist to look ahead
}	
//
//	testGoalPointAgainstBoundaries  -- test whether a given goal point is reachable in current circumstances
//
//	Tests dynamics and boundaries, but not obstacles.
//
bool NewSteer::testGoalPointAgainstBoundaries(const WaypointTriple& wp, float basewidth, const vec2& goalpt)
{
#ifdef OBSOLETE
	float curvature;
	bool good = tangentarcthroughpoints(m_inposition, m_inforward, goalpt, curvature);
	if (!good)
	{	logprintf("testGoalPointAgainstBoundaries: unable to compute tangent, goal point badly positioned.\n");	 // turn > 180 degrees
		return(false);																	// fails
	}
	double dist = arclength(m_inposition, goalpt, curvature);			// distance to goal point along arc
	int sideout = 0;
	vec2 impingement;
	return(testCurvatureAgainstBoundaries(wp, basewidth, curvature, dist, impingement, sideout));	// use common curvature test
#endif // OBSOLETE
	//	Use common test, but with an arc, not an S-curve.
	ArcPath path;
	path.setstart(m_inposition, m_inforward,  m_inspeed, m_incurvature, getSteeringDwellDistance(), getmincurv(), getmaxcurv(), 
		9999.0, getSteeringLookaheadDistance());	// set starting params of path
	path.setend(goalpt, m_inforward);											// end dir is ignored for arcs
	ImpingementGroup impingements;											// impingements to report
	const float shoulderwidth = 0;													// no shoulders here
	const float dwelldistance = 0;													// look at everything, because ArcPath doesn't have dwell.
	scanPathAgainstWaypoints(wp, path, basewidth, shoulderwidth, dwelldistance, path.getlength(), impingements);
	bool good = !impingements.m_center.m_valid;							// true if no impingement
	if (good) return(true);																// success
	//	Hits boundaries. Report problem, path will be shortened.
	if (getVerboseLevel() >= 3)														// only for high verbosity
	{	logprintf("testGoalPointAgainstBoundaries failed: goal (%1.2f, %1.2f) width %1.2f m hit at (%1.2f, %1.2f)\n",
			goalpt[0], goalpt[1], basewidth, impingements.m_center.m_pos[0], impingements.m_center.m_pos[1]);
	}
	return(false);
}
#ifdef OBSOLETE

//
//	testCurvatureAgainstBoundaries -- tests dynamics and boundaries, but not obstacles.
//
//	Short form, does not try to fix the arc length.
//
bool NewSteer::testCurvatureAgainstBoundaries(const WaypointTriple& wp, float basewidth, float curvature, float arclen, vec2& impingement, int& sideout)
{
	if (curvature > getmaxcurv() || curvature < getmincurv()) 						// if curvature outside limits
	{	//	We can't turn as tight as we need to. Fails
		return(false);																						// can't turn that tight
	}
	//	Compute turn arc info
	double dist = arclen;																// arc length to test
	//	Calculate distance required to get steering to match desired curvature.
	//	Within that distance, we will assume the old curvature applies, for a conservative path estimate.
	float dt = fabs(curvature-m_incurvature) / m_steeringrate;		// time required for steering gear to move
	double dist1 = fabs(m_inspeed)*dt;											// distance required for steering to move
	dist1 = std::max(dist1, m_vehicledim[1]*0.5);							// can't turn within own length
	dist1 = std::min(dist1, dist);														// can't be more than distance to end of turn
	float dist2 = dist -	dist1;															// distance in turn after steering completed
	//	Check arc
	bool good = checkWedgeAgainstWaypoints(
		m_inposition, m_inforward, basewidth, m_incurvature, curvature, dist1, dist2, 
		wp, impingement, sideout, m_outwedgepoints);
	if (!good)																					// if path collides with boundary
	{	logprintf("testCurvatureAgainstBoundaries failed: curv1 %1.4f dist1 %1.2fm curv2 %1.4f dist2 %1.2fm width %1.2fm hit at (%1.2fm, %1.2fm)\n",
			m_incurvature, dist1, curvature,  dist2, basewidth, impingement[0], impingement[1]);
		////logprintf("Steering rate %1.4f curv/sec  maxcurv %1.4f time %1.2f secs, distance %1.2f out of %1.2f\n", m_steeringrate, m_maxcurvature, dt, dist1, dist);	// ***TEMP***
	}
	return(good);
}
#endif // OBSOLETE
//
//	getGoodGoalPointOnCenterline  --  generate a goal point on the centerline
//
//	This always produces a goal point on the "centerline".  That's what we'd like, but we
//	can't always get it.
//
//	Maxspeed comes in at the speed we'd like to go, and may be reduced.
//
bool NewSteer::getGoodGoalPointOnCenterline(const WaypointTriple& wp, double distaheadin, vec2& goalpt, float& maxspeed)
{
	const double mindisttogoal = m_vehicledim[1]*0.5;									// goal must be ahead of vehicle at all times
	const char* why = "???";																			// reason for trouble
	//	Iterates, reducing the move distance, until we find a move that will work, or we give up.
	for (double distahead = distaheadin; distahead >= mindisttogoal; distahead *= 0.95)	// reduce distahead until success
	{	bool inturn;																								// sink
		bool good = getGoalPointOnCenterline(wp,  inturn, distahead, goalpt);	// calc new goal point on centerline
		if (!good) 
		{	why = "unable to get goal point on centerline";								// why
			continue;																							// try again
		}
		//	That's our goal point.
		float curvature = 0;
		//	Calculate curvature for this goal point
		good = tangentarcthroughpoints(m_inposition, m_inforward, goalpt, curvature);
		if (!good)
		{	why = "unable to compute tangent - probably too tight a turn";										// for later msg
			continue;																					// fails
		}
		//	Limit future speed, so we can make the turn we'd like to make next
		maxspeed = std::min(maxspeed, calcSafeSpeedForCurvature	(curvature));	// we need to get down to this speed
		//	Limit curvature to avoid steering into a rollover
		//	Apply vehicle dynamics limits to prevent steering into a rollover
		if (curvature > getmaxcurv() || curvature < getmincurv()) 			// if curvature outside limits
		{	//	We can't turn as tight as we would like, so we have to try again.
			why = "roll risk";
			continue;																					// try again with a shorter move
		}
		////		Shortened paths are narrower, so in narrow spots, we slow down.
		////float testdist = std::min(distahead, getstoppingdist());				// distance to test used to control width
		float testdist = distahead;																// distance to test used to control width
		float width = getBaseWidth(true) +2*getShoulderWidth(testdist);	// get test width, with safety margin
		good = testGoalPointAgainstBoundaries(wp, width, goalpt);			// test wedge, width depends on lookahead dist
		if (good) 
		{	if (distahead < distaheadin)														// if had to shorten move
			{	logprintf("Shortened move from %1.2f to %1.2f: %s.\n",
					distaheadin, distahead, why);
			}
			return(true);																				// arc is OK. Go with it.
		}
		//	We're about to hit a waypoint boundary.  Next iteration will try to find a steering and speed correction that will 
		//	avoid a collision.
		why = "test wedge hit boundary";												// explain
	}
	logprintf("Unable to find any valid move to goal point on centerline: %s\n", why);		// what went wrong
	return(false);																				// failed
}
//
//	getGoodGoalPointNearCenterline  --  generate a goal point near the centerline
//
//	This we use if we can't get to a goal point on the centerline due to curvature limits.
//
//	Cuts the speed way down, so we can turn tighter later.
//
//	Maxspeed comes in at the speed we'd like to go, and may be reduced.
//
bool NewSteer::getGoodGoalPointNearCenterline(const WaypointTriple& wp, double distahead, vec2& goalpt, float& maxspeed)
{
	const double mindisttogoal = m_vehicledim[1]*0.5;									// goal must be ahead of vehicle at all times
	if (distahead < mindisttogoal)																	// if so close we have to give up
	{	return(false);	}																					// fails
	bool inturn;																								// sink
	bool good = getGoalPointOnCenterline(wp,  inturn, distahead, goalpt);	// calc new goal point on centerline
	if (!good) return(false);																				// fails
	//	That's the goal point we'd like to reach. But if we're here, we can't.
	float curvature = 0;
	//	Calculate curvature for this goal point
	good = tangentarcthroughpoints(m_inposition, m_inforward, goalpt, curvature);
	if (!good)
	{	logprintf("getGoodGoalPointNearCenterline: unable to compute tangent, goal point badly positioned.\n");	 // turn > 180 degrees
		return(false);																	// fails
	}
	//	Limit future speed, so we can make the turn we'd like to make next
	maxspeed = std::min(maxspeed, calcSafeSpeedForCurvature	(curvature));	// we need to get down to this speed
	//	Limit curvature to avoid steering into a rollover
	//	Apply vehicle dynamics limits to prevent steering into a rollover
	curvature = std::max(std::min(getmaxcurv()*0.99f, curvature), getmincurv()*0.99f);		// limit curvature, avoid rounding errors
	//	Recompute goal point, as nearest to desired goal we can reach.
	//	We project along the arc that's as close to the goal arc as we can get.
	vec2 sink;
	goalpt = pointalongarc(m_inposition, m_inforward, curvature, distahead, sink);
	float width = getBaseWidth(false) +2*getShoulderWidth(distahead);		// get test width, less safety margin than above
	good = testGoalPointAgainstBoundaries(wp, width, goalpt);			// test wedge
	if (good)																						// arc is OK. Go with it.
	{
		return(true);	
	}
	logprintf("getNearCenterline: cannot reach (%1.2f, %1.2f) distance %1.2f  curvature %1.4f\n",
		goalpt[0], goalpt[1],  distahead, curvature);											// ***TEMP***
	//	We're about to hit a waypoint boundary.  Try to find a steering and speed correction that will 
	//	avoid a collision.
	//	Pull in the goal point, but keep it on the centerline, until we find a wedge that works.
	distahead = 0.95*distahead;													// pull in distance, slowly
   	//	Test recursively with new goal point.
	good = getGoodGoalPointNearCenterline(wp, distahead, goalpt, maxspeed);		// try nearer goal, recursively
	return(good);																			// return result after recursive fix
}


	