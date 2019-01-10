//
//	goalsetting2.cpp  -- setting of the goal point
//
//	Part of NewSteer
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
#include "logprint.h"
#include "curvedpath.h"
#include "terrainmap.h"
//
//	Constants ***TEMP*** duplicate values in newsteer.cpp
//
const Tuneable k_min_full_turn_speed("MINFULLSPEED", 0.3, 3.0, 2.0, "Minimum forward speed allowing fullest turn (m/sec)");
const Tuneable k_min_move("MINMOVE",0.1, 3.0, 2.0, "Minimum move dist (m)");
//	MINUNKNOWNDIST should be further than the distance to the LIDAR's blind spot.
const Tuneable k_min_unknown_dist("MINUNKNOWNDIST",1.0, 10.0, 6.0, "Closest distance to approach unknown area (m)");
const Tuneable k_goal_dist("GOALDIST",1, 40, 15, "Distance to goal point (m)");
const Tuneable k_planning_safety_margin("PLANNINGMARGIN",0.20, 2.0, 1.5, "Desirable clearance on either side of vehicle");
const Tuneable k_steering_dwell_time("STEERINGDWELLTIME",0.0, 1.0, 0.300	, "Delay before steering command takes effect");
const Tuneable k_lock_to_lock_time("LOCKTOLOCKTIME", 3.0, 7.0, 4.0, "Steering time, lock to lock (secs)");
const float k_big_value = 99999999999.0;						// a big number for worst cases 
const Tuneable k_safety_margin("SAFETYMARGIN",0.20, 2.0, 0.40, "Minimum clearance on either side of vehicle");
//
//	Road follower constants
//
const Tuneable k_road_follower_bias("ROADFOLLOWERBIAS",0.0, 1.0, 0.25, "Road follower scale factor (0 to 1)");
const Tuneable k_road_follower_max("ROADFOLLOWERMAX",0, 4.0, 0, "Road follower max path increase value (m)");
//
//	getBaseWidth  -- get base width for path.
//
//	Larger for obtacle check, so we can get closer to the boundary in narrow sections.
//
float NewSteer::getBaseWidth(bool forobstacles)
{
	float basewidth = m_vehicledim[0];											// minimal value, without safety margin
	if (forobstacles) basewidth += (2.0*k_safety_margin);				// add extra margin for obstacles
	return(basewidth);																	// return good value
}
//
//	getShoulderWidth  -- get appropriate "shoulder width" to add to vehicle width as a safety margin
//
//	This is for the "safety wedge", which grows based on stopping distance, to account for steering error.
//
//	Note that basewidth already includes k_safety_margin on both sides.
//	
//	Shoulder width is for EACH shoulder.
//
float NewSteer::getShoulderWidth(float dist)
{	float shoulderwidth = 0.5*(dist-m_vehicledim[1])*m_trackingerrratio;		// grow width with distance as safety margin
	shoulderwidth = std::max(shoulderwidth, 0.0f);								// but never less than zero
	////logprintf("getShoulderWidth: dist %1.2f m, shoulder width %1.2f m\n", dist, shoulderwidth);	// ***TEMP***
	return(shoulderwidth);
}
//
//	Combined obstacle avoidance and boundary checking.
//
//	Algorithm:
//
//		Pass 1: Look out to LMS scan distance within reachable curvature range.
//			Pick best distance. 
//
//		If best distance is worse than LMS scan distance,
//
//		Pass 2: Look out to LMS scan distance outside reachable curvature
//			range. Pick closest curvature to reachable with better than
//			best distance above.	
//
//
//	distanceToImpingement  -- distance to specified impingement point from beginning of path
//
//	dumb version - linear distance.
//	Should always underestimate distance.
//
static double distanceToImpingement(const CurvedPath& path, const vec2& pt)
{	return((path.getstartpos() - pt).length());								// linear distance to impingement
}
//
//	distanceToImpingement  -- distance to specified impingement point from beginning of path
//
static double distanceToImpingement(const CurvedPath& path, const ImpingementInfo& impingement)
{	if (!impingement.m_valid) return(k_big_value);						// big bogus value if no problem
	return(distanceToImpingement(path, impingement.m_pos));	// compute distance
}
//
//	getBoundaryHitPt  -- get closest boundary hit point from impingement
//
static bool getBoundaryHitPt(const ImpingementGroup& impingements, const CurvedPath& path, vec2& hitpt, int& side)
{
	bool centerobs = impingements.m_center.m_valid && !impingements.m_center.m_unknown;
	bool leftobs = impingements.m_left.m_valid && !impingements.m_left.m_unknown;
	bool rightobs = impingements.m_right.m_valid && !impingements.m_right.m_unknown;
	if (centerobs)
	{	hitpt = impingements.m_center.m_pos;								// use center impingement hit point
		side = 0;
		return(true);
	}
	if ((!rightobs) && (!leftobs)) return(false);									// no impingements
////#ifdef OBSOLETE
	//	Pick closest impingement, if on both sides
	if (distanceToImpingement(path, impingements.m_left) < distanceToImpingement(path, impingements.m_right))
	{	hitpt = impingements.m_left.m_pos; side = -1;	}				// hit point is on left
	else
	{	hitpt = impingements.m_right.m_pos;	side = 1;}			// hit point is on right
////#endif // OBSOLETE
#ifdef OBSOLETE
	//	Hit in shoulder area only. Align to path, but retain original end point.
	side = 0;																				// no center area hit
	hitpt = path.getendpos();														// use end of path for calc.
#endif // OBSOLETE
	return(true);
}
//
//	testBoundaryHit  -- test whether path hits boundary, and set ending direction if necessary.
//
//	This is what forces S-curved paths near a boundary.
//
//	Ending direction is set tangent to the boundary if there is a hit.
//	This is called with impingements for the boundary only.
//
bool testBoundaryHit(const WaypointTriple& wp, const CurvedPath& path, const ImpingementGroup& impingements, vec2& hitpt,  float& endlength, vec2& enddir)
{
#ifdef OBSOLETE
	bool centerobs = impingements.m_center.m_valid && !impingements.m_center.m_unknown;
	float centerdist = distanceToImpingement(path, impingements.m_center);
	if (!centerobs) return(false);												// no center impingement, ignore
	float dist = centerdist;														// only consider center impingements
	if (dist >= k_big_value) return(false);								// no distance to impingement, ignore
	hitpt = impingements.m_center.m_pos;								// use center impingement hit point
#endif // OBSOLETE
	int hitside;																			// which side of boundary did we hit, relative to us
	bool hit = getBoundaryHitPt(impingements, path, hitpt, hitside);
	if (!hit) return(false);															// no boundary hit
	//	Get distance to boundary at starting point.
	//	If the path curves away from the boundary, we don't need to make it tangent.
	////int startside, endside;
	vec2 centerlinept;
	vec2 sink;
#ifdef TEMPTURNOFF
	double startdistoutside = distanceoutsidewaypoints(path.getstartpos(), wp, startside, centerlinept, enddir, sink);
	double enddistoutside = distanceoutsidewaypoints(path.getendpos(), wp, endside, centerlinept, enddir, sink);
	if (enddistoutside < startdistoutside && (startside == endside))		// if getting further from boundary
	{	logprintf("Boundary hit at %1.2fm, but getting further from it.\n", endlength);
		return(false);																// don't need to force tangent to boundary
	}
#endif // TEMPTURNOFF
	//	Now find boundary at trouble spot. 
	//	Find forward direction at impingement point
	int side;
	distanceoutsidewaypoints(hitpt, wp, side, centerlinept, enddir, sink);	// for forward dir only
	endlength = distanceToImpingement(path, hitpt);									// use distance to impingement to recompute path
	////logprintf("Boundary hit at %1.2fm (%1.2f, %1.2f) - path end direction becomes (%1.2f, %1.2f)\n",
	////	endlength, hitpt[0], hitpt[1], enddir[0], enddir[1]);
	return(true);																		// force direction change
}
//
//	getSteeringDwellDistance  -- get delay before steering command takes effect
//
float NewSteer::getSteeringDwellDistance() const
{
	return(k_steering_dwell_time * m_inspeed);						// time before steering command takes effect
}
//	
//	getSteeringLookaheadDistance -- get how far ahead to look when getting curvature for a steering command
//
float NewSteer::getSteeringLookaheadDistance() const
{
	const float k_steer_to_path_vehicle_lengths = 2.0;			// steer to this many vehicle lengths out
	double curvdist = getSteeringDwellDistance() + k_steer_to_path_vehicle_lengths*m_vehicledim[1];	// get curvature from this far out
	return(curvdist);
}
//
//	adjustMetricForRoad  -- adjust for road following
//
//	Provide a slight preference for a path on the road
//
float	NewSteer::adjustMetricForRoad(const TerrainMap& map, float curv, float metric)
{
	const RoadFollowInfo& roadinfo =  map.getroadfollowinfo();			// get road info
	if (roadinfo.m_confidence <= 0) return(metric);								// ignore if low confidence
	float curvdiff = fabs(curv - roadinfo.m_curvature) / m_maxcurvature;	// difference from ideal curvature, range 0 to 1
	float curverr = 1.0 - curvdiff;															// invert
	float extrametric = curverr * k_road_follower_bias * k_road_follower_max;	// gain 
	extrametric = std::min(extrametric, float(k_road_follower_max));				// maximum metric increase 
	return(metric + extrametric);															// return adjusted path
}

//
//	constructPathFromCurvature  -- given a desired test curvature and length, construct a path to go there.
//
//	The path will be a simple curve unless the path is close to the boundary, in which case the ending direction
//	for the path will be adjusted to be tangent to the boundary at that point.
//
bool NewSteer::constructPathFromCurvature(const WaypointTriple& wp, const TerrainMap& map, float curv, float pathlen, CurvedPath& path, 
			ImpingementGroup& impingements)
{
	//	Find endpoint of proposed arc.
	pathlen = k_goal_dist;	// ***TEMP TEST*** always test with full distance
	const float basewidthboundaries = getBaseWidth(false);				// smaller base width for boundaries
	////const float basewidthobstacles = getBaseWidth(true);					// bigger base width for obstacles
	vec2 goalfwd;																				// forward direction at end of arc
	vec2 goalpt(pointalongarc(m_inposition, m_inforward, curv, pathlen, goalfwd)); 	// proposed goal point
	bool good = path.setend(goalpt, goalfwd);									// construct initial path, no end dir constraint
	if (!good)																						// impossible geometry
	{	if (getVerboseLevel() >= 2)
		{	logprintf("Invalid path with curvature %1.4f to (%1.2f, %1.2f) default ending dir (%1.2f, %1.2f)\n",
				curv, goalpt[0], goalpt[1], goalfwd[0], goalfwd[1]);
		}
		return(false);																			// fails
	}
	ImpingementGroup bimpingements;												// impingement info for boundaries
	bimpingements.clear();																	// clear impingement info
	//	Test path against boundaries, to see if there is a need for an end direction constraint
	scanPathAgainstWaypoints(wp, path, basewidthboundaries, k_planning_safety_margin, getSteeringDwellDistance(), path.getlength(), bimpingements);
	vec2 hitptfwd;																					// direction needed at end of path, if any, for goals a and b
	float endlen;
	vec2 hitpt;																						// where path hits boundary
	bool needdirconstraint = testBoundaryHit(wp, path, bimpingements, hitpt, endlen, hitptfwd);				// check if need end direction constraint
	if (needdirconstraint)																		// if need to make path tangent to boundary
	{	const vec2 oldgoalpt(goalpt);														// for debug message
		bool centerimpingement = bimpingements.m_center.m_valid && !bimpingements.m_center.m_unknown;	// only if center hit, change end pos
		if (centerimpingement)																	// if obstacle in center
		{	//	Center impingement.  Must shorten path and change end dir
			//	Compute a new, closer goal point, near where the original arc hit the boundary.
			//	The idea is to place the vehicle as close to the boundary as is safely possible, and parallel to it.
			//	We want to generate a goal position that doesn't result in a boundary impingement, so we have to offset the
			//	position slightly more than half the base width.
			goalfwd = hitptfwd;																	// use forward dir consistent with hit pt
			const float k_boundary_offset = m_vehicledim[0]*0.5 + k_safety_margin;				// ofset this far from impingement as safe position
			vec2 rightenddir(hitptfwd[1], -hitptfwd[0]);								// right dir, relative to fwd at hit point
			//	One of these two goal points is inside the boundary, and one is outside.
			vec2 goala(hitpt + rightenddir*k_boundary_offset);					// One possible goal point
			vec2 goalb(hitpt - rightenddir*k_boundary_offset);					// Other possible goal point
			float goalaout = distanceoutsidewaypoints(goala, wp);			// goal point distances outside waypoints
			float goalbout = distanceoutsidewaypoints(goalb, wp);
			if (goalaout > 0 && goalbout > 0)											// if both outside, may have hit end of waypoint.
			{	//	Just use original path in this case, but trim it to boundary so it will be be evaluated properly.
				if (getVerboseLevel() >= 2)
				{	logprintf("constructPathFromCurvature: hit point (%1.2f, %1.2f) far outside boundary.\n", hitpt[0], hitpt[1]); }
				float goaldist = ::distanceToImpingement(path, hitpt);			// distance to hit point along path		
				goaldist *= 0.5;																		// this path hits boundary, derate its length
				goaldist = std::min(goaldist, float(path.getlength()*0.99));	// avoid overlong path
				good = path.pointalongpath(goaldist, goalpt, goalfwd);		// project path to where it hits the boundary
				if (!good) return(false);															// bogus, reject
			} else {																					// at leat one is inside, use it
				if (goalaout < goalbout)														// use most-inside goal
				{	goalpt = goala; 																// goal a wins, more inside
				}
				else 	
				{	goalpt = goalb;  																// goal b wins
				}
			}
		} else {
			//	Shoulder impingement only.  Must change end dir, but not shorten path.
			//	End dir is parallel to the waypoints. This may force an S-curve.
			goalpt = path.getendpos();														// get end position as end of path
			int side;
			vec2 centerlinept, sink;																// unwanted outputs
			float goalcout = distanceoutsidewaypoints(goalpt, wp, side, centerlinept, goalfwd, sink);	// calculate direction at end
			if (goalcout > 0)																		// if outside waypoints
			{	if (getVerboseLevel() >= 2)
				{	logprintf("constructPathFromCurvature: path just touches shoulder, but end point (%1.2f, %1.2f) is %1.2f m outside boundary.\n", 
						goalcout, goalpt[0], goalpt[1]); 
				}
				return(false);																		// reject this path
			}
		}
		//	Change goal point to new point at end of useful portion of previous arc.
		good = path.setend(goalpt, goalfwd);											// add end dir constraint to path
		if (!good)																						// impossible geometry
		{	
			if (getVerboseLevel() >= 2)														// very verbose
			{	logprintf("Invalid path with curvature %1.4f to (%1.2f, %1.2f) adjusted ending dir (%1.2f, %1.2f)\n",
				curv, goalpt[0], goalpt[1], goalfwd[0], goalfwd[1]);
			}
			return(false);																			// fails
		}
		if (getVerboseLevel() >= 2)															// very verbose
		{	logprintf("Boundary hit at (%1.2f, %1.2f) - path end position changed from (%1.2f, %1.2f) to (%1.2f, %1.2f)\n",
				hitpt[0], hitpt[1], oldgoalpt[0], oldgoalpt[1], goalpt[0], goalpt[1]);	
			////path.dump("Path to test");																				// ***TEMP***	
		}
	}
	//	Finally do the real impingement test.
	//	This path should not have boundary impingements if at all possible.
	float pathlenout;
	return(testPath(wp, map, false, k_planning_safety_margin, path, path.getlength()*0.99, pathlenout, impingements));
}
//
//	tryPathCurvature -- try a path with a specific curvature, and evaluate its metric
//
bool NewSteer::tryPathCurvature(const WaypointTriple& wp, const TerrainMap& map, const float testcurv,
	const vec2& ingoalpt,	CurvedPath& path,	float& metric)																			
{
	//	Calculate vector to ideal goal point
	vec2 idealvectogoal = (ingoalpt - m_inposition);						// we'd like to go here
	const float inpathlen = idealvectogoal.length();						// distance we' d like to go
	idealvectogoal.normalize();														// unit vector towards goal
	ImpingementGroup impingements;											// obstacle info
	//	Construct path which goes to a point on an arc from the vehicle position, but is a general path.
	//	The generated path may be an S-curve if necessary.
	bool good = constructPathFromCurvature(wp, map,  testcurv, inpathlen, path, impingements);
	if (!good) return(false);																// rejected
	//	Found a usable point, but not necessarily the best one.
	float pathclearlength = path.getlength();						// get clear length of path
	bool centerobs = impingements.m_center.m_valid && (!impingements.m_center.m_unknown);	// only real obstacles, not unknowns
	bool leftobs = impingements.m_left.m_valid && (!impingements.m_left.m_unknown);	// only real obstacles, not unknowns
	bool rightobs = impingements.m_right.m_valid && (!impingements.m_right.m_unknown);	// only real obstacles, not unknowns
	if (centerobs)																// if obstacle in center
	{	pathclearlength = ::distanceToImpingement(path, impingements.m_center);	}	// use distance to center impingement
	vec2 endclearpos, sink;														// get point on path at end of clear section
	good = path.pointalongpath(pathclearlength, endclearpos, sink);			
	if (!good) return(false);													// unlikely, but ignore path if happens	
	vec2 vectogoal = (endclearpos - m_inposition);			// vector toward new goal, relative to vehicle position
	float towardsgoal = vectogoal * idealvectogoal;			// component towards goal
	//	Compute added distance from change, straight line assumption.
	vec2 v1(m_outpathendpos - m_inposition);					// remainder of last move
	vec2 v2(endclearpos - m_outpathendpos);					// from end of last move to new move
	double lenlongway = v1.length() + v2.length();			// length as if we went to previous goal, then this one
	double lenshortway = vectogoal.length();					// length by direct route
	float extradistfromchange = (lenlongway - lenshortway);	// added distance
	if (!finite(m_outpathendpos[0])) extradistfromchange = 0;		// initial condition check
	extradistfromchange = std::min(towardsgoal*0.75f, extradistfromchange);	// don't let change check dominate goal
	towardsgoal -= extradistfromchange;							// penalize change in direction by extra distance moved
	//	End of new added distance algorithm
	//	Finally compute metric
	metric = towardsgoal;													// no casing
	//	Add to metric if road follower indicates this is near the road
	metric = adjustMetricForRoad(map, testcurv, metric);	// add road follower bias
	if (centerobs) 
	{	metric *= 0.5;	}														// disfavor center obstacles substantially
	else if (rightobs || centerobs)										// disfavor side obstacles somewhat
	{	metric *= 0.75;	}
	if (getVerboseLevel() >= 2)															// very verbose
	{	path.dump("Curvature search");								// Dump path again ***TEMP***
		logprintf("Curvature search: metric %1.2f test curvature %1.4f  path length %1.2f %s%s%s\n",
			    	metric, testcurv, path.getlength(), 
			    	(centerobs ? " CENTER" : ""),
			    	(leftobs ? " LEFT" : ""),
			    	(rightobs ? " RIGHT" : ""));
	}
	return(true);																	// success
}
//
//	logPathEndpoint  -- save path endpoint for debug
//
void NewSteer::logPathEndpoint(const CurvedPath& path, uint8_t color)
{
	//	Debug logging of all tested paths
	PathEndpoint endpt;														// create item for debug
	endpt.m_pos = path.getendpos();									// ending position (point of arrow)
	endpt.m_dir = path.getendforward();							// ending dir
	endpt.m_color = color;													// color
	m_outpathendpoints.push_back(endpt);						// add to debug
}

//
//	searchPathRange -- try a range of arcs against obstacles and boundaries
//
//	Test can be either inside or outside the curvature limits.  We try inside first, which means we can make the turn
//	without slowing down, and then try outside.
//
//		Test sequence is
//			Pass 1: 
//				current curvature
//				near current curvature out to steerable limits - fast obstacle avoidance
//			Pass 2:
//				curvatures outside limits, picking first one that will work at all - slow, must brake first
//
//		The actual search generates curved paths. If the path hits a boundary at its end, it is constrained
//		to be parallel to that boundary.
//		
//	***NEEDS WORK***
//	***NEED TO ENABLE PATHS WITH CURVATURES OUTSIDE LIMITS WHEN testinsidebounds = false***
//
bool NewSteer::searchPathRange(const WaypointTriple& wp, const TerrainMap& map, 
	bool testinsidebounds, 
	const vec2& ingoalpt,																// we want to get here
	CurvedPath& path)																	// path output path
{
	float bestcurv = 0;																	// best curvature found
	float bestmetric = -1;																// metric used to select winning curve
	float curvescale = testinsidebounds ? (getmaxcurv() - getmincurv()) : m_maxcurvature*2;	// range to test
	//	Calculate vector to ideal goal point
	vec2 idealvectogoal = (ingoalpt - m_inposition);						// we'd like to go here
	const float inpathlen = idealvectogoal.length();						// distance we' d like to go
	//	Calculate curvature to ideal goal point, and always try that.
	{	float testcurv = 0;
		float metric;
		//	Compute ideal arc
		bool good = tangentarcthroughpoints(m_inposition, m_inforward, ingoalpt, testcurv);
		bool isinsidebounds = (testcurv >= getmincurv() && testcurv <= getmaxcurv());	// inside current curvature limits?
		if (good && isinsidebounds)													// if possible curve
		{	//	Test this one, the ideal curve.
			bool good = tryPathCurvature(wp, map, testcurv, ingoalpt, path, metric);
			if (good)
			{	bestcurv = testcurv;														// initial test curvature
				bestmetric = metric*1.1;												// slight bias in favor of ideal path
		   		logPathEndpoint(path);													// create item for debug
			}
		}
	}
	//	Try curvature range
	for (size_t i = 0; i < getCurveTestSchedule().size(); i++)			// for curvature test schedule
	{	const float testcurv = m_incurvature + curvescale*getCurveTestSchedule()[i];	// curvature to test, starting from current steering
		bool isinsidebounds = (testcurv >= getmincurv() && testcurv <= getmaxcurv());	// inside current curvature limits?
		if (testinsidebounds != isinsidebounds) continue;				// test only appropriate range
		if (testcurv > m_maxcurvature || testcurv < -m_maxcurvature) continue;	 // avoid totally hopeless
		//	Construct path which goes to a point on an arc from the vehicle position, but is a general path.
		//	The generated path may be an S-curve if necessary.
		float metric;
		bool good = tryPathCurvature(wp, map, testcurv, ingoalpt, path, metric);
		if (good)
		{	//	Found a usable point, but not necessarily the best one.
		    if (metric > bestmetric)												// if new winner
		    {	bestcurv = testcurv;
			   	bestmetric = metric;
		    }
		    //	Debug logging of all tested paths
		    logPathEndpoint(path);													// create item for debug
		}
	}
	if (bestmetric < 0) return(false);										// if no valid path found, reject
	ImpingementGroup impingements;											// obstacle info
	bool good = constructPathFromCurvature(wp, map,  bestcurv, inpathlen, path, impingements);	// rebuild winning path
	if (getVerboseLevel() >= 1)													// somewhat verbose
	{	logprintf("Winner: metric %1.2f  Clear to %1.2f m on curvature %1.4f is winner.\n", bestmetric, path.getlength(), bestcurv);		}
	if (!good) return(false);													// if winning path rejected
	return(path.getlength() > k_min_move);						// success if found some workable path
}
//
//	constructPath2 -- reactive obstacle avoidance and path planning - the hard cases
//
//	Called when we have to do some obstacle avoidance
//
//	Path is an output only.
//
//	First pass tries to find some path that will work within the turning limits.
//	If that fails, we try "brake then steer" mode - look for the longest path
//	in any direction, regardless of dynamics limits, and slam on the brakes
//	while turning.  The actual steering command will be limited later.
//	***MAKE SURE THAT CHECK IS MADE***
//
//	***NEEDS WORK***
//	***SEARCHING OUTSIDE LIMITS WILL ALWAYS FAIL***
//
bool NewSteer::constructPath2(const WaypointTriple& wp, const TerrainMap& map, const vec2& ingoal, CurvedPath& path, bool& brakehard)
{	const double inpathlen = (m_inposition - ingoal).length();			// straight line distance to goal point
	//	Pass 1 search - within reachable limits.
	//	If we can move inpathlen distance without trouble, we're OK.
	bool good = searchPathRange(wp, map, true, ingoal, path);
	if (!good)
	{	logprintf("No possible path avoids obstacles and boundaries.\n");	// fails
		return(false);																			// fails
	}
	// If initial path is good
	if (path.getlength() >= (inpathlen*0.90))										// if near-complete success
	{	brakehard = false;																	// don't need to brake
		return(true);																				// done
	}
#ifdef TEMPTURNOFF	// Doesn't work with new obstacle checks
	//	Obstacle ahead is unavoidable at current speed, although it should be within the stopping distance.
	//	Check whether braking and a hard turn would help.
	//	This is a guess; it may or may not work.
	const vec2 pathendpos1(path.getendpos());							// save pass 1 result
	const vec2 pathendforward1(path.getendforward());
	const float pathlen1 = path.getlength();									// length
	//	***Need to check for stopped by unknown area during first pass*** 
	if (getVerboseLevel() >= 1)															// very verbose
	{	logprintf("Trying obstacle pass 2: wanted %1.2f m, could only go %1.2f m.\n", inpathlen, path.getlength());}
	//	Pass 2 search -- outside reachable limits		
	//	We try to find the curvature nearest to that last commanded where the vehicle could fit.
	//	This test is made without shoulders and without dynamics limitations. 
	//	We then view it as a best-effort curvature to try.
	good = searchPathRange(wp, map, false, ingoal, path);
	if (!good || (path.getlength() < pathlen1))								// if pass 2 worse than pass 1
	{	good = path.setend(pathendpos1, pathendforward1);			// use pass 1 result
		if (!good)
		{	logprintf("ERROR: Restored path failed geometry check.\n");
			return(false);																	// fails
		}
		////brakehard = false;															// on valid path, don't need to brake hard
		brakehard = true;																	// ***TEMP*** always brake hard when turning to avoid obstacle
		return(true);																			// fails
	}
	//	Pass 2 result looks better. Use it. Try a brake-then-steer
#endif // TEMPTURNOFF
	brakehard = true;																		// must brake hard
	logprintf("No good path within turn limits. Trying brake-then-steer.\n");
	return(true);																				// go for it
}
//
//	constructPath  -- reactive obstacle avoidance and path planning
//
//	Incoming goal point should indicate a path to the ideal point.
//	Path is output.
//
//	Any path derived from CurvedPath can be used.
//
bool NewSteer::constructPath(const WaypointTriple& wp, const TerrainMap& map, const vec2& ingoalpt, CurvedPath& path, float& movedist, float& maxspeed)
{	path.clear();																					// no path at all yet
	//	Find the ideal goal point, adjusted for boundaries, but not obstacles.
	vec2 goalpt = ingoalpt;																// working goal point
    bool good = adjustGoalPointForBoundaries(wp, k_goal_dist, goalpt, maxspeed);
    if (!good)
    {	setFault(Fault::boundary, "Cannot find a path that clears the boundaries");
        return(false);						// trouble
    }
    //	Construct the path to be improved.
    //	Any class derived from CurvedPath can be used.
    //	For now, the ending direction is initially constrained to be that of a curved arc. 
    float goalcurv;
    good =  tangentarcthroughpoints(m_inposition, m_inforward, goalpt, goalcurv);	// compute arc through points
    if (!good)																						// optional math check
    {	logprintf("ERROR: constructPath arc generation to goal point failed.\n");
    	return(false);
    }
    double goaldist = arclength(m_inposition, goalpt, goalcurv);		// compute length to goal point
	vec2 goalfwd;																				// forward dir at goal point
    vec2 goalpointchk = pointalongarc(m_inposition, m_inforward, goalcurv, goaldist, goalfwd); // compute dir at end of arc
    if ((goalpt - goalpointchk).length2() > 0.1)									// optional math check
    {	logprintf("ERROR: constructPath goal point check failed.\n");
    	return(false);
    }
    const float k_max_curvature_rate = (2.0*m_maxcurvature) / k_lock_to_lock_time;		// max speed at which curvature can change
	path.setstart(m_inposition, m_inforward,  m_inspeed, m_incurvature, 
		getSteeringDwellDistance(), getmincurv(), getmaxcurv(), k_max_curvature_rate, getSteeringLookaheadDistance());	// set starting params of path
	bool brakehard = false;																// do we need to slow down to get more turning options?
	bool needavoid = false;																// do we need to avoid an obstacle?
	bool tightspot = false;																	// are we in a tight spot?
	float shoulderwidth = k_planning_safety_margin;							// width to try to keep clear, if possible
	good = path.setend(goalpt, goalfwd);											// set end point - no direction constraint yet
	if (!good)
	{	logprintf("Invalid ideal path with curvature %1.4f to (%1.2f, %1.2f) default ending dir (%1.2f, %1.2f)\n",
			goalcurv, goalpt[0], goalpt[1], goalfwd[0], goalfwd[1]);
		needavoid = true;																		// must search
	} else {
		const double inpathlen = path.getlength();
		double pathlen = inpathlen;															// working path length
		//	Test of ideal point. If this succeeds, we will take this path, but may slow down.
		if (getVerboseLevel() >= 2)															// very verbose
		{	path.dump("Ideal path");		}													// dump ideal path
		ImpingementGroup impingements;												// collision info for area we need
		float pathlenout;																			// path length sink for test
		good = testPath(wp, map, false, shoulderwidth, path, path.getlength(), pathlenout, impingements);
		pathlenout = pathlenout;																// ignore this value
	  	if (!good)																						// if initial arc was no good
	  	{	needavoid = true; }																	// force obstacle avoidance
	  	if (impingements.m_center.m_valid && !impingements.m_center.m_unknown)									// if stopped by obstacle, must search				
	  	{	needavoid = true;	}																// force obstacle avoidance
	  	if (impingements.m_left.m_valid && !impingements.m_left.m_unknown)					// if stopped by obstacle, must search				
	  	{	needavoid = true;	}																// force obstacle avoidance
	  	if (impingements.m_right.m_valid && !impingements.m_right.m_unknown)				// if stopped by obstacle, must search				
	  	{	needavoid = true;	}																// force obstacle avoidance	
		if (needavoid)
		{
	 	  	if (getVerboseLevel() >= 1)															// very verbose
	 	  	{	logprintf("Ideal path obstacles: %s %s %s  arclen %1.2f m arclenout %1.2f m.\n",
		  	  		(impingements.m_center.m_valid && !impingements.m_center.m_unknown ? "CENTER " : ""),
  			  		(impingements.m_left.m_valid && !impingements.m_left.m_unknown ? "LEFT " : ""),
   			 		(impingements.m_right.m_valid && !impingements.m_right.m_unknown ? "RIGHT " : ""),
					pathlen, pathlenout);									
			}	
		}
	}
  	if (needavoid)																				// do we need to avoid an obstacle?
  	{	//	Now the hard cases
 	  	//	At this point, we have an obstacle problem.
 	  	if (getVerboseLevel() >= 1)															// very verbose
 	  	{	logprintf("Starting obstacle avoidance\n");									
		}	
	  	good = constructPath2(wp, map, goalpt, path, brakehard);		// do a search for a better path
	  	if (!good)																					// if fail
	  	{	setFault(Fault::obstacle, "Cannot clear obstacle/boundary.\n");
	  		path.clear();																			// clear path, so as not to try to use it
	  		return(false);																		// fails
	  	}
	  	//	We have found a path. Try to improve the clearances around it by minor adjustment.
	  	//	Even if we fault after this point, we will use this path, as the best we've got, during emergency stop.
 	  	if (getVerboseLevel() >= 1)															// very verbose
		{	path.dump("Obstacle-free path");		}										// dump ideal path
	  	//	This never makes things worse.
		improvePath(wp, map, path, shoulderwidth, tightspot);				// try to improve the path
	}
  	//	Final safety check. Sets move distance based on stopping distance, using safety wedge.
	ImpingementGroup impingements;												// collision info for area we need
	const float safeshoulderwidth = getShoulderWidth(getstoppingdist());
	good = testPath(wp, map, true, safeshoulderwidth, path, path.getlength(), movedist, impingements);		// test proposed arc path against obstacles and boundaries
	if (!good)																						// if trouble fitting in
	{	logprintf("Final check for obstacle failed with shoulder width %1.2f m.\n", safeshoulderwidth); // error logging
		dump("Final impingements", impingements);								// dump trouble info
		//	Retry with narrower path and reduced speed
		if (m_inspeed <= k_min_full_turn_speed)									// if already down to slow speed
		{
			setFault(Fault::obstacle, "Final check failed. Cannot clear obstacle/boundary, even at slow speed.");
			return(false);
		}
		//	We may just be going too fast in a tight spot.  Try again with a slowdown.
		float widthspeedlim = std::max(float(k_min_full_turn_speed), m_inspeed*0.75f);	// slow down 25% and try again
		maxspeed = std::min(maxspeed, widthspeedlim);					// cut speed to slow down a little
		//	Retest with narrower path and lower desired speed, rather than actual speed. We'll be down to that speed shortly.
		float narrowshoulderwidth;
		calcStoppingDistance(maxspeed, m_inpitch, m_inroll, narrowshoulderwidth);	// calc safe stopping distance for proposed speed
		good = testPath(wp, map, true, narrowshoulderwidth, path, path.getlength(), movedist, impingements);		// test proposed arc path against obstacles and boundaries
		if (!good)
		{	setFault(Fault::obstacle, "Final check failed. Cannot clear obstacle/boundary, even with narrow path.");
			return(false);
		}
	}
	//	Check for excessive tilt on path. 
	//	This check should also be made in planning, but this will at least save us.
	float tiltpenalty;
	good = calcTiltPenalty(impingements, tiltpenalty);
	if (!good)
	{	logprintf("Selected path is tilted too much. Up vector (%1.2f %1.2f %1.2f), penalty %1.2f\n", 
			impingements.m_worsttiltvector[0],
			impingements.m_worsttiltvector[1],
			impingements.m_worsttiltvector[2],
			tiltpenalty);
#ifdef TEMPTURNOFF	// too high a false alarm rate. Triggers on elevation noise. Need better tilt estimator. 
		setFault(Fault::tiltedpath, "Selected path tilted too much");					// fails
		return(false);
#endif // TEMPTURNOFF
		tightspot = true;																		// but slow way down
	}	
	//	Avoid running right up to obstacles.  Planning is relative to the vehicle position at the GPS receiver.
	//	So we subtract a vehicle length so we stop BEFORE the obstacle.
	if (impingements.m_center.m_valid)												// if any trouble ahead
	{	const float k_closest_approach_to_obstacle = m_vehicledim[1] + k_planning_safety_margin;	// good safe distance
		movedist -= k_closest_approach_to_obstacle;							// shorten move
		if (movedist <= k_min_move)
		{	setFault(Fault::obstacle, "Too close to obstacle");					// fails
			return(false);
		}
		if (!impingements.m_center.m_unknown)									// if real obstacle ahead
		{	brakehard = true;		}															// tight spot, slow down to low speed
	}
 	if (getVerboseLevel() >= 1)															// somewhat verbose
	{	path.dump("Adjusted path");		}												//
	//	Recheck for tight spot.
	brakehard |= tightspot;																// slow down if in tight spot
  	//	Desperation mode.  Must brake hard, hoping to slow enough to allow a turn past the obstacle.
	if (brakehard && maxspeed > k_min_full_turn_speed)
	{	logprintf("Obstacle or tight spot %1.2f m ahead. Braking hard to increase turning options.\n", path.getlength());
		maxspeed = k_min_full_turn_speed;													// force speed down now. May be too drastic.
	}
	return(true);																				// done  	
}
//
//	Misc. access functions
//
//
//	getPathLength -- get length of last path generated
//
//	For debug output.
//
const float NewSteer::getPathLength() const
{	return(m_outpath.getlength());	}	
//
//	getPointAlongPath  -- get point along the path
//
//	For debug output
//
const vec2 NewSteer::getPointAlongPath(float dist) const
{	vec2 dir;
	vec2 pt;
	bool good = m_outpath.pointalongpath(dist, pt, dir);					// get desired point along path
	if (!good)																					// bogus request, using starting pos
	{	pt = m_inposition;	}
	return(pt);																					// return point
}	
//
//	getCurvatureSchedule  -- get schedule of curvatures to try.
//
//	The idea is to have a range of arcs to try, closer together near the initial arc, further apart outward.
//
//	If there are too many entries, the CPU time consumption gets too large and the move server will time out.
//	About 10-12 entries appears to be safe.
//
//	The result is a table of numbers from -1 to +1
//
const std::vector<float>& NewSteer::getCurveTestSchedule()
{
	if (m_curvatureschedule.size() > 0) return(m_curvatureschedule);	// already initialized
	//	Initialize 
	m_curvatureschedule.push_back(0);							// try the obvious - whatever we were doing.
	//	Space logarithmically across the turning range
	const float k_curve_start = 0.05;									// start 2% apart and grow from there.
	const float k_curve_ratio = 1.3;										// grow by this much each time
	const float k_maxcurv = 0.99;										// limit of turning
	for (float curv = k_curve_start; curv < k_maxcurv; curv *= k_curve_ratio)	// curves get increasingly far apart
	{
		m_curvatureschedule.push_back(curv);					// add to list
		m_curvatureschedule.push_back(-curv);					// both dirs
	}
	m_curvatureschedule.push_back(k_maxcurv);				// add max value to list
	m_curvatureschedule.push_back(k_maxcurv);				// both dirs
	logprintf("Curvature schedule has %d curves to try.\n", m_curvatureschedule.size());	
	return(m_curvatureschedule);
}
	