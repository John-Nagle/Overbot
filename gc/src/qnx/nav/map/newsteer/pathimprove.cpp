//
//	pathimprove.cpp  -- fine tuning of the path.
//
//	At this point, we already have an approximate path that will almost work. Here we fine tune it
//	for greater clearance against obstacles and tangency to the boundary at the end, if necessary.
//
//	Part of NewSteer
//
//	John Nagle
//	Team Overbot
//	June, 2005
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
//
//	Constants ***TEMP*** duplicate values in newsteer.cpp
//
const Tuneable k_min_move("MINMOVE",0.1, 3.0, 2.0, "Minimum move dist (m)");
const Tuneable k_min_unknown_dist("MINUNKNOWNDIST",1.0, 10.0, 6.0, "Closest distance to approach unknown area (m)");
const Tuneable k_planning_safety_margin("PLANNINGMARGIN",0.20, 2.0, 1.5, "Desirable clearance on either side of vehicle");
const Tuneable k_max_tilt_degrees("MAXTILT", 5 , 15, 13.5, "Maximum allowed tilt for a path (degrees)"); // equals 15% grade for Ranger
const float k_big_value = 9999999.0;						// a big number for worst cases
const double k_max_tilt_radians = k_max_tilt_degrees * (M_PI / 180.0);	// max tilt radians
//
//	pathDistanceToImpingement  -- distance along path to impingement
//
//	Never return a value longer than the path length.
//
float pathDistanceToImpingement(const CurvedPath& path, const ImpingementInfo& impingement)
{	return(std::min(path.distanceto(impingement.m_pos), path.getlength())); }

//
//
//	combinePathImpingements  -- combine impingements to get most restrictive one.
//
static void combinePathImpingements(const CurvedPath& path, bool iscenter, ImpingementInfo& imp1, const ImpingementInfo& imp2)
{
	if (!imp2.m_valid) return;														// nothing to do
	if (!imp1.m_valid)																	// first is null, second is nonnull															
	{	imp1 = imp2;																	// use second
		return;																				// done
	}
	//	Actually have to combine.
	//	***LOOK AT PASSABLE/POSSIBLE/UNKNOWN INFO**
	if (iscenter)					
	//	For now, take nearest impingement.
	{	//	In center, nearest dominates
		float dist1 = pathDistanceToImpingement(path, imp1);
		float dist2 = pathDistanceToImpingement(path, imp2);
		if (dist1 < dist2) return;															// dist1 wins
		imp1 = imp2;																		// dist2 wins
	} else {																					// if shoulder
		//	For shoulders, smallest offset dominates.
		if (fabs(imp1.m_offset) < fabs(imp2.m_offset)) return;			// imp1 wins
		imp1 = imp2;																		// imp2 wins	
	}
}
//
//	calcTiltPenalty  -- calculate penalty associated with tilt in path
//
//	For now, both roll and pitch have an equal penalty.
//
//	Totally flat terrain has a penalty of 0
//	Maximally tilted terrain has a penalty of 1
//	Returns false if too tilted to drive upon.
//
bool NewSteer::calcTiltPenalty(const ImpingementGroup& impingements, float& tiltpenalty)
{
	float costilt = impingements.m_worsttiltvector * vec3(0,0,1);	// cosine of tilt
	costilt  = std::min(1.0f, costilt);													// avoid arccos trouble
	float tiltradians = acos(costilt);													// tilt angle in radians
	tiltpenalty = tiltradians / k_max_tilt_radians;								// fraction of max allowed
	return(tiltpenalty < 1.0);															// allowed up to this max
}
//
//	testPath  -- test a given path, returning best distance free of obstacles
//
//	The question here is how far we can advance. Any distance greater than the vehicle length returns true.
//	"pathlenout" tells how far we can go.
//
//	This does NOT test for vehicle dynamics.
//
//	In "safe mode", unknown areas stop the search. So safety tests have safemode=true.  Planning test have safemode=false.
//
//	Returns true unless path has a center obstacle so close as to make the path useless.
//
//	Shoulder width is for EACH shoulder.
//
bool NewSteer::testPath(const WaypointTriple& wp, const TerrainMap& map, bool safemode, float shoulderwidth, const CurvedPath& path, 
	const float pathlenin, float& pathlenout, ImpingementGroup& impingements)
{
#ifdef OBSOLETE //	may need to check this in paths.
	//	Trim back arclen to keep turn under a half circle. The boundary tests break for arcs bigger than that.
	if (fabs(curvature) > 0.0001)														// if not straight
	{	float halfcircumference = M_PI/(fabs(curvature));						// half-circle arc
		arclen = std::min(halfcircumference*0.95f, arclen);					// trim arc length back
	}
#endif // OBSOLETE
	impingements.clear();																	// nothing seen yet
	//	First test against boundaries
	pathlenout = -1;																			// no move yet
	//	Path properties
	float basewidthboundaries = getBaseWidth(false);						// smaller base width for boundaries
	//	Allow zero clearance from obstacles in reverse. If we're brushing something, we can still back out.
	bool widerforobstacles = m_indir > 0;											// no extra width for obstacles in reverse
	float basewidthobstacles = getBaseWidth(widerforobstacles);		// bigger base width for obstacles, in forward
	if (safemode)																				// in safe mode, shoulders become part of base width
	{	basewidthboundaries += 2*shoulderwidth;								// add for each side
		basewidthobstacles += 2*shoulderwidth;
		shoulderwidth = 0;
	}
	float boundarypathlen = pathlenin;												// incoming path length to test
	//	Test path (including shoulders)
	ImpingementGroup bimpingements;												// impingement info for boundaries
	scanPathAgainstWaypoints(wp, path, basewidthboundaries, shoulderwidth, getSteeringDwellDistance(), boundarypathlen, bimpingements);
	const ImpingementInfo& cen = bimpingements.m_center;			// center impingement
	if (cen.m_valid)																				// hit in center
	{	boundarypathlen = pathDistanceToImpingement(path, cen);	// distance to trouble spot, shorten path accordingly
		if (getVerboseLevel() >= 2)
		{	logprintf("scanPathAgainstWaypoints found center obstacle at (%1.2f, %1.2f), length reduced from %1.2fm to %1.2fm.\n",
				cen.m_pos[0], cen.m_pos[1], path.getlength(), boundarypathlen);		
		}
	}
	pathlenout = boundarypathlen;															// length so far
	//	Then test against obstacles
	//	Scan curved wedge for obstacles
	//	Use new path test. This tests both the base path and the "shoulders" on either side,
	//	allowing later adjustment of position.
	scanPathAgainstObstacles(map, path, basewidthobstacles, shoulderwidth, boundarypathlen, m_terrainthreshold, impingements);
		
	//	Analyze results
	//	Combine obstacle impingements in shoulders and shoulder impingements
	combinePathImpingements(path, true, impingements.m_center, bimpingements.m_center);								// combine, keeping most restrictive
	combinePathImpingements(path, false, impingements.m_left, bimpingements.m_left);				// combine, keeping most restrictive
	combinePathImpingements(path, false, impingements.m_right, bimpingements.m_right);		// combine, keeping most restrictive
	if (getVerboseLevel() >= 2)												// very verbose
	{	
		dump("Obstacles", impingements);								// dump all impingements
#ifdef OBSOLETE
		if (impingements.m_center.m_valid)
		{	logprintf("%s in center, %1.2f m to right of centerline at (%1.2f, %1.2f) distance %1.2f m.\n", 
				(impingements.m_center.m_unknown ? "Unknown" : "Obstacle"), 
				impingements.m_center.m_offset,												// sign of offset is always positive to right
				impingements.m_center.m_pos[0], impingements.m_center.m_pos[1], 
				pathDistanceToImpingement(path, impingements.m_center));
		}
		// analyze shoulder obstacles
		if (impingements.m_left.m_valid)
		{	logprintf("%s on left, %1.2f m to left of centerline at (%1.2f, %1.2f) distance %1.2f m.\n", 
				(impingements.m_left.m_unknown ? "Unknown" : "Obstacle"), 
				-impingements.m_left.m_offset,												// sign of offset is always positive to right
				impingements.m_left.m_pos[0], impingements.m_left.m_pos[1], 
				pathDistanceToImpingement(path, impingements.m_left));
		}
		if (impingements.m_right.m_valid)
		{	logprintf("%s on right, %1.2f m to right of centerline at (%1.2f, %1.2f) distance %1.2f m.\n", 
				(impingements.m_right.m_unknown ? "Unknown" : "Obstacle"),
				impingements.m_right.m_offset,
				impingements.m_right.m_pos[0], impingements.m_right.m_pos[1], 
				pathDistanceToImpingement(path, impingements.m_right));
		}
#endif // OBSOLETE
	}
	if (impingements.m_center.m_valid && !impingements.m_center.m_unknown)						// always reduce if obstacle in center
	{	pathlenout = std::min(pathlenout, pathDistanceToImpingement(path, impingements.m_center));
	}
	if (safemode)																				// if shoulders must be clear and not unknown
	{	if (impingements.m_center.m_valid)											// reduce if obstacle in center
		{	pathlenout = std::min(pathlenout, pathDistanceToImpingement(path, impingements.m_center)); }
#ifdef OBSOLETE	// only consider center area in safe mode
		if (impingements.m_right.m_valid)												// include right shoulder check
		{	pathlenout = std::min(pathlenout, pathDistanceToImpingement(path, impingements.m_right));	}
		if (impingements.m_left.m_valid)												// include left shoulder check
		{	pathlenout = std::min(pathlenout, pathDistanceToImpingement(path, impingements.m_left));	}	
#endif // OBSOLETE
	} else {																			// if planning mode	
		//	Planning mode check. if in planning mode, and stopped by unknown area,  use distance to boundary.
		//	This way, we'll pick longer paths, even into unknown areas, during planning search	
		if (impingements.m_center.m_valid && impingements.m_center.m_unknown)					// if planning mode, and stopped by unknown
		{	float impingementdist = pathDistanceToImpingement(path, impingements.m_center);
			if (impingementdist < k_min_unknown_dist)				// if too close to unknown
			{	pathlenout = 0;	
				pathlenout = boundarypathlen;										// ***TEMP TEST*** value as boundary, even if unknown
				logprintf("Too close to unknown area: %1.2f m.\n", impingementdist);	// avoid, or force a rescan
			}
			else
			{	pathlenout = boundarypathlen; }										// use boundary arc length
		}
	}					
	////bool success = (pathlenout >= m_vehicledim[1]);				// at least a vehicle length is good
	bool success = (pathlenout >= m_vehicledim[1]*0.5);				// at least half a vehicle length is good
	return(success);																	// need at least a vehicle length to be good
}
//
//	calcImprovementMetric  -- calculate the value of an improved path
//
//		Must favor long center paths over short paths.
//		Must favor shallow impingements over deep impingements.
//
//		Smaller metrics are better.
//
static void calcImprovementMetric(const CurvedPath& path, float basewidth, float shoulderwidth, 
	const ImpingementGroup& impingements, float& outmetric, float& outoffset)
{
	bool leftobs = impingements.m_left.m_valid && !impingements.m_left.m_unknown;	// obstacle on left
	bool rightobs = impingements.m_right.m_valid && !impingements.m_right.m_unknown; // obstacle on right
	bool centerobs = impingements.m_center.m_valid && !impingements.m_center.m_unknown;				// obstacle in center
	float leftdist = pathDistanceToImpingement(path, impingements.m_left);		// distance to impingement on left (big value if none)
	float rightdist = pathDistanceToImpingement(path, impingements.m_right);	// distance to impingement on right  (big value if none)
	////float centerdist = pathDistanceToImpingement(path, impingements.m_center);			// distance to impingement on right  (big value if none)
	//	Although unlikely, it is possible to detect a center obstacle, even though the path is supposedly clear
	//	in the center at this point. This is due to quantization error in the path tests, which work by
	//	dividing continuous curves into trapezoids. So we have to look at the offset of
	//	the center obstacle and search in that direction.
#ifdef OBSOLETE
	//	***Center obstacle handling may be broken***
	if (centerobs)																					// if obstacle in center
	{	if (impingements.m_center.m_offset > 0) 										// use offset to 
		{	rightobs = true; 																		// treat as obstacle on right
			rightdist = std::min(rightdist, centerdist);
		}
		else
		{	leftobs = true; 																			// treat as obstacle on left
			leftdist = std::min(leftdist, centerdist);
		}
	}
#endif // OBSOLETE
	//	Calculate offset.  This is signed, with + meaning an obstacle on the right.
	const float k_impingementbase = shoulderwidth + basewidth*0.5;						// subtract offset from this to get impingement
	float leftoffset = std::max((k_impingementbase + impingements.m_left.m_offset),0.0f);		// left impingement depth, signed (>0,  for right turn)
	float rightoffset =  std::min(-(k_impingementbase - impingements.m_right.m_offset),0.0f);	// right impingement depth, signed (<0)
	float offset = 0;
	float offsetdist = 0;
	outmetric = 0;																				// zero metric means no impingements
	if (centerobs)																				// obstacle in center -- bad																		
	{	outoffset = -impingements.m_center.m_offset;							// so steer away from it
		outmetric = k_big_value;															// use big, failing metric
		return;																						// this path will be rejected
	} else {
		if (rightobs && leftobs)																// if trouble on both sides
		{	//	Hard case -- must calculate weighted offset based on both obstacles.
			float leftweight = 1.0;															// assume equal weighting
			float rightweight = 1.0;
			//	Calculate offset.  This is signed, with + meaning an obstacle on the right.
			offset = leftoffset * leftweight + rightoffset*rightweight;		// offset, signed, moving AWAY from obstacle
			//	Calculate metric, the deepest of the weighted impingements. This is always positive
			outmetric = std::max(leftoffset * leftweight,  - rightoffset*rightweight);		// metric, always positive
			offsetdist = std::min(leftdist, rightdist);									// use minimum distance		
		} else if (rightobs)																	// if obstacle on right
		{	offsetdist = rightdist;																// use obstacle on right info
			offset = rightoffset;																// impingement depth, signed (<0)
			outmetric = -offset;
		} else if (leftobs) {																	// if obstacle on left
			offsetdist = leftdist;																// use left info
			offset = leftoffset;																	// impingement depth, signed (>0)
			outmetric = offset;																// offset will be positive
		}
	}
	outoffset = -offset;																		// steer AWAY from obstacle
}
//
//	improvePath  -- try to improve path by fine adjustment
//
//	If the proposed path is close to a boundary or an obstacle, this will adjust it slightly
//	to try to clear it by a safe distance.  Worst case, you get back what you put in.
//
//	If the path ends by hitting a boundary and is turning towards it, the path
//	is modified so that the path is parallel to the boundary where it reaches it.
//
//	The input path may have impingements in the shoulder areas, but not in the center.
//
//	Path improvement maintains the exit angle of the path as input, while moving it
//	perpendicular to the direction at its end for the best clearance.
//
void NewSteer::improvePath(const WaypointTriple& wp, const TerrainMap& map, CurvedPath& path, float shoulderwidth, bool& tightspot)
{	tightspot = false;																	// not in tight spot yet
	//	Save properties of input path, independent of its form
	const vec2 inendpos = path.getendpos();										// point at end of path
	const vec2 inendforward = path.getendforward();							// direction at end of path
	vec2 inendright(inendforward[1], -inendforward[0]);						// unit vector to right
	ImpingementGroup impingements;
	//	Search for a better curvature.
	//	Moves end point perpendicular to end direction.
	const float basewidth = getBaseWidth(false);						// use narrow base width, for maximum improveability
	const float k_max_offset = k_planning_safety_margin;			// max offset is shoulder width in either direction.
	////float lo = leftobs ? -k_max_offset : 0;									// set search limits
	////float hi = rightobs ? k_max_offset : 0;
	float lo = -k_max_offset;														// set search limits
	float hi = k_max_offset;
	const int k_max_improvement_tries = 15;								// after this many, give up
	////const float k_pos_converged = 0.20;								// when this close, stop
	//	Decide how much of path to look at for centering purposes.
	//	If we look at too much, faraway problems have too much effect.
	//	If we look at too little, we don't avoid obstacles.
#ifdef OBSOLETE
	const float k_path_improve_range_factor = 3.0;					// look out N times the stopping distance
	//	Typically look several stopping distances out. This is the primary control on test range
	const float adj1 = getSteeringDwellDistance() + getstoppingdist() * k_path_improve_range_factor;
	const float adj2 = std::max(float(k_min_move), getSteeringDwellDistance()) * k_path_improve_range_factor;		// always look at least this far
	const float adj3 = std::max(adj1, adj2);									// max of above constraints
	const float adjustpathlen = std::min(adj3, float(path.getlength()));	// trim to path length if necessary
	//	The idea is not to look out to the entire path length, so that the part of the path that is near the
	//	boundary doesn't dominate.
	const float adjustpathlen = path.getlength() * 0.5;				// go out to end of path, but not quite ***TEMP TEST***
#endif // OBSOLETE
	float adjustpathlen = path.getlength();									// just use path length for now
	//	Calculate metric for initial arc
	float trydir;
	float trymetric;
	//	Test incoming arc
	float initialpathlen;																// good distance for input path
	if (getVerboseLevel() >= 1)													// somewhat verbose
	{	path.dump("Path before improvement");	}
	bool good = testPath(wp, map, false, shoulderwidth, path, adjustpathlen, initialpathlen, impingements); // test initial path
	if (!good) return;																	// should not happen, but reverts to old path
	calcImprovementMetric(path, basewidth, shoulderwidth, impingements, trymetric, trydir);	// initial metric
	float bestmetric = trymetric*0.95;											// best metric seen so far, biased in favor of original
	if (getVerboseLevel() >= 1)													// somewhat verbose
	{	logprintf("Looking out %1.2f m. Metric before improvement: %1.4f reduced to %1.4f\n", adjustpathlen, trymetric, bestmetric);	}
	//	Tight spot handling.  Bring endpoint closer to beginning of path, to force tighter s-curves
	vec2 searchendpos(inendpos);											// starting end position to try
	tightspot = intightspot(path, m_incurvature, impingements);			// if in tight spot, cut planning horizon for improvement
	if (tightspot)
	{	const float pathincr = path.getlength() * 0.05;					// advance by this fraction of a path
		float maxinmove = path.getlength()*0.5;							// don't cut more than half the path
		float startdistoutside = distanceoutsidewaypoints(path.getendpos(), wp);	// initial distance outside waypoints
		//	Try shorter paths until we can't turn any tighter at this speed
		for (float inmove = pathincr; inmove <= maxinmove; inmove += pathincr)
		{	vec2 tryendpos = inendpos - inmove*inendforward;	// construct a shorter path
			bool good = path.setend(tryendpos, inendforward);	// validate path is possible at current speed
			if (!good) break;															// too tight, done
			//	Path geometry is possible. Check that it is inside the waypoints, and no closer to the edge than the initial value
			float testdistoutside = 	distanceoutsidewaypoints(path.getendpos(), wp);	// this dist outside waypoints
			if (testdistoutside >= 0 || (testdistoutside > startdistoutside * 0.99)) // if outside, or getting worse
			{	break;	}
			//	Check that new path isn't too short to use.
			if (path.getlength() <= k_min_move*2.0) break;			// too short
			searchendpos = tryendpos; 										// OK to use this as search base
		}
		if (getVerboseLevel() >= 1)												// somewhat verbose
		{	logprintf("Tight spot, changed path end from (%1.2f, %1.2f) to  (%1.2f, %1.2f)\n", 
				inendpos[0], inendpos[1], searchendpos[0], searchendpos[1]);
		}
	}
	logPathEndpoint(path, 1);														// log, as blue outline
	float bestoffset = 0;																// best so far
	vec2 bestendpos(inendpos);												// best end pos seen so far, from original point
	//	Try moving endpoint transverse to ending dir
	for (int i=0; i <= k_max_improvement_tries; i++)					// limit number of tries
	{	float s = i * (1.0/(k_max_improvement_tries));					// range 0 to 1
		float tryoffset = lo*s + hi*(1-s);										// exhaustive search
		vec2 tryendpos(searchendpos + tryoffset*inendright);	// new end point, offset slightly from original
		bool good = path.setend(tryendpos, inendforward);		// set new end of path
		if (!good)
		{	if (getVerboseLevel() > 2)
			{	logprintf("Improved endpoint (%1.2f, %1.2f) invalid.\n", tryendpos[0], tryendpos[1]);	}
			continue;																		// try next path, unaware of which direction is better
		}
		if (getVerboseLevel() >= 2)												// very verbose
		{	path.dump("Test path");	}			
		float trypathlenout;															// unobstructed length for this path		
		float trypathlen = std::min(float(path.getlength()), adjustpathlen);	// never exceed validpathlength					
		testPath(wp, map, false, shoulderwidth, path, trypathlen, trypathlenout, impingements); 	// try to find a goal point
		//	Validate improvement of path. Note that path may actually be invalid, in which case
		//	all we use from this is a value from trydir.
		calcImprovementMetric(path, basewidth, shoulderwidth, impingements, trymetric, trydir);
		trymetric += 0.01*fabs(tryoffset);									// apply slight penalty to prefer value near initial path
		if (getVerboseLevel() >= 2)												// very verbose
		{	logprintf("Improving path: offset %1.2fm, metric %1.4f  dir %1.2f\n", tryoffset, trymetric, trydir);	}
		if (trymetric < bestmetric)													// we have a new winner
		{	bestmetric = trymetric;													// save new winning metric
			bestendpos = tryendpos;												// save new winning point
			bestoffset = tryoffset;
		}
	}
	if (getVerboseLevel() >= 1)													// somewhat verbose
	{	logprintf("Improving path done: offset %1.2fm, metric %1.4f, end point (%1.2f, %1.2f)\n",
		bestoffset, bestmetric, bestendpos[0], bestendpos[1]);
	}
	bool validpath = path.setend(bestendpos, inendforward);	// set best path found
	if (!validpath)																		// if best path is not valid
	{	path.dump("best path failing");											// trouble
		throw("ERROR: improvePath: previously valid path now fails.");				// major failure
	}
}


