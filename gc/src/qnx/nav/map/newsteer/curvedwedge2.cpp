//
//	curvedwedge2.cpp  -- curved wedge utilities
//
//	Supports spline-based paths
//
//	John Nagle
//	Team Overbot
//	February, 2005
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
#include "curvedwedge.h"
#include "curvedpath.h"
#include "waypointutil.h"
#include "logprint.h"
//
//
//	Constants
//
const double k_min_length = 0.01;				// below this, length is effectively zero

class Waypoint;

//
//	class CurvedPathBoundaryScanner -- scans a curved path, trapezoid by trapezoid
//
//	Uses common path/trapezoid generator
//
class CurvedPathBoundaryScanner: public CurvedPathScanner  {
private:
	const WaypointTriple& m_wp;					// waypoint input
	ImpingementGroup& m_result;					// result output
	const float m_dwelldistance;						// ignore shoulder impingements closer than this
public:
	CurvedPathBoundaryScanner(const WaypointTriple& wp, ImpingementGroup& result, const float dwelldistance)
		: m_wp(wp), m_result(result), m_dwelldistance(dwelldistance)
		{}
protected:
	bool raster_ordered_trapezoid(const vec2 quad[4], const vec2& last_sforward, const vec2& next_sforward,
		double distalongpath, float basewidth, float shoulderwidth);
};

//
//	updatewaypointhit  -- update impingement info if "worse"
//
//	This is used to test arcs against the waypoints.
//
//	"distanceoutside" is the distance that the impingement point on the side of the path is outside the waypoint boundaries.
//	"hitpt" is the relevant point on the test quad.
//
//	"offset" in outputs is the actual offset from the center of the test quad used for the test.
//	"impingement" in outputs is the intersection of the waypoint boundary and the test quad.
//
//	"side" > 0 means that we are examining the right side of the path.
//	
//
static bool updatewaypointhit(const WaypointTriple& wp, float basewidth, float shoulderwidth, bool pastdwell, int side, 
		const vec2& hitpt, float distanceoutside, ImpingementInfo& center, ImpingementInfo& shoulder)
{
	if (distanceoutside <= 0) return(true);										// no problem, inside
	float offset = basewidth*0.5+shoulderwidth - distanceoutside;		// offset from centerline of test quad (unsigned)
	if (offset < 0)
	{	logprintf("ERROR in updatewaypointhit: negative offset %1.2f, distanceoutside %1.2f\n",
			offset, distanceoutside);													// should not happen
			offset = 0;																			// force offset to zero
	}
	//	Compute proper impingement point.
	vec2 forward;																			// forward direction
	vec2 centerlinept;
	int sidehit;																					// > 0 if hitpt is on right of waypoint centerline
	//	Redo distance outside, using long-form check, to get actual impingement point
	vec2 impingement;
	distanceoutside = distanceoutsidewaypoints(hitpt, wp, sidehit, centerlinept,  forward, impingement);
#ifdef OBSOLETE
	//	***TEMP*** recheck consistency 
	//	May be OK for sides to differ, if test quad entirely outside waypoint.
	if (fabs(distanceoutsidecheck - distanceoutside) > 0.05
		|| (sidehit != side))
	{	logprintf("ERROR in updatewaypointhit check: side %d != %d or distance %1.2f != %1.2f at hitpt (%1.2f, %1.2f)\n",
			sidehit, side, distanceoutsidecheck, distanceoutside, hitpt[0], hitpt[1]);
	}
#endif // OBSOLETE
	if (distanceoutside > shoulderwidth)										// base part of trapezoid hits waypoint boundary
	{	//	This means a hit in the central area of the path.
		center.m_valid = true;
		center.m_pos = impingement;												// actual point where boundary hit
		center.m_offset = offset*side;												// actual offset
		center.m_passable = center.m_possible = center.m_unknown = false;
		return(false);																		// fails
	}
	//	No hit in central area.  Examine shoulder info.
	if (side == 0) return(true);														// if not a shoulder test, stop now
	if (!pastdwell) return(true);														// ignore hit if not past dwell distance
	float intoshoulder =  distanceoutside;										// depth of impingement into shoulder
	if (intoshoulder < 0) return(true);												// no impingement
	if ((!shoulder.m_valid) || (fabs(shoulder.m_offset) > fabs(offset)))	// if new impingement is worse
	{	shoulder.m_valid = true;															// update shoulder impingement
		shoulder.m_pos = impingement;
		shoulder.m_offset = offset*side;											// not at edge
		shoulder.m_passable = shoulder.m_possible = shoulder.m_unknown = false;
	}
	return(true);																				// no central hit
}
//
//	raster_ordered_trapezoid  -- test single trapezoid along the path.
//
bool CurvedPathBoundaryScanner::raster_ordered_trapezoid(const vec2 quad[4], const vec2& last_sforward, const vec2& next_sforward, 
	double distalongpath, float basewidth, float shoulderwidth)
{
	//	New, simplified version. Just does a single point test on each side.
	//	This must be called frequently, at least every half meter along the path.
	const vec2& qleft(quad[2]);									// test point on left
	const vec2& qright(quad[3]);								// test point on right
	//	Test left side
	float distanceoutside = distanceoutsidewaypoints(qleft, m_wp);	// get distance test point is outside waypoints
	if (!updatewaypointhit(m_wp, basewidth, shoulderwidth, (distalongpath > m_dwelldistance),  -1, 
			qleft, distanceoutside, m_result.m_center, m_result.m_left))
	{	return(false);	}										// found and marked hit, can quit scan
	//	Test right side
	distanceoutside = distanceoutsidewaypoints(qright, m_wp);
	if (!updatewaypointhit(m_wp, basewidth, shoulderwidth, (distalongpath > m_dwelldistance), 1, 
			qright, distanceoutside, m_result.m_center, m_result.m_right))
	{	return(false);	}										// found and marked hit, can quit scan
	return(true);														// no central hit
}
/**
  glue logic between this module and the rest of newsteer
*/
void scanPathAgainstWaypoints(const WaypointTriple& wp,
								  const CurvedPath& path, 
                                  const float basewidth,
                                  const float shoulderwidth,
                                  const float dwelldistance, 
                                  const float pathlength, 
                                  ImpingementGroup& impingements)
{
    // The correct value for num_segments depends heavily on the
    // shape of the curved wedge. As an approximation we use a single
    // constant. Optimally, we would want to calculate num_segments based
    // on some product of arclength and curvature.
    const float k_trapezoids_per_meter = 5.0;								// number of points to test per meter
    const int num_segments = int(pathlength * k_trapezoids_per_meter) + 1;
    impingements.clear();																// clear impingement info
    CurvedPathBoundaryScanner scanner(wp, impingements, dwelldistance*1.1);		// create scan object
    bool good = scanner.scan_wedge(path, basewidth, shoulderwidth, pathlength, num_segments);			// do the actual scan
    if (!good)																										// trouble
    {	impingements.m_center.m_valid = true;													// create phony collision at start
        impingements.m_center.m_pos = path.getstartpos();
        impingements.m_center.m_offset = 0;
        impingements.m_center.m_passable = true;
        impingements.m_center.m_unknown = true;
        impingements.m_center.m_possible= true;
	}
}
//
//	Dump utilities
//
//	dump  -- dump an impingement, with message
//
static void dump(char* msg, size_t msgl, const char* name, const ImpingementInfo& impingement)
{	if (!impingement.m_valid) 
	{	msg[0] = '\0';											// nothing to edit
		return;
	}
	//	Edit out the impingment info into a terse string.
	snprintf(msg, msgl," %s: (%1.2f, %1.2f) offset %1.2f %s%s%s", 
		name, 
		impingement.m_pos[0],
		impingement.m_pos[1],
		impingement.m_offset,	
		impingement.m_unknown ? " UNK" : "",
		impingement.m_passable ? " PASS" : "",
		impingement.m_possible ? " POSS" : "");	
}
//
//	dump  -- dump an impingement group on one line
//
//	A tight fit.
//
void dump(const char* msg, const ImpingementGroup& impingements)
{	const size_t k_msgl = 100;
	char msgcenter[k_msgl];
	char msgleft[k_msgl];
	char msgright[k_msgl];
	//	Edit center, left, right into working strings
	dump(msgcenter, k_msgl, "CENTER", impingements.m_center);
	dump(msgleft, k_msgl, "LEFT", impingements.m_left);
	dump(msgright, k_msgl, "RIGHT", impingements.m_right);
	logprintf("%s --%s%s%s\n", msg, msgcenter, msgleft, msgright);	
}
//
//	compareimpingement  -- compare two impingements
//
bool compareimpingement(const ImpingementInfo& i0, const ImpingementInfo& i1)
{	if (i0.m_valid != i1.m_valid) return(false);
	if (!i0.m_valid) return(true);			// no impingements
	if (fabs(i0.m_offset - i1.m_offset) > 0.1) return(false);
	if ((i0.m_pos - i1.m_pos).length2() > 0.1) return(false);
	return(true);
}


  
