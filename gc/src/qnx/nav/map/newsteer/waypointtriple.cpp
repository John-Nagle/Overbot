//
//	waypointtriple.cpp  -- find relevant waypoint triple.
//
//	John Nagle
//	Team Overbot
//	September, 2005
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
#include "logprint.h"
//
//	We must find a waypoint triple that
//	-- contains the vehicle
//	-- or, if the vehicle is not inside any triple, is the nearest one
//	-- if the vehicle is entirely within a single waypoint, has the next waypoint in the direction of travel
//
//	In addition, when the vehicle is within more than one waypoint segment
//	-- we must choose a waypoint sequential with the current waypoint if possible
//	-- except that we may be inside non-sequential waypoints (as at waypoints 630-632 of the 2004 GC course)
//	-- When we are entirely within a waypoint, but close to its end, we must ignore previous waypoints
//		and look ahead to the next waypoint
//
//	So the basic cases are:
//	1.	Center of vehicle is not inside any waypoint segment. (off course)
//		Find nearest waypoint segment. Use it. Return only one segment. 
//	2.	Center of vehicle is inside exactly one waypoint segment (simplest case)
//		That waypoint segment is used, plus the next segment in the direction of travel. Return two segments.
//	3.	Center of vehicle is inside exactly two waypoint segments and the segments are sequential. (in turn)
//		Return those two waypoint segments, EXCEPT that if entirely inside second segment in direction of travel and
//		close to its end, drop first segment and proceed as for case 2.
//	4.	Center of vehicle is inside exactly two waypoint segments and the segments are not sequential. (course intersects self)
//		Drop segment which is not sequential with incoming waypoint triple and proceed as in case 2.
//		***WRONG - won't handle waypoints 630-632 of 2004 GC course.
//	5.	Center of vehicle is inside more than two waypoint segments (the hard cases).
//		
//		-- Find last waypoint segment (in direction of travel) that vehicle is inside
//		-- Check whether vehicle is entirely within that segment
//		
//
//	Constants (may be duplicates)
//
const Tuneable k_min_move("MINMOVE",0.1, 3.0, 2.0, "Minimum move dist (m)");
#define NEWWAYPOINTAPPROACH 1
#if NEWWAYPOINTAPPROACH
//
//	getRelevantSegmentPair  -- get the two waypoints relevant now
//
//	New algorithm:
//
//		Find waypoint segment we are "most inside".
//		Add next waypoint in direction we are going.
//
//	The one we are in, and the next one.
//
//	Distance is distance to end of next waypoint straight section or end of turn.
//	inoverlap is true if we are within both waypoints, i.e. in a turn.
//
//	If the course is a closed course, waypoint 1 may appear as both the first and last waypoint.  This is a signal
//	that it's a closed course and we're near the close point. 
//
//	Assumes incoming waypoints are in ascending order (including wrap) but may not be contiguous.
//
//	Handles being off course.  Returns closest waypoint.
//	Does NOT yet handle a course that crosses itself.
//
bool NewSteer::getRelevantSegmentPair(const ActiveWaypoints& waypoints, WaypointTriple& wpts)
{	const bool closedcourse = waypoints.getClosed();		// true if closed course
	const int currentturnnum = m_currentwaypoint.m_turnnum;		// the current "turn number"
	const int maxturnnum = waypoints.getLastTurnNum();			// the largest "turn number"
	int nextturnnum = currentturnnum + 1;						// previous turn number
	int prevturnnum = currentturnnum - 1;							// previous turn number
	if (closedcourse && nextturnnum > maxturnnum) nextturnnum = 0;	// handle wrap
	if (closedcourse && prevturnnum < 0) prevturnnum = maxturnnum;	// handle wrap
	wpts[0] = wpts[1] = wpts[2] = k_null_waypoint;			// no waypoints yet
	double bestdistoutside = 9999999999.0;						// big value, way outside
    //	Full search of relevant waypoint pair list.  Find waypoint pairs we are most inside.
    for (size_t i=0; i < waypoints.size()-1; i++)					// for waypoint pairs, including closing link at end if closed course
    {	
    	const Waypoint& w0 = waypoints[i];						// the current waypoint pair of interest
        const Waypoint& w1 = waypoints[(i+1)];
        if ((w1.m_serial != w0.m_serial+1)							// if not a connected pair
       	&& (!closedcourse || (w1.m_serial != 1) || w0.m_serial != waypoints.getLastSerial())) 	// special closed-course case, waypoint 1 has a predecessor
       	{  continue;																// not in sequence, skip pair
       	}
       	//	We have a set of sequential waypoints.
       	//	Closed course ambiguity check. Check that waypoints are from the same or adjacent "turn numbers".
       	const int turnnum = w0.m_turnnum;							// this turn number
       	//	Turn numbers can wrap for a closed course. 
       	if (!(turnnum == currentturnnum) && !(turnnum == prevturnnum) && !(turnnum == nextturnnum))
       	{
       		////logprintf("Waypoint %d is from turn number %d which is too far from current turn number %d\n",
       		////		w0.m_serial, turnnum, currentturnnum);		// ***TEMP***
       		continue;
		}
        double disttostart, disttoend;									// sinks
		vec2 centerlinept;
		int side;
		double distoutside = distanceoutsidewaypoint(m_inposition, w0, w1, disttostart, disttoend, side, centerlinept);
        if (distoutside < bestdistoutside)								// if closer than previous winner
        {	wpts[0] = w0;														// save new winner
        	wpts[1] = w1;
        	bestdistoutside = distoutside;								// value of new winner
        }
   }
    //	Done with search; analyze result.
 	if (nullwaypoint(wpts[0])) return(false);					// no find at all, fails
 	//	Turn numbers can wrap for a closed course. 
    const int turnnum = wpts[0].m_turnnum;						// this turn number
   	if (!(turnnum == currentturnnum) && !(turnnum == prevturnnum) && !(turnnum == nextturnnum))
	{	logprintf("Waypoint %d is from turn number %d which is too far from current turn number %d\n",
       					wpts[0].m_serial, turnnum, currentturnnum);	// warning message
  	}
 	//	Need another waypoint to fill out the set of 3.  Search for next in sequence and use it.
 	int nextwaypoint = -1;
 	if (m_indir > 0)															// if going forward
 	{	nextwaypoint = wpts[1].m_serial + 1;
 		if (closedcourse && nextwaypoint > waypoints.getLastSerial()) nextwaypoint = 1;	// handle closed course
 	} else {																	// if going backwards
 		nextwaypoint = wpts[0].m_serial - 1;					// get previous waypoint number
 		std::swap(wpts[0], wpts[1]);									// sway first two, so we get a decreasing sequence
 		if (closedcourse && nextwaypoint == 0) nextwaypoint = waypoints.getLastSerial();	// handle closed course
 	}
 	for (size_t i=0; i<waypoints.size(); i++)					// search waypoints for desired waypoint number
 	{	const Waypoint& wp = waypoints[i];					// looking at this waypoint
 		if (wp.m_serial != nextwaypoint) continue;			// no match
 		//	Have match
 		wpts[2] = wp;														// third in sequence
 	}
 	if (getVerboseLevel() >= 2)											// if very verbose
 	{	logprintf("Active waypoint triple: %d %d %d\n", wpts[0].m_serial, wpts[1].m_serial, wpts[2].m_serial); }
 	m_outwaypoint = wpts[0].m_serial;								// save for debug
 	return(true);																	// success, at least one pair
}
#else			// old approach
//
//	insideWaypoint  -- true if pos is inside specified waypoint segment.
//
//	Also returns distance to starting and ending waypoints along the centerline, which is negative if in the end caps.
//
static bool insideWaypoint(const vec2& pt, const Waypoint& wp0, const Waypoint& wp1, double& disttostart, double& disttoend)
{	int side;
	vec2 centerlinept;
	double dist = distanceoutsidewaypoint(pt, wp0, wp1, disttostart, disttoend, side, centerlinept);
	return(dist < 0);
}
//
//	getRelevantSegmentPair  -- get the two waypoints relevant now
//
//	***OLD ALGORITHM BEFORE AUGUST DARPA CHANGES TO RULES***
//
//	The one we are in, and the next one.
//
//	Distance is distance to end of next waypoint straight section or end of turn.
//	inoverlap is true if we are within both waypoints, i.e. in a turn.
//
//	If the course is a closed course, waypoint 1 may appear as both the first and last waypoint.  This is a signal
//	that it's a closed course and we're near the close point. 
//
//	Assumes incoming waypoints are in ascending order (including wrap) but may not be contiguous.
//
//	If off course, will not get vehicle back on course.
//
bool NewSteer::getRelevantSegmentPair(const ActiveWaypoints& waypoints, WaypointTriple& wpts)
{	
	if (m_indir >= 0)
	{	return(getRelevantSegmentPairFwd(waypoints, wpts));	}	// forward direction waypoint set
	else
	{	return(getRelevantSegmentPairRev(waypoints, wpts));		}	// reverse direction waypoint set
}
//
//	getRelevantSegmentPairFwd  -- get the two waypoints relevant now
//
//	The one we are in, and the next one.
//
//	If the course is a closed course, waypoint 1 may appear as both the first and last waypoint, so that we
//	always have only valid sequential pairs.
//
//	Assumes incoming waypoints are in ascending order (including wrap) but may not be contiguous.
//
//	Valid for forward travel only. 
//	If seriously off course, will not get vehicle back on course.
//
bool NewSteer::getRelevantSegmentPairFwd(const ActiveWaypoints& waypoints, WaypointTriple& wpts)
{
	bool closedcourse = waypoints.getClosed();				// true if closed course
	wpts[0] = wpts[1] = wpts[2] = k_null_waypoint;			// no waypoints yet
    //	Full search of relevant waypoint pair list.  Find waypoint pairs we are inside.
    for (size_t i=0; i < waypoints.size()-1; i++)					// for waypoint pairs, including closing link at end if closed course
    {	
    	const Waypoint& w0 = waypoints[i];						// the current waypoint pair of interest
        const Waypoint& w1 = waypoints[(i+1)];
        if ((w1.m_serial != w0.m_serial+1)							// if not a connected pair
       	&& (!closedcourse || (w1.m_serial != 1) || w0.m_serial != waypoints.getLastSerial())) 	// special closed-course case, waypoint 1 has a predecessor
       	{  continue;																// not in sequence, skip pair
       	}
       	//	We have a set of sequential waypoints.
        double disttostart, disttoend;									// sinks
        if (insideWaypoint(m_inposition, w0, w1, disttostart, disttoend))	// if find
        {	//	We are inside this waypoint.
        	//	We want the LAST waypoints we are inside.
        	//	Add new waypoint pair, shifting down.
        	if (nullwaypoint(wpts[2]) || (wpts[2].m_serial == w0.m_serial))	// if OK to add new one at end
        	{	wpts[0] = wpts[1]; }										// shift old waypoint down one
        	else if (closedcourse && !nullwaypoint(wpts[2]) && (w1.m_serial == wpts[1].m_serial) && (w1.m_serial == 1))
        	{	//	Closed course wrap point.  We have waypoints 1,2, and are adding N,1.
        		wpts[0] = w0;													// add new one at beginning
        		continue;															// done
        	} else {
       			//	Special case for being inside non-sequential waypoints.
       			//	This can happen in wierd overlap cases, such as waypoints 630-632 in the 2004 Grand Challenge RDDF.
        		//	At this point, we have to completely replace the waypoints we've got.
        		wpts[0] = k_null_waypoint;								// lose old out-of-sequence waypoint
        	}
        	wpts[1] = w0;														// add new segment at end
        	wpts[2] = w1;
        	//	Check for multiple waypoint overlap problem.
        	//	When waypoints are very close together, we may have to drop the previous waypoint, since we only 
        	//	look at two waypoint segments at a time. This has risk; if the previous waypoint is bigger than
        	//	the current one, it can seem to place us off course.
        	if (!nullwaypoint(wpts[0]) && disttostart > 0 && disttoend < (k_min_move*1.5))	
        	{	logprintf("Getting very close (%1.2f m) to waypoint %d, dropping waypoint %d.\n", 
        			disttoend, wpts[2].m_serial, wpts[0].m_serial);
        		wpts[0] = k_null_waypoint;
        	}
        	continue;
		}
    }
    //	Done with search; analyze result.
 	if (nullwaypoint(wpts[2])) return(false);						// no find at all, fails
 	if (nullwaypoint(wpts[0]))												// only have one segment, add on one we are not in, if any
 	{	wpts[0] = wpts[1];													// shift down by one
 		wpts[1] = wpts[2];
 		wpts[2] = k_null_waypoint;
 		//	Need another waypoint to fill out the set of 3.  Search for next in sequence and use it.
 		{	for (size_t i=0; i<waypoints.size(); i++)				// search waypoints for desired waypoint number
 			{	const Waypoint& wp = waypoints[i];				// looking at this waypoint
 				if ((wp.m_serial == wpts[1].m_serial + 1)			// if next in sequence
 				|| (closedcourse && wpts[1].m_serial == waypoints.getLastSerial() && wp.m_serial == 1)) // or next in sequence, wrapped
 				{	wpts[2] = wp;												// use it
 					break;															// done
 				}
 			}
 		}
 	}
 	if (getVerboseLevel() >= 2)											// if very verbose
 	{	logprintf("Active waypoint triple: %d %d %d\n", wpts[0].m_serial, wpts[1].m_serial, wpts[2].m_serial); }
 	m_outwaypoint = wpts[0].m_serial;								// save for debug
 	return(true);																	// success, at least one pair
}
//
//	getRelevantSegmentPairRev  -- get the two waypoints relevant now
//
//	The one we are in, and the previous one.
//
//	If the course is a closed course, waypoint 1 may appear as both the first and last waypoint.  This is a signal
//	that it's a closed course and we're near the close point. 
//
//	Assumes incoming waypoints are in ascending order (including wrap) but may not be contiguous.
//
//	Valid for reverse travel only. 
//	If off course, will not get vehicle back on course.
//
bool NewSteer::getRelevantSegmentPairRev(const ActiveWaypoints& waypoints, WaypointTriple& wpts)
{
	bool closedcourse = waypoints.getClosed();				// true if closed course
	wpts[0] = wpts[1] = wpts[2] = k_null_waypoint;			// no waypoints yet
    //	Full search of relevant waypoint pair list.  Find waypoint pairs we are inside.
    for (size_t i=0; i < waypoints.size()-1; i++)						// for waypoint pairs, including closing link at end if closed course
    {	
    	const Waypoint& w0 = waypoints[i];						// the current waypoint pair of interest
        const Waypoint& w1 = waypoints[(i+1)];
        if ((w1.m_serial != w0.m_serial+1)							// if not a connected pair
       	&& (!closedcourse || (w1.m_serial != 1) || w0.m_serial != waypoints.getLastSerial())) 	// special closed-course case, waypoint 1 has a predecessor
       	{  continue;																// not in sequence, skip pair
       	}
       	//	We have a set of sequential waypoints.
        double disttostart, disttoend;									// sinks
        if (insideWaypoint(m_inposition, w0, w1, disttostart, disttoend))	// if find
        {	//	We are inside this waypoint.
        	//	We want the FIRST waypoints we are inside.
        	//	Add new waypoint pair, shifting down.
			if (!nullwaypoint(wpts[0])) continue;					// if we have three, we're done.
        	if (nullwaypoint(wpts[2]) || (wpts[2].m_serial == w0.m_serial))	// if OK to add new one at end
        	{	wpts[0] = wpts[1]; }										// shift old waypoint down one
        	wpts[1] = w0;														// add new segment at end
        	wpts[2] = w1;
 		}
    }
    //	Done with search; analyze result.
 	if (nullwaypoint(wpts[2])) return(false);						// no find at all, fails
 	if (nullwaypoint(wpts[0]))												// only have one segment, add on one we are not in, if any
 	{	wpts[0] = wpts[1];													// shift down by one
 		wpts[1] = wpts[2];
 		wpts[2] = k_null_waypoint;
 	}
	//	Waypoints are in ascending order, and we have 0 and 1, and may have 2.  Reverse them.
	{	Waypoint tmp = wpts[0];								// swap
		wpts[0] = wpts[2];
		wpts[2] = tmp;
	}
	//	Waypoints are now in descending order. We have 1 and 2, and may have 0.
	//	 If only have two, get a third one.
 	if (nullwaypoint(wpts[0]))												// only have one segment, add on one we are not in, if any
 	{	wpts[0] = wpts[1];													// shift down by one
 		wpts[1] = wpts[2];
 		wpts[2] = k_null_waypoint;
 		//	Need another waypoint to fill out the set of 3.  Search for previous in sequence and use it.
 		{	for (size_t i=0; i<waypoints.size(); i++)				// search waypoints for desired waypoint number
 			{	const Waypoint& wp = waypoints[i];				// looking at this waypoint
 				if ((wp.m_serial + 1 == wpts[1].m_serial)			// if previous in sequence
 				|| (closedcourse && wpts[1].m_serial == 1 && wp.m_serial == waypoints.getLastSerial())) // or next in sequence, wrapped
 				{	wpts[2] = wp;												// use it
 					break;															// done
 				}
 			}
 		}
 	}
 	if (getVerboseLevel() >= 2)								// if very verbose
 	{	logprintf("Active waypoint reverse triple: %d %d %d\n", wpts[0].m_serial, wpts[1].m_serial, wpts[2].m_serial); }
 	m_outwaypoint = wpts[0].m_serial;					// save for debug
 	return(true);														// success, at least one pair
}
#endif // old approach
