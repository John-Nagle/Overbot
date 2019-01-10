//
//	arctosegment.cpp  -- more geometry utilities
//
//	John Nagle
//	Team Overbot
//	May, 2005
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
#include "algebra3.h"
#include "geomutil.h"
//
//	updatedist  --  update max distance
//
//	For arctosegmentmaxdist below.
//
static void updatedist(double& distsq, vec2& ppt, vec2& qpt, const vec2& p, const vec2& q)
{
	double newdistsq = (p-q).length2();				// distance squared
	if (newdistsq > distsq)							// if new winner
	{	distsq = newdistsq;							// update distance
		ppt = p;									// save new relevant points
		qpt = q;
	}
}
//
//	isonarc -- is a point within the arc given?
//
//	The arc starts at q0, ends at q1, has center c, and curvature curv.
//	q should be on the circle centered at c with curvature curv, but
//	may or may not be within the arc points.
//
//	NOTE: assumes arc is not greater than a half circle.
//
static bool isonarc(const vec2& q, const vec2& q0, const vec2& q1, const vec2& c, float curv)
{
	return(((q1-q0) * (q-q0) > 0.0) 
		&& ((q1-q0) * (q-q1) < 0.0));			// vectors consistent for "between"
}
//
//	segmenttoarcmaxdist  --  "maximum distance" between line segment and arc.
//
//	"Maximum distance" here is defined as follows:
//		For an arc q0 -- q1, with center c, and
//		a segment p0 -- p1,
//		For each point q on q0 -- q1, find the nearest
//		point p on p0 -- p1.  For all pairs (p, q),
//		pick the one with the longest distance between p and q.
//
//	The longest distance will either be to the endpoint of the arcs.
//	or to the points on a line perpendicular to p0 -- p1 and
//	through c.
//
//	NOTE: assumes arc is not greater than a half circle.
//
double arctosegmentmaxdist(const vec2& p0, const vec2& p1, const vec2& q0, const vec2& q1, const vec2& c, const float curv,
							vec2& ppt, vec2& qpt)
{

	double distsq = -9999999999.0;						// really should use -inf.
	//	Endpoint cases
	double U;											// fraction along segment
	vec2 p;												// nearest point on p0 -- p1
	pointtolinesegmentdistance(q0, p0, p1, U, p);		// find nearest point (pwork) on segment
	updatedist(distsq, ppt, qpt, p, q0);				// get distance to arc endpoint
	pointtolinesegmentdistance(q1, p0, p1, U, p);		// find nearest point (pwork) on segment
	updatedist(distsq, ppt, qpt, p, q1);				// get distance to arc endpoint
	//	Perpendicular case
	if ((fabs(curv) > 0.00001) && ((p1-p0).length2() > 0.000001))	// if not straight line, and non-null case
	{	pointtolinesegmentdistance(c, p0, p1, U, p);	// find nearest point to center of arc
		if (U > 0.0 && U < 1.0)							// if not at an endpoint
		{
			//	Find furthest point on arc from p.
			//	This is perpendicular to p0 -- p1, from p, distance radius.
			double radius = fabs(1.0/curv);				// radius of arc circle (unsigned)
			vec2 forward(p1 - p0);						// segment direction, null case checked above
			forward.normalize();						// unit vector in segment direction
			vec2 right(forward[1], -forward[0]);		// unit vector to right of segment
			vec2 qa(c + right*radius);					// point on circle, may be on arc
			if (isonarc(qa, q0, q1, c, curv))			// check if point is within arc
			{	updatedist(distsq, ppt, qpt, p, qa);}	// get distance to arc point
			vec2 qb(c - right*radius);					// point on circle, may be on arc
			if (isonarc(qa, q0, q1, c, curv))			// check if point is within arc
			{	updatedist(distsq, ppt, qpt, p, qb);}	// get distance to arc point
		}
	}
	return(sqrt(distsq));								// return final distance
}

