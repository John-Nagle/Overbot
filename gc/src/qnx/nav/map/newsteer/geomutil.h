//
//	geomutil.h  -- non-class geometry utilities
//
//	Used by NewSteer
//
//	Replacement for OpenSteer
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
#ifndef GEOMUTIL_H
#define GEOMUTIL_H
#include <vector>
#include "algebra3.h"
//
//	Support functions.  Mostly geometry
//
float parallelcurve(float incurv, float offset);				// curvature parallel to a given curvature. Returns NaN on fail
//	pointtolinesegment  -- distance between point and line segment
double pointtolinesegmentdistance(const vec2& p, const vec2& p0, const vec2& p1, double& U, vec2& intersect);
//	pointalongline  -- project along line line starting at pt for distance dist.
vec2 pointalongline(const vec2& pt, const vec2& p0, const vec2 p1, double dist);
//	pointalongarc  -- project along arc starting at pt for distance dist
vec2 pointalongarc(const vec2& pt, const vec2& forward, const float curvature, double dist, vec2& endforward);
double anglefromvector(const vec2& v);							// angle from vector
bool tangentarcthroughpoints(const vec2& p0, const vec2& forward, const vec2& p1, float& curvature);
double arclength(const vec2& p0, const vec2& p1, float curvature);
vec2 arcmidpoint(const vec2& p0, const vec2& p1, float curvature);

//
//	Implementations of the smaller functions
//
//
//	pointalongline  -- project along line line starting at pt for distance dist.
//
inline vec2 pointalongline(const vec2& pt, const vec2& p0, const vec2 p1, double dist)
{
   	return(pt + (p1-p0)*(dist/(p1-p0).length()));						// calc position along ine
}
//
//	pointtolinesegmentdistance  -- signed distance between point and line segment
//
//	Output is SIGNED distance.  distance < 0 means on LEFT of line p0..p1
//
inline double pointtolinesegmentdistance(const vec2& p, const vec2& p0, const vec2& p1, double& U, vec2& intersect)
{
    double linemagsq = (p1-p0).length2();							// length of segment, squared
    if (linemagsq < 0.00000001) 										// if identical points
    {	U = 0;																		// fraction along line not meaningful.
    	intersect = p0;															// intersection not meaningful
    	return((p-p0).length());												// distance from point, avoiding div by zero
    }											
    vec2 d0(p-p0);																// start of line segment to p as a vector
    vec2 d1(p1-p0);															// the line segment as a vector
    vec2 toright(d1[1], -d1[0]);											// perpendicular vector to right of line segment
    double sign = (toright * d0) > 0 ? 1 : -1;						// -1 if p0 is on left of p0..p1, else 1
    U = (d0[0]*d1[0] + d0[1]*d1[1]) / linemagsq;				// fraction along line
    intersect = p0 + U*(p1-p0);											// calc intersection point
    if (U < 0)																		// if off end at p0 end
    {	return(sign*((p-p0).length()));									// distance from p0
    }											
    if (U > 1)																		// if off end at p1 end
    {	return(sign*((p-p1).length()));									// distance from p1
    }											
    return(sign*((p-intersect).length()));								// return signed distance from intersection
}
//
//	anglefromvector -- angle from vector
//
inline double anglefromvector(const vec2& v)
{	return(atan2(v[1], v[0]));	}											// this is the arctangent
//
//	lineintersection -- Intersection of two lines
//
//	The lines are p1 -- p2, and p3 -- p4.
//
//	after Pascal code at "http://www.pdas.com/lineint.htm"
//
inline bool lineintersection(const vec2 p1, const vec2 p2, const vec2 p3, const vec2& p4, vec2& intersection)
{
	const double a1 = p2[1] - p1[1];							// y2-y1;
	const double b1 = p1[0] - p2[0];							// x1-x2;
	const double c1 = p2[0]*p1[1] - p1[0]*p2[1];		// x2*y1 - x1*y2;		 a1*x + b1*y + c1 = 0 is line 1 

	const double a2 = p4[1] - p3[1];							// y4-y3;
	const double b2 = p3[0] - p4[0];							// x3-x4;
	const double c2 = p4[0]*p3[1] - p3[0]*p4[1];		// x4*y3 - x3*y4;		a2*x + b2*y + c2 = 0 is line 2  

	const double denom = a1*b2 - a2*b1;					// determinant
	if (fabs(denom) < 0.0001) return(false);				// avoid divide by zero

  	const double x = (b1*c2 - b2*c1)/denom;			// compute intersection point
  	const double y = (a2*c1 - a1*c2)/denom;
  	intersection = vec2(x,y);
  	return(true);
}

#endif // GEOMUTIL_H
