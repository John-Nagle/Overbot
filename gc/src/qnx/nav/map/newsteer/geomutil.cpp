//
//	geomutil.cc  -- geometry utilities
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
#include "nan.h"
//
//	Constants
//
const float k_min_curvature = 0.001;						// less than this is a straight line
const double k_min_length = 0.00001;						// less than this, do not take reciprocal
//
//	parallelcurve  -- curvature for another arc concentric with the first
//
//	If offset is positive, the new arc is outside
//
float parallelcurve(float incurv, float offset)
{
	if (fabs(incurv) < 0.00001) return(0);		// handle straight case
	if (incurv < 0) offset = -offset;					// offset is to the outside
	float radius = 1/incurv;								// radius from curvature
	float offsetradius = radius + offset;			// offset radius
	if ((fabs(offsetradius) < 0.00001)				// trouble.  Too tight for offset
	|| ((offsetradius > 0) != (radius > 0)))		// offset so large that arc is on wrong side of center
	{	return(k_NaN);	}									// fails
	return(1/offsetradius);								// valid
}
//
//	pointalongarc  -- project point along arc
//
//	Positive curvature means to the right.
//
//	***NEED CHECK FOR MORE THAN A SEMICIRCLE***
//
vec2 pointalongarc(const vec2& pt, const vec2& forward, const float curvature, double dist, vec2& endforward)
{
	if (fabs(curvature) < k_min_curvature)					// if a straight line
	{	endforward = forward;										// tangent at end is same as at start
		return(pointalongline(pt, pt, pt+forward, dist)); // project along straight line
	}
	if (dist < 0)															// treat negative distances as zero-length arcs
	{	endforward = forward;
		return(pt);
	}
	//	Curved line case
	double radius = 1.0/curvature;								// radius of arc, positive if to right
	vec2 side(forward[1], -forward[0]); 					// sideways vector (to right)
	vec2 center(pt+side*radius);								// center of circle
	////double circlefract = dist/(M_PI*2*radius);			// fraction of a circle
	////double arcangle = circlefract*M_PI*2;				// angle in radians
	double arcangle = dist/radius;								// angle of arc, in radians, positive if clockwise
	double ptangle = anglefromvector(pt - center);	// starting angle
	double pangle = ptangle - arcangle;					// ending angle
	vec2 p(cos(pangle), sin(pangle));						// vector from center towards ending point
	endforward = vec2(p[1], -p[0]);							// vector forward from ending point
	endforward.normalize();										// unit vector
	if (curvature < 0) 												// if left turning arc
	{	endforward = -endforward;	}							// point forward, not backwards
	p = p*fabs(radius) + center;								// ending point
	return(p);
}
//
//	tangentarcthroughpoints  -- arc through p0 and p1, with tangent direction forward at p0
//
bool tangentarcthroughpoints(const vec2& p0, const vec2& forward, const vec2& p1, float& curvature)
{
	vec2 togoal(p1 - p0);															// vector from vehicle position to goal
	double d = togoal.length();													// distance to goal
	if (d < 0.0001) return(false);													// at goal, nothing to do
	togoal *= (1/d);																	// normalize vector
	double costurnangle = togoal*forward;								// cos of desired turn angle (unsigned)
	if (costurnangle <= 0)															// if goal behind us, fails.
	{	
		return(false);																	// this should catch arcs of more than a semicircle
	}
	costurnangle = std::max(std::min(costurnangle,1.0),-1.0);	// avoid trouble in acos
	double turnangle = acos(costurnangle);								// turn angle
	if (fabs(turnangle) < 0.001)													// if tiny turnangle
	{	curvature = 0; return(true);	}											// no turn
	double alpha = (M_PI*0.5) - turnangle;									// base angle of triangle
	//	Now we have an isosceles triangle, with base angles alpha and base d. We want the
	//	length of the two equal sides of the triangle. That's the turning radius.	
	double beta = 0.5*(M_PI-2*alpha);										// half-angle of vertex of isoceles triangle
	//	Now we have a right triangle, with base d/2, hypotenuse r, and angle beta.
	//	so sin(beta) = (d/2)/r
	//	and so r = (d/2)/sin(beta)
	//	and 1/r = sin(beta)/(d/2) = 2*sin(beta)/d;
	curvature = 2*sin(beta)/d;													// calc curvature
	//	Finally, we must decide whether we're turning right or left.
	vec2 side(forward[1], -forward[0]);										// perpendicular
	if (togoal*side < 0)																// if left turn
	{	curvature = -curvature;	}												// reverse dir
	return(true);
}
//
//	arclength -- compute length of an arc
//
double arclength(const vec2& p0, const vec2& p1, float curvature)
{
	if (fabs(curvature) < k_min_curvature)								// if straight line
	{	return((p1-p0).length());	}											// length
	//	Curved arc, must do real work.
	const double r = 1.0 / curvature;										// radius of arc
	vec2 chord(p1-p0);															// chord
	double c = chord.length();												// length of chord
	if (c < k_min_length) return((p1-p0).length());					// very short chord, use linear length
	double theta = 2*asin(c/(2*r));											// base angle of arc
	//	***MORE*** trouble if more than 180 degrees
   	double s = r*theta;															// arc length
   	return(s);
}
//
//	arcmidpoint  -- compute midpoint of an arc
//
//	Ref: "Ask Dr. Math" (http://mathforum.org/dr.math/faq/faq.circle.segment.html), case 7
//
vec2 arcmidpoint(const vec2& p0, const vec2& p1, float curvature)
{	const vec2 midpoint((p0+p1)*0.5);									// midpoint of chord
	if (fabs(curvature) < k_min_curvature)								// if straight line
	{	return(midpoint);	}														// just average points
	//	Curved arc, must do real work.
	const double r = fabs(1.0 / curvature);								// radius of arc
	vec2 chord(p1-p0);															// chord
	double c = chord.length();												// length of chord
	if (c < k_min_length) return(midpoint);								// very short chord, use other
	double theta = 2*asin(c/(2*r));											// base angle of arc
	//	***MORE*** trouble if more than 180 degrees
	double d = r*cos(theta/2);												// distance from center to chord
	double h = r - d;																// distance from chord to midpoint
	vec2 chordvector(chord/c);												// chord direction unit vector
	vec2 radial(chordvector[1], -chordvector[0]);					// radial to right of chord
	if (curvature > 0) radial = -radial;										// radial is unit vector from center towards midpoint	
	return(midpoint + radial*h);												// return midpoint					
}
