//
//	curvedpathscanner.cpp  -- scan a curved path, dividing it into trapezoids
//	
//	Common code for both obstacle and boundary scanning
//	This module scans the interior of a curved wedge and tests for
//	collisions with the terrain map.
//
//	Originally by
// 
//	Paul Upchurch
//	Team Overbot
//	2005-03
 
//	TODO: tune the value of num_segments
//
//	Converted to spline-based paths by John Nagle
//
#include "curvedwedge.h"														// use common definitions
#include "curvedpath.h"
#include "logprint.h"
const double k_right_outside = -1.0;											// what used to be "outside" is now "left side"
/**
  Divides the wedge into quadrilateral segments and then checks
  for collisions by scanline rasterizing the quads. If no collision
  is found the metric returned in result will be less than zero.
  \param path the path to be tested (may be arc or spline)
  \param forward the unit vector in the direction of the wedge
  \param basewidth width of the wedge
  \param shoulderwidth width is basewidth+2*shoulderwidth
  \param n the number of segments
  \param result the collision is stored here
*/
bool CurvedPathScanner::scan_wedge(const CurvedPath& path, float basewidth, float shoulderwidth, float pathlength, int n)
{
    if(n<1)																				// bad trapezoid count - must fail
        return(false);
    if (pathlength < 0.01) return(true);									// nothing to do
    const float k_min_trapezoid_length = 1.0;									// trapezoid must be at least 1m long
    //		Get basic curve info
    const double arclen = path.getlength();							// length of the curve
    if (pathlength > arclen*1.1) 
    {	logprintf("ERROR in scan_wedge - path length %1.2f >  actual path length %1.2f.\n",
    		pathlength, arclen);
    	return(false);																// bogus path length
    }
    n = std::min(n, int(pathlength*k_min_trapezoid_length + 1));	// shorter paths need fewer trapezoids
    n = std::max(n,2);																// but always at least 2
	const vec2 pt(path.getstartpos());									// starting point
	const vec2 forward(path.getstartforward());					// forward direction
    const double hw=basewidth*0.5+shoulderwidth;			// halfwidth
	const vec2 right(forward[1], -forward[0]);						// unit vectors to left and right
    // compute the first two points of the first trapezoid
    vec2 quad[4];																	// corners of the trapezoid of interest
	quad[2] = pt - hw*right;													// corners of furthest end of next quad
	quad[3] = pt + hw*right;
	vec2 last_sforward(path.getstartforward());						// forward dir at beginning of quad
    // walk the parametric curved line for s in [0,1], where
    // s=0 corresponds to the start of the wedge, and
    // s=1 corresponds to the end of the wedge
    for(int i=0; i<n; i++)															
    {
        // setup
        double s=1.0*(double)(i+1)/(double)n;						// fraction into arc
        quad[0]=quad[3];														// current points becomes previous points
        quad[1]=quad[2];
		vec2 spnt, sforward;																// point and direction along arc
		double distalongpath = pathlength*s;									// distance along path
		bool good = path.pointalongpath(distalongpath, spnt, sforward);	// get point at appropriate point along arc
		if (!good) 																				// should not happen
		{	////***BAD*** force impingement now
			logprintf("ERROR: curvedpathscanner fail: quad %d of %d (fract=%1.4f), path length %1.2f m.\n",
				i,n, s, pathlength);
			return(false);															// fails
		}
		vec2 sright(sforward[1], -sforward[0]);								// direction to right
		quad[2] = spnt - hw*sright;													// left and right along boundaries
		quad[3] = spnt + hw*sright;											

		//	rasterize
		//	note that the vertices of the quad are ordered
		good = raster_ordered_trapezoid(quad, last_sforward, sforward, distalongpath, basewidth, shoulderwidth);
		if (!good) break;																	// early termination
        // save forward for calculating the sideways vector from mid-quad.
        last_sforward = sforward;
    }
    return(true);																			// success
}