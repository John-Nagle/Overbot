//
//	curvedpath.cpp  -- generic functions for all subclasses of CurvedPath
//
//	John Nagle
//	Team Overbot
//	August, 2005
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
#include "curvedpath.h"
#include "logprint.h"
#include "geomutil.h"
//
//	steertopath  -- steer to follow indicated path
//
//	Currently, we always take the initial curvature of the generated path.
//	This is meaningful for arcs and dual arcs.
//	But we need to be careful that the path doesn't have some wierd initial curvature due to
//	a boundary condition.
//
//	Tries to compute the curvature that will hit a point one vehicle length out. This compensates for
//	initial S-curve issues.
//
//	Must return a best-effort curvature value even if it returns false, so that steering during fault stops is sane.
//
//
bool CurvedPath::steertopath(float& curvature, float lookaheaddist)
{
	const float k_min_check_dist = 1.0;														// look at least 1m ahead
	////const float k_steer_to_path_vehicle_lengths = 2.0;							// steer to this many vehicle lengths out
	//////	***FIX*** don't have info for this
	////double curvdist = getSteeringDwellDistance() + k_steer_to_path_vehicle_lengths*m_vehicledim[1];	// get curvature from this far out
	lookaheaddist = std::max(lookaheaddist, k_min_check_dist);				// look at least this far ahead
	lookaheaddist = std::min(lookaheaddist, float(getlength()));					// don't go off end of path
	//	Desired path is an arc through the current vehicle position, tangent to the current
	//	direction of travel, and through the specified point on the path. Calculate curvature.
	vec2 goalpt, endforward;																		// get goal point we're trying to steer to
	bool good = pointalongpath(lookaheaddist, goalpt, endforward);		// get point along path
	if (!good)																								// probably has U-turn or some such stupidity
	{	logprintf("steerToPath: Can't get point %1.2f m along path.\n",lookaheaddist);	// should not happen
		return(false);
	}
	good = tangentarcthroughpoints(getstartpos(), getstartforward(), goalpt, curvature);
	if (!good)																								// probably has U-turn or some such stupidity
	{	logprintf("steerToPath: Can't construct a steering angle for this path.\n");
		return(false);
	}
	if (curvature > getmaxcurv() || curvature < getmincurv())	// limit to safe limits
	{	logprintf("ERROR: commanded curvature %1.4f outside safe limits [%1.4f .. %1.4f].\n",
			curvature, getmincurv(), getmaxcurv());
		curvature = std::max(std::min(curvature, getmaxcurv()), getmincurv());		// bound during fault stops
	}
	return(true);
}
//	
//	dump -- dump key params to log file, with message
//
void CurvedPath::dump(const char* msg) const
{	logprintf("%s: path from (%1.2f, %1.2f) to (%1.2f, %1.2f), end dir (%1.2f, %1.2f) curv. %1.4f\n",
		msg, getstartpos()[0], getstartpos()[1], getendpos()[0], getendpos()[1],  getendforward()[0], getendforward()[1],getcurvatureat(0));	// dump path info
}