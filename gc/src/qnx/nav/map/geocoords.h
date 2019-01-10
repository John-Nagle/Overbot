//
//	geocoords.h -- geometry of GPS/INS fixes
//
//	Useful conversion routines, some of which are non-obvious.
//
//
//	John Nagle
//	Team Overbot
//	February, 2004
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
#ifndef GEOCOORDS_H
#define GEOCOORDS_H

#include "algebra3.h"
#include "Nav.h"
class GPSINSMsgRep;																// forward

//	Convert GPS fix to vehicle pose matrix
void posefromfix(const GPSINSMsgRep& fix, mat4& vehpose);

//	Convert XYZ to NED coordinate systems
vec3 XYZfromNED(const Vector<3>& ned);								// using Vector input
vec3 XYZfromNED(const double ned[3]);									// using array input

//	Convert LLH to XYZ coordinate systems
vec3 XYZfromLLH(const Vector<3>& llh, const Vector<3>& llhorigin);
vec3 XYZfromLLH(const double llh[3], const Vector<3>& llhorigin);

//	convert feet to meters
inline double feet2meters(double s)	{ return ((s) * 12.0 * 2.54 / 100.);	}
//	convert mph to meters/s
inline double mph2metric(double s) {	return ((s) * 5280.0 * 12.0 / 3600.0 * 2.54 / 100.);	}
//	convert degrees to radians
inline double deg2radians(double s) { return(s*(M_PI/180.0));	}
//	convert radians to degrees
inline double radians2deg(double s) { return(s*(180.0/M_PI));	}
//
//	Pose interpolation
//
bool interpolateposedumb(const mat4& prevpose, const uint64_t prevtime, const mat4& nextpose, const uint64_t nexttime, 
	const uint64_t desiredtime, mat4& pose);
bool interpolateposeSLERP(const mat4& prevpose, const uint64_t prevtime, const mat4& nextpose, const uint64_t nexttime, 
	const uint64_t desiredtime, mat4& pose);


#endif // GEOCOORDS_H
