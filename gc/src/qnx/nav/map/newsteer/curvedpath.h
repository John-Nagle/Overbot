//
//	curvedpath.h  -- spline path support
//
//	John Nagle
//	Team Overbot
//	May, 2005
//
//	Arcs are not enough. Sometimes we need to exit a turn going in a specific direction.
//	This requires splines.
//
//	Splines with four control points are sufficient. We need to specify position and
//	direction at turn entry and turn exit.
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
#ifndef CURVEDPATH_H
#define CURVEDPATH_H

#include "algebra3.h"													// for vectors
#include "logprint.h"														// for errors
//
//	class CurvedPath  -- base class for SpinePath, ArcPath, etc.
//
class CurvedPath
{
protected:
public:
	virtual ~CurvedPath() {};																	// destructor
	//	Units:
	//		Curvatures are 1/turning radius.
	//		Speed is meters per second.
	//		Curvature rate is rate of curvature change per unit time.
	virtual void setstart(const vec2& pos, const vec2& forward, const float speed, 
				const float curvature, const float initialcurvaturedist, 
				const float mincurvature, const float maxcurvature, const float maxcurvaturerate, const float lookaheaddist) = 0;
	virtual bool setend(const vec2& pos) = 0;											// specify end of path, direction is don't care
	virtual bool setend(const vec2& pos, const vec2& forward) = 0;		// specify end of path, direction is forward
	virtual double getlength() const  = 0;													// get length of path
	virtual const vec2& getstartpos() const = 0;										// get starting point
	virtual const vec2& getstartforward() const = 0;									// get starting direction
	virtual const vec2 getendpos() const = 0;											// get ending position
	virtual const vec2 getendforward() const = 0;										// get ending direction (null vector if no constraint)
	virtual bool pointalongpath(const float dist, vec2& pos, vec2& forward) const = 0;	// get point on path, measuring from start
	virtual float getcurvatureat(const float dist) const = 0;							// get curvature at specified distance from start
	virtual double distanceto(const vec2& pt) const;									// distance to this point from path start (approx)
	virtual float getsteeringcurvature() const = 0;										// get best curvature for steering use (bad)
	virtual bool steertopath(float& curv, float lookaheaddist);					// get best curvature for steering use (better)
	virtual void clear() = 0;																			// clear path (sets length to zero)	
	virtual bool getvalid() const = 0;															// true if path valid		
	virtual float getmincurv() const = 0;														// curvature limits
	virtual float getmaxcurv() const = 0;
	virtual void dump(const char* msg) const;											// dump, with message
};

//
//	Implementation
//
//
//	distance to -- distance to point on path (approx)
//
//	Default implementation is linear distance.  Derived classes can override this if they can do better.
//
inline double CurvedPath::distanceto(const vec2& pt) const
{	return((pt - getstartpos()).length()); }
#endif // CURVEDPATH_H