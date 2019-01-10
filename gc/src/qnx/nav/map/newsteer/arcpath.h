//
//	arcpath.h  -- arc path support
//
//	John Nagle
//	Team Overbot
//	May, 2005
//
//	Circular arc type path support.
//	
//	This is a backwards compatibility and test feature, so that we can use either circular arcs or splines.
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
#ifndef ARCPATH_H
#define ARCPATH_H

#include "algebra3.h"													// for vectors
#include "curvedpath.h"

//
//	class ArcPath  -- one path determined by an arc
//
//	Positions are 2D vectors.
//	Directions are 2D unit vectors.
//
//	Notes:
//
//		"setstart" specifies two constraints - the tightest curvature, and
//		the maximum rate of change of curvature.  Maximum rate of change is
//		in units of curvature change per unit distance. (The caller must compute
//		this based on the steering rate and forward speed.)
//
//		"setend" returns true if successful, and false if the required constraints cannot be
//		maintained.
//
//		"setstart" must be called before "setend".
//		"setstart" and "setend" must be called before any of the other functions.
//		"setend" can be called multiple times.
//
//		"getlength" can be an approximation, but the approximation must agree with that
//		used by "pointalongpath", so that when "pointalongpath" is called with "dist"
//		equal to "getlength()", the output "pos" is consistent with "setend".
//
//		When "setend" is called with the one-argument form, the path generated should
//		be roughly an arc.
//
//		"pointalongpath" will be called most often, and must be fast.  It will be called
//		7 to 10 times for each call to "setend".
//
class ArcPath: public CurvedPath
{
private:
	//	Curvature limits
	float m_mincurvature;																		// min curvature
	float m_maxcurvature;																		// max curvature
	float m_maxcurvaturerate;																// max curvature rate change per unit distance along path
	vec2 m_startpos;																				// starting point of arc
	vec2 m_startforward;																		// forward direction
	vec2 m_endpos;																				// ending point of arc
	//	Precomputed values
	float m_curvature;																				// curvature of arc
	float m_arclength;																				// length of arc
	bool m_valid;																					// precomputed values are valid
public:
	ArcPath();																							// constraints on how tight the curve can be
	void setstart(const vec2& pos, const vec2& forward, const float speed, const float curvature, const float mincurvature, const float maxcurvature, const float maxcurvaturerate);
	void setstart(const vec2& pos, const vec2& forward, const float speed,
				const float curvature, const float initialcurvaturedist, 
				const float mincurvature, const float maxcurvature, const float maxcurvaturerate, const float lookaheaddist);
	bool setend(const vec2& pos);														// specify end of path, direction is don't care
	bool setend(const vec2& pos, const vec2& forward);						// specify end of path, direction is forward
	double getlength() const;																	// get length of path
	bool pointalongpath(const float dist, vec2& pos, vec2& forward) const;	// get point on path, measuring from start
	const vec2& getstartpos() const { return(m_startpos); }					// get starting point
	const vec2& getstartforward() const { return(m_startforward); }	// get starting direction
	const vec2 getendpos() const { return(m_endpos); }						// get ending position
	const vec2 getendforward() const  { return(vec2(0,0)); }					// get ending direction (null vector if no constraint)
	float getcurvatureat(const float dist) const										// get curvature at specified distance from start
	{	return(m_curvature);	}																// curvature is constant
	float getsteeringcurvature() const													// get best curvature for steering use					
	{	return(m_curvature);	}																// curvature is constant
	void dump(const char* msg) const;													// dump, with message
	void clear();																						// clear path, make its length zero
	bool getvalid() const { return(m_valid);	}										// true if valid path
	float getmincurv() const;																	// curvature limits
	float getmaxcurv() const;
};
#endif // ARCPATH_H