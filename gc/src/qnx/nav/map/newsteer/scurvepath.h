//
//	scurvespath.h  -- two-pass process over code in splinepath.h to approximately account for finite steering speed
//
//	By Ron Avitzur.
//
//
//
//	Copyright 2005 by Ron Avitzur
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
#ifndef SCURVESPATH_H
#define SCURVESPATH_H

#include "curvedpath.h"													// the base class
#include "splinepath.h"													// used in implementation

//
//	class SCurvesPath  -- three circular arcs where the initial one is chosen to account for
//   steering motor latency, steering motor finite speed, and initial steering, and following
//   two arcs are chosen to reach the goal point.

class SCurvePath: public CurvedPath
{	
private:
	// Save the parameters to setstart
	vec2 start;
	vec2 start_heading;
	float start_speed;																	// speed, in m/sec, nonnegative
	double start_curvature;
	double start_curvature_distance;
	double start_min_curvature;
	double start_max_curvature;
	double start_max_curvature_rate;
	double start_lookahead_distance;
	
	SplinePath path;

public:
	SCurvePath();																		// constraints on how tight the curve can be
	void setstart(const vec2& pos, const vec2& forward, const float speed, const float curvature, const float initialcurvaturedist,
		const float mincurvature, const float maxcurvature, const float maxcurvaturerate, float lookaheaddist);
	
	bool setend(const vec2& pos);													// specify end of path, direction is don't care
	bool setend(const vec2& pos, const vec2& forward);					// specify end of path, direction is forward
	double getlength() const;																// get length of path
	bool pointalongpath(const float dist, vec2& pos, vec2& forward) const;	// get point on path, measuring from start
	const vec2& getstartpos() const;													// get starting point
	const vec2& getstartforward() const;											// get starting direction
	const vec2 getendpos() const;														// get ending position
	const vec2 getendforward() const;												// get ending direction (null vector if no constraint)
	float getcurvatureat(const float dist) const;									// get curvature at specified distance from start
	float getsteeringcurvature() const;												// get best curvature for steering use					
	float getmincurv() const;																	// curvature limits
	float getmaxcurv() const;
	void clear();																					// clear path, invalidate it
	bool getvalid() const;																	// true if valid path
	void dump(const char* msg) const;												// dump, with message

#ifdef DEBUGOUTPUT
	void debugoutput();
#endif
};

//	Neary everything in SCurvesPath passes straight through to SplinePath.
//	But any information determined entirely by startpath and changed must be returned from the local values

inline SCurvePath::SCurvePath() {}
//	Meaningful before setendpos
inline const vec2& SCurvePath::getstartpos() 		const { return(start); };								
inline const vec2& SCurvePath::getstartforward()	const { return(start_heading); };		
inline float SCurvePath::getmincurv() const { return(start_min_curvature); }
inline float SCurvePath::getmaxcurv() const { return(start_max_curvature); }

//	Meaningful only after setendpos
inline bool SCurvePath::getvalid() 						const { return(path.getvalid()); 	}
inline const vec2 SCurvePath::getendpos() 			const { return(path.getendpos()); };							
inline const vec2 SCurvePath::getendforward() 		const { return(path.getendforward()); };		
inline double SCurvePath::getlength() 					const { return(path.getlength()); }
inline void SCurvePath::clear() 									{	path.clear(); }
inline float SCurvePath::getsteeringcurvature() 	const	{ return path.getsteeringcurvature(); }
inline float SCurvePath::getcurvatureat(const float distance_along_path) const	{ return path.getcurvatureat(distance_along_path); }
inline bool SCurvePath::pointalongpath(const float distance_along_path, vec2& pos, vec2& forward) const
	{ return path.pointalongpath(distance_along_path, pos, forward); }

inline bool SCurvePath::setend(const vec2& pos) 
	{
	return SCurvePath::setend(pos, start_heading);
	} 


#endif