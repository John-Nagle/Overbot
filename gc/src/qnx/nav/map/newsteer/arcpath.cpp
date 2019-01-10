//
//	arcpath.cpp  -- arc path support
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
#include "arcpath.h"
#include "geomutil.h"
#include "logprint.h"
//
ArcPath::ArcPath()																					// constructor
: m_valid(false)
{}
//
//	setstart -- set starting point values, longer form
//
void ArcPath::setstart(const vec2& pos, const vec2& forward, float /* speed */, 
				const float curvature, const float /*initialcurvaturedist*/, 
				const float mincurvature, const float maxcurvature, const float maxcurvaturerate, float lookaheaddist)
{	m_startpos = pos;																				// save params
	m_startforward = forward;															
	m_maxcurvature = maxcurvature;
	m_mincurvature = mincurvature;
	m_maxcurvaturerate = maxcurvaturerate;
	m_arclength = 0;																				// no length yet
	m_valid = false;																				// invalidate precomputation
}
//
//	setend -- set ending point values
//
bool ArcPath::setend(const vec2& pos)
{	m_endpos = pos;																				// set ending position
	m_valid = tangentarcthroughpoints(m_startpos, m_startforward, m_endpos, m_curvature);	// compute curvature
	if (!m_valid) return(false);
	m_arclength = arclength(m_startpos, m_endpos, m_curvature);		// compute arc length
	m_valid = (m_curvature >= m_mincurvature && m_curvature <= m_maxcurvature);	// validate curvature
	return(m_valid);
}
//
//	setend -- set ending point values
//
//	Just ignores ending direction.  Only splines can do that.
//
bool ArcPath::setend(const vec2& pos, const vec2& forward)
{	return(setend(pos));
}
//
//	clear -- clear path, make its length zero
//
void ArcPath::clear()
{
	m_endpos = m_startpos;																	// starts at end
	m_curvature = 0;																				// no curvature
	m_arclength = 0;																				// no length
	m_valid = false;																				// not valid
}
//
//	getlength -- get length of arc
//
double ArcPath::getlength() const
{	return(m_arclength);	}																		// access function
//
//	getmincurv -- get minimum allowed curvature
//
float ArcPath::getmincurv() const
{
	return(m_mincurvature);
}
//
//	getmaxcurv -- get maximum allowed curvature
//
float ArcPath::getmaxcurv() const
{
	return(m_maxcurvature);
}
//
//	pointalongpath  -- get point along the path at given distance
//
//	Easy for an arc.
//	Could be speeded up by precomputing the center.
//
bool ArcPath::pointalongpath(const float dist, vec2& pos, vec2& forward) const
{	if (!m_valid) return(false);																	// fails if not valid
	pos = pointalongarc(m_startpos, m_startforward, m_curvature, dist, forward);		// get point along arc
	return(true);
}
//
//	dump -- for debug
//
void ArcPath::dump(const char* msg) const
{	logprintf("%s: path from (%1.2f, %1.2f) to (%1.2f, %1.2f), curv. %1.4f\n",
		msg, m_startpos[0], m_startpos[1], m_endpos[0], m_endpos[1], m_curvature);	// dump path info
}
