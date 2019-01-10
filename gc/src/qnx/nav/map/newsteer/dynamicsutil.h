//
//	dynamicsutil.h  -- dynamics utililty functions  
//
//	SAFETY CRITICAL
//
//	John Nagle
//	Team Overbot
//	March, 2005.
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
#ifndef DYNAMICSUTIL_H
#define DYNAMICSUTIL_H
class CurvedPath;
class ImpingementGroup;
void calcSafeCurvatureLimits(float speed, float pitchradians, float rollradians, float abscurvlimit, float& mincurv, float& maxcurv);
void calcStoppingDistance(float speed, float pitchradians, float rollradians, float &dist);
float calcSafeSpeedForDistance(float pitchradians, float dist);
float calcSafeSpeedForCurvature(float pitchradians, float rollradians, float curvature);
bool intightspot(const CurvedPath& path, float prevcurvature, const ImpingementGroup& impingements);
#endif // DYNAMICSUTIL_H
