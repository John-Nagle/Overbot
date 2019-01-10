//
//	curvedwedge.h  -- geometry of the curved wedge
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
#ifndef CURVEDWEDGE_H
#define CURVEDWEDGE_H
#include <vector>
#include "algebra3.h"
//
//	Forward declarations
//
class TerrainMap;
class Waypoint;
class WaypointTriple;
class CurvedPath;
//
//	Support objects
//
//
//	ImpingementInfo  -- information about one impingement
//
//	if m_passable is true, "unknown" and "possible" can be set to false.
//
struct ImpingementInfo {
	bool	m_valid;										// there is data here, false if no impingement
	vec2	m_pos;											// position of impingement (coords, not cell index)
	float	m_offset;										// offset from centerline (signed, + to right)
	bool m_passable;									// true if passableCell() == true
	bool	m_unknown;									// true if unknownCell() == true
	bool m_possible;									// true if possibleCell() == true
	//	Error info from terrain evaluator
	bool	m_terrainimpassable;					// true if evaluated passable cell and it was rejected
	float	m_elev;											// elevation of failing cell (this plus m_pos is 3D position)
	float 	m_eleverr;									// distance from plane to cell elevation
	vec4 m_plane;										// plane of failed quad being tested
public:														// functions for class
	ImpingementInfo() { clear(); }				// default constructor
	vec3 getpos3D() const { return(vec3(m_pos[0], m_pos[1], m_elev)); }	// get 3D position
	void clear();
};
//	
//	Implementation
//
//
//	clear -- usual clear
//
inline void ImpingementInfo::clear()
{
	m_valid = false;													// no impingement
	m_terrainimpassable = false;								// not rejected by rough terrain check
	m_elev = 0.0;
	m_plane = vec4(0,0,0,0);										// no plane
}
//
//	ImpingementGroup -- convenience for passing center, left, right impingements around
//
struct  ImpingementGroup {
	ImpingementInfo  m_center;									// center part - we could hit this
	ImpingementInfo m_left;										// left shoulder - try for extra clearance here
	ImpingementInfo m_right;										// right shoulder - try for extra clearance here
	vec3 m_worsttiltvector;											// least vertical up vector
public:
	void clear();
};

//
//	clear -- clear for ImpingementGroup
//
inline void ImpingementGroup::clear()
{	m_center.clear();
	m_left.clear();
	m_right.clear();
	m_worsttiltvector = vec3(0,0,1);							// straight up is default
};
//
//	CurvedPathScanner  -- abstract class for scanning curved wedges
//
//	Subclass for boundaries or obstacles.
//
class CurvedPathScanner  {
public:
	bool scan_wedge(const CurvedPath& path, float basewidth, float shoulderwidth, float pathlength, int n);	// generic wedge scan
protected:
	virtual bool raster_ordered_trapezoid(const vec2 quad[4], 
		const vec2& last_sforward, const vec2& next_sforward, 
		double distalongpath, float basewidth, float shoulderwidth) = 0;
};
//
//	Utility functions
//
void dump(const char* msg, const ImpingementGroup& impingements);
bool compareimpingement(const ImpingementInfo& i0, const ImpingementInfo& i1);

//
//	scanCurvedWedge -- scans a curved wedge for impingements
//
//	Basic scan, accepts flat areas only.
//
bool scanCurvedWedge(const TerrainMap& map,			// the terrain map
					 const vec2& pt,				// starting point of wedge
					 const vec2& forward,			// unit vector in forward direction of wedge
					 const float basewidth,			// base width of wedge
					 const float growratio,			// wedge widens with this aspect ratio
					 const float curvature,			// curvature (1/r) of wedge
					 const float arclength,			// arc length to search
					 float &impingementarclength,	// arc length to first impingement
					 vec2& impingement,				// coords of impingement if any
					 bool& impingementunknown,		// true if impingement area unknown
					 bool& impingementimpassable);	// true if is impassable
//
//
//	scanCurvedWedgeWithShoulders  -- scan terrain map for obstructions
//
//	A curved wedge of constant width is scanned, aloing with its
//	a "shoulder area" on each side adjacent to the curved wedge.
//	The wedge has width basewidth and center arclength arclength,
//	and starts at "pt" in direction unit vector "forward",
//
//	Three impingements are reported: left, center, and right:
//
//		(left) Obstruction nearest to centerline of wedge, further than basewidth/2
//			from centerline but less than basewidth/2 + shoulderwidth from centerline, and
//			on left of centerline. This is the "impingement on the left shoulder".
//		(center) Obstruction nearest to start of wedge, within basewidth/2 of arc centerline.
//			This is the "obstacle", with the same meaning as for ScanCurvedWedge above.
//		(right) As above for (left),  but on right of centerline.
//			This is the "impingement on the right shoulder".
//
//		If a center impingement  is found, left and right  don't
//		really matter much.   
//
//
//	It is not required to support a wedge which represents more than a half
//	circle. Any problems, return false with a center impingement at "pt".
//
//	Return true only if there is no center impingement.

//
bool scanCurvedWedgeWithShoulders(
					const TerrainMap& map,		// the terrain map
					 const vec2& pt,					// starting point of wedge
					 const vec2& forward,			// unit vector in forward direction of wedge
					 const float basewidth,			// base width of wedge
					 const float shoulderwidth,	// additional width on either side of base width
					 const float curvature,			// curvature (1/r) of wedge
					 const float arclength,			// arc length to search
					ImpingementGroup& impingements); // impingements out
					 
//
//	scanCurvedWedgeWithShoulders  -- accepts both arc and spline paths
//
class CurvedPath;
//
//	scanPathAgainstObstacles  -- scan path against obstacles
//
//	Marginal terrain is acceptable if all marginal cells are within the terrain threshold of the plane
//	determined by the base width (not shoulder width).
//
void scanPathAgainstObstacles(const TerrainMap& map,
								const CurvedPath& path, 
								const float basewidth,
								const float shoulderwidth,
								const float pathlen,											// only search this much of path
								const float terrainthreshold, 							// roughness threshold for marginal terrain
								ImpingementGroup& impingements);
//
//	scanPathAgainstObstacles  -- similar test, but for waypoints, rather than obstacles
//
void scanPathAgainstWaypoints(const WaypointTriple& wp,
								  const CurvedPath& path, 
                                  const float basewidth,
                                  const float shoulderwidth,
                                  const float dwelldist,								// ignore shoulder impingements out to here
                                  const float pathlen,									// only search this much of path
                                  ImpingementGroup& impingements);

#endif // CURVEDWEDGE_H
