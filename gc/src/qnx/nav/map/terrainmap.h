//
//    terrainmap.h -- the local terrain map around the vehicle.
//
//	An early version was written by Achut Reddy.
//	Rewritten by J. Nagle for better compatibilty with OpenSteer.
//	Then rewritten again for NewSteer.
//
//	John Nagle
//	Team Overbot
//	December, 2004
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

#ifndef TERRAINMAP_H
#define TERRAINMAP_H

#include <vector>
#include "scrollablemap.h"
#include "dummyopensteer.h"
#include "waypoints.h"
#include "mapcell.h"
#include "mapconfig.h"
#include "roadfollow.h"



////typedef ScrollableMap<CellData, AbstractTerrainMap> TerrainMap;
class MapServer;															// forward
//
//	class TerrainMap  -- the big scrollable map of cells, and other info about the real world
//
//	ScrollableMap does most of the work, but we have to provide some functions to update
//	the boundaries as the map scrolls.
//
class TerrainMap: public ScrollableMap<CellData, AbstractTerrainMap>
{
private:
	MapServer& m_owner;
	ActiveWaypoints m_activewaypoints;																	// active waypoint set
	RoadFollowInfo m_roadfollowinfo;																			// latest road follower info
	uint32_t m_cyclestamp;																							// map update cycle serial number
	uint32_t m_ancientstamp;																						// older than this, override
public:
	TerrainMap(MapServer& owner, int dimincells, double cellspermeter)					// constructor
	: ScrollableMap<CellData, AbstractTerrainMap>(dimincells, cellspermeter),			// initialize parent
	m_owner(owner), m_cyclestamp(0), m_ancientstamp(0)										// link back to owner
	{}
	virtual ~TerrainMap() {}
	const ActiveWaypoints& getActiveWaypoints() const { return(m_activewaypoints); }
protected:
	void fillmap(int ixmin, int ixmax, int iymin, int iymax);// fill rectangle of map
	void fillmapx(int ix, int iymin, int iymax);					// fill column that just scrolled on
	void fillmapy(int ixmin, int ixmax, int iy);					// fill row that just scrolled on
public:
	void updateactivewaypoints();									// update the active waypoint list
	uint32_t incrementcyclestamp()								// access to cycle serial number
	{	m_cyclestamp++;	 return(m_cyclestamp); }
	uint32_t getcyclestamp() const { return(m_cyclestamp); }
	void setancientstamp(uint32_t val) { m_ancientstamp = val; }
	uint32_t getancientstamp() const { return(m_ancientstamp); }
	void updateroadfollowinfo();
	const RoadFollowInfo& getroadfollowinfo() const 
	{	return(m_roadfollowinfo);	}									// access
};
#endif // TERRAINMAP_H
