//
//	terrainmap.cc  -- terrain map implementation
//
//	Handles updates of waypoint boundaries in the map as it scrolls.
//
//	John Nagle
//	Team Overbot
//	August, 2004
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
#include <terrainmap.h>
#include "logprint.h"
#include "vehicledriver.h"
#include "mapserver.h"
#include "mutexlock.h"
//
//
//	fillmapx  -- fill boundary info for indicated map column and Y range
//
//	Called when this column has just scrolled onto the map
//
void TerrainMap::fillmapx(int ix, int iymin, int iymax)
{
	////logprintf("Need to fill terrain map column %d from %d to %d\n", ix, iymin, iymax);	// ***TEMP***
	ost::MutexLock lok(m_owner.getMapLock());							// protect map during update
	updateactivewaypoints();														// update active waypoint list
}
//
//
//	fillmapy  -- fill boundary info for indicated map row and X range
//
//	Called when this column has just scrolled onto the map
//
void TerrainMap::fillmapy(int ixmin, int ixmax, int iy)
{
	////logprintf("Need to fill terrain map row %d from %d to %d\n", iy, ixmin, ixmax);	// ***TEMP***
	ost::MutexLock lok(m_owner.getMapLock());							// protect map during update
	updateactivewaypoints();
}
//
//	fillmap  -- fill boundary info for entire map
//
//	May happen after a big scroll
//
void TerrainMap::fillmap(int ixmin, int ixmax, int iymin, int iymax)
{	
	////logprintf("Need to fill entire map\n");									// ***TEMP***
	ost::MutexLock lok(m_owner.getMapLock());							// protect map during update
	updateactivewaypoints();
}
//
//	updateactivewaypoints  -- update the list of active waypoints, called once per steering cycle
//
void TerrainMap::updateactivewaypoints()
{
	ost::MutexLock lok(m_owner.getMapLock());							// protect map during update
	const TerrainMap& map(m_owner.getMap());						// access to map
	const int k_extracells = 4;														// allow a few cells off map for fence
	const double tol = k_extracells/map.getcellspermeter();		// fence is outside waypoint region; bring in fence area
	const double x0 = map.celltocoord(map.getminix()) - tol;
	const double y0 = map.celltocoord(map.getminiy()) - tol;
	const double x1 = map.celltocoord(map.getmaxix()) + tol;
	const double y1 = map.celltocoord(map.getmaxiy()) + tol;
	m_owner.getAllWaypoints().waypointsInRect(m_activewaypoints, x0,y0,x1,y1);	// get relevant waypoints
}
//
//	updateroadfollowinfo  -- update the road follower info, once per steering cycle
//
void TerrainMap::updateroadfollowinfo()
{
	ost::MutexLock lok(m_owner.getMapLock());							// protect map during update
	m_owner.getRoadFollow().getRoadSteeringHint(m_roadfollowinfo);	
}

