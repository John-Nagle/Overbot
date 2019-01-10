//
//	mapserver.h  -- main class of the MAP program.
//
//	John Nagle
//	Team Overbot
//	December, 2004
//
//	An early version was written by Achut Reddy.
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
#ifndef MAPSERVER_H
#define MAPSERVER_H

#include <list>
#include <vector>
#include <stdio.h>
#include <pthread.h>

#include "lidarserver.h"
#include "mutexlock.h"
#include "mapservermsg.h"
#include "terrainmap.h"
#include "LMSmapupdate.h"															// SICK LMS support
#include "VORADmapupdate.h"													// VORAD update
#include "moveservermsg.h"
#include "waypoints.h"
#include "maplog.h"
#include "vehicledriver.h"
#include "vehicleposes.h"
#include "roadfollow.h"

class MapLog;																					// forward
//
//	class MapServer  -- one map server, with map
//
class MapServer {
private:
    // message handling
    MsgServerPort	m_serverPort;														// message port
	    
    // internal state 
    int m_verboselevel;
    TerrainMap	m_map;																		// the map
	WaypointSet m_allwaypoints;														// all the waypoints
    ost::Mutex 	m_maplock;																// lock that protects the map
	LMSmapUpdater	m_lmsupdater;													// SICK LMS support
	VORADmapUpdater	m_voradupdater;											// VORAD support
	RoadFollow	m_roadfollower;														// road follower support
	VehicleDriver	m_driver;																// driving level
	VehiclePoses	m_poses;																// pose info
	MapLog		m_log;																		// associated log
	double			m_steertimestamp;													// last steering cycle start
public:
    MapServer();			// constructor
    ~MapServer();			// destructor
    
    void messageThread();		// main thread to receive messages
	//	Dummy test mode
    void playbackTest(const char* dummylidarin, const char* dummygpsinsin, const char* waypointin, const char* logdirout);	// playback from test files
	//	Real mode
	bool executeMission(const char* waypointin, const char* logdirout);	// does the actual work
	void SetFault(Fault::Faultcode newfault);										// set and report a fault	
    void setVerboseLevel(int lev);
    bool getVerbose() const { return(m_verboselevel > 0); }
    int getVerboseLevel() const { return(m_verboselevel); }
    LMSmapUpdater& getLMSupdater() { return(m_lmsupdater); }	// access
    TerrainMap& getMap() { return(m_map);	}									// access
    WaypointSet& getAllWaypoints() { return(m_allwaypoints); }		// access
    ost::Mutex& getMapLock() { return(m_maplock); }						// access
    RoadFollow& getRoadFollow() { return(m_roadfollower); }			// access
    MapLog& getLog() { return(m_log); }											// return log
    VehiclePoses& getPoses() { return(m_poses); }							// access
public:
	//	Portable update functions
	void updateMapRectangle(const vec3& p1, const vec3& p2, const vec3& p3, const vec3& p4, float minrange, uint32_t cyclestamp, bool forceallgreen);	
	void updateMapTriangle(const vec3& p1, const vec3& p2, const vec3& p3, bool sweeping,  float minrange, uint32_t cyclestamp);
	void updateMapEdge(const vec3& p1, const vec3& p2, float r1, float r2, bool sweeping, uint32_t cyclestamp);
	void updateCell(const vec3& p, CellData::CellType newtype, bool sweeping, uint16_t minrange, uint8_t roughness, float elev, uint32_t cyclestamp);	    
	void updateCell(int ix, int iy, CellData::CellType newtype, bool sweeping, uint16_t minrange, uint8_t roughness, float elev, uint32_t cyclestamp);	    
	//	Misc. access
	static float getClearCellRoughnessLimit();										// from tuneable constants
	static float getNogoCellRoughnessLimit();
	float getCurvature();																		// get current turning curvature
private:
	//	Mission control
	void stopMission();
	bool startMission();
	//	Incoming message processing    	    
    void handleMessage(int rcvid, MapServerMsg& msg);
	void handleStop(const MsgSpeedStop& msg);
	void handleVorad(const VoradServerMsgVDTG& msg);
	void handleSonar(const SonarObstacleMsgReq& msg);
	void handleQuery(int rcvid);
};

#endif // MAPSERVER_H
