//
//	mission.cc  --  executes driving missions
//
//	The level above driving.
//
//	J. Nagle
//	Team Overbot
//	December, 2004.
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
#include "mapserver.h"
#include "signserver.h"
#include "logprint.h"
#include "sys/stat.h"
//	
//	stopMission -- stop any mission in progress
//
void MapServer::stopMission()
{
	m_driver.stopDriving();															// stop any mission in progress
}
//
//	executeMission -- start a mission 
//
bool MapServer::executeMission(const char* waypointin, const char* logdirout)
{	//	Initialization
	stopMission();																		// stop whatever we were doing
	//	Waypoint support
	m_allwaypoints.setVerbose(getVerbose());							// set verboseness
	m_allwaypoints.clear();															// clear waypoints
	m_map.clearmap();																// clear map
	if (!waypointin)
	{	logprintf("No waypoint file name given. Cannot drive.\n");
		SignDisplayPriority("No waypoint file name given. Cannot drive.");
		return(false);
	}
	for (;;)																					// until we get good waypoints
	{	struct stat fileinfo;																// file info status
		int stat = ::stat(waypointin, &fileinfo);								// get file info
		if (stat < 0)																		// if file problem
		{	char ss[100] = "Unreadable waypoint file";
			switch(errno) {
			case ENOENT:	snprintf(ss,sizeof(ss),"Please load a waypoint file into \"%s\".", waypointin); break;
			default: snprintf(ss, sizeof(ss), "Error reading waypoint file  \"%s\": %s", waypointin, strerror(errno));
			}
			SignDisplayPriority(ss);
			logprintf("%s\n",ss);
			sleep(3);
			continue;
		}
		//	We have a file
		stat = m_allwaypoints.readWaypoints(waypointin);			// read the file
		if (stat < 0)
		{	logprintf("Error in waypoint file  \"%s\".\n",waypointin);
			SignDisplayPriority("Error in waypoint file. Please replace waypoint file.");
			sleep(3);
			continue;
		}
		if (m_allwaypoints.size() < 2)											// if not at least two waypoints
		{	logprintf("No waypoints in file  \"%s\".\n",waypointin);
			SignDisplayPriority("No waypoints in file.");
			sleep(3);
			continue;
		}
		break;																				// success, loaded waypoints
	}
	SignDisplayPriority("");													// clear any previous message
	SignDisplay("Waypoint file read.");
	//	Log file support
	if (logdirout)
	{	m_log.openlogfile(logdirout);										// create a log file
		m_log.logHeader(k_vehlength, k_vehwidth, m_map.getcellspermeter());		// log file header info
	}
#ifdef OBSOLETE	// not needed when we don't build path fences
	bool good = m_driver.initPosition();								// initialize vehicle position
	if (!good)
	{	logprintf("Can't get initial vehicle position. Can't drive.\n");	// fatal - not good.
		return(false);
	}
	//	We now have a properly centered map. Fill it with the waypoint fence and create the initial log frame.
	m_map.clearmap();														// this forces a waypoint and map update.
	m_log.logWaypoints(m_map.getActiveWaypoints());		// log the waypoints
	m_log.logFrameEnd();													// finish initial log frame 
#endif // OBSOLETE
	//	Preliminary work done. Start driving
	return(m_driver.startDriving());										// go
}
//
//	Support functions
//
float MapServer::getCurvature()										// get curvature from vehicle
{	return(m_driver.getCurvature());	}

