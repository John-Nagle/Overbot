//
//	maplog.cc  -- map event logger
//
//	John Nagle
//	Team Overbot
//	December, 2004
//
//	This logs changes to the map, which can then be played back by a viewer program.
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
#include <iomanip>
#include <sstream>
#include "maplog.h"
#include "algebra3aux.h"
#include "eulerangle.h"
#include "waypoints.h"
//
//	Configuration constants
//
const char* k_log_prefix = "steer";									// prefixed to log file name
const char* k_log_suffix = "maplog";								// suffixed to log file name
//
//	Implementation
//
//
//	openlog -- open log file if needed
//
void MapLog::openlogfile(const char* logdir)
{
	if (logdir)																	// if log dir named
	{
		char logfilename[512];											// build log file name
		buildlogfilename(logfilename,sizeof(logfilename),logdir,k_log_prefix,k_log_suffix);
		int stat = m_outputlogfile.open(logfilename,true);			// open log file for writing
		if (stat)
		{	printf("Unable to create log file \"%s\" - logging disabled.\n",logfilename);
			return;
		}
		printf("Logging to \"%s\"\n",logfilename);
	}
}
//
//	logHeader -- log header, once only per file. Should be first item
//
void MapLog::logHeader(double vehlength, double vehwidth, double cellspermeter)
{	if (m_wroteheader) return;													// only write header once
	m_framenumber = 0;															// now at frame zero
	LogItemHeader item;																// the item
	item.m_logversion = k_log_version;										// version of log file
	item.m_vehlength =vehlength;												// rectangular vehicle model
	item.m_vehwidth = vehwidth;
	item.m_vehcenteroffset = 0.0;												// centered at origin
	item.m_cellspermeter = cellspermeter;									// cells per meter
	add(item);																				// add to log
	m_wroteheader = true;															// wrote header
}
//
//	logMapChanges  -- log changes to the map
//
void MapLog::logMapChange(const CellData& cell, int ix, int iy)
{	const int k_largest_rel_coord = 2047;									// given a 12 bit field
	int irx = ix - m_center_ix;														// make coords relative to map center
	int iry = iy - m_center_iy;
	if (std::abs(irx) > k_largest_rel_coord) return;						// off the current map
	if (std::abs(iry) > k_largest_rel_coord) return;						// off the current map
	LogItemCell item;
	item.m_type = cell.m_type;													// pack minimal cell info into item
	item.m_valid = cell.m_valid;
	item.m_irx = irx;
	item.m_iry = iry;
	int relevcm = int((cell.avgelev() - m_origin_z)*100);				// elev in cm, relative to level for frame
	item.m_rz = std::min(std::max(relevcm, -2047),2047);			// fit into 12 bits
	add(item);																				// add to the log
}
//
//	logVehiclePosition -- log info about current vehicle position
//
//	This starts a new "frame" in the log file.
//
void MapLog::logVehiclePosition(const mat4& vehpose, int centerix, int centeriy, uint64_t timestamp, 
	double lat, double lng, float speed)
{	m_center_ix = centerix;														// save map center for relative cell values
	m_center_iy = centeriy;
	const EulerAngles angles(Eul_FromHMatrix(vehpose, EulOrdXYZs));
	vec3 position(ExtractTranslation(vehpose));						// get position
	mat3 orientation(ExtractRotation(vehpose));						// get orientation
	vec3 forward(orientation*vec3(1,0,0));								// get forward pointing unit vector
	//// vec3 upvec(orientation*vec3(0,0,1));									// get up-pointing unit vector
	LogItemFrame item;																// log item being built
	item.m_framenumber = m_framenumber;								// add to frame number
	item.m_timestamp = timestamp;											// timestamp
	item.m_x = m_origin_x = position[0];									// position in meters
	item.m_y = m_origin_y = position[1];						
	item.m_z = m_origin_z = position[2];										// elevation in meters				
	item.m_heading = atan2(forward[1], forward[0]);				// heading in radians, +X is 0
	/*
	item.m_pitch = -asin(std::min(1.0,upvec*vec3(1,0,0)));			// pitch
	item.m_roll = -asin(std::min(1.0,upvec*vec3(0,1,0)));			// roll
	*/
	item.m_roll = angles[0];                                                        // roll
	item.m_pitch = angles[1];                                                     // pitch
	item.m_latitude = lat;																// latitude in degrees								
	item.m_longitude = lng;															// longitude in degrees										
	item.m_speed = speed;														// speed in m/sec											
	item.m_ix = centerix;															// log map center cell
	item.m_iy = centeriy;																		
	add(item);																				// add the item
	m_framenumber++;																// add to frame number
}
//
//	logWaypoints  -- log active waypoint list
//
void MapLog::logWaypoints(const ActiveWaypoints& wpts)
{
	for (size_t i=0; i < wpts.size(); i++)										// for all waypoints
	{	const Waypoint& w = wpts[i];											// the waypoint
		LogItemWaypoint item;														// log item being built
		item.m_serial = w.m_serial;												// waypoint number
		item.m_rx = w.m_x - m_origin_x;										// X and Y relative to frame
		item.m_ry = w.m_y - m_origin_y;
		item.m_halfwidth = w.m_width*0.5;									// allowed deviation from centerline
		item.m_speedlimit = w.m_speedlimit;									// max allowed speed (m/s)
		add(item);
	}
}
//
//	logSteeringGoal  -- log the current steering goal
//
void MapLog::logSteeringGoal(const vec2& pt)
{	const int k_goalcircle_size = 3;											// radius in pixels
	LogItemCircle item;															// log item being built
	item.m_rx = pt[0] - m_origin_x;											// X and Y relative to frame
	item.m_ry = pt[1] - m_origin_y;
	item.m_r = k_goalcircle_size;
	item.m_fill = true;
	item.m_color = 2;																// means "green"
	add(item);
}
//
//	logMiscPoint  -- log a miscellaneous point for debug
//
void MapLog::logMiscPoint(const vec2& pt, LogColor color, int circlerad)
{	
	LogItemCircle item;															// log item being built
	item.m_rx = pt[0] - m_origin_x;											// X and Y relative to frame
	item.m_ry = pt[1] - m_origin_y;
	item.m_r = circlerad;															// size of circle to draw
	item.m_fill = true;
	item.m_color = color;															// 3-bit RGB for now
	add(item);
}
//
//	logLine -- log a curved arc wedge for debug
//
void MapLog::logLine(const vec2& pt0, const vec2& pt1, LogColor color)
{	LogItemLine	item;
	item.m_x0 = pt0[0] - m_origin_x;											// X and Y relative to frame
	item.m_y0 = pt0[1] - m_origin_y;
	item.m_x1 = pt1[0] - m_origin_x;											// X and Y relative to frame
	item.m_y1 = pt1[1] - m_origin_y;
	item.m_color = color;
	add(item);
}
//
//	logArc -- log a curved arc wedge for debug
//
void MapLog::logArc(const vec2& pt0, const vec2& pt1, float curvature, LogColor color)
{	LogItemArc	item;
	item.m_r0x = pt0[0] - m_origin_x;											// X and Y relative to frame
	item.m_r0y = pt0[1] - m_origin_y;
	item.m_r1x = pt1[0] - m_origin_x;											// X and Y relative to frame
	item.m_r1y = pt1[1] - m_origin_y;
	item.m_curvature = curvature;
	item.m_color = color;
	add(item);
}
//
//	logArrow -- log an arrow for debug
//
void MapLog::logArrow(const vec2& pos, const vec2& dir, LogColor color)
{	LogItemArrow	item;
	item.m_rx = pos[0] - m_origin_x;											// X and Y relative to frame
	item.m_ry = pos[1] - m_origin_y;
	item.m_dirx = dir[0];																// direction
	item.m_diry = dir[1];
	item.m_length = 0;																// no specified length, for now
	item.m_color = color;
	add(item);
}
//
//	logEndFrame -- put end of frame marker in log
//
void MapLog::logFrameEnd()
{	LogItemFrameEnd item;
	add(item);
}
//
//	logFlush  -- flush the log
//
void MapLog::logFlush()
{
	m_outputlogfile.flush();
}












