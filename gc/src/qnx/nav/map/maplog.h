//
//	maplog.h  -- map event logger
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
#ifndef MAPLOG_H
#define MAPLOG_H
#include <time.h>
#include "logitem.h"
#include "logfile.h"
#include "algebra3aux.h"
//
class Waypoint;
class ActiveWaypoints;
//
//	Message to client support
//
class OutputLogMsg: public LogMsg {
public:
	OutputLogMsg()
	{}
	void Reply(int rcvid);													// reply to message request with latest info
};
//
//	class MapLog -- logging for debugging and drawing
//
class MapLog {
private:
	LogFile	m_outputlogfile;
	OutputLogMsg	m_outputlogmsg;
	double m_origin_x;														// 2D vehicle position, and origin for drawing until next vehicle move
	double m_origin_y;	
	double m_origin_z;														// elevation (up) at latest frame
	int m_center_ix;																// center cell of map
	int m_center_iy;																	
	bool m_wroteheader;													// true if wrote header	
	int m_framenumber;														// current frame number											
public:
	MapLog() 
	: m_origin_x(0), m_origin_y(0), 
	m_center_ix(0), m_center_iy(0),
	m_wroteheader(false),
	m_framenumber(0)
	{}
	template <class T> void add(T& item)							// add an item
	{	m_outputlogfile.add(item);											// add to log file, if log file is on
		m_outputlogmsg.add(item);										// add to log msg string, if enabled
	}
	void openlogfile(const char* logdir);							// open log file in logdir if nonnull logdir
	void MapLog::logVehiclePosition(const mat4& vehpose, int centerix, int centeriy, uint64_t timestamp, 
		double lat, double lng, float speed);
	void logHeader(double vehlength, double vehwidth, double cellspermeter);		// log file header info
	void logWaypoints(const ActiveWaypoints& wpts);		// log waypoint list for a single frame
	void logSteeringGoal(const vec2& pt);							// log the current steering goal point
	void logMiscPoint(const vec2& pt, LogColor color, int circlerad = 3);		// log a misc. point.
	void logLine(const vec2& pt0, const vec2& pt1, LogColor color);
	void logArc(const vec2& pt0, const vec2& pt1, float curvature, LogColor color);
	void logArrow(const vec2& pos, const vec2& dir, LogColor color);
	void logFrameEnd();														// log end of frame
	void logMapChange(const CellData& cell, int ix, int iy);	// log one map change
	void logFlush();																// flush the log
};
#endif // MAPLOG_H









