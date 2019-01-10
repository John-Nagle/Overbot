//
//	waypoints.h  -- a set of waypoints describing an entire course.
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

#ifndef WAYPOINTS_H
#define WAYPOINTS_H

#include <vector>
#include "Vector.h"											// for autopilot math library
//
class ActiveWaypoints;										// forward
//
//	struct Waypoint	-- one waypoint
//
//	An array of waypoints defines "pathways", upon which the vehicle must
//	stay.  The vehicle's objective is to move along the pathways in the
//	direction of later waypoints.
//
struct Waypoint
{
    int m_serial;       // serial number (begins at 1)
    float m_speedlimit; // maximum speed in meters per second
    double m_x, m_y;    // waypoint coordinates
    float m_width;      // allowed "road width"
    int m_turnnum;			// "turn number, used to handle the path crossing itself
};
//
//	class WaypointSet  --  a set of waypoints
//
//	This is the entire waypoint list, not the active set.
//
class WaypointSet {
private:
	//	The waypoint data
	Vector<3> m_llhorigin;										// base latitude and longitude (first waypoint)
    vector<Waypoint>	m_waypoints;					// list of waypoints
	bool		m_verbose;											// true if verbose
	bool		m_valid;												// waypoint set is valid
	bool 		m_originvalid;										// origin latitude and longitude are valid
	bool		m_closed;												// closed course. go round and round
public:
    WaypointSet();													// constructor
    ~WaypointSet();												// destructor
    int readWaypoints(const char *fileName);		// read in waypoint list
    void setVerbose(bool on) { m_verbose = on;  }
    bool getValid() { 	return(m_valid); }					// true if waypoint set valid
    bool getClosed() { return(m_closed);	}			// true if waypoint set closed
	void clear() { m_valid = false; m_originvalid = false; m_closed = false; m_waypoints.clear(); }
	void waypointsInRect(ActiveWaypoints& wpts,double x0, double y0, double x1, double y1);
	const vector<Waypoint> getWaypoints() const { return(m_waypoints); }	// ***TEMP*** for path fence only
	size_t size() const { return(m_waypoints.size()); }	// size of waypoint set
	const Vector<3> getOrigin() { return(m_llhorigin); }	// get origin
	const Waypoint& getWaypoint(size_t i) const	
	{	if (i < m_waypoints.size()) return(m_waypoints[i]);	//	 if in range, good
		if (m_closed)													// if closed course
		{	return(m_waypoints[i % m_waypoints.size()]);}	// treat waypoint numbers as modular
		throw("Waypoint number out of range");		// subscript error
	}
	int getLastSerial() const 									// serial number of last waypoint
	{	if (size() >= 1)												// if valid
		{	return(getWaypoint(size()-1).m_serial); 	}				// return last waypoint serial
		return(-1);														// else -1
	}
	int getLastTurnNum() const
	{	if (size() >= 1)												// if valid
		{	return(getWaypoint(size()-1).m_turnnum); 	}				// return last waypoint serial
		return(-1);														// else -1
	}

private:
	bool parseWaypointLine(int lineno, const char* line);
	void WaypointError(int lineno, const char* line, const char* msg);	// report a bad waypoint in the file
	void checkclosedcourse();								// check for a closed course
	void buildturnnumbers();									// annotate waypoints with turn numbers
	bool getllhorigin(Vector<3>& llh);					// get the LLH origin from GPSINS server
	bool setllhorigin(const Vector<3>& llh);			// set the origin in the GPSINS server
};
//
//	class ActiveWaypoints  -- the set of active waypoints
//
//	Includes only the ones of current interest. near the vehicle location.
//
//	Should be in ascending order but need not be dense.
//
class ActiveWaypoints {
private:
	vector<Waypoint> m_waypoints;						// the waypoints
	int m_lastwaypointserial;									// serial number of last waypoint
	int m_lastwaypointturnnum;								// turn number of last waypoint
	bool m_closedcourse;										// true if a closed course (loop)
public:
	ActiveWaypoints()											// constructor
	: m_lastwaypointserial(-1), m_lastwaypointturnnum(-1), m_closedcourse(false)
	{}
	//	Access
	const Waypoint& operator[](size_t ix) const	// access
	{	return(m_waypoints[ix]);	}
	size_t size() const { return(m_waypoints.size()); }	// size
	bool getClosed() const { return(m_closedcourse); }	// true if closed
	int getLastSerial() const { return(m_lastwaypointserial); }	// serial number of last waypoint, even if not in active set
	int getLastTurnNum() const { return(m_lastwaypointturnnum); }	// turn number of last waypoint, even if not in active set
	//	Change
	void push_back(const Waypoint& wp)				// add to list
	{	m_waypoints.push_back(wp);	}
	void clear()														// clear list
	{	m_waypoints.clear(); }
	void setProperties(int lastserial, int lastturnnum, bool closedcourse)		// set info needed for closed course
	{	m_lastwaypointserial = lastserial; m_lastwaypointturnnum = lastturnnum, m_closedcourse = closedcourse; }
};

#endif // WAYPOINTS_H
