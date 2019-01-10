//
//	waypointfile.cc  -- read and parse waypoint file.
//	John Nagle
//	Team Overbot
//	January, 2004
//
//	A previous version was by Achut Reddy, but this code has been completely rewritten.
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

#include <stdio.h>
#include <string>
#include <strings.h>
#include <ctype.h>
#include <errno.h>
#include <math.h>
#include "Vector.h"
#include "waypoints.h"
#include "logprint.h"
#include "gpsins_messaging.h"
#include "geocoords.h"
#include "waypointutil.h"
#include "tuneable.h"
//
//	Extra waypoint width - adjustable
//
const Tuneable k_extra_waypoint_width("EXTRAWAYPOINTWIDTH", 0.0, 4.0, 2.0," Additional total width added to each waypoint (m)");
//
//	Constructor
//
WaypointSet::WaypointSet()
: m_verbose(false),
  m_valid(false)
{
	clear();
}
//
//	Destructor
//
WaypointSet::~WaypointSet()
{
}
//
//	readWaypoints  -- read in waypoints list
//
int WaypointSet::readWaypoints(const char *fileName)
{
	clear();																					// clear the waypoints
    if (!fileName) return(-1);														// no filename given
	FILE* fp = fopen(fileName, "r");												// open for reading
	if (!fp)
	{	WaypointError(0,fileName,strerror(errno));
		return(-1);
    }
	//	Read RDF file and parse
	for (int lineno = 1; ; lineno++)												// line count
	{	char line[1000];																// allow for huge line
		char* s = fgets(line, sizeof(line), fp);
		if (!s)																				// if no read
		{	if (ferror(fp))
			{	WaypointError(lineno,"READ ERROR",strerror(errno));	// trouble
				return(-1);																	// fails
			}
			if (feof(fp))																	// if normal EOF
			{	break;	}																	// done
		}
		bool good = parseWaypointLine(lineno, line);					// parse the line
		if (!good) return(-1);															// handle error
	}
	//	Finally check for a closed course, and set closed course flag if necessary
	checkclosedcourse();															// closed course check
	buildturnnumbers();																// and build the turn number information
	return(0);																				// success
}
//
//	isblankline  -- is a line all blank?
//
static bool isblankline(const char* s)
{	for (int i=0; s[i]; i++)														// for all up to null
	{	if (s[i] == '#') return(true);										// lines beginning with "#" are comments.
		if (!isspace(s[i])) return(false);									// some non blank
	}
	return(true);																	// all blank
}
//
//	parseWaypointLIne  --  parse one input line from 2005 waypoint file
//
//	This is a simple comma-delimited file. Format is:
//
//	WAYPOINTNUM, LAT, LONG, LATERALBOUNDARYOFFSET, MAXSPEED
//
bool WaypointSet::parseWaypointLine(int lineno, const char* line)
{	////printf("Waypoint line %d.  %s\n",lineno, line);				// ***TEMP***
	if (isblankline(line)) return(true);									// ignore blank lines
	int waypointnum;															// fields to read in
	double latitude, longitude, lbofeet, speedmph;
	int cnt = sscanf(line,"%d,%lf,%lf,%lf,%lf",&waypointnum, &latitude, &longitude, &lbofeet, &speedmph);
	if (cnt != 5)																	// if not five fields
	{	WaypointError(lineno,line,"Incorrect format");
		return(false);															// fails
	}
	//	We have read a waypoint.
	//	Construct waypoint entry and add to table.
	Waypoint wpt;																// working waypoint
	wpt.m_serial = waypointnum;										// waypoint number
	Vector<3> llh;																// latitude, longitude, height.
	llh[0] = deg2radians(latitude);										// in radians
	llh[1] = deg2radians(longitude);
	llh[2] = 0;
	if (!m_originvalid)															// if first waypoint
	{	m_llhorigin = llh;														// use as origin of X. Y coordinate system
		//	set first waypoint as origin in GPSINS server	
		bool good = setllhorigin(m_llhorigin);						// set the origin for future GPS readings
		if (!good)
		{	WaypointError(lineno, line, "Unable to set origin into GPSINS server.");
			return(false);														// fails
		}
		good = getllhorigin(m_llhorigin);						// get the origin
		if (!good)
		{	WaypointError(lineno, line, "Unable to get origin from GPSINS server.");
			return(false);														// fails
		}
		m_originvalid = true;													// origin is now valid
	}
	vec3 wptxyz = XYZfromLLH(llh, m_llhorigin);				// get coords
	wpt.m_x = wptxyz[0];													// extract X and Y
	wpt.m_y = wptxyz[1];
	wpt.m_width = 2*feet2meters(lbofeet);						// calc width (meters) from DARPA's halfwidth
	float origwidth = wpt.m_width;										// orig width, for msg
	if (wpt.m_width > 0)														// if not ending waypoint
	{	wpt.m_width += k_extra_waypoint_width;	}			// add extra width to compensate for GPS/waypoint errors
    wpt.m_speedlimit = mph2metric(speedmph);				// calc speed limit
	m_waypoints.push_back(wpt);										// add to limits	
	if (m_verbose)
	{	logprintf("Waypoint %4d: (%1.9f N %1.9f E)   X=%1.2f  Y=%1.2f width=%1.1fm -> %1.1fm.  %1.1f m/s.\n",
			wpt.m_serial, latitude, longitude, wpt.m_x, wpt.m_y, origwidth, wpt.m_width, wpt.m_speedlimit);
	}
	return(true);
}
//
//	WaypointError -- report a bad waypoint in the waypoint file.
//
//	May need to send this other places, too.
//
void WaypointSet::WaypointError(int lineno, const char* line, const char* msg)
{
	logprintf("WAYPOINT FILE ERROR: %d. %s\n",lineno, line);
	logprintf("WAYPOINT FILE ERROR: %s\n",msg);
}
//
//	waypointInRect -- true if any part of waypoint is in indicated rectangle
//
//	Note that segment width always comes from waypoint 0 of the pair.
//
//	x0,y0 is lower left (southwest) corner.
//
//	This is conservative; it will report some waypoints as being meaningful even though they are not.
//
static bool waypointInRect(const Waypoint& wpt0, const Waypoint& wpt1, double x0, double y0, double x1, double y1)
{	double halfwidth = wpt0.m_width*0.5;											// halfwidth, or distance from centerline
	if ((wpt0.m_x-halfwidth > x1)
	&& (wpt1.m_x-halfwidth > x1)) return(false);								// segment is east of rectangle
	if ((wpt0.m_x+halfwidth < x0)
	&& (wpt1.m_x+halfwidth < x0)) return(false);								// segment is west of rectangle
	if ((wpt0.m_y-halfwidth > y1)
	&& (wpt1.m_y-halfwidth > y1)) return(false);								// segment is north of rectangle
	if ((wpt0.m_y+halfwidth < y0)
	&& (wpt1.m_y+halfwidth < y0)) return(false);								// segment is south of rectangle
	return(true);																					// there is some overlap
}

//
//	waypointsInRect  -- return waypoints within a specified rectangle in X,Y space
//
//	The input is treated as a hint.  Most of the time, the waypoints won't change.
//	So this is a linear search from the neighborhood of the active waypoint set.
//	
//	Note that if the vehicle is off the course and the waypoint set is totally bogus,
//	it may not find a waypoint set at all. 
//
//	The output waypoints must be in ascending order, except for closed courses.
//
//	If the course is closed, Waypoint #1 may appear at the end, following the last
//	waypoint.  
//
void WaypointSet::waypointsInRect(ActiveWaypoints& wpts,double x0, double y0, double x1, double y1)
{
	//	Check number of waypoints, including check for closed course
	wpts.setProperties(getLastSerial(), getLastTurnNum(), getClosed());		// set info needed for looped courses
	if (m_waypoints.size() < 2) return;									// no waypoints, ignore
	size_t maxwpt = m_waypoints.size();								// waypoint limit
	if (m_closed) maxwpt++;													// one more if closed course, to close the loop
	//	Find range to search, based on last search
	size_t firstwaypointix = 0;													// search range starts as entire range
	size_t lastwaypointix = maxwpt;										// search range ends as entire range					
	if (wpts.size() > 0)
	{	firstwaypointix = wpts[0].m_serial;								// get first waypoint number of active set
		firstwaypointix = std::max(int(firstwaypointix) -1, 0);	// back up one, in case we're going backwards
		lastwaypointix = wpts[wpts.size()-1].m_serial;				// get last waypoint of active set
		lastwaypointix = std::min(lastwaypointix + 2, maxwpt-1);	// advance 2, in case we need to add some
	}
	wpts.clear();																		// clear old active set
	//	Linear search, starting from previous waypoint - 1
	//	Search will continue until a waypoint containing the vehicle is found and all waypoints near the old location have been found
	const vec2 pt((x0+x1)*0.5, (y0+y1)*0.5);						// center of map, vehicle location
	bool ptfound = false;														// have not found vehicle location yet
	for (size_t i = firstwaypointix; i < maxwpt-1; i++)				// for all sequential waypoint pairs
	{	const Waypoint& wp0(getWaypoint(i));
		const Waypoint& wp1(getWaypoint(i+1));
		if (waypointInRect(wp0, wp1,x0,y0,x1,y1))					// if waypoint of interest
		{	if ((wpts.size() == 0) || (wpts[wpts.size()-1].m_serial != wp0.m_serial)) // if not a duplicate
			{	wpts.push_back(wp0);		}									// add first waypoint
			wpts.push_back(wp1);												// always add second waypoint
			if (!ptfound && distanceoutsidewaypoint(pt, wp0, wp1) < 0)		// found vehicle location
			{	ptfound = true;	}													// note find
		} else {
			if (ptfound && i > lastwaypointix) break;					// found and done
		}
	}
	if (!ptfound)
	{	logprintf("Cannot find map center location (%1.2f, %1.2f) in waypoints file.\n", pt[0], pt[1]);		}
}
//
//	checkclosedcourse -- check if this is a closed course.
//	
//	If the first and last waypoints are the same, it is a closed course.
//	We then delete the duplicate waypoint at the end and set the "closed" flag.
//
void WaypointSet::checkclosedcourse()
{	m_closed = false;																// assume not closed
	if (m_waypoints.size() < 3) return;									// must have at least 3 for a closed course
	size_t last = m_waypoints.size() - 1;									// last waypoint index
	const Waypoint& wpt0 = m_waypoints[0];						// first waypoint
	const Waypoint& wptlast = m_waypoints[last];				// last waypoint
	if ((fabs(wpt0.m_x - wptlast.m_x) < 1.0)
	&& (fabs(wpt0.m_y - wptlast.m_y) < 1.0)							// if start and end are within a meter
	&& (fabs(wpt0.m_width - wptlast.m_width) < 1.0)			// and width matches
	&& (fabs(wpt0.m_speedlimit - wptlast.m_speedlimit) < 1.0))		// and speed matches
	{	m_closed = true;															// mark as closed
		m_waypoints.pop_back();												// drop last waypoint, whichis a duplicate
	}
	//	Save last waypoint serial after dropping duplicate at end, if any.
	logprintf("Course has %d waypoints. %s\n", m_waypoints.size(), (m_closed ? "Closed course, will circle indefinitely." : ""));
}
//
//	getllhorigin -- get origin of XYZ coordinate system
//
//	We can also set the origin, but we're not doing that yet.
//
//	This makes a request of the GPS/INS server.
//	Returned LLH is in radians.
//
bool WaypointSet::getllhorigin(Vector<3>& llh)
{	const int k_maxtries = 60;												// if not up in 60 seconds, fails
	MsgClientPort m_gpsclient("GPSINS",1.0);							// try to talk to GPS server
	for (int tries=0; tries<k_maxtries; tries++)						// to get us through startup
	{
		GPSINSGetBasepoint request;										// request
		request.m_msgtype = 	GPSINSGetBasepoint::k_msgtype;	// message type
		GPSINSGetBasepointRep reply;										// reply
		int stat = m_gpsclient.MsgSend(request, reply);			// ask for basepoint
		if (stat < 0)																	// if trouble
		{	if (m_verbose)															// this is normal during startup
			{	logprintf("Error getting basepoint from GPSINS server: %s. Retrying.\n", strerror(errno));	}
			sleep(1);																	// wait, system probably starting up
			continue;																	// try again
		}
		//	Got basepoint
		//	***FRANK ZHANG - check ***
		if (m_verbose)
		{	logprintf("XY coordinate system basepoint is at %1.8f N %1.8f E.\n", reply.llh[0], reply.llh[1]);	}
		llh[0] = deg2radians(reply.llh[0]);								// apply correction
		llh[1] = deg2radians(reply.llh[1]);
		llh[2] = reply.llh[2];
		return(true);
	}
	logprintf("Unable to get basepoint from GPSINS server.\n");
	return(false);
}
//
//	setllhorigin -- get origin of XYZ coordinate system
//
//	We can also set the origin, but we're not doing that yet.
//
//	This makes a request of the GPS/INS server.
//	Returned LLH is in radians.
//
bool WaypointSet::setllhorigin(const Vector<3>& llh)
{	const int k_maxtries = 60;												// if not up in 60 seconds, fails
	MsgClientPort m_gpsclient("GPSINS",1.0);							// try to talk to GPS server
	for (int tries=0; tries<k_maxtries; tries++)						// to get us through startup
	{
		GPSINSBasepoint request;											// request
		request.m_msgtype = 	GPSINSBasepoint::k_msgtype;	// message type
		request.llh[0] = radians2deg(llh[0]);							// set LLH in degrees
		request.llh[1] = radians2deg(llh[1]);
		request.llh[2] = llh[2];
		int stat = m_gpsclient.MsgSend(request);						// set basepoint
		if (stat < 0)																	// if trouble
		{	if (m_verbose)															// this is normal during startup
			{	logprintf("Error setting basepoint in GPSINS server: %s. Retrying.\n", strerror(errno));	}
			sleep(1);																	// wait, system probably starting up
			continue;																	// try again
		}
		//	Basepoint was set
		logprintf("XY coordinate system basepoint set to %1.8f N %1.8f E.\n", request.llh[0], request.llh[1]);	
		return(true);
	}
	logprintf("Unable to set basepoint in GPSINS server.\n");
	return(false);
}
//
//	getpos --  Conversion to vec2 from waypoint
//
inline vec2 getpos(const Waypoint& wp)							// conversion from waypoint to position
{	return(vec2(wp.m_x, wp.m_y)); }
//
//	buildturnnumbers  -- build the "turn number" for each waypoint.
//
//	Turn numbers are used to resolve ambiguities when the course crosses itself. We can't assume we'll
//	always hit every waypoint, but we can assume we'll hit every turn number.
//
//	The "turn number" changes each time the course reverses direction.
//
//	If the course crosses itself, the difference betwen the turn numbers of the intersecting
//	segments will be at least 2.
//
void WaypointSet::buildturnnumbers()
{	const double k_turn_angle = 60 * (M_PI / 180);				// 60 degrees
	const double k_turn_angle_cos = cos(k_turn_angle);	// more than this, and we have done a "turn"
	if (m_waypoints.size() < 2) return;								// nothing  to do
	int turnnum = 0;															// starting turn number
	m_waypoints[0].m_turnnum = turnnum;						// set turn number of first waypoint
	vec2 p0 = getpos(m_waypoints[0]);							// first point
	vec2 p1 = getpos(m_waypoints[1]);							// second point
	vec2 prevdir(p1 - p0);													// initialize previous direction of travel
	prevdir.normalize();														// make unit vector
	for (size_t i=0; i < m_waypoints.size()-1; i++)				// for the rest of the waypoints
	{	Waypoint& wp0 = m_waypoints[i];							// previous waypoint
		Waypoint& wp1 = m_waypoints[i+1];						// waypoint being worked on
		p0 = getpos(wp0);
		p1 = getpos(wp1);													// latest waypoint
		vec2 currentdir(p1 - p0);											// latest dir
		currentdir.normalize();												// make unit vector
		if (currentdir * prevdir < k_turn_angle_cos)				// if direction change big enough
		{	turnnum++;															// advance turn number
			prevdir = currentdir;												// use new current direction
		}
		wp0.m_turnnum = turnnum;										// set turn number for first waypoint of segment
		if (m_verbose)	
		{	logprintf("Waypoint %4d (turn #%d):   X=%1.2f  Y=%1.2f width=%1.1fm.  %1.1f m/s.\n",
			wp0.m_serial, wp0.m_turnnum, wp0.m_x, wp0.m_y, wp0.m_width, wp0.m_speedlimit);
		}
	}
	m_waypoints[m_waypoints.size()-1].m_turnnum = turnnum;	// set turn number of last waypoint
}


