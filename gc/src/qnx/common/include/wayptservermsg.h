////////////////////////////////////////////////////////////////////////////////
//
//    File:
//	wayptservermsg.h
//
//    Usage:
//
//	In the watchdog startup file (startfile.txt):
//		ID=WAYPT wayptserver
//
//    Description:
//
//	(see wayptserver.h)
//
//    Messages defined:
//
//	WayptServer::GetFirstWaypoint
//	WayptServer::GetCurrentWaypoint
//	WayptServer::NextWaypoint
//	
//
//    Written By:
//
//      Achut Reddy
//      Team Overbot
//      January 2004
//
////////////////////////////////////////////////////////////////////////////////

#ifndef WAYPTSERVERMSG_H
#define WAYPTSERVERMSG_H

#include <time.h>

#include "messaging.h"
#include "rddf.h"

#define WAYPTSERVER_ID	"WAYPT"


//
// Waypoint definition
//

struct Waypoint {
    int		number;			// Waypoint number (integer)
    double	latitude;		// Latitude in decimal degrees
    double	longitude;		// Longitude in decimal degrees
    double	x;			// distance East of origin (in meters)
    double	y;			// distance North of origin (in meters)
    float	boundary;		// Lateral Boundary in meters
    float	speedLimit;		// Speed Limit in meters/sec 
    time_t	deadline;		// Maximum time to reach waypoint
    					// 0 indicates no time limit
};

//
// Waypt Server Message Interface
//

class WayptServerMsg {

public:

    // Return codes from Waypt server
    enum Err {
	OK,				// no error
	EMPTY,				// waypoint list is empty
	OOB				// waypoint index is out of bounds
    };
	    

    //
    // Messages Implemented by Waypt Server
    //

    //  GetNumberOfWaypoints - Get number of waypoints 
    struct GetNumberOfWaypoints : public MsgBase {
	static const uint32_t k_msgtype = char4('W','A','G','N');
	int		number;		// returns number of waypoints here
    };

    //  GetWaypoint - Get specified waypoint by sequence number
    //		Note:  the first waypoint is 0 not 1
    struct GetWaypoint : public MsgBase {
	static const uint32_t k_msgtype = char4('W','A','G','W');
	int		index;		// waypoint index (zero-based)
	Waypoint	waypt;		// waypoint returned here
    };

    //  GetWaypointByXY - Get specified waypoint by X,Y location
    //
    //	The waypoint whose corridor contains the point (x,y) is returned.
    //  If there is more than one such waypoint, the first one is returned.
    //  If no corridor contains the given point, the closest one within
    //	the specified margin is returned
    struct GetWaypointByXY : public MsgBase {
	static const uint32_t k_msgtype = char4('W','A','G','X');
	int		startNum;	// waypoint number to start search from
	double		x;		// X location
	double		y;		// Y location
	double		margin;		// acceptable margin of error distance
	Waypoint	waypt1;		// first waypoint of corridor
					//   returned here
	Waypoint	waypt2;		// second waypoint of corridor
					//   returned here
    };

    union MsgUnion {
	MsgBase			m_header;
	GetNumberOfWaypoints	m_getnumberofwaypoints;
	GetWaypoint 		m_getwaypoint;
	GetWaypointByXY 	m_getwaypointbyxy;
    };

};


#endif // WAYPTSERVERMSG_H
