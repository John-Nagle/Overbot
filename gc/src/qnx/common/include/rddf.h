////////////////////////////////////////////////////////////////////////////////
//
//    File:
//	rddf.h
//
//    Description:
//	Route Description Data File Format (defined by DARPA)
//
//    See also:
//
//    Written By:
//       Achut Reddy
//       Team Overbot
//       January, 2004
//
/////////////////////////////////////////////////////////////////////////////////

#ifndef RDDF_H
#define RDDF_H

#include <queue>
#include <stdio.h>
#include <pthread.h>

using namespace std;

// RDDF data structure
//
// RDDF is an ASCII file, one waypoint definition per line.
// Each line contains comma-separated fields:
//
//	NUM, LAT, LONG, LB, SP LIM, PL HR, PL MIN, PL SEC
//
//	NUM 	Waypoint number (integer)
//	LAT 	Latitude in decimal degrees(7-place precision float)
//	LONG	Longitude in decimal degrees(7-place precision float)
//	LB	Lateral Boundary in feet
//	SP LIM 	Speed Limit in MPH (int? float?)
//	PL HR	Phase Line Hour (PST, 24 hr clock)
//	PL MIN	Phase Line Minute
//	PL SEC	Phase Line Second
//
// Additional notes:
//	- Speed limit of 999 indicates unlimited speed
//	- "Phase Line" is the Maximum Crossing Time for that waypoint.
//	- NULL fields are indicated by "####" (only applies to PL fields??)  

struct RDDF {
    int		number;			// Waypoint number (integer)
    double	latitude;		// Latitude in decimal degrees
    double	longitude;		// Longitude in decimal degrees
    float	boundary;		// Lateral Boundary in feet
    float	limit;			// Speed Limit in MPH 
    int		phaseLineHour;		// Phase Line Hour (PST, 24 hr clock)
    					// -1 indicates no time limit
    int		phaseLineMinute;	// Phase Line Minute
    					// -1 indicates no time limit
    int		phaseLineSecond;		// Phase Line Second
    					// -1 indicates no time limit
};

#endif // RDDF_H
