// ----------------------------------------------------------------------------
//
// Overbot/OpenSteer utilities
// by Craig Reynolds
//
// Contains support classes used for the Overbot/OpenSteer interface:
//
//     OoTerrainMap:   O/O map class (used only for OpenSteerDemo)
//
//     OoGCRoute:      O/O Grand Challenge Route class
//
//     Waypoint:       a route waypoint, specifies width of next leg
//         Waypoints:  STL vector of Waypoint objects
//         WPI:        iterator over Waypoints
//         utilities:  makePathFromWaypoints
//                     copyPathToWaypoints
//
// 01-21-05 cwr: split off from OverbotPlugIn.cpp
//
// ----------------------------------------------------------------------------


#include "oou.h"


// ----------------------------------------------------------------------------


// Take a list (STL vector) of Waypoints and allocate a new OoGCRoute object
// with corresponding geometry
//
OoGCRoute* makePathFromWaypoints (const Waypoints& waypoints)
{
    const int count = waypoints.size();
    Vec3 pathPoints [count];
    float pathRadii [count];
    int j = 0;

    // iterate over Waypoints, copy into arrays for path
    for (WPI i = waypoints.begin(); i != waypoints.end(); i++)
    {
        pathPoints[j] = convertOverbotXYtoVec3 ((*i).m_x, (*i).m_y);
        pathRadii[j+1] = ((*i).m_width) / 2.0f; // 1-based numbering for leg widths
        j++;
    }

    // return newly allocated path
    return new OoGCRoute (count, pathPoints, pathRadii, false);
}


// given a path and a vector of waypoints, copy the path's contents into the
// waypoints.
//
void copyPathToWaypoints (const OoGCRoute& path, Waypoints& waypoints)
{
    // clear waypoint list, then push on each waypoint from path
    const int count = path.pointCount;
    waypoints.clear ();
    for (int i = 0; i < count; i++)
    {
        Waypoint waypoint; 
        waypoint.m_x = convertVec3toOverbotX (path.points[i]);
        waypoint.m_y = convertVec3toOverbotY (path.points[i]);

        // 1-based numbering for leg widths
        waypoint.m_width = path.radii[i+1] * 2.0f;

        waypoints.push_back (waypoint);
    }
}


// ----------------------------------------------------------------------------
