// ----------------------------------------------------------------------------
//
// Overbot:  OpenSteer based steering controller
//
// by Craig Reynolds
//
// Defines an abstract steering controller for an automobile-like vehicle.
// This class gets specialized by an application which provides interfaces
// into data prepared from the vehicle's sensors.
//
// Based on the earlier Mapdrive.cpp plug-in which was based on an earlier
// version of Overbot.cpp
//
// 01-04-05 cwr: copied from Mapdrive.cpp, added Ob prefix to names
//
// ----------------------------------------------------------------------------
//
// First there was the class Overbot, later renamed to MapDriver and released
// with OpenSteer 0.8.2.  In this code we split MapDriver into a ObMapDriver
// vehicle (for the ObMapDrivePlugIn) and a "ReactiveDriver" object which is
// shared with the runtime software on the real Overbot.
//
// ----------------------------------------------------------------------------


#include "ReactiveDriver.h"


// ----------------------------------------------------------------------------


// Execute one STEER update: given current vehicle state (global
// position, heading, speed), elapsed time since last steer update,
// current terrain traversability map, and nearby waypoints -- this
// function returns:
//
//   (0) a bool indicating successful driving, returns false if stuck
//   (1) distance vehicle can safely move along its  (curved) path
//   (2) the steering controller's recommended path curvature
//   (3) the steering controller's recommended speed
//
bool ReactiveDriver::steer (// start point (world coordinates)
                            const double startx,
                            const double starty,

                            // going in this direction (unit vector)
                            const float startdirx,
                            const float startdiry,

                            // current speed (in meters per second)
                            const float startSpeed,

                            // current path curvature (1/turn_radius)
                            const float startCurvature,

                            // time since last steer update (in seconds)
                            const float elapsedTime,

                            // map of nearby terrain traversability
                            // XXX really should be CONST but would require
                            // XXX global changes, so I would rather wait
                            /* const */ AbstractTerrainMap& map,

                            // relevant waypoints (only the nearby ones)
                            const Waypoints& waypoints,

                            // output: OK to move this distance
                            float& safeMoveDistance,

                            // output: with this 1/turning radius
                            // (right turn > 0)
                            float& recommendedCurvature,

                            // output: desired speed
                            float& recommendedSpeed)
{
    const Vec3 location = convertOverbotXYtoVec3 (startx, starty);
    const Vec3 heading = convertOverbotXYtoVec3 (startdirx, startdiry);

    // set our position, orientation and speed to the given values.  (Note:
    // for now, the reorientation is always a pure Y rotation, since headings
    // are always on the XZ plane)
    setPosition (location);
    regenerateOrthonormalBasis (heading);
    setSpeed (startSpeed);
    setCurvature (startCurvature);
    //	Set previous state, to resync with real world (JN)
    setLastPosition(position());
    setLastForward(forward());

    // keep track of global time
    localClock += elapsedTime;

    // set our map
    setMap (&map);

    // set our path to a new temporary OoGCRoute object created from waypoints
    OoGCRoute* tempPath = makePathFromWaypoints (waypoints);
    setPath (tempPath);

    // invoke OpenSteer simulation update step
    update (localClock, elapsedTime);

    // delete temp path object
    setPath (NULL);
    delete (tempPath);

    // OUTPUT

    // pass back our new speed
    recommendedSpeed = speed ();

    // pass back our new path curvature
    recommendedCurvature  = curvature ();

    // pass back a distance along this path known to be safe
    safeMoveDistance = safeDistanceForMoveCommand ();

    // returns true during normal operation, returns false when "stuck"
    // (note: to wait for full vehicle stop, return "stuckAndStopped ()")
    return !stuck;
}


// ----------------------------------------------------------------------------
