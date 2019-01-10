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


#ifndef ReactiveDriver_H
#define ReactiveDriver_H


#include "Homunculus.h"  // the little simulated vehicle inside Overbot


// ----------------------------------------------------------------------------
// This is the class that will be used by Overbot as the interface to OpenSteer
//
// XXX in addition to speed, don't we have to add a "time since last update"
// XXX argument to "steer"?
//
// XXX eventually it should be "class ReactiveDriver : PRIVATE Homunculus"
// XXX forcing everything to go through the ReactiveDriver API


class ReactiveDriver : public Homunculus
{
public:

    // default constructor
    ReactiveDriver () : localClock (0.0f) {}

    // default destructor
    virtual ~ReactiveDriver () {}

    // cross-track error < forward distance*ratio 
    void setVehicleTrackingError (float ratio) {vehicleTrackingError = ratio;}

    // John called this "settings complete", what does that mean?
    void init () {reset ();}

    // vehicle rectangle, 1/turning radius
    // XXX not currently implemented XXX
    void setVehicleProperties (float x, float y, float invturnradius) {}

    // Execute one STEER update: given current vehicle state (global
    // position, heading, speed), elapsed time since last steer update,
    // current terrain traversability map, and nearby waypoints -- this
    // function returns:
    //
    //   (0) a bool indicating successful driving, returns false if stuck
    //   (1) distance vehicle can safely move along its (curved) path
    //   (2) the steering controller's recommended path curvature
    //   (3) the steering controller's recommended speed
    //
    bool steer  (// start point (world coordinates)
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
                 float& recommendedSpeed);

private:
    float vehicleTrackingError;
    float localClock;
};


// ----------------------------------------------------------------------------
#endif // ReactiveDriver_H
