// ----------------------------------------------------------------------------
//
// Homunculus: the "little simulated car" inside the real car's controller
//
// Overbot/OpenSteer API
// by Craig Reynolds
//
// This class represents OpenSteer's internal model of the current state of
// Overbot
//
// XXX eventually this will want a different base class, without annotation
// XXX or conditionalize out annotation from Simple vehicle?
//
// XXX The entire Homunculus class is defiend inline in this .h file,
// XXX consider breaking some of itit off into a .cpp file
//
// 01-24-05 cwr: split off ouu.h, renamed (maybe Homunculus is only temporary)
//
// ----------------------------------------------------------------------------


#include "Homunculus.h"

// define map size (and compute its half diagonal)
float Homunculus::worldSize = 200;
float Homunculus::worldDiag = sqrtXXX (square (worldSize) / 2);

// 0 = obstacle avoidance and speed control
// 1 = wander, obstacle avoidance and speed control
// 2 = path following, obstacle avoidance and speed control
// int Homunculus::demoSelect = 0;
int Homunculus::demoSelect = 2;

float Homunculus::savedNearestWR = 0;
float Homunculus::savedNearestR = 0;
float Homunculus::savedNearestL = 0;
float Homunculus::savedNearestWL = 0;


// ----------------------------------------------------------------------------
