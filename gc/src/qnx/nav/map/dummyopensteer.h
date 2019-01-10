//
//	OpenSteer dummy class structure
//
//
//	This is something so that we can go ahead with map building 
//	without getting into the internals of OpenSteer
//
//	John Nagle
//	Team Overbot
//	December, 2004
//
//	DRAFT
//
//	Uncopyrighted, no copying restrictions.
//
#ifndef DUMMYOPENSTEER_H
#define DUMMYOPENSTEER_H

#define NEWSTEER														// using newsteer, not opensteer

#include <vector>
#include "newsteer.h"
#include "waypoints.h"
class AbstractTerrainMap												// We don't inherit
{};
typedef NewSteer ReactiveDriver;
#endif // DUMMYOPENSTEER_H

