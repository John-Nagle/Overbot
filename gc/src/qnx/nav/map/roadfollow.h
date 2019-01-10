//
//	Road follower interface
//
//	John Nagle
//	Team Overbot
//	September, 2005
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
#ifndef ROADFOLLOW_H
#define ROADFOLLOW_H
#include <stdio.h>
#include "messaging.h"
#include "mutexlock.h"
//
//	struct RoadFollowInfo  -- one set of road follower data
//
struct RoadFollowInfo {
	float m_confidence;															// last confidence reported
	float m_curvature;																// last curvature reported
public:
	RoadFollowInfo()																// dummy constructor
	: m_confidence(-999), m_curvature(0) 
	{}
};
//
//
//	Class RoadFollow -- interface to one road follower
//
class RoadFollow {
private:
	MsgClientPort	m_roadport;												// connection to road server port
	RoadFollowInfo m_lastinfo;												// last set of info
	ost::Mutex m_lock;																// lock on data
private:
	void* roadThread();															// roadThread -- local thread to manage tiliting.
    // need static function for pthread_create
    static void* roadThreadStart(void* arg)							// start the gaze thread
	{ return(reinterpret_cast<RoadFollow*>(arg)->roadThread()); }
public:
	RoadFollow();																	// constructor
	void init();																			// start road follower interface
	bool getRoadSteeringHint(float& curv);							// get steering hint from road follower, non blocking
	void getRoadSteeringHint(RoadFollowInfo& info);				// get steering hint from road follower, non blocking
};
#endif // ROADFOLLOW_H