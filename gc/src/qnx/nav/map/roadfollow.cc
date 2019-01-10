//
//	roadfollow.cc  --  Road follower interface
//
//	The road follower is a separate program; this is just its interface to the map server.
//	
//	This was never used in the field. It runs, but the code never affects steering decisions.
//
//	John Nagle
//	Team Overbot
//	September, 2005
//
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
#include <strings.h>
#include <unistd.h>
#include "roadfollow.h"
#include "roadserver.h"
#include "logprint.h"
//
//	Constants
//
const char* k_roadserver_name = "ROAD";							// name of server
const double k_roadserver_timeout = 0.600;						// 600 ms; may only be 2 FPS
const float k_no_confidence = -999;										// no confidence
const float k_good_confidence = 0;										// > 0 is good
//
//	Constructor
//
RoadFollow::RoadFollow()
: m_roadport(k_roadserver_name, k_roadserver_timeout)								
{
	m_lastinfo.m_confidence = k_no_confidence;		// not valid yet
	m_lastinfo.m_curvature = 0;
}
//
//	roadThread -- actually does the work
//
void* RoadFollow::roadThread()
{
	for (;;)
	{	RoadServerMsgRDDR roadquery;							// build query to send to server
		roadquery.m_msgtype = RoadServerMsgRDDR::k_msgtype;
		roadquery.pavement = RoadServerMsgRDDR::RoadUnknown;			// pavement type unknown
		roadquery.upvector[0] = roadquery.upvector[1] = 0.0;
		roadquery.upvector[2] = 1.0;								// assume straight up
		RoadServerMsgReply roadreply;							// reply area
		int stat = m_roadport.MsgSend(roadquery, roadreply);	// query road follower
		if (stat < 0) 
		{ 	logprintf("ROAD server not replying: %s\n", strerror(errno));
			sleep(1);	 														// wait for road server to come up, avoid hang
		}
		{	ost::MutexLock lok(m_lock);								// lock
			if (stat < 0)														// if problem
			{	m_lastinfo.m_confidence = k_no_confidence;	// not valid yet
				m_lastinfo.m_curvature = 0;
			} else {															// valid data
				m_lastinfo.m_confidence =  roadreply.m_confidence;		// not valid yet
				m_lastinfo.m_curvature = roadreply.m_curvature;
			}
		}
	}
}
//
//	init -- start the system
//
void RoadFollow::init()
{	pthread_create(0, 0, roadThreadStart, this);						// start the road follower query thread
}
//
//	getRoadSteeringHint  -- get the latest steering hint
//
//	Non-blocking
//
bool RoadFollow::getRoadSteeringHint(float& curv)
{	ost::MutexLock lok(m_lock);												// lock	
	curv = m_lastinfo.m_curvature;
	return(m_lastinfo.m_confidence > k_good_confidence);	// true if greater than min good value (zero)
}
//
//	getRoadSteeringHint
//
void RoadFollow::getRoadSteeringHint(RoadFollowInfo& info)
{	ost::MutexLock lok(m_lock);												// lock	
	info = m_lastinfo;																// return most recent
}





