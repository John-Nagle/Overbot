//
//	vehicleposes.cc  -- pose in 3D space of the vehicle
//
//	J. Nagle
//	Team Overbot
//	February, 2005
//
//	We have to be able to obtain the position of the vehicle at any moment in recent time.
//	This requires interpolation
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
#include <inttypes.h>
#include <algebra3aux.h>
#include "vehicleposes.h"
#include "eulerangle.h"
#include "geocoords.h"
#include "logprint.h"
//
//	Constants
//
const uint64_t k_max_pose_age = 200000000;								// 200ms in nanoseconds
const size_t k_max_saved_poses = 4;												// save this many
#ifdef OBSOLETE
//
//
//	interpolatepose  -- interpolate a pose matrix given two fixes
//
//	Preliminary version - only interprets position, not rotation
//
static bool interpolatepose(const mat4& prevpose, uint64_t prevtime, const mat4& nextpose, uint64_t nexttime,
	uint64_t timewanted, mat4& vehpose)
{	uint64_t dtperiod = nexttime - prevtime;								// delta between fixes
	if (timewanted < prevtime) return(false);								// not in this block
	uint64_t dt = timewanted - prevtime;									// delta from start to end
	if (dt > k_max_pose_age) return(false);								// delta too big, reject
	float fract = (dt/float(dtperiod));											// fraction into period
	//	Just interpolate position for now.  Should do a full SLERP interpolation.
	vec3 pprev(ExtractTranslation(prevpose));							// position at start of interval
	vec3 pnext(ExtractTranslation(nextpose));							// position at end of interval
	vec3 position = pnext*fract + pprev*(1.0-fract);					// linearly interpolate position
	//	Get rotation entirely from first fix
	vehpose = rotation3D(ExtractRotation(prevpose));				// get rotation
	vehpose = translation3D(position)*vehpose;		// apply translation
#ifdef OBSOLETE
	vec3 trans(ExtractTranslation(vehpose));								// ***TEMP***
	printf("interpolatepose: fract %6.2f  pos [%6.2f %6.2f %6.2f]  trans  [%6.2f %6.2f %6.2f]\n",
		fract, prevfix.pos[0],prevfix.pos[1],prevfix.pos[2],trans[0],trans[1],trans[2]);		// ***TEMP***
#endif // OBSOLETE
#ifdef OBSOLETE
	vec3 trans2(ExtractTranslation(vehpose));								// ***TEMP***
	printf("interpolatepose:  trans2  [%6.2f %6.2f %6.2f]\n",
		trans2[0],trans2[1],trans2[2]);		// ***TEMP***
#endif // OBSOLETE
	return(true);
}
#endif // OBSOLETE
//
//	class VehiclePoses  -- where we were at some recent moments in time
//
//	We need more than one of these, because we have to precisely associate locations
//	with GPS data.
//
		
//
//	addPose  -- add a pose to the pose queue
//
void VehiclePoses::addPose(const mat4& pose, float cep, const uint64_t timestamp)
{	ost::MutexLock lok(m_lock);															// lock	
	if (m_poses.size() >= k_max_saved_poses)									// if too many stored
	{	m_poses.pop_front();	}															// drop oldest
	m_poses.push_back(VehiclePose(pose, cep, timestamp));			// push new pose
}
//	getposeattime  -- get pose at specified time, interpolating as necessary
//
//	We have available a queue of recent poses.
//
bool VehiclePoses::getposeattime(VehiclePose& pose, uint64_t posetime, bool& toolate)
{	ost::MutexLock lok(m_lock);															// lock	
	//	Linear search for a matching pose. Works backwards from most recent pose, the most likely match.
	std::deque<VehiclePose>::reverse_iterator pnext;						// next-item iterator (in reverse)
	bool first = true;																			// true first time through search
	for (std::deque<VehiclePose>::reverse_iterator p = m_poses.rbegin(); p != m_poses.rend(); p++)	// for all in queue
	{	if (!first)
		{	const VehiclePose& prevpose = *p;										// next pose
			const VehiclePose& currpose = *pnext;								// prev pose
			////logprintf("Testing time range %lld to %lld against %lld\n", prevpose.m_timestamp, currpose.m_timestamp, posetime); // ***TEMP***
			if (posetime > currpose.m_timestamp)									// if pose not available yet
			{	toolate = false; return(false);	}										// try later
			if (posetime >=  prevpose.m_timestamp)								// if pose in range
			{	//	Find. Handle it.
				toolate = false;																	// not too late
				bool good = interpolate(prevpose, currpose, posetime, pose);// interpolate between poses
				if (good) return(true);														// success
				logprintf("Unexpected pose interpolation failure.\n");		// should not happen
				toolate = true;																	// do not try again
				return(false);																	// fails
			}
		}
		first = false;																				// not first time through
		pnext = p;																				// save "next"
	}
	//	No find. Fails.
	toolate = true;																				// not going to succeed
	logprintf("Unable to find GPS fix for time %lld. %d poses stored.\n", posetime, m_poses.size());
	return(false);
}
//
//	interpolate -- interpolate between two poses
//
bool VehiclePoses::interpolate(const VehiclePose& prevpose, const VehiclePose& currpose, uint64_t posetime, VehiclePose& pose)
{	mat4 outpose;
	bool good = interpolateposedumb(prevpose.m_vehpose, prevpose.m_timestamp, currpose.m_vehpose, currpose.m_timestamp,
		posetime, outpose);
		pose = VehiclePose(outpose, currpose.m_cep, posetime);		// construct result
	return(good);																				// return status
}
