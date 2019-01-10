//
//	vehicleposes.h  -- pose in 3D space of the vehicle
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
#ifndef VEHICLEPOSES_H
#define VEHICLEPOSES_H
#include <inttypes.h>
#include <deque>
#include "algebra3.h"
#include "mutexlock.h"
//
//	Constants
//
//
//	struct VehiclePose  -- one vehicle pose, with time and error
//
struct VehiclePose {
	mat4 	m_vehpose;																		// vehicle pose
	uint64_t 	m_timestamp;																// timestamp of pose
	float 		m_cep;																				// circular error probability
public:
	VehiclePose(const mat4& vehpose, float cep, uint64_t timestamp)	// constructor
	: m_vehpose(vehpose), m_timestamp(timestamp), m_cep(cep)
	{}
	VehiclePose() {}																			// empty constructor
};
//
//	class VehiclePoses  -- where we were at some recent moments in time
//
//	We need more than one of these, because we have to precisely associate locations
//	with GPS data.
//
class VehiclePoses {
private:
	ost::Mutex m_lock;																			// lock
	std::deque<VehiclePose> m_poses;												// the saved poses
public:
	VehiclePoses()	{}																			// constructor
	void addPose(const mat4& pose, float cep, const uint64_t timestamp);
	bool getposeattime(VehiclePose& pose, uint64_t posetime, bool& toolate);
private:
	bool interpolate(const VehiclePose& prevpose, const VehiclePose& currpose, uint64_t posetime, VehiclePose& pose);
};
#endif // VEHICLEPOSES_H