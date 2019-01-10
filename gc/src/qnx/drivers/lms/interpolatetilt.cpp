//
//	interpolatetilt.cpp  -- tilt interpolation and synchronization
//
//	Tilt and scan lines are read at slightly different times, and we have to interpolate
//
//	J. Nagle
//	Team Overbot
//	February, 2005
//
//	We have to be able to obtain the position of the vehicle at any moment in recent time.
//	This requires interpolation
//
#include <inttypes.h>
#include "interpolatetilt.h"
#include "logprint.h"
//
//	Constants
//
const uint64_t k_max_pose_age = 200000000;								// 200ms in nanoseconds
const size_t k_max_saved_poses = 12;											// enough for 4K of scan lines 

//
//	class TiltPositions  -- where we were at some recent moments in time
//
//	We need more than one of these, because we have to precisely associate locations
//	with GPS data.
//
		
//
//	addPose  -- add a pose to the pose queue
//
void TiltPositions::addPose(const float pose, const uint64_t timestamp)
{	ost::MutexLock lok(m_lock);															// lock	
	if (m_poses.size() >= k_max_saved_poses)									// if too many stored
	{	m_poses.pop_front();	}															// drop oldest
	m_poses.push_back(TiltPosition(pose, timestamp));			// push new pose
}
//	getposeattime  -- get pose at specified time, interpolating as necessary
//
//	We have available a queue of recent poses.
//
bool TiltPositions::getposeattime(TiltPosition& pose, uint64_t posetime, bool& toolate)
{	ost::MutexLock lok(m_lock);															// lock	
	//	Linear search for a matching pose. Works backwards from most recent pose, the most likely match.
	std::deque<TiltPosition>::reverse_iterator pnext;						// next-item iterator (in reverse)
	bool first = true;																			// true first time through search
	for (std::deque<TiltPosition>::reverse_iterator p = m_poses.rbegin(); p != m_poses.rend(); p++)	// for all in queue
	{	if (!first)
		{	const TiltPosition& prevpose = *p;										// next pose
			const TiltPosition& currpose = *pnext;								// prev pose
			////logprintf("Testing time range %lld to %lld against %lld\n", prevpose.m_timestamp, currpose.m_timestamp, posetime); // ***TEMP***
			if (posetime > currpose.m_timestamp)									// if pose not available yet
			{	toolate = false; return(false);	}										// try later
			if (posetime >=  prevpose.m_timestamp)								// if pose in range
			{	//	Find. Handle it.
				toolate = false;																	// not too late
				bool good = interpolate(prevpose, currpose, posetime, pose);// interpolate between poses
				if (good) return(true);														// success
				logprintf("Unexpected tilt interpolation failure.\n");		// should not happen
				toolate = true;																	// do not try again
				return(false);																	// fails
			}
		}
		first = false;																				// not first time through
		pnext = p;																				// save "next"
	}
	//	No find. Fails.
	toolate = true;																				// not going to succeed
	logprintf("Unable to find tilt for time %lld. %d tilts stored.\n", posetime, m_poses.size());
	return(false);
}
//
//	interpolate -- interpolate between two poses
//
bool TiltPositions::interpolate(const TiltPosition& prevpose, const TiltPosition& currpose, uint64_t posetime, TiltPosition& pose)
{	
	uint64_t dt = currpose.m_timestamp - prevpose.m_timestamp;	// time between poses
	if (dt > k_max_pose_age)															// if too old
	{	return(false);	}																	// fails
	if (posetime < prevpose.m_timestamp || posetime > currpose.m_timestamp) return(false);	// not in this interval
	uint64_t dt0 = posetime - prevpose.m_timestamp;					// time since previous pose
	double s = 0;																			// weight for previous timestamp
	if (dt > 0)
	{	s = (double(dt0) / double(dt));	}											// compute fraction into timestamp
	float outpose = (1.0-s)*prevpose.m_tilt + s*(currpose.m_tilt);	// interpolate tilt
	pose = TiltPosition(outpose, posetime);									// construct result
	return(true);																				// return status
}
