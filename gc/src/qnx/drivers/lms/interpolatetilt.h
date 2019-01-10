//
//	interpolatetilt.h  -- pose in 3D space of the vehicle
//
//	J. Nagle
//	Team Overbot
//	February, 2005
//
//	We have to be able to obtain the position of the vehicle at any moment in recent time.
//	This requires interpolation
//
#ifndef INTERPOLATETILT_H
#define INTERPOLATETILT_H
#include <inttypes.h>
#include <deque>
#include "algebra3.h"
#include "mutexlock.h"
//
//	struct TiltPosition  -- one vehicle pose, with time and error
//
struct TiltPosition {
	float 	m_tilt;																					// vehicle pose
	uint64_t 	m_timestamp;																// timestamp of pose
public:
	TiltPosition(const float tilt,  uint64_t timestamp)								// constructor
	: m_tilt(tilt), m_timestamp(timestamp)
	{}
	TiltPosition() {}																				// empty constructor
};
//
//	class TiltPositions  -- where we were at some recent moments in time
//
class TiltPositions {
private:
	ost::Mutex m_lock;																			// lock
	std::deque<TiltPosition> m_poses;												// the saved poses
public:
	TiltPositions()	{}																			// constructor
	void addPose(const float pose, const uint64_t timestamp);
	bool getposeattime(TiltPosition& pose, uint64_t posetime, bool& toolate);
private:
	bool interpolate(const TiltPosition& prevpose, const TiltPosition& currpose, uint64_t posetime, TiltPosition& pose);
};
#endif // INTERPOLATETILT_H