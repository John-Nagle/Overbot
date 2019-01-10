//
//	gasecontrol.cc -- Gaze management
//
//	John Nagle
//	Team Overbot
//	April. 2005
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
#ifndef GAZECONTROL_H
#define GAZECONTROL_H
#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include "mutexlock.h"
#include "messaging.h"
//
//	GazeParams  -- parameters of gazing
//
//	One set for forward, one for reverse
//
//	All angles are in radians. 0 is down, PI/2 is forward
//
struct GazeParams {
	float	m_uplimit;															// highest angle to ever look
	float m_upsafelimit;													// highest angle for which "safe to move" is returned.
	float m_downlimit;														// lowest angle to look, before seeing vehicle
	float	m_upsign;															// +1 if up is positive
};
class LMSmapUpdater;
//
//	GazeController  -- controls one titltable device
//
class GazeController {
private:
	LMSmapUpdater& m_owner;										// owned by an LMS updater
	MsgClientPort m_tiltport;											// for talking to client
	GazeParams m_forwardparams;								// forward params
	GazeParams	m_reverseparams;								// reverse parameters
	ost::Mutex m_lock;														// lock
	float m_lasttilt;															// last tilt value seen
	float m_nexttilt;															// where we want to tilt to now
	float m_rangegoal;													// range we would like to see out to (m)
	float m_rangetolerance;											// allowed range error
	float m_bestrangeerr;												// closest range to goal seen in last sweep
	float m_besttilt;															// tilt for bestrangeerr
	bool m_backwards;													// true if moving backwards, and should be looking back
	bool m_initialized;														// true if initialized
	enum GazeState { gaze_tracking, gaze_sweeping_start, gaze_sweeping, gaze_sweeping_end, gaze_washing };
	GazeState m_gazestate;											// current state
public:
	GazeController(LMSmapUpdater& owner, const char* tiltserver, const GazeParams& fwd, const GazeParams& rev);				// owning object
	bool reportranges(float tilt, float avgrange, bool badscan);	// LIDAR reception reports what it sees here
	bool gazingdirsafe();												// true if safe to move - gaze covers immediate obstacles.
	bool gazeistracking();												// true if gaze is up, tracking and safe
	bool gazeissweeping();											// sweeping, presumably with vehicle stationary
	void setrangegoal(float rangegoal);							// we will try to servo to this range
	void requestsweep();												// request a sweep outward from the bumper
	void setdir(bool backwards);									// set backwards-look
private:
	//	Called from outside thread
	void adjust(float tilt, float avgrange, bool badscan);	// adjust the range goal
	void init();																	// initialize
	//	Called from within thread
	void* gazeThread();													// gazeThread -- local thread to manage tiliting.
    // need static function loopStart() for pthread_create
    static void* gazeThreadStart(void* arg)					// start the gaze thread
	{ return(reinterpret_cast<GazeController*>(arg)->gazeThread()); }
	bool dotilt(float pos, float speed, float& currenttilt);	// actually command a tilt
	const GazeParams& activeGazeParams() const		// get active set of gaze parameters
	{	return(m_backwards ? m_reverseparams : m_forwardparams);	}
	bool gazingdirvalid(float tilt);									// true if gazing in valid direction (not at bumper, etc.)
	void doGazeCycle();													// gaze cycle		
};
#endif // GAZECONTROL_H

