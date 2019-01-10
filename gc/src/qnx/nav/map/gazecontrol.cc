//
//	gasecontrol.cc -- Gaze management
//
//	John Nagle
//	Team Overbot
//	April. 2005
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
#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include "gazecontrol.h"
#include "logprint.h"
#include "mutexlock.h"
#include "messaging.h"
#include "lidarserver.h"
#include "logprint.h"
#include "geocoords.h"
#include "LMSmapupdate.h"
#include "tuneable.h"

//
//	Constants
//
const int k_gaze_cycle_us = 100000;										// update gaze at 10Hz.
const float k_position_tolerance = deg2radians(0.1);				// done when within this number of degrees
const float k_nod_move_angle = deg2radians(10.0);				// quick nod down when there's a problem
////const float k_slow_move_angle = deg2radians(2.0);				// slow move for tracking
const float k_creep_move_angle = deg2radians(0.2);				// slow move for upward tracking
const float k_full_speed =  deg2radians(40);							// top speed (rad/sec)
const float k_sweep_speed =  deg2radians(7.5);						// sweep speed (rad/sec), too fast and we get gaps in the scan
const float k_default_range_tolerance = 0.5;							// within 1m is good enough
const int k_gaze_thread_priority = 14;										// not too high, this is non-critical
//
//	Tuneable constants
//
const Tuneable k_slow_move_angle_deg("TILTRATEDOWN", 0.0, 2.0, 0.2, "Downward tilt speed, deg/sec."); 
const float k_slow_move_angle = deg2radians(k_slow_move_angle_deg);		// slow move for tracking

//
//	Public functions -- called from outside the thread
//
//
//	Constructor
//
GazeController::GazeController(LMSmapUpdater& owner, const char* tiltserver, const GazeParams& fwd, const GazeParams& rev)
: 	m_owner(owner),
	m_tiltport(tiltserver, 0.2), 
	m_forwardparams(fwd),
	m_reverseparams(rev),
	m_lasttilt(0),																
	m_nexttilt(deg2radians(5.0)),									// some initial tilt, to get an initial move
	m_rangegoal(5.0),													// initial range goal, just to get going
	m_rangetolerance(k_default_range_tolerance),
	m_bestrangeerr(0),
	m_besttilt(0),
	m_backwards(false),
	m_initialized(false),
	m_gazestate(gaze_tracking)
{
}
//
//	init  -- do post-constructor intialization
//
//	Called on first good scan line.
//
void GazeController::init()
{
	if (m_initialized) return;
	//	Start tilt control thread
	ost::MutexLock lok(m_lock);										// lock
	if (m_initialized) return;												// avoid unlikely race condition
	pthread_create(0, 0, gazeThreadStart, this);				// start thread
	m_initialized = true;													// initialize
}
//	
//	setrangegoal -- set the range goal (in meters)
//
void GazeController::setrangegoal(float rangegoal)
{	
	ost::MutexLock lok(m_lock);										// lock
	m_rangegoal = rangegoal;										// set
}
//
//	requestsweep -- request a sweep cycle
//
void GazeController::requestsweep()
{	
	ost::MutexLock lok(m_lock);											// lock
	switch (m_gazestate) {
	case gaze_sweeping:													// if already gazing, don't do it again
	case gaze_sweeping_start:
	case gaze_sweeping_end:											// moving to best position at end of gaze sweep
		break;
		
	default:
		m_gazestate = gaze_sweeping_start;						// force a sweep
	}
	logprintf("Gaze sweep requested.\n");
}
//
//	reportranges -- called on each scan line to report some statistics about the scan
//
//	Returns false if line should not be processed for tilt reasons.
//	If "busy" is set, ignore line, wait. Tilting or washing is in progress, and scanner data is bad.
//
bool GazeController::reportranges(float tilt, float avgrange, bool badscan)
{
	ost::MutexLock lok(m_lock);										// lock
	init();																			// initialize if necessary
	float rangeerr = m_rangegoal - avgrange;				// range error
	////logprintf("GazeController::reportranges: badscan %d   avgrange %1.2f  tilt %1.2f\n",
	////	badscan, avgrange, tilt);										// ***TEMP***
	if ((!badscan) && (fabs(rangeerr) < m_rangetolerance) && gazingdirvalid(tilt))
	{	return(true);															// No problems.
	}																				// everything OK, continue
	//	May need to tilt.
	adjust(tilt, avgrange, badscan);								// adjust the range goal
	if (badscan) return(false);											// bad scan, can't use
	if (!gazingdirvalid(tilt)) return(false);							// fails, looking at vehicle body
	return(true);																// OK
}
//
//	gazingdirvalid  -- true if gazing in a direction where data is meaningful
//
bool GazeController::gazingdirvalid(float tilt)
{
	const GazeParams& gazelimits = activeGazeParams();	// get active gaze parameters
	//	Test tilt against limits that indicate we're looking at the vehicle.
	if (gazelimits.m_upsign > 0)
	{	return(tilt <= gazelimits.m_uplimit && tilt >= gazelimits.m_downlimit); }
	else 																									// looking backwards, signs reversed
	{	return(tilt >= gazelimits.m_uplimit && tilt <= gazelimits.m_downlimit); }
}
//
//	gazeistracking -- gaze is in tracking mode
//
//	(Otherwise wait)
//
bool GazeController::gazeistracking()  
{
	ost::MutexLock lok(m_lock);												// lock
	return(m_initialized && (m_gazestate == gaze_tracking) && gazingdirsafe());		// if actively tracking
}
//
//	gazeissweeping -- gaze is in sweep mode
//
//	(Otherwise wait)
//
bool GazeController::gazeissweeping()  
{
	ost::MutexLock lok(m_lock);												// lock
	return(m_initialized && (m_gazestate == gaze_sweeping));		// if actively tracking
}
//
//	gazingdirsafe  -- true if gazing in a safe direction, so we have some obstacle detection
//
bool GazeController::gazingdirsafe()
{
	ost::MutexLock lok(m_lock);												// lock
	float tilt = m_lasttilt;															// use last tilt
	const GazeParams& gazelimits = activeGazeParams();	// get active gaze parameters
	if (gazelimits.m_upsign > 0)
	{	return(tilt <= gazelimits.m_upsafelimit && tilt >= gazelimits.m_downlimit); }
	else 																									// looking backwards, signs reversed
	{	return(tilt >= gazelimits.m_upsafelimit && tilt <= gazelimits.m_downlimit); }
}
//
//	adjust  -- adjust range goal and desired tilt, based on return info
//
//	If the scan is bad, go to max down.
//	If the no-return fraction is nonzero, reduce the desired range. Fast.
//	If the average range is too big, tilt down. Slowly.
//	If the average range is too small, tilt up, but cautiously.
//
void GazeController::adjust(float tilt, float avgrange, bool badscan)
{	if (m_gazestate == gaze_sweeping)								// if in sweep mode, find closest to desired range
	{	if (!badscan)																	// if valid scan 
		{	float rangeerr = fabs(avgrange - m_rangegoal);		// absolute range error
			////logprintf("gaze adjust: avg range %1.2f m, range goal %1.2f m, tilt %1.1f deg.\n", avgrange, m_rangegoal, radians2deg(tilt)); // ***TEMP***
			if (rangeerr < m_bestrangeerr)								// if best so far
			{	m_bestrangeerr = rangeerr;									// save best
				m_besttilt = tilt;														// we will go to best tilt at end of sweep
			}
		}
	}	
	if (m_gazestate != gaze_tracking) return;							// only in track mode
	float newtilt = m_lasttilt;													// assume no change
	////logprintf("GazeController::adjust: badscan %d   avgrange %1.2f  tilt %1.2f\n", badscan, avgrange, tilt);												// ***TEMP***
	const GazeParams& gazelimits = activeGazeParams();	// get active gaze parameters
	if (badscan)																		// if no beam returned
	{	logprintf("GazeController::adjust: bad scan   avgrange %1.2f  tilt %1.2f\n", avgrange, tilt);	// not seeing the ground
		newtilt = tilt - gazelimits.m_upsign * k_nod_move_angle; // do a quick nod down
	}
	else if (avgrange > m_rangegoal + m_rangetolerance)	// if looking too far out
	{	newtilt = tilt - gazelimits.m_upsign * k_slow_move_angle; } // look slightly closer
	else if (avgrange < m_rangegoal - m_rangetolerance)	// if looking too close
	{	newtilt = tilt + gazelimits.m_upsign * k_creep_move_angle; }	// look very slightly further out
	//	Bound tilt to safe limits range
	float mintilt = std::min(gazelimits.m_downlimit, gazelimits.m_upsafelimit);
	float maxtilt = std::max(gazelimits.m_downlimit, gazelimits.m_upsafelimit);
	newtilt = std::min(std::max(newtilt, mintilt), maxtilt);			// apply bounds
	if (fabs(m_lasttilt - newtilt) > k_position_tolerance)			// if move needed
	{	////logprintf("Gaze change from %1.4f to %1.4f\n", m_lasttilt, newtilt);	// change tilt
		m_nexttilt = newtilt;														// set new tilt goal
	}
}
//
//	Gaze thread  -- talks to tilt controller, servoes tilt to maintain desired range.
//
//	This must be a separate thread, because the LMS calls us and we call the LMS. We would deadlock if we called back.
//
void* GazeController::gazeThread()
{
	struct sched_param param = {k_gaze_thread_priority};		// set priority
	pthread_setschedparam(pthread_self(),SCHED_RR,&param);	
	logprintf("Gaze control started.\n");							
	for (;;)																		// forever
	{	doGazeCycle();														// do one gaze cycle
		usleep(k_gaze_cycle_us);										// wait
	}
}
//
//	doGazeCycle -- do one gaze cycle
//
void GazeController::doGazeCycle()
{
	//	Decide what to do
	float tiltpos =0, tiltspeed = k_full_speed;					// what we want to happen
	{	ost::MutexLock lok(m_lock);									// lock data
		switch (m_gazestate) {										// fan out on gaze state
		case gaze_sweeping_start:									// going to new position
			if (fabs(m_lasttilt - activeGazeParams().m_downlimit) < k_position_tolerance)	// if at goal
			{	m_gazestate = gaze_sweeping;					// start actual sweep
				logprintf("Begin gaze sweep from %1.1f deg.\n", radians2deg(m_lasttilt));
				m_owner.resetAncientStamp();						//  data from here on will override old data
				tiltpos = activeGazeParams().m_uplimit;		// scan from downlimit to uplimit
				tiltspeed = k_sweep_speed;							// do slowly
				m_besttilt = tiltpos;										// default place to resume tracking
				m_bestrangeerr = 9999999;							// but when error is smaller than this, replace it
			} else {															// not at goal, command motion
				logprintf("Positioning for gaze sweep.\n");
				tiltpos = activeGazeParams().m_downlimit;		// go to up limit
				tiltspeed = k_full_speed;								// do this at full speed
			}
			break;
			
		case gaze_sweeping:											// sweep in progress
			if (fabs(m_lasttilt - activeGazeParams().m_uplimit) < k_position_tolerance)	// if at goal
			{	m_gazestate = gaze_sweeping_end;			// reposition to begin tracking
				//	Get best tilt from last gaze sweep and go there immediately.
				tiltpos = m_besttilt;										// use this position
				tiltspeed = k_full_speed;								// go there at full speed
				logprintf("Finishing gaze sweep, repositioning to tilt %1.4f deg.\n", radians2deg(tiltpos));
				break;															// tracking starts on next cycle
			} else {															// gaze sweep continues
				tiltpos = activeGazeParams().m_uplimit;
				tiltspeed = k_sweep_speed;							// do slowly
				////logprintf("Gaze sweep in progress: position %1.4f deg.\n", radians2deg(m_lasttilt));	// ***TEMP***
			}
			break;					
			
		case gaze_sweeping_end:									// moving to best position at end of gaze sweep
			if (fabs(m_lasttilt - m_besttilt) < k_position_tolerance)	// if at goal
			{	m_gazestate = gaze_tracking;						// return to normal tracking mode
				//	Continue from current position.
				tiltpos = m_lasttilt;											// use this position
				tiltspeed = 0;												// no movement needed right now
				m_nexttilt = tiltpos;										// resume tracking from here
				logprintf("End gaze sweep, resuming tracking at tilt %1.4f deg.\n", radians2deg(tiltpos));
				break;															// tracking starts on next cycle
			} else {
				tiltpos = m_besttilt;										// go to best tilt
				tiltspeed = k_full_speed;								// go there at full speed
				////logprintf("Gaze sweep in progress: position %1.4f deg.\n", radians2deg(m_lasttilt));	// ***TEMP***
			}
			break;									
			
		case gaze_tracking:												// normal case, do tracking
			tiltpos = m_nexttilt;											// get current goal
			break;
			
		default:
			break;																// just go to home position
		}
	}
	//	Do it.
	float currenttilt;
	bool good = dotilt(tiltpos, tiltspeed, currenttilt);			// request tilt
	//	Analyze results of doing it
	{	ost::MutexLock lok(m_lock);									// lock data
		m_lasttilt = currenttilt;											// save tilt, which will be zero if there is trouble.
	}
	if (!good)																	// tilt trouble or LMS may be resetting or washing
	{	logprintf("Tilt controller or LMS not ready.\n");
	}
}
//
//	dotilt  --  command a tilt angle
//
//	Blocking. Don't leave the object locked while this is in progress
//
bool GazeController::dotilt(float pos, float speed, float& currenttilt)
{
	LidarServerMsgTILT tiltmsg, tiltreply;							// tilt message
	tiltmsg.m_msgtype = LidarServerMsgTILT::k_msgtype;	// set header
	tiltmsg.m_tilt = pos;												// set params
	tiltmsg.m_tiltrate = speed;
	int stat = m_tiltport.MsgSend(tiltmsg, tiltreply);		// ask for a tilt
	if (stat < 0)															// send problem
	{	logprintf("Can't send tilt command: %s.\n",strerror(errno));
		currenttilt = 0;													// bogus tilt position (straight down)
	}
	currenttilt = tiltreply.m_tilt;										// return current tilt
	////logprintf("Gaze: at %1.4f, tilting to %1.4f\n", currenttilt, pos);	// ***TEMP***
	if (tiltreply.m_err != Controller::ERR_OK) return(false);			// controller trouble
	if (tiltreply.m_washing) return(false);					// wash cycle in progress
	return(true);															// tilt OK
}