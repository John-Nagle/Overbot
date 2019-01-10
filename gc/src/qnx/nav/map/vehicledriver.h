//
//	vehicledriver.h  -- main driving class
//
//	John Nagle
//	Team Overbot
//	February, 2005
//
//	Contains the main driving thread
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
#ifndef VEHICLEDRIVER_H
#define VEHICLEDRIVER_H

#include <vector>
#include <stdio.h>

#include "mutexlock.h"
#include "messaging.h"
#include "timedloop.h"
#include "dummyopensteer.h"
#include "terrainmap.h"
#include "waypoints.h"
#include "mapservermsg.h"
#include "gpsins_messaging.h"
//
//	RecoveryAction -- current or last recovery action
//
enum RecoveryAction { recovery_normal, recovery_gazesweep, recovery_wait, recovery_reverse, recovery_clearmap, 
	recovery_panic, recovery_missioncompleted };

class MapServer;																				// forward
//
//	class VehicleDriver  -- one driver, with NewSteer instance.
//
//	This is a timed loop, executed every 100ms.
//	It doesn't own the map, and the map must be locked separately.
//
class VehicleDriver: private TimedLoop {
private:
	MapServer& m_owner;																	// the owner
    MsgClientPort 	m_moveClientPort;												// client port for sending out move commands
    MsgClientPort 	m_gpsinsClientPort;												// client port for querying GPS/INS	    
	ReactiveDriver	m_VehicleDriver;													// driving level
	double			m_steertimestamp;													// last steering cycle start
	float	m_lastspeed;																		// vehicle speed, last control cyccle
	float m_lastcurvature;																	// steering curvature, last control cycle
	ost::Mutex		m_lock;																		// lock for this thread
	bool	m_busy;																				// busy, waiting for shifting, pause, etc.
	bool	m_initialized;																			// first initialization complete.
	MoveServerMsg::MsgMoveReply m_lastmovereply;						// reply from last move
	//	Recovery control
	double m_distancetoback;															// if nonzero, will try to back up this far
	RecoveryAction m_lastrecoveryaction;											// last recovery action
	vec2 m_lastrecoverypos;																// location at last recovery action start
	bool 	m_missioncompleted;															// set when mission completed
	//	GPS/Map resynchronization
	GPSINS_MSG::Err m_lastgpsinserrorstatus;									// last error status from GPS/INS
	uint64_t m_timedloopoverruns;														// tally times that timed loop overran
public:																								// called from OUTSIDE the thread
    VehicleDriver(MapServer& owner);												// constructor
    virtual ~VehicleDriver();																// destructor
    void stopDriving();																		// stop driving
    bool startDriving();																		// start driving
    bool initPosition();																			// initialize position from GPS before driving.
    void getStatus(MapServerMsg::MsgMapQueryReply& status);		// return current move status
    float getCurvature();																		// get turning curvature (LIDAR needs this)
    void setVerboseLevel(int lev);														// set verbosity level
private:																								// called from WITHIN the thread
	void code();																					// the timed loop
	bool driveStep();																			// one driving cycle
	bool commandMove(float dist, float curvature, float speed, MsgSpeedSet::Gear desiredgear, bool& busy,
		Fault::Faultcode& faultid); //  command a move
	void SetFault(Fault::Faultcode newfault);										// set and report a fault	
    int		getVerboseLevel();																// if verbose mode
	bool requestRunMode(bool& busy);												// request to go to run mode
	void initDriving();																			// initialize steering level
	bool 	getPosition(vec3& startpos, vec2& startforward, double& startspeed, double &cep, float& roll, float& pitch);		// get current vehicle situation
	bool updateMoveStatus();															// update status by querying move server
	void updateBlindSpot(const vec3& pos, const vec2& forward, bool fullfill);	// update "blind spot" under vehicle at startup
	void updateDrivingFault(const vec3& vehpos);							// clears problems
	bool checkOverrun();																	// check for CPU overrun
	bool handleDrivingFault(Fault::Faultcode newfault);						// handle problems
	void driveLog(const vec3& startpos, const vec2& startforward);	// logging
    MapServer& getOwner() { return(m_owner); }							// get owning object
    const ActiveWaypoints& getActiveWaypoints() const;					// access to active waypoints.
};

#endif // VehicleDriver_H
