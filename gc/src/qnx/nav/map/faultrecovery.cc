//
//	Fault recovery from driving faults
//
//	faultrecovery.cc  --  fault recovery for driving
//
//	J. Nagle
//	Team Overbot
//	July, 2005
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
#include "vehicledriver.h"
#include "logprint.h"
#include "tuneable.h"
#include "mapserver.h"
#include "signserver.h"
//
//	Constants
//
const Tuneable k_backup_distance("BACKUPDISTANCE", 0, 20, 3.0, "Distance to back up, m");		// distance to back up when trouble encountered
const Tuneable k_successful_recovery_distance("SUCCESSDISTANCE", 0, 20, 3.0, "Recovery successful if moved this far, m");		
//	Recovery actions
//
//	analyzeFault  -- examine fault code, decide on recovery action
//
//	Needs to be smarter.
//
static RecoveryAction analyzeFault(Fault::Faultcode newfault, RecoveryAction recoverystate, bool busy)
{
	switch (newfault) {										// fan out on fault type
	case Fault::movetimeout:								// CPU overrun
	case Fault::speedtimeout:								// Speed server timeout
	case Fault::nearcollision:								// VORAD tripped
	case Fault::slipback:										// slipped back, chassis enters hill holding mode
	case Fault::eventsequenceerror:					// usually means attempt to shift before stopped
		if (busy) return(recoverystate);				// if busy, no change		
		return(recovery_gazesweep);					// just do a rescan and restart
	
	case Fault::obstacle:										// obstacle detected
	case Fault::drivingfault:									// general driving fault
	case Fault::boundary:									// near boundary
	case Fault::mapcleared:								// map cleared, must sweep
	case Fault::tiltedpath:									// path selected tilts too much
		if (busy) return(recoverystate);				// if busy, no change		
		if (recoverystate == recovery_gazesweep)	// if already tried a gaze sweep
		{	
			return(recovery_reverse);					// try reversing
		}
		return(recovery_gazesweep);					// otherwise try a gaze sweep
	
	case Fault::none:											// no fault, should not happen
	case Fault::noposition:									// lost good GPS signal 	
	case Fault::manualmode:								// just in manual, keep waiting
	case Fault::nomission:									// no waypoints yet, keep waiting
	case Fault::cpuoverrun:								// CPU overrun
	case Fault::offcourse:									// off course, may be transient GPS error
		if (busy) return(recoverystate);				// if busy, no change		
		return(recovery_wait);								// just wait
		
	case Fault::done:											// all done
		return(recovery_missioncompleted);		// just park
		
	//	ADD MORE RECOVERY ACTIONS HERE
	
	default:															// all others
		logprintf("No recovery action for %s fault.\n", Fault::ErrMsg(newfault));
		if (busy) return(recoverystate);				// if busy, no change		
		return(recovery_gazesweep);					// ***TEMP*** try a gaze sweep, might get better
		////return(recovery_panic);							// trouble
	}
}
//
//	updateDrivingFault  -- update driving fault info on each cycle.
//
//	Terminates recovery actions that need to be terminated.
//
//	The last recovery action persists for a while, so that the next recovery action will be taken.
//
void VehicleDriver::updateDrivingFault(const vec3& vehpos3d)
{	vec2 vehpos(vehpos3d[0], vehpos3d[1]);	// ignore elevation here
	switch (m_lastrecoveryaction) {				// have we recovered?
	case recovery_reverse:
	{	double dist = (m_lastrecoverypos - vehpos).length();		// how far have we moved since recovery?
		if (dist > m_distancetoback)					// if moved enough
		{	m_distancetoback = 0;						// returned to normal
			m_lastrecoveryaction = recovery_normal;	// back to normal, but start an extra sweep at backup position
			logprintf("Reversing complete, back to normal mode.\n");
			getOwner().getLMSupdater().requestSweep();	// request a LIDAR sweep looking forward, no blind spot fill
			m_busy = true;										// go to busy
		}
		if (getVerboseLevel() >= 1)						// if verbose, track reversing info
		{	logprintf("Backed up %1.2f m out of %1.2f m from (%1.2f, %1.2f) to (%1.2f, %1.2f).\n",
			dist, m_distancetoback, m_lastrecoverypos[0], m_lastrecoverypos[1], vehpos[0], vehpos[1]); 
		}
		break;
	}
	
	case recovery_wait:										// just waiting
		if (!m_busy)												// if no longer busy
		{	m_lastrecoveryaction = recovery_normal;	
			logprintf("Wait complete, back to normal mode.\n");
			m_distancetoback = 0;							// get out of reverse mode
		}
		break;

	case recovery_gazesweep:							// stop, and rescan
		if (!m_busy)												// if no longer busy
		{	double dist = (m_lastrecoverypos - vehpos).length();		// how far have we moved since recovery?
			if (dist > k_successful_recovery_distance)					// if moved enough
			{	m_distancetoback = 0;						// returned to normal
				m_lastrecoveryaction = recovery_normal;	// back to normal
				logprintf("Gaze sweep successful,  back to normal mode.\n");
			}
		}
		break;
			
	case recovery_normal: 								// no problem
	case recovery_missioncompleted:				// mission completed
		m_distancetoback = 0;								// get out of reverse mode
		break;
				
	default:
		logprintf("Other recovery complete, back to normal mode.\n");
		m_lastrecoveryaction = recovery_normal;		// when in doubt, reset
		m_distancetoback = 0;								// get out of reverse mode
		break;
	}
}
//
//	handleDrivingFault  -- try to recover from driving faults.
//
//	Called from within driving loop task.
//
//	This needs to be smarter.
//
bool VehicleDriver::handleDrivingFault(Fault::Faultcode newfault)
{
	logprintf("Attempting to recover from %s fault.\n", Fault::ErrMsg(newfault));
	//	Get positional info
	vec3 startpos3d;
	vec2 startforward;
	float pitch, roll;
	double startspeed, cep;
	bool 	good = getPosition(startpos3d, startforward, startspeed, cep, pitch, roll);		// get current vehicle situation
	if (!good)													// if lost GPS position
	{	SetFault(Fault::noposition);					// lost GPS
		m_busy = true;										// wait for GPS to come back
		int stat = SignDisplayPriority(Fault::ErrMsg(Fault::noposition));	// display on sign
		if (stat < 0) {	logprintf("Display sign error: %s\n", strerror(errno));	}
		return(true);											// recover
	}
	vec2 startpos(startpos3d[0], startpos3d[1]);	// current position in 2D
	int stat = SignDisplayPriority(Fault::ErrMsg(newfault));		// display on sign
	if (stat < 0) {	logprintf("Display sign error: %s\n", strerror(errno));	}
	RecoveryAction newaction = analyzeFault(newfault, m_lastrecoveryaction, m_busy);	// decide what to do
	m_lastrecoveryaction = newaction;						// note new state
	switch (newaction) {
	case recovery_wait:										// just wait
		logprintf("Recovery action: wait.\n");
		m_busy = true;											// wait
		break;
		
	case recovery_gazesweep:							// stop, and rescan
		if (!m_busy)												// if not busy
		{	updateBlindSpot(startpos3d, startforward, false);	// fill in blind spot, small size
			getOwner().getLMSupdater().requestSweep();	// request a LIDAR sweep
			logprintf("Recovery action: gaze sweep.\n");
			m_distancetoback = 0;								// will go forward
			m_lastrecoverypos = startpos;					// where we started scanning
			m_busy = true;											// go to busy
		}
		break;

	case recovery_reverse:
		if (!m_busy)												// if not busy
		{	updateBlindSpot(startpos3d, startforward, false);	// fill in blind spot, small size
			getOwner().getLMSupdater().requestSweep();	// request a LIDAR sweep
			//	Reverse gear hack.  After trouble. back up a bit and try again
			m_distancetoback = k_backup_distance; 
			m_lastrecoverypos = startpos;					// where we started backing up
			logprintf("Recovery action: shifting to reverse.\n");
			m_busy = true;											// go to busy
		}
		break;
		
	case recovery_normal:									// no problem
		if (!m_busy)
		{	logprintf("Recovery action: return to normal driving.\n"); }
		else
		{	logprintf("Recovery action: normal, but still busy.\n"); }
		break;
	
	case recovery_missioncompleted:				// mission completed
		m_missioncompleted = true;						// begin shutdown
		SetFault(Fault::done);								// take lower levels down
		m_busy = true;											// busy from now on
		logprintf("Recovery action: mission completed, shut down.\n");
		break;

	default:															// all others
		logprintf("Unable to recover from %s fault.\n", Fault::ErrMsg(newfault));
		m_distancetoback = 0;								// will go forward
		return(false);											// no recovery
	}
	return(true);													// recovery action taken
}