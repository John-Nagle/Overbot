//
//	VORADmapupdate.cc  --  map updating based on sensor data
//
//	Functions specific to EATON VORAD radar
//
//	John Nagle
//	Team Overbot
//	February 2005
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
#include <time.h>
#include "mapserver.h"
#include "logprint.h"
#include "tuneable.h"
#include "algebra3.h"
#include "voradserver.h"
#include "timeutil.h"
#include "VORADmapupdate.h"
//
//	Configurable constants
//
const float k_target_size =- 2.0;											// assumed diameter of a target
const size_t k_vorad_queue_size = 25;								// more than we really need, but safe
//
//	Constructor
//
VORADmapUpdater::VORADmapUpdater(MapServer& owner, const vec3& voradoffset)
	: m_owner(owner), m_offset(voradoffset)
{
	//	Allocate queue items for queue. Use "new" only at startup.
	for (size_t i=0;  i < k_vorad_queue_size; i++)
	{	VoradServerMsgVDTG* blank = new VoradServerMsgVDTG;	// get a blank message
		m_emptyqueue.push(blank);										// fill queue with empty messages
	}
}
#ifdef OBSOLETE
//
//	handleRadarData  -- handle one report from the VORAD
//
//	***NEEDS QUEUEING, LIKE LIDAR***
//
void VORADmapUpdater::handleRadarData(const VoradServerMsgVDTG& msg)
{	if (msg.m_targetcount == 0) return;									// nothing to do
	ost::MutexLock lok(m_owner.getMapLock());						// lock map during update
	VehiclePose vehpose;														// get vehicle pose
	bool toolate;																		// too late, give up
	bool good = m_owner.getPoses().getposeattime(vehpose, msg.m_timestamp, toolate);	// get pose at time of timestamp
	if (!good)
	{	logprintf("No valid, current GPS data. VORAD data ignored.\n");	
		return;
	}
	for (int i=0; i<msg.m_targetcount; i++)								// for each target
	{	handleRadarTarget(msg.m_targets[i], vehpose.m_vehpose);	}						// handle
}
#endif // OBSOLETE
//
//	handleRadarData  -- handle one report from the VORAD
//
//	GPS/INS synchronization requires a queue of messages and of GPS poses,
//	which have to be matched and interpolated.
//
void VORADmapUpdater::handleRadarData(const VoradServerMsgVDTG& msg)
{	if (msg.m_targetcount == 0) return;									// nothing to do
	ost::MutexLock lok(m_owner.getMapLock());						// lock map during update
	//	Buffered line processing
    //	Put new line on queue
    //
    if (m_emptyqueue.empty())												// if no available buffers
    {	logprintf("VORAD queue stuck. Flushing.\n");					// should not happen
    	while (!m_linequeue.empty())										// flush queue
    	{	VoradServerMsgVDTG* p =  m_linequeue.front();		// get first entry
    		m_linequeue.pop();													// pop from line queue
    		assert(p);																	// must be nonempty
    		m_emptyqueue.push(p);											// push on empty queue, ignoring
    	}
    }
    assert(!m_emptyqueue.empty());										// must have a working buffer
    VoradServerMsgVDTG* work = m_emptyqueue.front();	// get a working buffer
    m_emptyqueue.pop();														// remove it from the queue
    assert(work);
    *work = msg;																	// save new line
    m_linequeue.push(work);													// push onto work queue
  	//	Process the queue, in order
  	while (!m_linequeue.empty())											// while lines to process
  	{	VoradServerMsgVDTG* first = m_linequeue.front();		// get first item
  		assert(first);																	// must get it
		VehiclePose vehpose;													// get vehicle pose
		bool toolate;			
		//	Try to get a relevant vehicle pose from interpolation								
		bool good = m_owner.getPoses().getposeattime(vehpose, first->m_timestamp, toolate);	// get pose at time of timestamp
		if ((!good) && (!toolate))												// if not ready to process the first line
		{	
			break;																		// try again later
		}
		//	We will use up this line
		if (good)																		// we have GPS data
		{	handlePosedData(*first, vehpose.m_vehpose);		// process line with vehicle position
		} else {
			logprintf("No valid, current GPS data. VORAD data ignored.\n");	
		}
		m_linequeue.pop();													// remove from work queue
		m_emptyqueue.push(first);										// move to empty queue
	}
}
//
//	handlePosedData -- handle data for which we have a pose
//
void VORADmapUpdater::handlePosedData(const VoradServerMsgVDTG& msg, const mat4& vehpose)
{
	for (int i=0; i<msg.m_targetcount; i++)								// for each target
	{	handleRadarTarget(msg.m_targets[i], vehpose);	}		// handle
}
//
//	handleRadarTarget  -- handle one target from the VORAD
//
//	Map is locked.
//
void VORADmapUpdater::handleRadarTarget(const VoradTargetItem& target, const mat4& vehpose)
{
	vec3 targ(target.m_targ_x, target.m_targ_y,0);					// target relative to scanner
	uint16_t minrange = uint16_t((targ.length())*100);			// distance to target						
	targ += m_offset;																// target relative to GPS
	vec3 pt = vehpose*targ;													// target relative to world
	if (m_owner.getVerboseLevel() >= 3)								// very verbose
	{	logprintf("VORAD target at (%1.2f, %1.2f) ->  (%1.2f %1.2f %1.2f)\n", targ[0], targ[1], pt[0], pt[1], pt[2]);	}
	uint32_t cyclestamp = m_owner.getMap().getcyclestamp();	// get map update cycle serial number
	float elev = pt[2];																// get elevation
	m_owner.updateCell(pt, CellData::NOGO, false, minrange, 0, elev , cyclestamp);		// update one cell, as a NOGO
}
