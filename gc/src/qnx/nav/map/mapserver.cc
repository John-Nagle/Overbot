//
//	mapserver.cc  -- main object of the MAP program
//
//	Perliminary version by Achut Reddy. Completely rewritten by John Nagle.
//
//	John Nagle
//	Team Overbot
//	December, 2004
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

#include <unistd.h>

#include "mapserver.h"
#include "logprint.h"
#include "tuneable.h"
#include "mapservermsg.h"
#include "timeutil.h"

//
//	Configuration constants
//
const Tuneable k_cellspermeter("CELLSPERMETER",1.0,10.0,5.0,"map cells per meter (1/m)");
const Tuneable k_mapsizeincells("CELLSACROSSMAP",100,2000,1000,"cells across the map");
const size_t k_mapdimension = k_mapsizeincells;		// map size in cells, as an integer
//
//	Sensor positions relative to GPS antenna.
//	Z values are relative to ground, not GPS antenna
//
const vec3 k_scanneroffset(1.65,0,2.08);						// offset between GPS antenna and LMS scanner, meters
const vec3 k_voradoffset(2.80,0,0.40);							// offset between GPS antenna and VORAD radar, meters
//
// Class member functions
//

// constructor
MapServer::MapServer()
:	m_serverPort(0.0),
	m_verboselevel(0),													// debug off
	m_map(*this, k_mapdimension, k_cellspermeter),		// the map
	m_lmsupdater(*this, k_scanneroffset)	,					// main SICK LMS support
	m_voradupdater(*this, k_voradoffset),						// VORAD support
	m_driver(*this)															// the driving thread
{
}

// destructor
MapServer::~MapServer()
{
}

void
MapServer::setVerboseLevel(int lev)
{
    m_verboselevel = lev;
    m_driver.setVerboseLevel(lev);								// set in driver, too
    logprintf("Logging verbosity set to %d.\n", lev);		// note level
}
//
//	messageThread -- usual QNX input processing loop
//
void
MapServer::messageThread()
{
	logprintf("Message thread: priority %d\n", getthreadpriority());		// ***TEMP***
    // create a channel, tell watchdog
    int stat = m_serverPort.ChannelCreate(_NTO_CHF_FIXED_PRIORITY);	// Don't let the GUI program slow us down
    if  (stat ) {
		perror("MapServer::messageThread - ChannelCreate failed.\n");
		m_serverPort.Dump();
		abort();																						// should never happen
    }    
	//	Start any other threads that need starting
	m_roadfollower.init();																			// start the road follower thread
    // loop collecting messages forever
    while (true) {
		MapServerMsg msg;																			// working msg
		// wait for message
		int rcvid = m_serverPort.MsgReceive(msg);
	
		// handle errors
		if ( rcvid < 0 ) {
	 	   fflush(stdout);
	 	   perror("MapServer::messageThread - MsgReceive failed\n");
	 	   sleep(1);		// avoid tight loop if repeated trouble FIX
	 	   continue;
		} else if ( rcvid == 0 ) {
	 	   perror("MapServer::messageThread - received a pulse\n");
		    // pulses don't require a reply
		    continue;
		}
	
		// handle message
		handleMessage(rcvid, msg);																// handles the reply
	
		// tell the watchdog we are still alive FIX
		////serverport.watchdogreset();
    }
}
//
//	handleMessage  --  handle one message
//
void MapServer::handleMessage(int rcvid, MapServerMsg& msg)
{
#ifdef OBSOLETE															// just too verbose
    if (m_verbose) {
		logprintf("MapServer::handleMessage - received message %d\n",
	  		 msg.m_un.m_header.m_msgtype );
    }
#endif // OBSOLETE
    
    // Dispatch on message type
    switch ( msg.m_un.m_header.m_msgtype ) {

    case LidarServerMsgLISN::k_msgtype:						// incoming LMS LIDAR data msg
		m_lmsupdater.LMShandleLidarData(msg.m_un.m_lidardata.m_data);	// handle lidar data
		MsgError(rcvid, EOK);											// no data is returned
		return;
		
	case VoradServerMsgVDTG::k_msgtype:					// incoming VORAD RADAR data msg
		handleVorad(msg.m_un.m_voraddata);					// handle VORAD message
		MsgError(rcvid, EOK);											// no data is returned
		return;
		
	case MsgSpeedStop::k_msgtype:								// incoming E-stop message
		handleStop(msg.m_un.m_stop);							// handle stop
		MsgError(rcvid, EOK);											// no data is returned
		return;
		
	case MoveServerMsg::MsgMoveQuery::k_msgtype:	// incoming
		handleQuery(rcvid);												// no incoming data
		return;
		
	case SonarObstacleMsgReq::k_msgtype:					// sonar message
		handleSonar(msg.m_un.m_sonar);						// handle a sonar event
		MsgError(rcvid, EOK);											// no data is returned
		return;
		    
    default:																		// bad message type
	   	logprintf("MapServer::handleMessage - unknown message type: 0x%8x\n", msg.m_un.m_header.m_msgtype);
	   	MsgError(rcvid,EINVAL);										// unknown type
	   	return;
    }    
}
//
//	handleStop -- handle an emergency stop request
//
//	Most servers handle this.
//	Caller handles reply.
//
void MapServer::handleStop(const MsgSpeedStop& msg)
{
	stopMission();															// just stop mission, for now.
	logprintf("Misssion stopped due to remote manual stop request.\n");
}
//
//	handleQuery  -- handle status query
//
//	Just returns the latest reply from the move server.
//
//	We handle reply
//
void MapServer::handleQuery(int rcvid)
{	MapServerMsg::MsgMapQueryReply reply;				// the reply
	m_driver.getStatus(reply);										// get status from driver level
	MsgReply(rcvid, reply);												// send the reply
}
//
//	handleVorad  -- incoming data from VORAD radar
//
//	Most servers handle this.
//	Caller handles reply.
//
void MapServer::handleVorad(const VoradServerMsgVDTG& msg)
{
	m_voradupdater.handleRadarData(msg);					// handled by VORAD updater
}
//
//	handleSonar -- incoming data from sonars
//
//	Caller handles reply.
//
void MapServer::handleSonar(const SonarObstacleMsgReq& msg)
{
	//	***MORE*** must handle message
}


