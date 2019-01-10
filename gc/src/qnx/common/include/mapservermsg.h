////////////////////////////////////////////////////////////////////////////////
//
//    File:
//	 mapservermsg.h
//
//    Usage:
//        In the watchdog startup file (startfile.txt):
//		ID=MAP mapserver
//
//    Description:
//       
//       A simulation mode is provided for testing purposes.
//
//    Written By:
//        Achut Reddy
//        Team Overbot
//        December, 2003
//
/////////////////////////////////////////////////////////////////////////////////

#ifndef MAPSERVERMSG_H
#define MAPSERVERMSG_H

#include <stdio.h>

#include "lidarserver.h"
#include "voradserver.h"
#include "moveservermsg.h"
#include "simplesonar.h"

const char MAPSERVER_ID[] = "MAP";						// watchdog ID of this server


class MapServerMsg {

public:
    //
    //  MsgUnion - all Map server messages as a union
    //
    //  Used as argument to MsgReceive. Size of union is size of largest 
    //  acceptable message
    //
    union MsgUnion {
	MsgBase							m_header;	
	LidarServerMsgLISN			m_lidardata;							// incoming LIDAR data
	VoradServerMsgVDTG		m_voraddata;						// incoming VORAD data
	MoveServerMsg::MsgMoveQuery		m_query;				// query only, no command
	MoveServerMsg::MsgMoveStop		m_stop;					// do E-stop
	SonarObstacleMsgReq		m_sonar;								// incoming SONAR data
    } m_un;

	//
	//	Reply types
	//
	struct MsgMapQueryReply {
		MoveServerMsg::MsgMoveReply m_movereply;			// reply from move server
		int		m_waypointserial;													// current waypoint serial number
	};
};

#endif // MAPSERVERMSG_H
