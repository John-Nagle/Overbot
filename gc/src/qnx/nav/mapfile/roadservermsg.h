////////////////////////////////////////////////////////////////////////////////
//
//    File:
//		roadmapservermsg.h
//
//    Usage:
//
//	In the watchdog startup file (startfile.txt):
//		ID=WAYPT wayptserver
//
//    Description:
//
//	(see roadserver.h)
//
//    Messages defined:
//
//	RoadServer::GetRoadList
//
//    Written By:
//
//      Tim Nicholson
//      Team Overbot
//      January 2004
//
////////////////////////////////////////////////////////////////////////////////

#ifndef ROADSERVERMSG_H
#define ROADSERVERMSG_H

#include "messaging.h"
#include "time.h"

#define WAYPTSERVER_ID	"WAYPT"


//
// Message definition
//

struct RoadPoint {
    double	x;			// distance East of origin (in meters)
    double	y;			// distance North of origin (in meters)
};

//
// Road Server Message Interface
//

class RoadServerMsg {

public:

    // Return codes from server
    enum Err {
	OK,				// no error
	EMPTY,
    };

    //
    // Messages Implemented by Server
    //

    //  GetRoadList
    //
	struct GetRoadList : public MsgBase {
		static const uint32_t k_msgtype = char4('R','D','G','R');
		RoadPoint	magPoint;
    };

    union MsgUnion {
	MsgBase				m_header;
	GetRoadList			m_getroadlist;
    } m_un;

};


#endif // ROADSERVERMSG_H
