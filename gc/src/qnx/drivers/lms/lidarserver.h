////////////////////////////////////////////////////////////////////////////
//
//    File: lidarserver.h
//
//    Usage:
//        In the watchdog startup file (startfile.txt):
//				ID=DIR lidarserver
//
//
//    Description:
//       The Lidar Server controls SICK Laser Range-finder Unit.  It
//       provides (or will provide) the following capabilities:
//           - initialization of the laser range-finder system.
//           - providing the most recent laser data to other processes
//
//    Written By:
//        Eric Seidel
//		  (Modeled after code from Celia Oakley)
//        Team Overbot
//        December, 2003
//
//		Major revisions by J. Nagle, December 2004,
//
/////////////////////////////////////////////////////////////////////////////

#ifndef LIDARSERVER_H
#define LIDARSERVER_H
#include "messaging.h"
#include "controller.h"


////#define LMS_MAX_DATA_POINTS		401
const size_t LMS_MAX_DATA_POINTS = 181;										// we run in 1 degree mode


//
//  LidarServer - Laser Range-finder Server errors
//
//  example usage:
//      MyClass::MyFunc(LidarServer::Err err)
//      {
//          if ( err == LidarServer::ERR_STUCK ) {
//              ...
//          }
//      }
//
class LidarServer {
public:
	enum Err {
		ERR_OK,						// good
		ERR_LMS_FAULT,			// LMS has failed
		ERR_TILT_FAULT			// tilt head has failed
	};
};

//
//  LidarServerMsgINIT - INIT: Laser Range-finder Server initialization command
//
//  Same message structure used to get the initialization status and to
//  initialize.
//  To reset the LIDAR system, use get=false.
//  To get the initialization status, use get=true.
//
struct LidarServerMsgINIT: public MsgBase {
	static const uint32_t k_msgtype = char4('I','N','I','T');
	LidarServer::Err m_err;			// returned, ERR_0K=no error, otherwise error
	bool m_get;							// true=get, false=initialize
	bool m_initialized;					// returned, true=done initializing, false=not
};
//
//  LidarServerMsgTILT - TILT: request to point tilt head
//
//  Same message structure used to get the initialization status and to
//  initialize.
//	
//	Replies with the same format, with the actual tilt angle
//
struct LidarServerMsgTILT: public MsgBase {
	static const uint32_t k_msgtype = char4('T','I','L','T');
	Controller::Err m_err;				// returned, ERR_0K=no error, otherwise error
	float	m_tilt;							// tilt angle, radians, 0 is down, PI/2 is forward
	float m_tiltrate;						// tilt rate, radians/sec, unsigned
	bool m_washing;					// wash cycle in progress, cannot tilt now
};
//
//	LidarServerMsgWASH -- request wash cycle
//
//	No reply, just status
//
struct LidarServerMsgWASH: public MsgBase {
	static const uint32_t k_msgtype = char4('W','A','S','H');
};
//
//	LidarScanLineHeader  -- header for a scan line
//
struct LidarScanLineHeader {
	uint64_t	m_timestamp;											// CLOCK_REALTIME, in nanoseconds
	float			m_tilt;														// tilt angle of unit. 0=straight down, pi/2=straight ahead
	uint8_t		m_sensorid;												// which LMS (future expansion)
	uint8_t		m_statusByte;											// LMS status byte
	uint8_t		m_scanIndex;											// scan index (cyclic)
	uint8_t		m_unused1;												// (future expansion)
	uint16_t	m_unused2;												// (future expansiion, and fill to word)
	uint16_t	m_valueCount;											// count of m_range
};
//
//	LidarScanLine -- one scan line
//
struct LidarScanLine {
	LidarScanLineHeader m_header;								// the header
	uint16_t	m_range[LMS_MAX_DATA_POINTS];			// depth data (m_valuecount entries)
};

//
//  LidarServerMsgLISN - LISN Laser Range-finder Server data message
//
//	This is sent, by the LIDAR server acting as a client, to a specified using server.
//	We do it this way because the data volume is so high.
//
struct LidarServerMsgLISN: public MsgBase {
	static const uint32_t k_msgtype = char4('L','I','S','N');	// "LIdar SCan"
	LidarServer::Err m_err;			// returned, ERR_0K=no error, otherwise error
	LidarScanLine m_data;			// the scan line
};

//
//  LidarServerMsg - all LIDAR Server input messages as a union
//
//  Used as argument to MsgReceive. Size of union is size of largest 
//  acceptable message
//
union LidarServerMsg {
	LidarServerMsgINIT m_init;
	LidarServerMsgTILT m_tilt;
	LidarServerMsgWASH m_wash;
};

#endif // LIDARSERVER_H
