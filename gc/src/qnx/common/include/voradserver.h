/////////////////////////////////////////////////////////////////////////////
//
//    File: voradserver.h
//
//    Usage:
//        In the watchdog startup file (startfile.txt):
//				ID=SIGN voradserver /dev/ser1 -v 
///
//    Description:
//       The Vorad Server reads from the VORAD radar and provides information about nearby vehicles.
//       
//
//    Written By:
//        John Nagle
//        Team Overbot
//        November, 2003
//
/////////////////////////////////////////////////////////////////////////////

#ifndef VORADSERVER_H
#define VORADSERVER_H
#include <strings.h>
#include "messaging.h"

//
//  VoradServer - Vorad messages
//
//
//  VoradServerMsgVDTG - VDRQ: Vorad Target Request
//
// 	Target request to VORAD
//
struct VoradServerMsgVDRQ: public MsgBase {
	static const uint32_t k_msgtype = char4('V','D','R','Q');
	//	To VORAD
	float m_steeringangle;								// current steering angle (deg, <0 is left)
	float m_speed;											// forward speed, meters/second
};
//
//  VoradServerMsgVDTG - VDGT: Vorad Target
//
// 	Target report from VORAD
//
//	Reports targets with their "threat level"
//
struct VoradTargetItem									// data for one target
	{	
		bool m_movingtarget;							// the target appears to be moving
		float m_collisionthreatlev;						// collision threat level (>1, trigger E-stop NOW)			
		float m_targ_x, m_targ_y;						// relative vector to target (meters)
		float m_targ_rrange;								// range rate (<0 is getting closer)
	};																// reported targets
//
//	The most threatening target is the one whose range, N seconds from
//	now, is the closest. 
//
struct VoradServerMsgVDTG: public MsgBase {
	static const uint32_t k_msgtype = char4('V','D','T','G');
	//	From VORAD
	uint64_t	m_timestamp;							// nanoseconds since epoch
	static const int k_maxtargets = 7;				// max targets reported
	bool m_estop;											// collision likely. Trigger E-stop NOW
	uint8_t 	m_targetcount;								// total target count (if 0)
	VoradTargetItem m_targets[k_maxtargets];	// the targets
};

//
//  VoradServerMsg - all VORAD Server messages as a union
//
//  Used as argument to MsgReceive. Size of union is size of largest 
//  acceptable message
//
union VoradServerMsg {
	VoradServerMsgVDRQ m_vdrq;					// the request
};

#endif // VORADSERVER_H
