////////////////////////////////////////////////////////////////////////////////
//
//    File:
//	moveservermsg.h
//
//    Usage:
//
//	In the watchdog startup file (startfile.txt):
//		ID=MOVE moveserver
//
//    Description:
//
//	Takes high-level move commands from mapserver and implements them
//	by translating them into low-level commands to the vehicle actuator
//	servers (steering, throttle, brake, trasmission).
//       
//      Performs sanity checks on the move commands.  It contains a simple
//      dynamics model and will refuse to move in any manner which threatens
//      vehicle safety.  If necessary it will reduce the specified speed until
//      it is deemed safe.
//      
//      Also checks for potential collisions by querying the vorad server;
//	if a collision appears imminent, it will halt the vehicle in spite
//	of any move commands.
//
//	Keeps track of predicted position vs. actual position error so the
//	algorithm can adapt and adjust for it.
//      
//      
//      Simulation modes are provided for testing purposes
//		(see moveservermenu.h)
//
//    Messages received:
//
//	MoveServer::MsgMOVE
//	
//    Messages sent:
//
//
//    Written By:
//
//      Achut Reddy
//      Team Overbot
//      January 2004
//
////////////////////////////////////////////////////////////////////////////////

#ifndef MOVESERVERMSG_H
#define MOVESERVERMSG_H

#include "messaging.h"
#include "speedservermsg.h"
#include "faultcodes.h"

#define MOVESERVER_ID	"MOVE"


//
// Move Server Message Interface
//

class MoveServerMsg {

public:
    //
    //	Messages Implemented by Move Server
    //
	//
    //  MsgMOVE - Move vehicle a specified distance, at a specified
    //		  (maximum) speed, and turning in a specified curvature.
    //		  Replaces any move command that might be currently in 
    //		  execution.
    //
    //	Returns a MsgMoveReply
    //
    struct MsgMove: public MsgBase {

	static const uint32_t k_msgtype = char4('M','V','M','O');

	float	m_distance;	// distance to move (in meters)

	float	m_speed;		// maximum speed (in meters/sec)
				// Move Server will not exceed this speed

	float	m_curvature;	// 1/r, r = radius of curvature (in meters^-1)
				// by convention: 
				//	zero curvature means straight ahead,
				//	negative curvature means turn left,
				//	positive curvature means turn right
	MsgSpeedSet::Gear m_gear;					// desired gear
    };
    //
    //	MsgMoveReply  -- reply from MsgMove
    //
    struct MsgMoveReply {
    	MsgSpeedSetReply m_speedreply;	// reply from speed server
    	float m_lastdistance;							// last distance request
    															// more fields may be added
     };
    
    //
	//	MsgMoveStop
	//
	//	Anybody can send this, which will force a recoverable emergency stop.
	//	When in doubt, send this.
	//
	typedef MsgSpeedStop MsgMoveStop;					// same as with speed server
	//	
	//	MsgMoveQuery -- query server status
	//
	//	Anybody can send this, and it just returns the current status as a MsgMoveReply.
	//
	struct MsgMoveQuery: public MsgBase {
		static const uint32_t k_msgtype = char4('M','V','Q','U');

	};

    //  MsgUnion  - all Move Server messages as a union
    //
    //  Used as argument to MsgReceive. Size of union is size of largest 
    //  acceptable message
    union MsgUnion {
		MsgBase				m_header;
		MsgMove				m_move;								// command a move
		MsgMoveQuery	m_query;								// query only, no command
		MsgMoveStop		m_stop;									// request E-stop
    } m_un;

};


#endif // MOVESERVERMSG_H
