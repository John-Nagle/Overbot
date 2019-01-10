/////////////////////////////////////////////////////////////////////////////
//
//    File: gearserver.h
//
//    Usage:
//        In the watchdog startup file (startfile.txt):
//              # to start the gearserver in a window with a menu
//				ID=GEAR pterm gearserver -m
//
//              # to start the gearserver in a window with a menu
//              # in simulation mode
//              ID=GEAR pterm gearserver -m -s
//
//        To send a message:
//            see gearservermenu.cc and actservermenu.cc for examples
//
//    Description:
//       The Gear Server controls the transmission shifting.  It provides
//       (or will provide) the following capabilities:
//           - initialization of the transmission system
//           - calibration of the potentiometer
//           - providing the current shifter setting
//           - control the shifter setting
//
//       A simulation mode is provided for testing purposes.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        November, 2003
//
/////////////////////////////////////////////////////////////////////////////

#ifndef GEARSERVER_H
#define GEARSERVER_H

#include "actserver.h"

//
//  GearServer - Gear Server positions and errors
//
//  example usage:
//      MyClass::MyFunc(GearServer::Err err)
//      {
//          if ( err == GearServer::ERR_STUCK ) {
//              ...
//          }
//      }
//
class GearServer: public ActServer {
public:
	// gear states
	enum Gear {
		GEAR_H,			// high
		GEAR_L,			// low
		GEAR_N,			// neural
		GEAR_R,			// reverse
		GEAR_TEST,		// gear location w/ arrow keys for testing
		GEAR_INIT,		// initial position (could be a gear)
		GEAR_MOVING,	// moving to a gear
		GEAR_TIMEOUT	// timed out trying to move to a gear
	};
	// gear messages
	static char *GearMsg(Gear gear)
	{
		switch ( gear ) {
			case GEAR_H:
				return "HIGH";
				break;
			case GEAR_L:
				return "LOW";
				break;
			case GEAR_N:
				return "NEUTRAL";
				break;
			case GEAR_R:
				return "REVERSE";
				break;
			case GEAR_INIT:
				return "Starting Position";
				break;
			case GEAR_MOVING:
				return "Moving";
				break;
			case GEAR_TIMEOUT:
				return "Timeout";
				break;
			default:
				return "unknown gear";
				break;
		}
	}
	// errors
	enum Err {
		ERR_OK,							// SIMU, INIT, VERB, DATA
		ERR_INIT_NEEDED,				// DATA
	};
	// error messages
	static char *ErrMsg(Err err)
	{
		switch ( err ) {
			case ERR_INIT_NEEDED:
				return "need to initialize Gear Server";
				break;
			default:
				return "unknown error";
				break;
		}
	}
};


//
//  GearServerMsgDATA - DATA: Gear Server data request
//
//  Same message structure used to set the target gear and to get the
//  current gear state.
//  To set the target gear, use get=false.
//  To get the current gear state, use get=true.
//
struct GearServerMsgDATA: public MsgBase {
	static const uint32_t k_msgtype = char4('D','A','T','A');
	GearServer::Err err;			// returned, ERR_0K=no error,otherwise error
	bool get;						// true=get gear state, false=set state
	GearServer::Gear gear_target;	// returned, target gear
	GearServer::Gear gear;			// returned, actual gear
};

//
//  GearServerMsg - all Gear Server messages as a union
//
//  Used as argument to MsgReceive. Size of union is size of largest 
//  acceptable message
//
union GearServerMsg {
	ActServerMsgSIMU m_simu;
	ActServerMsgINIT m_init;
	ActServerMsgVERB m_verb;
	GearServerMsgDATA m_data;
};

#endif // GEARSERVER_H
