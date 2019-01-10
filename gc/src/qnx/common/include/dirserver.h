/////////////////////////////////////////////////////////////////////////////
//
//    File: dirserver.h
//
//    Usage:
//        In the watchdog startup file (startfile.txt):
//              # to start the dirserver in a window with a menu
//				ID=DIR pterm dirserver -m
//
//              # to start the dirserver in a window with a menu
//              # in simulation mode
//              ID=DIR pterm dirserver -m -s
//
//        To send a message:
//            see dirservermenu.cc and actservermenu.cc for examples
//
//    Description:
//       The Direction Server controls the steering of the vehicle.  It
//       provides the following capabilities:
//           - initialization of the steering system, including homing
//             the motor encoder
//           - calibration of sensors, for development use
//           - controlling steering angle and angular rate
//           - providing the most recent steering angle to other processes
//       
//       A simulation mode is provided for testing purposes.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        November, 2003
//
/////////////////////////////////////////////////////////////////////////////

#ifndef DIRSERVER_H
#define DIRSERVER_H

#include "actserver.h"

// FIX - need to figure out these numbers for the final system
// Right now, angle is the lower steering angle, not the tire angle
// The angle rate is limited by the speed rating on the gearhead
#define DIRSERVER_ANGLE_MAX			( 420.0)	// degrees
#define DIRSERVER_ANGLE_MIN			(-420.0)	// degrees
#define DIRSERVER_ANGLE_RATE_MAX	( 200.0)	// degrees/sec

//
//  DirServer - Direction Server errors
//
//  example usage:
//      MyClass::MyFunc(DirServer::Err err)
//      {
//          if ( err == DirServer::ERR_STUCK ) {
//              ...
//          }
//      }
//
class DirServer: public ActServer {
public:
	enum Err {
		ERR_OK,							// SIMU, INIT, TARG, DATA
		ERR_INIT_NEEDED,				// TARG, DATA
		ERR_ANGLE_OUT_OF_RANGE,			// TARG
		ERR_ANGLE_RATE_OUT_OF_RANGE		// TARG
	};
	static char *ErrMsg(Err err)
	{
		switch ( err ) {
			case ERR_INIT_NEEDED:
				return "need to initialize Direction Server";
				break;
			case ERR_ANGLE_OUT_OF_RANGE:
				return "angle out of range";
				break;
			case ERR_ANGLE_RATE_OUT_OF_RANGE:
				return "angular rate out of range";
				break;
			default:
				return "unknown error";
				break;
		}
	}
};

//
//  DirServerMsgDATA - DATA: Direction Server data request
//
//  Same message structure used to set the targets and to get the
//  current targets.
//  To set the targets, use get=false.
//  To get the current targets, use get=true.
//
struct DirServerMsgDATA: public MsgBase {
	static const uint32_t k_msgtype = char4('D','A','T','A');
	DirServer::Err err;			// returned, ERR_0K=no error, otherwise error
	bool get;					// true=get targets, false=set targets
	float angle_target;			// returned, in degrees
	float angle_rate_max_target;// returned, in degrees/sec
	float angle;				// returned, in degrees
	float angle_rate;			// returned, in degrees/sec
};


//
//  DirServerMsg - all Direction Server messages as a union
//
//  Used as argument to MsgReceive. Size of union is size of largest 
//  acceptable message
//
union DirServerMsg {
	ActServerMsgSIMU m_simu;
	ActServerMsgINIT m_init;
	ActServerMsgVERB m_verb;
	DirServerMsgDATA m_data;
};

#endif // DIRSERVER_H
