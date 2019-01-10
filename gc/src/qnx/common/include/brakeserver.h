/////////////////////////////////////////////////////////////////////////////
//
//    File: brakeserver.h
//
//    Usage:
//        In the watchdog startup file (startfile.txt):
//				ID=GEAR brakeserver -m
//
//        To send a message:
//            TBD
//
//    Description:
//       The Brake Server controls the brake pressure.  It provides
//       (or will provide) the following capabilities:
//           - initialization of the brake system
//           - providing the current pressure setting
//           - control the pressure setting
//
//       A simulation mode is provided for testing purposes.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        December, 2003
//
/////////////////////////////////////////////////////////////////////////////

#ifndef BRAKESERVER_H
#define BRAKESERVER_H

#include "actserver.h"

#define BRAKESERVER_PRESSURE_MAX			(100.0)	// percent
#define BRAKESERVER_PRESSURE_MIN			(0.0)	// percent

//
//  BrakeServer - Brake Server errors
//
//  example usage:
//      MyClass::MyFunc(BrakeServer::Err err)
//      {
//          if ( err == BrakeServer::ERR_STUCK ) {
//              ...
//          }
//      }
//
class BrakeServer: public ActServer {
public:
	// errors
	enum Err {
		ERR_OK,							// SIMU, INIT, VERB, DATA
		ERR_INIT_NEEDED,				// DATA
		ERR_PRESSURE_OUT_OF_RANGE,		// DATA
	};
	// error messages
	static char *ErrMsg(Err err)
	{
		switch ( err ) {
			case ERR_INIT_NEEDED:
				return "need to initialize Brake Server";
				break;
			case ERR_PRESSURE_OUT_OF_RANGE:
				return "target brake pressure out of range";
				break;
			default:
				return "unknown error";
				break;
		}
	}
};


//
//  BrakeServerMsgDATA - DATA: Brake Server data request
//
//  Same message structure used to set the target pressure and to get the
//  current pressure state.
//  To set the target gear, use get=false.
//  To get the current gear state, use get=true.
//
struct BrakeServerMsgDATA: public MsgBase {
	static const uint32_t k_msgtype = char4('D','A','T','A');
	BrakeServer::Err err;	// returned, ERR_0K=no error,otherwise error
	bool get;				// true=get brake state, false=set state
	float pressure_target;	// returned, target pressure 0.0-100.0%
	float pressure;			// returned, actual pressure 0.0-100.0%
};

//
//  BrakeServerMsg - all Brake Server messages as a union
//
//  Used as argument to MsgReceive. Size of union is size of largest 
//  acceptable message
//
union BrakeServerMsg {
	ActServerMsgSIMU   m_simu;
	ActServerMsgINIT   m_init;
	ActServerMsgVERB   m_verb;
	BrakeServerMsgDATA m_data;
};

#endif // BRAKESERVER_H
