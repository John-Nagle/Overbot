/////////////////////////////////////////////////////////////////////////////
//
//    File: actserver.h
//
//    Usage:
//        In a source file:
//            #include "actserver.h"
//
//        In a Makefile:
//            LIBS+=gcactserver
//
//    Description:
//       The Actuator Server class is a base class from which other
//       actuator control servers can be derived (e.g., the gearserver).
//
//       This allows all the basic functionality common to all the
//       actuator-related servers to be located in one place.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        December, 2003
//
/////////////////////////////////////////////////////////////////////////////

#ifndef ACTSERVER_H
#define ACTSERVER_H

#include "messaging.h"

//
//  ActServer - Actuator Server errors
//
//  example usage:
//      MyClass::MyFunc(ActServer::Err err)
//      {
//          if ( err == ActServer::ERR_OK ) {
//              ...
//          }
//      }
//
class ActServer {
public:
	// In the derived class, define error types, as in the following example:
	enum Err {
	     ERR_OK,						// SIMU, INIT, VERB, DATA
	     ERR_INIT_NEEDED,				// DATA
	};
	
	// In the derived class, define error messages, as in the following example:
	static char *ErrMsg(Err err) {
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
//  ActServerMsgSIMU - SIMU: Actuator Server simulation mode
//
//  Same message structure used to get and set the simulation mode.
//  To turn on the simulation mode, use get=false, simulation=true.
//  To turn off the simulation mode, use get=false, simulation=false.
//  To query the simulation mode, use get=true.
//
//  For now, the simulation mode will return the target position as the 
//  actual position.
//
struct ActServerMsgSIMU: public MsgBase {
	static const uint32_t k_msgtype = char4('S','I','M','U');
	ActServer::Err err;		// returned, ERR_0K=no error, otherwise error
	bool get;				// true=get, false=set
	bool simulation;		// true=on, false=off
};

//
//  ActServerMsgINIT - INIT: Actuator Server initialization command
//
//  Same message structure used to get the initialization status and to
//  initialize.
//  To initialize the steering system, use get=false.
//  To get the initialization status, use get=true.
//
struct ActServerMsgINIT: public MsgBase {
	static const uint32_t k_msgtype = char4('I','N','I','T');
	ActServer::Err err;			// returned, ERR_0K=no error, otherwise error
	bool get;					// true=get, false=initialize
	bool initialized;			// returned, true=done initializing, false=not
};

//
//  ActServerMsgVERB - VERB: Actuator Server verbose mode
//
struct ActServerMsgVERB: public MsgBase {
	static const uint32_t k_msgtype = char4('V','E','R','B');
	ActServer::Err err;		// returned, ERR_0K=no error, otherwise error
	bool get;				// true=get, false=set
	bool verbose;			// true=on, false=off
};

//
//  ActServerMsg - all default Actuator Server messages as a union
//
//  Used as argument to MsgReceive. Size of union is size of largest 
//  acceptable message.
//
union ActServerMsg {
	ActServerMsgSIMU m_simu;
	ActServerMsgINIT m_init;
  	ActServerMsgVERB m_verb;
};

#endif // ACTSERVER_H
