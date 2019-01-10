////////////////////////////////////////////////////////////////////////////
//
//    File: message.cc
//
//    Usage:
//        See actserver.h.
//
//    Description:
//        Routines related to the message handling of the Actuator Server.
//
//        These functions need to be defined in the derived class so that
//        "msg" is large enough.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        December, 2003
//
/////////////////////////////////////////////////////////////////////////////

#include "actserver_.h"

void ActServer_::MessageThread()
{
	// default is to print this message
	fprintf(stderr, "\n ActServer_::MessageThread - Need to define ");
	fprintf(stderr, "MessageThread in derived Server\n");
	sleep(60);
}

void ActServer_::messageHandle()
{
	// default is to do nothing
}