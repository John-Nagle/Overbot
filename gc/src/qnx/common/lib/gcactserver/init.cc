////////////////////////////////////////////////////////////////////////////
//
//    File: init.cc
//
//    Usage:
//        See actserver.h.
//
//    Description:
//        Routines related to the initialization of an Actuator Server.
//
//        This is a separate thread from the message-receiving thread so
//        it is okay that it may block while waiting for the MVP Server.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        December, 2003
//
/////////////////////////////////////////////////////////////////////////////

#include "actserver_.h"

//
//  ActServer_::InitializeThread - launch thread to do the initialization
//
//  Returns nothing
//
void ActServer_::InitializeThread()
{
	// initialize Direction Server
	if ( pthread_create(NULL, NULL, &initThreadStart, this) != EOK ) {
		fprintf(stderr, 
			"ActServer_::InitializeThread - error calling pthread_create()\n");
	}
}

//
//  ActServer_::initThread - where the actual initialization work is done
//
//  Returns nothing effectively
//
void *ActServer_::initThread()
{
	// default action
	initialized = true;
	
	return (void *) NULL;	// so compiler won't complain
}
