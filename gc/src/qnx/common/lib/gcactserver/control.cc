////////////////////////////////////////////////////////////////////////////
//
//    File: control.cc
//
//    Usage:
//        See actserver.h.
//
//    Description:
//        Routines related to the actuator control loop.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        December, 2003
//
/////////////////////////////////////////////////////////////////////////////

#include "actserver_.h"

//
//  ActServer_::ControlLoopStart - start loop to control actuator
//
//  Returns nothing
//
void ActServer_::ControlLoopStart()
{
	// launch timed loop
	if ( tc->Start() != EOK ) {
		fprintf(stderr, "ActServer_::ControlLoopStart - error starting loop\n");
	}
}

//
//  ActServer_::ControlLoopPause - pause loop to control actuator
//
//  Returns nothing
//
void ActServer_::ControlLoopPause()
{
	// pause timed loop
	tc->Pause();
}

//
//  ActServer_::ControlLoopResume - resume loop to control actuator
//
//  Returns nothing
//
void ActServer_::ControlLoopResume()
{
	// resume timed loop
	tc->Resume();
}

//
//  ActServer_::ControlLoopStop - stop loop to control actuator
//
//  Returns nothing
//
void ActServer_::ControlLoopStop()
{
	// stop timed loop
	tc->Stop();
}

//
//  ActServer_::controlLoop - loop that controls actuator
//
//  Returns nothing
//
void *ActServer_::controlLoop()
{
	// default is to do nothing
	
	return (void *) NULL;
}