////////////////////////////////////////////////////////////////////////////
//
//    File: data.cc
//
//    Usage:
//        See actserver.h.
//
//    Description:
//        Routines related to the providing the server data.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        December, 2003
//
/////////////////////////////////////////////////////////////////////////////

#include "actserver_.h"

//
//  ActServer_::DataLoopStart - start loop to collect data
//
//  Returns nothing
//
void ActServer_::DataLoopStart()
{
	// launch timed loop
	if ( td->Start() != EOK ) {
		fprintf(stderr, "ActServer_::DataLoopStart - error starting loop\n");
	}
}

//
//  ActServer_::DataLoopPause - pause loop to collect data
//
//  Returns nothing
//
void ActServer_::DataLoopPause()
{
	// pause timed loop
	td->Pause();
}

//
//  ActServer_::DataLoopResume - resume loop to collect data
//
//  Returns nothing
//
void ActServer_::DataLoopResume()
{
	// resume timed loop
	td->Resume();
}

//
//  ActServer_::DataLoopStop - stop loop to collect data
//
//  Returns nothing
//
void ActServer_::DataLoopStop()
{
	// stop timed loop
	td->Stop();
}

//
//  ActServer_::dataLoop - loop that collects data
//
//  FIX - will eventually have to notify Trouble Server or the like if error
//        getting data; don't want to print error each tick
//
//  Returns nothing
//
void *ActServer_::dataLoop()
{
	// default is to do nothing
	
	return (void *) NULL;
}