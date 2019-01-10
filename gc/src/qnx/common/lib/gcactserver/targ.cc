////////////////////////////////////////////////////////////////////////////
//
//    File: targ.cc
//
//    Usage:
//        See actserver.h.
//
//    Description:
//        Routines related to setting the target actuator configuration.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        December, 2003
//
/////////////////////////////////////////////////////////////////////////////

#include "actserver_.h"

//
//  ActServer_::TargetThread - launch thread to do issue target command
//
//  Returns nothing
//
void ActServer_::TargetThread()
{
	// create thread to issue message to MVP Server (may be blocked)
	if ( pthread_create(NULL, NULL, &targThreadStart, this) != EOK ) {
		fprintf(stderr, 
		        "ActServer_::TargetThread - error calling pthread_create\n");
	}
}

void *ActServer_::targThread()
{
	// default is to do nothing; thread will be launched then exit
	
	// Example: code to create target thread that persists
	// It "wakes up" when the target thread semaphore is posted
	//
	// static bool firstTime = true;
	// // last variables here
	// MVPServer::Err err;
	//
	// for ( ; ; ) {		// persist forever
	//	// wait for target values
	//	targetThreadSem->wait();
	//
	//	// lock and save target values
	//	{
	//		MutexLock lok(targetLock);
	//		// assign save value to actual values
	//	}
	//	
	//	// after first time, check if targets changed; don't send if not
	//	if ( firstTime ) {
	//		firstTime = false;
	//	} else {
	//		if ( targetSave == targetLast ) {
	//			continue;			// don't send targets if no change
	//		}
	//	}
	//	
	//	//
	//	// issue control commands here
	//	//
	//	// if PID loop on MVP unit,
	//	//     issue target instructions, and move
	//	//
	//	// if software control loop,
	//	//     zero current limit, home node, enable node
	//	//     start control loop if not already started and set period
	//	
	//	// store last gear target sent
	//	targetLast = targetSave;
	//}
	
	return (void *) NULL;	// so compiler won't complain
}