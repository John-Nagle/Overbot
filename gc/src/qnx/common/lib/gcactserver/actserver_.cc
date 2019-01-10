////////////////////////////////////////////////////////////////////////////
//
//    File: actserver.cc
//
//    Usage:
//        See actserver.h.
//
//    Description:
//        The Actuator Server consists of the following threads:
//            - a main thread, which accepts messages
//            - a menu thread
//            - an initialization thread, that exists only during actuator
//              system initialization
//            - a timed-loop thread to collect data, if needed
//            - a timed-loop thread to do control, if needed
//              (this is for control based on an input other than an encoder;
//               control on an encoder is done in the MVP unit firmware)
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        December, 2003
//
/////////////////////////////////////////////////////////////////////////////

#include "actserver_.h"

ActServer_::ActServer_()
{
	ServerName("No Name");
	
	// initialize target thread variables
//	targetThreadSem = new Semaphore(0);
	
	// create timed-loop for control
	tc  = new TimedLoop(&controlLoopStart, this, 0.1, 13);
	controlLoopPeriod = 0.1;
	controlLoopTimeout = 2.0;

	// create timed-loop for data
	td  = new TimedLoop(&dataLoopStart, this, 0.1, 13);
	dataLoopPeriod = 0.1;
	
	// initialize server state
	simulation  = false;
	initialized = false;
	verbose     = true;
};

ActServer_::~ActServer_()
{
	// do nothing;
}

void ActServer_::ServerName(char *name)
{
	snprintf(serverName, ACTSERVER_NAME_LEN, name);
}

char *ActServer_::ServerName()
{
	return serverName;
}

void ActServer_::Simulation(bool iSimulation)
{
	simulation = iSimulation;
}

bool ActServer_::Simulation()
{
	return simulation;
}

void ActServer_::Initialized(bool iInitialized)
{
	initialized = iInitialized;
}

bool ActServer_::Initialized()
{
	return initialized;
}

void ActServer_::Verbose(bool iVerbose)
{
	verbose = iVerbose;
}

bool ActServer_::Verbose()
{
	return verbose;
}
