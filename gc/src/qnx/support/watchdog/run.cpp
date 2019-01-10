//
//	Watchdog  -- top-level watchdog program for QNX real-time applications
//
//	File run --  runs and monitors the programs
//
//	J.	Nagle
//	August, 2002
//
#include <stdio.h>
#include <assert.h>
#include <unistd.h>
#include "messaging.h"
#include "watchdog.h"
#include "logprint.h"
//
//	run -- run and monitor requested programs
//
//	Starts the server, watchdog, and monitor threads.
//
int Watchdog::run()
{
	//	Start server thread.
	m_serverchid = ChannelCreate(_NTO_CHF_UNBLOCK);	// create a channel
	if (m_serverchid <= 0) panic("Unable to create server channel");
	int stat = pthread_create(&m_serverthread,0,startserver,this);	// start server thread
	if (stat) panic("Unable to start server thread.");		// handle failure
	//	Prepare all programs to run. This should not fail; any errors should have been detected already.
	for (vector<WatchedProgram*>::iterator p = m_programs.begin(); p != m_programs.end(); p++)
	{	WatchedProgram* prg = *p;									// the program
		assert(prg);
		int stat = prg->prep();											// prepare the program for running
		if (stat) watchdogfail("Unable to prep program for running",prg);	// Unlikely
	}	
	logprintf("Beginning run mode.\n");								// run
	//	Start all the monitor threads, which then start and run their programs.
	for (vector<WatchedProgram*>::iterator p = m_programs.begin(); p != m_programs.end(); p++)
	{	WatchedProgram* prg = *p;									// the program
		assert(prg);
		int stat = prg->start();											// launch the monitor thread for the program
		if (stat)
		{	watchdogfail("Unable to start monitor thread",prg);	}	// unlikely
	}
	//	***NEED TO START WATCHDOG TIMER THREADS HERE***
	return(0);																	// *** TEMP***
}