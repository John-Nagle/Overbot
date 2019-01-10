/////////////////////////////////////////////////////////////////////////////
//
//    File: actserver_.h
//
//    Usage:
//       #include "actserver_.h"
//
//    Description:
//       See actserver.h.
//
//       This is the internal header file (actserver.h is the external one).
//       Include this file in the internal header file for your server.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        December, 2003
//
/////////////////////////////////////////////////////////////////////////////

#ifndef ACTSERVER__H
#define ACTSERVER__H

#include "ask.h"
#include "menu.h"
#include "mvp.h"
#include "mvpmenu.h"
#include "timedloop.h"
#include "timedloopmenu.h"
#include "mvpserver.h"
#include "mutexlock.h"

#include "actserver.h"

#define ACTSERVER_NAME_LEN		(82)
#define ACTSERVER_PROMPT_LEN	(82)

using namespace ost;	// in mutexlock.h

class ActServer_ {
public:
	ActServer_();
	virtual ~ActServer_();			// needs virtual destructor

	void ServerName(char *name);	// assign server name
	char *ServerName();				// get server name
	
	virtual void MessageThread();	// main thread	
	
	void MenuThread();				// auxillary threads
	
	void InitializeThread();
	
	void TargetThread();
	
	TimedLoop *Tc() { return tc; }
	void ControlLoopStart();
	void ControlLoopPause();
	void ControlLoopResume();
	void ControlLoopStop();
	void ControlLoopCount(_uint64 count) { controlLoopCount = count; }
	_uint64 ControlLoopCount() { return controlLoopCount; }
	void ControlLoopPeriod(double period) { controlLoopPeriod = period; }
	double ControlLoopPeriod() { return controlLoopPeriod; }
	void ControlLoopTimeout(double timeout) { controlLoopTimeout = timeout; }
	double ControlLoopTimeout() { return controlLoopTimeout; }

	TimedLoop *Td() { return td; }
	void DataLoopStart();
	void DataLoopPause();
	void DataLoopResume();
	void DataLoopStop();

	void Simulation(bool iSimulation);
	bool Simulation();
	void Initialized(bool iInitialized);
	bool Initialized();
	void Verbose(bool iVerbose);
	bool Verbose();
private:
	char serverName[ACTSERVER_NAME_LEN];	// name of server (e.g., "Gear")
	
	// server message handling (main thread)
	virtual void messageHandle();

	virtual void *menuThread();
    // need static function loopStart() for pthread_create
    // loopStart calls the instance-specific function loop()
    static void* menuThreadStart(void* arg)
	    { return(reinterpret_cast<ActServer_*>(arg)->menuThread()); }

	virtual void *initThread();
    // need static function loopStart() for pthread_create
    // loopStart calls the instance-specific function loop()
    static void* initThreadStart(void* arg)
	    { return(reinterpret_cast<ActServer_*>(arg)->initThread()); }

	virtual void *targThread();
    // need static function loopStart() for pthread_create
    // loopStart calls the instance-specific function loop()
    static void* targThreadStart(void* arg)
	    { return(reinterpret_cast<ActServer_*>(arg)->targThread()); }
	    
	TimedLoop *td;					// timed loop for data
	virtual void *dataLoop();
    // need static function codeStart() for TimedLoop
    // codeStart calls the instance-specific function code()
    // TimedLoop will pass "this" as "arg"
    static void* dataLoopStart(void* arg)
	    { return(reinterpret_cast<ActServer_*>(arg)->dataLoop()); }
	float dataLoopPeriod;

	TimedLoop *tc;					// timed loop for control
	virtual void *controlLoop();
    // need static function codeStart() for TimedLoop
    // codeStart calls the instance-specific function code()
    // TimedLoop will pass "this" as "arg"
    static void* controlLoopStart(void* arg)
	    { return(reinterpret_cast<ActServer_*>(arg)->controlLoop()); }
	_uint64 controlLoopCount;
	float controlLoopPeriod;
	double controlLoopTimeout;

	// server state
	bool simulation;
	bool initialized;
	bool verbose;
};

#endif // ACTSERVER__H
