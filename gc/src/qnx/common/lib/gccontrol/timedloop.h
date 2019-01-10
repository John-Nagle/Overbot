/////////////////////////////////////////////////////////////////////////////
//
//    File: timedloop.h
//
//    Usage:
//        #include "timedloop.h"
//
//        // execute code() every second for ten seconds at priority 14
//        TimedLoop t("my timed loop", &code, 1.0, 14);
//        t.Start();
//        sleep(10);
//        t.Stop();
//        
//    Description:
//       A uniform interface for executing code periodically. A TimedLoop
//       object could be used, for example, for a control loop, or for 
//       sending information to a watchdog program at a regular interval.
//
//       When the Start() member function is called, a new thread is created
//       and the code is executed in that thread.
//
//		 Use the QNX command 'pidin' in a terminal window to view information
//       about created threads, including their state and priority.
//
//       See example code in check_timedloop.cc.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        October, 2003
//
//	Modified by J. Nagle to use a derived-class approach
//
/////////////////////////////////////////////////////////////////////////////

#ifndef TIMEDLOOP_H
#define TIMEDLOOP_H

#include <stdio.h>
#include <pthread.h>
#include <sched.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include "mutexlock.h"

const long TIMEDLOOP_SEC2NSEC  = 1000000000L;	// convert from secs to nanosecs
const long TIMEDLOOP_MSEC2NSEC = 1000000L;		// convert from msecs to nanosecs
//
//	TimedLoop  --  abstract class for timed loops
//
//	Subclass with desired functionality for your timed loop.
//
class TimedLoop {
public:
    // default constructor, period in sec
    TimedLoop(double iPeriod,
              int32_t iPriority);
    virtual ~TimedLoop();											// destructor
    int  Start();
    void Pause();
    void Resume();
    void Stop();
    void PeriodSet(double iPeriod);
    double PeriodGet();
    void PrioritySet(int32_t iPriority);
    void PriorityGet(int32_t *oPriority);
    bool Looping();
    bool Paused();
    void StatsExecGet(_uint64 *timeMin, _uint64 *timeMax);
    void StatsOverrunGet(float *percent, _uint64 *numLoops);
    _uint64 OverrunCountGet() const { return(countOverrun); }
protected:
	virtual void code() = 0;										// where the work gets done
private:
    timespec period;
    int32_t priority;
    void *loop();
    // need static function loopStart() for pthread_create
    // loopStart calls the instance-specific function loop()
    static void* loopStart(void* arg)
	    { return(reinterpret_cast<TimedLoop*>(arg)->loop()); }
    pthread_t thread;
    pthread_attr_t thread_attr;
    bool looping;			// to start and stop looping
    ost::Mutex lock;								// locked while thread is executing (not waiting)
	ost::Semaphore loopingSem;	// to pause and resume looping
	bool pause;				// to indicate whether to pause
    _uint64 timeExecMin;
    _uint64 timeExecMax;
    _uint64 countOverrun;
    _uint64 countTotal;
};

#endif // TIMEDLOOP_H