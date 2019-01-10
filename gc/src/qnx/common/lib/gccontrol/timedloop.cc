////////////////////////////////////////////////////////////////////////////
//
//    File: timedloop.cc
//
//    Usage:
//        #include "timedloop.h"
//
//    Description:
//        See timedloop.h.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        October, 2003
//
/////////////////////////////////////////////////////////////////////////////

#include "timedloop.h"
using namespace ost;

//
//    TimedLoop::TimedLoop - default class constructor
//
TimedLoop::TimedLoop(double iPeriod,
                     int iPriority)
: loopingSem(0)
{	
	// store the desired period
	PeriodSet(iPeriod);
	
    // store the desired priority
    priority = iPriority;
    
    // initialize thread attributes
	if ( pthread_attr_init(&thread_attr) != EOK ) {
		fprintf(stderr, 
		        "TimedLoop::TimedLoop - error calling pthread_attr_init()\n");
	}
	
	// initialize looping control variables
	looping = false;				// not looping until call Start()
	pause = false;					// don't pause until call Pause()
	
	// initialize execution statistics
	timeExecMin = UINT64_MAX;
	timeExecMax = 0;
	
	// initialize overrun statistics
	countOverrun = 0;
	countTotal   = 0;
}
//
//	Destructor
//
//	Stops looping, and waits for thread to exit.
//
TimedLoop::~TimedLoop()
{
	Stop();																// shut down thread
}
	
//
//    TimedLoop::Start - start executing code periodically
//
//    Launches a new thread.
//
//    Returns value returned by pthread_create
//
int TimedLoop::Start()
{
	ost::MutexLock lok(lock);									// protect against reentrancy
	if (looping) return(EBUSY);								// already looping, don't start again 
	sched_param param;
	
	// want to keep looping
	looping = true;
	
	// reset execution statistics
	timeExecMin = UINT64_MAX;
	timeExecMax = 0;

	// reset overrun statistics
	countOverrun = 0;
	countTotal   = 0;
	
	// set thread attributes
	if ( pthread_attr_setinheritsched(&thread_attr, PTHREAD_EXPLICIT_SCHED) != 
	     EOK ) {
		fprintf(stderr, 
		"TimedLoop::Start - error calling pthread_attr_setinheritsched()\n");
	}
	if ( pthread_attr_getschedparam(&thread_attr, &param) ) {
		fprintf(stderr,
		"TimedLoop::Start - error calling pthread_attr_getschedparam()\n");
	}
	param.sched_priority = priority; // change only the priority
	if ( pthread_attr_setschedparam(&thread_attr, &param) != EOK ) {
		fprintf(stderr,
		"TimedLoop::Start - error calling pthread_attr_setschedparam()\n");
	}
	
	// launch thread to execute code periodically
	return pthread_create(&thread, &thread_attr, loopStart, this);
}

//
//    TimedLoop::Pause - pauses (gently) executing code periodically
//
//    This will result in a gentle pause---the thread will be paused,
//    waiting for a semaphore at the start of a sample period.
//
//    Returns nothing
//
void TimedLoop::Pause()
{
	pause = true;
}

//
//    TimedLoop::Resume - resumes (gently) executing code periodically
//
//    The thread will be resumed where it was last paused.
//
//    Returns nothing
//
void TimedLoop::Resume()
{
	pause = false;
	
	if ( looping ) {
		loopingSem.post();
	}
}

//
//    TimedLoop::Stop - stops (gently) executing code periodically
//
//    This will result in a reasonably graceful stop. The thread
//	will be cancelled, but not during its actual work period.
//
//	Note that this will not stop a runaway thread. 
//
//	Does not return until the thread is stopped.
//
//    Returns nothing
//
void TimedLoop::Stop()
{
	// indicate that controller of thread wants it to stop looping
	ost::MutexLock lok(lock);									// protect against reentrancy
	if (!looping) return;											// not doing anything, skip
	if (thread == pthread_self())							// if cancelling self
	{	looping = false; return;	}								// ask for an exit
	//	At this point, we're sure that the thread isn't executing the user code.
	//	So we can cancel it out of its wait. 
	int stat = pthread_cancel(thread);					// cancel the thread, which will be waiting
	if (stat != EOK)
	{	perror("TimedLoop::Stop: thread did not cancel properly");	}
	stat = pthread_join(thread,0);							// wait for thread to exit
	if (stat != EOK)
	{	perror("TimedLoop::Stop: thread did not exit properly");	}
	looping = false;												// ask for a stop
	pause = false;													// not paused
}

//
//    TimedLoop::PeriodSet - set the sample period (in sec)
//
//    Returns nothing
//
void TimedLoop::PeriodSet(double iPeriod)
{
	// period is in seconds; convert to l_period timespec
	// could have problem where tv_sec gets changes and used before tv_nsec?
	period.tv_sec  = int(iPeriod);
	period.tv_nsec = long((iPeriod - period.tv_sec) * 
	                        (double)TIMEDLOOP_SEC2NSEC);
}

//
//    TimedLoop::PeriodGet - get the sample period (in sec)
//
//    Returns the period in seconds
//
double TimedLoop::PeriodGet()
{
	// interval is a timespec; convert to period in seconds
	return double(period.tv_sec +
	              (period.tv_nsec / (double)TIMEDLOOP_SEC2NSEC));
}

//
//    TimedLoop::PrioritySet - set the priority of the timed loop thread
//
//    Returns nothing
//
void TimedLoop::PrioritySet(int32_t iPriority)
{
	ost::MutexLock lok(lock);									// protect against reentrancy
	// update stored priority
	priority = iPriority;
	
	// update priority of thread if it's running
	if (looping) {
		int policy;
		sched_param param;
	
		if ( pthread_getschedparam(thread, &policy, &param) != EOK ) {
			fprintf(stderr,
			"TimedLoop::PrioritySet - error calling pthread_getschedparam()\n");
		}
		param.sched_priority = priority;
		if ( pthread_setschedparam(thread, policy, &param) != EOK ) {
			fprintf(stderr,
			"TimedLoop::PrioritySet - error calling pthread_setschedparam()\n");
		}
	}
}

//
//    TimedLoop::PriorityGet - get the priority of the timed loop thread
//
//    Returns nothing
//
void TimedLoop::PriorityGet(int32_t *oPriority)
{
	ost::MutexLock lok(lock);									// protect against reentrancy
	if (looping) {
		// get priority from running thread (more accurate)
		int policy;
		sched_param param;
	
    	if ( pthread_getschedparam(thread, &policy, &param) != EOK ) {
    		fprintf(stderr,
    		"TimedLoop::PriorityGet - error calling pthread_getschedparam()\n");
    	}
    	*oPriority = param.sched_priority;
	} else {
		// get stored priority
		*oPriority = priority;
	}
}

//
//    TimedLoop::Looping - indicates if running loop
//
//    Returns value of looping data member
//
bool TimedLoop::Looping()
{
	return looping;
}

//
//    TimedLoop::Paused - indicates if loop paused
//
//    Returns value of pause data member
//
bool TimedLoop::Paused()
{
	return pause;
}

//
//    TimedLoop::StatsExecGet - get execution time information
//
//    Get access to the minimum and maximum loop code execution times..
//
//    Returns nothing
//
void TimedLoop::StatsExecGet(_uint64 *timeMin, _uint64 *timeMax)
{
	*timeMin = timeExecMin;
	*timeMax = timeExecMax;
}

//
//    TimedLoop::StatsOverrunGet - get fraction of runs that are overruns
//
//    Calulates the percent of runs that are overruns, and the total
//    number of loops performed.
//
//    Returns nothing
//
void TimedLoop::StatsOverrunGet(float *percent, _uint64 *numLoops)
{
	*percent = float(double(countOverrun)/double(countTotal));
	*numLoops = countTotal;
}

//
//    TimedLoop::loop - this routine gets executed on the interval
//
//    Wrapper code around the user code.
//
//    Returns nothing.
//
//	Exited when owner of object calls Stop,, which cancels this thread.
//	Protected against cancellation while doing actual work.
//
void *TimedLoop::loop()
{
	timespec timeNext, timeDone;
	_uint64 timeExec;
	
	// get start time
	// could check that closck_gettime returns 0, but don't want
	// to print during loop
	clock_gettime(CLOCK_REALTIME, &timeNext);

	do {
	    // check if periodic excution should be paused (gently)
	    // after full code execution
	    // (at start of loop so can call Pause() before Start() and
	    //  have loop paused)
	    if ( pause ) {
	    	// wait for looping to be resumed
	    	loopingSem.wait();
	    	
	    	// start looping at time now
	    	clock_gettime(CLOCK_REALTIME, &timeNext);
	    }

	    // execute user code, compute execution statistics, ignore first loop
	    {	MutexLock lok(lock);											// lock against cancellation
		   	code();																// call the function
		   	//	If the code function calls Stop, looping will be cleared, but the thread does not stop.
		   	//	We must stop it here. Note that the return both releases the lock and causes
		   	//	the thread to exit.
		   	if (!looping)														// if thread stopped itself
		   	{	return(0);	}													// thread must exit 
	    }
	    if (countTotal > 0) {
	    	// could check that clock_gettime returns 0, but don't want 
	    	// to print during loop
	    	clock_gettime(CLOCK_REALTIME, &timeDone);
	    	timeExec    = timespec2nsec(&timeDone) - timespec2nsec(&timeNext);

	   		timeExecMin = __min(timeExec, timeExecMin);
	    	timeExecMax = __max(timeExec, timeExecMax);
	    }
	    
		// when to start loop again
		nsec2timespec(&timeNext,
		              timespec2nsec(&timeNext) + timespec2nsec(&period));

		// check if next start time already passed
		if ( timespec2nsec(&timeDone) > timespec2nsec(&timeNext) ) {
			// overrun, so increment counter and don't sleep
			countOverrun++;
		} else {
			// didn't overrun, so sleep until end of sample period
			// could check that clock_nanosleep returns EOK, but don't want
			// to print during loop
		    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &timeNext, NULL);
		}

		// increment total counter
		countTotal++;
	} while ( 1 );
}
