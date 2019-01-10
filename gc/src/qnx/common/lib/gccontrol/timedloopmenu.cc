/////////////////////////////////////////////////////////////////////////////
//
//    File: timedloopmenu.cc
//
//    Usage:
//        see timedloopmenu.h
//
//    Description:
//        See timedloopmenu.h
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        October, 2003
//
/////////////////////////////////////////////////////////////////////////////

#include "timedloopmenu.h"

//
//    TimedLoopMenu::Display - display options of TimedLoop menu
//
//    Returns nothing
//
void TimedLoopMenu::Display()
{
	printf("  s - start timed loop (launches thread)\n");
	printf("  p - pause timed loop (pauses looping)\n");
	printf("  r - resume timed loop (resumes looping)\n");
	printf("  x - stop timed loop (exits thread)\n");
	printf("  f - change sample period of timed loop\n");
	printf("  i - change priority of timed loop thread\n");
	printf("  e - display code execution statistics\n");
	printf("  o - display overrun statistics\n");
	printf("  c - display the clock resolution\n");
}

//
//    TimedLoopMenu::Process - process options for TimedLoop menu
//
//    Returns true if menu option processed, false if not
//
bool TimedLoopMenu::Process(char c)
{
    switch (c) {
	    case 's':
		    start();
		    break;
	    case 'p':
		    pause();
		    break;
	    case 'r':
		    resume();
		    break;
	    case 'x':
		    stop();
		    break;
	    case 'f':
		    samplePeriod();
		    break;
	    case 'i':
		    priority();
		    break;
	    case 'e':
		    execution();
		    break;
	    case 'o':
		    overrun();
		    break;
	    case 'c':
		    resolution();
		    break;
		default:
		    return false;
			break;
	}

	return true;
}

void TimedLoopMenu::start()
{
	if ( t->Looping() ) {
		printf("%s already started.\n", NameGet());
	} else {
   		// start the timed loop
    	if ( Ask::YesNo("Start Timed Loop", 'y') ) {
	    	if ( t->Start() != EOK ) {
		    	fprintf(stderr,
		    "TimedLoopMenu::start - not able to create timed loop thread.\n");
			}
		}
	}
	
	Ask::Pause();
}

void TimedLoopMenu::pause()
{
	if ( t->Paused() ) {
		printf("%s already paused.\n", NameGet());
	} else {
		if ( !t->Looping() ) {
			printf("%s stopped.  Will pause when started.\n", NameGet());
		}
   		// pause the timed loop
    	if ( Ask::YesNo("Pause Timed Loop", 'y') ) {
	    	t->Pause();
    	}
	}
	
	Ask::Pause();
}

void TimedLoopMenu::resume()
{
	if ( !t->Paused() ) {
		printf("%s not paused.\n", NameGet());
	} else {
		if ( !t->Looping() ) {
			printf("%s stopped. Will resume when started.\n", NameGet());
		}

   		// resume the timed loop
    	if ( Ask::YesNo("Resume Timed Loop", 'y') ) {
	    	t->Resume();
		}
	}
	
	Ask::Pause();
}

void TimedLoopMenu::stop()
{
	if ( !t->Looping() ) {
		printf("%s already stopped.\n", NameGet());
	} else {
   		// stop the timed loop
    	if ( Ask::YesNo("Stop Timed Loop", 'y') ) {
	    	t->Stop();
		}
	}
	
	Ask::Pause();
}

void TimedLoopMenu::samplePeriod()
{
	double period;
	
	period = t->PeriodGet();
	period = Ask::Double("Sample period (sec)", period);
	t->PeriodSet(period);

    Ask::Pause();
}

void TimedLoopMenu::priority()
{
	int32_t priority = 0;
	
	t->PriorityGet(&priority);
	
	// NEED TO CHANGE 999 TO REASONSABLE VALUE
	priority = (int32_t) Ask::Int("Priority", priority, 0, 999);
	t->PrioritySet(priority);
           
    Ask::Pause();
}

void TimedLoopMenu::execution()
{
	_uint64 timeMin, timeMax;
	
	t->StatsExecGet(&timeMin, &timeMax);
	printf("Minimum execution time = %f ms\n", 
	       float(timeMin/double(TIMEDLOOP_MSEC2NSEC)));
	printf("Maximum execution time = %f ms\n", 
	       float(timeMax/double(TIMEDLOOP_MSEC2NSEC)));
	
	Ask::Pause();
}

void TimedLoopMenu::overrun()
{
	float percent;
	_uint64 numLoops;
	
	t->StatsOverrunGet(&percent, &numLoops);
	printf("Overrun %f percent of the time.\n", percent);
	printf("Number of loops = %.0f\n", double(numLoops));
	
	Ask::Pause();
}

// Could eventually allow the resolution to be changed with the ClockPeriod()
// kernel call.  Typical resolutions are 1ms or 10ms.  Can set down to 10usec.
void TimedLoopMenu::resolution()
{
	timespec res;
	
    // display clock resolution
    if ( clock_getres(CLOCK_REALTIME, &res) != 0 ) {
    	fprintf(stderr,
    	        "TimedLoopMenu::resolution - error calling clock_getres()\n");
    }
    printf("Clock resolution = %.3f ms\n",
           float(res.tv_nsec / (double) TIMEDLOOP_MSEC2NSEC));
           
    Ask::Pause();
}

