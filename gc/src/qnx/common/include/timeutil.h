//
//	timeutil.h  -- time related utilities for QNX
//
//	John Nagle
//	Team Overbot
//	March, 2005
//
#ifndef TIMEUTIL_H
#define TIMEUTIL_H
#include <inttypes.h>
#include <time.h>
#include <pthread.h>
//
//	gettimenowns  -- get time now, in nanoseconds
//
inline uint64_t gettimenowns()
{	struct timespec tm;											// don't wait past this time
	clock_gettime(CLOCK_REALTIME, &tm);				// get time now
	uint64_t timens(timespec2nsec(&tm));				// convert now to nanoseconds
	return(timens);													// time in nanoseconds since epoch
}
//
//	gettimenow  -- get time now, in seconds.
//
inline double gettimenow()
{		return(gettimenowns()*0.000000001);			// time in seconds since epoch
}
//
//	Priority support
//
//
//	getthreadpriority  -- get priority of current thread
//
inline int getthreadpriority()
{
	struct sched_param param;
	int policy;
	int stat = pthread_getschedparam(pthread_self(), &policy, &param);	// get scheduling info
	if (stat < 0) return(stat);														// fails
	return(param.sched_priority);												// return priority
}
//
//	setthreadpriority  -- set priority of current thread
//
inline int setthreadpriority(int pri)
{
	struct sched_param param;													// thread scheduling parameters
	int policy;																				// thread scheduling policy
	int stat = pthread_getschedparam(pthread_self(), &policy, &param);	// get scheduling info
	if (stat < 0) return(stat);														// fails
	param.sched_priority = pri;													// set priority
	policy =  (pri > 12) ? SCHED_FIFO : SCHED_RR;						// greater than 12 is hard real time
	stat = pthread_setschedparam(pthread_self(), policy, &param);	// set scheduling info
	if (stat < 0) return(stat);														// fails
	return(param.sched_priority);												// return priority
}
#endif // TIMEUTIL_H