//
//	threadobjects.h --  wrapper for Posix threads
//
//	Developed for QNX, but may work on other Posix-compliant systems.
//
//	Maintains much of the interface of the GNU Common C++ library,
//	but is all-inline and removes all the GNU macro dreck.
//
//	License: LGPL
//
//	John Nagle
//	Animats
//	January 2003
//
#ifndef THREADOBJECT_H
#define THREADOBJECT_H
#include <stdint.h>
#include <pthread.h>
#include <semaphore.h>
#include <time.h>
#include "errnoexception.h"
#include "mutexlock.h"
namespace ost {
using namespace std;
//
//	Constants
//
const pthread_t k_invalid_thread = -1;						// an invalid thread number
//
//	class Pthread --  a Posix thread
//
//	Runs a thread in the specified object.
//	Derive classes from this to hold the data of a thread.
//	Implement "run" in the derived class to do the thread's work.
//	Return from "run" when done with the thread.
//
//
class Pthread
{
private:
	pthread_t	m_thread;												// the thread
public:
	Pthread()
	: m_thread(k_invalid_thread) {}
	virtual ~Pthread()													// will wait for thread to exit before destroying
	{	join();	}
	pthread_t& getthread() { return(m_thread); }		// return the thread 
	int create()															// create thread
	{	return(pthread_create(&m_thread,0,startthread,this));	}	// create the thread
	int join();																// wait for thread to exit
	int join(float timeout);											// wait for thread to exit, with time limit
	bool isrunning() const { return(m_thread != k_invalid_thread); }// true if thread running
protected:
	virtual void run() = 0;											// the thread. Must be defined in derived class
																				// static to start pthread
	static void* startthread(void* threaddata) { reinterpret_cast<Pthread*>(threaddata)->run(); return(0);}
};	
//
//	Implementation
//
//
//	join -- wait forever
//
inline int Pthread::join()
{	int stat = pthread_join(m_thread,0);						// join (wait for thread to finish)
	m_thread = k_invalid_thread;								// no more thread
	return(stat);
}
//
//	join -- with timeout in seconds
//
inline int Pthread::join(float timeout)							// QNX only
{	
	struct timespec tm;												// don't wait past this time
	clock_gettime(CLOCK_REALTIME, &tm);					// get time now
	uint64_t timens(timespec2nsec(&tm));					// convert now to nanoseconds
	timens += uint64_t(timeout * 1e9);						// compute end of wait
	nsec2timespec(&tm,timens);									// convert back to time structure
	int stat = pthread_timedjoin(m_thread,0,&tm);		// wait with time limit
	if (stat == 0)															// if normal exit
	{	m_thread = k_invalid_thread; return(0); 	}		// success
	return(stat);															// fail, return status														
}

};																				// end namespace
#endif // THREADOBJECT_H
