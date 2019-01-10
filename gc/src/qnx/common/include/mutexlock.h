//
//	mutexlock.h --  mutex and semaphore wrapper.
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
#ifndef MUTEXLOCK_H
#define MUTEXLOCK_H
#include <stdint.h>
#include <pthread.h>
#include <semaphore.h>
#include <time.h>
#include <atomic.h>
#include "errnoexception.h"
namespace ost {
using namespace std;
//
//	class Mutex -- classic mutex
//
//	This just encapsulates Posix mutexes.
//
//	These mutexes are recursive; you can lock the same
//	object twice from the same thread.  But these mutexes
//	do not have P and V type counters; see "Semaphore"
//	below for that.
//	Based on interface defined by David Sugar <dyfet@ostel.com>
//
//	constants
//
const pthread_mutex_t initialmutexstate = PTHREAD_RMUTEX_INITIALIZER;	// initial state of mutex
//
class Mutex
{
private:
	pthread_mutex_t	m_mutex;								// Posix mutex object
private:
	void error_check(int stat)									// report Posix error if any
	{	if (stat == EOK) return;									// if OK, done
		throw(errno_error(stat));								// otherwise fails and throws input error number
	}
	Mutex& operator=(Mutex& m) { throw(errno_error(EINVAL)); }		// private/unreachable - unassignable
	Mutex(Mutex& m) { throw(errno_error(EINVAL)); }							// private/unreachable - uncopyable
public:
	Mutex()																// constructor (makes recursive mutex)
	{	pthread_mutexattr_t attr;								// construct attributes of mutex
		error_check(pthread_mutexattr_init(&attr));				// initialize to default attributes
		error_check(pthread_mutexattr_setrecursive(&attr, PTHREAD_RECURSIVE_ENABLE));			// make it a recursive mutex
		error_check(pthread_mutex_init(&m_mutex,&attr));	// initialze the mutex
		error_check(pthread_mutexattr_destroy(&attr));	// release the attribute object
	}
	virtual ~Mutex()												// destructor (throws if any threads waiting)
	{	error_check(pthread_mutex_destroy(&m_mutex)); }
	void enterMutex(void)										// locks the mutex for the current thread
	{	error_check(pthread_mutex_lock(&m_mutex));	}

	bool tryEnterMutex(void)									// like EnterMutex, but returns false if already locked.
	{	int stat = pthread_mutex_trylock(&m_mutex);	// try to lock
		if (stat == EOK) return(true);						// got lock
		if (stat == EBUSY) return(false);					// busy
		throw(errno_error(stat));								// error, throw
	}

	void leaveMutex(void)										// obvious unlock function
	{	error_check(pthread_mutex_unlock(&m_mutex));	}		// unlock
};
//
//	class MutexLock  -- scope-based lock for mutex
//
//	Locks when constructed, unlocks when destructed.
//	Exception-safe.
//
//	This should be used whenever possible, in preference
//	to locking and unlocking mutexes directly.
//
//	Usage:
//		{	MutexLock lock(mutex);
//			... // locked region
//		}		// unlocks at scope exit.
//
class MutexLock
{
private:
	Mutex& m_mutex;													// reference to associated mutex
public:
	//	Constructor - locks
	MutexLock( Mutex& mutex ) : m_mutex( mutex ) 
	{	m_mutex.enterMutex(); }
	// 	Destructor - unlocks
	~MutexLock()														// non-virtual destructor - do not subclass
	{	m_mutex.leaveMutex(); }
};
//
//	class Semaphore -- Djkystra-type P and V primitives.
//
//	These allow usage counts > 1, unlike mutexes.
//
//	The initial count can be specified, and is zero by default.
//	Each "wait" decrements the count, blocking until the count is > 0.
//	Each "post" increments the count. "Post" never blocks.
//
//	Semaphores created with "shared=true" can be used in
//	shared memory.
//
//	"wait" can be given a timeout value, in seconds, as a floating point number.
//
class Semaphore
{
private:
	sem_t	m_semaphore;										// the semaphore
private:
	virtual void error_check(int stat)						// report Posix error if any
	{	if (stat == EOK) return;									// if OK, done
		throw(errno_error(stat));								// otherwise fails and throws errno
	}
private:
	Semaphore(Semaphore& dummy) {}				// make class uncopyable
	Semaphore& operator=(Semaphore& dummy) { return(*this);} // make class uncopyable
public:
	
	//	Constructor.
	Semaphore(size_t resource = 0,  bool shared=false)
	{	error_check(sem_init(&m_semaphore,shared,resource));	}
	
	//	Destructor
	virtual ~Semaphore()
	{	sem_destroy(&m_semaphore);	}

	//	wait -- wait for counter > 0
	void wait()
	{	error_check(sem_wait(&m_semaphore));	}

	 //	trywait -- lock if counter > 0, return false otherwise.
	bool trywait()
	{	int stat = sem_trywait(&m_semaphore);			// try to lock
		return(stat == 0);												// success if stat==0
	}
	
	//	timedwaitns  -- wait no longer than specified number of nanoseconds
	bool timedwaitns(uint64_t ns)
	{	struct timespec tm;											// don't wait past this time
		clock_gettime(CLOCK_REALTIME, &tm);				// get time now
		uint64_t timens(timespec2nsec(&tm));				// convert now to nanoseconds
		timens += ns;													// compute end of wait
		nsec2timespec(&tm,timens);								// convert back to time structure
		int stat = sem_timedwait(&m_semaphore,&tm);// wait, no longer than timelimit
		if (stat == 0) return(true);									// normal case - locked
		//	We have to check errno to find out whether this is simply a timout
		//	or a hard error. Under QNX, errno is thread-local, so this is safe.
		if (errno == ETIMEDOUT) return(false);				// timeout case - not locked
		throw(errno_error(errno));								// error case - throw
	}
	
	//	wait -- with timeout argument (seconds)
	bool wait(double secs)
	{	return(timedwaitns(uint64_t(secs*1.0e9)));	}	// wait no more than secs

	//	post -- increment counter, unblocking anyone waiting.
	void post()
	{	error_check(sem_post(&m_semaphore));	}
};
//
//	atomic_incmod_value  -- add to value, modular, as atomic operation
//
//	This is subtle. 
//
inline unsigned atomic_incmod_value(volatile unsigned* loc,
                unsigned mod)
{	unsigned oldval = atomic_add_value(loc, 1);	// add 1, return value, atomic operation
	if (oldval >= mod)				// if overflow
	{	if (oldval == mod)			// if exactly at overflow
		{	atomic_sub(loc,mod); }// must reduce by one cycle
		oldval %= mod;				// must reduce result
	}
	return(oldval);
}
//
//	Bounded buffer package
//
//	Reminiscent of John Walker's bounded buffer package in FANG, circa 1972.
//	
//	This template class implements a fixed-size bounded buffer.
//	The buffer stores N objects of type T, for implementing
//	producer-consumer relationships. 
//	"put" inserts, blocking if the buffer is full.
//	"get" removes, blocking if the buffer is empty.
//
//	Bounded buffers of size 1 work, and are quite useful for
//	simple event synchronization.
//	A size of 0, however, is not useful.
//
template<class T, unsigned int N>  class BoundedBuffer
{
private:
	Semaphore m_lock_head;				// lock for output end
	Semaphore m_lock_tail;				// lock for input end
	size_t	m_getpos;							// next avail for read
	size_t	m_putpos;							// next avail for write
	T m_data[N];									// the data

private:
	size_t next(size_t& pos)				// advance to next item, returning current
	{	if (N > 1)									// for N=1, no arithmetic is required.
		{	return(atomic_incmod_value(&pos,N));	// next pos
		} else {									// size 1, no computation required
			return(0);
		}
	}
public:
	BoundedBuffer<T,N>()	:				// constructor - create an empty object
		m_lock_head(0),						// no slots are full
		m_lock_tail(N),							// N slots available 
		m_getpos(0),							// start at 0
		m_putpos(0) 							// start at 0
		{}
	
	void get(T& item)							// get one from queue - blocks
	{	m_lock_head.wait();					// wait for an item
		item = m_data[next(m_getpos)];		// item available, get it
		m_lock_tail.post();						// allow another put
	}
	
	void put(T item)							// put item - blocks
	{	m_lock_tail.wait();						// wait for space
		m_data[next(m_putpos)] = item;		// put item in slot
		m_lock_head.post();					// allow another get
	}
	
	bool tryget(T& item)						// get one from queue - non-blocking
	{	bool locked = m_lock_head.trywait();	// wait for an item
		if (!locked) return(false);			// didn't get one
		item = m_data[next(m_getpos)];		// item available, get it
		m_lock_tail.post();						// allow another put
		return(true);								// got one
	}
	
	bool tryput(T item)						// put item - non-blocking
	{	bool locked = m_lock_tail.trywait();	// wait for space
		if (!locked) return(false);			// didn't get one
		m_data[next(m_putpos)] = item;	// put item in slot
		m_lock_head.post();					// allow another get
		return(true);								// put one
	}
	
	bool get(T& item, double secs)		// get one from queue - with timeout on block
	{	bool locked = m_lock_head.wait(secs);	// wait for an item
		if (!locked) return(false);			// didn't get one
		item = m_data[next(m_getpos)];	// item available, get it
		m_lock_tail.post();						// allow another put
		return(true);								// got one
	}
	
	bool put(T item, double secs)			// put item - with timeout on block
	{	bool locked = m_lock_tail.wait(secs);	// wait for space
		if (!locked) return(false);			// didn't get one
		m_data[next(m_putpos)] = item;	// put item in slot
		m_lock.head.post();					// allow another get
		return(true);								// put one
	}
	
	bool peek(T& item)						// look at top of queue, but do not remove item
	{	bool locked = m_lock_head.trywait();	// try to lock
		if (!locked) return(false);			// nothing to return
		item = m_data[m_getpos];		// item available, get it
		m_lock.head.post();					// unlock
		return(true);								// peeked successfully
	}
	
	unsigned int size() const 				// size of buffer
	{	return(N);	}								// return fixed size					
};
};																				// end namespace
#endif // MUTEXLOCK_H
