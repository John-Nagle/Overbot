//
//	logprint.h  -- print to log
//
//	John Nagle
//	Animats
//	March, 2003
//
//	Prints to standard output, or log, but does not block.
//	Can be called from real-time threads without impacting performance.
//	Some output may be lost, but a message will appear indicating
//	that messages have been lost.
//	Print operations are atomic; each one should be one line, and 
//	lines from different threads will not be interleaved.
//
//
//	Compatible with usual printf.
//
//	QNX only.
//	GCC only - uses GCC printf format checking.
#ifndef LOGPRINT_H
#define LOGPRINT_H
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include "mutexlock.h"
//
extern void logprintf(const char* format, ...)
                __attribute__ ((format (printf, 1, 2)));			// GCC extension - requires GCC format checking
                
     
inline void logperror(const char* msg)
{	logprintf("%s: %s\n", msg, strerror(errno)); }
                
//
//	Class Logprint  --  an object for managing a log of printable lines
//
//	This never blocks the thread that's doing the printing. If the buffer is full,
//	print lines are lost, but appear in output as "...".  So it's safe to call this
//	from real-time tasks.
template<unsigned int BUFLINES, unsigned int LINELENGTH>  class Logprint {
private:
	struct Logline {														// one line in the log
		char m_line[LINELENGTH];									// the text to print
		bool m_lost;														// true if lost some data
	public:
		Logline(const char* msg, bool lost)
		: m_lost(lost) { strncpy(m_line,msg,sizeof(m_line)); }	// constructor
		Logline() { m_line[0] = '\0';	}							// empty constructor
		void print(int fd)												// print to indicated fd
		{	if(m_lost) write(fd,"...\n",4);								// if lost previous line(s), preface with "..."
			write(fd,m_line,strlen(m_line));						// write to output, ignoring status - should use strnlen, but QNX lacks it.
		}
	};
	ost::BoundedBuffer<Logline,BUFLINES>m_queue;// queue of lines to print
	int m_fd;																// output file descriptor
	pthread_t m_printthread;										// the thread that prints
	volatile bool m_lost;												// true if last line lost
	volatile bool m_exiting;											// true after destructor called
	int m_priority;														// priority of print thread
private:
	void printthread()													// the printing thread
	{	struct sched_param param = {m_priority};		// set priority
		errno_error::checkok(pthread_setschedparam(pthread_self(),SCHED_RR,&param));		// set scheduling parameters 
		for (;;)
		{	Logline item;
			if (m_exiting)													// if exiting 
			{	bool success = m_queue.tryget(item);		// try to get, no block
				if (!success) break;									// queue empty, done
			} else { m_queue.get(item);	}						// get with block
			item.print(m_fd);											// print it
		}
	}
	static void* startprintthread(void* param)			// object of thread fork
	{	Logprint* obj = (Logprint*)(param);					// cast into object
		obj->printthread();											// run
		return(param);													// unused
	}
public:
	Logprint(int outfd = 0, int priority=10): m_fd(outfd),m_lost(false),m_exiting(false),m_priority(priority)		// constructor
	{	errno_error::checkok(pthread_create(&m_printthread,NULL,&startprintthread,this)); }
	~Logprint() { m_exiting = true;	  log(""); pthread_join(m_printthread,0); }				// destructor
	void printf(const char* format, ...)							// use like printf
                __attribute__ ((format (printf, 2, 3)));			// GCC extension - requires GCC format checking
    void log(const char line[LINELENGTH])					// add a preformatted line
	{	m_lost = !m_queue.tryput(Logline(line,m_lost));}	// log, tallying failures
};
#endif // LOGPRINT_H

                
