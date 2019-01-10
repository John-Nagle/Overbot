//
//	logprint.cpp  -- print to log
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
#include <stdio.h>
#include <stdarg.h>                           
#include "logprint.h"
//
//	Constants
//
const size_t logBufSize = 200;									// stores this many lines
const size_t logLineSize = 132;								// traditional print length
const int logPriority = 9;											// background priority
const int logFd = 1;													// log to standard output
static Logprint<logBufSize,logLineSize> mainlog(logFd,logPriority);		// the main log, goes to standard output
//
//	logprintf -- prints to standard output, via the queuing mechanism
//
void logprintf(const char* format, ...)
            ////    __attribute__ ((format (printf, 1, 2)))
{	char msg[logLineSize];
    va_list arglist;														// variable number of args (but checked at compile time)
    va_start (arglist, format);
    ::vsnprintf (msg,sizeof(msg), format, arglist);			// print into target string
    va_end (arglist);
    mainlog.log(msg);
}                