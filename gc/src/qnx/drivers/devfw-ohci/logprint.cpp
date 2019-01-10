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
//
//	Copyright 2005 by John Nagle
//	999 Woodland Avenue
//	Menlo Park, CA  94025
//
//	This program is free software; you can redistribute it and/or modify
//	it under the terms of the GNU General Public License as published by
//	the Free Software Foundation; either version 2 of the License, or
//	(at your option) any later version.

//	This program is distributed in the hope that it will be useful,
//	but WITHOUT ANY WARRANTY; without even the implied warranty of
//	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//	GNU General Public License for more details.

//	You should have received a copy of the GNU General Public License
//	along with this program; if not, write to the Free Software
//	Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//
#include <stdio.h>
#include <stdarg.h>                           
#include "logprint.h"
//
//	Constants
//
const size_t logBufSize = 100;									// stores this many lines
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
    vsnprintf (msg,sizeof(msg), format, arglist);			// print into target string
    va_end (arglist);
    mainlog.log(msg);
}                