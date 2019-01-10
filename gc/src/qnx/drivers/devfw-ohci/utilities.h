//
//	utilities.h  --  misc. support functions
//
//
//	John Nagle
//	Animats
//	January, 2002
//
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
#ifndef UTILITIES_H
#define UTILITIES_H
#include "logprint.h"
//
//	Our priority names
//
const int MillisecondResponsePriority = 13;								// priority above most other things. Don't overuse
const int BasicRealTimePriority = 12;										// main real-time priority
const int NormalPriority = 10;													// normal priority
//
//	Endian support for 64-bit objects
//
uint64_t htonll(uint64_t in);
uint64_t ntohll(uint64_t in);
int findfirstbitmsb(uint32_t in);
//	Generate a 1 bit in indicated position, from either MSB or LSB end.
inline uint32_t bitmsb(uint8_t bitnum) {	return((uint32_t(1)<<31) >> bitnum);	}
inline uint32_t bitlsb(uint8_t bitnum) {	return((uint32_t(1)) << bitnum);	}
//
//	dbuf encapsulation
//
//
//	DbufPool  -- encapsulates LLA dbuf system
//
class DbufPool {
private:
	dbufPool_t*	m_pool;														// the pool
	int m_bufcount;																// number of buffers
	size_t m_bufsize;															// size of the buffer
public:
	dbufPool_t* getpool()
	{	return(m_pool); }
	DbufPool()																	// constructor
	: m_pool(0),m_bufcount(0),m_bufsize(0)
	{}
	~DbufPool();																	// destructor
	int create(size_t size, uint32_t bufcount, int flags);		// create the pool
	void destroy();																// destroy the pool, and check for buffer leaks.
	bool initialized()															// true if initialized
	{	return(m_pool != 0);	}
};	
//
//	Misc. support
//
const char* Err2Str (int error);
const char *TransactionCodeStr (uint32_t tcode);
int SetThreadPriority(int priority, int policy);


#endif // UTILITIES_H