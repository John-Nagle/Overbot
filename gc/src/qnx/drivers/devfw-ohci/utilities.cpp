//
//	Misc. utility functions for devfw-ohci
//
//	John Nagle
//	Animats
//	March, 2003
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
#include <assert.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <errno.h>
#include <sched.h>
#include <pthread.h>

#include "mindreadyforcpp.h"
#include "utilities.h"
#include "llaError.h"
#include "logprint.h"

using namespace std;
//
//	SetThreadPriority  --  set priority of a thread
//
//	Normal time-sharing priority is 10. Larger numbers are higher.
//	Scheduling policy is SCHED_FIFO or SCHED_RR.
//
//	Returns Posix error code
//
int SetThreadPriority(int priority, int policy)
{	struct sched_param param;						//	scheduling parameters */
	param.sched_priority = priority;				// set priority, the only field */
	return(pthread_setschedparam(pthread_self(),policy,&param));		// set scheduling parameters 
}

//
//	TransactionCodeStr  -- returns transaction code string in ASCII
//
const char* TransactionCodeStr(uint32_t tcode)
{
	switch (tcode)
	{
		case IEEE1394_QUAD_WRREQ:
			return("QUAD_WRREQ");
		case IEEE1394_BLOCK_WRREQ:
			return("BLOCK_WRREQ");
		case IEEE1394_WRRESP:
			return("WRRESP");
		case IEEE1394_QUAD_RDREQ:
			return("QUAD_RDREQ");
		case IEEE1394_BLOCK_RDREQ:
			return("BLOCK_RDREQ");
		case IEEE1394_QUAD_RDRESP:
			return("QUAD_RDRESP");
		case IEEE1394_BLOCK_RDRESP:
			return("BLOCK_RDRESP");
		case IEEE1394_CYCLE_START_REQ:
			return("CYCLE_START_REQ");
		case IEEE1394_LOCK_REQ:
			return("LOCK_REQ");
		case IEEE1394_ISOBLOCK_REQ:
			return("ISOBLOCK_REQ");
		case IEEE1394_LOCK_RESP:
			return("LOCK_RESP");
		case IEEE1394_TCODE_INTERNAL:
			return("TCODE_INTERNAL");
		default:
			return("???");
	}
	return("???");
}







//	
//	Err2Str
//
//	Converts SedNet LLA error values to constant strings.
//
const char* Err2Str (int error)
{
	switch (error)
	{
		/* General Error code */
		case SDERR_HW_ACCESS_FAILED:   return "HW_ACCESS_FAILED";
		case SDERR_NOT_ISR_CALLABLE:   return "SDERR_NOT_ISR_CALLABLE";
		case SDERR_NOT_SUPPORTED_BY_HW: return "SDERR_NOT_SUPPORTED_BY_HW";
		case SDERR_NODE_ALONE_ON_BUS:  return "SDERR_NODE_ALONE_ON_BUS";
		case SDERR_SYS_UNAVAILABLE:    return "SDERR_SYS_UNAVAILABLE";
		case SDERR_INVALID_GEN_COUNT:  return "SDERR_INVALID_GEN_COUNT";
		case SDERR_INVALID_CHAN_LIST:  return "SDERR_INVALID_CHAN_LIST";
		case SDERR_NOT_BUS_ROOT:       return "SDERR_NOT_BUS_ROOT";
		case SDERR_INVALID_ADDR:       return "SDERR_INVALID_ADDR";
		case SDERR_UNSUPPORTED:        return "SDERR_UNSUPPORTED";
		case SDERR_INVALID_HANDLE:     return "SDERR_INVALID_HANDLE";
		case SDERR_INVALID_CMD:        return "SDERR_INVALID_CMD";
		case SDERR_INVALID_PACKET:     return "SDERR_INVALID_PACKET";
		case SDERR_TOO_MUCH_CBACK:     return "SDERR_TOO_MUCH_CBACK";
		case SDERR_INVALID_DESC:       return "SDERR_INVALID_DESC";
		case SDERR_COMM_DISABLED:      return "SDERR_COMM_DISABLED";
		case SDERR_INVALID_PARAM:      return "SDERR_INVALID_PARAM";
		case SDERR_BUFFER_EMPTY:       return "SDERR_BUFFER_EMPTY";
		case SDERR_CHAN_UNAVAILABLE:   return "SDERR_CHAN_UNAVAILABLE";
		case SDERR_MULTI_CHAN_UNAVAIL: return "SDERR_MULTI_CHAN_UNAVAIL";
		case SDERR_CTX_UNAVAILABLE:    return "SDERR_CTX_UNAVAILABLE";
		case SDERR_INVALID_ADP_INDEX:  return "SDERR_INVALID_ADP_INDEX";
		case SDERR_DEV_UNAVAILABLE:    return "SDERR_DEV_UNAVAILABLE";
		default:      return "???";
	}
	return "???";
}
//
//	findfirstbitmsb  --  find first one bit, counting from msb end
//
//	Used in IEEE-1394 isochronous channel allocation
//
int findfirstbitmsb(uint32_t in)
{	for (unsigned int i=0; i<sizeof(in)*8; i++)
	{	uint32_t mask =  bitmsb(i);					// all zeroes except the wanted bit
		if ((in & mask)) return(i);						// if desired bit is set
	}
	return(-1);													// failure
}
//
//	class DbufPool -- Mindready dbuf encapsulation
//
//	Destructor
//
DbufPool::~DbufPool()
{
	destroy();																		// destroy buffer pool
}
//
//	destroy --  destroy pool
//
//	Checks for leaks
//
void DbufPool::destroy()
{	if (m_pool)
	{	int count = dbufGetAvailableCount(m_pool);				// get available buffer count
		if (count != m_bufcount)											// if leaks
		{	logprintf("ERROR: Buffer pool leaked %d of %d buffers.\n",m_bufcount-count,m_bufcount);
			sleep(5);																// ***TEMP*** wait for message to print
			abort();																	// ***TEMP*** abort program, avoid DMA into released memory
		}
		dbufDestroyPool(m_pool);											// done with buffer pool
		m_pool = 0;																// done
	}
}
//
//	create --  create a buffer pool
//
int DbufPool::create(size_t size, uint32_t bufcount, int flags)
{	destroy();																		// get rid of any existing pool, with leak check
	m_pool = dbufCreatePool(size,bufcount,flags);				// create the pool
	if (m_pool == 0) return(ENOMEM);								// out of memory if did not create
	m_bufcount = dbufGetAvailableCount(m_pool);			// get initial buffer count
	m_bufsize = size;															// keep size info
	return(EOK);																	// success
}

