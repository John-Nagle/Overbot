//
//	replywait.h  --  Generic support for waiting for a reply for something.
//
//	Part of
//	devfw-ohci  -- FireWire (IEEE-1394) driver for OHCI-compatible devices
//
//	John Nagle
//	Animats
//	
//	January. 2003
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
#ifndef REPLYWAIT_H
#define REPLYWAIT_H
#include <limits.h>
#include <string>
#include <map>
#include <vector>
#include "mutexlock.h"
//
//	class replywait  -- a query and its reply
//
//	Generally, one process puts the query, another takes it,
//	and later replies. The original process then takes the reply.
//
//	The replying process can block waiting for the reply.
//	The replying process can time out while waiting, in which
//	case the query is cancelled.
//
//	All functions return true if success.
//
//	***POTENTIAL RACE CONDITION***
//		put query
//		get reply starts
//										peek query, true
//		get reply times out
//		put query
//		get reply starts
//										put reply
//		get reply succeeds, with wrong reply
//
//	So peek query and put reply needs to be an atomic operation.
//	Do	MutexLock lok(replywaitobj.getlock()) around peek/put sequence
//
template<class T, class U>  class ReplyWait {
private:
	Mutex m_lock;											// general lock
	BoundedBuffer<T,1> m_query;					// the query area
	BoundedBuffer<U,1> m_reply;					// the reply area
public:
	//	Querying process side.
	bool putquery(const T& query)				// put query - no block
	{	return(m_query.tryput(query)); }			// try put into queue
	bool getreply(U& reply)							// take reply - block
	{	m_reply.get(reply);	}							// wait for reply in bounded buffer

	bool getreply(U& reply, float maxwait);	// take reply - time-limited block
	//	Replying process side.
	bool peekquery(T& query)						// return query if one available - no block
	{	return(m_query.peek(query));	}			// peek at query queue
	void getquery(T& query);							// take query if one available - no block
	{	return(m_query.get(query);	}				// get it
	bool putreply(const U& reply)					// put reply - no block
	{	return(m_reply.tryput(reply));	}			// put, fails if full
	
	Mutex& getlock() { return(m_lock);	}		// return lock
};
//	Implementation
//
//	takereply  --  take reply from bounded buffer, blocking, with timeout
//
//	The only complicated case. On a timeout, the query is deleted.
//
inline bool ReplyWait<T,U> takereply(U& reply, float maxwait)
{	MutexLock lok(m_lock);								// note lock surrounds a block
	bool success = m_reply.get(reply,maxwait);// wait for reply
	if (success) return(true);							// if success, we are done
	T querysink;												// discard query if present
	m_query.tryget(querysink);						// discard any query present.
	return(false);											// fails
}

#endif // REPLYWAIT_H