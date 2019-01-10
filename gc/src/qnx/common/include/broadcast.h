//
//	broadcast.h  -- broadcast message support for QNX
//
//	John Nagle
//	Team Overbot
//	October, 2003
//
//	A message-passing library for use with the "watchdog" program.
//
//	Any program running under the watchdog can send a message to
//	the broadcast program, which will relay it to all other subscribing
//	programs. 
//
//	Don't overdo this; you'll bother the other servers with unwanted traffic.
//	it's for unusual conditions, not repeated messages.
//
//	Message types for broadcast messages must begin with "BC", or the
//	broadcaster will reject them. This avoids confusion between broadcast
//	and non-broadcast messages. Other than that, the broadcaster does not
//	care about message content.
//
//	There is a broadcast message size limit.  See below.
//
//	Sending a broadcast message is non-blocking; if the queue is full,
//	an error status will be returned. 
//
#ifndef BROADCAST_H
#define BROADCAST_H
#include "messaging.h"
//
//	class BroadcastMsg  -- package for sending broadcast messages
//
//	Everything is static; there's no need for an instance
//
class BroadcastMsg {
	const size_t MaxSize = 512;																		// max broadcast message size
#ifdef NOTYET																								// for now, all clients are subscribed to all messages
	static int subscribeall();																				// subscribe to all broadcast messages
	static int unsubscribeall();																			// unsubscfibe from all broadcast messages
	static int subscribe(const MsgBase& type);												// add subscription for this type
	static int unsubscribe(const MsgBase& type);											// delete subscription for this type
#endif // NOTYET
	static int send(void* msg, size_t msgsize);												// send a broadcast message
	static template<class > int send(const T& msg)										// template form, for convenience
	{	return(send(coid, &msg, sizeof(msg)); }
};

#endif // BROADCAST_H