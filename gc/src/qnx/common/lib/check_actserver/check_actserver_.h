/////////////////////////////////////////////////////////////////////////////
//
//    File: check_actserver_.h
//
//    Usage:
//        See check_actserver.h.
//
//    Description:
//       See check_actserver.h.
//
//       This is the internal header file (check_actserver.h is the external one).
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        December, 2003
//
/////////////////////////////////////////////////////////////////////////////

#ifndef CHECK_ACTSERVER__H
#define CHECK_ACTSERVER__H

#include "actserver_.h"
#include "check_actserver.h"

#define CHECK_ACTSERVER_MVPSERVER_ID			("MVPC")
#define CHECK_ACTSERVER_MVPSERVER_TIMEOUT	(0.2)
#define CHECK_ACTSERVER_MVP_NODE_NUM			(2)

using namespace ost;	// in mutexlock.h

class MyServer_: public ActServer_ {
public:
	MyServer_();

	void MessageThread();	// main thread (defined here so "msg" large enough)

private:
	// server message handling (main thread)
	MsgServerPort *serverport;	// message port
	int rcvid;
	MyServerMsg msg;			// area for incoming and outgoing message
	void messageHandle();
	void messageHandleAdditional(uint32_t m_msgtype) {}; // do nothing
		
	// server data
	// declare server data variables here
		
	// MVP object to send INST messages to MVP Server
	MVP *m;
};

#endif // CHECK_ACTSERVER__H
