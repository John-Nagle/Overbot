////////////////////////////////////////////////////////////////////////////
//
//    File: check_actserver_.cc
//
//    Usage:
//        See check_actserver.h.
//
//    Description:
//        The routines related to setting up a My Server object.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        December, 2003
//
/////////////////////////////////////////////////////////////////////////////

#include "check_actserver_.h"

MyServer_::MyServer_()
{
	ServerName("My");
	
	// initialize server messaging
	serverport = new MsgServerPort(0.0);	// define message port, no timeout
	int stat = serverport->ChannelCreate();	// create a channel, tell watchdog
	if  (stat ) {
		perror("MyServer_::MyServer_ - ChannelCreate failed.\n");
		serverport->Dump();
	}
	
	// initialize server variables here
	
	// initialize init thread variables here
	
	// initialize target thread variables here
	
	// initialize data loop variables here
	
	// initialize control loop veriables here
	
	// initialize MVP object
	m  = new MVP(CHECK_ACTSERVER_MVPSERVER_ID, CHECK_ACTSERVER_MVPSERVER_TIMEOUT, 
	             CHECK_ACTSERVER_MVP_NODE_NUM);
};