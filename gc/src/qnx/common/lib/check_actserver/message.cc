////////////////////////////////////////////////////////////////////////////
//
//    File: message.cc
//
//    Usage:
//        See actserver.h.
//
//    Description:
//        Routines related to the message handling of the Server.
//
//        Need to have this file in the project of the derived Server to
//        make sure that "msg" is large enough for both the base and
//        derived messages.
//
//        This file should be the same for all derived classes.  Only need
//        to change messageadditional.cc.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        December, 2003
//
/////////////////////////////////////////////////////////////////////////////

#include "check_actserver_.h"

void MyServer_::MessageThread()
{
	// start collecting messages forever
	for ( ; ; ) {		
		// wait for message
		rcvid = serverport->MsgReceive(msg);
		
		// handle errors
		if ( rcvid < 0 ) {
			fflush(stdout);
			perror("MyServer_::MessageThread - MsgReceive failed\n");
			sleep(1);		// avoid tight loop if repeated trouble FIX
			continue;
		}
		if ( rcvid == 0 ) {
			perror("MyServer_::MessageThread - received a pulse\n");
			// pulses don't require a reply
			continue;
		}
		
		//	handle message
		//if ( verbose ) {
		//	perror("MyServer_::MessageThread - received a message\n");
		//}
		MyServer_::messageHandle();
		
		// tell the watchdog we are still alive FIX
		////serverport.watchdogreset();
	}
}

void MyServer_::messageHandle()
{
	// fan out on server requests, based on code at beginning of message
	switch ( msg.m_simu.m_msgtype ) {
		case ActServerMsgSIMU::k_msgtype:			// SIMU msg
			if ( msg.m_simu.get ) {
				msg.m_simu.simulation = Simulation();
			} else {
				Simulation(msg.m_simu.simulation);
			}
			msg.m_simu.err = ActServer::ERR_OK;
			break;
		case ActServerMsgINIT::k_msgtype:			// INIT msg
			if ( !msg.m_init.get ) {
				if ( Simulation() ) {
					Initialized(true);
				} else {
					// get=false, need to initialize
					Initialized(false);
					InitializeThread();
				}
			}
			msg.m_init.initialized = Initialized();
			msg.m_init.err = ActServer::ERR_OK;
			break;
		case ActServerMsgVERB::k_msgtype:			// VERB msg
			if ( msg.m_verb.get ) {
				msg.m_verb.verbose = Verbose();
			} else {
				Verbose(msg.m_verb.verbose);
			}
			msg.m_verb.err = ActServer::ERR_OK;
			break;
		default:
			messageHandleAdditional(msg.m_simu.m_msgtype);
			break;
	}
	
	// reply to message by sending it back
	int err = MsgReply(rcvid, EOK, msg);
	if ( err ) {
		perror("MyServer_::messageHandle - MsgReply failed\n");
	}
}