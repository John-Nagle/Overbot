#ifdef OBSOLETE
//
//      client_test  -- test for lidarserver under watchdog
//		Eric Seidel, modeled after code from J. Nagle
//      December, 2003
//
//      Exercises lidarserver/lms driver program
//
//      Usage: client_test [options]
//
//      Options:        
//            currently none.
//      

#include "lidar.h"


void messageHandle()
{
	// fan out on server requests, based on code at beginning of message
	switch ( msg.m_simu.m_msgtype ) {
		case LidarServerMsgDATA::k_msgtype:			// DATA msg

			printf("Got data message from lidar driver...\n");
			// do something interesting with the data...
			//msg.m_data.data.timestamp; 
			break;
		default:
			printf("Sent unhandled data request (type: %i)\n", msg.m_simu.m_msgtype);
			break;
	}
	
	// reply to message by sending it back
	// really should be empty...
	int err = MsgReply(rcvid, EOK, msg);
	if ( err ) {
		perror("LidarClient: MsgReply failed.\n");
	}
}


serverLoop() {
	// initialize server messaging
	MsgServerPort serverport = new MsgServerPort(0.0);	// define message port, no timeout
	
	int stat = serverport->ChannelCreate();	// create a channel, tell watchdog
	if  ( stat ) {
		perror("LidarClient: ChannelCreate failed.\n");
		serverport->Dump();
	}
	
	// start collecting messages forever
	while (1) {		
		// wait for message
		rcvid = serverport->MsgReceive(msg);
		
		// handle errors
		if ( rcvid < 0 ) {
			fflush(stdout);
			perror("LidarClient: MsgReceive failed.\n");
			sleep(1);		// avoid tight loop if repeated trouble FIX
			continue;
		}
		if ( rcvid == 0 ) {
			perror("LidarClient: received a pulse.\n");
			// pulses don't require a reply
			continue;
		}
		
		//	handle message
		if ( verbose ) {
			perror("LidarServer: received a message.\n");
		}
		messageHandle();
		
		// tell the watchdog we are still alive FIX
		//serverport.watchdogreset();
	}
}

int main(int argc, const char* argv[]) {
	Lidar l;
	
	l->InitializeServer();
	
	// really should probably make sure the server thread is up first...
	serverLoop();
	return 0;
}
#endif // OBSOLETE