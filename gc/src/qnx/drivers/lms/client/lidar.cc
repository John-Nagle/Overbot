
#include "lidar.h"


Lidar::Lidar(char *serverID, int timeout)
{
	// define client port
	clientport = new MsgClientPort(serverID, timeout);	
}


LidarServer::Err Lidar::InitializeServer() {
	LidarServerMsg msg;
	int err;
	
	msg.m_init.m_msgtype = LidarServerMsgINIT::k_msgtype;
	msg.m_init.get = false;
	msg.m_init.initialized = false; // not necessary.
	
	err = clientport->MsgSend(msg, msg);
	if ( err ) {
		perror("Lidar::Initialize: MsgSend failed");
		return LidarServer::ERR_MSGSEND_FAILED;
	} else {
		// FIX - do something interesting here.
		return msg.m_init.err;
	}
}

LidarServer::Err Lidar::InitializeStatusGet(bool &status) {
	LidarServerMsg msg;
	int err;
	
	msg.m_init.m_msgtype = LidarServerMsgINIT::k_msgtype;
	msg.m_init.get = true;
	msg.m_init.initialized = false; // not necessary.
	
	err = clientport->MsgSend(msg, msg);
	if ( err ) {
		perror("Lidar::InitializeStatusGet: MsgSend failed");
		return LidarServer::ERR_MSGSEND_FAILED;
	} else {
		status = msg.m_init.initialized;
		return msg.m_init.err;
	}
}