
#ifndef LIDAR_H
#define  LIDAR_H

#include <stdio.h>		// messaging.h needs to include this
#include "messaging.h"
#include "lidarserver.h"

class Lidar {
public:
	Lidar(char *serverID, int timeout);
	
	LidarServer::Err InitializeServer();
	LidarServer::Err InitializeStatusGet(bool &status);
	
private:
	// client port for instructions
    MsgClientPort *clientport;
};

#endif