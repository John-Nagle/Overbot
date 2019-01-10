////////////////////////////////////////////////////////////////////////////////
//
//    File: wayptserver.h
//
//    Description:
//	Waypt Server main class
//
//    See also:
//        wayptserver.h
//
//    Written By:
//       Achut Reddy
//       Team Overbot
//       December, 2003
//
/////////////////////////////////////////////////////////////////////////////////

#ifndef WAYPTSERVER_H
#define WAYPTSERVER_H

#include <pthread.h>
#include <stdio.h>
#include <vector>

#include "messaging.h"
#include "mutexlock.h"

#include "wayptservermsg.h"
#include "rddf.h"

using namespace std;

// WayptServer main class
class WayptServer {

public:
    WayptServer();			// constructor
    ~WayptServer();			// destructor

    void messageThread();		// main thread to receive messages
    void menuThreadCreate();		// menu thread

    int readWaypts(const char *fileName);	// read in waypoint list

    void setVerbose(bool on) {
        m_verbose = on;
    }

private:

    // Message handling
    MsgServerPort	m_serverPort;	// receiving port for msgs

    bool		m_verbose;
    bool		m_useWatchdog;

    vector<Waypoint>	m_waypointList;	// list of waypoints
    Waypoint		m_firstWaypoint;	// first waypoint

    // need static function loopStart() for pthread_create
    // loopStart calls the instance-specific function loop()
    void *menuThread();
    static void *menuThreadStart(void* arg) {
        return(reinterpret_cast<WayptServer *>(arg)->menuThread());
    }

	//	Message processing
    void handleMessage(int rcvid, const WayptServerMsg::MsgUnion& msg);
    void handleGetNumberOfWaypoints(int rcvid, const WayptServerMsg::GetNumberOfWaypoints& msg);
    void handleGetWaypoint(int rcvid, const WayptServerMsg::GetWaypoint& msg);
    void handleGetWaypointByXY(int rcvid, const WayptServerMsg::GetWaypointByXY& msg);

};

#endif // WAYPTSERVER_H
