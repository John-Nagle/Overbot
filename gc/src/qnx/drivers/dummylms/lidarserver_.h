////////////////////////////////////////////////////////////////////////////
//
//    File: lidarserver_.h
//
//    Usage:
//        See lidarserver.h.
//
//    Description:
//        See lidarserver.h.
//
//        This is the internal header file (lidarserver.h is the external one).
//
//        The primary purpose of a Lidar Server is to interface with the
//		  the Laser Range-finder via a high-speed serial port.  The Lidar
//		  accepts only a few external commands, it's primary function being
//		  recieving data from the laser range-finder, and vending that recieved
//		  data to other proccesses.
//
//    See also:
//        lidarserver.h, main.cc,
//
//    Written By:
//        Eric Seidel
//		  (Modeled after code from Celia Oakley)
//        Team Overbot
//        December, 2003
//
/////////////////////////////////////////////////////////////////////////////

#ifndef LIDARSERVER__H
#define LIDARSERVER__H

#include <vector>
#include <pthread.h>
#include "mutexlock.h"
#include "lidarserver.h"
#include "tiltcontroller.h"
#include "timejitter.h"
#include "timeutil.h"

//
//	Constants
//

// default clientID
const char LIDARSERVER_CLIENTID[] = "MAP";									// server that gets LIDAR lines

#define LIDARSERVER_CLIENTID_LEN	(12)// max clientname length


//	Talking to the tilt controller
const char k_tiltcontroller[] = "gctilt";											// the tilt controller


// internal LidarServer class
class LidarServer_ {
public:
	LidarServer_();				// constructor
	~LidarServer_();			// destructor
	void InitializeThread(bool detach);
	
	// accessors
	void SetVerbose(bool verbose) { m_verbose = verbose; }
	
	// for talking to other servers
	void SetupClientConnection(const char *serverID, int timeout);	
	
	void MessageServer();
private:
	//	Tilt control
	TiltController m_tilt;																	// the tilt controller
	// server message handling
	MsgServerPort m_serverport;		// message port	
	
    //	operatingThread -- runs the LIDAR, restarting it when necessary.
    void initializeThreads();
    void* operatingThread();
    static void* operatingThreadStart(void* arg)
	{ return(reinterpret_cast<LidarServer_*>(arg)->operatingThread()); }
	
	//	dataThread -- reads from the LIDAR
	void* dataThread();
    // need static function loopStart() for pthread_create
    // loopStart calls the instance-specific function loop()
    static void* dataThreadStart(void* arg)
	{ return(reinterpret_cast<LidarServer_*>(arg)->dataThread()); }
	
	// QNX message handling as server
	void messageHandle(int rcvid, LidarServerMsg& msg);
	
	// LMS message handling
	SimplePLL	m_lidarpll;													// LIDAR message jitter correction
	//
	//	Simulation handling
	//
	void simulateScan();														// simulate one scan line
	void simulateDataValues(LidarScanLine& scandata);
	void simulateSending();												// send data to the map process
	ost::BoundedBuffer<LidarServerMsgLISN, 50> m_linequeue;		// buffer some scan lines 
	
	// server state
	bool	m_verbose;															// verbose mode, print too much
	bool	m_collectingData;													// true if collecting data
	uint8_t	m_lineserial;														// line serial number (cyclic)
	
	// client port for sending data out on.
    MsgClientPort* m_dataclientport;

};

#endif
