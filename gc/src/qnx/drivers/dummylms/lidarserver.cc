////////////////////////////////////////////////////////////////////////////
//
//    File: lidarserver.cc
//
//		This is the DUMMY LIDAR server for offline testing.
//
//    Usage:
//        See lidarserver.h.
//
//    Description:
//        See lidarserver.h.
//
//        The Lidar Server consists of the following threads:
//            - the main thread, which accepts messages, sends to SICK unit
//            - a timed loop thread, which collects data on a regular basis
//
//    Written By:
//        Eric Seidel
//		  (Modeled after code from Celia Oakley)
//        Team Overbot
//        December, 2003
//
/////////////////////////////////////////////////////////////////////////////

#include <unistd.h> 
#include "logprint.h"
#include "lidarserver_.h"

//
//	LIDAR time corrector constants
//
const uint64_t k_lidarperiodns = 1000000000/75;						// 1/75 second, in nanoseconds
const uint64_t k_lidarmaxerrns = k_lidarperiodns*5;					// max jitter is huge, because data comes in in bunches
const float k_lidarmaxslewrate = 0.05;											// but only allow a small slew rate
//
//    LidarServer::LidarServer - default constructor
//
LidarServer_::LidarServer_()
: 	m_tilt(k_tiltcontroller,false),
////	m_wash(m_tilt),
	m_serverport(0.0),
	m_lidarpll(k_lidarperiodns, k_lidarmaxerrns, k_lidarmaxslewrate),
	m_verbose(false),
	m_dataclientport(0)
	
{
	assert(!m_verbose);											// ***TEMP***
	// initialize server messaging
	
	int stat = m_serverport.ChannelCreate();	// create a channel, tell watchdog
	if  ( stat ) {
		perror("LidarServer: ChannelCreate failed.\n");
		m_serverport.Dump();
		exit(1);															// fails
	}
};

//
//    LidarServer::~LidarServer - default destructor
//
LidarServer_::~LidarServer_()
{
}

//
//	SetupClientConnection -- set up connection to data destination
//
//	We're the client here.
//
//	MsgClientPort will open a connection as necessary on MsgSend.
//
void LidarServer_::SetupClientConnection(const char *serverID, int timeout)
{
	// define client port
	assert(!m_dataclientport);								// can only do this once
	m_dataclientport = new MsgClientPort(serverID, timeout);	
	//	"new" can't fail without a throw.
}
//
//	initializeThreads  -- start the threads
//
void LidarServer_::initializeThreads()
{  
	pthread_create(NULL, NULL, &operatingThreadStart, this);		// start the static that starts the thread in the object
	pthread_create(NULL, NULL, &dataThreadStart, this);		// start the static that starts the thread in the object
}
//
//	operatingThread  --  starts up LMS and keeps it alive.
//
//	This is the only thread allowed to write to the TCP connection.
//	Another thread does all the reading.
//
//	Timeout values here are critical. The LMS is really slow for some operations.
//	See LMS/LMI400, "Definition of interface and telegram traffic, Version 05.00", page 7/99.
//
void* LidarServer_::operatingThread()
{	//	This thread writes; the DataThread reads.
	simulateSending();										// dequeue messages							
	return(0);														// not reached
}


//
//	dataThread -- receive side thread
//
//	Runs forever.
//
void *LidarServer_::dataThread()
{
	const int k_linedelay = (1000000 / 75);		// 75 lines per second
	logprintf("Simulated Input data thread started.\n");	
	for (;;)
	{	usleep(k_linedelay);									// wait for next line time
		simulateScan();											// simulate one scan line
	}
}

//
//	MessageServer -- QNX server, accepts and acts on messages.
//
void LidarServer_::MessageServer()
{
	logprintf("Starting LidarServer.\n");
	//	Start other threads of this object
	initializeThreads();
	// start collecting messages forever
	for ( ; ; ) {		
		// wait for message
		LidarServerMsg msg;													// input message goes here
		int rcvid = m_serverport.MsgReceive(msg);					// wait for a message		
		// handle errors
		if ( rcvid < 0 ) {
			fflush(stdout);
			logprintf("LidarServer: MsgReceive failed: %s.\n",strerror(errno));
			sleep(1);		// avoid tight loop if repeated trouble FIX
			continue;
		}
		if ( rcvid == 0 ) {
			logprintf("LidarServer: received a pulse.\n");
			// pulses don't require a reply
			continue;
		}
		LidarServer_::messageHandle(rcvid, msg);		
		// tell the watchdog we are still alive FIX
		//serverport.watchdogreset();
	}
}
//
//	messageHandle  -- handle a single incoming message
//
void LidarServer_::messageHandle(int rcvid, LidarServerMsg& msg)
{
	// fan out on server requests, based on code at beginning of message
	switch (msg.m_init.m_msgtype ) {
		case LidarServerMsgINIT::k_msgtype:			// INIT msg -- forces a reset
			if ( !msg.m_init.m_get ) {							// if not just getting status
				////forceReset();											// force a reset
				////m_tilt.ResetToPowerUp();						// reset the tilt head, too
			}
			msg.m_init.m_initialized = m_collectingData;	// if collecting data, good
			msg.m_init.m_err = LidarServer::ERR_OK;
			break;
			
		case LidarServerMsgTILT::k_msgtype:			// TILT msg - tilt request
#ifdef WASHER		// no washer in dummy server
			msg.m_tilt.m_washing = m_wash.washing();	// true if washing
			if (!msg.m_tilt.m_washing)							// if not washing
#endif // WASHER
			{	msg.m_tilt.m_err = m_tilt.setTilt(msg.m_tilt.m_tilt, msg.m_tilt.m_tiltrate);	}	// do the tilt request
			msg.m_tilt.m_tilt = m_tilt.getTilt();				// return actual tilt, which may be a NaN
			msg.m_tilt.m_tiltrate = 0;							// we don't return actual tilt rate
			msg.m_tilt.m_err = Controller::ERR_OK;
			msg.m_tilt.m_washing = false;							// no washing
			break;
			
		case LidarServerMsgWASH::k_msgtype:		// WASH msg - wash request
#ifdef WASHER		// no washer in dummy server
			m_wash.requestWash();							// request a wash cycle
#endif // WASHER
			MsgError(rcvid, EOK);								// always permitted
			return;														// done
			
		default:															// bad message
			logprintf("Server received unknown request (type: 0x%08x)\n", msg.m_init.m_msgtype);
			MsgError(rcvid, EINVAL);							// fails
			return;
	}
	
	// reply to message by sending it back (???)
	int err = MsgReply(rcvid, msg);
	if ( err < 0 ) {
		logprintf("LidarServer: MsgReply failed: %s.\n",strerror(errno));
	}
}

