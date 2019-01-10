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
#include "messages.h"
#include "tiltcontroller.h"
#include "washcontrol.h"
#include "interpolatetilt.h"
#include "timejitter.h"
#include "timeutil.h"
#include "lidarsocket.h"

//
//	Constants
//
// default TCP port settings
const char LMS_SERVER_NAME[] = "lmsserial";									// name of Ethernet to serial converter for LMS
const int LMS_SERVER_PORT = 4760;												// Sealevel Systems raw output data

// default clientID
const char LIDARSERVER_CLIENTID[] = "MAP";									// server that gets LIDAR lines

#define LIDARSERVER_CLIENTID_LEN	(12)// max clientname length

// Talking to the LMS:
#define PASSWORD			"SICK_LMS"
#define PASSWORD_LEN		(8)

const size_t k_output_queue_size = 20;										// store this many scan lines

//	Talking to the tilt controller
const char k_tiltcontroller[] = "gctilt";											// the tilt controller


// we will never receive more than 812 bytes, but we use
// as 2048 byte buffer for now.
#define		BUFFER_SIZE		2048
#define		MESSAGE_HEADER_SIZE		4
#define		MESSAGE_CHECKSUM_SIZE	2


// internal LidarServer class
class LidarServer_ {
public:
	LidarServer_();				// constructor
	~LidarServer_();			// destructor
	void MenuThread();
	void InitializeThread(bool detach);
	
	// accessors
	void SetVerbose(bool verbose) { m_verbose = verbose; }
	
	// for talking to other servers
	void SetupClientConnection(const char *serverID, int timeout);	
	
	void MessageServer();
	void forceReset();																		// forces an LMS reset
private:
	//	Tilt control
	TiltController m_tilt;																	// the tilt controller
	TiltPositions m_tiltpositions;														// recent tilt positions, for interpolation
	WashControl m_wash;																// wash control
	// server message handling
	ost::BoundedBuffer<LidarServerMsgLISN, k_output_queue_size> m_outputqueue;		// output message queue
	MsgServerPort m_serverport;		// message port	
	
    //	operatingThread -- runs the LIDAR, restarting it when necessary.
    void initializeThreads();
    void* operatingThread();
    static void* operatingThreadStart(void* arg)
	{ return(reinterpret_cast<LidarServer_*>(arg)->operatingThread()); }
	void parkLidar();																		// parks tilt head when necessary
	
	//	dataThread -- reads from the LIDAR
	void* dataThread();
    // need static function loopStart() for pthread_create
    // loopStart calls the instance-specific function loop()
    static void* dataThreadStart(void* arg)
	{ return(reinterpret_cast<LidarServer_*>(arg)->dataThread()); }
	
	//	outputThread -- sends data to map server
	void* outputThread();
    // need static function loopStart() for pthread_create
    static void* outputThreadStart(void* arg)
	{ return(reinterpret_cast<LidarServer_*>(arg)->outputThread()); }
	
	
	// QNX message handling as server
	void messageHandle(int rcvid, LidarServerMsg& msg);
	
	// LMS message handling
	SimplePLL	m_lidarpll;													// LIDAR message jitter correction
	ost::BoundedBuffer<uint8_t,5> m_msgin;					// last incoming message type
	bool waitForData(unsigned long leftToRead);
	unsigned char handleIncomingData();
	void adjustPointers();
	void resetPointers();													// reset input buffer to empty
	unsigned char parseDataTelegram(char* messagePtr, size_t messageLength);	// parse one telegram
	bool handleResponseValuesTelegram(char* &messagePtr, LidarScanLine &newData);
	bool waitForReply(LMSIncomingTelegrams msgtype, double timeout)	// wait for this telegram type
	{	uint8_t reply=0; return(waitForReply(msgtype,msgtype,timeout, reply));	}
	bool waitForReply(LMSIncomingTelegrams msgtype1, LMSIncomingTelegrams msgtype2, 
		double timeout, uint8_t& reply);	// two type form
	
	// server state
	bool	m_verbose;															// verbose mode, print too much
	bool	m_collectingData;													// pass scan lines to server (if false, flushing)
	bool m_receivingData;													// a scan line came in recently
	int		m_badReads;
	int		m_badResets;														// number of reset failures
	
	// outgoing messages
	int requestSwitchToMode(unsigned char mode);		// request a mode change
	int requestSensorStatus();
	int requestErrorTelegram();
	int requestReset();														// request a reset
	int requestEmptyMessage();										// send big block of zeroes to flush pipe
	int sendSimpleTelegram(unsigned char type);
	int readLMSConfiguration();
	int sendLMSConfiguration();

	// server resources
	LidarSocket m_lidarsocket;										// the LIDAR's connection
#ifdef OBSOLETE
	int m_sock;				//	Input TCP socket
	
	ost::Mutex m_opening;												// open/close lock
	int openSocket(const char *hostname, unsigned short portnum);
	void closeSocket();
	int read_data(char* buffer, int length);
	int write_data(const char* buffer, int length);
#endif // OBSOLETE
	
	// client port for sending data out on.
    MsgClientPort* m_dataclientport;

	// LMS input buffer - not circular, note.
	char		m_inBuffer[BUFFER_SIZE];									// the buffer
	char		*m_inWritePtr;													// writing here
	char		*m_inReadPtr;													// reading here
	unsigned long	m_leftToRead;	
	
	// prints buffer to stdout.
	void dumpBuffer();
	//	Tilt interpolatoin
	bool gettiltattime(uint64_t desiredtime, float& tilt);			// get tilt at this time
};

#endif
