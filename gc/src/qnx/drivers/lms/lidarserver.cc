////////////////////////////////////////////////////////////////////////////
//
//    File: main.cc
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
const float k_lidarmaxslewrate = 0.01;											// but only allow a small slew rate
const int k_max_bad_resets = 12;												// this many bad resets and we cycle LIDAR power. Just in case.
//
//    LidarServer::LidarServer - default constructor
//
LidarServer_::LidarServer_()
: 	m_tilt(k_tiltcontroller,false),
	m_wash(m_tilt),
	m_serverport(0.0),
	m_lidarpll(k_lidarperiodns, k_lidarmaxerrns, k_lidarmaxslewrate),
	m_verbose(false),
	m_collectingData(false),
	m_receivingData(false),
	m_badReads(0),
	m_badResets(0),
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
	
	// initialize server state
	resetPointers();													// reset input buffering to empty
	memset(m_inBuffer, 0, BUFFER_SIZE);
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
	pthread_create(NULL, NULL, &outputThreadStart, this);			// dequeues output and sends it out
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
	logprintf("Operating thread started.\n");
	//	Keep-alive loop.  Works to keep the connection alive.
	for (;;)
	{	
		if (m_receivingData)																// if data is coming in
		{	m_receivingData = false;													// clear, incoming data will set
			sleep(1);																			// check again later
			continue;
		}
		//	Not receiving data. Begin TCP reconnection and LMS reset sequence.
		m_collectingData = false;														// flush any scan lines received during restart
		parkLidar();																			// return LIDAR to parked position
		if (m_badResets++ > k_max_bad_resets)									// if too many bad resets
		{	logprintf("Too many failed resets. Cycling LIDAR power.\n");
			m_lidarsocket.close();															// cycle TCP connection and LIDAR power
			m_badResets = 0;																// reset power cycling counter
		}
		//	Not receiving data, trouble.
		if (!m_lidarsocket.isopen())														// if somebody closed socket
		{	
			Controller::Err err = m_tilt.setLidarPower(false);					// power off LIDAR
			if (err != Controller::ERR_OK)													// if trouble
			{	logprintf("Unable to power LIDAR off.\n");
				sleep(3);																			// allow enough time for LIDAR and Sealevel box to reset.
				continue;
			}
			sleep(2);																				// allow time for unit power off
			err = m_tilt.setLidarPower(true);											// power on LIDAR
			if (err != Controller::ERR_OK)													// if trouble
			{	logprintf("Unable to power LIDAR on.\n");							// report
				sleep(1);
				continue;
			}
			logprintf("Connecting to LMS on \"%s\".\n", LMS_SERVER_NAME);								// attempt connection
			int stat = m_lidarsocket.open(LMS_SERVER_NAME, LMS_SERVER_PORT);	// try to connect
			if (stat < 0)																			// TCP problem
			{	logprintf("Unable to connect to  \"%s\", port %d: %s\n", LMS_SERVER_NAME, LMS_SERVER_PORT, strerror(errno));
				sleep(1);																			// avoid going compute bound
				continue;																			// try again later
			}
			m_badResets = 0;																	// no bad resets since last power cycle
		}
		usleep(500000);																	// wait for read thread to become ready
		//	We are connected.  Reset device, repeatedly if necessary, until we get a reset reply.
		//	On a reset, we get a kLMSConfirmSoftwareResetTelegram almost immediately, unless the device is
		//	already resetting or powering up. Then there's a long delay, followed by a kLMSPowerOnTelegram.
		logprintf("Resetting LMS.\n");													// send reset sequences
		if ((m_inWritePtr != m_inBuffer)												// ***TEMP***
		|| (m_inReadPtr != m_inBuffer))
		{		logprintf("Buffers out of sync at reset.  in=%d  out=%d\n",
					m_inWritePtr-m_inBuffer, m_inReadPtr-m_inBuffer);	// should be at look for sync.
		}		
		int stat = requestEmptyMessage();										// flush output
		if (stat < 0)																			// if fail
		{	logprintf("Error writing to TCP connection.\n");
			m_lidarsocket.close();														// close socket
			continue;
		}
		usleep(200000);																	// allow idle time on serial line after flush
		waitForReply(kLMSConfirmSoftwareResetTelegram,0);			// flush input queue
		logprintf("Sending reset to LMS.\n");										// sending
		stat = requestReset();															// reset device, which takes tens of seconds
		if (stat < 0)																			// if fail
		{	logprintf("Error writing to TCP connection.\n");
			m_lidarsocket.close();														// close socket
			continue;
		}
		uint8_t reply = 0;
		//	We can either get a Confirm Software Reset or a PowerOnTelegram next.  
		if (!waitForReply(kLMSConfirmSoftwareResetTelegram, kLMSPowerOnTelegram,4.0, reply))		// should get reset confirmation immediately
		{	logprintf("No reply to reset command, retrying.\n");			// retry
			continue;
		}
		if (reply != kLMSPowerOnTelegram)										// if not a power-on completion
		{	logprintf("LMS acknowledged reset request. Waiting for power-on completion.\n");
			//	We should next get a power-on telegram.	This can take a while
			if (!waitForReply(kLMSPowerOnTelegram,30.0))
			{	logprintf("No power-on telegram from LMS after reset.\n");	// fails, and normally should not
				continue;																		// retry
			}
		}
		logprintf("LMS completed power-on.\n");
		//	Try to get sensor status.
		stat = requestSensorStatus();												// get sensor info
		if (stat < 0)																			// if fail
		{	logprintf("Error writing to TCP connection.\n");
			m_lidarsocket.close();														// close socket
			continue;
		}
		if (!waitForReply(kLMSRespondSensorStatusTelgram,6.0))		// if didn't get proper response
		{	logprintf("Trouble getting LMS status, retrying.\n");
			continue;
		}
#ifdef TEMPTURNOFF // LMS not responding to these, unclear why
		//	Read configuration of device
		stat = readLMSConfiguration();
		if (stat < 0)																			// if fail
		{	logprintf("Error writing to TCP connection.\n");
			m_lidarsocket.close();														// close socket
			continue;
		}
		if (!waitForReply(kLMSRespondCurrentLMSConfiguration,6.0))		// if didn't get proper response
		{	logprintf("Trouble reading LMS configuration, retrying.\n");
			continue;
		}
		//	Define configuration - sets up device.
		//	Done to set availability mode, so we can recover from dazzle faster.
		stat = sendLMSConfiguration();
		if (stat < 0)																			// if fail
		{	logprintf("Error writing to TCP connection.\n");
			m_lidarsocket.close();														// close socket
			continue;
		}
		if (!waitForReply(kLMSRespondDefineLMSConfiguration,6.0))		// if didn't get proper response
		{	logprintf("Trouble setting LMS configuration, retrying.\n");
			continue;
		}
		////requestSwitchToMode(kLMSContinuousAllValuesMode);		// ask for send-all-values mode
#endif // TEMPTURNOFF
		logprintf("Requesting continuous scan lines.\n");
		stat = requestSwitchToMode(kLMSContinuousPartialValuesMode);	// send 1 degree, 75Hz.
		if (stat < 0)																			// if fail
		{	logprintf("Error writing to TCP connection.\n");
			m_lidarsocket.close();														// close socket
			continue;
		}
		if (!waitForReply(kLMSRespondChangeOperatingModeTelegram,6.0))		// if didn't get proper response
		{	logprintf("Trouble switching to continuous scan mode, retrying.\n");
			continue;
		}
		logprintf("LMS acknowledged request for continuous scan. Line scans should now start.\n");
		m_collectingData = true;														// now accept scan lines coming in
		sleep(5);																				// allow time for data to start
		//	Data should now start coming in. We will check on the next loop
	}
}
//
//	forceReset -- force an LMS reset
//
void LidarServer_::forceReset()
{	if (m_collectingData == false) return;										// we're already resetting
	m_collectingData = false;															// stop data collection
	m_receivingData = false;															// not collecting data now, will reset within 1s.
}
//
//	waitForData -- wait for incoming data
//
bool LidarServer_::waitForData(unsigned long leftToRead)
{
	if (!m_lidarsocket.isopen())														// not open
	{	usleep(100000);																	// wait while output side retries
		return(false);																		// error
	}
	int read = 0;
	
	if ( (m_inWritePtr + m_leftToRead) > (m_inBuffer + BUFFER_SIZE)) {
		perror("LidarServer: leftToRead got too big");
		// we can't read that much... so something must be wrong.
		// dropping down to smaller size.
		// probably just bad data on the wire which confused us.
		m_leftToRead = (m_inBuffer + BUFFER_SIZE) - m_inWritePtr;
	}
	
	if ( (read = m_lidarsocket.read_data(m_inWritePtr, leftToRead) ) != (int)m_leftToRead ) {
		if (read == -2)																// timeout
		{	resetPointers();															// go back to ground state
			return(false);														
		}
		if (read <= 0) {
			perror("LidarServer: Error while reading.");
			usleep(100000);														// might have closed, don't go compute bound
			m_badReads++;
			return false;
		} else {
			perror("LidarServer: Failed to read enough.");
			logprintf("Read only %i, expected %lu\n", read, leftToRead);
			return false;
		}
	}
	
	m_inWritePtr += read;
	m_badReads = 0;
	return true;
}
//
//	parkLidar -- return LIDAR to parked position
//
//	Done during resets, primarily to make it clear what's going on.
//	Also has the effect that the input side of the map program knows the LIDAR is in an invalid state.
//
void LidarServer_::parkLidar()
{	const double k_parked_tilt = 0.0;							// straight down
	const double k_parked_tilt_speed = 999;				// max speed
	if (m_wash.washing()) return;
	m_tilt.setTilt(k_parked_tilt, k_parked_tilt_speed);	// set tilt to 0
}
//
//	dataThread -- receive side thread
//
//	Runs forever.
//
void *LidarServer_::dataThread()
{
	logprintf("Input data thread started.\n");	
	resetPointers();												// reset buffer to begin reading
	
	unsigned long dataCount = 0;
	
	uint64_t start, stop;
	bool first = true;											// first time, for debug count
	
	for (;;)
	{	if (waitForData(m_leftToRead)) 				// if no error
		{	if (handleIncomingData() == kLMSRespondValuesTelegram)
				dataCount++;									// tally lines received
			adjustPointers();									// use up data
		} else {													// if fail
			resetPointers();										// discard any data on err
		}
		//	Debug output
		const unsigned int k_count = 750;			// report this often
		if (first)
		{	first = false; 
			ClockTime(CLOCK_REALTIME,0,&start); 
		}
		if (dataCount > k_count) {
			ClockTime(CLOCK_REALTIME,0,&stop);
			double elapsed = stop - start;				// elapsed time in ns.
			elapsed *= (1.0e-9);								// seconds
			double persec = k_count/elapsed;		// rate
			logprintf("Received %d scan lines in %3.3f seconds (%3.3f telegrams/sec)\n",  k_count, elapsed, persec);
			start = stop;
			dataCount = 0;
		}
	}
	//	Never exits
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
				forceReset();											// force a reset
				m_tilt.ResetToPowerUp();						// reset the tilt head, too
			}
			msg.m_init.m_initialized = m_collectingData;	// if collecting data, good
			msg.m_init.m_err = LidarServer::ERR_OK;
			break;
			
		case LidarServerMsgTILT::k_msgtype:			// TILT msg - tilt request
			msg.m_tilt.m_washing = m_wash.washing();	// true if washing
			if (!msg.m_tilt.m_washing && m_collectingData)	// if not washing and LIDAR is working
			{	msg.m_tilt.m_err = m_tilt.setTilt(msg.m_tilt.m_tilt, msg.m_tilt.m_tiltrate);	}	// do the tilt request
			msg.m_tilt.m_tilt = m_tilt.getTilt();				// return actual tilt, which may be a NaN
			msg.m_tilt.m_tiltrate = 0;							// we don't return actual tilt rate
			break;
			
		case LidarServerMsgWASH::k_msgtype:		// WASH msg - wash request
			m_wash.requestWash();							// request a wash cycle
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

