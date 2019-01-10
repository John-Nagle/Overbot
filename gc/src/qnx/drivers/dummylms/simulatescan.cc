//
//	simulatescan.cc  -- simulate a LIDAR scan
//	
//
//	John Nagle
//	Team Overbot
//	April, 2005
//
#include <unistd.h> 
#include "logprint.h"
#include "lidarserver_.h"
//
//	Constants
//
const float k_scanner_elev = 2.08;								// scanner elevation
const float k_max_range = 20.0;									// scanner max range
const uint16_t k_no_return_value = 8193;					// ***CHECK THIS***
//
//	simulateRange -- Simulate data value in the scan line
//
//	Just simulates a flat plane for now
//
static uint16_t simulateRange(int i, float aheaddist)
{	
	float scanangle = ((180/2)-i)*(M_PI/180);					// scan data angle, 0 is straight ahead
	float range = aheaddist / cos(scanangle);				// calculate range
	if (range > k_max_range) return(k_no_return_value);
	return(uint16_t(range*100));									// return range
}											
//
//	Simulate an entire scan line
//
void LidarServer_::simulateDataValues(LidarScanLine& scandata)
{	int cnt = scandata.m_header.m_valueCount;				// count of entries
	float tilt  = scandata.m_header.m_tilt;						// tilt
	assert(cnt <= 181);													// max entries
	float aheaddist = k_scanner_elev / cos(tilt);				// distance ahead
	for (int i=0; i<cnt; i++)
	{	scandata.m_range[i] = simulateRange(i, aheaddist);	}	// simulate a range item
}
//
//	Simulate a scan line
//
void LidarServer_::simulateScan()
{
	LidarServerMsgLISN scanmsg;									// build scan line to send as a message
	scanmsg.m_msgtype = LidarServerMsgLISN::k_msgtype;
	scanmsg.m_err = LidarServer::ERR_OK;								// no error
	//	Construct scan line
	scanmsg.m_data.m_header.m_timestamp = gettimenowns();	// timestamp
	scanmsg.m_data.m_header.m_tilt = m_tilt.updateTilt();				// tilt
	scanmsg.m_data.m_header.m_sensorid = 0;							// only one scanner for now
	scanmsg.m_data.m_header.m_statusByte = 0;							// no errors
	scanmsg.m_data.m_header.m_scanIndex = m_lineserial++;	// cycle serial number
	scanmsg.m_data.m_header.m_valueCount = (scanmsg.m_data.m_header.m_scanIndex & 1) ? 181 : 180;	// even or odd line
	simulateDataValues(scanmsg.m_data);									// add simulated data values
	//	Send scan line
#ifdef OBSOLETE
	int err = m_dataclientport->MsgSend(scanmsg);
	if (err < 0)																// if trouble
	{	logprintf("Unable to send scan line to server: %s\n",strerror(errno));	// note
		sleep(2);																// probably starting up
	}
#endif // OBSOLETE
	bool good = m_linequeue.tryput(scanmsg);				// queue message
	if (!good) {	logprintf("Unable to queue scan line for server - queue full.\n"); }
}
//
//	simulateSending -- send data to the map process
//
//	Thread, runs forever
//
//	We need to queue here, so as not to mess up the timestamps when MsgSend is blocked.
//
void LidarServer_::simulateSending()
{
	for (;;)
	{
		LidarServerMsgLISN scanmsg;									// build scan line to send as a message
		scanmsg.m_msgtype = LidarServerMsgLISN::k_msgtype;
		m_linequeue.get(scanmsg);										// get queued message to send
		int err = m_dataclientport->MsgSend(scanmsg);
		if (err < 0)																// if trouble
		{	logprintf("Unable to send scan line to server: %s\n",strerror(errno));	// note
			sleep(2);																// probably starting up
		}
	}
}