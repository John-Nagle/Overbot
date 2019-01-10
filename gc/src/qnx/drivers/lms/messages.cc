////////////////////////////////////////////////////////////////////////////
//
//    File: messages.cc
//
//    Usage:
//        See lidar.h.
//
//    Description:
//        See lidar.h.
//
//        This file handles the parsing and interpreting of messages
//        sent to us from the Laser Range-finder.
//
//    Written By:
//        Eric Seidel
//		  (Modeled after code from Celia Oakley)
//        Team Overbot
//        December, 2003
//
/////////////////////////////////////////////////////////////////////////////
#include <strings.h>
#include <unistd.h> /* for sleep() */
#include "logprint.h"
#include "lidarserver_.h"
#include "telegram.h"
#include "messages.h"

void LidarServer_::dumpBuffer() {
	logprintf("Raw buffer data (%p):\n", m_inBuffer);
	for (char *ptr = m_inBuffer; ptr < m_inWritePtr; ptr++) {
		//printf("%02x (%c) ", *ptr, *ptr);
		logprintf("%02x ", *ptr);
	}
	logprintf("\n");
}


inline unsigned char read_char(char* &messagePtr) {
	unsigned char value = *messagePtr;
	messagePtr += 1;
	return value;
}


inline unsigned short read_short(char* &messagePtr) {
	unsigned short value = *((unsigned short *)(messagePtr));
	messagePtr += 2;
	return value;
}
//
//	handleResponseValuesTelegram  -- actually get a scan line and do something with it
//
bool LidarServer_::handleResponseValuesTelegram(char* &messagePtr, LidarScanLine &newData)
{	
	ushort_t valueWord = read_short(messagePtr);
	newData.m_header.m_valueCount = (valueWord & kLMSMeasuredValuesCountMask);
	
	bool partialScan = valueWord & kLMSPartialScanMask;
	bool usingMM = ((valueWord & kLMSMeasuredValueUnitMask) & kLMSMeasuredValueUnitMM);
	//	If too many data points, the LMS is in the wrong mode and should be reset.
	if (newData.m_header.m_valueCount > LMS_MAX_DATA_POINTS) {
		logprintf("LidarServer: Error, bad number of data values: %i, ignoring packet.\n", newData.m_header.m_valueCount);
		forceReset();																// force a reset. How did this happen?
		return(false);															// line fails
	}
	
	//	Timestamp the incoming data.
	//	This is important because the timestamp is used to locate the vehicle position for when the line
	//	was scanned. We correct the timebase so that lines come in at a uniform 75HZ, with
	//	timestamps consistent with that.
	uint64_t now = gettimenowns();									// get time now
	uint64_t lastnow = m_lidarpll.gettimebase();					// get last time
	bool good = m_lidarpll.correcttimebase(now);				// perform timebase correction
	if (!good)																		// if unable to correct time
	{	uint64_t diff = now - lastnow;									// get time
		int period = m_lidarpll.getperiod();							// get period
		int errns = int(diff) - period;										// error in nanoseconds
		logprintf("LMS timestamp correction failed. Error %1.4f secs. Resynching.\n", errns*0.000000001);
		return(false);															// drop message
	}
	newData.m_header.m_timestamp = now;						// set adjusted time
	newData.m_header.m_sensorid = 0;								// which sensor, always 0 for now.
	//	Tiilt data is requested on every scan line.  But the scan line data is earlier than
	//	the tilt data, so we have to save and interpolate tilt data. Without this, we get banding
	//	in the tilt info.
	float tilt;																			// current tilt, radians
	bool goodtilt = gettiltattime(now, tilt);							// get interpolated tilt at corrected time
	if (!goodtilt)																	// could not interpolate tilt
	{	logprintf("Unable to interpolate LMS tilt for time %lld.\n", now);	
		return(false);															// fails
	}
	newData.m_header.m_tilt = tilt;										// get actual tilt angle
	
	// copy over the range data values.
	size_t datalength = newData.m_header.m_valueCount * sizeof(newData.m_range[0]);	// size of data in bytes
	memcpy(newData.m_range, messagePtr, datalength); // copy range data to output
	messagePtr += datalength;											// use up the data

	
	uchar_t scanIndex = 0, telegramIndex = 0;
	
#if 1
	// if we set it up to send indicies
	scanIndex	= read_char(messagePtr);
	telegramIndex = read_char(messagePtr);
#endif
	newData.m_header.m_scanIndex = scanIndex;							// scan index, cyclic
	// read status byte.
	newData.m_header.m_statusByte  = read_char(messagePtr);
	newData.m_header.m_unused1 = 0;											// future expansion
	newData.m_header.m_unused2 = 0;
	if (m_verbose)
	{	logprintf("DataMessage: time: %llu vCount: %i partialScan: %i usingMM: %i status: %02x scanIdx: %i teleIdx: %i\n",
			newData.m_header.m_timestamp, newData.m_header.m_valueCount, partialScan, usingMM, 
			newData.m_header.m_statusByte, scanIndex, telegramIndex);
	}
	return(true);																					// success, keep message
}
//
//	handleIncomingData -- reads one data telegram from LMS
//
//	Returns telegram type.
//
//	Messes with buffer pointers in obscure, somewhat painful way.
//
//	The general idea is that the message always begins at the beginning of the buffer,
//	because reads are aligned so as to make that happen.  Exceptions to that rule
//	are handled by copying the data back to the beginning of the buffer.  This
//	does not happen often, hopefully.
//
unsigned char LidarServer_::handleIncomingData()
{
	unsigned short totalRead =  m_inWritePtr - m_inBuffer;							// bytes available for read
	
	// make sure we have enough for a header
	if ( totalRead < MESSAGE_HEADER_SIZE ) {
		// we don't have enough for a header...
		m_leftToRead = MESSAGE_HEADER_SIZE - totalRead;
	}
	
	char *messagePtr = m_inBuffer;
	
	unsigned char messageSTX = read_char(messagePtr);
	
	// sanity check the STX
	if ( messageSTX != kLMSTelegramSTX ) {
		m_inReadPtr++;
		if (m_verbose)
		{	logprintf("LMS data stream out of sync (Expected STX, got 0x%02x)!\n", messageSTX);
			////dumpBuffer();
		}
		// we need one more to replace this one...
		m_leftToRead = 1;
		return kLMSNoTelegram;
	}
	
	unsigned char messageAddress = read_char(messagePtr);
	
	// sanity check the Address
	if (messageAddress != kLMSTelegramHostComputerAddress ) {
		if (m_verbose)
		{	logprintf("Dropping bad data from serial (Address: %02x)!\n", messageAddress);
			dumpBuffer();
		}
		// we need two more to replace the first one...
		m_inReadPtr += 1;
		m_leftToRead = 1;
		return kLMSNoTelegram;
	}
	
	unsigned short messageLength = read_short(messagePtr);
	
	unsigned long totalLength = messageLength + MESSAGE_HEADER_SIZE + MESSAGE_CHECKSUM_SIZE;
	
	// sanity check the Length...
	if (totalLength > kLMSMaximumTelegramLength) {
		m_inReadPtr += 2;
		logprintf("Dropping bad data from serial (Length: %04x)!\n", messageLength);
		if (m_verbose) dumpBuffer();
		// we need at least 3 more to replace the first two...
		m_leftToRead = 2;
		return kLMSNoTelegram;
	} else if (totalLength > totalRead ) {
		//logprintf("Not enough read yet.  Need %lu, only have %i\n", totalLength, totalRead);
		m_leftToRead = totalLength - totalRead;
		// not enough message yet.
		return kLMSNoTelegram;
	}
	
	// sanity check the crc
	ushort_t expected_crc = createCRC((unsigned char *)m_inBuffer, MESSAGE_HEADER_SIZE + messageLength);
	ushort_t actual_crc = *((ushort_t *)(messagePtr + messageLength));
	
	if (expected_crc != actual_crc) {
		logprintf("Invalid checksum on message (expected: %04x actual: %04x).  Disgarding.\n",
			expected_crc, actual_crc);
		if (m_verbose) dumpBuffer();
		m_inReadPtr += totalLength;
		return kLMSNoTelegram;
	}
	unsigned char messagetype = parseDataTelegram(messagePtr, messageLength)	;			// valid message received, parse it
	// we've read this message, use up its space in the buffer
	m_inReadPtr += totalLength;
	return(messagetype);
}	
//
//	parseDataTelegram  -- parse one valid data telegram from the SICK LMS
//
//	This gets the portion of the data telegram after the header up to, but not including, the CRC
//	
unsigned char LidarServer_::parseDataTelegram(char* messageBuf, size_t messageLength)
{	
	char* messagePtr = messageBuf;											// working local pointer used during parse
	// actually start parsing the message
	unsigned char messageCmd = read_char(messagePtr);		// get message-type byte
	
	// we have the entire message, now handle it.
	switch(messageCmd) {
		case kLMSPowerOnTelegram:
			char temp[100];
			memset(temp, 0, 100);
			snprintf(temp, (100 <= messageLength)?100:(messageLength - 2), "%s", messagePtr);
			messagePtr += (messageLength-2);							// advance past message
			logprintf("Received power-on message: %s.\n", temp);
			m_msgin.tryput(messageCmd);										// so send side can tell we got it
			break;
			
		case kLMSConfirmSoftwareResetTelegram:
			logprintf("Received software-reset confirmed message.\n");
			m_msgin.tryput(messageCmd);										// so send side can tell we got it
			break;
			
		case kLMSNotAcknowledgedTelegram:
			logprintf("Received NAK message.\n");
			m_msgin.tryput(messageCmd);										// so send side can tell we got it
			break;
			
		case kLMSRespondChangeOperatingModeTelegram:
			logprintf("Received Operating-Mode change response message.\n");
			m_msgin.tryput(messageCmd);										// so send side can tell we got it
			break;
			
		case kLMSRespondValuesTelegram:
			if (!m_collectingData) break;										// ignore if not in collecting mode, during reset
			m_receivingData = true;											// we are receiving valid data

			{	// 	Send scan line to server program that wants it.
				LidarServerMsgLISN scanmsg;									// build scan line to send as a message
				scanmsg.m_msgtype = LidarServerMsgLISN::k_msgtype;
				bool good = handleResponseValuesTelegram(messagePtr, scanmsg.m_data);	// build reply msg
				//	Check for dirt on scanner, and trigger a wash cycle if needed
				const uint8_t k_pollution_status = (1<<7);				// bit 7 means "pollution"
				if (scanmsg.m_data.m_header.m_statusByte & k_pollution_status)	// if "scanner pollution
				{	if (!m_wash.washing())											// if not already washing
					{	logprintf("LMS window dirty. Requesting wash cycle.\n");
						m_wash.requestWash();									// trigger a wash cycle
					}
					break;																	// washing, do not process line
				}					
				if (!good) break;														// rejected line, do not use
#ifdef OBSOLETE
				int sink;																		// there's no real reply to this
				int err = m_dataclientport->MsgSend(scanmsg,sink);
				if (err < 0)																// if trouble
				{	logprintf("Unable to send scan line to server: %s\n",strerror(errno));	// server must do the recovery
				}
#endif // OBSOLETE
				bool queued = m_outputqueue.tryput(scanmsg);		// queue for sending, does not block
				if (!queued)																// drop if not queued
				{	logprintf("Unable to send scan line to server - queue full.\n");	}
			}
			break;
			
		case kLMSRespondSensorStatusTelgram:
			logprintf("Received sensor status message.\n");
			m_msgin.tryput(messageCmd);										// so send side can tell we got it
			break;
			
		case kLMSRespondCurrentLMSConfiguration:
			logprintf("Got current configuration:\n");
			
			// dump the config to the log
			logprintf("Blanking: 0x%02x ", read_char(messagePtr));
			logprintf("PeakThreshold: 0x%02x ", read_char(messagePtr));
			logprintf("StopThreshold: 0x%02x ", read_char(messagePtr));
			logprintf("Availability: 0x%02x ", read_char(messagePtr));
			logprintf("MeasurementMode: 0x%02x ", read_char(messagePtr));
			logprintf("Units: %02x ", read_char(messagePtr));
			logprintf("TemporaryField: %i ", read_char(messagePtr));
			logprintf("SubtractiveFields: 0x%02x ", read_char(messagePtr));
			logprintf("MultipleEvaluation: %i \n", read_char(messagePtr));
			m_msgin.tryput(messageCmd);										// so send side can tell we got it
			break;
			
		case kLMSRespondDefineLMSConfiguration:
			logprintf("LMS configuration change acknowledged.\n");
			m_msgin.tryput(messageCmd);										// so send side can tell we got it
			break;
			
			
		default:
			logprintf("Received unhandled message of type 0x%02x.\n", messageCmd);
	}
	//	Check message length, to insure we didn't process a short message incorrectly.
	 if (messagePtr > (messageBuf + messageLength)) {
		logprintf("Error: Handled %i bytes of the %u total message bytes.\n",
			messagePtr - messageBuf, messageLength);
	}	
	return messageCmd;
}
//
//	Buffer management
//
//	Painful
//
void LidarServer_::adjustPointers() {
	int size = m_inWritePtr - m_inReadPtr;
	
	if (size < 0) {
		logprintf("Connection Buffer is corrupt!!!  Data Size returned: %i\n", size);
		exit(1);
	}
	
	if (size == 0) {
		m_inWritePtr = m_inReadPtr = m_inBuffer;
	} else if (size < BUFFER_SIZE)
	{	//	Move data down to beginning of buffer, in place.
		memmove(m_inBuffer, m_inReadPtr, size);
		m_inWritePtr = m_inBuffer + size;
		m_inReadPtr = m_inBuffer;
	} else if (size > BUFFER_SIZE) {
		logprintf("Connection Buffer is corrupt!!!  Data Size returned: %i  Buffer only: %i\n", size, BUFFER_SIZE);
		exit(1);
	} else {
		logprintf("Buffer full!\n");
		// FIX - we should probably detect if this is repeated, and abort
	}
	// otherwise we're full, can't clear anything.
}
//
//	resetPointers  --  clear input buffer
//
void LidarServer_::resetPointers()
{
	m_inWritePtr = m_inReadPtr = m_inBuffer;						// resets buffer to empty
	m_leftToRead = MESSAGE_HEADER_SIZE;							// prepare to read a header
}



/***
 * Outgoing messages
 */

inline void write_char(char* &messagePtr, unsigned char value) {
	*messagePtr = value;
	messagePtr += 1;
}

inline void write_short(char* &messagePtr, unsigned short value) {
	*((unsigned short *)(messagePtr)) = value;
	messagePtr += 2;
}

int LidarServer_::readLMSConfiguration() {
	return(sendSimpleTelegram(kLMSReadLMSConfiguration));
}

int LidarServer_::sendLMSConfiguration() {
	char msg[kLMSMaximumTelegramLength];
	
	char *messagePtr = msg;
	
	write_char(messagePtr, kLMSTelegramSTX);
	write_char(messagePtr, kLMSTelegramAllDevicesAddress);
	// ignore length for now...
	messagePtr += 2;
	
	// set the type.
	write_char(messagePtr, kLMSDefineLMSConfiguration);

	// set pretty much the defaults.
	write_char(messagePtr, kLMSDefaultBlanking);
	write_char(messagePtr, kLMSDefaultPeakThreshold);
	write_char(messagePtr, kLMSDefaultStopThreshold);
	//write_char(messagePtr, kLMSDefaultAvailablity);
	write_char(messagePtr, kLMSSendRealtimeIndicies);	// ***DO WE WANT TO ADD HIGH AVAILABILTY?*** Test this
	write_char(messagePtr, kLMSDefaultMeasurementMode);
	write_char(messagePtr, kLMSDefaultUnits);
	write_char(messagePtr, kLMSDefaultTemporaryField);
	write_char(messagePtr, kLMSDefaultSubtractiveFields);
	write_char(messagePtr, kLMSDefaultMultipleEvaluation);
	
	char *temp = msg + 2;
	// write the length
	write_short(temp, messagePtr - msg - MESSAGE_HEADER_SIZE);
	
	// calculate and write the CRC
	write_short(messagePtr, createCRC((const unsigned char *)msg, messagePtr - msg));
			 
	return(m_lidarsocket.write_data(msg, messagePtr - msg));
}
//	
//	requestEmptyMessage -- send a maximum message length block of zeroes
//
//	This flushes any junk in the pipe and gets the LMS listening.
//	There's no STX in this data, which is all zeroes, and it's longer
//	than the longest possible message, so this should get the
//	LMS back into look-for-sync mode.
//
int LidarServer_::requestEmptyMessage()
{	char msg[kLMSMaximumTelegramLength];
	bzero(msg,sizeof(msg));																	// zero fill
	return(m_lidarsocket.write_data(msg, sizeof(msg)));							// send fill blind
}

//
//	requestReset  -- request an LMS reset
//
int LidarServer_::requestReset()
{
	return(sendSimpleTelegram(kLMSInitializationTelegram));					// send a reset message
}

//
//	requestSwitchToMode  -- request switch device to requested mode
//
int LidarServer_::requestSwitchToMode(unsigned char mode)
{
	char msg[kLMSMaximumTelegramLength];
	
	char *messagePtr = msg;
	
	write_char(messagePtr, kLMSTelegramSTX);
	write_char(messagePtr, kLMSTelegramAllDevicesAddress);
	// ignore length for now...
	messagePtr += 2;
	
	write_char(messagePtr, kLMSChangeOperatingModeTelegram);
	
	write_char(messagePtr, mode);
	
	switch(mode) {
		case kLMSInstallationMode:
			strncpy(messagePtr, PASSWORD, PASSWORD_LEN);
			messagePtr += PASSWORD_LEN;
			break;
		case kLMSContinuousMinimumValuesMode:
		case kLMSContinuousAllValuesMode:
		case kLMSContinuousPartialValuesMode:			// 1 degree apart, 75Hz.
			break; // nothing more needed.
		case kLMSConfigTo500000Baud:
		case kLMSConfigTo38400Baud:
		case kLMSConfigTo19200Baud:
		case kLMSConfigTo9600Baud:
			break; // nothing more needed for baud change.
		default:
			logprintf("Mode 0x%02x currently not supported!\n", mode);
			fflush(stdout);
	}
	
	char *temp = msg + 2;
	// write the length
	write_short(temp, messagePtr - msg - MESSAGE_HEADER_SIZE);
	
	// calculate and write the CRC
	write_short(messagePtr, createCRC((const unsigned char *)msg, messagePtr - msg));
	if (m_verbose)
	{	logprintf("Raw buffer data (before write) (%p):\n", msg);
		for (char *ptr = msg; ptr < messagePtr; ptr++) {
			logprintf("%02x ", *ptr);
		}
		logprintf("\n");
	}
	int ret = m_lidarsocket.write_data(msg, messagePtr - msg);
	return(ret);
}

int LidarServer_::requestSensorStatus() {
	return(sendSimpleTelegram(kLMSRequestStatusTelegram));
}

int LidarServer_::requestErrorTelegram() {
	return(sendSimpleTelegram(kLMSRequestErrorTelegram));
}

int LidarServer_::sendSimpleTelegram(unsigned char type)
{
	char msg[kLMSMaximumTelegramLength];
	
	char *messagePtr = msg;
	
	write_char(messagePtr, kLMSTelegramSTX);
	write_char(messagePtr, kLMSTelegramAllDevicesAddress);
	// ignore length for now...
	messagePtr += 2;
	
	write_char(messagePtr, type);

	char *temp = msg + 2;
	// write the length
	write_short(temp, messagePtr - msg - MESSAGE_HEADER_SIZE);
	
	// calculate and write the CRC
	write_short(messagePtr, createCRC((const unsigned char *)msg, messagePtr - msg));
			 
	int ret = m_lidarsocket.write_data(msg, messagePtr - msg);
	return(ret);
}
//
//	waitForReply  -- wait for a specific message type, or a timeout
//
//	A request with a timeout of 0 flushes the incoming message type queue.
//
bool LidarServer_::waitForReply(LMSIncomingTelegrams msgtype1, LMSIncomingTelegrams msgtype2,
	double timeout, uint8_t& reply)
{	double endtime = gettimenow() + timeout;					// done when this time passes
	for (;;)
	{	uint8_t inmsgtype = 0;												// incoming message type
		if (m_msgin.get(inmsgtype, timeout))						// wait for reply
		{	
			reply = inmsgtype;												// return type
			if (inmsgtype == msgtype1) return(true);				// success
			if (inmsgtype == msgtype2) return(true);				// success
		}
		if (gettimenow() > endtime) 
		{	
			return(false);														// fails, timeout
		}
	}
	//	Unreachable
}
//
//	gettiltattime  -- get tilt at specific time
//
//	Tiilt data is requested on every scan line.  But the scan line data is earlier than
//	the tilt data, so we have to save and interpolate tilt data. Without this, we get banding
//	in the scan lines, as bunches of scan lines come in together
//
//	We subtract a constant from desiredtime based on measurements from the real system.
//	This reflects the propagation delay from the LIDAR through the serial link through
//	the network.  It's about three scan line times.
//
//	Occasionally, due to processing delays, this adjustment gets slightly off. If it's not off by more than
//	a few extra scan line times, we keep the scan line, to avoid holes in the LIDAR data which cause
//	the vehicle to stop.
//
bool LidarServer_::gettiltattime(uint64_t desiredtime, float& tilt)
{
	const uint64_t k_scan_line_delay = (1000000000/75) * 3;	// 3 * 1/75 sec, in ns.
	const uint64_t k_max_allowed_early_scan_line_time = (1000000000/75) * 2;	// max allowed tilt time jitter
	desiredtime -= k_scan_line_delay;								// actual LIDAR time is earlier
	//	Add new tilt to recent tilt history, for interpolation.
	float currenttilt = m_tilt.getTilt();										// get tilt. This queries controller synchronously.
	uint64_t now = gettimenowns();									// get time AFTER getting tilt
	m_tiltpositions.addPose(currenttilt, now);						// get tilt at this time
	//	Interpolate tilt
	bool toolate;																	// should never be too late
	TiltPosition pos;
	bool good = m_tiltpositions.getposeattime(pos, desiredtime, toolate);	// interpolate from history
	if (!good)																		// if interpolation failed
	{	if (!toolate)																// LMS data should always be earlier than tilt data.
		{	uint64_t tdiff = desiredtime - now;						// difference in ns
			double secs = (tdiff*0.000000001);						// difference in seconds
			logprintf("Tilt data (latest %lld ns) %1.5fs earlier than LMS data. PLL bug?.\n", now, secs);
			//	Try to work around bad data, if not too bad
			if (tdiff < k_max_allowed_early_scan_line_time)	// if within tolerable limits
			{	good = m_tiltpositions.getposeattime(pos, now, toolate);	// use current tilt
				if (good) return(true);										// accept it anyway
			}
			//	Workaround failed. So note.
			logprintf("Tilt data (latest %lld ns) %1.5fs much earlier than LMS data. Scanline rejected.\n", now, secs);
		}
		return(false);															// fails
	}
	tilt = pos.m_tilt;																// extract tilt
	return(true);																	// return success
}
//
//	Output queue -- reads from queue, sends to MAP server.
//
//	This prevents delays in the map server from stalling the operation.
//
//	We could cut two copying operations out of this if we had to get the performance.
//
void* LidarServer_::outputThread()
{	for (;;)
	{	
		LidarServerMsgLISN scanmsg;											// msg to send
		m_outputqueue.get(scanmsg);											// get a message to send
		int err = m_dataclientport->MsgSend(scanmsg);
		if (err < 0)																		// if trouble
		{	logprintf("Unable to send scan line to server: %s\n",strerror(errno));	// server must do the recovery
		}
	}
}