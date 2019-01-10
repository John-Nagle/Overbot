/* -*- indent-tabs-mode:T; c-basic-offset:8; tab-width:8; -*- vi: set ts=8:
 *
 * Frank Zhang, 3/30/05, major additions to support fusednav approach
 * Khian Hao Lim
 * Oct 03
 * Overbot
 * Revised 2005, Dwayne Hull
 *
 * Object that encapsulates all the data from the GPS
 */
#include "GPS.h"
#include <sys/types.h>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include "thread.h"
#include "gpsins_messaging.h"
#include "fusednav.h"
#include "logprint.h"
#include "tuneable.h"	
//
// constants
//
const Tuneable k_gps_sample_period("GPSSAMPLEPERIOD", 0.05, 1.0, 0.10, "GPS sample period in seconds.");	



GPS::GPS(char* gpsDev,char* gpsLog ):
	mDev(gpsDev),
	mLogFd(0),
	mBytesRead(0),
	mBytesToRead(1),
	mXYZsd(0,0,0),
	mXYZVel(0,0,0),
	mXYZVelsd(0,0,0),
	mLLHZero(0,0,0),
	mXYZZero(0,0,0),
	mXYZ(0,0,0),
	mLLH(0,0,0),
	mNED(0,0,0),
	mNEDsd(0,0,0),
	mNEDVel(0,0,0),
	mNEDVelsd(0,0,0),
	mAbs(0,0,0),
	mLastSampleTime(0),
	mHeading(0),
	mNEDAccumulate(0,0,0),
	mGPSNoMoveSampleCntr(0),
	mVel(0,0,0)

{
	// Default workshop coordinates outside overhead door
	mXYZZero[0] = -2701613.1890054550021886;
	mXYZZero[1] = -4287886.4690471673384308;
	mXYZZero[2] = 3859437.4093420552089810;

	mLLHZero = ECEF2llh(mXYZZero);
	
		if (gpsLog) {
		mLogFd = open( gpsLog, O_WRONLY | O_CREAT, 0666 );

		if( mLogFd < 0  && verbose) 
		{
				perror( gpsLog );
				//	***NEED TO TAKE ERROR ACTION***
		}
	}
		mSerial = new Serial(gpsDev, 57600, "8N1", 999999);
	if (mSerial->Open() < 0) {
		cout << "Failed to open 57600 serial" << endl;
		//	***NEED TO TAKE ERROR ACTION***
		throw("Unable to open serial port");	// ***TEMP***
	}

}

GPS::~GPS()
{
}
//
//	setBasePoint -- set basepoint of tangent plane coordinate system used in MAP.
//
//	Input is in degrees.
//
void GPS::setBasePoint(Vector<3> bPointllh)
{
	logprintf("GPS::setBasePoint called with %4.6f  %4.6f  %4.6f\n",bPointllh[0],bPointllh[1],bPointllh[2]);	// print in degrees	
	ost::MutexLock lok(mLock);
	bPointllh[0] = (M_PI/180) * bPointllh[0];						// convert latitude to radians
	bPointllh[1] = (M_PI/180) * bPointllh[1];						// convert longitude to radians
	mLLHZero = bPointllh;												// set zero of lat, long system
	mXYZZero = llh2ECEF(mLLHZero);							        // set zero of tangent plane
	mKF->mFusedNav->mNavMode = NAV_INIT;
	
}

void
GPS::log(char* line, int len)
{
	if (mLogFd) {
        uint64_t  t;
	    t = gettimenowns();
		write(mLogFd, (char*)&t, sizeof(t));
		write(mLogFd, line, len);
	}
		
}

/* Some functions to handle check sum */

#define CRC32_POLYNOMIAL 0xEDB88320L

/* --------------------------------------------------------------------------
Calculate a CRC value to be used by CRC calculation functions.
-------------------------------------------------------------------------- */
unsigned long CRC32Value(int i)
{
    int j;
    unsigned long ulCRC;
    ulCRC = i;
    for ( j = 8 ; j > 0; j-- ) {
        if ( ulCRC & 1 )
            ulCRC = ( ulCRC >> 1 ) ^ CRC32_POLYNOMIAL;
        else ulCRC >>= 1;
    }
    return ulCRC;
}

/* --------------------------------------------------------------------------
Calculates the CRC-32 of a block of data all at once
-------------------------------------------------------------------------- */
 unsigned long
CalculateBlockCRC32( unsigned long ulCount, /* Number of bytes in the data block */
                     unsigned char *ucBuffer ) /* Data block */
{
    unsigned long ulTemp1;
    unsigned long ulTemp2;
    unsigned long ulCRC = 0;
    while ( ulCount-- != 0 ) {
        ulTemp1 = ( ulCRC >> 8 ) & 0x00FFFFFFL;
        ulTemp2 = CRC32Value( ((int) ulCRC ^ *ucBuffer++ ) & 0xff );
        ulCRC = ulTemp1 ^ ulTemp2;
    }
    return( ulCRC );
}

 // method to read the GPS data from serial port. Frank Zhang, 2/8/05
 void GPS::readGPSData()
 {
	//	***Note undesirable pointer arithmetic below. Check buffer size.***
	if (mBytesRead + mBytesToRead > k_gpsmaxbuf)	  // if buffer overflow
	{	logprintf("GPS packet too big! mBytesRead: %d mBytesToRead %d \n",mBytesRead, mBytesToRead );
		mBytesRead = 0;				  // reset state
		mBytesToRead = 1;
	}
	assert(mBytesRead >= 0);			  // avoid wrap
	assert(mBytesRead + mBytesToRead <= k_gpsmaxbuf); // must fit
	int ret = mSerial->ReadBuf(mBuf + mBytesRead, mBytesToRead - mBytesRead);
	if (ret <= 0) {					  // error in read. This includes select timeout
		logprintf("GPS Read error: %d\n",ret);			  // force buffer reset
		mBytesRead = 0;
		mBytesToRead = 1;
		return;					  // try again on next step
	}
	mBytesRead += ret;
	
	if (mBytesRead != mBytesToRead) {
		return; //not ready yet;
	}
	
	if (mBytesRead == 1) { //header
		if ((unsigned char)mBuf[0] != 170) { //first sync
			mBytesRead = 0;
			mBytesToRead = 1;
		}
		else {
			mBytesToRead = 2;
		}
		return;
	}
	
	if (mBytesRead == 2) {
		if ((unsigned char)mBuf[1] != 68) { //second sync
			logprintf("GPS: Didn't see second sync char in GPS packet.\n");
			mBytesRead = 0;
			mBytesToRead = 1;
			return;
		}		
		else {
			mBytesToRead = 3;
		}
		return;
	}
	
	if (mBytesRead == 3) {
		if ((unsigned char)mBuf[2] != 18) { //third sync
			logprintf("GPS: Didn't see third sync char in GPS packet.\n");
			mBytesRead = 0;
			mBytesToRead = 1;
		}
		else {
			mBytesToRead = 3 + 25; //header
		}
		return;
	}
	
	if (mBytesRead == 3 + 25) { //got header
		mMsgId = *((unsigned short *)(mBuf+3+1));
		mMsgLen = *((unsigned short *)(mBuf+3+5));
		mBytesToRead = 3 + 25 + mMsgLen + 4;
		return;		
	}
	
	if (mBytesRead == 3 + 25 + (unsigned int)mMsgLen + 4) { //got the whole packet {
		if (CalculateBlockCRC32(3 + 25 + mMsgLen + 4, (unsigned char*)mBuf)) {
			logprintf("CRC error in GPS Packet.\n");
		}
		else {
			handleData();	// process newly received data
		}
		mBytesRead = 0;		// reset for next read
		mBytesToRead = 1;
		return;
	}
 
} // end of  readGPSData()


 void GPS::gpsStart()
 {
 	logprintf("Initializing Novatel GPS Receiver\n");
	char * unlogallMsg = "UNLOGALL\n";
	mSerial->WriteBuf(unlogallMsg, strlen(unlogallMsg));
	sleep(1);
	int ret = mSerial->ReadBuf(mBuf, k_gpsmaxbuf);
	if (ret < 3)
	{
		logprintf("Invalid Novatel response to unlogall command.\n");
		logprintf("Check wiring and/or comm settings on Novatel receiver.\n");
		logprintf("The comm settings expected are 57600,N,8,1,N,OFF,ON\n");
		sleep(1);
		exit (FUSEDNAV_GPS_COM_FAILURE);
	}
	else
	{
		logprintf("Novatel serial communications ok\n");
	}
	
		
	// There is a very good chance that there is some left over stuff
	// if any logs were active when the unlog was issued. We let the
	// readGPSData() method deal with sorting this out rather than attemping
	// to drain it here. 
	
	//command to do waas correction
	//char * cmdwaas = "waascorrection enable 0 waastestmode\n";
	//mSerial->WriteBuf(cmdwaas, strlen(cmdwaas));
	//	***TEMP*** turn off some GPS receivers to check CPU load problem in Novatel
	const int k_rcvturnoff = 0;							// turn off this many receivers. Set to 0 for none.
	for (int i=0; i<k_rcvturnoff; i++)
	{	char s[100];	
		snprintf(s,sizeof(s),"assign %d idle\n", i);
		mSerial->WriteBuf(s,strlen(s));
	}
	
	//request for bestxyzb
	char logbestxyzb[80];
	double samplePeriod = k_gps_sample_period;
	sprintf(logbestxyzb,"log bestxyzb ontime %1.2f\n", samplePeriod);
	mSerial->WriteBuf(&logbestxyzb,strlen(logbestxyzb));
	
	//	request for range (debug)
	//char * logrange = "log rangeb ontime 10\n";
	//mSerial->WriteBuf(logrange, strlen(logrange));
 
	//	request for lbandstat (debug)
	//char * lbandstat = "log lbandstatb ontime 1\n";
	//mSerial->WriteBuf(lbandstat, strlen(lbandstat));

	//	request for ephemeris (debug)
	//char * ephemeris = "log rawephemb ontime 10\n";
	//mSerial->WriteBuf(ephemeris, strlen(ephemeris));

 	//request for satvisb
	//char* logsatvisb = " log satvisb ontime 10\n";
	//mSerial->WriteBuf(logsatvisb, strlen(logsatvisb));
	//mSerial->	();	
	
	pthread_t gpsio_thread_id;
	pthread_create(&gpsio_thread_id, 0, gpsio_threadstart, (void*)this);
 }

 void * GPS::gpsio_thread()
 {
 	// run forever to receive gps data via serial port
	while (true) {
		readGPSData();
	}
 } // end of gpsio_thread()
 

bool
GPS::doCheckSum(char * data, unsigned short msglen, char * checksum)
{
	unsigned long c = 
		CalculateBlockCRC32((unsigned long)msglen, (unsigned char*) data);
	return c == *((unsigned long *) checksum);
}

void
GPS::handleBestXYZ()
{
	ost::MutexLock lok(mLock);
	//	Read additional fields from header
	uint32_t hdrstatus = *((uint32_t*) (mBuf+20));	// per Novatel manual rev 14 vol 2 table 4 page 17
	logHeaderStatus(hdrstatus);			// log header status
	//	End additional header processing
	char * data = mBuf + 3 + 25;
	int posStat = *((int*)(data));    
	mPosStat = (GPSINS_MSG::SolStatusEnum)posStat;   
	int posType = *((int*)(data+4));	 
	mPosType = (GPSINS_MSG::PosVelTypeEnum)posType; 
    switch (mPosType)
    {
	case GPSINS_MSG::OMNISTAR_HP:
	case GPSINS_MSG::OMNISTAR:
	case GPSINS_MSG::WAAS:
	case GPSINS_MSG::PSRDIFF:
	case GPSINS_MSG::SINGLE:
	{
		// with good position data, update GPS object
		mXYZ[0]  = *((double*)(data+8));     
		mXYZ[1]  = *((double*)(data+16));	 
		mXYZ[2]  = *((double*)(data+24)); 	 
		mXYZsd[0]= *((float*)(data+32));
		mXYZsd[1]= *((float*)(data+36));
		mXYZsd[2]= *((float*)(data+40));
		int velStat = *((int*)(data+44));	
		mVelStat = (GPSINS_MSG::SolStatusEnum)velStat;	
		int velType = *((int*)(data+48));
		mVelType = (GPSINS_MSG::PosVelTypeEnum)velType; 
		mXYZVel[0]  = *((double*)(data+52));
		mXYZVel[1]  = *((double*)(data+60));
		mXYZVel[2]  = *((double*)(data+68));
		mXYZVelsd[0]= *((float*)(data+76));
		mXYZVelsd[1]= *((float*)(data+80));
		mXYZVelsd[2]= *((float*)(data+84));
		memcpy(mStnID,data+88,4);
		mVelLat  = *((float*)(data+92));	
		mDiffAge = *((float*)(data+96));	
		mSolAge	 = *((float*)(data+100));	
		mNumObs	 = *((unsigned char*)(data+104));
		mNumGPSL1= *((unsigned char*)(data+105));
		mNumL1	 = *((unsigned char*)(data+106));
		mNumL2	 = *((unsigned char*)(data+107));
	                
                mLLH = ECEF2llh(mXYZ);
	        mLastSampleTime = gettimenowns();// timestamp of GPS update
	        
		//Convert to NED
		mNED = ECEF_dist(mXYZ, mXYZZero, mLLHZero[0], mLLHZero[1]);
		mNEDsd = ECEF_dist(mXYZsd + mXYZZero, mXYZZero, mLLHZero[0], mLLHZero[1]);
		mNEDVel = ECEF_dist(mXYZVel + mXYZZero, mXYZZero, mLLHZero[0], mLLHZero[1]);
		mNEDVelsd = ECEF_dist(mXYZVelsd + mXYZZero, mXYZZero, mLLHZero[0], mLLHZero[1]);
		mAbs = mNED;                
		if (!mKF->mFusedNav->mMoving)
		{
			mVel=Vector<3>(0.0, 0.0, 0.0);
			mHeading = 0;
		}
		else
		{
			// Vehicle is moving
			mAbs = mNED;
			mVel = mNEDVel;
			calculateHeading(); // Sets mHeading
		}
		mKF->mFusedNav->fuseGPS();
		break;
    }
	case GPSINS_MSG::NONE:
    {
		mKF->mFusedNav->fuseGPS(); // A call is required even if the GPS pos data isn't updated  
		break;
    }
    default:
		if (verbose)
		{
			logprintf("Unhandled GPS position type. (type %d)\n", mPosType); 
		}

		break;
    } // end of switch (mPosType)
} // end of GPS::handleBestXYZ()

// Calculate the heading in degrees based on GPS velocity
// input: mNEDVel
// output: mHeading
void
GPS::calculateHeading()
{
	// Don't update heading unless moving N/S or E/W
	if((mNEDVel[0] == 0) && (mNEDVel[1] == 0))
	{
		return;
	}
	
	if(mNEDVel[0] == 0)
	{
		if(mNEDVel[1] >0)
		{ 
			mHeading = 90;
		}
		else if (mNEDVel[1] < 0)
		{
			mHeading = 270;
		}
		return;
	}
	else if(mNEDVel[1] == 0)
	{
		if(mNEDVel[0] >0)
		{ 
			mHeading = 0;
		}
		else if (mNEDVel[0] < 0)
		{
			mHeading = 180;
		}
		return;
	}

	double angleRadians = 	atan(abs(mNEDVel[1]) / abs(mNEDVel[0])); // Returns 0 to + PI/2
	// Correct heading to 0 to 2PI radians
	if(mNEDVel[0] > 0)
	{
		if(mNEDVel[1] < 0) // If true, heading NW else heading NE (no correction needed)
		{
			angleRadians = 2.0 * M_PI - angleRadians;
		}
	}
	else
	{
		if(mNEDVel[1] < 0) // if true, heading SW
		{
			angleRadians = M_PI + angleRadians;
		}
		else
		{
			angleRadians = M_PI - angleRadians;
		}
	}
	// Result in degrees (0 <= result < 360)
	mHeading = 180. * angleRadians / M_PI;
		
} // End of calculateHeading()

void
GPS::handleOmniStat()
{
	char * data = mBuf + 3 + 25;
	
	//Reserve field mentioned on page 174 of log reference is missing
	//all fields after it shifted back by 4 bytes
	mFreq = *((unsigned long*)data);
	mCN0  = *((float*)(data+4));
	mLockTime = *((float*)(data+8));
	memcpy(mTracking, data+12, 2);
	memcpy(mVBStat, data+16, 2);
	mNumBytesVBS = *((unsigned long*)(data+20));
	mNumGoodDGPS = *((unsigned long*)(data+24));
	mBadData = *((unsigned long*) (data+28));
	memcpy(mHPStat1, data+32, 2);
	memcpy(mHPStat2, data+34, 2);
	mNumBytesHP = *((unsigned long*) (data+36));
		
	if (verbose) {

	cout << "handleOmnistat mFreq: " << mFreq
	<< " mCNO " << mCN0
	<< " mLockTime " << mLockTime
	<< " mTracking " << mTracking
	<< " mVBStat " << mVBStat
	<< " mNumBytesVBS " << mNumBytesVBS
	<< " mNumGoodDGPS " << mNumGoodDGPS
	<< " mBadData " << mBadData
	<< " mHPStat1 " << mHPStat1
	<< " mHPStat2 " << mHPStat2
	<< " mNumBytesHP " << mNumBytesHP
	<< endl;
	
	}
}  // end of GPS::handleOmniStat()

void
GPS::handleData()
{
	ost::MutexLock lok(mLock);
	//	Fan out on message type
	if (verbose)
	{	logprintf("GPS message received, type %d\n", mMsgId);	}	
	switch (mMsgId) {
	case BESTXYZ_ID:
		handleBestXYZ();
		break;
	
	case OMNISTAT_ID:
		handleOmniStat();
		break;
		
	case LBANDSTAT_ID:			// Omnistar diagnostic output
		handleLbandStat();
		break;
	
	case RANGE_ID:				// Another diagnostic output
		handleRange();
		break;
		
	case RAWEPHEMB_ID:			// Ephemeris changed
		handleRawEphemeris();
		break;
		
	default: 																	// unknown type
		logprintf("Unknown message type from GPS: %d\n", mMsgId);
		break;
	}
	log(mBuf, 3 + 25 + mMsgLen + 4);	
	
}
//
//	Diagnostic logging for GPS problems
//
void GPS::handleLbandStat()
{
	const char *data = mBuf + 3 + 25;
	float cn0 = *((float*)(data+4));
	float locktime = *((float*)(data+8)); 
	uint16_t tracking = *((uint16_t*)(data+16)); 
	uint16_t status = *((uint16_t*)(data+20)); 
	uint32_t gooddgps = *((uint32_t*)(data+28)); 
	uint32_t baddgps = *((uint32_t*)(data+32)); 
	uint16_t hpstatus1 = *((uint16_t*)(data+36)); 
	uint16_t hpstatus2 = *((uint16_t*)(data+40)); 
	uint32_t hpbytes = *((uint32_t*)(data+44)); 
	logprintf("LBANDSTAT: c/No %1.6f  locktime %1.6f  tracking 0x%04x  status 0x%04x  gooddgps %d  baddgps %d \n",
		cn0, locktime, tracking, status, gooddgps, baddgps);
	logprintf("LBANDSTAT: hpstatus1 0x%04x  hpstatus2 0x%04x  hpbytes %d\n",
		hpstatus1, hpstatus2, hpbytes);
}
void GPS::handleRange()
{

}

void GPS::handleRawEphemeris()
{
	logprintf("GPS ephemeris change.\n");
}

void GPS::logHeaderStatus(uint32_t errs)
{

}
