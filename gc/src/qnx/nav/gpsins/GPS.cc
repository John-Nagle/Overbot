/* -*- indent-tabs-mode:T; c-basic-offset:8; tab-width:8; -*- vi: set ts=8:
 *
 * Khian Hao Lim
 * Oct 03
 * Overbot
 *
 * Object that encapsulates all the data from the GPS
 */
#include <GPS.h>
#include <sys/types.h>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include "thread.h"

GPS::GPS(char* gpsDev,char* gpsLog ):
	mDev(gpsDev),
	mLogFd(0),
	mBytesRead(0),
	mBytesToRead(1),
	mNumSamples(0),
	mXYZ(0,0,0),
	mXYZsd(0,0,0),
	mXYZVel(0,0,0),
	mXYZVelsd(0,0,0),
	mLLHZero(0,0,0),
	mXYZZero(0,0,0),
	mNED(0,0,0),
	mNEDsd(0,0,0),
	mNEDVel(0,0,0),
	mNEDVelsd(0,0,0)
{
	//	***MUST BE FIXED - the initial position must be obtained from the waypoints. ***
	//taken directly from the gps reading outside workshop
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
	
	//now we set the serial to be faster
	mSerial = new Serial(gpsDev, 57600, "8N1", 999999);
	if (mSerial->Open() < 0) {
		cout << "Failed to open 57600 serial" << endl;
		//	***NEED TO TAKE ERROR ACTION***
		throw("Unable to open GPS serial port");				// JN
	}
	
	char * unlogallMsg = "unlogall\n";
	mSerial->WriteBuf(unlogallMsg, strlen(unlogallMsg));
	
	//command to do waas correction
	char * cmdwaas = "waascorrection enable 0 waastestmode\n";
	mSerial->WriteBuf(cmdwaas, strlen(cmdwaas));
	
	//request for bestxyzb
	char * logbestxyzb = "log bestxyzb ontime 0.05\n";
	mSerial->WriteBuf(logbestxyzb,strlen(logbestxyzb));
	
	//request for omnistat
	char * logomnistatb = "log omnistatb ontime 1\n";
	mSerial->WriteBuf(logomnistatb, strlen(logomnistatb));
	
	//request for satvisb
	//char * logsatvisb = " log satvisb ontime 10\n";
	//mSerial->WriteBuf(logsatvisb, strlen(logsatvisb));
	
	//mSerial->	();	
	gettimeofday(&mStartTime, 0);
	
}

GPS::~GPS()
{
}

void
GPS::log(char* line, int len)
{
	if (mLogFd) {
		struct timeval t;
		gettimeofday(&t, 0);
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
//
//	step  -- get more data from GPS
//
//	Attempts to acquire a full message from the GPS unit, which is then
//	processed.
//
//	Message format expected is
//		170
//		68
//		some fixed length header bytes
//		2 bytes of length.
//		more header bytes
//		variable length part
//
void
GPS::step()
{
	//	***Note undesirable pointer arithmetic below. Check buffer size.***
	if (mBytesRead + mBytesToRead > k_gpsmaxbuf)		// if buffer overflow
	{	printf("GPS packet too big.\n");
		mBytesRead = 0;						// reset state
		mBytesToRead = 1;
	}
	assert(mBytesRead >= 0);			// avoid wrap
	assert(mBytesRead + mBytesToRead <= k_gpsmaxbuf);		// must fit
	int ret = mSerial->ReadBuf(mBuf + mBytesRead, mBytesToRead - mBytesRead);
	if (ret <= 0) {								// error in read. This includes select timeout
		perror("GPS:: Read");					// force buffer reset
		mBytesRead = 0;
		mBytesToRead = 1;
		return;										// try again on next step
	}
	mBytesRead+= ret;
	
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
			if (verbose)
				cout << "Second sync wrong" << endl;
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
			if (verbose)
				cout << "Third sync wrong" << endl;
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
	
	if (mBytesRead == 3 + 25 + mMsgLen + 4) { //got the whole packet {
		if (CalculateBlockCRC32(3 + 25 + mMsgLen + 4, (unsigned char*)mBuf)) {
			if (verbose)
				cerr << "GPS: Err in check sum" << endl;
		}
		else {
			handleData();

		}
		mBytesRead = 0;
		mBytesToRead = 1;
		return;
	}
	
	cout << "GPS:: Reached illegal point in step" << endl;		
	
}

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
	
	char * data = mBuf + 3 + 25;
	
	int posStat = *((int*)(data));    
	mPosStat = (GPSINS_MSG::SolStatusEnum)posStat;   
	int posType = *((int*)(data+4));	 
	mPosType = (GPSINS_MSG::PosVelTypeEnum)posType;  
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
	
/*
	Vector<3> llh = ECEF2llh(mXYZ);
	printf("mXYZ: %1.16f %1.16f %1.16f, llh: %1.16f, %1.16f, %1.16f\n",
			mXYZ[0], mXYZ[1], mXYZ[2], llh[0], llh[1], llh[2]);
*/	
	
	
	if (verbose) {
		cout << "handleBestXYZ " 
			<< " mPosStat: " << mPosStat
			<< " mPosType: " << mPosType
			<< " mXYZ: " << mXYZ
			<< " mVelStat: " << mVelStat
			<< " mVelType: " << mVelType
		 	<< " mVelLat " << mVelLat
			<< " mDiffAge " << mDiffAge
			<< " mSolAge " << mSolAge 
			<< " mXYZsd: " << mXYZsd
			
			
			<< endl;
			
	}
	
	//do derived state
	mNED = ECEF_dist(mXYZ, mXYZZero, mLLHZero[0], mLLHZero[1]);
	mNEDsd = ECEF_dist(mXYZsd + mXYZZero, mXYZZero, mLLHZero[0], mLLHZero[1]);
	mNEDVel = ECEF_dist(mXYZVel + mXYZZero, mXYZZero, mLLHZero[0], mLLHZero[1]);
	mNEDVelsd = ECEF_dist(mXYZVelsd + mXYZZero, mXYZZero, mLLHZero[0], mLLHZero[1]);
	/*
	cout << "Derived state "
		<< " NED " << mNED
		<< " mNEDsd " << mNEDsd
		<< " mNEDVel " << mNEDVel
		<< " mNedVelsd " << mNEDVelsd
		<< endl;
	*/
	mKF->corrector(mPosStat == GPSINS_MSG::SOL_COMPUTED);
	
	mNumSamples++;
	
	struct timeval now;
	gettimeofday(&now, 0);
	if (verbose) {
	    if (getNumSecPassed(&mStartTime, &now) > 1.0) {
		    printf("GPS: number of samples per sec: %d\n", mNumSamples);
		    mNumSamples = 0;
		    gettimeofday(&mStartTime, 0);
		}
	}
	
}

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
}


void
GPS::handleData()
{
	
	if (mMsgId == BESTXYZ_ID) {
		handleBestXYZ();
	}
	else if (mMsgId == OMNISTAT_ID) {
		handleOmniStat();
	}
	else {
		cout << "Unknown data: msgid " << mMsgId << endl;
	}
	
	log(mBuf, 3 + 25 + mMsgLen + 4);	
	
}


