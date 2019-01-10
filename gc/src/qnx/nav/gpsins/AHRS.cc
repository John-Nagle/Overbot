/* -*- indent-tabs-mode:T; c-basic-offset:8; tab-width:8; -*- vi: set ts=8:
 *
 * Khian Hao Lim
 * Oct 03
 * Overbot
 *
 * Object that encapsulates all the data from the AHRS
 *
 */

#include <AHRS.h>
#include <sys/types.h>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include "thread.h"
#include "logprint.h"


AHRS::AHRS(char* ahrsDev, char* ahrsLog) :
	mDev(ahrsDev),
	mACC(0,0,0),
	mPQR(0,0,0),
	mRPY(0,0,0),
	mVel(0,0,0),
	mDist(0,0,0),
	mACCZero(0,0,0),
	mLogFd(0),
	mBytesRead(0),
	mNumSamples(0),
	mDt(1.0/60.0),
	mCalCnt(0)
	
{
		
	if (ahrsLog) {
		mLogFd = open( ahrsLog, O_WRONLY | O_CREAT, 0666 );

		if( mLogFd < 0  && verbose) 
		{
				perror( ahrsLog );
		}
	}
	/*
	//now we set the serial to be faster
	mSerial = new Serial(ahrsDev, 9600, "8N1", 999999);
	
	if (mSerial->Open() < 0) {
		cout << "Failed to open 9600 serial" << endl;
	}

	char * changeBaudMsg = "b";
	printf("AHRS sent b\n");
	mSerial->WriteBuf(changeBaudMsg, strlen(changeBaudMsg));
	mSerial->Close();
	sleep(1);
	delete mSerial;
	*/
	mSerial = new Serial(ahrsDev, 38400, "8N1", 999999);
	if (mSerial->Open() < 0) {
		cout << "Failed to open 38400 serial" << endl;
	}
			
	char * angleModeMsg= "a";
	mSerial->WriteBuf(angleModeMsg, strlen(angleModeMsg));
	printf("AHRS sent a\n");
	sleep(1);
	
	char * contModeMsg = "C";
	mSerial->WriteBuf(contModeMsg, strlen(contModeMsg));
	printf("AHRS sent C\n");
	sleep(1);

	gettimeofday(&mStartTime, 0);
	
	//TODO
	//set mDt correctly
}

AHRS::~AHRS()
{
}

void
AHRS::log(char* line, int len)
{
	if (mLogFd) {
		struct timeval t;
		gettimeofday(&t,0);
		write(mLogFd, (char*)&t, sizeof(t));
		write(mLogFd, line, len);
	}
		
}

void
AHRS::doubleIntegrate()
{
	
	//define rate here
	int freq = 60;
	double timeStep = 1.0 / (double) freq;
	/*
	if (mCalCnt < 60) {
		mACCZero += mACC;
		mCalCnt++;
		return;
	}
	if (mCalCnt == 60) {
		mCalCnt++;
		mACCZero /= 60;
		mAccZero[2] = 0.0; //we will leave the vertical axis g alone
	}
	*/
	
	//here mAcc can be zeroed
	//mACC -= mACCZero;
	//mVel += mACC * timeStep;
	//mDist += mVel * timeStep;
	
}

void
AHRS::handleData()
{
	short int raw;										// accumulate 2-byte signed values
	char * data = mBuf + 1;
	//	Read in two-byte SIGNED values.  Must OR bytes togther, then treat as signed.
	raw = (signed char) data[0];
	raw <<= 8;
	raw |= (unsigned char) data[1];
	mRPY[0] = (float ) ((float)raw * 180.0/ 32768.0);
	
	raw = (signed char) data[2];
	raw <<= 8;
	raw |= (unsigned char) data[3];
	//TODO issit?
	mRPY[1] = (float ) ((float)raw * 180.0/ 32768.0);
	
	
	raw = (signed char) data[4];
	raw <<= 8;
	raw |= (unsigned char) data[5];
	mRPY[2] = (float ) ((float)raw * 180.0/ 32768.0);
	
	raw = (signed char) data[6];
	raw <<= 8;
	raw |= (unsigned char) data[7];
	mPQR[0] = (float ) ((float)raw * 200.0 * 1.5/ 32768.0);
	
	raw = (signed char) data[8];
	raw <<= 8;
	raw |= (unsigned char) data[9];
	mPQR[1] = (float ) ((float)raw * 200.0 * 1.5/ 32768.0);
	
	raw = (signed char) data[10];
	raw <<= 8;
	raw |= (unsigned char) data[11];
	mPQR[2] = (float ) ((float)raw * 200.0 * 1.5/ 32768.0);
	
	raw = (signed char) data[12];
	raw <<= 8;
	raw |= (unsigned char) data[13];
	mACC[0] = (float ) ((float)raw * 10.0 * C_G0MPERSEC2* 1.5/ 32768.0);

	raw = (signed char) data[14];
	raw <<= 8;
	raw |= (unsigned char) data[15];
	mACC[1] = (float ) ((float)raw * 10.0 * C_G0MPERSEC2*1.5/ 32768.0);


	raw = (signed char) data[16];
	raw <<= 8;
	raw |= (unsigned char) data[17];
	mACC[2] = (float ) ((float)raw * 10.0 * C_G0MPERSEC2*1.5/ 32768.0);

	if (verbose) {
	cout << "angles " << mRPY
		<< " pqr " << mPQR
		<< " acc " << mACC << endl;
	}
#ifdef OBSOLETE		// turned out to be a grounding problem
	static unsigned int logcnt = 0;
	if (((logcnt++)% 100) == 0)							// every 100th time
	{	//	***TEMP*** check for bad yaw data
		uint8_t raw1 = data[4];
		uint8_t raw2 = data[5];
		short int rawint = (((signed char) raw1) << 8 ) | ((unsigned char) raw2);	// convert as above
		float yawdeg = rawint * (180.0/32768.0);
		logprintf("Yaw check: raw 0x%02x 0x%02x = %d = %1.4f deg = %1.4f deg in RPY.\n",raw1, raw2, rawint, yawdeg, mRPY[2]);
	}
#endif // OBSOLETE
	//	Correct for magnetic compass deviation (JN)
	mRPY[2] = magneticDeclinationCorrection(mRPY[2]);
	
	doubleIntegrate();
	mKF->predictor();

	log(mBuf, PACKETSIZE);
	mNumSamples++;
	
	struct timeval now;
	gettimeofday(&now, 0);

	if (getNumSecPassed(&mStartTime, &now) > 1.0) {
		if (verbose)																					// only display in verbose mode.
		{	printf("AHRS: number of samples per sec: %d\n", mNumSamples);	}
		mNumSamples = 0;
		gettimeofday(&mStartTime, 0);
	
	}
	

}

/**
 * returns true when checksum is fine
 * false otherwise
 */
bool
AHRS::verifyCheckSum()
{
	unsigned int sum = 0;	
	for (int i=1; i < PACKETSIZE - 1 ;i++) {
		sum += (unsigned char)(mBuf[i]);
	}
	return (sum % 256 )== (unsigned char)mBuf[PACKETSIZE - 1];
		
}

void
AHRS::step()
{
	
	if (mBytesRead ==0 ) {
		if (mSerial->ReadBuf(mBuf,1) != 1) {
			if (verbose)
				printf("AHRS failed to read header byte\n");
			return;
		}
		if (((unsigned char)mBuf[0]) != 255) {
			if (verbose) 
				printf("header not 255\n");
			return;
		}
		mBytesRead++;
		return;
	}

	int ret = mSerial->ReadBuf(mBuf + mBytesRead, PACKETSIZE - mBytesRead);
	if (ret <= 0) {
		if (verbose) {
			perror("AHRS:: Read buf");
		}
		return;
	}
	
	mBytesRead += ret;

	if (mBytesRead < PACKETSIZE /* packet size */) {
		return;
	}
	
	//mBytesRead == PACKETSIZE, got a packet, reset mBytesRead
	mBytesRead = 0;
				
	if (!verifyCheckSum()) {
		if (verbose) 
			cerr << "AHRS: Err in check sum" << endl;
		//we try to clear all the data from the buffer now
		char unused[65536];
		mSerial->ReadBuf(unused, 65536);
		return;
	}

	handleData();
	
	
}
//
//	magneticDeclinationCorrection  -- correct compass for magnetic declination
//
//	DUMB VERSION - applies constant value suitable for middle of California.
//	Really should have a correction model.
//
//	Data from National Geophysical Data Center as of 16 FEB 2005.
//	www.ngdc.noaa.gov
//
const float k_barstow_correction = 13 + 11/60.0;					// correction at Barstow, CA: 13 deg 11 minutes
const float k_rwc_correction = 14 + 44/60.0;						// correction at Redwood City, CA: 14 degrees 44 minutes
const float k_midcalifornia_correction = (k_barstow_correction + k_rwc_correction)*0.5;		// mean value for areas of interest 
//
float AHRS::magneticDeclinationCorrection(float yawdeg)
{	float uncorrected = yawdeg;	
	yawdeg = fmod(yawdeg+k_midcalifornia_correction+360, 360);	// apply correction, keep in range
	//	Temporary debug print.  Prints compass values as hardware check.
	static unsigned int cnt = 0;
	if (((cnt++)% 100) == 0)
	{	logprintf("Compass: uncorrected %1.2f deg.  Corrected %1.2f deg.\n", uncorrected, yawdeg);}
	return(yawdeg);
}
