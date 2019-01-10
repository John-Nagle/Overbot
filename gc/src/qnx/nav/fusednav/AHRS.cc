/* -*- indent-tabs-mode:T; c-basic-offset:8; tab-width:8; -*- vi: set ts=8:
 *
 * Frank Zhang, added to existing AHRS class to support fused nav approach.
 * Khian Hao Lim
 * Oct 03
 * Overbot
 *
 * Object that encapsulates all the data from the AHRS
 *
 */

#include "AHRS.h"
#include <sys/types.h>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include "thread.h"
#include "fusednav.h"
#include "logprint.h"
//#include <algorithm>



AHRS::AHRS(char* ahrsDev, char* ahrsLog) :
	mDev(ahrsDev),
	mACC(0,0,0),
	mPQR(0,0,0),
	mRPY(0,0,0),
	mVel(0,0,0),
	mRel(0,0,0),
	mDist(0,0,0),
	mACCZero(0,0,0),
	mPQRZero(0,0,0),
	mLogFd(0),
	mBytesRead(0),
	mCalCnt(0),
	mLastSample(0),
	mEncoderDistThisAHRSCycle(0.0)	
{
		
	if (ahrsLog) {
		mLogFd = open( ahrsLog, O_WRONLY | O_CREAT, 0666 );

		if( mLogFd < 0  && verbose) 
		{
			perror( ahrsLog );
		}
	}
	

	mSerial = new Serial(ahrsDev, 38400, "8N1", 999999);
	if (mSerial->Open() < 0) {
		cout << "Failed to open 38400 serial" << endl;
	}
			
	char * angleModeMsg= "a";
	mSerial->WriteBuf(angleModeMsg, strlen(angleModeMsg));
	sleep(1);
	
	char * contModeMsg = "C";
	mSerial->WriteBuf(contModeMsg, strlen(contModeMsg));
	sleep(1);
        
	mLastGPSSampleTime = 0;
}

AHRS::~AHRS()
{
}
//
//	log -- generates binary log of GPS info
//
void
AHRS::log(char* line, int len)
{
	if (mLogFd) {
		uint64_t t = gettimenowns();							// current time in ns
		write(mLogFd, (char*)&t, sizeof(t));
		write(mLogFd, line, len);
	}	
}

void
AHRS::handleData()
{
	short int raw;		  // accumulate 2-byte signed values
	char * data = mBuf + 1;
	//	Read in two-byte SIGNED values.  Must OR bytes togther, then treat as signed.
	raw = (signed char) data[0];
	raw <<= 8;
	raw |= (unsigned char) data[1];
	mRPY[0] = (float ) ((float)raw * 180.0/ 32768.0); // Roll Angle, degrees
	
	raw = (signed char) data[2];
	raw <<= 8;
	raw |= (unsigned char) data[3];
	mRPY[1] = (float ) ((float)raw * 180.0/ 32768.0); // Pitch Angle, degrees
	
	raw = (signed char) data[4];
	raw <<= 8;
	raw |= (unsigned char) data[5];
	mRPY[2] = (float ) ((float)raw * 180.0/ 32768.0); // Heading Angle, degrees
	
	raw = (signed char) data[6];
	raw <<= 8;
	raw |= (unsigned char) data[7];
	mPQR[0] = (float ) ((float)raw * 200.0 * 1.5/ 32768.0); // Roll Anguler Rate, degrees/sec
	
	raw = (signed char) data[8];
	raw <<= 8;
	raw |= (unsigned char) data[9];
	mPQR[1] = (float ) ((float)raw * 200.0 * 1.5/ 32768.0); // Pitch Angular Rate, degrees/sec
	
	raw = (signed char) data[10];
	raw <<= 8;
	raw |= (unsigned char) data[11];
	mPQR[2] = (float ) ((float)raw * 200.0 * 1.5/ 32768.0); // Yaw Angular Rate, degrees/sec
	
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
	
	//	Correct for magnetic compass deviation (JN)
	mRPY[2] = magneticDeclinationCorrection(mRPY[2]);
	//	Correct acceleration for gravity
	Vector <3> angles (mRPY[0] * C_DEG2RAD,						// compute euler angles for orientation
						mRPY[1] * C_DEG2RAD,
						mRPY[2] * C_DEG2RAD);	
	const Matrix<3,3> cBE(eulerDC(angles));							// convert to rotation matrix
	const Vector<3> gVector (0,0,9.81);									// Earth gravity
	Vector<3> vehicleFrameGravity = cBE * gVector;			// gravity in vehicle frame
	mACC -= vehicleFrameGravity;											// subtract out gravity
	
	// mKF->predictor();
	
	// log(mBuf, PACKETSIZE);
	
}// End of handleData()

/**
 * verifyCheckSum returns true when checksum is fine
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

/* Called from step() in Kalman_Filter which is in turn called from main
*/
void
AHRS::step()
{
	
	if (mBytesRead ==0 ) {
		if (mSerial->ReadBuf(mBuf,1) != 1) {
			if (verbose)
				logprintf("AHRS failed to read header byte\n");
		return;
		}
		if (((unsigned char)mBuf[0]) != 255) {
			if (verbose) 
				logprintf("header not 255\n");
			return;
		}
		mBytesRead++;
		return;
	}

	int ret = mSerial->ReadBuf(mBuf + mBytesRead, PACKETSIZE - mBytesRead);
	if (ret <= 0) {
		if (verbose) {
			logprintf("AHRS:: Read buf failed\n");
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
	// trigger fusednav, make sure GPS, AHRS, and fusednav in task(s) w/ equal priority
	mKF->mFusedNav->fuseAHRS();
	
} // End of step()

/*

  -- correct compass for magnetic declination

DUMB VERSION - applies constant value suitable for middle of California.
Really should have a correction model.

Data from National Geophysical Data Center as of 16 FEB 2005.
www.ngdc.noaa.gov
*/

const float k_barstow_correction = 13 + 11/60.0;					// correction at Barstow, CA: 13 deg 11 minutes
const float k_rwc_correction = 14 + 44/60.0;						// correction at Redwood City, CA: 14 degrees 44 minutes
const float k_midcalifornia_correction = (k_barstow_correction + k_rwc_correction)*0.5;	// mean value for areas of interest 
//
float AHRS::magneticDeclinationCorrection(float yawdeg)
{
	yawdeg = fmod(yawdeg+k_midcalifornia_correction+360, 360);	// apply correction, keep in range
	return(yawdeg);
}
