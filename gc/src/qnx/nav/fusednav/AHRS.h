/* -*- indent-tabs-mode:T; c-basic-offset:8; tab-width:8; -*- vi: set ts=8:
 *
 * Frank Zhang, modified to support fusednav approach
 * March 2005
 * Khian Hao Lim
 * Oct 03
 * Overbot
 *
 * Object that encapsulates all the data from the AHRS
 *
 */
#ifndef AHRS_H
#define AHRS_H

#include <stdio.h>
#include "Vector.h"
#include "myserial.h"

//gain access to 'OdoSpeedometer::getOdometer()', Frank Zhang, 3/6/05
#include "odospeedometer.h" 

#define TRAVELDATAHIST_SIZE 5

class Kalman_Filter;

using namespace std;

class AHRS
{
public:
	AHRS(char * ahrsDev, char* ahrsLog);
	~AHRS();
	
	void step();
	void log(char * msg, int len);
	bool verifyCheckSum();
	void handleData();
	void processOdometerData();
    void maintainOdometers();
	float magneticDeclinationCorrection(float yawdeg);
	
	Kalman_Filter * mKF;
	char * 			mDev;
	Vector<3>		mACC;
	Vector<3>		mPQR;
	Vector<3>		mRPY;
	Vector<3>		mVel;  // integrated velocity, north, east, z; also x, y, z
	Vector<3>		mRel;  // integrated relative location, north, east, z; also x, y, z
                                       // origin from robot starting point (key on)

	Vector<3>		mPreviousVel;	
	Serial * mSerial;

	//testing of double integration
	//Vector<3> mVel;
	Vector<3> mDist;
	Vector<3> mACCZero;  // drift in acceleration when no moving
	Vector<3> mPQRZero;  // drift in PQR when no moving: pitch change rate, roll change rate, heading rate
	
	int mLogFd;
	
	char mBuf[64];
	
	int mBytesRead;
			
	uint64_t mLastGPSSampleTime; // Time of the last GPS sample used to set the known position
	
	int mCalCnt;
	
	static const int PACKETSIZE = 30;
        short int mLastSample;               // index to the latest sample from last time
        double mEncoderDistThisAHRSCycle;

private:
};

#endif
