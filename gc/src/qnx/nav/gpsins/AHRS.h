/* -*- indent-tabs-mode:T; c-basic-offset:8; tab-width:8; -*- vi: set ts=8:
 *
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
#include <Vector.h>
#include <myserial.h>

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
	void doubleIntegrate();
	float magneticDeclinationCorrection(float yawdeg);
	
	Kalman_Filter * mKF;
	char * 			mDev;
	Vector<3>		mACC;
	Vector<3>		mPQR;
	Vector<3>		mRPY;
	
	Serial * mSerial;

	//testing of double integration
	Vector<3> mVel;
	Vector<3> mDist;
	Vector<3> mACCZero;
	
	int mLogFd;
	
	char mBuf[64];
	
	int mBytesRead;
	
	int mNumSamples;
	
	double mDt;
	
	struct timeval mStartTime;
	
	int mCalCnt;
	
	static const int PACKETSIZE = 30;
private:
};

#endif
