/* -*- indent-tabs-mode:T; c-basic-offset:8; tab-width:8; -*- vi: set ts=8:
 *
 * Khian Hao Lim
 * Oct 03
 * Overbot
 *
 * Object that encapsulates all the data from the GPS
 */
#ifndef GPS_H
#define GPS_H

class Kalman_Filter;

#include <stdio.h>
#include <../../common/include/gpsins_messaging.h>
#include <../../common/include/Nav.h>
#include <Vector.h>
#include <myserial.h>
//
//	Constants
//
#define BESTXYZ_ID 241
#define OMNISTAT_ID 510
#define SATVIS_ID 48

const size_t k_gpsmaxbuf = 512;									// maximum size of GPS buffer

using namespace std;

class GPS
{
public:
	GPS(char* gpsDev, char* gpsLog);
	~GPS();

	void 
	step();
	
	void 
	log(char * line, int len);
	
	void 
	GPS::handleBestXYZ();
	
	void 
	GPS::handleOmniStat();

	void
	handleData();

	bool
	doCheckSum(char * data, unsigned short msglen, char * checksum);
	char * mDev; //the serial device we are using

	Kalman_Filter * mKF;
	
	Serial * mSerial;
	
	int mLogFd;
	
	int mBytesRead;
	int mBytesToRead;
	
	int mNumSamples;
	struct timeval mStartTime;
	
	char mBuf[k_gpsmaxbuf];									// input buffer for current GPS message from serial port
	unsigned short mMsgId;
	unsigned short mMsgLen;
	
	//see data log of novatel command and log reference for more details

	//from bestxyz log
	GPSINS_MSG::SolStatusEnum	mPosStat;//position status,
	GPSINS_MSG::PosVelTypeEnum	mPosType; //position type, 
	Vector<3>		mXYZ;   //meters
	Vector<3>		mXYZsd; //standard deviation for mXYZ, in meters
	GPSINS_MSG::SolStatusEnum	mVelStat;//velocity status
	GPSINS_MSG::PosVelTypeEnum  mVelType;//velocity type
	Vector<3>		mXYZVel;   //m/s
	Vector<3>		mXYZVelsd; //standard deviation for mVel, in m/s
	char			mStnID[4]; //base station identification
	float			mVelLat;  //latency in the velocity time tag
	float			mDiffAge; //differential age
	float			mSolAge;//solution age
	unsigned char	mNumObs;//number of observations
	unsigned char	mNumGPSL1;//number of GPSL1 ...
	unsigned char	mNumL1;//number of L1 ranges ...
	unsigned char	mNumL2;//number of L2 ranges ...

	//really should be the starting point, currently set to the outside the workshop
	Vector<3>		mLLHZero; //rad,rad,m
	Vector<3>		mXYZZero; //m
	
	//derived state
	Vector<3>		mNED;
	Vector<3>		mNEDsd;
	Vector<3>		mNEDVel;
	Vector<3>		mNEDVelsd;
	
	//from omnistat log, see pg 174 of cmd and log reference
	unsigned long 	mFreq; 
	float 			mCN0;
	float 			mLockTime;
	char 			mTracking[2];
	char 			mVBStat[2];
	unsigned long 	mNumBytesVBS;
	unsigned long 	mNumGoodDGPS;
	unsigned long 	mBadData;
	char			mHPStat1[2];
	char			mHPStat2[2];
	unsigned long 	mNumBytesHP;

private:
};
#endif
