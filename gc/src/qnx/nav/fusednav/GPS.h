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

#include <cstring>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "Conversions.h"

#include "macros.h"


#include <stdio.h>
#include "gpsins_messaging.h"
#include "Nav.h"
#include "myserial.h"
#include "mutexlock.h"
//gain access to 'SimpleOdometer::getOdometer()', Frank Zhang, 3/16/05
#include "odospeedometer.h"

//
//	Constants
//
//	GPS message types
#define BESTXYZ_ID 241
#define OMNISTAT_ID 510
#define LBANDSTAT_ID 731		// Omnistar diagnostic output
#define RANGE_ID 43			// Another diagnostic output
#define RAWEPHEMB_ID 41			// Ephemeris changed
#define SATVIS_ID 48


#define GPSMODELSIZE 20
const size_t k_gpsmaxbuf = 2048;	// maximum size of GPS buffer

using namespace std;

class GPS
{
public:
	GPS(char* gpsDev, char* gpsLog);
	~GPS();
	void GPS::setBasePoint(Vector<3> bPointllh);
	void syncToStartingPoint();
	void GPS::gpsStart();

private:	
	void 
	log(char * line, int len);
	
	void 
	GPS::handleBestXYZ();
	
	void 
	GPS::handleOmniStat();

	void
	handleData();
	
	void
	calculateHeading();

	bool
	doCheckSum(char * data, unsigned short msglen, char * checksum);
	char * mDev; //the serial device we are using

	void readGPSData(); // method to read the GPS data from serial port. Frank Zhang, 2/8/05
	void *gpsio_thread();
	
	static void *gpsio_threadstart(void * arg) { return(reinterpret_cast<GPS*>(arg)->gpsio_thread()); }
public:    
	Kalman_Filter * mKF;		// Kalman filter stores into this during initialization
private:
	Serial * mSerial;
	
	int mLogFd;
	
	unsigned int mBytesRead;
	unsigned int mBytesToRead;
		
	char mBuf[k_gpsmaxbuf];		// input buffer for current GPS message from serial port
	unsigned short mMsgId;
	unsigned short mMsgLen;
	
	//see data log of novatel command and log reference for more details

	//from bestxyz log
	Vector<3>	mXYZsd;		//standard deviation for mXYZ, in meters
	GPSINS_MSG::SolStatusEnum mVelStat;  //velocity status
	GPSINS_MSG::PosVelTypeEnum  mVelType;//velocity type
	Vector<3>	mXYZVel;	// m/s
	Vector<3>	mXYZVelsd;	// standard deviation for mVel, in m/s
	char		mStnID[4];	// base station identification
	float		mVelLat;	// latency in the velocity time tag
	float		mDiffAge;	// differential age
	float		mSolAge;	// solution age
	unsigned char	mNumObs;	// number of observations
	unsigned char	mNumGPSL1;	// number of GPSL1 ...
	unsigned char	mNumL1;		// number of L1 ranges ...
	unsigned char	mNumL2;		// number of L2 ranges ...
public:
	//really should be the starting point, currently set to the outside the workshop
	Vector<3>	mLLHZero;	// rad,rad,m
	Vector<3>	mXYZZero;	// m
public:
	//	This data is shared with the Kalman Filter thread.
	ost::Mutex  mLock;		// added after splitting io read for gps into a thread, Frank Zhang, 2/7/05
	//derived state
	GPSINS_MSG::SolStatusEnum mPosStat;//position status,
	GPSINS_MSG::PosVelTypeEnum	mPosType; 
	Vector<3>	mXYZ;		// position in ECEF coordinate in meters
	Vector<3>	mLLH;		// position in Long., Lat., Height coordinate system in deg, deg, meter 
	Vector<3>	mNED;
	Vector<3>	mNEDsd;
	Vector<3>	mNEDVel;
	Vector<3>	mNEDVelsd;
	//	End of shared data
private:	
	//from omnistat log, see pg 174 of cmd and log reference
	unsigned long 	mFreq; 
	float 		mCN0;
	float 		mLockTime;
	char		mTracking[2];
	char 		mVBStat[2];
	unsigned long 	mNumBytesVBS;
	unsigned long 	mNumGoodDGPS;
	unsigned long 	mBadData;
	char		mHPStat1[2];
	char		mHPStat2[2];
	unsigned long 	mNumBytesHP;

        // added to support deep processing of GPS data to support fusednav
public:																		// more shared data, protected by mLock above
        Vector<3>	mAbs;
	uint64_t	mLastSampleTime; // time of last GPS sample
	double		mHeading;	 // Heading in degrees       

private:
        Vector<3>	mNEDAccumulate;
public:
        uint32_t	mGPSNoMoveSampleCntr;
private:
        Vector<3>	mPreAbs;
        Vector<3>	mVel;

private:
	// Diagnostics for GPS
	void		handleLbandStat();
	void		handleRange();
	void		handleRawEphemeris();
	void		logHeaderStatus(uint32_t errs);
};
#endif
