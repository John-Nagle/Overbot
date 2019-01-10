/* -*- indent-tabs-mode:T; c-basic-offset:8; tab-width:8; -*- vi: set ts=8:
 *
 * Frank Zhang
 * March 2005
 * Overbot
 *
 * Object that encapsulates fusednav
 *
 */
#ifndef FUSEDNAV_H
#define FUSEDNAV_H

#include <stdio.h>

#include "mutexlock.h"
#include "Kalman_Filter.h"
#include "AHRS.h"
#include "GPS.h"

#define FZFILTERSIZE 30
#define FUSEDNAV_SOFTWARE_ERROR -2
#define FUSEDNAV_GPS_COM_FAILURE -3

using namespace std;

enum NavModeEnum{
   NAV_INIT         = 0,
   NAV_OMNISTAR     = 1,
   NAV_OMNISTAR_HP  = 2,
   NAV_DEADRECKONING= 3,
   NAV_POSITION_LOST= 4,
   NAV_NOT_READY    =99
};

struct fzFilterBuffer
{
   Vector<3>	positionDelta[FZFILTERSIZE];		// offset between the current DR position and reported GPS position
   Vector<3>	positionDeltaSum;			// the sum of all deltas in positionDeta
   Vector<3>	averageDelta;				// Average of the deltas stored in the buffer
   Vector<3>	savedAverageDelta;			// For discontinuity check
   Vector<3>	fusedPos[FZFILTERSIZE];			// Buffer of fused positions
   Vector<3>	fusedPosUnc[FZFILTERSIZE];		// Buffer of fused position uncertainties
   NavModeEnum	navMode[FZFILTERSIZE];			// Nav mode for fused postion
   uint16_t	countOfSeqDiscontinuities;		// Number of sequential discontinuities
   uint16_t	countEntries;				// number of points in the buffer
   uint16_t	index;					// points to next entry
};

class FusedNav
{
public:
	FusedNav(char* fusednavLog, Kalman_Filter * pKF,  AHRS * pAHRS, GPS * pGPS);
	~FusedNav();
	
	void fuseAHRS();   // fuse the latest AHRS sample result into fusednav
	void fuseGPS();    // fuse the latest GPS sample result into fusednav	
private:
	double getOdometerCalibration();
	void discontinuityCheck();
	void periodicTasks();
	void updateFilterBuffer();
	void clearFilterBuffer();
	void updatePosition();
	void updateDRPosition();
	void updateFusedPosition();
	double calcCircUncertainty( Vector<3> unc);
	ost::Mutex m_lock;			// lock during updates
public:

	Kalman_Filter * mKF;
	AHRS *		mAHRS;
	GPS *		mGPS;
        fzFilterBuffer	mFilterBuffer;		// Holds the information necessary to smooth reported position
	Vector<3>	mAbs;			// Most recent valid GPS position - n, e, d in meters
	Vector<3>	mDRPosition;		// The current dead reckoning position
	Vector<3>	mDRUnc;			// The current dead reckoning uncertainty
	Vector<3>	mDRBase;		// DR Position is relative to this starting point
	Vector<3>	mDRBaseUnc;		// Uncertainty when DRBase was recorded
	NavModeEnum	mDRBaseNavMode;		// Nav mode when DRBase was recorded
	Vector<3>	mAbsPrevious;		// Second to most recent valid GPS position
	Vector<3>	mUnc;			// Most recent valid GPS uncertainty in meters
	Vector<3>	mRel;			// rel position from mAbs n, e, d in meters w/o GPS correction
	Vector<3>       mVel;			// velocity (NED, m/sec)
	Vector<3>	mACC;			// accelerations (NED, m/sec2)
	Vector<3>	mPQR;			// angular rates
	Vector<3>	mRPY;			// roll, pitch, yaw in degrees
	Vector<3>	mLLH;			// position Long., Lat, Heigt (degrees, degrees, m)
	Vector<3>       mRelPiece;		// The piece-wise moved between last 2 AHRS update in m
	Vector<3>       mZeroVector;            // a constant zero vector
        OdoSpeedometer	mOdoSpeedometer;
        TravelData	mTravelData;
	Vector<3>       mFusedPosition;     	// Depends on navigation mode. Will be mAbs+mRel in dead reckoning mode.
	Vector<3>	mFusedUncertainty;	// Uncertainty of estimated position. Depends on navigation mode calculated
						// when in dead reckoning mode, else filled with the GPS reported value
	double		mOdometerDistThisGPSCycle;
	double		mOdometerValueCurrent;
	double		mOdometerValuePrevious;
	double		mOdometerCalibration;
	uint64_t	mGPSMessageTimestamp;	// Timestamp for most recent GPS message received
	uint64_t	mNavPosLostTimestamp;	// Timestamp when position lost declared - used to force this state before trying to init	
	uint64_t	mRepTimestamp;		// Timestamp of most recent position report
	uint64_t	mCalibrateTimestamp;	// Timestamp of most recent odometer calibration
	NavModeEnum	mNavMode;		// The latest nav mode
	NavModeEnum	mPrevNavMode;		// Nav mode in the previous cycle
	bool		mStaleGPSReport;	// Flag stale GPS data
	bool		mUsefulGPSReport;	// If true t's OMNISTAR or OMNISTAR_HP with good position status
	bool		mMoving;		// true if vehicle moving this cycle
	bool		mPrevMoving;		// true if vehicle moving last cycle  

	inline const char *
	decodeNavMode(NavModeEnum e)
	{
	    switch (e)
	    {
	    case	NAV_INIT:
	        return "NAV_INIT";
	    case NAV_OMNISTAR:
	        return "NAV_OMNISTAR";
	    case NAV_OMNISTAR_HP:
	        return "NAV_OMNISTAR_HP";
	    case NAV_DEADRECKONING:
	        return "NAV_DEADRECKONING";
	    case NAV_POSITION_LOST:
	        return "NAV_POSITION_LOST";
	    default:
	    return "UNKNOWN NAV MODE";
	    }
	}
	
public:
	Vector<3>	getFusedPositionNED() const { return(mFusedPosition); }	// position to use (NED, m)
	Vector<3>	getFusedUncertainty() const;			// uncertainty (m)
	Vector<3>	getRPY() const { return(mRPY); }		// roll, pitch, yaw (degrees)
	Vector<3>	getVelocity() const { return(mVel); }		// velocity (NED, m/sec)
	Vector<3>	getAngularRates() const { return(mPQR); }	// roll, pitch, yaw (degrees/sec)
	Vector<3>	getAcceleration() const { return(mACC); }	// accelerations (NED, m/sec2)
	Vector<3>	getLLH() const;															// latitude, longitude, height
//	uint64_t	getTimestamp() const {return(mRepTimestamp); }	// timestamp
	//
	void
 	copyState(GPSINSMsgRep& rep);					// fill reply message
};

#endif
