/* -*- indent-tabs-mode:T; c-basic-offset:8; tab-width:8; -*- vi: set ts=8:
 *
 * Frank Zhang, Created to support fused nav approach.  3/10/2005
 * Overbot
 * Dwayne Hull, Complete Rewrite August 2005
 *
 * class that encapsulates all information of fusednav 

 *
 */

#include "fusednav.h"
#include <sys/types.h>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <math.h>
#include "Vector.h"
#include "logprint.h"
#include "tuneable.h"

//
//	Constants
//
const Tuneable k_gps_discontinuity_distance("GPSDISCONTINUITY", 0.01, 5.0, 0.5, "Maximum allowed position jump before reset (m)");
const Tuneable k_max_uncertainty("MAXPOSITIONUNCERTAINTY", 0.0, 5.0, 5.0, "Maximum allowed position uncertainty before reset (m)");
const Tuneable k_max_run_uncertainty("MAXRUNUNCERTAINTY", 0.0, 5.0, 0.5, "Maximum allowed position uncertainty releasing NAV_INIT (m)");
const Tuneable k_odometer_calibration_factor("ODOMETERCALIBRATION", 0.9, 1.1, 1.0, "Odometer calibration factor. Value other than 1.0 overrides auto adjustment.");	
const Tuneable k_odometer_error_per_meter("ODOMETERERROR", 0.0, 0.05, 0.02, "Odometer error per meter of travel.");	

const Tuneable k_max_gps_delay("GPSMAXDELAY", 0.1, 2.0, 0.25, "Maximum GPS Message delay, seconds before considering it stale");	

const double k_pos_lost_wait = 3.0;							// Delay in position lost state, seconds
const double k_cal_wait =    1.0;						        // Delay between calibration attempts, seconds

const double k_min_odometer_delta = 0.0001;						// Minimum movement, meters that flags movement
const double k_min_odometer_cal_delta = 0.0166;						// Minimum movement for odometer calibration

FusedNav::FusedNav(char* fusednavLog,  Kalman_Filter * pKF,  AHRS * pAHRS, GPS * pGPS) :
        mAbs(0,0,0),
        mDRPosition(0,0,0),
        mDRUnc(0,0,0),
        mDRBase(0,0,0),
        mDRBaseUnc(0,0,0),
        mDRBaseNavMode(NAV_NOT_READY),
        mAbsPrevious(0,0,0),
        mUnc(0,0,0),
        mRel(0,0,0),
	mVel(0,0,0),
	mACC(0,0,0),
	mPQR(0,0,0),
	mRPY(0,0,0),
	mLLH(0,0,0),
        mRelPiece(0,0,0),
        mZeroVector(0,0,0),
        mFusedPosition(0,0,0),
        mFusedUncertainty(1000.,1000.,1000.),
        mOdometerDistThisGPSCycle(0.0),
        mOdometerValueCurrent(0.0),
        mOdometerValuePrevious(0.0),
        mOdometerCalibration(1.0),
        mGPSMessageTimestamp(0),
        mNavPosLostTimestamp(0),
        mRepTimestamp(0),
	mCalibrateTimestamp(0),
	mNavMode(NAV_INIT),
	mPrevNavMode(NAV_INIT),
	mStaleGPSReport(false),
	mUsefulGPSReport(false),
	mMoving(false)
{
	mKF = pKF;
	mAHRS = pAHRS;
	mGPS = pGPS;
	clearFilterBuffer();
}

FusedNav::~FusedNav()
{
}

/* Called by AHRS.step(), which in turn is called by Kalman_Filter.step() which in turn is called by main.
   It only gets called when there is a new, complete AHRS message to process, about 60 times/sec.
*/
void
FusedNav::fuseAHRS()
{
	ost::MutexLock lok(m_lock);
        mACC = mAHRS->mACC;
        mVel = mAHRS->mVel;
        mPQR = mAHRS->mPQR;
        mRPY = mAHRS->mRPY;

} // end of FusedNav::fuseAHRS()


/*******************************************************************************************
 *	fuseGPS:
 *	Called by GPS thread when a new GPS pos update is delivered.
 *	FuseGPS will maintain the following:
 *		
 *	- Navigation mode:
 *	  1) Omnistar (w/blended INS influence)
 *	  2) Omnistar HP (w/blended INS influence)
 *	  3) Dead Reckoning (Pure INS)
 *	- Position Uncertainty
 *	  1) Dead Reckoning (calculated based on the uncertainty of the last valid GPS report
 *	     and the relative distance from that position)
 *	  2) The uncertainty reported by the GPS if in Omnistar or Omnistar HP
 *
 *	The following mNavMode states are maintained:
 *	NAV_INIT		Waiting for GPS Omnistar HP data to determine current position
 *	NAV_POS_LOST		Multiple sequential discontinuities in GPS data
 *				We stay in this mode for a few seconds before
 *				entering NAV_INIT to reestablish our current position.
 *	NAV_OMNISTAR_HP		Using GPS data to calculate the fused position
 *	NAV_OMNISTAR		Using GPS data to calculate the fused position
 *	NAV_DEADRECKONING	Not using GPS data to calculate the fused position
 *******************************************************************************************/
	
void
FusedNav::fuseGPS()
{
	//	No need to lock fuseGPS() as it is called within
	//	a lock in GPS.cc



	uint64_t now = gettimenowns();					// now, in nanoseconds
	uint64_t tdiff = now - mGPSMessageTimestamp;
	mStaleGPSReport = false;
	mUsefulGPSReport = false;
	double tdiffSecs = tdiff * 0.000000001;
 	if (tdiffSecs > (k_max_gps_delay) && (tdiffSecs < 3600.0) )					// if falling behind
 	{
		logprintf("GPS not responding fast enough. Delay %1.4f sec.\n", tdiffSecs);
		mStaleGPSReport = true;
 	}
 	mGPSMessageTimestamp = now;
	// Copy GPS info
	mAbsPrevious = mAbs;			// Needed later for odometer calibration
	mAbs = mGPS->mAbs;
	mUnc = mGPS->mNEDsd;
	if( (mGPS->mPosStat == GPSINS_MSG::SOL_COMPUTED)  &&
		    ((mGPS->mPosType == GPSINS_MSG::OMNISTAR_HP) ||
		    (mGPS->mPosType == GPSINS_MSG::OMNISTAR) ) &&
		    !mStaleGPSReport ) 
	{
		mUsefulGPSReport = true;
		mRepTimestamp = now;
	}
	mPrevNavMode = mNavMode;
	
	switch (mNavMode)
	{
	    case NAV_POSITION_LOST:
	    {
	    	// Stay in this state until required delay has expired
	    	tdiff = now - mNavPosLostTimestamp;
	    	if (tdiff *.000000001 > (k_pos_lost_wait))
	    	{
	    		mNavMode = NAV_INIT;
	    		logprintf("Entering NAV_INIT state.\n");
	    	}
	    	break;
	    }
	
	    case NAV_INIT:
	    {                                
		// We require OMNISTAR_HP OR OMNISTAR with uncertainty less than a
		// threshold to get out of NAV_INIT.
		// Check if we have it.
		if( mUsefulGPSReport )
		{
			Vector<3>unc = mZeroVector;
			unc = mUnc;
			double circUnc = calcCircUncertainty( unc );

			if(mGPS->mPosType == GPSINS_MSG::OMNISTAR)
			{
				if(circUnc > k_max_run_uncertainty)
				{
					return;
				}
				mNavMode = NAV_OMNISTAR;
			}
			else
			{
				mNavMode = NAV_OMNISTAR_HP;
			}
			
			// Ok! We're good to go
			mTravelData = mOdoSpeedometer.getTravelData();	// read odomoter
			mOdometerValueCurrent = mTravelData.odometer;
			mOdometerValuePrevious = mOdometerValueCurrent; // Zero the odometer history
			mOdometerDistThisGPSCycle = 0.0;
			mRel = mZeroVector;
			mDRPosition = mAbs;
			mFusedPosition = mAbs;
			mFusedUncertainty = mUnc;
			mDRBase = mAbs;
			mDRBaseUnc = mUnc;
			mDRBaseNavMode = mNavMode;
			mDRUnc = mDRBaseUnc;
			clearFilterBuffer();
			mFusedUncertainty = mUnc;
			mMoving = false;
			logprintf("Exiting NAV_INIT...\n");
			logprintf("Switching to %s \n",decodeNavMode(mNavMode));
			logprintf("Circular uncertainty = %1.2f\n", circUnc);
			mCalibrateTimestamp = now; // Set here to delay the first calibrate attempt
		}
		break;
	    } // End of NAV_INIT handling
	    default:
	    {
	    	if(mUsefulGPSReport && (mGPS->mPosType == GPSINS_MSG::OMNISTAR_HP))
		{
			mNavMode = NAV_OMNISTAR_HP;
		}
		else if(mUsefulGPSReport && (mGPS->mPosType == GPSINS_MSG::OMNISTAR))
		{
			mNavMode = NAV_OMNISTAR;
		}
		else
		{
			mNavMode = NAV_DEADRECKONING;
		}
		updateDRPosition();
		updateFusedPosition();
		periodicTasks(); // Perform Odometer calibration and output periodic debugging logs
	    	break;
	    }		    		
	}// End switch (mNavMode)
	
} // end of fuseGPS()


void
FusedNav::updateDRPosition()   
{
	// Check for movement and set mMoving accordingly
	// Calculate the dead reckoning position relative
	// to the current base position
	
	mTravelData = mOdoSpeedometer.getTravelData();	// read odomoter
	mOdometerValueCurrent = mTravelData.odometer;
	mOdometerDistThisGPSCycle = mOdometerValueCurrent - mOdometerValuePrevious;
	mOdometerValuePrevious = mOdometerValueCurrent;
	if(abs(mOdometerDistThisGPSCycle) > k_min_odometer_delta)
	{
		mMoving = true;
		mOdometerDistThisGPSCycle = mOdometerDistThisGPSCycle * mOdometerCalibration;
		mRelPiece[0] =  mOdometerDistThisGPSCycle * cos(mRPY[2]*M_PI/180);  // north, note: heading(yaw) in deg
		mRelPiece[1] =  mOdometerDistThisGPSCycle * sin(mRPY[2]*M_PI/180);  // east, note: heading(yaw) in deg
		mRelPiece[2] = -mOdometerDistThisGPSCycle * sin(mRPY[1]*M_PI/180);  // down, note: pitch in deg, '-' sign is due to down is positive
		mRel += mRelPiece;				// based on odometer(encoder), and AHRS yaw (heading)
		mDRPosition = mDRBase + mRel;
		for (int i=0;i<3;i++)
		{
			mDRUnc[i] += abs( mRelPiece[i] ) * k_odometer_error_per_meter;
		}		
	}
	else
	{
		mMoving = false;
	}
}
	     
        
void
FusedNav::updateFusedPosition()
{
	// Flush the position history if we switched nav Modes to DR because
	// we lost the GPS signal needed
	
	if ((mPrevNavMode != NAV_DEADRECKONING) && (mNavMode == NAV_DEADRECKONING))
	{
		mFusedPosition = mDRPosition + mFilterBuffer.averageDelta;
		clearFilterBuffer();
		mDRBase = mFusedPosition;
		mDRBaseUnc = mFusedUncertainty;
		mDRUnc = mDRBaseUnc;
		mRel = mZeroVector;
		return;
	}
	else if (!mMoving)
	{
		return;
	}
	
	// We are moving, handle the moving fused position calculations
				
	switch (mNavMode)
	{	    
	    case NAV_OMNISTAR_HP: //We have a good solid GPS signal and are navigating with it
	    {
	    	mFusedUncertainty = mUnc;
		updateFilterBuffer(); // This is where fused position is calculated for the GPS case
		break;
	    }
	    
	    case NAV_OMNISTAR: //We have a fairly good GPS signal and are navigating with it	    
	    {
		// Report the smaller value of either DR or Omnistar uncertainty
		// Circular uncertainty ignores the down component
		
		double circDRUncertainty = calcCircUncertainty( mDRUnc );
		double circOmnistarUncertainty = calcCircUncertainty( mUnc );
        
		if(circDRUncertainty > circOmnistarUncertainty)
		{
			mFusedUncertainty = mUnc;
		}
		else
		{
			mFusedUncertainty = mDRUnc;
		}
		updateFilterBuffer(); // This is where fused position is calculated for the GPS case
		break;
	    }
	    
	    case NAV_DEADRECKONING: // We are relying on the odometer and AHRS only
	    {
		mFusedPosition = mDRPosition;
		mFusedUncertainty = mDRUnc;
		// No FilterBuffer update in Dead Reckoning
		break;	    
	    }	
        
	    default:
	    {
	    	// NAV_INIT and NAV_POSITION_LOST are not handled here.
	    	logprintf("Undefined or unexpected nav mode: %d\n", mNavMode);
	    	sleep(1);
	    	exit (FUSEDNAV_SOFTWARE_ERROR); // Panic! Software error.
	    }
	} // End switch (mNavMode)
	// One last thing to do: check if our uncertainty value is still ok
	
	double circUnc = calcCircUncertainty( mFusedUncertainty );
	if(circUnc > k_max_uncertainty)
	{
		logprintf("Fused uncertainty too big: %1.2f\n",circUnc);
		mPrevNavMode = mNavMode;
		mNavPosLostTimestamp = gettimenowns();
		mNavMode = NAV_POSITION_LOST;
	}
	
} // End updateFusedPosition()
 


void
FusedNav::periodicTasks()
{
	//	Periodic debug output and odometer calibration adjustiment
	uint64_t now = gettimenowns();					// now, in nanoseconds		
	if( (gettimenowns() - mCalibrateTimestamp) *.000000001 > k_cal_wait)
	{
		mCalibrateTimestamp = now;

		logprintf("\nPosType: %s PosStat: %s\n", decodePosType(mGPS->mPosType), decodePosStat(mGPS->mPosStat));
		logprintf("NavMode: %s\n", decodeNavMode(mNavMode));
		logprintf("mOdometerDistThisGPSCycle: %1.6f\n", mOdometerDistThisGPSCycle);                                         

		logprintf("Fused Position: %1.2f %1.2f %1.2f\n", mFusedPosition[0],mFusedPosition[1],mFusedPosition[2]);                                          		                    
		logprintf("Fused Unc:      %1.2f %1.2f %1.2f\n", mFusedUncertainty[0], mFusedUncertainty[1], mFusedUncertainty[2]);
		logprintf("DRBase      :   %1.2f %1.2f %1.2f\n", mDRBase[0],mDRBase[1],mDRBase[2]);
		logprintf("DR  Position:   %1.2f %1.2f %1.2f\n", mDRPosition[0],mDRPosition[1],mDRPosition[2]);
		logprintf("Average Delta:  %1.2f %1.2f %1.2f\n", mFilterBuffer.averageDelta[0],
								mFilterBuffer.averageDelta[1],mFilterBuffer.averageDelta[2]);
		logprintf("GPS Latest:     %1.2f %1.2f %1.2f\n", mAbs[0],mAbs[1],mAbs[2]);
		logprintf("GPS Latest Hdg: %1.2f\n", mGPS->mHeading);
		logprintf("INS Latest Hdg: %1.2f\n", mRPY[2]);
		logprintf("=========\n");
		/*for (int i=0; i<FZFILTERSIZE; i++)
		{
			logprintf("Delta[%d]: %1.2f %1.2f %1.2f\n",i,mFilterBuffer.positionDelta[i][0],
			  mFilterBuffer.positionDelta[i][1],mFilterBuffer.positionDelta[i][2]);
		logprintf("Filterbuffer index: %d\n",mFilterBuffer.index);
		}*/
		logprintf("=========\n");
			
		mOdometerCalibration = getOdometerCalibration(); // Update the odometer calibration
	}	
}

/***************************************************************************************
 * Called when we have a good GPS positon and are moving.
 * Calcualte and record the fused positon that
 * includes GPS and INS blending.
 ***************************************************************************************/

void
FusedNav::updateFilterBuffer()
{
	uint16_t index = mFilterBuffer.index;
	
	// Calculate what the new Delta Average will be with this fused position
	Vector<3> tempDeltaSum = mZeroVector;
	for (int i=0; i<FZFILTERSIZE; i++)
	{
		if(i != index)
		{
			tempDeltaSum += mFilterBuffer.positionDelta[i];
		}
		else
		{
			tempDeltaSum += mAbs - mDRPosition;
		}		
	}
	Vector<3> tempDeltaAvg = (tempDeltaSum / FZFILTERSIZE);
	
	// Compare it to the current Delta Average
	// If the difference is longer than k_gps_discontinuity_distance
	// the current mAbs fails the discontinuity test
	
	Vector<3> tempVec = tempDeltaAvg - mFilterBuffer.averageDelta;
	double tempVecLen = 0;
	for (int i=0; i<2; i++) // Ignore Z
	{
		tempVecLen += tempVec[i] * tempVec[i];
	}
	tempVecLen = sqrt(tempVecLen);
	
	if(tempVecLen > k_gps_discontinuity_distance)
	{
		logprintf ("Saw large discontinuity: %1.2f\n", tempVecLen);
		mFilterBuffer.countOfSeqDiscontinuities +=1;
		if(mFilterBuffer.countOfSeqDiscontinuities >2 )
		{
			// Go into position lost state
			mNavPosLostTimestamp = gettimenowns();
			mNavMode = NAV_POSITION_LOST;                                               
			logprintf ("Entering NAV_POSITION_LOST state: too many discontinuities.\n");
			mFilterBuffer.countOfSeqDiscontinuities = 0;
			return;
		}
		else
		{
			// Keep going, but use the current average delta
			// instead of mAbs - mDRPosition for this delta
			mFilterBuffer.positionDelta[index] = mFilterBuffer.averageDelta;
			
			// Calculate the final position delta sum and average	
			mFilterBuffer.positionDeltaSum = mZeroVector;		 // Zero delta position sum first
			for (int i=0; i<FZFILTERSIZE; i++)
			{
				mFilterBuffer.positionDeltaSum += mFilterBuffer.positionDelta[i];
			}
			mFilterBuffer.averageDelta = mFilterBuffer.positionDeltaSum / FZFILTERSIZE;
		}
			
	}
	else
	{
		// Good mAbs, use it & we already know the sum and average
		mFilterBuffer.positionDelta[index] = mAbs - mDRPosition;
		mFilterBuffer.positionDeltaSum = tempDeltaSum;
		mFilterBuffer.averageDelta = tempDeltaAvg;		
	}
									
	mFusedPosition = mDRPosition + mFilterBuffer.averageDelta;

	//
	// Update fused position history
	//

	if(mFilterBuffer.countEntries < FZFILTERSIZE)
	{
		mFilterBuffer.fusedPos[index] = mFusedPosition;
		mFilterBuffer.fusedPosUnc[index] = mFusedUncertainty;
		mFilterBuffer.navMode[index] = mNavMode;
		mFilterBuffer.countEntries += 1;
	}
	else
	{
		mDRBase = mFilterBuffer.fusedPos[index];		// Drag base to the oldest fused position
		mDRBaseUnc = mFilterBuffer.fusedPosUnc[index];
		mDRUnc = mDRBaseUnc;
		mDRBaseNavMode = mFilterBuffer.navMode[index];
		mRel = mDRPosition - mDRBase;				// Adjust mRel to correspond
		mFilterBuffer.fusedPos[index] = mFusedPosition;	// Clobber the oldest fused position
		mFilterBuffer.fusedPosUnc[index] = mFusedUncertainty;
		mFilterBuffer.navMode[index] = mNavMode;
	}
	index++;		
	index = index % FZFILTERSIZE; // Wrap around if necessary
	mFilterBuffer.index = index;
	
}// End updateFilterBuffer


void
FusedNav::clearFilterBuffer()
{
	if(mFilterBuffer.countEntries == 0)
	{
		return;
	}
	
	mFilterBuffer.index = 0;
	mFilterBuffer.countEntries =0;
	mFilterBuffer.positionDeltaSum = mZeroVector;
	mFilterBuffer.averageDelta = mZeroVector;
	mFilterBuffer.countOfSeqDiscontinuities = 0;	
	
	for(int i=0; i<FZFILTERSIZE; i++)
	{
		mFilterBuffer.fusedPos[i] = mZeroVector;
		mFilterBuffer.fusedPosUnc[i] = mZeroVector;
		mFilterBuffer.navMode[i] = NAV_NOT_READY;
		mFilterBuffer.positionDelta[i] = mZeroVector;
	}	
}

double
FusedNav::getOdometerCalibration()
{

	double calibrateValue = k_odometer_calibration_factor;
	if (k_odometer_calibration_factor != 1.0)
	{
		return (k_odometer_calibration_factor);
	}
	else
	{
		calibrateValue = mOdometerCalibration;
		if( mOdometerDistThisGPSCycle < k_min_odometer_cal_delta ) // Only adjust if we are robustly moving forward
		{
			return (calibrateValue);
		}
		// We need two sequential points in OMNISTAR_HP to calculate a new value
		if( (mPrevNavMode == mNavMode) && mNavMode == NAV_OMNISTAR_HP)
		{
			double deltaN = mAbsPrevious[0]-mAbs[0];
			double deltaE = mAbsPrevious[1]-mAbs[1];
			double deltaD = mAbsPrevious[2]-mAbs[2];
			double gpsDistThisCycle = 
			  sqrt( (deltaN * deltaN) + (deltaE * deltaE) + (deltaD * deltaD) );
			calibrateValue = gpsDistThisCycle / mOdometerDistThisGPSCycle;
			calibrateValue = (calibrateValue + (99.0*mOdometerCalibration))/100.0; // Smooth the result
			logprintf("New Odometer Calibration: %1.4f\n", calibrateValue);
		}
		return (calibrateValue);
	}
	
} // end of FusedNav::getOdometerCalibration()

	

//
//	convert  -- type conversion
//
inline void convert(double out[3], const Vector<3>& in)
{	out[0] = in[0];
	out[1] = in[1];
	out[2] = in[2];
}
//
//	convert  -- type conversion
//
inline void convert(float out[3], const Vector<3>& in)
{	out[0] = in[0];
	out[1] = in[1];
	out[2] = in[2];
}

/*************************************************************************
 *	copyState  -- copy FusedNav positions to output message
 *
 *	Called by handle_request() in server-thread.
 *	Report status rep.err is set here as follows:
 *
 *	NAV Mode		rep.err
 *	--------		-------
 *	NAV_INIT		INIT
 *	NAV_OMNISTAR_HP		OK
 *	NAV_OMNISTAR		OK
 *	NAV_DEADRECKONING	OK
 *	NAV_POS_LOST		INIT
 *
 *	mRepTimestamp is checked for age of useful GPS information.
 *	If it is older than 100ms, a new fused position  is calculated
 *	based on dead reckoning and that is returned rather than
 *	the older one based on GPS info.
 * 
 ************************************************************************/
void
 FusedNav::copyState(GPSINSMsgRep& rep)
{
	
	ost::MutexLock lok(m_lock);
	rep.timestamp = mRepTimestamp;  // Get the timestamp from the most recent useful GPS report
	uint64_t now = gettimenowns();
	switch (mNavMode)
	{
	    case NAV_INIT:
	    case NAV_POSITION_LOST:
	    {
		rep.err = GPSINS_MSG::INITIALIZATION;
		break;
	    }
	    case NAV_OMNISTAR_HP:
	    case NAV_OMNISTAR:
	    case NAV_DEADRECKONING:
	    default:
	    {
		uint64_t tdiff = now - rep.timestamp;
		double tdiffSecs = tdiff * 0.000000001;
 		if (tdiffSecs > k_max_gps_delay)		// Don't use GPS data that is too old
 		{	    	
			updateDRPosition();
			mPrevNavMode = mNavMode;
			mNavMode = NAV_DEADRECKONING;
			updateFusedPosition();
			rep.timestamp = now;
		}
			rep.err = GPSINS_MSG::OK;
	    }
	}// End switch
	
	rep.posStat = mGPS->mPosStat;
	rep.posType = mGPS->mPosType;
	convert(rep.llh ,getLLH());																// latitude, longitude, height
	convert(rep.pos, getFusedPositionNED());
	convert(rep.unc, getFusedUncertainty());
	convert(rep.vel, getVelocity());
	convert(rep.acc, getAcceleration());
	convert(rep.rpy, getRPY());
	convert(rep.pqr, getAngularRates());
} // End copyState

//
//	getFusedUncertainty  -- get fused uncertainty value
//
//	This is one of the key outputs from Fusednav.  The
//	location should be within the specified distance a high percentage of the time.
//
Vector<3> FusedNav::getFusedUncertainty() const
{
	Vector<3> uncertainty;
	uncertainty = mFusedUncertainty;
	return (uncertainty);
}

double
FusedNav::calcCircUncertainty( Vector<3> unc)
{
	return ( sqrt((unc[0]*unc[0]) + (unc[1]*unc[1])) );
}


//
//	getLLH  -- 	get latitude, longitude, height from GPS only
//
Vector<3> FusedNav::getLLH() const
{
	Vector<3> llh = ECEF2llh(mGPS->mXYZ);
								// get lat, long from GPS ECEF coords.
	llh[0] = llh[0] * C_RAD2DEG;				// lat and long in degrees
	llh[1] = llh[1] * C_RAD2DEG;
	return(llh);
}

