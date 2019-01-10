/* -*- indent-tabs-mode:T; c-basic-offset:8; tab-width:8; -*- vi: set ts=8:
 *
 * Khian Hao Lim
 * Oct 03
 * Overbot
 *
 * Object that encapsulates all the data and states of the kalman filter
 */
#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <cstring>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "Conversions.h"

#include "Matrix.h"
#include "Matrix_Invert.h"
#include "Vector.h"

#include "macros.h"

#include "GPS.h"
#include "AHRS.h"
#include "timer.h"
#include <iostream>
#include "map"
#include "thread.h"

#define BUF_SIZE 256

using namespace std;
class FusedNav;																	// forward

class Kalman_Filter
{
public:

	ost::Mutex  mLock;
	AHRS	    mAHRS;
	GPS	    mGPS;
	FusedNav  * mFusedNav;
	double			mTime;
	const 	double		mDt;
	int mLogFd;
	//this should be the world frame
	struct GPSINSMsgRep state; //the state that will get sent out

	Kalman_Filter(
		      char*			ahrsDev,
		      char*			ahrsLog,
		      char*			gpsDev,
		      char*			gpsLog,
		      char*			kfLog
	);

	//keep compiler happy
	Kalman_Filter(Kalman_Filter & kf):
				mLock(),
			mAHRS("", ""),
			mGPS("", "")

	{
	}
	
	double
	getTrace();
	
	void
	log(char * line, int len);
	
	void
	predictor();

	void
	corrector(bool good);
	
	bool
	step( );

	void
	copyState( struct GPSINSMsgRep * rep);

	void reset();

	//kalman filtering state
	static const int	NUMSTATES = 6;

	void makeAMatrix(Matrix<NUMSTATES,NUMSTATES> &	A);
	void makeCMatrix(Matrix<NUMSTATES,NUMSTATES> &  C);

	void propagateState();

	void propagateCovariance(const Matrix<NUMSTATES,NUMSTATES> &	A);

private:
	
	/*
	 *  Our Kalman filter uses its own state object that we
	 * hide here.
	 */
	 
	//this should all be world frame
	Vector<NUMSTATES>		kfState; //pos[0], pos[1], pos[2], vel[0], vel[1], vel[2]

	// Covariance matrix
	Matrix<NUMSTATES,NUMSTATES>		P;

	// covariance of dynamic disturbance noise (accels)
	Matrix<NUMSTATES,NUMSTATES>		Q;

	// convariance of gps sensor noise
	Matrix<NUMSTATES,NUMSTATES>		R;

	// Wrapper to serialize our state and jump into the kalman filter
	template<
		int			m
	>
	void
	do_kalman(
		const Matrix<m,NUMSTATES> &	C,
		const Matrix<m,m> &	R,
		const Vector<m> &	eTHETA
	);

};

#endif //KALMAN_FILTER_H
