/* -*- indent-tabs-mode:T; c-basic-offset:8; tab-width:8; -*- vi: set ts=8:
 *
 * Khian Hao Lim
 * Oct 03
 * Overbot
 *
 * Object that encapsulates all the data and states of the kalman filter
 */

#include <Kalman_Filter.h>

Kalman_Filter::Kalman_Filter(
			  char *		ahrsDev,
			  char *        ahrsLog,
			  char *		fogDev,
			  char *		fogLog,
			  char *		gpsDev,
			  char *		gpsLog,
			  char * 		kfLog
			  ) :
	mAHRS					(ahrsDev, ahrsLog),
	mFOG                    (fogDev, fogLog),
	mGPS                    (gpsDev, gpsLog),
	mMETER                  ( ),
	mTime					(0),
	mDt						(0),
	mLogFd					(0)
{
	mAHRS.mKF = this;
	mFOG.mKF = this;
	mGPS.mKF = this;
	mMETER.mKF = this;
	reset();
	
	if (kfLog) {
		mLogFd = open( kfLog, O_WRONLY | O_CREAT, 0666 );

		if( mLogFd < 0  && verbose) 
		{
				perror( kfLog );
		}
	}

}

void
Kalman_Filter::log(char* line, int len)
{
	if (mLogFd) {
		struct timeval t;
		gettimeofday(&t, 0);
		write(mLogFd, (char*)&t, sizeof(t));
		write(mLogFd, line, len);
	}
		
}

double
Kalman_Filter::getTrace()
{
	double trace = 0;
	for (int i = 0; i < NUMSTATES ;i++) {
		trace += P[i][i];
	}
	return trace;
}

void
Kalman_Filter::predictor()
{
	ost::MutexLock lok(mLock);

	char line[512];
    struct timespec tp;

	clock_gettime(CLOCK_REALTIME, &tp);
    state.timestamp = timespec2nsec(&tp);
	state.rpy[0] = mAHRS.mRPY[0];
	state.rpy[1] = mAHRS.mRPY[1];
	state.rpy[2] = mAHRS.mRPY[2];

	state.pqr[0] = mAHRS.mPQR[0];
	state.pqr[1] = mAHRS.mPQR[1];
	state.pqr[2] = mAHRS.mPQR[2];
	
	Vector <3> angles (state.rpy[0] * C_DEG2RAD,
						state.rpy[1] * C_DEG2RAD,
						state.rpy[2] * C_DEG2RAD);
	
	const Matrix<3,3> cBE(eulerDC(angles));
	Vector<3> earthFrameAcc = cBE.transpose() * mAHRS.mACC;
	Vector<3> gVector (0,0,9.81);
	earthFrameAcc -= gVector;
	
	//we will store earth frame accelerations in state
//	state.acc[0] = earthFrameAcc[0];
//	state.acc[1] = earthFrameAcc[1];
//	state.acc[2] = earthFrameAcc[2];

	state.acc[0] = earthFrameAcc[1];
	state.acc[1] = -earthFrameAcc[0];
	state.acc[2] = -earthFrameAcc[2];


	sprintf(line,"%3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f\n",
			state.rpy[0], state.rpy[1], state.rpy[2],
			state.pqr[0], state.pqr[1], state.pqr[2],
			state.acc[0], state.acc[1], state.acc[2],
			state.vel[0], state.vel[1], state.vel[2],
			state.pos[0], state.pos[1], state.pos[2]);
	
	log((char*)line, strlen(line));

	//do kf prediction update
	Matrix<NUMSTATES, NUMSTATES> A;
	makeAMatrix(A);
	propagateState();
	propagateCovariance(A);
	
	//copy from kfState into state	
	state.pos[0] = kfState[0];
	state.pos[1] = kfState[1];
	state.pos[2] = kfState[2];
	state.vel[0] = kfState[3];
	state.vel[1] = kfState[4];
	state.vel[2] = kfState[5];

	if (verbose)
		printf("trace: %f\n", getTrace());
}

void
Kalman_Filter::corrector(bool good)
{
	ost::MutexLock lok(mLock);
    struct timespec tp;

	clock_gettime(CLOCK_REALTIME, &tp);
    state.timestamp = timespec2nsec(&tp);
	state.posStat = mGPS.mPosStat;
	state.posType = mGPS.mPosType;

	if (good) {
/*
	state.unc[0] = mGPS.mXYZsd[0];
	state.unc[1] = mGPS.mXYZsd[1];
	state.unc[2] = mGPS.mXYZsd[2];
*/
	state.unc[0] = mGPS.mNEDsd[0];
	state.unc[1] = mGPS.mNEDsd[1];
	state.unc[2] = mGPS.mNEDsd[2];
		
	
	
	//do kf corrector step
	Matrix <NUMSTATES, NUMSTATES> C;
	makeCMatrix(C);
	Vector<NUMSTATES> err;
	/*
	err[0] = mGPS.mXYZ[0] - kfState[0];
	err[1] = mGPS.mXYZ[1] - kfState[1];
	err[2] = mGPS.mXYZ[2] - kfState[2];
	err[3] = mGPS.mXYZVel[0] - kfState[3],
	err[4] = mGPS.mXYZVel[1] - kfState[4],
	err[5] = mGPS.mXYZVel[2] - kfState[5];
	*/
	err[0] = mGPS.mNED[0] - kfState[0];
	err[1] = mGPS.mNED[1] - kfState[1];
	err[2] = mGPS.mNED[2] - kfState[2];
	err[3] = mGPS.mNEDVel[0] - kfState[3],
	err[4] = mGPS.mNEDVel[1] - kfState[4],
	err[5] = mGPS.mNEDVel[2] - kfState[5];

	// We throw away the K result
	Matrix<NUMSTATES,NUMSTATES>		K;
	Kalman(P,kfState,C,R,err,K);
	//copy from kfState into state
	state.pos[0] = kfState[0];
	state.pos[1] = kfState[1];
	state.pos[2] = kfState[2];
	state.vel[0] = kfState[3];
	state.vel[1] = kfState[4];
	state.vel[2] = kfState[5];


	//TODO
	//	***TO BE CHECKED BY FRANK ZHANG***
	//really this should be a function that goes from NED to llh
	//need to write that
	//predictor will have to do this too
	Vector<3> llh = ECEF2llh(mGPS.mXYZ);
	state.llh[0] = llh[0] * C_RAD2DEG;
	state.llh[1] = llh[1] * C_RAD2DEG;
	state.llh[2] = llh[2];
	
	}
}

void
Kalman_Filter::copyState(struct GPSINSMsgRep * rep)
{
	
	ost::MutexLock lok(mLock);
	memcpy(rep, &state, sizeof(struct GPSINSMsgRep));

}

bool
Kalman_Filter::step ()
{
   struct timeval tv;

	fd_set fds;
	int rc, max_fd;
	max_fd = 0;
	
	FD_ZERO(&fds);

    // set the timeout
    tv.tv_sec = 0;
    tv.tv_usec = 99999;
 	
	if (mGPS.mSerial) {
		FD_SET(this->mGPS.mSerial->fd, &fds);
		if (this->mGPS.mSerial->fd > max_fd) {
			max_fd = this->mGPS.mSerial->fd;
		}
	}
	
	if (mAHRS.mSerial) {
		FD_SET(this->mAHRS.mSerial->fd, &fds);
		if (this->mAHRS.mSerial->fd > max_fd) {
			max_fd = this->mAHRS.mSerial->fd;
		}
	}

	rc = select (max_fd + 1, &fds, 0, 0, &tv);
	if (rc <= 0) {
		if (verbose) {
			perror("select in Kalman_Filter::step timed out");
		}
		return false;
	}
	
	if (FD_ISSET(this->mGPS.mSerial->fd, &fds)) {
		mGPS.step();
	}
	if (FD_ISSET(this->mAHRS.mSerial->fd, &fds)) {
		mAHRS.step();
	}
	return true;

}


void
Kalman_Filter::makeAMatrix(
			     Matrix<NUMSTATES,NUMSTATES> &		A
			     )
{
	A.fill();
	
	//d (Vel[0] / d(Pos[i] == 0, d(Vel[i]/ d(Vel[i] == 1 where i = 0, otherwise 0
	A[0][3] =    1; //d(Vel[0]) / d(Vel[0))
	A[1][4] =    1; //d(Vel[1]) / d(Vel[1))
	A[2][5] =    1; //d(Vel[2]) / d(Vel[2))
	//d (Acc[i] / ..) == 0;

}

void
Kalman_Filter::makeCMatrix (Matrix <NUMSTATES,NUMSTATES> & C)
{
	C.fill();
	C = eye<NUMSTATES, double>(); //C is really the identity in this case
	
}

void
Kalman_Filter::propagateState()
{
	Vector< NUMSTATES> kfStateDot;
	kfStateDot[0] = kfState[3];
	kfStateDot[1] = kfState[4];
	kfStateDot[2] = kfState[5];
	kfStateDot[3] = state.acc[0];
	kfStateDot[4] = state.acc[1];
	kfStateDot[5] = state.acc[2];
	
	kfState += kfStateDot * mAHRS.mDt;
}


void
Kalman_Filter::propagateCovariance(const Matrix<NUMSTATES,NUMSTATES> &	A)
{
	Matrix<NUMSTATES,NUMSTATES>		Pdot
			= A*P + P*A.transpose() + Q;
	P += Pdot * mAHRS.mDt;
}

void
Kalman_Filter::reset()
{
	
	
	kfState.fill();
	
	//arbitrary first
	P			= eye<NUMSTATES,double>();
	
	Q.fill();
	Q[0][0] = 1.0; //arbitrary first
	Q[1][1] = 1.0;
	Q[2][2] = 1.0;
	Q[3][3] = 1.0;
	Q[4][4] = 1.0;
	Q[5][5] = 1.0;

	//we trust the gps quite a lot
	R.fill();
	
	R[0][0] = 0.01; //0.1 m * 0.1m
	R[1][1] = 0.01;
	R[2][2] = 0.01;
	R[3][3] = 0.01;
	R[4][4] = 0.01;
	R[5][5] = 0.01;

	//for testing, set gps weight lower
	/*
	R[0][0] = 100;
	R[1][1] = 100;
	R[2][2] = 100;
	R[3][3] = 100;
	R[4][4] = 100;
	R[5][5] = 100;
	*/
}
