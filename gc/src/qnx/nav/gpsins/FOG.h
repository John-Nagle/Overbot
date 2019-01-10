/* -*- indent-tabs-mode:T; c-basic-offset:8; tab-width:8; -*- vi: set ts=8:
 *
 * Khian Hao Lim
 * Oct 03
 * Overbot
 *
 * Object that encapsulates all the data from the FOG
 *
 */
#ifndef FOG_H
#define FOG_H
class Kalman_Filter;

class FOG
{
public:
	FOG(char * fogDev, char * fogLog);
	~FOG();
	
	char *			mDev;
	double          mR;//yaw rate

	Kalman_Filter * mKF;
private:
};

#endif
