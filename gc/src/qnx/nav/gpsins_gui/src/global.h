/* Header "global.h" for gpsins_gui Application */

#ifndef GLOBAL_H
#define GLOBAL_H

#include <photon/realtime/RtTimer.h>
#include <../../../../common/include/gpsins_messaging.h>
#include <math.h>
#include <../../../../common/include/Conversions.h>
#include <vector>
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"
#include <sys/time.h>

using namespace std;

//declared in positionViewDraw
extern vector <struct GPSINSMsgRep> poses;
extern bool mPause;
extern float inc;
extern double posZero[3];
extern struct timeval startTime;
extern int g_waypointoutfd;
extern int g_waypointnum;

#define UPDATEFREQ 10 //times per sec
#define MAXBUFSIZE 1000

//angle in deg
inline void rotate (int x, int y, short int * new_x, short int * new_y, double angle)
{
	angle = angle * C_DEG2RAD;
	*new_x = (short int) (cos (angle)  * x - sin (angle) * y);
	*new_y = (short int) (sin (angle) * x + cos (angle) * y);
}


inline
void updateZoom()
{

		double zoomVals [9] = {inc * -4, inc * -3, inc * -2, inc * -1, inc *0,
									     inc * 1, inc * 2, inc *3, inc *4};
		
		PtSetResource (ABW_valZoomX0, Pt_ARG_NUMERIC_VALUE, &zoomVals[0], 0); 
		PtSetResource (ABW_valZoomX1, Pt_ARG_NUMERIC_VALUE, &zoomVals[1], 0); 
		PtSetResource (ABW_valZoomX2, Pt_ARG_NUMERIC_VALUE, &zoomVals[2], 1); 
		PtSetResource (ABW_valZoomX3, Pt_ARG_NUMERIC_VALUE, &zoomVals[3], 1); 
		PtSetResource (ABW_valZoomX4, Pt_ARG_NUMERIC_VALUE, &zoomVals[4], 1); 
		PtSetResource (ABW_valZoomX5, Pt_ARG_NUMERIC_VALUE, &zoomVals[5], 1); 
		PtSetResource (ABW_valZoomX6, Pt_ARG_NUMERIC_VALUE, &zoomVals[6], 1); 
		PtSetResource (ABW_valZoomX7, Pt_ARG_NUMERIC_VALUE, &zoomVals[7], 1); 
		PtSetResource (ABW_valZoomX8, Pt_ARG_NUMERIC_VALUE, &zoomVals[8], 1); 
		
		PtSetResource (ABW_valZoomY0, Pt_ARG_NUMERIC_VALUE, &zoomVals[0], 1); 
		PtSetResource (ABW_valZoomY1, Pt_ARG_NUMERIC_VALUE, &zoomVals[1], 1); 
		PtSetResource (ABW_valZoomY2, Pt_ARG_NUMERIC_VALUE, &zoomVals[2], 1); 
		PtSetResource (ABW_valZoomY3, Pt_ARG_NUMERIC_VALUE, &zoomVals[3], 1); 
		PtSetResource (ABW_valZoomY4, Pt_ARG_NUMERIC_VALUE, &zoomVals[4], 1); 
		PtSetResource (ABW_valZoomY5, Pt_ARG_NUMERIC_VALUE, &zoomVals[5], 1); 
		PtSetResource (ABW_valZoomY6, Pt_ARG_NUMERIC_VALUE, &zoomVals[6], 1); 
		PtSetResource (ABW_valZoomY7, Pt_ARG_NUMERIC_VALUE, &zoomVals[7], 1); 
		PtSetResource (ABW_valZoomY8, Pt_ARG_NUMERIC_VALUE, &zoomVals[8], 1); 

}

#endif