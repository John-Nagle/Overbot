/////////////////////////////////////////////////////////////////////////////
//
//    File: check_linearreg.cc
//
//    Usage:
//        check_linearreg
//
//    Description:
//        A test file for linearreg.cc.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        July, 2003
//
////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <float.h>
#include <math.h>
#include "linearreg.h"

// made up numbers
//#define POINT_NUM	(4)
//float x[] = {4.0,6.0,8.0,9.0};
//float y[] = {5.0,8.0,10.0,12.0};

// MVP2001 HPD Current Limit Calibration
//#define POINT_NUM	(21)//(22)
//float x[] = {0, 0.2, 0.4, 0.8, 1.3, 1.7, 2.2, 2.8, 3.5, 4.5,
//	         5.3, 6.2, 7.3, 8.2, 9.7, 11.3, 12.8, 14.8, 16, 16.6,
//	         18.2};//,18.2};
//float y[] = {2000, 2100, 2200, 2300, 2400, 2500, 2600, 2700, 2800, 2900,
//	         3000, 3100, 3200, 3300, 3400, 3500, 3600, 3700, 3800, 3900,
//	         4000};//, 4095};
// piecewise calibration
//#define POINT_NUM	(3)
//float x[] = {0, 0.2, 0.4};
//float y[] = {2000, 2100, 2200};
//#define POINT_NUM  (4)
//float x[] = {0.4, 0.8, 1.3, 1.7};
//float y[] = {2200, 2300, 2400, 2500};
//#define POINT_NUM (4)
//float x[] = {1.7, 2.2, 2.8, 3.5};
//float y[] = {2500, 2600, 2700, 2800};
#if (0)
#define POINT_NUM (4)
float x[] = {3.5, 4.5, 5.3, 6.2};
float y[] = {2800, 2900, 3000, 3100};
#endif

#if (0)
#define POINT_NUM (4)
float x[] = {6.2, 7.3, 8.2, 9.7};
float y[] = {3100, 3200, 3300, 3400};
#endif

#if (0)
#define POINT_NUM (4)
float x[] = {9.7, 11.3, 12.8, 14.8};
float y[] = {3400, 3500, 3600, 3700};
#endif

#if (1)
#define POINT_NUM (4)
float x[] = {14.8, 16, 16.6, 18.2};
float y[] = {3700, 3800, 3900, 4000};
#endif

int main(int argc, char *argv[]) {
	int i,j;
	LinearRegression<float> l;
	
	for ( j=0; j<2; j++ ) {
		for ( i=0; i<POINT_NUM; i++ ) {
			printf("Adding point #%d: %f, %f\n", i+1, x[i], y[i]);
			l.PointAdd(x[i], y[i]);
			printf("Now have %ld points.\n", l.PointNum());
			printf("CalcsReady = %s\n", l.CalcsReady() ? "true" : "false");
			if ( l.CalcsReady() ) {
				printf("Slope = %f\n", l.Slope());
				printf("Offset = %f\n", l.Offset());
				printf("CoefCorr = %f\n", l.CoefCorr());
				printf("ErrStd = %f\n", l.ErrStd());
			}
			printf("\n");
		}
		printf("Resetting linear regression.\n\n");
		l.PointReset();
	}
	
	return 0;
}