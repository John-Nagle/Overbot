//
// Test tilt correct
//
#include <stdio.h>
#include "algebra3.h"
#include "eulerangle.h"
#include "LMStiltcorrect.h"
#include "LMSmapupdate.h"

//
//	deg2radians  -- degrees to radians conversion
//
/*
static double deg2radians(double deg)
{	return(deg * (M_PI/180.0));	}
*/

//
//	main program
//
int main(int argc, const char* argv[])
{
    LMSmapUpdater *dummyptr; // just a hack to get it to compile
	LMStiltCorrector tiltcorrector(*dummyptr);
	mat4 dummy;
	
	for (int i=0; i<20; i++) {
		float intilt = 0.5 + (float)i/100.0;
		float outtilt;
		float range = 100.0 + rand() % 6;
		tiltcorrector.correctTilt(intilt, range, dummy, 0, 0, outtilt);
		printf("Intilt=%f, Range=%f, Outtilt=%f\n", intilt, range, outtilt); 
	}	

	return(0);
}
