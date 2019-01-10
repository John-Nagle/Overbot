//
//	interpolatelib.h -- interpolation library
//
//	Useful interpolation routines.
//
//
//	Anil Punjabi
//	Team Overbot
//	April, 2005
//
#ifndef INTERPOLATELIB_H
#define INTERPOLATELIB_H
#include "algebra3.h"
#include "geocoords.h"

//
//	 Interpolates a 4x4 matrix using LERP given time at the 2 boundary points 
bool interpolateLERP( const mat4& prevmat, uint64_t prevtime, const mat4& nextmat, uint64_t nexttime,
	uint64_t wantedtime, mat4& wantedmat);


#endif // INTERPOLATELIB_H
