//
//	Rotations test for Euler angles
//
#include <stdio.h>
#include "algebra3.h"
#include "eulerangle.h"
#include "geocoords.h"

//
//	deg2radians  -- degrees to radians conversion
//
/*
static double deg2radians(double deg)
{	return(deg * (M_PI/180.0));	}
*/
//
//	buildmatrix  -- build matrix from Euler angles
//
static void buildmatrix(const vec3& angs, mat4& vehpose)
{
	Eul_ToHMatrix(angs,vehpose,EulOrdXYZs);							// This is the correct axis order per Crossbow AHRS 400 manual.
}
//
//	dumpmatrix -- dump info from matrix
//
static void dumpmatrix(const mat4& mat)
{
	printf("Dump of matrix.\n");
	for (int i=0; i<4; i++)
	{	printf("%7.4f %7.4f %7.4f %7.4f\n", mat[i][0], mat[i][1], mat[i][2], mat[i][3]);	}
}
//
//	trymatrix -- multiply matrix tiimes vector and dump
//
static void trymatrix(vec3& ptin, mat4& mat)
{
	vec3 ptout = mat * ptin;
	printf("Point (%1.2f, %1.2f, %1.2f) transformed to  (%1.2f, %1.2f, %1.2f)\n",
		ptin[0], ptin[1], ptin[2], ptout[0], ptout[1], ptout[2]);
}

//
//	usage  -- standard usage
//
static void usage()
{	printf("Usage:  rottest roll pitch yaw <roll2> <pitch2> <yaw2>\n");
	exit(1);
}
//
//	main program
//
int main(int argc, const char* argv[])
{
	if (argc < 4) usage();
	bool interpolate = false;
	double roll, pitch, yaw;
	double roll2, pitch2, yaw2;  //  for testing interpolation
	
	sscanf(argv[1], "%lf", &roll);
	sscanf(argv[2], "%lf", &pitch);
	sscanf(argv[3], "%lf", &yaw);
	if (argc == 7) {
	    interpolate = true;
		sscanf(argv[4], "%lf", &roll2);
		sscanf(argv[5], "%lf", &pitch2);
		sscanf(argv[6], "%lf", &yaw2);
	}
	printf("Roll %1.4f deg.  Pitch %1.4f deg.  Yaw %1.4f deg.\n", roll, pitch, yaw);
	//	Values have been set. Now build the matrix.
	//
	//const vec3 translation(1000, 2000, 3000);					// big dummy translation
	const vec3 translation(0,0,0);					// big dummy translation
	float xrot = deg2radians(roll);										// x rotation, roll, axis is vehicle forward, zero is flat
	float yrot = -deg2radians(pitch);									// y rotation, pitch must flipped
	float zrot = (M_PI*0.5)-deg2radians(yaw);					// z rotation, axis is up, aimed in +X is zero 
	vec3 rpy(xrot, yrot, zrot);
	mat4 rotmat;
	printf("Rotation matrix,\n");
	buildmatrix(rpy, rotmat);
	dumpmatrix(rotmat);
	mat4 transmat;
	printf("Translation matrix,\n");
	transmat = translation3D(translation);							// translation matrix
	dumpmatrix(transmat);
	mat4 mat = transmat * rotmat;
	printf("Combined matrix,\n");
	dumpmatrix(mat);
	//	Test some vectors
	vec3 pt0(1/sqrt(3.0),1/sqrt(3.0),1/sqrt(3.0));
	trymatrix(pt0, mat);
	vec3 ptx(1,0,0);
	vec3 pty(0,1,0);
	vec3 ptz(0,0,1);
	trymatrix(ptx, mat);
	trymatrix(pty, mat);
	trymatrix(ptz, mat);
	vec3 pt1(1/sqrt(2.0),1/sqrt(2.0),0);
	trymatrix(pt1, mat);
	
	if (interpolate) {
		float xrot2 = deg2radians(roll2);										// x rotation, roll, axis is vehicle forward, zero is flat
		float yrot2 = -deg2radians(pitch2);									// y rotation, pitch must flipped
		float zrot2 = (M_PI*0.5)-deg2radians(yaw2);					// z rotation, axis is up, aimed in +X is zero 
		vec3 rpy2(xrot2, yrot2, zrot2);
		
		// create pose matrix for next rpy values
		mat4 nextmat;
		buildmatrix(rpy2, nextmat);
		nextmat = transmat * nextmat;  // assume we don't translate between poses for this test
		printf("Next pose matrix,\n");
		dumpmatrix(nextmat);
		
		// perform interpolation
		mat4 newmat;
		uint64_t prevtime = 0;
		uint64_t nexttime = 100;
		uint64_t mytime = 50;  // halfway
		 
		interpolateposedumb(mat, prevtime,nextmat, nexttime, mytime, newmat);
		printf("Interpolated pose matrix:\n");
		dumpmatrix(newmat);
		interpolateposeSLERP(mat, prevtime,nextmat, nexttime, mytime, newmat);
		printf("SLERP  pose matrix:\n");
		dumpmatrix(newmat);
	}
	
	return(0);
}
