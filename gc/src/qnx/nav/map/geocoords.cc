//
//	geocoords.cc -- geometry of GPS/INS fixes
//
//	John Nagle
//	Team Overbot
//	February, 2005
//
//
//	Copyright 2005 by John Nagle
//	999 Woodland Avenue
//	Menlo Park, CA  94025
//
//	This program is free software; you can redistribute it and/or modify
//	it under the terms of the GNU General Public License as published by
//	the Free Software Foundation; either version 2 of the License, or
//	(at your option) any later version.

//	This program is distributed in the hope that it will be useful,
//	but WITHOUT ANY WARRANTY; without even the implied warranty of
//	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//	GNU General Public License for more details.

//	You should have received a copy of the GNU General Public License
//	along with this program; if not, write to the Free Software
//	Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//
#include "algebra3aux.h"
#include "eulerangle.h"
#include "geocoords.h"
#include "quaternion.h"
#include "gpsins_messaging.h"
#include "Nav.h"

//
//	XYZfromNED  -- converts North, East, Down coordinates to X,Y,Z coordinates
//
//	fix is NED (North, East, Down) in meters
//	position is X(East), Y(North) Z(up)
//
//	Z is "up" to keep the matrices right-handed.
//
vec3 XYZfromNED(const Vector<3>& ned)								// using Vector input
{	return(vec3(ned[1],ned[0],-ned[2]));	}
vec3 XYZfromNED(const double ned[3])									// using array input
{	return(vec3(ned[1],ned[0],-ned[2]));	}
//
//	XYZfromLLH  -- XYZ from latitude, longitude, height
//
//	Requires a "base point" in LLH form.
//
//	One can argue whether the tangent plane used for measurement
//	should be tangent at the origin or at the point being input.
//	Because the GPSINS server currently does this at the origin,
//	we do that here to be consistent.	
//
vec3 XYZfromLLH(const Vector<3>& llh, const Vector<3>& llhorigin)
{
    Vector<3> ned = LLH_dist(llhorigin, llh, llhorigin[0], llhorigin[1]);			// convert from LLH to NED 
    return(XYZfromNED(ned));													// convert from North, East, Down to X,Y
}
vec3 XYZfromLLH(const double llh[3], const Vector<3>& llhorigin)
{	Vector<3> llhvec(llh[0], llh[1], llh[2]);									// reformat
	return(XYZfromLLH(llhvec, llhorigin));									// convert
}
//
//	posefromfix  -- gets a pose matrix from a GPS/INS fix
//
//	AHRS coordinate system:
//		X is forward. 
//		Y is to the right
//		Z is down. Z rotation (yaw) is zero when pointing north, and 90 degrees when pointing east.
//
//	Vehicle coordinate system:
//		X is forward
//		Y is left
//		Z is up.
//
//	World coordinate system:
//		X is East
//		Y is North
//		Z is up.
//
//	Both systems are right-handed.
//
void posefromfix(const GPSINSMsgRep& fix, mat4& vehpose)
{																								// fix is NED (North, East, Down) in meters
																								// position is X(East), Y(North) Z(Up)
	vec3 position(XYZfromNED(fix.pos));									// translation from NED coords
	float xrot = deg2radians(fix.rpy[0]);										// x rotation, roll, axis is vehicle forward, zero is flat
	float yrot = -deg2radians(fix.rpy[1]);									// y rotation, pitch must flipped
	float zrot = (M_PI*0.5)-deg2radians(fix.rpy[2]);					// z rotation, axis is up, aimed in +X is zero 
	//	Get pose at fix
	const EulerAngles angs(xrot, yrot, zrot);								// Roll, pitch, yaw in vehicle axis system	
	Eul_ToHMatrix(angs,vehpose,EulOrdXYZs);							// This is the correct axis order per Crossbow AHRS 400 manual.
	vehpose = translation3D(position)*vehpose;						// apply translation
}
//
//	interpolateangle  -- interpolate between two angles in radians
//
inline float interpolateangle(float prevang, float nextang, float fract)
{
	vec2 prevdir(cos(prevang),sin(prevang));						// convert angle to unit vector
	vec2 nextdir(cos(nextang),sin(nextang));						// convert angle to unit vector
	vec2 dir(nextdir*fract + prevdir*(1.0-fract));					// interpolate yaw by weighted vector addition
	return(atan2(dir[1], dir[0]));												// convert vector back to angle
}
//
//	interpolateposedumb  -- interpolate poses, dumb version
//
//	Temporary version until Anll's code works.
//	This is too slow and has too many trig functions.
//	Also, interpolating angles separately introduces errors for big changes between poses.
//

bool interpolateposedumb(const mat4& prevpose, const uint64_t prevtime, const mat4& nextpose, const uint64_t nexttime, 
	const uint64_t desiredtime, mat4& pose)
{
	if (nexttime <= prevtime) return(false);								// if not increasing timestamps, fail
	if (desiredtime < prevtime) return(false);								// out of range
	if (desiredtime > nexttime) return(false);								// out of range
	uint64_t elapsed = nexttime - prevtime;								// elapsed time (nanoseconds)
	uint64_t intointerval = desiredtime - prevtime;						// time into interval
	const float fract = float(intointerval) / float(elapsed);			// fraction of time into interval (in range 0..1)
	const EulerAngles prevangs(Eul_FromHMatrix(prevpose, EulOrdXYZs));	// extract angles
	const EulerAngles nextangs(Eul_FromHMatrix(nextpose, EulOrdXYZs));	// extract angles
	vec3 pprev(ExtractTranslation(prevpose));							// position at start of interval
	vec3 pnext(ExtractTranslation(nextpose));							// position at end of interval
	vec3 position = pnext*fract + pprev*(1.0-fract);					// linearly interpolate position
	//	Interpolate angles individually.
	const float roll = interpolateangle(prevangs[0], nextangs[0], fract);
	const float pitch = interpolateangle(prevangs[1], nextangs[1], fract);
	const float yaw = interpolateangle(prevangs[2], nextangs[2], fract);
	const EulerAngles angs(roll, pitch, yaw);								// Roll, pitch, yaw (input is relative to N, CW).
	Eul_ToHMatrix(angs,pose,EulOrdXYZs);									// Order is roll, pitch, yaw, sequentially, per AHRS 400 manual.
	pose = translation3D(position)*pose;									// apply translation
	return(true);																			// success
}

bool interpolateposeSLERP(const mat4& prevpose, const uint64_t prevtime, const mat4& nextpose, const uint64_t nexttime, 
	const uint64_t desiredtime, mat4& pose)
{
	if (nexttime <= prevtime) return(false);								// if not increasing timestamps, fail
	if (desiredtime < prevtime) return(false);								// out of range
	if (desiredtime > nexttime) return(false);								// out of range
	uint64_t elapsed = nexttime - prevtime;								// elapsed time (nanoseconds)
	uint64_t intointerval = desiredtime - prevtime;						// time into interval
	const float fract = float(intointerval) / float(elapsed);			// fraction of time into interval (in range 0..1)
	const EulerAngles prevangs(Eul_FromHMatrix(prevpose, EulOrdXYZs));	// extract angles
	const EulerAngles nextangs(Eul_FromHMatrix(nextpose, EulOrdXYZs));	// extract angles

	// interpolate position	
	vec3 pprev(ExtractTranslation(prevpose));							// position at start of interval
	vec3 pnext(ExtractTranslation(nextpose));							// position at end of interval
	vec3 position = pnext*fract + pprev*(1.0-fract);					// linearly interpolate position
	
	//	Interpolate angles using Spherical Linear Interpolation (SLERP) w/ Quaternionernions
	Quaternion q1(prevangs[0], prevangs[1], prevangs[2]);               // convert 1st rotation to a Quaternionernion
	Quaternion q2(nextangs[0], nextangs[1], nextangs[2]);                // convert 2nd rotation to a Quaternionernion
	Quaternion q3;
	q1.normalize();
	q2.normalize();
	bool rslerp = Quaternion::slerp(q1, q2, fract, q3);                         // get an interpolated Quaternionernion
	if (!rslerp) return(false);
	
	Quaternion::QuaternionToMat(q3, pose);                                                // convert interpolated Quaternionernion in a matrix

	pose = translation3D(position)*pose;									// apply translation
	return(true);																			// success
}
