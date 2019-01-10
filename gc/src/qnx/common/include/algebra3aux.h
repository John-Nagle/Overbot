//
//	Functions of general utility for use with "algebra3.h".
//
//	John Nagle
//	nagle@animats.com
//	This code may be freely used by others.
//
#ifndef ALGEBRA3AUXH
#define ALGEBRA3AUXH

#include "algebra3.h"
//
//	Vector utilities
//
//
//	MultiplyElementwise  --  element by element multiplication
//
inline vec2 MultiplyElementwise(const vec2& v1, const vec2& v2)
{	return(vec2(v1[0]*v2[0],v1[1]*v2[1]));	}
inline vec3 MultiplyElementwise(const vec3& v1, const vec3& v2)
{	return(vec3(v1[0]*v2[0],v1[1]*v2[1],v1[2]*v2[2]));	}
inline vec4 MultiplyElementwise(const vec4& v1, const vec4& v2)
{	return(vec4(v1[0]*v2[0],v1[1]*v2[1],v1[2]*v2[2],v1[3]*v2[3]));	}
//
//	Matrix utilities
//
mat3 ExtractRotation(const mat4& pose);
vec3 ExtractTranslation(const mat4& pose);
vec3 ExtractScaling(const mat4& pose);
mat4 rotation3D(const mat3& m);

//
//	rotation3D -- convert 3x3 rotation matrix to 4x4
//
#ifdef TEMPTURNOFF	// GCC 2.93 on QNX can't compile this.
inline mat4 rotation3D(const mat3& m)
{	return(mat4(vec4(m[0][0],m[0][1],m[0][2],0.0),
				vec4(m[1][0],m[1][1],m[1][2],0.0),
				vec4(m[2][0],m[2][1],m[2][2],0.0),
				vec4(0.0,0.0,0.0,1.0)));
}
#endif // TEMPTURNOFF
inline mat4 rotation3D(const mat3& m)
{	vec4 v0(m[0][0],m[0][1],m[0][2],0.0);
	vec4 v1(m[1][0],m[1][1],m[1][2],0.0);
	vec4 v2(m[2][0],m[2][1],m[2][2],0.0);
	vec4 v3(0.0,0.0,0.0,1.0);
	return(mat4(v0,v1,v2,v3));		
}
//
//	Matrix utilities for affine matrices.
//
//	The input matrix must be affine, but need not be orthogonal.
//	In other words, it may contain scaling, rotation, and translation,
//	but not perspective projection.
//
//	ExtractTranslation  --  extract translation vector
//
inline vec3 ExtractTranslation(const mat4& pose)
{	//	Ref. Foley and Van Dam, 2nd ed, p. 217.
	return(vec3(pose[0][3],pose[1][3],pose[2][3]));
}
//
//	ExtractScaling
//
inline vec3 ExtractScaling(const mat4& mat)
{
	//	Per Valerie Demers, Softimage.
	//	Transposed for mat4 matrices
	return(vec3(
	sqrt( mat[ 0 ][ 0 ] * mat[ 0 ][ 0 ] + mat[ 1 ][ 0 ] * mat[ 1 ][ 0 ] + mat[ 2 ][ 0 ] * mat[ 2 ][ 0 ] ),
	sqrt( mat[ 0 ][ 1 ] * mat[ 0 ][ 1 ] + mat[ 1 ][ 1 ] * mat[ 1 ][ 1 ] + mat[ 2 ][ 1 ] * mat[ 2 ][ 1 ] ),
	sqrt( mat[ 0 ][ 2 ] * mat[ 0 ][ 2 ] + mat[ 1 ][ 2 ] * mat[ 1 ][ 2 ] + mat[ 2 ][ 2 ] * mat[ 2 ][ 2 ] )));
}
//
//
//	ExtractRotation  --  extract rotation matrix from transformation matrix
//
inline mat3 ExtractRotation(const mat4& pose)
{
	vec3 scale(ExtractScaling(pose));	// get scaling
	//	Compute inverse of scaling
	vec3 invscale(1.0/scale[0],1.0/scale[1],1.0/scale[2]);
	//	Apply inverse of scaling as a transformation, to get unit scaling.
	mat4 unscaled(pose*scaling3D(invscale));// unscale pose
	//	Return pure rotation matrix
#ifdef TEMPTURNOFF			// GCC 2.93 on QNX can't compile this
	return(mat3(						// drop last column and row
		vec3(unscaled[0],VW),
		vec3(unscaled[1],VW),
		vec3(unscaled[2],VW)));
#endif // TEMPTURNOFF
	vec3 v0(unscaled[0],VW);
	vec3 v1(unscaled[1],VW);
	vec3 v2(unscaled[2],VW);
	return(mat3(	v0,v1,v2));					// drop last column and row
}
//
//	PointToPlane  --  signed distance from a point to a plane
//
//	Outside the plane yields positive values
//
inline double PointToPlane(const vec3& point, const vec4& plane)
{
	vec3 planeNormal(plane[0],plane[1],plane[2]);
	assert(fabs(planeNormal.length2() - 1.0) < 0.00001);	// check normalized
	double dist = point * planeNormal + plane[3];
	return(dist);
}
//
//	fmin, fmax  -- min, max for various types 
//
//	Avoids conflicts with Windows macros.
//
inline double fmin(double x, double y) { return(x < y ? x : y); }
inline double fmax(double x, double y) { return(x > y ? x : y); }

inline vec2 fmin(const vec2& v1, const vec2& v2)
{	return(vec3(fmin(v1[0],v2[0]),fmin(v1[1],v2[1]))); }
inline vec2 fmax(const vec2& v1, const vec2& v2)
{	return(vec3(fmax(v1[0],v2[0]),fmax(v1[1],v2[1]))); }

inline vec3 fmin(const vec3& v1, const vec3& v2)
{	return(vec3(fmin(v1[0],v2[0]),fmin(v1[1],v2[1]),fmin(v1[2],v2[2]))); }
inline vec3 fmax(const vec3& v1, const vec3& v2)
{	return(vec3(fmax(v1[0],v2[0]),fmax(v1[1],v2[1]),fmax(v1[2],v2[2]))); }

inline vec4 fmin(const vec4& v1, const vec4& v2)
{	return(vec4(fmin(v1[0],v2[0]),fmin(v1[1],v2[1]),fmin(v1[2],v2[2]),fmin(v1[3],v2[3]))); }
inline vec4 fmax(const vec4& v1, const vec4& v2)
{	return(vec4(fmax(v1[0],v2[0]),fmax(v1[1],v2[1]),fmax(v1[2],v2[2]),fmax(v1[3],v2[3]))); }

#endif // ALGEBRA3AUXH
