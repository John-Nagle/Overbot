/* -*- indent-tabs-mode:T; c-basic-offset:8; tab-width:8; -*- vi: set ts=8:
 * $Id: Nav.cc,v 1.1 2003/10/26 17:17:27 khianhao Exp $
 *
 * This is the navigation related functions for the math library.
 *
 *	Author: Aaron Kahn, Suresh Kannan, Eric Johnson
 *	copyright 2001
 *	Portions (c) Trammell Hudson
 */
#include <Nav.h>
#include <Matrix.h>
#include <Conversions.h>
#include <cmath>

using std::sin;
using std::cos;
using std::atan;
using std::atan2;

static inline double
sqr(
	double			x
)
{
	return x * x;
}


const Vector<3>
euler_diff(
	const Vector<3> &	command,
	const Vector<3> &	current
)
{
	Vector<3>		diff( command - current );

	if( diff[0] < -C_PI )
		diff[0] += 2 * C_PI;
	if( diff[0] > C_PI )
		diff[0] -= 2 * C_PI;

	if( diff[1] > C_PI / 2 );
		diff[1] -= C_PI;
	if( diff[1] < -C_PI / 2 );
		diff[1] += C_PI;

	if( diff[2] > C_PI )
		diff[2] -= 2 * C_PI;
	if( diff[2] < -C_PI )
		diff[2] += 2 * C_PI;

	return diff;
}


/*
 * Computes the NED distance (m) between two LLH positions (rad)
 * in the tangent plane defined by lat/lon (rad).
 */
const Vector<3>
LLH_dist(
	const Vector<3> &	llh1,
	const Vector<3> &	llh2,
	const double		lat,
	const double		lon
)
{
	const Vector<3> &	ecef1( llh2ECEF( llh1 ) );
	const Vector<3> &	ecef2( llh2ECEF( llh2 ) );
	const Vector<3> &	diff( ecef2 - ecef1 );

	return ECEF2Tangent( diff, lat, lon );
}


/*
 * Computes the NED distance (m) between to ECEF positions (m)
 * in the tangent plane definde by lat/lon (rad)
 */
extern const Vector<3>
ECEF_dist(
	const Vector<3> &	ecef1,
	const Vector<3> &	ecef2,
	const double		lat,
	const double		lon
)
{
	return ECEF2Tangent( ecef1 - ecef2, lat, lon );
}


/*
 * This will convert from ECEF to local tangent plane coordinates
 * local(x,y,z) = Re2t*[X;Y;Z]
 * latitude and longitude are in radians
 */
const Vector<3>
ECEF2Tangent(
	const Vector<3> &	ECEF,
	const double		latitude,
	const double		longitude
)
{
	double			clat = cos(latitude);
	double			clon = cos(longitude);
	double			slat = sin(latitude);
	double			slon = sin(longitude);

	Matrix<3,3>		Re2t;

	Re2t[0][0] = -slat*clon;
	Re2t[0][1] = -slat*slon;
	Re2t[0][2] =  clat;

	Re2t[1][0] = -slon;
	Re2t[1][1] =  clon;
	Re2t[1][2] = 0.0;

	Re2t[2][0] = -clat*clon;
	Re2t[2][1] = -clat*slon;
	Re2t[2][2] = -slat;

	return Re2t * ECEF;
}


/*
 * This will convert from local tangent plane coordinates to ECEF
 * ECEF[X,Y,Z] = Rt2e*local(x,y,z)
 *
 * latitude and longitude are in radians
 */
const Vector<3>
Tangent2ECEF(
	const Vector<3> &	Local,
	const double		latitude,
	const double		longitude
)
{
	double			clat = cos(latitude);
	double			clon = cos(longitude);
	double			slat = sin(latitude);
	double			slon = sin(longitude);

	Matrix<3,3>		Rt2e;

	Rt2e[0][0] = -slat*clon;
	Rt2e[1][0] = -slat*slon;
	Rt2e[2][0] =  clat;

	Rt2e[0][1] = -slon;
	Rt2e[1][1] =  clon;
	Rt2e[2][1] = 0.0;

	Rt2e[0][2] = -clat*clon;
	Rt2e[1][2] = -clat*slon;
	Rt2e[2][2] = -slat;


	return Rt2e * Local;
}


/*
 * This will convert from ECEF to geodedic latitude, longitude, altitude
 * based on the WGS-84 elipsoid.
 *
 *	latitude and longitude (rad), altitude (m)
 *	ECEF (m)
 *	ECEF = [X Y Z] (m)
 *	llh = [latitude longitude altitude] (rad)(m)
 */
const Vector<3>
ECEF2llh(
	const Vector<3> &	ECEF
)
{
	double			X = ECEF[0];
	double			Y = ECEF[1];
	double			Z = ECEF[2];

	double			f = (C_WGS84_a-C_WGS84_b) / C_WGS84_a;
	double			e = sqrt(2*f - f*f);

	double			h = 0;
	double			N = C_WGS84_a;

	Vector<3>		llh;

	llh[1] = atan2(Y, X);	// longitude

	for( int n=0 ; n<50 ; ++n )
	{
		double			sin_lat = Z / (N*(1 - sqr(e)) + h);

		llh[0] = atan(
			(Z + e*e * N * sin_lat) / sqrt(X*X + Y*Y)
		);

		N = C_WGS84_a / sqrt( 1 - sqr(e)*sqr(sin(llh[0])) );
		h = sqrt(X*X + Y*Y) / cos( llh[0] ) - N;
	}

	llh[2] = h;

	return llh;
}


/*
 * This will convert from latitude, longitude, and altitude into
 * ECEF coordinates.  Note: altitude in + up
 *
 *	ECEF = [X Y Z] (m)
 *	llh = [latitude longitude altitude] (rad)(m)
 */
const Vector<3>
llh2ECEF(
	const Vector<3> &	llh
)
{
	double			f = (C_WGS84_a - C_WGS84_b) / C_WGS84_a;
	double			e = sqrt( 2*f - f*f );
	double			N = C_WGS84_a/sqrt( 1 - e*e*sqr(sin(llh[0])));
	Vector<3>		ECEF;

	ECEF[0] = (N + llh[2]) * cos(llh[0]) * cos(llh[1]);
	ECEF[1] = (N + llh[2]) * cos(llh[0]) * sin(llh[1]);
	ECEF[2] = (N*(1-e*e) + llh[2]) * sin(llh[0]);

	return ECEF;
}


/*
 * This will compute the current atmospheric properties given the
 * current altitude in feet above mean sea level (MSL).  The output
 * parameters are output with pointers with all of the useful information.
 *
 *	altitude	MSL altitude (ft) (NOTE: Only valid to 36,152 ft)
 *	density		density (slug/ft^3)
 *	pressure	pressure (lb/ft^2)
 *	temperature	temperature (degrees R)
 *	sp_sound	local speed of sound (ft/s)
 */
void
atmosphere(
	double			altitude,
	double *		density,
	double *		pressure,
	double *		temperature,
	double *		sp_sound
)
{
	double			temp;
	double			temp1;
	double			temp2;

	// compute density first
	temp = (1 - (0.68753 - 0.003264*altitude*1e-5)*altitude*1e-5);
	*density = pow( temp, 4.256)*0.0023769;

	// compute pressure
	temp = (1-(0.68753 - 0.003264*altitude*1e-5)*altitude*1e-5);
	temp1 = pow( temp, 4.256)*0.0023769;
	temp2 = 1716.5*(1 - 0.687532*altitude*1e-5 + 0.003298*sqr(altitude*1e-5))*518.69;
	*pressure = temp1*temp2;

	// compute temp
	*temperature = (1 - 0.687532*altitude*1e-5 + 0.003298*sqr(altitude*1e-5))*518.69;

	// compute speed of sound
	*sp_sound = 1116.45*sqrt( 1 - 0.687532*altitude*1e-5 + 0.003298*sqr(altitude*1e-5));
}			
