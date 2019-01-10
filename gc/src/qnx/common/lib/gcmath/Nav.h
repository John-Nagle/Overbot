/* -*- indent-tabs-mode:T; c-basic-offset:8; tab-width:8; -*- vi: set ts=8:
 * $Id: Nav.h,v 1.1 2003/10/26 17:17:27 khianhao Exp $
 *
 * This is the navigation related functions for the math library.
 *
 *	Author: Aaron Kahn, Suresh Kannan, Eric Johnson
 *	copyright 2001
 *	Portions (c) Trammell Hudson
 */
#ifndef _Nav_h_
#define _Nav_h_

#include <Conversions.h>
#include <Vector.h>
#include <Matrix.h>
#include <cmath>
#include <cstdlib>

#include <stdio.h>


/**
 *  Convert accelerations to euler angles
 */
static inline const Vector<3>
accel2euler(
	const Vector<3> &	a,
	double			heading
)
{
	const double		g = a.mag();

	return Vector<3>(
			 -std::atan2( a[1], -a[2]), //Roll

			 -std::asin( a[0] / -g ),	// Pitch //original
			 //-std::atan2( a[0], -a[2]),     //new

			 heading				// Yaw
	);

	//khianhao
	//trying to make tilt sensing tolerant to changes in z

//	return Vector<3>(
//			 -std::atan2( a[1], 9.81),     //Roll
//			 -std::atan2( a[0], 9.81 ),   //Pitch
//			 heading		       // Yaw
//	);


}


/*
 * Find the shortest distance around the circle from psi to desired.
 * Otherwise our controllers will have problems facing south.
 *
 * THIS IS DEPRECATED.  Use turn_direction instead.
 */
static inline double
smallest_angle(
	const double		psi,
	const double		desired
)
{
	if( desired - psi > C_PI )
		return psi + 2 * C_PI;

	if( psi - desired > C_PI )
		return psi - 2 * C_PI;

	return psi;
}


/*
 * Compute the shortest way from the current position to the
 * commanded position.  Both are in radians.
 */
static inline double
turn_direction(
	double			command,
	double			current
)
{
	if( current > C_PI/2 && command < -C_PI/2 )
		return 2 * C_PI + command - current;
	if( current < -C_PI/2 && command > C_PI/2 )
		return -2 * C_PI + command - current;
	return command - current;
}

extern const Vector<3>
euler_diff(
	const Vector<3> &	command,
	const Vector<3> &	current
);


/*
 * Computes the NED distance (m) between two LLH positions (rad)
 * in the tangent plane defined by lat/lon (rad).
 */
extern const Vector<3>
LLH_dist(
	const Vector<3> &	llh1,
	const Vector<3> &	llh2,
	const double		lat,
	const double		lon
);


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
);


/*
 * This will convert from ECEF to local tangent plane coordinates
 * local(x,y,z) = Re2t*[X;Y;Z]
 * latitude and longitude are in radians
 */
extern const Vector<3>
ECEF2Tangent(
	const Vector<3> &	ECEF,
	const double		latitude,
	const double		longitude
);


/*
 * This will convert from local tangent plane coordinates to ECEF
 * ECEF[X,Y,Z] = Rt2e*local(x,y,z)
 *
 * latitude and longitude are in radians
 */
extern const Vector<3>
Tangent2ECEF(
	const Vector<3> &	Local,
	const double		latitude,
	const double		longitude
);


/*
 * This will convert from ECEF to geodedic latitude, longitude, altitude
 * based on the WGS-84 elipsoid.
 *
 *	latitude and longitude (rad), altitude (m)
 *	ECEF (m)
 *	ECEF = [X Y Z] (m)
 *
 * Returns
 *	llh = [latitude longitude altitude] (rad)(m)
 */
extern const Vector<3>
ECEF2llh(
	const Vector<3> &	ECEF
);


/*
 * This will convert from latitude, longitude, and altitude into
 * ECEF coordinates.  Note: altitude in + up
 *
 *	ECEF = [X Y Z] (m)
 *	llh = [latitude longitude altitude] (rad)(m)
 */
extern const Vector<3>
llh2ECEF(
	const Vector<3> &	llh
);


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
extern void
atmosphere(
	double			altitude,
	double *		density,
	double *		pressure,
	double *		temperature,
	double *		sp_sound
);


static inline
Matrix<3,3>
euler_strapdown(
	const Vector<3> &	euler
)
{
	const double		sphi( std::sin(euler[0]) );
	const double		cphi( std::cos(euler[0]) );

	const double		ctheta( std::cos(euler[1]) );
	const double		ttheta( std::tan(euler[1]) );

	return Matrix<3,3>(
		Vector<3>( 1, sphi*ttheta, cphi*ttheta ),
		Vector<3>( 0, cphi,        -sphi ),
		Vector<3>( 0, sphi/ctheta, cphi/ctheta )
	);
}


/*
 *  Add random noise to a vector
 */
template<
	const int		n,
	class			T
>
const Vector<n>
noise(
	const T &		high,
	const T &		low = 0
)
{
	Vector<n,T>		v;

	for( typename Vector<n,T>::index_t i=0 ; i<n ; i++ )
		v[i] = drand48() * (high - low) + low;

	return v;
}

#endif
