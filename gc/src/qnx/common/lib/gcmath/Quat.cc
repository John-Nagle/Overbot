/* -*- indent-tabs-mode:T; c-basic-offset:8; tab-width:8; -*- vi: set ts=8:
 * $Id: Quat.cc,v 1.1 2003/10/26 17:17:27 khianhao Exp $
 *
 *	This is the source file for the matrix and math utiltiy
 *	library.
 *
 *	Author: Aaron Kahn, Suresh Kannan, Eric Johnson
 *	copyright 2001
 *	Portions (c) Trammell Hudson
 */

#include <cmath>

#include <Quat.h>


using std::sin;
using std::cos;


/*
 * This will construct a direction cosine matrix from 
 * euler angles in the standard rotation sequence 
 * [phi][theta][psi] from NED to body frame
 *
 * body = tBL(3,3)*NED
 */
const Matrix<3,3>
eulerDC(
        const Vector<3> &       euler
)
{
	const double &		phi	= euler[0];
	const double &		theta	= euler[1];
	const double &		psi	= euler[2];

	const double		cpsi	= cos(psi);
	const double		cphi	= cos(phi);
	const double		ctheta	= cos(theta);

	const double		spsi	= sin(psi);
	const double		sphi	= sin(phi);
	const double		stheta	= sin(theta);

	return Matrix<3,3>(
		Vector<3>(
			 cpsi*ctheta,
			 spsi*ctheta,
			 -stheta
		),
		Vector<3>(
			 -spsi*cphi + cpsi*stheta*sphi,
			  cpsi*cphi + spsi*stheta*sphi,
			                   ctheta*sphi
		),
		Vector<3>(
			  spsi*sphi + cpsi*stheta*cphi,
			 -cpsi*sphi + spsi*stheta*cphi,
			                   ctheta*cphi
		)
	);
}


/*
 * This will construct a direction cosine matrix from 
 * quaternions in the standard rotation  sequence
 * [phi][theta][psi] from NED to body frame
 *
 * body = tBL(3,3)*NED
 * q(4,1)
 */
const Matrix<3,3>
quatDC(
	const Quat &		q
)
{
	const double &		q0 = q[0];
	const double &		q1 = q[1];
	const double &		q2 = q[2];
	const double &		q3 = q[3];

	return Matrix<3,3>(
		Vector<3>(
			1.0-2*(q2*q2 + q3*q3),
			    2*(q1*q2 + q0*q3),
			    2*(q1*q3 - q0*q2)
		),
		Vector<3>(
			    2*(q1*q2 - q0*q3),
			1.0-2*(q1*q1 + q3*q3),
			    2*(q2*q3 + q0*q1)
		),
		Vector<3>(
			    2*(q1*q3 + q0*q2),
			    2*(q2*q3 - q0*q1),
			1.0-2*(q1*q1 + q2*q2)
		)
	);
}


/*
 * This will construct the euler omega-cross matrix
 * wx(3,3)
 * p, q, r (rad/sec)
 */
const Matrix<3,3>
eulerWx(
        const Vector<3> &	euler
)
{
	const double &		p = euler[0];
	const double &		q = euler[1];
	const double &		r = euler[2];

	return Matrix<3,3>(
		Vector<3>(  0, -r,  q ),
		Vector<3>(  r,  0, -p ),
		Vector<3>( -q,  p,  0 )
	);
}


/*
 * This will construct the quaternion omega matrix
 * W(4,4)
 * p, q, r (rad/sec)
 */
const Matrix<4,4>
quatW(
	const Vector<3>		euler
)
{
	const double		p = euler[0] / 2.0;
	const double		q = euler[1] / 2.0;
	const double		r = euler[2] / 2.0;

	return Matrix<4,4>(
		Vector<4>(  0, -p, -q, -r ),
		Vector<4>(  p,  0,  r, -q ),
		Vector<4>(  q, -r,  0,  p ),
		Vector<4>(  r,  q, -p,  0 )
	);
}



/*
 * This will convert from quaternions to euler angles
 * q(4,1) -> euler[phi;theta;psi] (rad)
 */
const Vector<3>
quat2euler(
	const Quat &		q
)
{
	const double &		q0 = q[0];
	const double &		q1 = q[1];
	const double &		q2 = q[2];
	const double &		q3 = q[3];

	double			theta	= -asin(
		  2*(q1*q3 - q0*q2)
	);

	double			phi	= atan2(
	 	  2*(q2*q3 + q0*q1),
		1-2*(q1*q1 + q2*q2)
	);

	double			psi	= atan2(
		  2*(q1*q2 + q0*q3),
		1-2*(q2*q2 + q3*q3)
	);
 
	return Vector<3>( phi, theta, psi );
}


/*
 * This will convert from euler angles to quaternion vector
 * phi, theta, psi -> q(4,1)
 * euler angles in radians
 */
const Quat
euler2quat(
	const Vector<3> &       euler
)
{
	const double		phi	= euler[0] / 2.0;
	const double		theta	= euler[1] / 2.0;
	const double		psi	= euler[2] / 2.0;

	const double		shphi0   = sin( phi );
	const double		chphi0   = cos( phi );

	const double		shtheta0 = sin( theta );
	const double		chtheta0 = cos( theta );

	const double		shpsi0   = sin( psi );
	const double		chpsi0   = cos( psi );

	return Quat(
		  chphi0 * chtheta0 * chpsi0 + shphi0 * shtheta0 * shpsi0,
		 -chphi0 * shtheta0 * shpsi0 + shphi0 * chtheta0 * chpsi0,
		  chphi0 * shtheta0 * chpsi0 + shphi0 * chtheta0 * shpsi0,
		  chphi0 * chtheta0 * shpsi0 - shphi0 * shtheta0 * chpsi0
	);
}



/*
 * Functions to compute the partial derivative of the quaterion with
 * respect to the Euler angles.  These are used for computation of the
 * matrix C in the Kalman filter that represents the relationship of
 * the measurements to the states.
 */
const Vector<4>
dphi_dq(
	const Vector<4> &	quat,
	const Matrix<3,3> &	DCM
)
{
	const double		q0 = quat[0];
	const double		q1 = quat[1];
	const double		q2 = quat[2];
	const double		q3 = quat[3];
	const double		dcm22 = DCM[2][2];
	const double		dcm12 = DCM[1][2];
	const double		err = 2 / (dcm22*dcm22 + dcm12*dcm12);

	return Vector<4>(
		(q1 * dcm22) * err,
		(q0 * dcm22 + 2 * q1 * dcm12) * err,
		(q3 * dcm22 + 2 * q2 * dcm12) * err,
		(q2 * dcm22) * err
	);
}


const Vector<4>
dtheta_dq(
	const Vector<4> &	quat,
	const Matrix<3,3> &	DCM
)
{
	const double		q0 = quat[0];
	const double		q1 = quat[1];
	const double		q2 = quat[2];
	const double		q3 = quat[3];
	const double		dcm02 = DCM[0][2];
	const double		err = -2 / sqrt( 1 - dcm02*dcm02 );

	return Vector<4>(
		-q2 * err,
		 q3 * err,
		-q0 * err,
		 q1 * err
	);
}


const Vector<4>
dpsi_dq(
	const Vector<4> &	quat,
	const Matrix<3,3> &	DCM
)
{
	const double		q0 = quat[0];
	const double		q1 = quat[1];
	const double		q2 = quat[2];
	const double		q3 = quat[3];
	const double		dcm00 = DCM[0][0];
	const double		dcm01 = DCM[0][1];
	const double		err = 2 / ( dcm00*dcm00 + dcm01*dcm01 );

	return Vector<4>(
		err * (q3 * dcm00),
		err * (q2 * dcm00),
		err * (q1 * dcm00 + 2 * q2 * dcm01),
		err * (q0 * dcm00 + 2 * q3 * dcm01)
	);
}

