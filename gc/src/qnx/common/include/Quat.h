/* -*- indent-tabs-mode:T; c-basic-offset:8; tab-width:8; -*- vi: set ts=8:
 * $Id: Quat.h,v 1.1 2003/12/15 00:31:06 khianhao Exp $
 *
 * (c) Aaron Kahn
 * (c) Trammell Hudson
 *
 * Vector rotation code
 *
 *************
 *
 *  This file is part of the autopilot simulation package.
 *
 *  For more details:
 *
 *	http://autopilot.sourceforge.net/
 *
 *  Autopilot is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  Autopilot is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Autopilot; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#ifndef _Quat_h_
#define _Quat_h_

#include <Matrix.h>
#include <Vector.h>

/*
 * This will construct a direction cosine matrix from
 * euler angles in the standard rotation sequence
 * [phi][theta][psi] from NED to body frame
 *
 * body = tBL(3,3)*NED
 */
extern const Matrix<3,3>
eulerDC(
	const Vector<3> &	euler
);


/*
 * This will construct a direction cosine matrix from
 * quaternions in the standard rotation  sequence
 * [phi][theta][psi] from NED to body frame
 *
 * body = tBL(3,3)*NED
 * q(4,1)
 */
extern const Matrix<3,3>
quatDC(
        const Quat &		q
);


/*
 * This will construct the euler omega-cross matrix
 * wx(3,3)
 * p, q, r (rad/sec)
 */
extern const Matrix<3,3>
eulerWx(
        const Vector<3> &	euler
);



/*
 * This will construct the quaternion omega matrix
 * W(4,4)
 * p, q, r (rad/sec)
 */
extern const Matrix<4,4>
quatW(
	const Vector<3>		euler
);


/*
 * This will convert from quaternions to euler angles
 * q(4,1) -> euler[phi;theta;psi] (rad)
 */
extern const Vector<3>
quat2euler(
	const Quat &		q
);


/*
 * This will convert from euler angles to quaternion vector
 * phi, theta, psi -> q(4,1)
 * euler angles in radians
 */
extern const Quat
euler2quat(
	const Vector<3> &	euler
);


/*
 * Compute the derivative of the Euler angle Phi with respect
 * to the quaternion Q.  The result is a row vector
 *
 * d(phi)/d(q0)
 * d(phi)/d(q1)
 * d(phi)/d(q2)
 * d(phi)/d(q3)
 *
 * The DCM is an optimization for repeated calls to the d*_dq
 * family of functions.  For a single call, you can use quatDC( q ).
 */
extern const Quat
dphi_dq(
	const Quat &		q,
	const Matrix<3,3> &	DCM
);


/*
 * Compute the derivative of the Euler angle Theta with respect
 * to the quaternion Q.  The result is a row vector
 *
 * d(theta)/d(q0)
 * d(theta)/d(q1)
 * d(theta)/d(q2)
 * d(theta)/d(q3)
 */
extern const Quat
dtheta_dq(
	const Quat &		q,
	const Matrix<3,3> &	DCM
);


/*
 * Compute the derivative of the Euler angle Psi with respect
 * to the quaternion Q.  The result is a row vector
 *
 * d(psi)/d(q0)
 * d(psi)/d(q1)
 * d(psi)/d(q2)
 * d(psi)/d(q3)
 */
extern const Quat
dpsi_dq(
	const Quat &		q,
	const Matrix<3,3> &	DCM
);


#endif
