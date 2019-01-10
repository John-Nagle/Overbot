/**** EulerAngles.c - Convert Euler angles to/from matrix or quat ****/
/* Ken Shoemake, 1993 */
#include <math.h>
#include <float.h>
#include "eulerangle.h"
//
//	Single-precision trig functions, required by VC++.
//
EulerAngles Eul_(double ai, double aj, double ah)
{
    EulerAngles ea(ai,aj,ah);
    return (ea);
}

/* Construct matrix from Euler angles (in radians). */
void Eul_ToHMatrix(EulerAngles ea, HMatrix& M, int order)
{
    double ti, tj, th, ci, cj, ch, si, sj, sh, cc, cs, sc, ss;
    int i,j,k,h,n,s,f;
    EulGetOrd(order,i,j,k,h,n,s,f);
    if (f==EulFrmR) {double t = ea[X]; ea[X] = ea[Z]; ea[Z] = t;}
    if (n==EulParOdd) {ea[X] = -ea[X]; ea[Y] = -ea[Y]; ea[Z] = -ea[Z];}
    ti = ea[X];	  tj = ea[Y];	th = ea[Z];
    ci = cos(ti); cj = cos(tj); ch = cos(th);
    si = sin(ti); sj = sin(tj); sh = sin(th);
    cc = ci*ch; cs = ci*sh; sc = si*ch; ss = si*sh;
    if (s==EulRepYes) {
	M[i][i] = cj;	  M[i][j] =  sj*si;    M[i][k] =  sj*ci;
	M[j][i] = sj*sh;  M[j][j] = -cj*ss+cc; M[j][k] = -cj*cs-sc;
	M[k][i] = -sj*ch; M[k][j] =  cj*sc+cs; M[k][k] =  cj*cc-ss;
    } else {
	M[i][i] = cj*ch; M[i][j] = sj*sc-cs; M[i][k] = sj*cc+ss;
	M[j][i] = cj*sh; M[j][j] = sj*ss+cc; M[j][k] = sj*cs-sc;
	M[k][i] = -sj;	 M[k][j] = cj*si;    M[k][k] = cj*ci;
    }
    M[W][X]=M[W][Y]=M[W][Z]=M[X][W]=M[Y][W]=M[Z][W]=0.0f; M[W][W]=1.0f;
}

/* Convert matrix to Euler angles (in radians). */
EulerAngles Eul_FromHMatrix(const HMatrix& M, int order)
{
    EulerAngles ea;
    int i,j,k,h,n,s,f;
    EulGetOrd(order,i,j,k,h,n,s,f);
    if (s==EulRepYes) {
	double sy = sqrt(M[i][j]*M[i][j] + M[i][k]*M[i][k]);
	if (sy > 16*FLT_EPSILON) {
	    ea[X] = atan2(M[i][j], M[i][k]);
	    ea[Y] = atan2(sy, M[i][i]);
	    ea[Z] = atan2(M[j][i], -M[k][i]);
	} else {
	    ea[X] = atan2(-M[j][k], M[j][j]);
	    ea[Y] = atan2(sy, M[i][i]);
	    ea[Z] = 0.0f;
	}
    } else {
	double cy = sqrt(M[i][i]*M[i][i] + M[j][i]*M[j][i]);
	if (cy > 16*FLT_EPSILON) {
	    ea[X] = atan2(M[k][j], M[k][k]);
	    ea[Y] = atan2(-M[k][i], cy);
	    ea[Z] = atan2(M[j][i], M[i][i]);
	} else {
	    ea[X] = atan2(-M[j][k], M[j][j]);
	    ea[Y] = atan2(-M[k][i], cy);
	    ea[Z] = 0.0f;
	}
    }
    if (n==EulParOdd) {ea[X] = -ea[X]; ea[Y] = - ea[Y]; ea[Z] = -ea[Z];}
    if (f==EulFrmR) {double t = ea[X]; ea[X] = ea[Z]; ea[Z] = t;}
    ////ea[W] = order;
    return (ea);
}
#ifdef QUATERNIONS			// if quaternion support
/* Construct quaternion from Euler angles (in radians). */
Quat Eul_ToQuat(EulerAngles ea, int order)
{
    Quat qu;
    double a[3], ti, tj, th, ci, cj, ch, si, sj, sh, cc, cs, sc, ss;
    int i,j,k,h,n,s,f;
    EulGetOrd(order,i,j,k,h,n,s,f);
    if (f==EulFrmR) {double t = ea[X]; ea[X] = ea[Z]; ea[Z] = t;}
    if (n==EulParOdd) ea[Y] = -ea[Y];
    ti = ea[X]*0.5f; tj = ea[Y]*0.5f; th = ea[Z]*0.5f;
    ci = cos(ti);  cj = cos(tj);  ch = cos(th);
    si = sin(ti);  sj = sin(tj);  sh = sin(th);
    cc = ci*ch; cs = ci*sh; sc = si*ch; ss = si*sh;
    if (s==EulRepYes) {
	a[i] = cj*(cs + sc);	/* Could speed up with */
	a[j] = sj*(cc + ss);	/* trig identities. */
	a[k] = sj*(cs - sc);
	qu[W] = cj*(cc - ss);
    } else {
	a[i] = cj*sc - sj*cs;
	a[j] = cj*ss + sj*cc;
	a[k] = cj*cs - sj*sc;
	qu[W] = cj*cc + sj*ss;
    }
    if (n==EulParOdd) a[j] = -a[j];
    qu[X] = a[X]; qu[Y] = a[Y]; qu[Z] = a[Z];
    return (qu);
}
/* Convert quaternion to Euler angles (in radians). */
EulerAngles Eul_FromQuat(const Quat& q, int order)
{
    HMatrix M;
    double Nq = q[X]*q[X]+q[Y]*q[Y]+q[Z]*q[Z]+q[W]*q[W];
    double s = (Nq > 0.0f) ? (2.0f / Nq) : 0.0f;
    double xs = q[X]*s,	  ys = q[Y]*s,	 zs = q[Z]*s;
    double wx = q[W]*xs,	  wy = q[W]*ys,	 wz = q[W]*zs;
    double xx = q[X]*xs,	  xy = q[X]*ys,	 xz = q[X]*zs;
    double yy = q[Y]*ys,	  yz = q[Y]*zs,	 zz = q[Z]*zs;
    M[X][X] = 1.0f - (yy + zz); M[X][Y] = xy - wz; M[X][Z] = xz + wy;
    M[Y][X] = xy + wz; M[Y][Y] = 1.0f - (xx + zz); M[Y][Z] = yz - wx;
    M[Z][X] = xz - wy; M[Z][Y] = yz + wx; M[Z][Z] = 1.0f - (xx + yy);
    M[W][X]=M[W][Y]=M[W][Z]=M[X][W]=M[Y][W]=M[Z][W]=0.0f; M[W][W]=1.0f;
    return (Eul_FromHMatrix(M, order));
}
#endif // QUATERNIONS
