//
//	Quaternionernion.cc -- Quaternionernion functionality, plus SLERP algorithm
//
//	John Pierre
//	Team Overbot
//	August, 2005
//
#include "quaternion.h"
#include "eulerangle.h"
#include <math.h>

Quaternion::Quaternion(float fRoll, float fPitch, float fYaw) : m_x(0.0), m_y(0.0), m_z(0.0), m_w(0.0)
{
       const float fSinPitch(sin(fPitch*0.5));
       const float fCosPitch(cos(fPitch*0.5));
       const float fSinYaw(sin(fYaw*0.5));
       const float fCosYaw(cos(fYaw*0.5));
       const float fSinRoll(sin(fRoll*0.5));
       const float fCosRoll(cos(fRoll*0.5));
       const float fCosPitchCosYaw(fCosPitch*fCosYaw);
       const float fSinPitchSinYaw(fSinPitch*fSinYaw);
       m_x = fSinRoll * fCosPitchCosYaw     - fCosRoll * fSinPitchSinYaw;
       m_y = fCosRoll * fSinPitch * fCosYaw + fSinRoll * fCosPitch * fSinYaw;
       m_z = fCosRoll * fCosPitch * fSinYaw - fSinRoll * fSinPitch * fCosYaw;
       m_w = fCosRoll * fCosPitchCosYaw     + fSinRoll * fSinPitchSinYaw;
}

void Quaternion::QuaternionToMat(Quaternion & q, mat4 &mat)
{
	float X = q.m_x;
	float Y = q.m_y;
	float Z = q.m_z;
	float W = q.m_w;
	
   float xx      = X * X;
    float xy      = X * Y;
    float xz      = X * Z;
    float xw      = X * W;
    float yy      = Y * Y;
    float yz      = Y * Z;
    float yw      = Y * W;
    float zz      = Z * Z;
    float zw      = Z * W;
    mat[0][0]  = 1 - 2 * ( yy + zz );
    mat[0][1]  =     2 * ( xy - zw );
    mat[0][2]  =     2 * ( xz + yw );
    mat[1][0]  =     2 * ( xy + zw );
    mat[1][1]  = 1 - 2 * ( xx + zz );
    mat[1][2]  =     2 * ( yz - xw );
    mat[2][0]  =     2 * ( xz - yw );
    mat[2][1]  =     2 * ( yz + xw );
    mat[2][2] = 1 - 2 * ( xx + yy );
    mat[0][3]  = mat[1][3] = mat[2][3] = mat[3][0] = mat[3][1] = mat[3][2] = 0;
    mat[3][3] = 1;
}

bool Quaternion::slerp(Quaternion & q1, Quaternion & q2, float t, Quaternion & q3) 
{
	float one_minus_t = 1.0 - t;
	
	// interpolation parameter must be between one and zero
	if (t < 0.0 || t > 1.0) return false;
	
	// need unit vectors, but could save comp time by elimintating this check
	if (q1.magnitude() < 0.99999 || q2.magnitude() < 0.99999) return false;
	
	float cos_omega = q1.m_x*q2.m_x + q1.m_y*q2.m_y + q1.m_z*q2.m_z + q1.m_w*q2.m_w;
	
	// if q2 is on opposite hemisphere flip sign
	bool flip = cos_omega < 0.0;
	if (flip) cos_omega = - cos_omega;
	
	float omega = acos(cos_omega);
	float sin_omega = sin(omega);
	
	float sin_one_minus_t_omega = sin(one_minus_t*omega);
	float sin_t_omega = sin(t*omega);
	
	if (flip) sin_t_omega = -sin_t_omega;
	
	// perform interpolation according to the SLERP formula
	q3.m_x = (sin_one_minus_t_omega * q1.m_x + sin_t_omega*q2.m_x)/sin_omega; 
	q3.m_y = (sin_one_minus_t_omega * q1.m_y + sin_t_omega*q2.m_y)/sin_omega; 
	q3.m_z = (sin_one_minus_t_omega * q1.m_z + sin_t_omega*q2.m_z)/sin_omega; 
	q3.m_w = (sin_one_minus_t_omega * q1.m_w + sin_t_omega*q2.m_w)/sin_omega; 
	
	return true;
}	

float Quaternion::magnitude()
{
	return sqrt(m_x*m_x + m_y*m_y + m_z*m_z + m_w*m_w);	
}

void Quaternion::normalize()
{
	float m = magnitude();
	if (m > 0.0)
	{
		float norm = 1.0 / m;
		*this *= norm; 
	}
}

Quaternion & Quaternion::operator *= (const float C)
{
	m_x *= C;
	m_y *= C;
	m_z *= C;
	m_w *= C;
	
	return *this;
}

inline Quaternion operator + (const Quaternion & q1, const Quaternion & q2)
{
	Quaternion r(
		q1.m_x+q2.m_x, 
		q1.m_y+q2.m_y, 
		q1.m_z+q2.m_z, 
		q1.m_w+q2.m_w
		);
	return r;
}

inline Quaternion operator * (const Quaternion & q1, const Quaternion & q2)
{
	Quaternion r(
				q1.m_w*q2.m_x + q1.m_x*q2.m_w + q1.m_y*q2.m_z - q1.m_z*q2.m_y, 
				q1.m_w*q2.m_y + q1.m_y*q2.m_w + q1.m_z*q2.m_x - q1.m_x*q2.m_z, 
				q1.m_w*q2.m_z + q1.m_z*q2.m_w + q1.m_x*q2.m_y - q1.m_y*q2.m_x, 
				q1.m_w*q2.m_w - q1.m_x*q2.m_x - q1.m_y*q2.m_y - q1.m_z*q2.m_z
				);
	return r;
}  
#/** PhEDIT attribute block
#-11:16777215
#0:3786:default:-3:-3:0
#3786:3830:TextFont9:-3:-3:0
#3830:3852:default:-3:-3:0
#**  PhEDIT attribute block ends (-0000169)**/
