#ifndef QUATERNIONS_H
#define QUATERNIONS_H

class mat4;

class Quaternion {
    friend Quaternion operator * (const Quaternion & q1, const Quaternion & q2);
    friend Quaternion operator + (const Quaternion & q1, const Quaternion & q2);
    
private:
	float m_x, m_y, m_z, m_w;
	
public:
	Quaternion() : m_x(0.0), m_y(0.0), m_z(0.0), m_w(0.0) {}
	Quaternion(float roll, float pitch, float yaw);
	Quaternion(float x, float y, float z, float w) : m_x(x), m_y(y), m_z(z), m_w(w) {}
	static bool slerp(Quaternion & q1, Quaternion & q2, float t, Quaternion & q3);
	static void QuaternionToMat(Quaternion & q, mat4 & m);	
	float magnitude();
	void normalize();
	
	Quaternion & operator *= (const float C);
};

#endif // QUATERNIONS_H