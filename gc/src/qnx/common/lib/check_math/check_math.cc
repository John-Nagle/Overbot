#include <iostream>

#include <Quat.h>
#include <Vector.h>
#include <Matrix.h>
#include <Matrix_Invert.h>
#include <Kalman.h>
#include <Nav.h>

using namespace std;


int main()
{

	Vector<3> v0 (1.0f , 2.0f, 3.0f);
	
	Matrix<3,3> m0;
	m0.fill();
	m0[0][0] = 2.0f;
	m0[1][1] = 2.0f;
	m0[2][2] = 2.0f;
	
	cout << "m0 * v0" << endl;
	cout << m0*v0 << endl;
	
	Matrix <3,3> m1;
	m1.fill();
	m1[0][0] = 1.0;
	m1[0][1] = 2.0;
	m1[0][2] = 3.0;
	m1[1][1] = 4.0;
	m1[1][2] = 5.0;
	m1[2][2] = 6.0;
	cout << "invert(m1)" << endl;
	cout << invert(m1) << endl;
	
	Vector<3> v1(1.0f, 1.0f, 1.0f);
	Vector<3> v2 = ECEF2llh(v1);
	cout << "llh " << v2 << endl;
	
	
return 0;
}
