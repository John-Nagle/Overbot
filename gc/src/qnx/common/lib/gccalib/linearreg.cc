/////////////////////////////////////////////////////////////////////////////
//
//    File: linearreg.cc
//
//    Usage:
//        see linearreg.h
//
//    Description:
//        see linearreg.h
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        October, 2003
//
/////////////////////////////////////////////////////////////////////////////

#include <math.h>
#include <float.h>
#include "linearreg.h"

//
//    LinearRegression::LinearRegression - default constructor for Linear 
//                                         Regression class
//
template<class Type>
LinearRegression<Type>::LinearRegression()
{
	initialize();
}

//
//    LinearRegression::PointAdd - add a data point to the collection
//
//    Returns nothing
//
template<class Type>
void LinearRegression<Type>::PointAdd(Type x, Type y)
{
	n++;
	
	sumX += x;
	sumY += y;
	sumXsquared += x*x;
	sumYsquared += y*y;
	sumXY += x*y;
	calculate();
}

//
//    LinearRegression::PointNum - use to get the number of data points
//
//    Returns the number of data points
//
template<class Type>
long LinearRegression<Type>::PointNum()
{
	return n;
}

//
//    LinearRegression::PointReset - removes all the points from the collection
//
//    Returns nothing
//
template<class Type>
void LinearRegression<Type>::PointReset()
{
	initialize();
}

//
//    LinearRegression::CalcsReady - use to check if the calculations are ready
//
//    Returns true if at least 3 points have been added to the data set
//
template<class Type>
bool LinearRegression<Type>::CalcsReady()
{
	return n>2 ? true : false;
}

//
//    LinearRegression::Slope - use to get the slope of the best-fit line
//
//    Returns the best-fit slope
//
template<class Type>
Type LinearRegression<Type>::Slope()
{
	return slope;
}

//
//    LinearRegression::Offset - use to get the offset of the best-fit line
//
//    Returns the best-fit offset
//
template<class Type>
Type LinearRegression<Type>::Offset()
{
	return offset;
}

//
//    LinearRegression::CoefCorr - use to get the correleation coefficient
//
//    Returns the correlation coefficient
//
template<class Type>
Type LinearRegression<Type>::CoefCorr()
{
	return coefCorr;
}

//
//    LinearRegression::ErrStd - use to get the standard error
//
//    Returns the standard error
//
template<class Type>
Type LinearRegression<Type>::ErrStd()
{
	return errStd;
}

//
//    LinearRegression::initialize - zeros all raw data members
//
template<class Type>
void LinearRegression<Type>::initialize()
{
	n = 0L;
	
	sumX = 0.0;
	sumY = 0.0;
	sumXsquared = 0.0;
	sumYsquared = 0.0;
	sumXY = 0.0;
}

//
//    LinearRegression::calculate - computes all the linear fit constants
//
template<class Type>
void LinearRegression<Type>::calculate()
{
	Type delta;
	Type sx, sy2, sy;
	
	if ( CalcsReady() ) {
		delta = double(n)*sumXsquared - sumX*sumX;
		
		// make sure we won't get a floating point exception
		if ( fabs(delta) > DBL_EPSILON ) {
			slope = (double(n)*sumXY - sumX*sumY) / delta;
			offset = (sumY -slope*sumX) / double(n);
			
			sx = slope * (sumXY - sumX*sumY/double(n));
			sy2 = sumYsquared - sumY*sumY/double(n);
			sy = sy2 - sx;
			
			coefCorr = sqrt(sx/sy2);
			errStd = sqrt(sy/double(n-2));
		} else {
			slope = 0.0;
			offset = 0.0;
			coefCorr = 0.0;
			errStd = 0.0;
		}
	}
}
