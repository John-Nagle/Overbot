/////////////////////////////////////////////////////////////////////////////
//
//    File: linearreg.h
//
//    Usage:
//        #include "linearreg.h"
//
//        LinearRegression<float> l;	// to instantiate a float regression
//        LinearRegression<double> l;	// to instantiate a double regression
//
//    Description:
//        Header file for linearreg.cc.
//
//        A linear regression class.  Computes the best fit straight line fit
//        to a set of coordinates.  Also computes the correlation coefficient
//        (CoefCorr) and standard error (ErrStd).
//				y = slope * x + offset
//
//        CoefCorr =1 means perfect correlation, =0 means no correlation.
//        ErrStd means 68% probability that the real value is between the
//        value estimated by the regression analysis plus the standard
//        error and the estimated value minus the standard error.  There
//        is a 99% probability that the real value lies within 2.58 times
//        the standard error of the estimated value.
//
//        Uses a template so that either floats or doubles may be used.
//
//        References: Statistical Treatment of Experimental Data by Young,
//                    and C/C++ Users Journal, August 2000
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        October, 2003
//
/////////////////////////////////////////////////////////////////////////////

#ifndef LINEAR_REG_H
#define LINEAR_REG_H


template<class Type>
class LinearRegression {
public:
	// default constructor
	LinearRegression();
	
	void PointAdd(Type x, Type y);	// add a data point to the collection
	long PointNum();				// return the number of data points
	void PointReset();				// remove all points from the collection
	
	bool CalcsReady();				// true if at least 3 points in collection
	Type Slope();					// returns the best-fit slope
	Type Offset();					// returns the best-fit offset
	Type CoefCorr();				// returns the correlation coefficient
	Type ErrStd();					// returns the standard error
private:
	long n;							// number of data points
	Type sumX;
	Type sumY;
	Type sumXsquared;
	Type sumYsquared;
	Type sumXY;
	
	Type slope;
	Type offset;
	Type coefCorr;
	Type errStd;
	
	void initialize();
	void calculate();
};

//
// Explicit template instantiation
//
// allow Type = float or double
//
template class LinearRegression<float>;
template class LinearRegression<double>;

#endif // LINEAR_REG_H