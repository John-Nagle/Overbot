//
//	nan.h  -- Not a Number constant
//
//	This is a workaround for a QNX bug.  NAN in the QNX libraries is bad.
//
//	John Nagle
//	Team Overbot
//
#ifndef NAN_H
#define NAN_H
#include <math.h>
const float k_NaN = std::_FNan._Float;											// NAN, float form
#endif // NAN_H