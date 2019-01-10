//
//	tuneable.h  --  support for tuning of numerical parameters
//
//	This allows changing some parameters via environment variables.
//
//	Values are set at initialization. Each parameter has a default value which
//	can be overridden by setting an associated environment variable.
//
//	This is intended for parameters which might need to be changed in the field,
//	but probably won't be. 
//
//	John Nagle
//	November, 2004
//	Team Overbot
//
#ifndef TUNEABLE_H
#define TUNEABLE_H
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
//
//	class Tuneable  -- describes one tuneable parameter
//
class Tuneable {
private:
	const char* m_name;					// name of param, and env. var  that controls it
	const double m_minval;				// minimum allowed value
	const double m_maxval;				// maximum allowed value
	const double m_initialval;				// initial value
	double m_val;								// value of parameter
	const char* m_msg;						// descriptive message
public:
	Tuneable(const char* name, double minval, double maxval, double val, const char* msg);
	const char* getname() const { return(m_name); } // name
	double getmin() const { return(m_minval); }
	double getmax() const { return(m_maxval); }
	void Dump();
	void FatalError(const char* msg);											// fatal error
	operator double() const { return(m_val); }							// implicit conversion to double
};
//
//	Implementation
//
//	Constructor
//
inline Tuneable::Tuneable(const char* name, double minval, double maxval, double val, const char* msg)
	: m_name(name), m_minval(minval), m_maxval(maxval), m_initialval(val), m_val(val), m_msg(msg)
{	assert(name);
	const char* envval = getenv(name);								// get environment variable if any
	if (envval)																			// if environment variable is present
	{	int cnt = sscanf(envval,"%lf",&m_val);							// read new value
		if (cnt != 1)																	// did not scan
		{	FatalError("Non-numeric value in environment variable");	}
		printf("Tuning parameter \"%s\" changed from %f to %f\n", name, m_initialval, m_val);	// debug output
		fflush(stdout);
	}
	if (val < minval || val > maxval)
	{	FatalError("Value out of range");	}
}
//
//	Dump  -- dump an entry
//
inline void Tuneable::Dump()
{	printf(" %s (%f..%f) = %f    (%s)	\n", getname(), getmin(), getmax(), double(*this),m_msg ? m_msg : "");
}
//
//	FatalError -- report error with a Tuneable
//
inline void Tuneable::FatalError(const char* msg)
{	printf("CONFIGURATION ERROR in tuneable parameter: %s.\n",msg);
	Dump();
	fflush(stdout);
	throw("CONFIGURATION ERROR in tuneable parameter");	// probably never caught, because we are in static init
}
#endif // TUNEABLE_H