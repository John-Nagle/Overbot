//
//	errnoexception.h  --  error exceptions for QNX
//
//	John Nagle
//	Team Overbot
//	May, 2003
//
#ifndef ERRNOEXCEPTION_H
#define ERRNOEXCEPTION_H

#include <string.h>
#include <errno.h>
#include <exception>
using namespace std;
//
//	class errno_error -- an error expressed as a POSIX error code
//
class errno_error: public exception {
private:
	int		m_errno;												// the error number
public:
	errno_error(int stat)										// constructor -takes POSIX error code
	: m_errno(stat) {}
	errno_error()												// default constructor - no error
	: m_errno(EOK) {}
	const char *what( ) const throw( )				// printable form of exception
	{	return(strerror(m_errno));	}						// use standard text from library
public:
	//	checkok  --  use to check functions which return a POSIX error code.
	static inline void checkok(int stat)				// check for EOK return, throw if fail	
	{	if (stat != EOK) throw(errno_error(stat));	}	// throw as errno_error
};
#endif // ERRNOEXCEPTION_H