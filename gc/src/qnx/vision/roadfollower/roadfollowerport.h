/**
 * This header file contains macros and typedefs used to provide portability
 * of common functions and data types.
 *
 */
#include <iostream.h>

#define MYERROR(msg)  {cerr << msg << endl; exit(1);}

#ifdef WIN32
#include <highgui.h>
#include <float.h>
typedef HWND MYWINDOW;
#define snprintf _snprintf
#define finite _finite
#endif
