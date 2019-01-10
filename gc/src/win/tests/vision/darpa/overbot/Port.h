/**
 * This header file contains macros and typedefs used to provide portability
 * of common functions and data types.
 *
 */
#include <iostream.h>
#include <highgui.h>

#define MYERROR(msg)  {cerr << msg << endl; exit(1);}

#ifdef WIN32
typedef HWND MYWINDOW;
#endif
