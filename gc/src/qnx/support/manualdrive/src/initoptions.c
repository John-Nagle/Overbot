/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

/* Standard headers */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"
#include "manualdrive.h"

/* Application Options string */
const char ApOptions[] =
	AB_OPTIONS ""; /* Add your options in the "" */


int
initoptions( int argc, char *argv[] )

	{

	/* eliminate 'unreferenced' warnings */
	argc = argc, argv = argv;
	Manualdrive::init(argc, argv);
	return( Pt_CONTINUE );

	}

