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
#include <sys/time.h>


int
timer_periodic_activate( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

    if (gear_initialized()) {
	    get_gear_data();
	}
	
	if (brake_initialized()) {
	    get_brake_data();
	}
	
   if (throttle_initialized()) {
	    get_throttle_data();
	}

    if (steering_initialized()) {
	    get_steering_data();
	}	

	return( Pt_CONTINUE );

	}

