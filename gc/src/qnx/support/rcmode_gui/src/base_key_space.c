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
#include "vehicle.h"

/**
 * cause brake to go to 100% and throttle to go to 0%
 */
int
base_key_space( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	//update numeric to the MAX_BRAKE
 	PtSetResource (ABW_numeric_throttle_and_brake, Pt_ARG_NUMERIC_VALUE, -MAX_BRAKE, 0);
 
	send_throttle_cmd();
	send_brake_cmd();
 
	return( Pt_CONTINUE );

	}

