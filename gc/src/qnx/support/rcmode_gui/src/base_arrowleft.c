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
 * Called everytime left button pressed in the big click button
 */

int
base_arrowleft( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	//printf("base_arrowleft\n");
		
	int * val;
	PtGetResource (ABW_numeric_steering, Pt_ARG_NUMERIC_VALUE, &val, 0);	
	int newval = *val - 1;

	//some bounds check
	if (newval < MIN_STEERING) {
		newval = MIN_STEERING;
	}
	else if (newval > MAX_STEERING) {
		newval = MAX_STEERING;
	}
	
	//set the value on the numeric 
 	PtSetResource (ABW_numeric_steering, Pt_ARG_NUMERIC_VALUE, newval, 0); 
 	
	//actually send a msg to the steering server 	
 	send_steering_cmd();
  	
	return( Pt_CONTINUE );

	}

