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
 * Called everytime up button pressed in the big click button
 */
int
base_arrowup( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	//printf("base_arrowup\n");
	
	//get the value	
	int * val;	
	PtGetResource (ABW_numeric_throttle_and_brake,  Pt_ARG_NUMERIC_VALUE, &val, 0);
	int newval = *val + 1;
	
	//do bounds check
	if (newval < -MAX_BRAKE) {
		newval = -MAX_BRAKE;
	}
	else if (newval > MAX_THROTTLE) {
		newval = MAX_THROTTLE;
	}
	
	//update numeric
 	PtSetResource (ABW_numeric_throttle_and_brake, Pt_ARG_NUMERIC_VALUE, newval, 0);
 
 	send_throttle_cmd();
 	send_brake_cmd();
  	
	return( Pt_CONTINUE );

	}

