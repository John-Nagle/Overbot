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

int
bigclickbutton_lostfocus( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	//printf("bigclickbutton_lostfocus\n");

	int * lost_focus_button_val;
	PtGetResource(ABW_onOffButtonLostFocusEmergency, Pt_ARG_ONOFF_STATE, &lost_focus_button_val, 0);
	
	//if this button is checked, we set the brake to 100% and throttle to 0%
	if (*lost_focus_button_val) {
	
		//update numeric to the MAX_BRAKEPRESSURE
 		PtSetResource (ABW_numeric_throttle_and_brake, Pt_ARG_NUMERIC_VALUE, -MAX_BRAKE, 0);
 
 		send_throttle_cmd();
 		send_brake_cmd();
 	}
	
	return( Pt_CONTINUE );

	}

