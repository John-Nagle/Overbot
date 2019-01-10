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


int
button_toggle_gear_sim_activate( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	//get the current sim status
	int * val;
	PtGetResource(ABW_onOffButton_gear_sim, Pt_ARG_ONOFF_STATE, &val, 0);
	
	//send the opposite
	send_gear_sim(*val == 0);
	
	return( Pt_CONTINUE );

	}

