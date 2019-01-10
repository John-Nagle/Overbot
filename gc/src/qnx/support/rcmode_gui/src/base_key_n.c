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
base_key_n( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	//printf("base_key_n\n");
	
	PtSetResource(ABW_onOffButton_gear_high, Pt_ARG_ONOFF_STATE, 0, 0);
	PtSetResource(ABW_onOffButton_gear_low, Pt_ARG_ONOFF_STATE, 0, 0);
	PtSetResource(ABW_onOffButton_gear_neutral, Pt_ARG_ONOFF_STATE, 1, 0);
	PtSetResource(ABW_onOffButton_gear_reverse, Pt_ARG_ONOFF_STATE, 0, 0);

	send_gear_cmd();		
	
	return( Pt_CONTINUE );

	}

