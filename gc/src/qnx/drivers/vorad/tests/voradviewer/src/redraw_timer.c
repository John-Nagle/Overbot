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
redraw_timer( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	//	If run button is on, force a redraw on every timer cycle.
	short *runstate;												// run/stopped state
	PtGetResource(ABW_vorad_run_button,Pt_ARG_ONOFF_STATE,&runstate,0);	// is run button turned on?
	if (*runstate)														// if in run state
	{	//	Force redraw of this window
		PtDamageWidget(ABW_vorad_port);				// forces a redraw
	}
	return( Pt_CONTINUE );

	}

