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


int
brake_pressure_goal_meter_move( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	Manualdrive* model = 0;									// pointer to model object
	PtWidget_t* window = 	ABW_base;					// get base window (NOTE: assumes single-window application)
	PtGetResource(window, Pt_ARG_POINTER, &model, 0 );	// get model link from base window
	assert(model);													// must have model
	short* meterval = 0;
	//	Get meter bounds. This makes the settings in the widget irrelevant.
	PtGetResource( widget, Pt_ARG_METER_NEEDLE_POSITION, &meterval, 0 );	// get pointer to widget data
	assert(meterval);												// meter value must exist
	model->brake_pressure_goal_meter_move(*meterval);		// do the work in the model
	return( Pt_CONTINUE );

	}

