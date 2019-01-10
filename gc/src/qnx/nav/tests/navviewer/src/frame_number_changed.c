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
#include "navread.h"
#include "abimport.h"
#include "proto.h"


int
frame_number_changed( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	NavRead* model = 0;										// pointer to model object
	PtWidget_t* window = 	ABW_base;					// get base window (NOTE: assumes single-window application)
	PtGetResource(window, Pt_ARG_POINTER, &model, 0 );	// get model link from base window
	assert(model);													// must have model
	int* framevalp;													// pointer to frame number within widget
	PtGetResource(ABW_frame_number_box, Pt_ARG_NUMERIC_VALUE, &framevalp,0);	// get frame number
	int frame = *framevalp;									// get frame value
	model->slew(frame);										// do the work in the model

	return( Pt_CONTINUE );

	}

