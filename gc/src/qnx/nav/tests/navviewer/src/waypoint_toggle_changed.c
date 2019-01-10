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
waypoint_toggle_changed( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	NavRead* model = 0;										// pointer to model object
	PtWidget_t* window = 	ABW_base;					// get base window (NOTE: assumes single-window application)
	PtGetResource(window, Pt_ARG_POINTER, &model, 0 );	// get model link from base window
	assert(model);													// must have model
	//	Get event info
	PhEvent_t* event = cbinfo ? cbinfo->event : NULL;
	PhPointerEvent_t *ptr = event ? (PhPointerEvent_t *)
      PhGetData
      (event) : NULL;

	bool showwaypoints = !(model->showwaypoints());			// invert toggle
	ApModifyItemState(&view_menu, (showwaypoints ? AB_ITEM_SET : AB_ITEM_NORMAL), ABN_waypoint_toggle, NULL);
 	model->waypoint_toggle_changed(showwaypoints);		// do the work in the model

	return( Pt_CONTINUE );

	}

