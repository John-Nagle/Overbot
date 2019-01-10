/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

/* Standard headers */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>

/* Local headers */
#include "ablibs.h"
#include "navread.h"
#include "abimport.h"
#include "proto.h"


int
base_window_closing( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	//	Window is closing; destroy model
	NavRead* model = 0;;													// pointer to object
	PtGetResource( widget, Pt_ARG_POINTER, &model, 0 );	// get model link from widget
	PtSetResource( widget, Pt_ARG_POINTER, 0 ,0);				// clear link
	assert(model);																// must have
	delete(model);																// gone

	return( Pt_CONTINUE );

	}

