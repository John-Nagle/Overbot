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
#include "voradread.h"


int
base_window_open( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	//	Allocate a VoradRead object and tie it to the window.
	//	This is a model/view/controller architecture. 
	//	The CameraRead object is the model.
	//	This widget is the view.
	VoradRead* model = new VoradRead;						// create the object
	PtSetResource(widget, Pt_ARG_POINTER, model, 0 );			// attach to widget

	return( Pt_CONTINUE );

	}

