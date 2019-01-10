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
base_window_open( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	//	Allocate a Manualdrive object and tie it to the window.
	//	This is a model/view/controller architecture. 
	//	The Manualdrive object is the model.
	//	This widget is the view.
	try {
		Manualdrive* model = new Manualdrive;							// create the object
		PtSetResource(widget, Pt_ARG_POINTER, model, 0 );			// attach to widget
		return( Pt_CONTINUE );
	} 
	catch (const char* msg)
	{	printf("Unable to start: %s\n",msg);									// fails
		exit(1);
	}

	}

