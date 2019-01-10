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
#include "cameraread.h"
#include "abimport.h"
#include "proto.h"


void video_port_draw( PtWidget_t *widget, PhTile_t *damage )  {
	PtSuperClassDraw( PtBasic, widget, damage );
	
	CameraRead* model = 0;									// pointer to model object
	PtWidget_t* window = 	ABW_base;					// get base window (NOTE: assumes single-window application)
	PtGetResource(window, Pt_ARG_POINTER, &model, 0 );	// get model link from base window
	assert(model);													// must have model
	model->video_port_draw(widget,damage);		// do the work in the model
	}

