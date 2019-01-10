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
load_waypoints_selected( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{
	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	Manualdrive* model = 0;									// pointer to model object
	PtWidget_t* window = 	ABW_base;					// get base window (NOTE: assumes single-window application)
	PtGetResource(window, Pt_ARG_POINTER, &model, 0 );	// get model link from base window
	assert(model);													// must have model
	//	File open button has been pushed.  Find out what we are opening.
	PtFileSelItem_t* selectionset[2];						// room for two selectors
	//	Get selection set. 
	//	NOTE: dialog must have flags set to limit to one selection, or buffer will overflow.
	PtFSSelectedItems(ABW_file_open_dlg_fileselector,selectionset);	// fill selection set
	PtFileSelItem_t* selection = selectionset[0];		// first selection or null
	if (selection == 0) return(Pt_END);					// no selection, ignore open button
	assert(selection->fullpath);								// we should have a full path
	int stat = model->openwaypoints(selection->fullpath);	// try to open log file
	if (stat != EOK && errno != EOK)						// if unsuccessful open
	{																		// pop up a standard dialog, centered over the open dialog
		PtNotice(widget,0,"Unable to open waypoint file",0,strerror(errno),0,"OK",0,Pt_MODAL|Pt_RELATIVE);
		return (Pt_END);											// consume this event; file dialog will not close
	}
	return( Pt_CONTINUE );

	}

