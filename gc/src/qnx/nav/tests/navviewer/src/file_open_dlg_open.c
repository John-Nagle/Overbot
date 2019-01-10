/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

/* Standard headers */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"
#include "navread.h"
#include <libgen.h>

int
file_open_dlg_open( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{
	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	NavRead* model = 0;										// pointer to model object
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
	////printf("Opening \"%s\"\n",selection->fullpath);	// ***TEMP***
	int stat = model->open(selection->fullpath);	// try to open log file
	if (stat != EOK && errno != EOK)						// if unsuccessful open
	{																		// pop up a standard dialog, centered over the open dialog
		PtNotice(widget,0,"Unable to open log file",0,strerror(errno),0,"OK",0,Pt_MODAL|Pt_RELATIVE);
		return (Pt_END);											// consume this event; file dialog will not close
	}
	//	Successful open, use same directory for next open.
	char dirwork[512];											// dir workspace
	strncpy(dirwork,selection->fullpath,sizeof(dirwork));	// copy filename just opened
	g_current_directory = dirname(dirwork);					// dir for next time
	return( Pt_CONTINUE );										// we are done, happy
	}


int
file_open_dlg_fileselector( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	//	Put directory being browsed in dialog box, so user can see directory.
	char* rootdir = 0;
	PtGetResource(widget, Pt_ARG_FS_ROOT_DIR, &rootdir, 0 );	// get root directory of file selector
	if (rootdir != NULL)																// if have root dir now
	{
		PtSetResource(ABW_file_open_dir_text, Pt_ARG_TEXT_STRING, rootdir, 0);	// set name in dialog box
	}
	return( Pt_CONTINUE );

	}

