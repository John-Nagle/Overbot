/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

/* Standard headers */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

/* Local headers */
#include "ablibs.h"
#include "global.h"
#include "abimport.h"
#include "proto.h"


//
//	waypoint_file_dlg_open  -- "open" button pushed in file dialog
//
int
waypoint_file_dlg_open( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{
	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	if (g_waypointoutfd >= 0)
	{	close(g_waypointoutfd);	g_waypointoutfd = -1;	}	// close if already open
	//	File open button has been pushed.  Find out what we are opening.
	PtFileSelItem_t* selectionset[2];						// room for two selectors
	//	Get selection set. 
	//	NOTE: dialog must have flags set to limit to one selection, or buffer will overflow.
	PtFSSelectedItems(ABW_waypoint_directory_selector,selectionset);	// fill selection set
	PtFileSelItem_t* selection = selectionset[0];		// first selection or null
	if (selection == 0) return(Pt_END);					// no selection, ignore open button
	assert(selection->fullpath);								// we should have a full path
	printf("Directory \"%s\"\n",selection->fullpath);	// ***TEMP***
	//// int stat = model->open(selection->fullpath);	// try to open log file
	const char* dir = selection->fullpath;				// directory name
	const char* fnp = 0;											// filename
	PtGetResource(ABW_waypoint_filename_text, Pt_ARG_TEXT_STRING, &fnp, 0);	// get text
	if (!fnp || (!fnp[0])) return(Pt_END);					// if got nothing, consume event, do not close
	char fname[512];												// build full filename
	snprintf(fname,sizeof(fname),"%s/%s",dir,fnp);	// build filename
	printf("Creating %s\n",fname);							// ***TEMP***
	g_waypointoutfd = creat(fname, S_IRUSR|S_IRGRP|S_IROTH);	// try to create as read only
	if (g_waypointoutfd < 0)									// if unsuccessful open
	{																		// pop up a standard dialog, centered over the open dialog
		PtNotice(widget,0,"Unable to create waypoint file",0,strerror(errno),0,"OK",0,Pt_MODAL|Pt_RELATIVE);
		return (Pt_END);											// consume this event; file dialog will not close
	}
	g_waypointnum = 0;											// restart waypoint numbering
	char s[100];														// put file header in file
	time_t time_of_day;
	time_of_day = time( NULL );
 	snprintf(s,sizeof(s), "#  Waypoint file created on %s", ctime( &time_of_day ));
	write(g_waypointoutfd, s, strlen(s));
	return( Pt_CONTINUE );										// we are done, happy
	}

