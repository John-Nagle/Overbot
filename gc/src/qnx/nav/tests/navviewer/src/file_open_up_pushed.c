/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

/* Standard headers */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <libgen.h>

/* Local headers */
#include "ablibs.h"
#include "navread.h"
#include "abimport.h"
#include "proto.h"


int
file_open_up_pushed( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	//	Move up one directory level.
	char* rootdir = 0;
	PtGetResource(ABW_file_open_dlg_fileselector, Pt_ARG_FS_ROOT_DIR, &rootdir, 0 );	// get root directory of file selector
	if (rootdir != NULL)																// if have root dir now
	{
		char path[512];																// traditional pathname length
		strncpy(path, rootdir, sizeof(path));									// copy dir
		const char* dir = dirname(path);										// extract parent dir
		if (dir)																				// if meaningful
		{	PtSetResource(ABW_file_open_dlg_fileselector, Pt_ARG_FS_ROOT_DIR, dir, 0);	// set new path
		}
	}
	return( Pt_CONTINUE );

	}

