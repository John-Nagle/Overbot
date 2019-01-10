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

std::string g_current_directory("/tmp/maplogs");										// the current directory (global)


int
file_open_dlg_setup( PtWidget_t *link_instance, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	link_instance = link_instance, apinfo = apinfo, cbinfo = cbinfo;
	printf("file_open_dlg_setup: setting root dir to %s\n", g_current_directory.c_str());		//***TEMP***
	if (g_current_directory.size() > 0)						// if there is a previous directory to start from
	{	PtSetResource(ABW_file_open_dlg_fileselector, Pt_ARG_FS_ROOT_DIR, g_current_directory.c_str(), 0);	// set dir before dialog opens
    	PtSetResource(ABW_file_open_dir_text, Pt_ARG_TEXT_STRING, g_current_directory.c_str(), 0);	// set name in dialog box
	}
	return( Pt_CONTINUE );

	}

