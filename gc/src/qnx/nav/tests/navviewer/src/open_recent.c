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

#include<sys/types.h>
#include<dirent.h>

int
open_recent( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo ) {
  /* eliminate 'unreferenced' warnings */
  widget = widget, apinfo = apinfo, cbinfo = cbinfo;



// ---------------------------------------------------

  NavRead *model=0;                // pointer to model object
  PtWidget_t *window=ABW_base;     // get base window (NOTE: assumes single-window application)
  char buffer[512]="/"; 						// path buffer, ugly to save stack space
  strncpy(buffer,g_current_directory.c_str(), sizeof(buffer));	// use latest directory opened
                                   // DO NOT initialize to a string of length 512

  // get model link from base window
  PtGetResource(window,Pt_ARG_POINTER,&model,0);
  // must have model
  assert(model);

  // at this point buffer contains the path
  DIR *dir=opendir(buffer);
  if(dir==NULL) return(Pt_END);
  // add a trailing slash
  int offset=strlen(buffer);
  buffer[offset]='/';
  buffer[offset+1]=0;
  offset++;
  // now buffer contains the /-terminated path
  // and buffer+offset will hold the filename
  while(1) {
    struct dirent *de=readdir(dir);
    if(!de) break;
    if(strcmp(buffer+offset,de->d_name)>=0) continue;
    strncpy(buffer+offset,de->d_name,512-1-offset);
    buffer[512-1]=0;
  }
  closedir(dir);

  // try to open log file
  int stat=model->open(buffer);
  if(stat!=EOK && errno!=EOK) {
    // if unsuccessful open
    // pop up a standard dialog, centered over the open dialog

    // copied from file_open_dlg_open.c
    // does this code even work ???
    //PtNotice(widget,0,"Unable to open log file",0,strerror(errno),0,"OK",0,Pt_MODAL|Pt_RELATIVE);

    return(Pt_END);
  }

// ---------------------------------------------------



  return(Pt_CONTINUE);
}

