/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

/* Standard headers */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <photon/realtime/RtTimer.h>

/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"
	
int positionViewInit( PtWidget_t *widget )  {
	

	//RtTimer_t * r = RtTimerCreate(CLOCK_SOFTTIME, 10, timer_callback, NULL);	
	return PtSuperClassInit( PtBasic, widget );
}

