/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

/* Standard headers */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

/* Local headers */
#include "ablibs.h"
#include "global.h"
#include "abimport.h"
#include "proto.h"
//
//	Waypoint file info
//
int g_waypointoutfd = -1;														// output file descriptor
int g_waypointnum = 0;															// current waypoint number
const double waypointwidth = 13.3;											// width at each waypoint
const double waypointspeed = 15;												// default speed
//
//	addwaypoint  --  add waypoint menu item
//
int
addwaypoint( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	//	Add a waypoint to the "waypoint file"
	if (g_waypointoutfd < 0)																		// if no waypoint file
	{
		////PtNotice(widget,0,"No waypoint file is open.",0,"Use \"New waypoint file.\" first.",0,"OK",0,Pt_MODAL);
		return( Pt_CONTINUE );
	}
	double *latp;
	double *longp;
	//	Get latitude and longitude from display
	PtGetResource (ABW_valLat, Pt_ARG_NUMERIC_VALUE, &latp, 0);
	PtGetResource (ABW_valLong, Pt_ARG_NUMERIC_VALUE, &longp, 0);
	double latitude = *latp;
	double longitude = *longp;
	//	Add a line to the waypoint file.
	{	char s[100];
		g_waypointnum++;
		snprintf(s,sizeof(s), "%d, %1.6f, %1.6f, %1.2f, %1.0f\n",
			g_waypointnum, latitude, longitude, waypointwidth, waypointspeed);
		write(g_waypointoutfd, s, strlen(s));
	}
	return( Pt_CONTINUE );

	}


