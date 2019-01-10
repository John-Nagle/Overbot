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
#include "global.h"
#include "abimport.h"
#include "proto.h"


int
trendUncRealized( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

//	PtTrendAttr_t one_attr, several_attr[3];
	PtArg_t args[2];

	// set the number of trends to 3
	PtSetArg (&args[0], Pt_ARG_TREND_COUNT, 3 , 1);
	PtSetResources (widget, 1, args);

	/* Set up the color list. */
	int trend_color_array[3] = {Pg_GREEN, Pg_RED, Pg_YELLOW};
	PtSetArg (&args[0], Pt_ARG_TREND_COLOR_LIST, 
          trend_color_array, 3);
	PtSetResources (widget, 1, args);

	PtTrendAttr_t several_attr[3];
	/* Map the trends to colors. */
	several_attr[0].map = 3;  /* Trend 0 is Pg_YELLOW */
	several_attr[1].map = 2;  /* Trend 1 is Pg_RED    */
	several_attr[2].map = 4;  /* trend 2 is Pg_BLUE   */

	PtSetArg (&args[0], Pt_ARG_TREND_ATTRIBUTES, several_attr, 0);
	PtSetResources (widget, 1, args);

	/* set min */
	//PtSetArg(&args[0], Pt_ARG_TREND_MIN, 0, 1);
	//PtSetResources(widget, 1, args);
	
	/* set max */
	//PtSetArg(&args[0], Pt_ARG_TREND_MAX, 1000, 1);
	//PtSetResources(widget, 1, args);	

	/* set the time frame that the trend would show */
	//PtSetArg(&args[0], Pt_ARG_TREND_PALETTE_END, 60 * UPDATEFREQ * 10, 0);
	//PtSetResources(widget, 1, args);

	return( Pt_CONTINUE );

	}

