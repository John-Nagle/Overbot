//
//	voradread.cpp  --  support for reading from a FireWire vorad using our driver
//
//	John Nagle
//	Team Overbot
//	October, 2003
//
#include <assert.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include "voradread.h"
#include "voradserver.h"
#include "abdefine.h"											// it would be better to include this only in a generated item
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"

//
//	Constants
//
const char* servername = "RVORAD";					// name of server as remote client
const char* servernode = "gcrear0";

const int k_target_radius = 20;								// target size on screen
const float k_scale = 10.0;									// pixels/meter
const float k_vscale = 60.0;									// pixels/mps
//
//	Constructor
//
VoradRead::VoradRead()
: m_voradserver(servernode, servername,0.0)
{
}
//	
//	Destructor
//
VoradRead::~VoradRead()
{	

}

//
//	drawtarget -- draw a target icon within a raw window
//
//	Draws a filled circle at PtPoint, then a tailing vector up to it.
//
static void drawtarget(const PhPoint_t& pos, const PhPoint_t& vel, const PgColor_t& color)
{	const PhPoint_t circle = { k_target_radius, k_target_radius };// defines a circle
	PgSetStrokeColor(Pg_BLACK);											// outline color
	PgSetFillColor(color);															// fill color
	PgDrawILine(pos.x,pos.y, pos.x-vel.x, pos.y-vel.y);			// draw line to target
	PgDrawArc(&pos, &circle,  0, 0, Pg_DRAW_FILL_STROKE | Pg_ARC );	// draw filled circle at target
}

//
//	Draw message in center of widget
//
static void drawmsg(PtWidget_t * widget, const char* s)
{	const int textHeight = 9;												// size of text, points
    PhPoint_t p = { 5,100 };												// ***TEMP*** should be center
    char Helvetica[MAX_FONT_TAG];
    if (PfGenerateFontName("Helvetica", 0, textHeight,
                          Helvetica) == NULL) {
        {	perror("Unable to find font"); return; }
    } else {
        PgSetFont( Helvetica );
    }
    PgSetTextColor( Pg_BLACK );
    PgDrawText( s, strlen( s ), &p, 0 );
}
//
//	drawtargetitem  -- draw target given target data
//
void VoradRead::drawtargetitem(PtWidget_t* widget, const VoradTargetItem& target)
{
	const PhRect_t* rect = PtCalcCanvas(widget,0);			// get rectangle for widget
	int width = rect->lr.x - rect->ul.x;									// width
	int height = rect->lr.y - rect->ul.y;								// height
	PhPoint_t origin = {height, width/2 };							// the vehicle is here
	int x = int((width/2) - target.m_targ_y*k_scale);			// the target is here.
	int y = int(height - target.m_targ_x*k_scale);				// X is ahead, Y is left
	PhPoint_t loc = { x, y };												// the target location
	int vx = 0;																		// we do not have cross-track velocity
	int vy = int(target.m_targ_rrange*k_vscale);					// velocity
	PhPoint_t vel = { vx, vy };
	drawtarget(loc,vel,Pg_RED);											// show target
}
//
//	video_port_draw -- draw video into indicated port
//
void VoradRead::video_port_draw(PtWidget_t *widget, PhTile_t *damage)
{
	//	Call server for an image
	VoradServerMsgVDRQ targreq;											// VORAD target request
	VoradServerMsgVDTG targreply;										// the targets
	targreq.m_msgtype = VoradServerMsgVDRQ::k_msgtype;	// set type
	//	Call the server
	int stat = m_voradserver.MsgSend(targreq,targreply);
	if (stat < 0)																		// if fail
	{	drawmsg(widget,strerror(errno));									// display error
		return;																			// done
	}
	//	We have a valid target message - draw its targets
	////printf("%d targets\n",targreply.m_targetcount);				// ***TEMP***
	float threatlev = 0.0;															// threat leve of most threatening target
	for (int i=0; i<targreply.m_targetcount; i++)						// for each target
	{	const VoradTargetItem& targ = targreply.m_targets[i];	// this target
		drawtargetitem(widget, targ);										// draw this target
		////printf("Target at (%3.1f %3.1f) vel %2.1f  Threat %1.1f\n",
			////targ.m_targ_x, targ.m_targ_y, targ.m_targ_rrange, targ.m_collisionthreatlev);
		if (targ.m_collisionthreatlev > threatlev) threatlev = targ.m_collisionthreatlev;	// save collision threat level
	}
	//	Set threat meter
	int threatlevint = int(threatlev * 100);								// threat level meter value													
	PtSetResource(ABW_threat_meter, Pt_ARG_METER_NEEDLE_POSITION, threatlevint, 0);	// is run button turned on?
}

