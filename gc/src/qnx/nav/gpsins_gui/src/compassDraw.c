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
#include "global.h"

static int compassSizeX;
static int compassSizeY;

void drawAxes()
{
	PhRect_t r = {0, 0, compassSizeX, compassSizeY};
	PhPoint_t g = { 2, 2 };

    PgSetStrokeColor( Pg_BLACK);
    PgDrawGrid( &r, &g );

}

void drawHeading()
{
	if (poses.size() == 0) {
		return;
	}
	
	//center in the middle of the grid
	PhPoint_t translation;
	translation.x = compassSizeX/2, translation.y = compassSizeY/2;
	PgSetTranslation(&translation, Pg_RELATIVE);	

	double angle = poses.back().rpy[2]	; //deg
	
	//draw polygon
    const PhPoint_t start_point = { 0,0};
    int num_points = 3;
	
	//here we perform the rotation for a triangle
	short new_x0, new_y0, new_x1, new_y1, new_x2, new_y2;
	
	rotate(0, -50, &new_x0, &new_y0, angle);
	rotate(-30, 50, &new_x1, &new_y1, angle);
	rotate(30, 50, &new_x2, &new_y2, angle);
    
    PhPoint_t points[3] = {     
	    { new_x0,  			  new_y0}, 
        { new_x1 , 		      new_y1}, 
        { new_x2, 			  new_y2}
    };
	
    PgSetFillColor( Pg_RED );
    PgSetStrokeColor( Pg_WHITE );
    PgDrawPolygon( points, num_points, &start_point, 
    Pg_DRAW_FILL_STROKE | Pg_CLOSED );

	PhPoint_t translationBack;
	translationBack.x = -compassSizeX/2, translationBack.y = -compassSizeY/2;
	PgSetTranslation(&translationBack, Pg_RELATIVE);	

}

void compassDraw( PtWidget_t *widget, PhTile_t *damage )  {
	PtSuperClassDraw( PtBasic, widget, damage );
	
	//get the raw canvas
	PhRect_t  raw_canvas;
	PtCalcCanvas (widget, &raw_canvas);
	//clip the sizes
	PtClipAdd(widget, &raw_canvas);	
	//translate relative to the canvas
	PgSetTranslation (&raw_canvas.ul, Pg_RELATIVE);

	compassSizeX = raw_canvas.lr.x - raw_canvas.ul.x;
	compassSizeY = raw_canvas.lr.y - raw_canvas.ul.y;
	
	drawAxes();
	drawHeading();

	//translate back
	raw_canvas.ul.x *= -1;
	raw_canvas.ul.y *= -1;
	PgSetTranslation (&raw_canvas.ul, Pg_RELATIVE);	

	//remove clipping	
	PtClipRemove();
	
}

