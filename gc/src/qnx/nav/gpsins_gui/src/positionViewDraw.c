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
#include "global.h"
#include "../../../../common/include/gpsins_messaging.h"
#include <vector>

using namespace std;

//the history of poses
vector <struct GPSINSMsgRep> poses;
bool mPause = false;

float inc = 1.0; //meters

double posZero[3] = {0.0, 0.0, 0.0};

int sizeX, sizeY;

int gridX = 8;
int gridY = 8;

void drawGrid()
{

	PhRect_t r = {0, 0, sizeX, sizeY};
	PhPoint_t g = { gridX, gridY };

    PgSetStrokeColor( Pg_BLACK);
    PgDrawGrid( &r, &g );
}

void scaleForCanvas(double x, double y, double * scaledX, double * scaledY)
{
	float pixelPerDivX = sizeX/gridX;
	float pixelPerDivY = sizeY/gridY;

	float pixelPerMeterX = pixelPerDivX / inc; //inc really is meterPerDiv
	float pixelPerMeterY = pixelPerDivY / inc;
	
	*scaledX = x * pixelPerMeterX;
	*scaledY = y * pixelPerMeterY;
}

/**
 * @param x and y in screen space
 */
void drawPose(double y, double x, double heading, int color)
{
	//draw polygon
    const PhPoint_t start_point = { 0,0};
    int num_points = 3;
    
    double scaledX, scaledY;
    
    //here we perform the rotation for a triangle
	short int new_x0, new_y0, new_x1, new_y1, new_x2, new_y2;
	
	rotate(0, -10, &new_x0, &new_y0, heading);
	rotate(-5, 10, &new_x1, &new_y1, heading);
	rotate(5, 10, &new_x2, &new_y2, heading);
    
    scaleForCanvas(x,y, &scaledX, &scaledY);
    
    PhPoint_t points[3] = {     
	    { new_x0 + (int)scaledX,  			new_y0 + (int)scaledY}, 
        { new_x1 + (int)scaledX, 		new_y1 + (int)scaledY}, 
        { new_x2 + (int)scaledX, 			new_y2 + (int)scaledY},
    };
	
    PgSetFillColor( color );
    PgSetStrokeColor( Pg_WHITE );
    PgDrawPolygon( points, num_points, &start_point, 
    Pg_DRAW_FILL_STROKE | Pg_CLOSED );
}

void drawPoses()
{
		//center in the middle of the grid
		PhPoint_t translation;
		translation.x = sizeX/2, translation.y = sizeY/2;
		PgSetTranslation(&translation, Pg_RELATIVE);	
		
		//zero if we have not zeroed
		if (poses.size() >= 1 && posZero[0] == 0.0) {
			posZero[0] = poses.back().pos[0];
			posZero[1] = poses.back().pos[1];
			posZero[2] = poses.back().pos[2];
		}
		
		//draw all poses
		vector<struct GPSINSMsgRep>::iterator iter;
		unsigned int cnt = 1;
		for (iter = poses.begin(); iter != poses.end(); ++iter) {
				drawPose(-((*iter).pos[0] - 	posZero[0]),
								(*iter).pos[1] - posZero[1],
								(*iter).rpy[2],
								cnt == poses.size() ? Pg_RED : Pg_BLUE
								);
				cnt++;
		}		
			
		PhPoint_t translationBack;
		translationBack.x = -sizeX/2, translationBack.y = -sizeY/2;
		PgSetTranslation(&translationBack, Pg_RELATIVE);	
		
}

void positionViewDraw( PtWidget_t *widget, PhTile_t *damage )  {
	PtSuperClassDraw( PtBasic, widget, damage );

	//get the raw canvas
	PhRect_t  raw_canvas;
	PtCalcCanvas (widget, &raw_canvas);
	//clip the sizes
	PtClipAdd(widget, &raw_canvas);	
	//translate relative to the canvas
	PgSetTranslation (&raw_canvas.ul, Pg_RELATIVE);

	sizeX = raw_canvas.lr.x - raw_canvas.ul.x;
	sizeY = raw_canvas.lr.y - raw_canvas.ul.y;
	
	drawGrid();
	drawPoses();
	updateZoom();

	//translate back
	raw_canvas.ul.x *= -1;
	raw_canvas.ul.y *= -1;
	PgSetTranslation (&raw_canvas.ul, Pg_RELATIVE);

	//remove clipping	
	PtClipRemove();
}

