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

using namespace std;

int artHorizonSizeX;
int artHorizonSizeY;

//angles in deg
double roll; 
double pitch;
double yaw;

/*
 * BOUNDARY LINES ON EDGES AND ALITUTDE/SPEED READOUT
 */
void
drawText()
{
/*
	char			buffer[20];

	fl_color( 0, 0, 0 );
	fl_begin_polygon();
		fl_vertex(  0.18, 0.16 );
		fl_vertex(  0.10, 0.16 );
		fl_vertex(  0.10, 0.13 );
		fl_vertex(  0.18, 0.13 );
	fl_end_polygon();

	fl_begin_polygon();
		fl_vertex( -0.19, 0.16 );
		fl_vertex( -0.10, 0.16 );
		fl_vertex( -0.10, 0.13 );
		fl_vertex( -0.19, 0.13 );
	fl_end_polygon();

	fl_begin_polygon();
		fl_vertex(  0.18, -0.18 );
		fl_vertex(  0.14, -0.18 );
		fl_vertex(  0.14, -0.15 );
		fl_vertex(  0.18, -0.15 );
	fl_end_polygon();

	// altitude readout
	fl_color( 0, 255, 0 );

	sprintf( buffer, "%3.2fft", altitude );
	this->show_message( buffer, -0.17, 0.14 );

	// speed readout
	sprintf(buffer, "%3.2fk", speed );
	this->show_message( buffer, 0.12, 0.14 );

	// message readout
	this->show_message( message, 0.145, -0.17 );
	*/
}


/*
 * Draw the blue and brown parts of the sky / ground display.
 */
void
drawSky()
{

	//center in the middle of the grid
	PhPoint_t translation;
	translation.x = 0.0, translation.y = pitch * C_DEG2RAD * artHorizonSizeY/2.0;
	PgSetTranslation(&translation, Pg_RELATIVE);	

	// draw the blue sky part	
	int X_bound	= (int) ((180 + 15) * C_DEG2RAD * 20 * artHorizonSizeY);
	int Y_bound	= (int) (( 90 + 15) * C_DEG2RAD * 20 * artHorizonSizeY);

	PhPoint_t start_point = { 0, 0 };
    int num_points = 4;
    PhPoint_t points[4];

	rotate(-X_bound,	 -Y_bound,  &(points[0].x), &(points[0].y), -roll);
	rotate(X_bound,	 -Y_bound,  &(points[1].x), &(points[1].y), -roll);
	rotate(X_bound,	 0,  &(points[2].x), &(points[2].y), -roll);
	rotate(-X_bound,	 0,  &(points[3].x), &(points[3].y), -roll);

    PgSetFillColor( Pg_BLUE );
    PgSetStrokeColor( Pg_BLUE );
    PgDrawPolygon( points, num_points, &start_point, 
    Pg_DRAW_FILL_STROKE | Pg_CLOSED );	

	// Bottom (ground) of display
	rotate(-X_bound,	 Y_bound,  &(points[0].x), &(points[0].y), -roll);
	rotate(X_bound,	 Y_bound,  &(points[1].x), &(points[1].y), -roll);
	rotate(X_bound,	 0,  &(points[2].x), &(points[2].y), -roll);
	rotate(-X_bound,	 0,  &(points[3].x), &(points[3].y), -roll);

    PgSetFillColor( Pg_BROWN );
    PgSetStrokeColor( Pg_BROWN);
    PgDrawPolygon( points, num_points, &start_point, 
    Pg_DRAW_FILL_STROKE | Pg_CLOSED );

	PhPoint_t translationBack;
	translationBack.x = 0.0, translationBack.y = pitch * C_DEG2RAD * artHorizonSizeY/2.0;
	PgSetTranslation(&translationBack, Pg_RELATIVE);	


}


/*
 * Draw the roll ticks at the top of the AI and the desired
 * roll angle indicator.
 */
void
drawRoll()
{
/*
	fl_color( 255, 255, 255 );

	//Roll angle ticks
	for( int n=-30 ; n<=30 ; n+=15 )
	{
		fl_push_matrix();
		fl_rotate( n );
		fl_begin_line();
			fl_vertex( 0.0, 0.23 );
			fl_vertex( 0.0, 0.24 );
		fl_end_line();
		fl_pop_matrix();
	}

	/// Tick triangle
	fl_push_matrix();
	fl_rotate( roll );
	fl_color( 216, 128, 25 );
	fl_begin_polygon();
		fl_vertex(  0.00, 0.23 );
		fl_vertex(  0.01, 0.21 );
		fl_vertex( -0.01, 0.21 );
	fl_end_polygon();
	fl_pop_matrix();

	if( this->draw_desired )
	{
		// "Zero" roll angle
		fl_push_matrix();
		fl_rotate( zero_roll );
		fl_color( 255, 0, 0 );
		fl_begin_polygon();
			fl_vertex(  0.000, 0.22 );
			fl_vertex(  0.015, 0.25 );
			fl_vertex( -0.015, 0.25 );
		fl_end_polygon();
		fl_pop_matrix();

		// Desired roll angle 
		fl_push_matrix();
		fl_rotate( desired_roll );
		fl_color( 255, 255, 0 );
		fl_begin_polygon();
			fl_vertex(  0.000, 0.22 );
			fl_vertex(  0.015, 0.25 );
			fl_vertex( -0.015, 0.25 );
		fl_end_polygon();
		fl_pop_matrix();
	}
*/
}



/*
 * Draw the pitch bars and desired pitch angle marker
 */
void
drawPitch()
{
/*
	char			buffer[20];

	fl_push_matrix();
	fl_rotate( roll );
	fl_translate( 0.0, -pitch*C_DEG2RAD );
	fl_color( 255, 255, 255 );

	for( int n=0 ; n<9 ; ++n )
	{
		const double temp = (double)(n*10+10)*C_DEG2RAD;

		// positive pitch lines
		fl_begin_line();
			fl_vertex( -0.10, temp - 0.01 );
			fl_vertex( -0.10, temp );
			fl_vertex( -0.03, temp );
		fl_end_line();

		fl_begin_line();
			fl_vertex(  0.10, temp - 0.01 );
			fl_vertex(  0.10, temp );
			fl_vertex(  0.03, temp );
		fl_end_line();

		sprintf( buffer, "%d", n*10+10 );
		this->show_message( buffer,  0.11, temp-0.007 );
		this->show_message( buffer, -0.13, temp-0.007 );

		// negative pitch lines
		fl_begin_line();
			fl_vertex( -0.10, -temp+0.01 );
			fl_vertex( -0.10, -temp );
			fl_vertex( -0.03, -temp );
		fl_end_line();

		fl_begin_line();
			fl_vertex(  0.10, -temp+0.01 );
			fl_vertex(  0.10, -temp );
			fl_vertex(  0.03, -temp );
		fl_end_line();

		sprintf( buffer, "%d", -(n*10+10) );
		this->show_message( buffer,  0.11, -temp );
		this->show_message( buffer, -0.14, -temp );
	}

	// +/- 5 degree tick marks
	fl_begin_line();
		fl_vertex( -0.05,  5.0*C_DEG2RAD );
		fl_vertex(  0.05,  5.0*C_DEG2RAD );
	fl_end_line();

	fl_begin_line();
		fl_vertex( -0.05, -5.0*C_DEG2RAD );
		fl_vertex(  0.05, -5.0*C_DEG2RAD );
	fl_end_line();

	// Desired pitch angle marker
	if( this->draw_desired )
	{
		const double desired	= desired_pitch * C_DEG2RAD;
		const double zero	= zero_pitch * C_DEG2RAD;

		// "Zero" pitch angle marker
		fl_push_matrix();
		fl_color( 255, 0, 0 );
		fl_begin_polygon();
			fl_vertex( -0.085, zero - 0.015 );
			fl_vertex( -0.070, zero );
			fl_vertex( -0.085, zero + 0.015 );
		fl_end_polygon();
		fl_pop_matrix();

		// Desired pitch marker
		fl_push_matrix();
		fl_color( 255, 255, 0 );
		fl_begin_polygon();
			fl_vertex( -0.085, desired - 0.015 );
			fl_vertex( -0.070, desired );
			fl_vertex( -0.085, desired + 0.015 );
		fl_end_polygon();
		fl_pop_matrix();

	}

	fl_pop_matrix();
	*/
}


/*
 * Center indicator with "wing" design
 */
void
drawCenter()
{
/*
	fl_push_matrix();
	fl_color( 255, 255, 255 );

	// right half
	fl_begin_line();
		fl_vertex(  0.000,  0.000 );
		fl_vertex(  0.015, -0.020 );
		fl_vertex(  0.030,  0.000 );
		fl_vertex(  0.060,  0.000 );
	fl_end_line();

	// left half
	fl_begin_line();
		fl_vertex(  0.000,  0.000 );
		fl_vertex( -0.015, -0.020 );
		fl_vertex( -0.030,  0.000 );
		fl_vertex( -0.060,  0.000 );
	fl_end_line();

	fl_pop_matrix();
	*/
}


/*
 * Heading strip ticker
 */
void
drawDirection()
{
/*
	static char		headlabels[37][4] = {
		"S",
		"19", "20", "21", "22", "23", "24", "25", "26",
		"W",
		"28", "29", "30", "31", "32", "33", "34", "35",
		"N",
		"01", "02", "03", "04", "05", "06", "07", "08",
		"E",
		"10", "11", "12", "13", "14", "15", "16", "17",
		"S"
	};

	fl_push_matrix();
	fl_rotate( roll );
	fl_translate( yaw*C_DEG2RAD, 0.0 );

	//HORIZON AND YAW TICK LINE
	fl_color( 255, 255, 255 );
	fl_begin_line();
		fl_vertex( -(180.0+15)*C_DEG2RAD, 0.0 );
		fl_vertex(  (180.0+15)*C_DEG2RAD, 0.0 );
	fl_end_line();

	for( int n=0 ; n<37 ; ++n )
	{
		const double	hdg = C_DEG2RAD * (n * 10 - 180);

		fl_begin_line();
			fl_vertex( hdg, 0.015 );
			fl_vertex( hdg, 0.000 );
		fl_end_line();

		const char *txt = &headlabels[n][0];

		this->show_message(
			txt,
			(double)(n*10 - 180)*C_DEG2RAD-0.01f,
			0.02
		);
	}

	// Extra tick mark past S (going W) for overview
	fl_begin_line();
		fl_vertex( 190.0*C_DEG2RAD, 0.02 );
		fl_vertex( 190.0*C_DEG2RAD, 0.00 );
	fl_end_line();


	this->show_message(
		"19\0",
		190.0*C_DEG2RAD-0.015,
		0.02
	);

	this->show_message(
		"17\0",
		-190.0*C_DEG2RAD-0.015,
		0.02
	);


	// Extra tick mark past S (going E) for overview
	fl_begin_line();
		fl_vertex( -190.0*C_DEG2RAD, 0.02 );
		fl_vertex( -190.0*C_DEG2RAD, 0.00 );
	fl_end_line();

	if( this->draw_desired )
	{
		const double desired	= desired_yaw * C_DEG2RAD;
		const double zero	= zero_yaw * C_DEG2RAD;

		//"Zero" yaw marker
		fl_push_matrix();
		fl_color( 255, 0, 0 );
		fl_begin_polygon();
			fl_vertex( zero,          0.00 );
			fl_vertex( zero + 0.015, -0.02 );
			fl_vertex( zero - 0.015, -0.02 );
		fl_end_polygon();
		fl_pop_matrix();

		///Desired yaw marker
		fl_push_matrix();
		fl_color( 255, 255, 0 );
		fl_begin_polygon();
			fl_vertex( desired,          0.00 );
			fl_vertex( desired + 0.015, -0.02 );
			fl_vertex( desired - 0.015, -0.02 );
		fl_end_polygon();
		fl_pop_matrix();

	}

	fl_pop_matrix();
	*/
}

void artHorizonDraw( PtWidget_t *widget, PhTile_t *damage )  {
	PtSuperClassDraw( PtBasic, widget, damage );

	if (poses.size() == 0) {
		return;
	}

	struct GPSINSMsgRep & rep = poses.back();
	roll = rep.rpy[0];
	pitch = rep.rpy[1];
	yaw = rep.rpy[2];
		
	//get the raw canvas
	PhRect_t  raw_canvas;
	PtCalcCanvas (widget, &raw_canvas);
	//clip the sizes
	PtClipAdd(widget, &raw_canvas);	
	//translate relative to the canvas
	PgSetTranslation (&raw_canvas.ul, Pg_RELATIVE);

	artHorizonSizeX = raw_canvas.lr.x - raw_canvas.ul.x;
	artHorizonSizeY = raw_canvas.lr.y - raw_canvas.ul.y;

	//center in the middle of the grid
	PhPoint_t translation;
	translation.x = artHorizonSizeX/2, translation.y = artHorizonSizeY/2;
	PgSetTranslation(&translation, Pg_RELATIVE);	

	drawSky();
	drawCenter();
	drawDirection();
	drawRoll();
	drawPitch();	
	drawText();
	
	PhPoint_t translationBack;
	translationBack.x = -artHorizonSizeX/2, translationBack.y = -artHorizonSizeY/2;
	PgSetTranslation(&translationBack, Pg_RELATIVE);	

	
	//translate back
	raw_canvas.ul.x *= -1;
	raw_canvas.ul.y *= -1;
	PgSetTranslation (&raw_canvas.ul, Pg_RELATIVE);

	//remove clipping	
	PtClipRemove();


}

