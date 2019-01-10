////////////////////////////////////////////////////////////////////////////////
//
//    File: roadserver.cc
//
//    Description:
//		Road Server main class
//
//		Reads in a road map binary file downloaded from:
//			www.usgs.seamless.gov
//		into memory. 
//
//    See also:
//
//    Written By:
//       Tim Nicholson (timnich@ix.netcom.com)
//       Team Overbot
//       January, 2004
//
////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <math.h>
#include <unistd.h>
#include <string.h>

#include "roadserver.h"


// Conversion Constants
const double pi = 3.141592654;

// constructor
RoadServer::RoadServer()
{
    // initialize server messaging
    serverPort = new MsgServerPort(0.0);	// define message port, no timeout
    int stat = serverPort->ChannelCreate();	// create a channel, tell watchdog
    if  (stat ) {
	perror("RoadServer::RoadServer - ChannelCreate failed.\n");
	serverPort->Dump();
    }
    
    // initialize server state
    verbose  = false;
    
	size = 0;
	debug = false;
	correction_list = NULL;
	waypoint_list = NULL;
	road_list = NULL;
	corridor_road_list = NULL;
	count = 0;
	count_parts = 0;
}

void error (const char* p1,const char* p2)
{
	cout << p1 << p2 << endl;
	exit (1);
}

// destructor
RoadServer::~RoadServer()
{
}

void
RoadServer::messageThread()
{

    // loop collecting messages forever
    while (true) {

	// wait for message
	rcvid = serverPort->MsgReceive(msg);
	
	// handle errors
	if ( rcvid < 0 ) {
	    fflush(stdout);
	    perror("RoadServer::messageThread - MsgReceive failed\n");
	    sleep(1);		// avoid tight loop if repeated trouble FIX
	    continue;
	} else if ( rcvid == 0 ) {
	    perror("RoadServer::messageThread - received a pulse\n");

	    // pulses don't require a reply
	    continue;
	}
	
	if ( verbose ) {
	    //perror("RoadServer::messageThread - received a message\n");
	}
	
	// handle message
	RoadServer::handleMessage();
	
    }
}

// Handle messages
void
RoadServer::handleMessage()
{
    // Dispatch on message type
    switch ( msg.m_header.m_msgtype ) {

    case RoadServerMsg::GetRoadList::k_msgtype:
		handleGetRoadList();
		break;
	
    default:
		fprintf(stderr, "RoadServer::handleMessage - unknown message type: %x\n",
	    	msg.m_header.m_msgtype);
		MsgError(rcvid, EBADMSG);
		break;
    }    
}

// Handle GetRoadList message
void 
RoadServer::handleGetRoadList()
{
}

void
RoadServer::readRoadFile(char *fileDirectory)
{
	char* input_filename = "/mapfile.shp";
	char* temp = "";
	// FILE *input;

	temp = strcpy (temp, fileDirectory);
	temp = strcat (temp, input_filename);
		
	input = fopen (temp, "rb"); 
	if (!input) error ("Cannot open input file ",temp);

	RoadServer::process_header();
		
	while (!feof(input))
	{
		RoadServer::process_record();
	}
		
	fclose(input);
	
	cout << "Processed all records" << endl;
	cout << "Count " << count << endl;
	cout << "Size " << size << endl;
	cout << "Records with more than one part: " << count_parts << endl;
}

//
// Flip integer from big endian to little endian
//
void flip_int (char p[4])
{
	char p2[4];
	
	p2[0] = p[3];
	p2[1] = p[2];
	p2[2] = p[1];
	p2[3] = p[0];
	
	p[0] = p2[0];
	p[1] = p2[1];
	p[2] = p2[2];
	p[3] = p2[3];
}

//
// process the first record in the binary file
//
void RoadServer::process_header()
{
	struct file_header_t
	{
		int 		file_code;
		char		unused[20];
		int			file_length;
		int			version;
		int			shape_type;
		double		xmin;
		double		ymin;
		double		xmax;
		double		ymax;
		char		unused2[32];
	};

	struct file_header_overlay_t
	{
		char		file_code[4];
		char		unused[20];
		char		file_length[4];
	};

	union file_header_union_t
	{
		file_header_t			header;
		file_header_overlay_t	overlay;
	};
	
	file_header_union_t rec;
	
	// read file header into rec union & decode file length
	fread (&rec, sizeof(rec),1, input);
	size = size + sizeof(rec);
	flip_int (rec.overlay.file_code);
	flip_int (rec.overlay.file_length);

	// display bounding box of the map	
	cout << "Xmin " << rec.header.xmin << endl;
	cout << "Ymin " << rec.header.ymin << endl;
	cout << "Xmax " << rec.header.xmax << endl;
	cout << "Ymax " << rec.header.ymax << endl;
	cout << "File length " << rec.header.file_length * 2 << endl;
	
	map_lower_left.x = rec.header.xmin;
	map_lower_left.y = rec.header.ymin;
	map_upper_right.x = rec.header.xmax;
	map_upper_right.y = rec.header.ymax;
}

//
// process one polyline record
//
void RoadServer::process_record ()
{
	struct record_header_t
	{
		int			record_number;		// record number in big endian format
		int			content_length;		// number of words 16 bits - big endian
		int			shape_type;
		double		xmin;
		double		ymin;
		double		xmax;
		double		ymax;
		int			numparts;
		int			numpoints;
	};
	struct record_overlay_t
	{
		char		record_number[4];
		char		content_length[4];
	};
	union record_header_union_t
	{
		record_header_t		header;
		record_overlay_t	overlay;
	};
	
	record_header_union_t rec;
	
	// read file header into rec union & decode file length
	fread (&rec, sizeof(rec),1, input);
	
	if (!feof(input))
	{
		count = count + 1;
		// flip the integers from big endian to little endian format
		flip_int (rec.overlay.record_number);
		flip_int (rec.overlay.content_length);
		
		// allocate a structure to hold a record - only process polylines
		if (rec.header.shape_type == 3)
			RoadServer::allocate_road_record (rec.header.numparts,
			                                  rec.header.numpoints); 

		// add record size to current file size		
		size = size + 8 + rec.header.content_length * 2;

		// position the file pointer at the beginning of the next record	
		fseek (input, size, SEEK_SET);
		
		// look for shape types that are not polylines - the mapfile
		// should only have polylines in it
		if (rec.header.shape_type != 3)
		{
			cout << "Shape type " << rec.header.shape_type << endl;
			cout << "Size " << size << endl;
			cout << endl;
			cout << setprecision(10);
			cout << "Xmin " << rec.header.xmin << endl;
			cout << "Ymin " << rec.header.ymin << endl;
			cout << "Xmax " << rec.header.xmax << endl;
			cout << "Ymax " << rec.header.ymax << endl;
			// cout << "NumParts " << rec.header.numparts << endl;
			// cout << "NumPoints " << rec.header.numpoints << endl;
		}
	} 	
}

//
// allocate space in the road record for the parts and points arrays
//		
void RoadServer::allocate_road_record (int parts, int points)
{
	static road_record_t* current_road = NULL;
	road_record_t*	road = NULL;
	int i;

	// Skip the record if it has less then two points in it	
	if (points < 2)
	{
		cout << "Record " << count << " has one or less points - error" << endl;
		return;
	}
	
	try {
		road = new road_record_t;
	}
	catch (bad_alloc) {
		error ("Memory exhausted in allocate road_record","");
	}
	if (count == 1)
	{
		current_road = road;
		road_list = road;    // road_list is a member variable of RoadServer
	}
	else
	{
		current_road -> next = road;
		current_road = road;
		
	};
	
	if (parts > 1)
		count_parts++;
		
	road -> numparts = parts;
	road -> numpoints = points;
	road -> count = count;     // store the record number
	try {
		road -> parts = new int [parts];
		road -> points = new point_t [points];
	}
	catch (bad_alloc)
	{ error ("Memory exhausted in allocate_road_record",""); }
	road -> next = NULL;
	
	fread (road -> parts, sizeof(int),parts, input);
	fread (road -> points, sizeof(point_t),points, input);
	
	// Apply map correction to road points
	for (i=0; i<road->numpoints; i++)
		get_correction (&(road->points[i]));	
}

//
// Return the cartesian distance between two points in meters.  Note that
// distance in degrees latitude is scaled different than degrees longitude
//
double RoadServer::distance (point_t* point1, point_t* point2)
{
	double d1;
	double d2;
	double mean_latitude;
	
	mean_latitude = (point1->y + point2->y)/2.0;
	d1 = (point1->x -point2->x); // * longitude_meters (mean_latitude);
	d2 = (point1->y -point2->y); // * latitude_meters (mean_latitude);
	return sqrt(d1*d1 + d2*d2);
}

void RoadServer::read_correction_file(char* fileDirectory)
{
	FILE *input;
	char* input_filename = "/correction.txt";
	char* temp = "";
	double x;
	double y;
	double delta_x;
	double delta_y;
	correction_record_t* correction;
	
	temp = strcpy (temp, fileDirectory);
	temp = strcat (temp, input_filename);
	
	input = fopen (temp, "r"); 
	if (!input) error ("Cannot open correction file ",temp);

	while (!feof(input))
	{
		fscanf (input, "%le %le %le %le", &x, &y, &delta_x, &delta_y);

		correction = new (correction_record_t);
		correction->next = correction_list;
		correction->point.x = x;
		correction->point.y = y;
		correction->delta_x = delta_x;
		correction->delta_y = delta_y;
		// put correction record at beginning of list
		correction_list = correction;
		if (debug)
			cout << "Correction " << x << "  " << y << endl;
	}
}

//
// Get the map correction for this point
//
void RoadServer::get_correction (point_t* point)
{
	correction_record_t* correction;
	correction_record_t* this_correction = NULL;
	double d;
	double min_d = 9999999.0;
	
	correction = correction_list;
	
	while (correction != NULL)
	{
		d = distance (point, &(correction->point));
		if (d < min_d)
		{	min_d = d;
			this_correction = correction;
		}
		correction = correction->next;
	}
	
	if (this_correction != NULL)
	{
		point->x = point->x + this_correction->delta_x;
		point->y = point->y + this_correction->delta_y;
	}
}

void RoadServer::get_perpendicular_point (point_t p1, point_t p2, point_t p3,
                              point_t* point)
{
	// Given a line passing through points p1 and p2
	//               Y = AX + B1, compute the slope A and y crossing B1
	// Determine a second line perpendicular to it passing through p3.
	//               Y = -(1/A)X + B2
	// Figure the location of p4 lying at the intersection of line1 and line2.
	 
	double a;  // the slope of the line
	double b1;
	double b2;
	const double underflow = 1.0e-5;
	
	// if the two points are coincident set point to the first one
	if (p1.x==p2.x && p1.y==p2.y)
	{
		point -> x = p1.x;
		point -> y = p1.y;
		return;
	}
	
	// see if the slope is perpendicular, if so we can determine 
	// point directly
	if (fabs(p2.x - p1.x) < underflow)
	{
		point->x = p2.x;
		point->y = p3.y;
	}
	// see if the slope is horizontal
	else if (fabs(p2.y - p1.y) < underflow)
	{
		point->x = p3.x;
		point->y = p2.y;
	}
	else
	{
		a = (p2.y - p1.y) / (p2.x - p1.x);
		b1 = p1.y - a * p1.x;
	
		b2 = p3.y + p3.x / a;
	
		point->x = (b2 - b1)/(a + 1/a);
		point->y = a * point->x + b1;
	}
	
}

//
// Return true if p3 is between p1 and p2
//
bool RoadServer::point_between (point_t p1, point_t p2, point_t p3)
{
	// see if p3 is between p2 and p1. Note that we
	if  ( ( (p3.x <= p2.x && p3.x >= p1.x) ||
	        (p3.x >= p2.x && p3.x <= p1.x)) &&
	        
	      ( (p3.y <= p2.y && p3.y >= p1.y) ||
		    (p3.y >= p2.y && p3.y <= p1.y)))
		return true;
	else return false;
}


void RoadServer::lookup_road_name (int recno,char* name)
{
	const int offset = 448;
	
	struct dbfrec_t
	{
		char	unused[113];
		char	street[30];
		char    unused2[8];
	};
	
	dbfrec_t rec;
		
	// position the file pointer at the beginning of the next record	
	fseek (inputdbf, offset + sizeof(dbfrec_t)*(recno-1), SEEK_SET);
		
	fread (&rec, sizeof(rec),1, inputdbf);
	
	cout << "Street name: <" << rec.street << ">" << endl;
	// cout << "Unused <" << rec.unused << ">" << endl;	
}

//
// Compute angle determined by two points.  Point P2 is the end of the vector.
// Return a number between -180 and 180 where a vector along the x-axis is a
// zero degrees.
//
double RoadServer::angle (point_t p1, point_t p2)
{
	double temp = 0;
	errno = 0;
	temp = atan2 (p2.y - p1.y, p2.x - p1.x);
	if (errno != 0)
	{
		cout << "Domain error in angle" << endl;
		return -1;
	}
	// convert from radians (-pi,pi) to degrees (-180,180)
	temp = temp * 180 / pi;
	return temp;
}

//
// Compute the heading difference between two angles)
// Note the boundry conditions where heading is +-180
//
double RoadServer::heading_difference (double h1, double h2)
{
	double h3;
	
	h3 = h2 - h1;
	if (h3 < -180) h3 = h3 + 360;
	else if (h3 > 180) h3 = h3 - 360;
	return h3;
}

//
// Read waypoint file and put waypoint records into a linked list
//
void RoadServer::get_waypoints ()
{
	double x=0;
	double y=0;
	double width=0;
	double speed=0;
	int count = 0;
	waypoint_record_t* waypoint;
	waypoint_record_t* last_waypoint = NULL;
	
	while (true)
	{
		count ++;
		if (debug)
		    cout << "waypoint " << count << " x = " << x << " y = " << y 
		         << " width = " << width << endl;

		waypoint = new (waypoint_record_t);
		waypoint->next = NULL;
		waypoint->count = count;
		waypoint->point1.x = x;
		waypoint->point1.y = y;
		waypoint->point2.x = 0;
		waypoint->point2.y = 0;
		waypoint->actual_width = width;
		
		// set the width to a minimum of 10 meters so that we find roads
		// in a narrow corridor
		if (width < 10)
		{
			width = 10;
			cout << "Waypoint " << count << " set to 30 feet - narrow corridor" << endl;
		} 
		
		waypoint->width = width;
		waypoint->speed = speed;
		waypoint->road_list = NULL;
		waypoint->beginning_roads = NULL;
		waypoint->ending_roads = NULL;
		
		if (waypoint_list == NULL) 
		{
			waypoint_list = waypoint;
			last_waypoint = waypoint;
		}
		else
		{
			// duplicate point info in previous waypoint record
			last_waypoint->point2.x = x;
			last_waypoint->point2.y = y;
			last_waypoint->next = waypoint;
			last_waypoint = waypoint;
		}
	}
	
	cout << "Waypoints read " << count << endl;
	
	fclose (input);
	
	// Now compute the rectangular coordinates for each waypoint corridor
	waypoint = waypoint_list;
	
	while (waypoint != NULL)
	{
		waypoint_rectangle (waypoint);
		waypoint = waypoint->next;
	}
}

//
// Return true if point is in the waypoint segment
//
bool RoadServer::point_in_waypoint (waypoint_record_t* waypoint, point_t point)
{
	double d;
	point_t point1;
	
	// See if the point is close to point1 or point2 - within corridor width	
	d = distance (&(waypoint->point1),&point);
	if (d < waypoint->width) 
	{
		// cout << "Found waypoint in circle1 " << waypoint->count << endl ;
		return true;
	}
	
	// Handle the case for the last waypoint which is just a circle
	if (waypoint->point2.x == 0) 
	{
		return false;
	}
	
	d = distance (&(waypoint->point2),&point);
	if (d < waypoint->width) 
	{
		// cout << "Found waypoint in circle2 " << waypoint->count << endl;
		return true;
	}
	
	// See if point is in rectangle defined by waypoint->p1 thru p4
	get_perpendicular_point (waypoint->p1,waypoint->p2,point,&point1);
	
	if (!point_between (waypoint->p1,waypoint->p2,point1))
	{
		return false;
	}
	
	get_perpendicular_point (waypoint->p1,waypoint->p3,point,&point1);
	if (!point_between (waypoint->p1,waypoint->p3,point1)) 
	{
		return false;
	}
	
	// cout << "Found waypoint in rectangle " << waypoint->count << endl;
		
	return true;
}

//
// Return true if the point is in the waypoint corridor
//
bool RoadServer::point_in_corridor (point_t point, road_record_t* road)
{
	waypoint_record_t* waypoint;
	bool in_corridor = false;
	
	waypoint = waypoint_list;
	
	while (waypoint != NULL)
	{
		if (point_in_waypoint(waypoint, point))
		{
			in_corridor = true;
			// lookup_road_name (road->count,street);
			break;
		}
		waypoint = waypoint->next;
	}		
	return in_corridor;
}

//
// Find all the roads in the waypoint corridor.
//
void RoadServer::waypoint_find_roads ()
{
	road_record_t*		road;
	corridor_road_t*	corridor_road;
	int i;
	bool in_corridor;
	char street[35];
	
	// go to the head of the road list	
	road = road_list;
		
	while (road != NULL)
	{
		in_corridor = true;
		for (i = 0; i < road->numpoints; i++)
		{
			if (!point_in_corridor(road->points[i],road))
			{
				in_corridor = false;
				break;
			}
		}
		if (in_corridor)
		{
			// add road to head of corridor_road_list;
			corridor_road = new (corridor_road_t);
			
			corridor_road->next = corridor_road_list;
			corridor_road->road = road;
			corridor_road_list = corridor_road;
			
			if (debug)
			{
				cout << "Found road in corridor ";
				lookup_road_name (road->count,street);
			}
		}
		road = road -> next;
	}
}

//
// Assign each road in corridor_road_list to one or more waypoint segments
// if it is contained partially or completely within the segment
//
void RoadServer::waypoint_assign_roads ()
{
	waypoint_record_t*	waypoint;
	waypoint_road_t*	waypoint_road;
	road_record_t*		road;
	corridor_road_t*	corridor_road;
	bool 				in_waypoint;
	int i;
	char street[35];
	
	corridor_road = corridor_road_list;
	
	while (corridor_road != NULL)
	{
		road = corridor_road->road;
		
		// see which waypoint segments the road is within
		waypoint = waypoint_list;
		
		while (waypoint != NULL)
		{
			// cout << "Searching waypoint " << waypoint->count << endl;
			
			in_waypoint = false;
			for (i = 0; i < road->numpoints; i++)
			{
				if (point_in_waypoint(waypoint, road->points[i]))
				{
					in_waypoint = true;
					break;
				}
			}
			if (in_waypoint)
			{
				// add the road to the waypoint's road list
				waypoint_road = new (waypoint_road_t);
				waypoint_road-> road = road;
				waypoint_road-> parent = NULL;
				waypoint_road-> child = NULL;
				// add waypoint_road to head of list
				waypoint_road->next = waypoint->road_list;
				waypoint->road_list = waypoint_road;
				
				if (debug)
				{
					cout << "Waypoint " << waypoint->count << "  ";
					lookup_road_name (road->count, street);
				}
			}
			
			waypoint = waypoint->next;
		}
		corridor_road = corridor_road->next;
	}
}

//
// Compute the rectangle (p1,p2,p3,p4) for each waypoint such that given
// points point1 and point2 and a width, make the line defined by (p1,p2)
// and the line (p3,p4) parallel to the line defined by (point1,point2).
//
void RoadServer::waypoint_rectangle (waypoint_record_t* way)
{	 
	double waypoint_angle = 0;
	double waypoint_sine;
	double waypoint_cosine;
	const double underflow = 1.0e-6;
	
	// if the two waypoints are coincident set point p1 to zero
	if (way->point1.x==way->point2.x && way->point1.y==way->point2.y)
	{
		way->p1.x = 0;
		way->p1.y = 0;
		return;
	}
	
	// If the second waypoint is (0,0) we are at the last waypoint
	// which degenerates to a circle
	if (way->point2.x==0)
	{
		way->p1.x = 0;
		way->p1.y = 0;
		
		way->p2.x = 0;
		way->p2.y = 0;
		
		way->p3.x = 0;
		way->p3.y = 0;
		
		way->p4.x = 0;
		way->p4.y = 0;
		return;
	}
		
	
	// see if the slope is perpendicular, if so we can determine 
	// point directly
	if (fabs(way->point2.x - way->point1.x) < underflow)
	{
		way->p1.x = way->point1.x - way->width;
		way->p1.y = way->point1.y;
		
		way->p2.x = way->point1.x - way->width;
		way->p2.y = way->point2.y;
		
		way->p3.x = way->point1.x + way->width;
		way->p3.y = way->point1.y;
		
		way->p4.x = way->point1.x + way->width;
		way->p4.y = way->point2.y;
	}
	// see if the slope is horizontal
	else if (fabs(way->point2.y - way->point1.y) < underflow)
	{
		way->p1.x = way->point1.x;
		way->p1.y = way->point1.y + way->width;
		
		way->p2.x = way->point2.x;
		way->p2.y = way->point2.y + way->width;
		
		way->p3.x = way->point1.x;
		way->p3.y = way->point1.y - way->width;
		
		way->p4.x = way->point2.x;
		way->p4.y = way->point2.y - way->width;
	}
	else
	{
		// Compute the angle of the line (point1,point2)
		waypoint_angle = angle (way->point1,way->point2);
		 
		// Compute the angle that is perpendicular
		// Make sure that the resulting angle is in -pi +pi
		waypoint_angle = waypoint_angle + 90;
		if (waypoint_angle > 180) waypoint_angle = waypoint_angle - 360;
		waypoint_angle = waypoint_angle / 180 * pi;

		// Compute the x,y offsets in meters		
		waypoint_sine = sin (waypoint_angle) * way->width;
		waypoint_cosine = cos (waypoint_angle) * way->width;
		
		way->p1.x = way->point1.x + waypoint_cosine;
		way->p1.y = way->point1.y + waypoint_sine;
		
		way->p2.x = way->point1.x - waypoint_cosine;
		way->p2.y = way->point1.y - waypoint_sine;
		
		way->p3.x = way->point2.x + waypoint_cosine;
		way->p3.y = way->point2.y + waypoint_sine;
		
		way->p4.x = way->point2.x - waypoint_cosine;
		way->p4.y = way->point2.y - waypoint_sine;
	}
	if (debug)
	{
		cout << "Waypoint angle " << waypoint_angle << endl;
		cout << "Waypoint p1 " << way->p1.x << "  " << way->p1.y << endl;
		cout << "Waypoint p2 " << way->p2.x << "  " << way->p2.y << endl;
		cout << "Waypoint p3 " << way->p3.x << "  " << way->p3.y << endl;
		cout << "Waypoint p4 " << way->p4.x << "  " << way->p4.y << endl;
	}
}