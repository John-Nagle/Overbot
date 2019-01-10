///////////////////////////////////////////////////////////////////////////////////
//
//		File:	check_mapfile.c
//
//		Usage:
//
//		Description
//			Reads in a road map binary file downloaded from:
//				www.usgs.seamless.gov
//			into memory.  Builds a 2-dimensional index into the linked list of
//			road vector records.  Given a vehicle position the program figures
//			what cell(s) to search in the index to find the closest road.
//
//		Written by:
//			Tim Nicholson (timnich@ix.netcom.com)
//			Team Overbot
//			October 2003
//
///////////////////////////////////////////////////////////////////////////////////
			
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <math.h>

using namespace std;

// Conversion Constants
const double meters_per_foot  = 0.3048;
const double pi = 3.141592654;

// Constants that determine the length of a degree in meters for both
// latitude and longitude at 36 degress and 35 degrees latitude.  See website
// http://pollux.nss.nima.mil 
// goto Marine Navigation Calculator, goto Distance Calculator,
// goto Length of a Degree of Latitude and Longitude
const double latitude_35 = 110941.0;
const double latitude_36 = 110959.0;
const double longitude_35 = 91288.0;
const double longitude_36 = 90164.0;

// structures for road record type
//
// define structure to hold an x-y coordinate
//
struct point_t
{
	double	x;
	double	y;
};

//
// structure to hold a linked list of road database records
// 
struct road_record_t
{
	int				numparts;	// number of line segments in this record
	int				numpoints;  // number of points for all line segments
	int				count;		// record count to look up street name
	int*			parts;		// index into points array where segment starts
	point_t*		points;
	road_record_t*	next;
};

//
// lookup table record
// each cell in the lookup table holds a linked list of lookup records
// pointing to a road that has a point in the cell
//
struct lookup_record_t
{
	lookup_record_t*	next;
	road_record_t*		road;
};

//
// enumerate the different types of intersections
//
enum intersection_t {
	UNKNOWN,
	THROUGH_WAY,	// No intersection, road continues in same direction
	FOUR_WAY,
	TEE,			// Road dead ends into intersecting road
	LEFT_TURN,
	LEFT_TURN_ONLY,
	RIGHT_TURN,
	RIGHT_TURN_ONLY };

//
//  Structure to hold intersection information
//	
struct intersection_rec_t
{
	road_record_t*	road;
	int				index;		// index into line segment on this road
	point_t			point;		// point where road intersects
	intersection_t	type;		// type of intersection e.g. LEFT_TURN
	double			angle;		// angle of the intersection -180 to 180
	double			distance;	// distance from vehicle to intersection
	intersection_rec_t* next;
};

//
// Structure to hold roads and intersection info for a waypoint.  Each road
// has zero or more children that represent roads that intersect with it.
// Sibling roads are those that share an intersection. At_End is true if the
// intersection(s) are at the end of the road and false if they are at the
// beginning.
//
struct waypoint_road_t
{
	road_record_t*		road;
	bool				at_end;
	waypoint_road_t*	next;   // points to siblings of this road
	waypoint_road_t*	child;
	waypoint_road_t*	parent;
};		
	
//
// Structure to hold waypoints.  Road_List points to list of roads that are
// in this segment.  Beginning_Roads points to list of roads that are both in
// the previous segment and this segment.  Ending_Roads points to a list
// of roads that are in this segment and the next segment.
//
struct waypoint_record_t
{
	point_t				point1;
	point_t				point2;
	point_t				p1;
	point_t				p2;
	point_t				p3;
	point_t				p4;
	int					count;			// waypoint number for reference
	double				width;			// width of corridor in feet
	double				actual_width;	// actual value if less than 30 feet		
	double				speed;
	waypoint_record_t*	next;
	waypoint_road_t*	road_list;
	waypoint_road_t*	beginning_roads;
	waypoint_road_t*	ending_roads;
};

//
// Structure for roads that are completely within the waypoint corridor
//
struct corridor_road_t
{
	corridor_road_t*	next;
	road_record_t*		road;
};

//
// Structure to hold map correction information.  Delta_x is the correction
// in degrees longitude that needs to be added to the map coordinate to get
// the true GPS coordinate.
//
struct correction_record_t
{
	point_t		point;
	double		delta_x;
	double		delta_y;
	correction_record_t* next;
};

// lookup table holds an array of linked list of pointers to roads
// in each cell.  A road may appear in multiple cells
lookup_record_t* lookup_table = NULL;

// number of cells in lookup table
const int cnt = 5; // cells per degree

int x_cells = 0;
int y_cells = 0;

// some useful statistics
int lookup_probes = 0;
int lookup_additions = 0;
int count_parts = 0;

// debug flag
bool debug = false;

// remember vehicle position after input
point_t vehicle_position;

// remember vehicle heading - angle of 0 degrees is due east
// value ranges between -180 and 180
double vehicle_heading;

// Heading from vehicle to end of current waypoint
double waypoint_heading;

waypoint_record_t* current_waypoint = NULL;

// closest road to vehicle
road_record_t* closest_road = NULL;
// index into closest_road -> points[i]
int closest_road_index = 0;

// Closest point on that road - this point may or may not be
// coincident with one of the points in the points array for this road
point_t closest_point;

// Direction to travel along the road to match the vehicle heading.
// Indicates how the index into the road points array must be incremented
// either up (1) or down (-1).
int closest_direction = 0;

// Closest intersections to the vehicle in the direction of travel
intersection_rec_t *intersection_list = NULL;

// List of waypoint records
waypoint_record_t* waypoint_list = NULL;

// List of coordinate correction records
correction_record_t* correction_list = NULL;

// store the boundries of the map
point_t map_lower_left;
point_t map_upper_right;

road_record_t* current_road = NULL;
road_record_t* road_list = NULL;

// List of roads that are completely within the waypoint corridor
corridor_road_t* corridor_road_list = NULL;

int count = 0;
int size = 0;
		
FILE *input;
FILE *inputdbf;
	
void error (const char* p1,const char* p2);
void flip_int (char p[4]);
void process_header();
void process_record();
void allocate_road_record (int parts, int points);
void dump_road_record (road_record_t* road);
void find_closest_road();
void get_position (point_t* pos);
double distance (point_t* point1, point_t* point2);
void get_vehicle_position ();
void build_lookup_table();
void lookup_table_add_point (road_record_t* road, point_t* point);
void get_xy_coordinates (point_t vpos, int& x, int& y,
                         int& x_offset, int& y_offset);
void lookup_table_find_closest_road();
void lookup_cell (int x, int y, point_t vehicle_position, 
                  road_record_t** road, 
                  double* road_distance, 
                  point_t* point,
                  int* index);
void get_fourth_point (point_t p1, point_t p2, point_t vpos,
				   point_t* point);
				   
void lookup_road_name (int recno,char* name);
double heading_difference (double h1, double h2);
void determine_road_direction();

void lookup_table_find_intersection (road_record_t* road,
									 point_t point,
                                     int index,
                                     int direction);
                                     
void add_intersection (intersection_rec_t intersection);
void free_intersection_list ();
void lookup_cell_intersection (int x, int y,
							   road_record_t* road,
							   point_t point,
							   int index,
							   int direction);
							   
void test_find_intersection ();
void find_intersection (point_t p1, point_t p2,
					    point_t p3, point_t p4, point_t* point);
					    
bool close_to (point_t point1, point_t point2);
bool at_end (road_record_t* road, int index);

void determine_intersection_type (point_t p1, point_t p2,
					    		  point_t p3, point_t p4, 
					    		  point_t point,
					    		  road_record_t* road1,
					    		  road_record_t* road2,
					    		  int index1,
					    		  int index2,
					    		  int direction,
					    		  intersection_rec_t* intersection);
					    		  
void read_waypoint_file ();
void waypoint_find_roads ();
void waypoint_assign_roads ();
void waypoint_rectangle (waypoint_record_t* way);
void test_get_perpendicular_point ();
void determine_waypoint_heading ();
void read_correction_file();	
void get_correction (point_t* point);
				    		                                        							   
int main(int argc, char *argv[]) {
	
	// see if we are to run in debug mode - any arg triggers debug mode
	if (argc > 1) 
	{
		debug = true;
		cout << "Debug mode" << endl;
	}
	
	cout << "Check_mapfile " << argc << endl;
		
	char* input_filename = "/home/timnich/mapfile.shp";
	char street[35];
	
	input = fopen (input_filename, "rb"); 
	if (!input) error ("Cannot open input file ",input_filename);

	// access DBF file (dbase II format) that contains road names
	input_filename = "/home/timnich/mapfile.dbf";
	
	inputdbf = fopen (input_filename, "rb"); 
	if (!input) error ("Cannot open input dbf file ",input_filename);

	cout << endl;
	cout << setprecision(12);
	
	read_correction_file();	
	process_header();
		
	while (!feof(input))
	{
		process_record();
	}
		
	fclose(input);
	
	cout << "Processed all records" << endl;
	cout << "Count " << count << endl;
	cout << "Size " << size << endl;
	cout << "Records with more than one part: " << count_parts << endl;

	if (debug)
	{	
		lookup_road_name (1,street);
		lookup_road_name (count-1,street);
		lookup_road_name (count,street);
	}

	build_lookup_table ();
	
	// test_get_perpendicular_point ();

	read_waypoint_file();
	waypoint_find_roads();  // find all roads in waypoint corridor
	waypoint_assign_roads ();  // assign them to waypoint segments
	
	// Test program for find_intersection
	if (debug) test_find_intersection();

	do 
	{
		cout << endl;
		get_vehicle_position();
		
		// Figure out which waypoint we are at and what the heading should be
		determine_waypoint_heading ();
		
		// find closest road in waypoint corridor
		find_closest_road ();
	
		// lookup_table_find_closest_road();
		
		// determine which direction to travel on the closest road
		determine_road_direction();

		free_intersection_list();
		lookup_table_find_intersection (closest_road,
									 closest_point,
                                     closest_road_index,
                                     closest_direction);
	}
	while (vehicle_position.y > 0);

	return EXIT_SUCCESS;	
	
}

void error (const char* p1,const char* p2)
{
	cout << p1 << p2 << endl;
	exit (1);
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
void process_header()
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
void process_record ()
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
			allocate_road_record (rec.header.numparts, rec.header.numpoints); 

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
void allocate_road_record (int parts, int points)
{
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
		road_list = road;
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

void dump_road_record (road_record_t* road)
{
	char street[35];
	
	lookup_road_name (road->count,street);
	// cout << "NumParts " << road -> numparts << endl;
	cout << "NumPoints " << road -> numpoints << endl;
	
	//for (int i = 0; i < road -> numparts; i++)
	//{
	//	cout << road -> parts[i] << " ";
	//}
	//cout << endl;

	
	for (int i = 0; i < road -> numpoints; i++)
	{
		cout << road -> points[i].x << "  ";
		cout << road -> points[i].y << endl;
		if (i>10) break;
	}	
}

//
// Use linear interpolation to determine length of a degree of latitude
//
double latitude_meters (double latitude)
{
	double meters;
	
	meters = (latitude - 35.0)*(latitude_36 - latitude_35) + latitude_35;
	
	return meters;
}

double longitude_meters (double latitude)
{
	double meters;
	
	meters = (latitude - 35.0)*(longitude_36 - longitude_35) + longitude_35;
	
	return meters;
}

//
// Return the cartesian distance between two points in meters.  Note that
// distance in degrees latitude is scaled different than degrees longitude
//
double distance (point_t* point1, point_t* point2)
{
	double d1;
	double d2;
	double mean_latitude;
	
	mean_latitude = (point1->y + point2->y)/2.0;
	d1 = (point1->x -point2->x) * longitude_meters (mean_latitude);
	d2 = (point1->y -point2->y) * latitude_meters (mean_latitude);
	return sqrt(d1*d1 + d2*d2);
}

//
// find the closest road by traversing the list of roads in the
// waypoint corridor
//
void find_closest_road()
{
	point_t temp_point;
	road_record_t* road;
	corridor_road_t* corridor_road;
	char street[35];
	double d;
	double min_d = 9999999.0;
	int i;

	// go to the head of the road list	
	corridor_road = corridor_road_list;
		
	while (corridor_road != NULL)
	{
		road = corridor_road->road;
		for (i = 0; i < road->numpoints-1; i++)
		{
			get_fourth_point (road->points[i],
			                  road->points[i+1],
			                  vehicle_position,
			                  &temp_point);
			                  
			d = distance (&vehicle_position, &temp_point);
			if (d < min_d)
			{
				min_d = d;
				closest_point = temp_point;
				closest_road = road;
				closest_road_index = i;
			}
		}
		corridor_road = corridor_road -> next;
	}
	
	cout << "Found closest road  " << min_d << endl;
	cout << closest_point.x << " " << closest_point.y << endl;
	if (closest_road != NULL)
	{
		if (debug)
		{ 
			// dump_road_record (closest_road);
			lookup_road_name (closest_road->count,street);
		}
	} 
}

//
// get current vehicle position from navigation server
//
void get_vehicle_position ()
{
	double d;
	cout << "Input X Position in degrees: ";
	cin >> d;
	vehicle_position.x = d;
	cout << "Input Y Position in degrees: ";
	cin >> d;
	vehicle_position.y = d;
	cout << "Input vehicle heading in degrees: ";
	cin >> vehicle_heading;
}

//
// Build a lookup table by dividing the map into cells.  Look at each road
// in the map and for each point on the road figure what cell that point is in.
// If the road is not in the cell then add it.
// 	
void build_lookup_table()
{
	int i_cells;
	int i;
	road_record_t* road;
	
	// compute the number of cells in the map
	// along the x and y coordinates. CNT cells per degree
	x_cells = int (ceil(map_upper_right.x*cnt) - floor(map_lower_left.x*cnt));
	y_cells = int (ceil(map_upper_right.y*cnt) - floor(map_lower_left.y*cnt));
	cout << "x_cells " << x_cells << endl;
	cout << "y_cells " << y_cells << endl;
	
	i_cells = x_cells * y_cells;
	
	// cannot dynamically allocate a 2-dimensional array
	// index into array by the following  i = x * x_cells + y
	try { lookup_table = new lookup_record_t [i_cells]; }
	catch (bad_alloc) { error ("Memory exhausted in build_lookup_table",""); }
	
	// initialize the list in each lookup cell to NULL
	for (i=0; i<i_cells; i++)
	{
		lookup_table[i].next = NULL;
		lookup_table[i].road = NULL;
	}
	
	// Add each road to the lookup table
	road = road_list;
	
	while (road != NULL)
	{
		for (i=0; i < road->numpoints; i++)
		{
			lookup_table_add_point (road, &(road->points[i]));
		};
		road = road -> next;
	}
	
	cout << "Lookup table probes: " << lookup_probes << endl;
	cout << "Lookup table additions: " << lookup_additions << endl;
}

//
// See if the cell in which this point lies already has this road in its 
// linked list of roads.  If not then add the road.
//
void lookup_table_add_point (road_record_t* road, point_t* point)
{
	int x;
	int y;
	int i;
	lookup_record_t* 	look; 
	lookup_record_t*	lastlook;
	bool found_road = false;
	
	// Figure out what cell in the lookup table this point lies.
	// See if the road is already in the cells' list of roads.
	// If not then add the road.
	
	x = int (ceil(point->x*cnt) - floor(map_lower_left.x*cnt))-1;
	y = int (ceil(point->y*cnt) - floor(map_lower_left.y*cnt))-1;
	
	// cout << x << " (x,y) " << y << endl;
	
	i = y * x_cells + x;  // index into lookup table
	
	
	if (i < 0 | i > (x_cells * y_cells - 1))
	{
		return; 
		// error ("Index error in lookup_table_add_point","");
	}
	
	look = &lookup_table[i];
	lastlook = look;
	lookup_probes++;
	
	while (look != NULL)
	{
		if (look->road == road)
		{
			// road is already in the list
			found_road = true;
			break;
		}
		lastlook = look;  // if we are at the end of the list save it
		look = look -> next;
	}
	if (found_road) return;
	
	lookup_additions ++;
	
	// special case for first record in the list
	// already allocated so fill in road info and return
	if (lastlook->road == NULL)
	{
		lastlook->road = road;
		return;
	}
	
	// else we need to allocate another lookup record
	// and fill it in
	try { lastlook -> next = new lookup_record_t; }
	catch (bad_alloc) { error ("Memory exhausted in lookup_table_add_point",""); }

	look = lastlook -> next;
	look -> road = road;
	look -> next = NULL;
}

//
// Figure out what cell to look in given a vehicle position (vpos).  Also
// figure the adjacent cells to search given by x_offset and y_offset.
//
void get_xy_coordinates (point_t vpos, int& x, int& y,
                         int& x_offset, int& y_offset)
{
	double remain;
	const double underflow = 1.0e-12;
	
	x = int (ceil(vpos.x*cnt) - floor(map_lower_left.x*cnt))-1;
	y = int (ceil(vpos.y*cnt) - floor(map_lower_left.y*cnt))-1;
	
	remain = vehicle_position.x * cnt - floor(vehicle_position.x * cnt);
	
	if (debug) cout << "Remainder x " << remain << endl;
	
	// Figure out which quadrant of the cell that the point lies within.
	// Note that when we are on a cell boundry we have to make an exception
	// for the remainder which is very close to zero and adjust the offset
	// to 1.  Otherwise we end up looking in the wrong cells for the nearest
	// road.
	if ((remain > 0.5) | (remain < underflow))
		x_offset = 1;
	else
		x_offset = -1;
		
	remain = vehicle_position.y * cnt - floor(vehicle_position.y * cnt);
	
	if (debug) cout << "Remainder y " << remain << endl;
	
	if ((remain > 0.5) | (remain < underflow))
		y_offset = 1;
	else
		y_offset = -1;

	if (debug)
	{		
		cout << "X offset " << x_offset << "  Y offset " << y_offset << endl;
		cout << "X        " << x        << "  Y        " << y << endl;
	}	
}

void lookup_table_find_closest_road()
{
	// point_t vehicle_position;
	point_t point;
	road_record_t* road = NULL;
	double distance;
	double closest_distance;
	road_record_t* closest_road = NULL;
	int closest_road_index;
	point_t closest_point;
	// char street[35];
	int x;
	int y;
	int x_offset;
	int y_offset;
	int ix;
		
	get_xy_coordinates (vehicle_position,x,y,x_offset,y_offset);
	
	// lookup the roads in the containing cell plus the three
	// adjacent cells that are closest to the vehicle position
	lookup_cell (x,y,vehicle_position,
	             &road,&distance,&point,&ix);
	closest_distance = distance;
	closest_point = point;
	closest_road = road;
	closest_road_index = ix;
	
	lookup_cell (x+x_offset,y,vehicle_position,
	             &road,&distance,&point,&ix);
	if (distance < closest_distance)
	{
		closest_distance = distance;
		closest_point = point;
		closest_road = road;
		closest_road_index = ix;
	}
	
	lookup_cell (x,y+y_offset,vehicle_position,
	             &road,&distance,&point,&ix);
	if (distance < closest_distance)
	{
		closest_distance = distance;
		closest_point = point;
		closest_road = road;
		closest_road_index = ix;
	}
	
	lookup_cell (x+x_offset,y+y_offset,vehicle_position,
	             &road,&distance,&point,&ix);
	if (distance < closest_distance)
	{
		closest_distance = distance;
		closest_point = point;
		closest_road = road;
		closest_road_index = ix;
	}
		
	cout << "Lookup table found closest road  " << closest_distance << endl;
	if (debug)
		cout << "Index into road points[] " << closest_road_index << endl;
	cout << "Closest Point " 
	     << closest_point.x << " " << closest_point.y << endl;
	if (closest_road == NULL) 
		cout << "No road was found " << endl;
	else
	{ 
		// cout << "Number of points in road " << closest_road -> numpoints << endl;
		// lookup_road_name (road->count,street);
		if (debug) dump_road_record (closest_road);
	};
	   
}

void lookup_cell (int x, int y, point_t vehicle_position,  // input values
                  road_record_t** road,                     // output values
                  double* road_distance, 
                  point_t* point,
                  int* index)
{
	road_record_t* temp_road;
	lookup_record_t* look;
	point_t temp_point;
	double d;
	double min_d = 9999999.0;
	int i;
	// int count = 0;

	// initialize return values for no road found	
	*road = NULL;
	*road_distance = min_d;
	point->x = 0;
	point->y = 0;
	*index = -1;
	// cout << x << " (x,y) " << y << endl;
	
	i = y * x_cells + x;  // index into lookup table
	
	if (i < 0 | i > x_cells * y_cells - 1)
	{
		cout << "Lookup Cell Outside map" << endl;
		return;
	}
	
	// cout << "lookup i: " << i << endl;
	
	look = &lookup_table[i];

	// return no data if there are no roads in this cell	
	if (look->road == NULL)
	{
		cout << "No roads in this cell" << endl;
		return;
	}
	
	while (look != NULL)
	{
		temp_road = look -> road;
		// count++;
		// cout << "Dumping road record " << count << endl;

		// the following should only occur if the table was build improperly		
		if (temp_road == NULL)
			error ("Empty cell in lookup table","");
			
		if (temp_road->numpoints < 2)
		{
			cout << "Road with just one point in it - map error" << endl;
			continue;  // ignore this road
		}
		
		for (i = 0; i < temp_road->numpoints-1; i++)
		{
			get_fourth_point (temp_road->points[i],
			                  temp_road->points[i+1],
			                  vehicle_position,
			                  &temp_point);
			                  
			d = distance (&vehicle_position, &temp_point);
			if (d < min_d)
			{
				min_d = d;
				point -> x = temp_point.x;
				point -> y = temp_point.y;
				*road = temp_road;
				*index = i;
				
				// cout << "d= " << d << endl;
			}
		}
		look = look -> next;
	}
	
	*road_distance = min_d;
	// cout << "Distance " << min_d << endl;
	// cout << "Point " << point->x << "  " << point->y << endl;	
}

void get_perpendicular_point (point_t p1, point_t p2, point_t p3,
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
bool point_between (point_t p1, point_t p2, point_t p3)
{
	// see if p3 is between p2 and p1. Note that we
	if  ( ( (p3.x <= p2.x && p3.x >= p1.x) ||
	        (p3.x >= p2.x && p3.x <= p1.x)) &&
	        
	      ( (p3.y <= p2.y && p3.y >= p1.y) ||
		    (p3.y >= p2.y && p3.y <= p1.y)))
		return true;
	else return false;
}

//
// Given two points defining a line and a third point defining the
// vehicle position, find a fourth point on that line closest to
// the vehicle.  If the fourth point is between the first and
// second point use that as the closest distance, otherwise use
// one of the first two points as the closest.
//
void get_fourth_point (point_t p1, point_t p2, point_t vpos,
				       point_t* point)
{
	double d1;
	double d2;
	point_t p4; 

	get_perpendicular_point (p1,p2,vpos,&p4);
	
	if (point_between (p1,p2,p4))	
	{
		point -> x = p4.x;
		point -> y = p4.y;
		//if (debug)
		// 	cout << "Found p4 " << p4.x << "  " << p4.y << endl;
		return;
	}
	
	// Point is not between p2 and p1. Compute the distance to p1 and p2
	// and see which one is closer.
	d1 = distance (&p1,&vpos);
	d2 = distance (&p2,&vpos);
	
	if (d1 < d2)
	{
		point -> x = p1.x;
		point -> y = p1.y;
	}
	else
	{
		point -> x = p2.x;
		point -> y = p2.y;
	}
	
	// cout << "Found p1, p2  " << d1 << "  " << d2 << endl;
	return;
}

void lookup_road_name (int recno,char* name)
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
double angle (point_t p1, point_t p2)
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
double heading_difference (double h1, double h2)
{
	double h3;
	
	h3 = h2 - h1;
	if (h3 < -180) h3 = h3 + 360;
	else if (h3 > 180) h3 = h3 - 360;
	return h3;
}

//
// Given the vehicle heading and the closest road to the vehicle, determine
// what direction on the road to travel to minimize the difference between the
// angle of the road and the vehicle heading
//	
void determine_road_direction()
{
	int direction = 0;	// direction we want to travel along the road
						// 1-positive -1-negitive 0-no direction
	point_t point1;
	point_t point2;
	road_record_t* road;
	int i;
	int j;
	int k;
	int count = 0;
	double angle1 = -1;
	double angle_diff;

	road = closest_road;						
	if (road == NULL) return;
	
	i = closest_road_index;
	j = road->numpoints - 1; // last point on the road
	
	if (i > j)
	{
		// This will only happen if we made a mistake in determining
		// the closest_road_index.
		cout << "Logic error in determine_road_direction()" << endl;
		return;
	}

	// if we are at the end of the road back off one point	
	if (i == j) i = i - 1;
	
	if (i < 0)
	{
		// fewer than two points in road - map error
		cout << "Map error in determine_road_direction()" << endl;
		return;
	}
	j = i + 1;
	
	point1.x = road -> points[i].x;
	point1.y = road -> points[i].y;
	point2.x = road -> points[j].x;
	point2.y = road -> points[j].y;

	// see if closest_point is coincident with point1 or point2	
	if ((point1.x==closest_point.x) && (point1.y==closest_point.y))
	{
		direction = 1;
		angle1 = angle (point1,point2);
		if (debug)
			cout << "Closest point is point1, angle: " << angle1 << endl;
	}
	else if ((point2.x==closest_point.x) && (point2.y==closest_point.y))
	{
		direction = -1;
		angle1 = angle (point2,point1);
		if (debug)
			cout << "Closest point is point2, angle: " <<  angle1 << endl;
	}
	else // closest_point is between point1 and point2
	{
		direction = 1;
		angle1 = angle (closest_point,point2);
		if (debug)
			cout << "Closest point is between point1 and point2, angle: " 
			 << angle1 << endl;
	};

	// Compute the difference between vehicle heading and road direction
							
	angle_diff = heading_difference (angle1,vehicle_heading);
		
	// See if the difference is plus or minus 90 degrees,
	// if so go in the opposite direction
	if ((angle_diff > 90) || (angle_diff < -90)) 
	{
		if (debug) cout << "Angle difference > 90 degrees" << endl;
		direction = -direction;
	}
	
	if (debug)	
	cout << "Direction: " << direction <<  "  I: " << i << "  J: " << j << endl;
	closest_direction = direction;
	
	// Change closest road index to J if we are decrementing the index
	if (direction == -1) closest_road_index = j;
	
	// Inform the user if we are going off the end of the road
	if ((i == 0) && (direction == -1))
	{
		if (debug)
			cout << "Going off beginning of the road at point: " 
		     	 << road->points[i].x << " " << road->points[i].y << endl;
	}
	else if ((j == road->numpoints - 1) && (direction == 1))
	{
		if (debug)
			cout << "Going off end of the road at point: "
		     	 << road->points[j].x << " " << road->points[j].y << endl;
	}
	else
	{
		// display the next five points along the direction of travel
		
		k = i;
		while (true)
		{
			if ((k<0) || (k>=road->numpoints)) break;
			cout << "[" << k << "] " 
			     << road->points[k].x << " " << road->points[k].y << endl;
			count++;
			k = k + direction;
			if (count >= 5) break;
		}
	};				
}

//
// Find the closest intersection to Road at point index Index in the given
// direction. Determine the intersection type also.
//
void lookup_table_find_intersection (road_record_t* road,
									 point_t point,
                                     int index,
                                     int direction)
{
	int x;
	int y;
	int x_offset;
	int y_offset;
		
	get_xy_coordinates (vehicle_position,x,y,x_offset,y_offset);
	
	lookup_cell_intersection (x,y,road,point,index,direction);
	
	lookup_cell_intersection (x+x_offset,y,road,point,index,direction);
	
	lookup_cell_intersection (x,y+y_offset,road,point,index,direction);
	                          
	lookup_cell_intersection (x+x_offset,y+y_offset,road,point,index,
	                          direction);

	if (intersection_list != NULL)
	{
		cout << "Found intersection at distance " 
		     << intersection_list->distance << endl;
	}
	else cout << "No intersection was found" << endl;		
}

//
// add intersection to the intersection list
//
void add_intersection (intersection_rec_t intersection)
{
	intersection_rec_t* inter;
	char street[35];
	
	inter = intersection_list;
	
	while (inter != NULL)
	{
		// If the intersecting road is already in the list do not duplicate it.
		if (inter->road == intersection.road) return;
		inter = inter->next;
	}

	if (debug)
	{
		lookup_road_name (intersection.road -> count, street);
		cout << "Intersection Distance " << intersection.distance << endl;
		cout << "Intersecting point "
			 << intersection.point.x << "  "
			 << intersection.point.y << endl;
		cout << "Intersection type: " << intersection.type << endl;
		cout << "Angle: " << intersection.angle << endl;
	}
	
	inter = new (intersection_rec_t);
	*inter = intersection;

	if (intersection_list == NULL)
	{
		intersection_list = inter;
		inter->next = NULL;
		return;
	}
		
	inter->next = intersection_list;
	intersection_list = inter;
					
}

//
// free intersection list if necessary
//
void free_intersection_list ()
{
	intersection_rec_t* inter;
	
	if (intersection_list == NULL) return;
	
	do
	{
		inter = intersection_list;
		intersection_list = inter -> next;
		delete (inter);
	} while (intersection_list != NULL);
}

//
// Look at all the roads that pass through a cell to see if one or
// more intersect with the road the vehicle is on.  Pick the one that is
// closest.
//
void lookup_cell_intersection (int x, int y,
							   road_record_t* road,
							   point_t point,
							   int index,
							   int direction)
{
	int i;
	int j;
	int k;
	lookup_record_t*	look;
	road_record_t*		temp_road;
	point_t				p1;
	point_t				p2;
	point_t				intersecting_point;
	point_t				close_point;
	intersection_rec_t	intersection;
	double				d;
	double				min_distance = 9999999;
	double				road_distance = 0;  // total distance searched
	
	// initialize intersection structures to not found
	intersection.road = NULL;
	intersection.point.x = 0;
	intersection.point.y = 0;
	intersection.distance = 9999999;
	
	i = y * x_cells + x;  // index into lookup table
	
	if (i < 0 | i > x_cells * y_cells - 1)
	{
		// cout << "Lookup Cell Outside map" << endl;
		return;
	}

	look = &lookup_table[i];

	// return no data if there are no roads in this cell	
	if (look->road == NULL)
	{
		cout << "No roads in this cell" << endl;
		return;
	}
	
	k = index;
	
	// make sure that the value for index is reasonable, otherwise
	// it was determined improperly
	if (((k <= 0) && (direction == -1)) || 
		((k >= road->numpoints-1) && (direction == 1)))
	{
		cout << "Invalid index in lookup_cell_intersection " << k << endl;
		return;
	}
	
	// 
	while (true)
	{
		if (((k <= 0) && (direction == -1)) || 
		    ((k >= road->numpoints-1) && (direction == 1)))  break;
		
		look = &lookup_table[i];
		
		// for the first time through the loop determine which point to
		// use
		if (k == index)
		{
			p1 = point;
			p2 = road->points[k+direction];
		}
		else
		{
				p1 = road->points[k];
				p2 = road->points[k+direction];
		};
	
		while (look != NULL)
		{
			temp_road = look -> road;
			
			// Avoid trying to find a road intersecting itself
			if (temp_road == road)
			{
				look = look -> next;
				continue;
			}
		
			for (j = 0; j < temp_road->numpoints-1; j++)
			{
				find_intersection (p1,p2, 
								   temp_road->points[j],
				                   temp_road->points[j+1],
				                   &intersecting_point);
				                   
				// if the point is (0,0) no intersection was found so continue
				if (intersecting_point.x==0 && intersecting_point.y==0)
					continue;
				
                determine_intersection_type (p1,p2,
					    		  temp_road->points[j],
					    		  temp_road->points[j+1], 
					    		  intersecting_point,
					    		  road,
					    		  temp_road,
					    		  k,
					    		  j,
					    		  direction,
					    		  &intersection);
					    		  
				// If we have a valid intersection for temp_road stop looking
				// on this road					    		  
				if (intersection.road != NULL) break;
			}
			
			if (intersection.road != NULL)
			{
				// See if this new intersection is closer than the current
				// one.  If the difference in the intersections is less 
				// than 10 meters then consider the two intersections
				// coincident.
				d = distance (&intersection.point,&p1);
				if (intersection_list != NULL)
					close_point = intersection_list->point;
				else
				{ close_point.x = 0; close_point.y = 0;}
				 
				if ((d < min_distance) ||
				    close_to(close_point,intersection.point))
				{
					min_distance = d;
					
					// save the distance of all segments searched
					intersection.distance = road_distance + d;
					
					// Add intersection to intersection_list
					add_intersection (intersection);
				}
			}
			
			// get the next road in the cell		
			look = look -> next;
		}
		// If we have found an intersection for the current road segment
		// then we are done
		if (intersection.road != NULL) break;
		
		// Add the current segments length to the road distance traveled
		road_distance = road_distance + distance(&p1,&p2);
		
		k = k + direction;
	}
}

//
// Test find_intersection
//
void test_find_intersection ()
{
	point_t p1, p2, p3, p4, p5;
	
	p1.x=1.5;	p1.y=1.0;
	p2.x=3.5;	p2.y=4.0;
	p3.x=1.5;	p3.y=4.0;
	p4.x=3.5;	p4.y=1.0;
	
	find_intersection (p1,p2,p3,p4,&p5);
	// Result should be p5.x=2.5, p5.y=2.5
	cout << "Test find_intersection " << p5.x << "  " << p5.y << endl;
	
	p4.x=4.5;	p4.y=5.0;
	find_intersection (p1,p2,p3,p4,&p5);
	// Result should be about p5.x=3.8, p5.y=4.7
	cout << "Test find_intersection " << p5.x << "  " << p5.y << endl;
	
	p3.x=1.5;	p3.y=5.0;
	find_intersection (p1,p2,p3,p4,&p5);
	// Result should be about p5.x=3.8, p5.y=5.0
	cout << "Test find_intersection " << p5.x << "  " << p5.y << endl;
	
	return;
}
//
// Find the intersection of two lines defined by points (p1,p2) and (p3,p4).
// Return the value in the variable point.
//
void find_intersection (point_t p1, point_t p2,
					    point_t p3, point_t p4, point_t* point)
{
	double a1;	// holds slope of (p1,p2)
	double a2;  // holds slope of (p3,p4)
	double b1;  // holds y intercept of (p1,p2)
	double b2;	// holds y intercept of (p3,p4)
	const double underflow = 1.0e-6;
	
	point->x = 0;
	point->y = 0;
	
	// If points p1 and p2 are coincident see if either p3 or p4 are close
	// enough to p1 to form an intersection.
	if (p1.x==p2.x && p1.y==p2.y)
	{
		return;
	};
	
	// If points p3 and p4 are coincident see if one is close enough to
	// the line (p1,p2) to form an intersection
	if (p3.x==p4.x && p3.y==p4.y)
	{
		return;
	};
	
	// See if (p1,p2) is perpendicular, if so we can determine 
	// point directly
	if (fabs(p2.x - p1.x) < underflow)
	{
		point->x = p1.x;
		
		// See if (p3,p4) is perpendicular if so the lines are parallel so
		// return with no intersection
		if (fabs(p4.x - p3.x) < underflow)
		{
			point->x = 0;
			point->y = 0;
			return;
		}
		else if (fabs(p4.y - p3.y) < underflow)
		{
			// line (p3,p4) is horizontal
			point->y = p3.y;
		}
		else
		{
			// determine the y intercept of (p3,p4)
			a2 = (p4.y-p3.y)/(p4.x-p3.x);
			b2 = p3.y - a2 * p3.x;
			point->y = p1.x * a2 + b2;
		}

	}
	// see if (p1,p2) is horizontal
	else if (fabs(p2.y - p1.y) < underflow)
	{
		point->y = p1.y;
		
		// See if (p3,p4) is perpendicular 
		if (fabs(p4.x - p3.x) < underflow)
		{
			point->x = p3.x;
		}
		else if (fabs(p4.y - p3.y) < underflow)
		{
			// line (p3,p4) is horizontal and the lines are parallel so
			// return with no intersection
			point->x = 0;
			point->y = 0;
			return;
		}
		else
		{
			a2 = (p4.y-p3.y)/(p4.x-p3.x);
			b2 = p3.y - a2 * p3.x;
			point->x = (p1.y - b2)/a2;
		}
	}
	else
	{
		a1 = (p2.y - p1.y) / (p2.x - p1.x);
		b1 = p1.y - a1 * p1.x;
		
		a2 = (p4.y - p3.y) / (p4.x - p3.x);
		b2 = p3.y - a2 * p3.x;
		
		if (fabs(a1-a2) < underflow)
		{
			// lines are parallel, return with no intersection
			point->x = 0;
			point->y = 0;
			return;
		}
		
		point->x = (b2 - b1) / (a1 - a2);
		point->y = a1 * point->x + b1;
	};
	return;
}

//
// Return true if we are at the beginning or end of a road
//
bool at_end (road_record_t* road, int index)
{
	if ((index >= road->numpoints-1) || (index <= 0))
		return true;
	else
		return false;
}

//
// Return true if the two points are within 10 meters of each other
//
bool close_to (point_t point1, point_t point2)
{
	double d;
	
	d = distance (&point1,&point2);
	if (d < 10.0) return true;
	return false;
}

void determine_intersection_type (point_t p1, point_t p2,
					    		  point_t p3, point_t p4, 
					    		  point_t point,
					    		  road_record_t* road1,
					    		  road_record_t* road2,
					    		  int index1,
					    		  int index2,
					    		  int direction,
					    		  intersection_rec_t* intersection)
{	
	bool between_line1 = false;
	bool between_line2 = false;
	double angle_diff;
	intersection_t intersection_type = UNKNOWN;
	point_t intersecting_point;
	
	// initialize values for return parameters
	intersection_type = UNKNOWN;
	intersecting_point.x = 0;
	intersecting_point.y = 0;
	
	// see if point is between p2 and p1
	between_line1 = point_between (p1,p2,point);
	
	// see if point is between p4 and p3
	between_line2 = point_between (p3,p4,point);	
	
	// Get the intersecting angle of road1 and road2..  Note that we use
	// the angle of the vector (p4,p3) so we are looking back down the
	// intersecting road.
	angle_diff = heading_difference (angle(p1,p2), angle(p4,p3));
		

	// See if p2 is close to p4.  If so we can determine the intersection
	// type by knowing the angle of the intersection and whether or not
	// p2 and/or p4 are endpoints on their respective roads.
	if  (close_to(p2,p4))
	{	
		intersecting_point = p4;
				
		// See if the angle is less than 30 degrees of the oncoming road
		if ((angle_diff < 30) && (angle_diff > -30))
		{
			if (at_end (road2,index2+1)) // p4 is an endpoint
			{
				intersection_type = THROUGH_WAY;
			}
			else if (at_end (road1,index1+direction))
				intersection_type = TEE;
			else
				intersection_type = FOUR_WAY;
		}
		else if (angle_diff < 0) // then it is a right turn
		{
			if (at_end (road2,index2+1)) // p4 is an endpoint
			{
				if (at_end (road1,index1+direction))  // p2 is an endpoint
					intersection_type = RIGHT_TURN_ONLY;
				else
					intersection_type = RIGHT_TURN;
			}
			else if (at_end (road1,index1+direction))
				intersection_type = TEE;
			else
				intersection_type = FOUR_WAY;
		}
		else
		{
			if (at_end (road2,index2+1))
			{
				if (at_end (road1,index1+direction))
					intersection_type = LEFT_TURN_ONLY;
				else
					intersection_type = LEFT_TURN;
			}
			else if (at_end (road1,index1+direction))
				intersection_type = TEE;
			else
				intersection_type = FOUR_WAY;
		}
	}
	// We must also perform the same test on p3 close to p2
	else if  (close_to(p2,p3))
	{	
		intersecting_point = p3;
		
		// Get the intersecting angle of line1 and line2
		// Note that we are using the vector (p3,p4) instead of (p4,p3)
		angle_diff = heading_difference (angle(p1,p2),angle(p3,p4));
		
		// See if the angle is less than 30 degrees of the oncoming road
		// Note that the test is different when the end point is p3 not p4
		if ((angle_diff < 30) && (angle_diff > -30))
		{
			if (at_end (road2,index2+1)) // p4 is an endpoint
			{
				intersection_type = THROUGH_WAY;
			}
			else if (at_end (road1,index1+direction))
				intersection_type = TEE;
			else
				intersection_type = FOUR_WAY;
		}
		// See if we are making a right turn
		else if (angle_diff < 0)
		{
			if (at_end (road2,index2+1)) // p4 is an endpoint
			{
				if (at_end (road1,index1+direction))  // p2 is an endpoint
					intersection_type = RIGHT_TURN_ONLY;
				else
					intersection_type = RIGHT_TURN;
			}
			else if (at_end (road1,index1+direction))
				intersection_type = TEE;
			else
				intersection_type = FOUR_WAY;
		}
		else
		{
			if (at_end (road2,index2+1))
			{
				if (at_end (road1,index1+direction))
					intersection_type = LEFT_TURN_ONLY;
				else
					intersection_type = LEFT_TURN;
			}
			else if (at_end (road1,index1+direction))
				intersection_type = TEE;
			else
				intersection_type = FOUR_WAY;
		}
	}
	else if (close_to(point,p2) && between_line2)
	{
		intersecting_point = point;
		
		if (at_end(road1,index1+direction))
			intersection_type = TEE;
		else
			intersection_type = FOUR_WAY; 
	}
	else if (between_line1 && between_line2)
	{
		intersecting_point = point;
		
		intersection_type = FOUR_WAY;
	}
	
	intersection->type = intersection_type;
	intersection->point = intersecting_point;
	
	if (intersection_type != UNKNOWN)
	{
		// we have found an intersection
		intersection->angle = angle_diff;
		intersection->index = index2;
		intersection->road = road2;
	}
	else
	{
		intersection->angle = 0;
		intersection->index = -1;
		intersection->road = NULL;
	}
}

//
// Read waypoint file and put waypoint records into a linked list
//
void read_waypoint_file ()
{
	FILE *input;
	char* input_filename = "/home/timnich/waypoints.txt";
	double x;
	double y;
	double width;
	double speed;
	int count = 0;
	waypoint_record_t* waypoint;
	waypoint_record_t* last_waypoint = NULL;
	
	input = fopen (input_filename, "r"); 
	if (!input) error ("Cannot open waypoint file ",input_filename);

	while (!feof(input))
	{
		fscanf (input, "%le %le %le %le", &x, &y, &width, &speed);
		count ++;
		if (debug)
		    cout << "waypoint " << count << " x = " << x << " y = " << y 
		         << " width = " << width << endl;

		// Convert from feet to meters		         
		width = width * meters_per_foot;

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
bool point_in_waypoint (waypoint_record_t* waypoint, point_t point)
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
// test get_perpendicular_point
//
void test_get_perpendicular_point ()
{
	point_t point1;
	point_t point2;
	point_t point3;
	point_t point4;
	
	point1.x = 1;
	point1.y = 1;
	point2.x = -5;
	point2.y = 6;
	point3.x = 4;
	point3.y = 2;
	
	get_perpendicular_point (point1,point2,point3,&point4);
	if (point_between (point1,point2,point4))
		cout << "Point4 between point1 and point2" << endl;
}

//
// Return true if the point is in the waypoint corridor
//
bool point_in_corridor (point_t point, road_record_t* road)
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
void waypoint_find_roads ()
{
	waypoint_record_t* waypoint;
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
	
	waypoint = waypoint_list;
	
}

//
// Assign each road in corridor_road_list to one or more waypoint segments
// if it is contained partially or completely within the segment
//
void waypoint_assign_roads ()
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
void waypoint_rectangle (waypoint_record_t* way)
{	 
	double d_latitude;
	double d_longitude;
	double waypoint_angle = 0;
	double waypoint_sine;
	double waypoint_cosine;
	const double underflow = 1.0e-6;

	// convert corridor width from meters to degrees	
	d_latitude = way->width / latitude_meters (way->point1.y);
	d_longitude = way->width / longitude_meters (way->point1.y);
	
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
		way->p1.x = way->point1.x - d_longitude;
		way->p1.y = way->point1.y;
		
		way->p2.x = way->point1.x - d_longitude;
		way->p2.y = way->point2.y;
		
		way->p3.x = way->point1.x + d_longitude;
		way->p3.y = way->point1.y;
		
		way->p4.x = way->point1.x + d_longitude;
		way->p4.y = way->point2.y;
	}
	// see if the slope is horizontal
	else if (fabs(way->point2.y - way->point1.y) < underflow)
	{
		way->p1.x = way->point1.x;
		way->p1.y = way->point1.y + d_latitude;
		
		way->p2.x = way->point2.x;
		way->p2.y = way->point2.y + d_latitude;
		
		way->p3.x = way->point1.x;
		way->p3.y = way->point1.y - d_latitude;
		
		way->p4.x = way->point2.x;
		way->p4.y = way->point2.y - d_latitude;
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

		// Compute the x,y offsets in degrees		
		waypoint_sine = sin (waypoint_angle) * d_latitude;
		waypoint_cosine = cos (waypoint_angle) * d_longitude;
		
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

void determine_waypoint_heading ()
{
	waypoint_record_t*	waypoint;
	waypoint_record_t*	new_waypoint = NULL;

	waypoint = waypoint_list;
	
	while (waypoint != NULL)
	{
		if (point_in_waypoint (waypoint, vehicle_position))
		{
			new_waypoint = waypoint;
		}
		waypoint = waypoint->next;
	}
	
	if (new_waypoint == NULL)
		cout << "Vehicle outside corridor" << endl;
	else
		current_waypoint = new_waypoint;
	
	if (current_waypoint != NULL)
	{
		waypoint_heading = angle (vehicle_position,current_waypoint->point2);
		cout << "Vehicle in waypoint " << current_waypoint->count << endl;
		cout << "Waypoint heading is " << waypoint_heading << endl;
	}
}

void read_correction_file()
{
	FILE *input;
	char* input_filename = "/home/timnich/correction.txt";
	double x;
	double y;
	double delta_x;
	double delta_y;
	correction_record_t* correction;
	
	input = fopen (input_filename, "r"); 
	if (!input) error ("Cannot open correction file ",input_filename);

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
void get_correction (point_t* point)
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

//
// Find the intersections at the beginning and ending of a road segement.  This
// routine assumes that intersections are found only at the beginning and end
// of the road segments. 
//
void find_intersecting_roads (road_record_t* road,
                              intersection_rec_t* intersection1,
                              intersection_rec_t* intersection2)
{
	corridor_road_t* 	corridor;
	road_record_t*		temp_road;
	intersection_rec_t*	temp_intersection;
	
	corridor = corridor_road_list;
	intersection1 = NULL;
	intersection2 = NULL;
	
	while (corridor != NULL)
	{
		temp_road = corridor->road;
		
		if (close_to(road->points[0],temp_road->points[0]) ||
		    close_to(road->points[0],temp_road->points[temp_road->numpoints-1]))
		{
			temp_intersection = new (intersection_rec_t);
			temp_intersection->road = temp_road;
			//add_intersection (intersection1, temp_intersection);
		}
		
		if (close_to(road->points[road->numpoints-1],temp_road->points[0]) ||
		    close_to(road->points[road->numpoints-1],temp_road->points[temp_road->numpoints-1]))
		{
			temp_intersection = new (intersection_rec_t);
			temp_intersection->road = temp_road;
			//add_intersection (intersection2, temp_intersection);
		}
		
		corridor = corridor->next;
	}
}
