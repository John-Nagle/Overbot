////////////////////////////////////////////////////////////////////////////////
//
//    File: roadserver.h
//
//    Description:
//		Road Server main class
//
//			Reads in a road map binary file downloaded from:
//				www.usgs.seamless.gov
//			into memory. 
//
//    See also:
//
//    Written By:
//       Tim Nicholson (timnich@ix.netcom.com)
//       Team Overbot
//       January, 2004
//
/////////////////////////////////////////////////////////////////////////////////

#ifndef ROADSERVER_H
#define ROADSERVER_H

#include <stdio.h>
#include <pthread.h>

#include "messaging.h"
#include "mutexlock.h"

#include "roadservermsg.h"

using namespace std;

// RoadServer main class 
class RoadServer {

public:
    RoadServer();			    // constructor
    ~RoadServer();			// destructor
    
	//
	// define structure to hold an x-y coordinate
	//
	struct point_t
	{
		double	x;
		double	y;
	};

	// Reads USGS road datafile and builds a list of roads	    
    void readRoadFile(char *fileDirectory);

    void setVerbose(bool on) {
    	verbose = on;
    }
    
	// Given a rectangle defined by point1 and point2, return a list of
	// roads within that rectangle.
    void handleRoadRequest(point_t point1, point_t point2);
	void get_waypoints ();
	void waypoint_find_roads ();
    
    void messageThread();
	    
private:

    // Message handling
    MsgServerPort	*serverPort;	// receiving port for msgs
    MsgClientPort 	*clientPort;	// client port for sending msgs out on
    int 			rcvid;			// channel id for reply
    RoadServerMsg::MsgUnion	msg;	// area for incoming and outgoing msgs
	    
	//
	// structure to hold a linked list of road database records
	// 
	struct road_record_t
	{
		int				numparts;	// number of line segments in this record
		int				numpoints;  // number of points for all line segments
		int				count;		// record count to look up street name
		int*			parts;		// index into points array where segment starts
		point_t*		points;		// lat-lon of point in degrees
		point_t*		xy_points;	// relative location of point in meters
		road_record_t*	next;
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

	// List of coordinate correction records
	correction_record_t* correction_list;
	
	// List of waypoint records
	waypoint_record_t* waypoint_list;

	// store the boundries of the map
	point_t map_lower_left;
	point_t map_upper_right;
	
	int size;  // keeps tract of byes read in process_record

	road_record_t* road_list;
	corridor_road_t* corridor_road_list;
		    
    bool		verbose;
    bool		debug;
    bool		useWatchdog;
    
    int			count;
    int			count_parts;
    
	FILE *inputdbf;
	FILE *input;

	void process_header();
	void process_record();
	void allocate_road_record (int, int);
	void read_correction_file(char* fileDirectory);
	double distance (point_t* point1, point_t* point2);
	void get_correction (point_t* point);
	void handleMessage();
	void handleGetRoadList();
	void get_perpendicular_point (point_t p1, point_t p2, point_t p3,
                              point_t* point);
	bool point_between (point_t p1, point_t p2, point_t p3);
	void lookup_road_name (int recno,char* name);
	double angle (point_t p1, point_t p2);
	double heading_difference (double h1, double h2);
	bool point_in_waypoint (waypoint_record_t* waypoint, point_t point);
	bool point_in_corridor (point_t point, road_record_t* road);
	void waypoint_assign_roads ();
	void waypoint_rectangle (waypoint_record_t* way);
};

#endif // ROADSERVER_H
