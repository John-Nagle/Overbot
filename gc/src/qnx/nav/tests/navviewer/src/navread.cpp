//
//	navread.cpp  --  support for displaying the navigation system results
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
#include <cmath>
#include "navread.h"
#include "logitem.h"
#include "logitem_impl.h"					//	so PhAB make system will see the dependency
#include "proto.h"
#include "abimport.h"

//
//	Constants
//
//	The outline of the vehicle, as a unit square. Scaled according to scaling info in log file header.
//	0,0 is the center of the vehicle.  Vehicle is facing in the +X direction.
//
const vec2 k_vehoutline[] = {
	vec2(-0.5,-0.5), vec2(-0.5,0.5), vec2(0.5,0.5), vec2(0.625,0), vec2(0.5,-0.5)
	};
	
const int k_vehoutlinen = sizeof(k_vehoutline) / sizeof(k_vehoutline[0]);		// number of points
                        
const char* servername = "ROAD";										// name of server in watchdog system
const char* k_untitled = "Overbot Navigation Viewer";			// Default window title
const size_t k_history_size = 10000;									// save and draw this much history
const int k_high_zoom = (1<<7);											// largest zoom factor
//
//	Uitlity functions
//
//
//	arccenter  -- compute center of an arc
//
//	Ref: "Ask Dr. Math" (http://mathforum.org/dr.math/faq/faq.circle.segment.html), case 7
//
static vec2 arccenter(const vec2& p0, const vec2& p1, float curvature)
{	const vec2 midpoint((p0+p1)*0.5);									// midpoint of chord
	const double k_min_curvature = 0.0001;							// to avoid divide by 0
	const double k_min_length = 0.0001;								// to avoid divide by 0
	if (fabs(curvature) < k_min_curvature)								// if straight line
	{	return(midpoint);	}														// just average points
	//	Curved arc, must do real work.
	const double r = fabs(1.0 / curvature);								// radius of arc
	vec2 chord(p0-p1);															// chord
	double c = chord.length();												// length of chord
	if (c < k_min_length) return(midpoint);								// very short chord, use other
	double theta = 2*asin(c/(2*r));											// base angle of arc
	if (!std::finite(theta)) return(midpoint);								// very tight arc, more than 180 degrees, return bogus result
	double d = r*cos(theta/2);												// distance from center to chord
	////double h = r - d;															// distance from chord to midpoint
	vec2 chordvector(chord/c);												// chord direction unit vector
	vec2 radial(chordvector[1], -chordvector[0]);					// radial to right of chord
	if (curvature > 0) radial = -radial;										// radial is unit vector from center towards midpoint	
	return(midpoint - radial*d);												// return center					
}
//
//	radians2bigrads --  convert angle in radians to "bigrads"
//
static int radians2bigrads(double ang)
{
	int bigrads = int(ang*(65536/(M_PI*2)));							// convert to bigrads
	return((bigrads+(65536*2)) % 65536);							// insure positive value
}
//
//	atan  -- arctangent of vector
//
static double atan(const vec2& v)
{	return(atan2(v[1],v[0]));	}
//
//	drawarc -- draw an arc, given endpoints and curvature
//
static void drawarc(const PhPoint_t& ip0, const PhPoint_t& ip1, float curvature)
{	const vec2 p0(ip0.x, ip0.y);												// endpoints of arc
	const vec2 p1(ip1.x, ip1.y);
	if (fabs(curvature) < 0.0001)											// if straight line
	{	PgDrawLine(&ip0, &ip1);	return;	}								// draw as a line
	////PgDrawLine(&ip0, &ip1);													// ***TEMP*** draw line anyway
	const vec2 center = arccenter(p0, p1, curvature);			// calc center of arc
	double ang0 = -atan(p0-center);										// angle of first radial
	double ang1 = -atan(p1-center);										// second angle 
	int gang0 = radians2bigrads(ang0);									// convert to "bigrads" (0..65535 is a circle)
	int gang1 = radians2bigrads(ang1);
	if (curvature > 0)																// draw is counterclockwise
	{	std::swap(gang0, gang1);	}											// reverse arcs if needed
	PhPoint_t centerp = { int(center[0]), int(center[1]) };	
	////PgDrawLine(&ip0, &centerp);											// ***TEMP*** draw line to center
	PhPoint_t radiusp = { int(1/curvature), int(1/curvature) };
	PgDrawArc(&centerp, &radiusp, gang0, gang1, Pg_ARC | Pg_DRAW_STROKE);		// draw the arc
}
//
//	Class LogFileReader -- handles reading the log file, along with positioning
//
//
//	open -- open log file
//
int LogFileReader::open(const char* filename)
{	close();
	////m_logfile.setverbose(true);								// ***TEMP***
	int stat = m_logfile.open(filename, false);			// open
	if (stat) return(stat);											// fails
	fpos_t pos;														// file position, an opaque type
	stat = m_logfile.getpos(pos);							// get position
	if (stat)																// if fail
	{	close(); return(stat); }
	m_framepositions.push_back(pos);					// save start of frame zero
	return(0);															// success
}
//
//	close -- close log file
//
void LogFileReader::close()
{	m_framenumber = 0;										// at frame 0 again
	m_framepositions.clear();									// clear positions
	m_logfile.close();												// close
}
//
//	seektoframe -- seek to a specific frame
//
//	We maintain a table of the file position at which each frame starts,
//	so we can traverse the file rapidly.  This consumes space but
//	makes moving backwards possible.
//
//	Running off the beginning or the end just leaves the position at the first
//	or last frame.
//
bool LogFileReader::seektoframe(int framenumber)
{	if (!m_logfile.isopen()) return(false);					// not open	
	if (framenumber < 0) framenumber = 0;			// must not be negative
	if (framenumber < m_framepositions.size())		// if we know this frame number's position
	{	int stat = m_logfile.setpos(m_framepositions[framenumber]);	// go there
		if (stat)															// if fail
		{	perror("Error during seek");						// fails - probably off end of file
			return(false);											// failed
		}
		m_framenumber = framenumber;					// current position is this frame
		return(true);													// success
	}
	//	Beyond point previously read, must advance
	assert(framenumber > 0);								// if zero, handled above
	assert(m_framepositions.size() > 0);				// always have frame 0 on file
	bool seekok = seektoframe(m_framepositions.size() - 1); // go to last frame
	if (!seekok) return(false);									// fails if bad seek
	//	Read until desired position reached
	while (m_framenumber < framenumber)			// until we get there
	{	LogItem item;												// scratch item
		LogItem::Type_e type;
		bool good = getitem(item, type);					// get next item
		if (!good)														// read EOF, or into garbage 
		{	//	Reposition to beginning of last good frame
			seektoframe(m_framepositions.size()-1);	
			return(false);
		}
	}
	assert(framenumber < m_framepositions.size());	// must be able to seek without another read
	return(seektoframe(framenumber));					// finish the seek
}
//
//	getitem -- read next item from file.
//
//	Returns true if success.
//
bool LogFileReader::getitem(LogItem& item, LogItem::Type_e& type)
{
	bool good = m_logfile.getitem(item, type);		// get the item
	if (!good) 															// if fail
	{	perror("Error reading log file");
		return(false);												// fails
	}
	if (type == LogItem::item_frameend)					// if end of frame
	{	m_framenumber++;										// advance frame
		if (m_framenumber >= m_framepositions.size()) // if new frame
		{	assert(m_framenumber == m_framepositions.size()); // must be in sync
			fpos_t pos;												// position in file
			int stat = m_logfile.getpos(pos);				// get current position
			if (stat) 
			{	perror("Error getting file position");
				return(stat);											// fails
			}
			m_framepositions.push_back(pos);			// add new position for later seeks
		}
	}
	return(true);														// now have item
}
//
//	Constructor
//
NavRead::NavRead()
:  	m_map(1,1),														// dummy values, will resize
	m_poshistory(*this, k_history_size, Pg_BLACK),
	m_goalhistory(*this, k_history_size, PgRGB(0,255,0)),	// must be same color as goal point circle
	m_headerread(false),
	m_readmode(mode_closed), m_playing(false), m_showobstacles(true), m_showwaypoints(true), m_image(0)
{	m_online = (getenv("ID") != 0);						// true if running under watchdog
	//	Set state of checkboxes before menu displays
	ApModifyItemState(&view_menu, (showobstacles() ? AB_ITEM_SET : AB_ITEM_NORMAL), ABN_obstacles_toggle, NULL);
	ApModifyItemState(&view_menu, (showwaypoints() ? AB_ITEM_SET : AB_ITEM_NORMAL), ABN_waypoint_toggle, NULL);
}
//	
//	Destructor
//
NavRead::~NavRead()
{	
	close();																				// close if open
}
//
//	drawcell  -- draw solid-color cell for indicated cell number
//
//	Must be set to draw into video port
//
//	Inefficient. 
//
void NavRead::drawcell(int ix, int iy, const PgColor_t color, float elev)
{	double rx = (ix / m_logheader.m_cellspermeter) - m_logframe.m_x;	// cell position relative to vehicle (center)
	double ry = (iy / m_logheader.m_cellspermeter) - m_logframe.m_y;	// cell position relative to vehicle (center)
	PhRect_t cellrect;
	if (convertmappt(cellrect.ul,rx,ry))										// if on grid
	{	cellrect.lr = cellrect.ul;														// construct rectangle
		const int dim = int(m_zoom / m_logheader.m_cellspermeter) +1;	// size of a cell, in pixels
		cellrect.lr.x += dim;
		cellrect.lr.y += dim;
		if (color != Pg_GREEN || (m_showobstacles)) 					// skip green areas if not showing them
		{	PgSetFillColor(color);														// set for desired color
			PgDrawRect(&cellrect, Pg_DRAW_FILL);							// draw filled rectangle
		}
		//	Put elevation on cell, but only at highest zooms. And in really tiny type.
		if (m_zoom >= k_high_zoom)											// if fully zoomed in
		{
			//	Draw elevation as number.
			char s[20];
			PhPoint_t center0 = { (cellrect.ul.x + cellrect.lr.x) / 2, (cellrect.ul.y + cellrect.lr.y)/2 };	// center of cell
			snprintf(s,sizeof(s),"%1.2f",elev);									// edit elevation as string
			PgDrawText(s,strlen(s),&center0, Pg_TEXT_CENTER | Pg_TEXT_MIDDLE);	// display the text
		}
	}
}

#ifdef OBSOLETE

	void processtriangleitem(const LogItemLine& item); // handle a line item
	void processquaditem(const LogItemLine& item); // handle a line item
//
//	processtriangleitem -- process a line drawing item
//
//	Must be set to draw into video port
//
void NavRead::processtriangleitem(const LogItemTriangle& item)
{
	PhPoint_t pts[3];
	if (convertmappt(pts[0],item.m_x1, item.m_y1)							// convert points to drawing coordinates
		&& convertmappt(pts[1], item.m_x2, item.m_y2)
		&& convertmappt(pts[2], item.m_x3, item.m_y3))
	{	PgColor_t color;	
		convertmapcolor(color, item.m_color);								// convert color
		PgSetStrokeColor(color);													// set drawing color
		PgSetFillColor(color);
		const PhPoint_t offset = { 0,0};											// no offset
		PgDrawPolygon(pts,3, &offset, Pg_DRAW_STROKE | Pg_CLOSED);						// don't fill triangles ***TEMP***
		////PgDrawPolygon(pts,3, &offset, (item.m_fill ? Pg_DRAW_FILL : Pg_DRAW_STROKE) | Pg_CLOSED);
	}
}
//
//	processquaditem -- process a line drawing item
//
//	Must be set to draw into video port
//
void NavRead::processquaditem(const LogItemQuad& item)
{
	PhPoint_t pts[4];
	if (convertmappt(pts[0],item.m_x1, item.m_y1)							// convert points to drawing coordinates
		&& convertmappt(pts[1], item.m_x2, item.m_y2)
		&& convertmappt(pts[2], item.m_x3, item.m_y3)
		&& convertmappt(pts[3], item.m_x4, item.m_y4))
	{	PgColor_t color;	
		convertmapcolor(color, item.m_color);								// convert color
		PgSetStrokeColor(color);													// set drawing color
		PgSetFillColor(color);
		const PhPoint_t offset = { 0,0};											// no offset
		PgDrawPolygon(pts,4, &offset,  Pg_DRAW_STROKE | Pg_CLOSED);					// don't fill quads ***TEMP***
		////PgDrawPolygon(pts,4, &offset, (item.m_fill ? Pg_DRAW_FILL : Pg_DRAW_STROKE) | Pg_CLOSED);
	}
}
#endif // OBSOLETE
//
//	processlineitem  -- process a line to draw
//
//	Just stores for later draw
//
void NavRead::processlineitem(const LogItemLine& item)
{
	m_lines.push_back(item);													// save for later draw
}
//
//	processcircleitem -- process a circle to draw
//
void NavRead::processcircleitem(const LogItemCircle& item)
{
	m_circles.push_back(item);												// save for later
	//	Based on color, draw appropriate goal history
	PgColor_t color;	
	convertmapcolor(color, item.m_color);								// convert color
	if (m_goalhistory.getcolor() == color)								// if right color to be goal history
	{	m_goalhistory.add(m_logframe.m_framenumber,
			item.m_rx+m_logframe.m_x,  item.m_ry+m_logframe.m_y);		// add to history
	}
}
//
//	processarcitem -- process an arc to draw
//
//	Just stores for later draw
//
void NavRead::processarcitem(const LogItemArc& item)
{
	m_arcs.push_back(item);
}
//
//	processarrowitem -- process an arrow to draw
//
//	Just stores for later draw
//
void NavRead::processarrowitem(const LogItemArrow& item)
{
	m_arrows.push_back(item);
}
//
//	drawlineitem -- draw a line item
//
//	Must be set to draw into video port
//
void NavRead::drawlineitem(const LogItemLine& item)
{
	PhPoint_t pt0, pt1;
	if (convertmappt(pt0,item.m_x0, item.m_y0)							// convert points to drawing coordinates
	&& convertmappt(pt1, item.m_x1, item.m_y1))
	{	PgColor_t color;	
		convertmapcolor(color, item.m_color);								// convert color
		PgSetStrokeColor(color);													// draw in specified color
		PgDrawLine(&pt0, &pt1);													// draw the line
	}
}
//
//	drawcircleitem -- draw a circle item
//
//	Must be set to draw into video port
//
void NavRead::drawcircleitem(const LogItemCircle& item)
{
	PhPoint_t pt;
	if (convertmappt(pt,item.m_rx, item.m_ry))							// convert points to drawing coordinates
	{	PgColor_t color;	
		convertmapcolor(color, item.m_color);								// convert color
		PgSetStrokeColor(Pg_BLACK);											// outline in black
		PgSetFillColor(color);															// fill in color
		const PhPoint_t radii = { int(item.m_r) , int(item.m_r) };		// radius of circle
		PgDrawEllipse(&pt, &radii, Pg_DRAW_FILL_STROKE );			// draw the circle
	}
}
//
//	drawarcitem -- draw an arc
//
//	Must be set to draw into video port
//
void NavRead::drawarcitem(const LogItemArc& item)
{
	PhPoint_t pt0, pt1;
	if (convertmappt(pt0,item.m_r0x, item.m_r0y)							// convert points to drawing coordinates
	&& convertmappt(pt1, item.m_r1x, item.m_r1y))
	{	PgColor_t color;	
		convertmapcolor(color, item.m_color);								// convert color
		PgSetStrokeColor(color);													// draw in specified color
		drawarc(pt0, pt1, item.m_curvature/m_zoom);					// draw the arc
	}
}
//
//	drawarrowitem -- draw an arrow
//
//	Must be set to draw into video port
//
void NavRead::drawarrowitem(const LogItemArrow& item)
{
	PhPoint_t pos  = { 0, 0 };													// for polygon center
	PgColor_t color;	
	convertmapcolor(color, item.m_color);								// convert color
	PgSetStrokeColor(color);													// outline in black
	//	Draw the arrow ending at the given position.
	//	The arrow has the shape of the vehicle
	PhPoint_t vehoutline[k_vehoutlinen];
	double headingdeg = (180.0/M_PI)*atan2(item.m_diry, item.m_dirx);	// angle from heading (???)
	mat3 rot(rotation2D(vec2(0,0),headingdeg));						// rotate around origin
	for (int i=0;  i< k_vehoutlinen; i++)
	{	vec2 pt(k_vehoutline[i][0]*m_logheader.m_vehlength-m_logheader.m_vehcenteroffset,
		k_vehoutline[i][1]*m_logheader.m_vehwidth);	// scale point from outline
		pt = rot * pt;																		// rotate
		pt += vec2(item.m_rx, item.m_ry);										// add relative position of 
		if (!convertmappt(vehoutline[i],pt[0],pt[1])) return;			// convert to screen coords, ignore if way offscreen
	}
	PgDrawPolygon(vehoutline, k_vehoutlinen, &pos, Pg_DRAW_STROKE |  Pg_CLOSED );	// draw it in outline
}
//
//	processcellitem  -- process a cell update
//
void NavRead::processcellitem(const LogItemCell& item)
{
	if (item.m_valid)
	{	int ix = item.m_irx + m_logframe.m_ix;							// get absolute cell coords
		int iy = item.m_iry	+ m_logframe.m_iy;
		////printf("Cell [%d,%d] is %d\n", ix,iy, item.m_type);			// ***TEMP***
		if (m_map.cellonmap(ix,iy)	)										// if cell is on the map
		{	DisplayCellData& cell = m_map.at(ix,iy);					// access cell
			cell.m_color = getcolor(item.m_type);						// set cell color
			cell.m_elev = item.m_rz*0.01 + m_logframe.m_z;		// absolute elevation
			cell.m_framenumber = m_logframe.m_framenumber;			// stamp when this cell was updated
		}
	}
}
//
//	processheaderitem -- process a header item
//
void NavRead::processheaderitem(const LogItemHeader& item)
{	const int k_mapdim = 1000;												// map is 1000 cells across (should be in header)
	if (m_headerread)																// do only once
	{	return;	}
	m_logheader = item;															// save header
	printf("%3.2f cells per meter.\n",m_logheader.m_cellspermeter);	// ***TEMP***
	m_map.resize(k_mapdim,m_logheader.m_cellspermeter);	// resize and clear the map
	m_headerread = true;														// we did it
}
//
//	processframeitem -- process a beginning of frame item
//
void NavRead::processframeitem(const LogItemFrame& item)
{
	m_logframe = item;															// this becomes the current frame item
	//	Scroll the map grid as required to recenter it on the vehicle
	m_map.setmapcentercell(m_logframe.m_ix, m_logframe.m_iy);	// set new center of cell map
	//	Clear the waypoints, etc. for this frame
	m_waypoints.clear();														// no more waypoints
	m_lines.clear();																	// clear line items
	m_circles.clear();																// clear circle items
	m_arcs.clear();																	// clear arc items
	m_arrows.clear();																// clear arrow items
	//	Add position to history
	m_poshistory.add(m_logframe.m_framenumber, item.m_x, item.m_y);		// add to history
}
//
//	processwaypointitem  --  process a waypoint item
//
//	For each frame, there is one waypoint item for every waypoint visible in that frame.
//
void NavRead::processwaypointitem(const LogItemWaypoint& item)
{
	m_waypoints.push_back(item);											// add to table for this frame
}
//
//	processitem  -- process all item types
//
void NavRead::processitem(const LogItem& item, LogItem::Type_e type)
{
	switch (type) {																	// fan out on type
	case LogItem::item_frame:
		processframeitem(item.m_frame);									// do it
		break;
	case LogItem::item_cell:
		processcellitem(item.m_cell);
		break;
	case LogItem::item_waypoint:											// waypoint item
		processwaypointitem(item.m_waypoint);
		break;
	case LogItem::item_line:
		processlineitem(item.m_line);
		break;
#ifdef OBSOLETE
	case LogItem::item_triangle:
		processtriangleitem(item.m_triangle);
		break;
	case LogItem::item_quad:
		processquaditem(item.m_quad);
		break;
#endif //  OBSOLETE
	case LogItem::item_circle:
		processcircleitem(item.m_circle);
		break;
	case LogItem::item_arc:
		processarcitem(item.m_arc);
		break;
	case LogItem::item_arrow:
		processarrowitem(item.m_arrow);
		break;
	case LogItem::item_header:
		processheaderitem(item.m_header);
		break;
	default:
		break;
	}
}
//
//	readframe  -- read one log frame from file or msg, update widgets
//
//	We must be at start of frame when this is called.
//	At return, we are at the end of the frame, positioned to read the next one.
//
bool NavRead::readframe()
{
	LogItem item;
	LogItem::Type_e type;
	if (m_online)																	// if online mode
	{	// ***MORE***
		return(false);															// fails for now
	}
	//	Offline mode, read file
	for (;;) {
		bool good = m_logfile.getitem(item, type);				// get the next item
		if (!good) return(false);												// trouble
		if (type == LogItem::item_frameend) break;				// done if end of frame
		processitem(item, type);											// handle the item
	}
	return(true);																	// success
}
//
//	readframe -- draw a specified frame, by frame number
//
bool NavRead::readframe(int frame)
{	if (m_online)
	{	//	***MORE*** 
		return(false);
	}
	else
	{	if (!m_logfile.valid()) return(false);									// not open or ready, no action
		m_logfile.seektoframe(frame);										// go to beginning of frame
		bool good = readframe();												// read frame
		m_logfile.seektoframe(frame);										// back to beginning again
		return(good);
	}
}
//
//	open --  Open log file, prepare to read
//
int NavRead::open(const char* filename)
{
	m_headerread = false;														// have not read header
	m_map.clearmap();															// clear the map
	m_poshistory.clear();														// clear the history
	m_goalhistory.clear();														// clear the history
	int stat = m_logfile.open(filename);									// open for reading
	if (stat)
	{	perror("Could not open log file");									// ***TEMP***
		return(-1);																		// fails, caller will display msg
	}
	PtSetResource(ABW_base, Pt_ARG_WINDOW_TITLE, filename,0);	 // set name in window
	forceredraw();																	// redraw to bring up image
	return(0);																			// open was successful
}
//	
//	close -- close log file if needed
//
void NavRead::close()
{	m_logfile.close();																// close log file
	m_readmode = mode_closed;											// log file closed
	m_headerread = false;														// no header read
	PtSetResource(ABW_base, Pt_ARG_WINDOW_TITLE, k_untitled,0);	 // set name in window
}
//
//	convertmappt  -- convert map point relative to vehicle to map window coords
//
//	Returns false if far offscreen, to avoid bad draws
//
//	Note sign inversion for Y.
//
bool NavRead::convertmappt(PhPoint_t& pt, float x, float y)
{
	double xpt = m_zoom * x-m_scrolloffset.x;
	double ypt = m_zoom * (-y)-m_scrolloffset.y;
	if (xpt < -32767 || xpt > 32767) return(false);
	if (ypt < -32767 || ypt > 32767) return(false);
	pt.x = short(xpt);
	pt.y = short(ypt);
	return(true);
}
//
//	convertmapwinpt  -- convert map window coords to map point relative to vehicle
//
//	Inverse of above transform
//
void NavRead::convertmapwinpt(double& x, double& y, const PhPoint_t& pt)
{
	x = (pt.x + m_scrolloffset.x) / double(m_zoom);	
	y = -(pt.y + m_scrolloffset.y) / double(m_zoom);	
}
//	
//	convertmapcolor  -- convert one-byte LogColor to Photon colors
//
//	LogColor is one-bit RGB, which is rather tacky.
//
void NavRead::convertmapcolor(PgColor_t& mapcolor, LogColor logcolor)
{
	mapcolor = PgRGB( (logcolor & 0x4) ? 255 : 0, (logcolor & 0x2) ? 255 : 0, (logcolor & 0x1) ? 255 : 0);
}

//
//	testpattern  -- puts a test pattern in a cell array
//
static void testpattern(std::vector<CellData>& cells, int h, int w)
{	cells.resize(w*h);																// size appropriately
	for (int i=0; i<w; i++)
	{	for (int j=0; j<h; j++)
		{	bool ondiagonal =  (i == j || i == -j);						// mark diagonals
			CellData& cell = cells[i*w+j];									// the cell
			cell.m_valid = ondiagonal;
			cell.m_type = CellData::CLEAR;									// green on the diagonal
		}
	}
}
//
//	calccellid  -- calculate data cell needed to fill a given pixel
//
inline int calccellid(int i, int j, const PhDim_t& datadim, const PhPoint_t& offset, int zoom)
{
	i += offset.x;
	j += offset.y;
	i /= zoom;
	j /= zoom;
	return( i*datadim.w+j);													// calculate subscript
}
//
//	ongrid -- true if pixel on grid line
//
inline bool ongrid(int i, int j, const PhPoint_t& offset, int zoom)
{	
	i += offset.x;
	j += offset.y;
	return((i % zoom == 0) && (j % zoom == 0));				// true if exactly on a grid line
}
//
//	getcolor  -- get color that represents a cell
//
PgColor_t NavRead::getcolor(const CellData::CellType type)
{	switch (type) {
	case CellData::NOGO:	return(Pg_RED);						// no go
	case CellData::CLEAR: return(Pg_GREEN);						// good
	case CellData::POSSIBLE: return(Pg_YELLOW);				// not so good
	default: return(Pg_GREY);												// unknown
	}
}
//
//	drawdata -- draw the requested cell data from the map
//
void NavRead::drawdata(PtWidget_t* widget, int zoom, const PhPoint_t& scrolloffset)
{
	////if (!m_showobstacles) return;										// do not draw if turned off
	//	Find cell boundaries of the window
	double ulx, uly, lrx, lry;
	PhRect_t bounds;															// bounds of widget
	PtCalcCanvas(widget, &bounds);									// get bounds of widget
	////printf("Screen bounds: [%d,%d] - [%d,%d]\n", bounds.ul.x, bounds.ul.y, bounds.lr.x, bounds.lr.y);	// ***TEMP***
	convertmapwinpt(ulx, uly, bounds.ul);							// get upper left in vehicle coordinates
	ulx += m_logframe.m_x;												// get upper left in world coordinates
	uly += m_logframe.m_y;
	convertmapwinpt(lrx, lry, bounds.lr);
	lrx += m_logframe.m_x;												// get lower right in world coordinates
	lry += m_logframe.m_y;
	////printf("Data draw range: [%6.3f, %6.3f] - [%6.3f, %6.3f]\n",ulx, uly, lrx, lry);	// ***TEMP***
	int ulix = m_map.coordtocell(ulx)-2;								// go a little overside to avoid annoying edge problems
	int uliy = m_map.coordtocell(uly)+2;
	int lrix = m_map.coordtocell(lrx)+2;
	int lriy = m_map.coordtocell(lry)-2;
	////printf("Cell draw range: [%d,%d] - [%d,%d]\n", ulix, uliy, lrix, lriy);	// ***TEMP***
	//	Set up font for elev draw
	//	Use Helvetica Bold. Size varies with zoom
	const int k_basetextsize = 8;											// 8 points in smallest scale
	int textsize = k_basetextsize;
	char font_name[MAX_FONT_TAG];
	if (PfGenerateFontName( "Helvetica", 
                        0 ,
                        textsize, font_name ) != NULL ) 
	{
		PgSetFont( font_name );												// set this font, by name
	}
	PgSetTextColor(Pg_BLACK);												// elevations in black
	//	Draw all cells
	for (int ix = ulix; ix <= lrix; ix++)
	{	for (int iy = uliy; iy >= lriy; iy--)									// coords count up, but screen conts down
		{	if (m_map.cellonmap(ix,iy))										// if cell is on map
			{	DisplayCellData& cell = m_map.at(ix,iy);				// the cell
				////if (cell.m_framenumber < m_logframe.m_framenumber) continue;	// not seen yet
				if (cell.m_color == Pg_GREY) continue;					// skip background color
				float relev = cell.m_elev - m_logframe.m_z;			// elev relative to frame, for compactness
				drawcell(ix,iy,cell.m_color, relev);							// draw the cell
			}
		}
	}
}
//
//	Draw message in center of widget
//
static void drawmsg(PtWidget_t * widget, const char* s)
{	const int textHeight = 18;												// size of text, points
    PhPoint_t p = { 100,100 };											// ***TEMP*** should be center
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
//	updatewidgets  -- update widgets. Do not call from draw
//
void NavRead::updatewidgets()
{	LogItemFrame& item = m_logframe;
	//	Update position indicators on screen
	PtSetResource(ABW_position_x_box, Pt_ARG_NUMERIC_VALUE, &item.m_x, 0);
	PtSetResource(ABW_position_y_box, Pt_ARG_NUMERIC_VALUE, &item.m_y, 0);
	PtSetResource(ABW_position_z_box, Pt_ARG_NUMERIC_VALUE, &item.m_z, 0);
	PtSetResource(ABW_latitude_box, Pt_ARG_NUMERIC_VALUE, &item.m_latitude, 0);
	PtSetResource(ABW_longitude_box, Pt_ARG_NUMERIC_VALUE, &item.m_longitude, 0);
	double headingdeg = item.m_heading*(180/M_PI);				// heading in degrees, east is 0, ccw
	headingdeg = fmod(360+90-headingdeg, 360);					// heading in degrees, north is 0, cw
	double pitchdeg = (item.m_pitch)*(180/M_PI);						// pitch in degrees
	double rolldeg = (item.m_roll)*(180/M_PI);							// roll in degrees
	PtSetResource(ABW_heading_box, Pt_ARG_NUMERIC_VALUE, &headingdeg, 0);	
	PtSetResource(ABW_pitch_box, Pt_ARG_NUMERIC_VALUE, &pitchdeg, 0);	
	PtSetResource(ABW_roll_box, Pt_ARG_NUMERIC_VALUE, &rolldeg, 0);	
	short speed = short(item.m_speed + 0.5);							// speed in m/sec
	PtSetResource(ABW_speed_meter, Pt_ARG_METER_NEEDLE_POSITION, speed,0);
	//	Update log clock
	time_t timestampsecs = item.m_timestamp / 1000000000;	// convert time to seconds
	int fract = item.m_timestamp % 1000000000;						// nanoseconds
	struct tm timestamptm;
	localtime_r(&timestampsecs, &timestamptm);		// convert to local time
	char s[16];
	snprintf(s,sizeof(s),"%02d:%02d:%02d", timestamptm.tm_hour, timestamptm.tm_min, timestamptm.tm_sec);
	PtSetResource(ABW_time_box, Pt_ARG_TEXT_STRING, s,0);
	//	Update frame counter, from our count, not field in log file
	int framenum = m_logfile.getframenumber();					// not meaningful in online mode
	PtSetResource(ABW_frame_number_box, Pt_ARG_NUMERIC_VALUE, framenum, 0);
}
//
//	updategridparams  -- get grid params from scroll and zoom bars
//
void NavRead::updategridparams()
{	//	Read scroll and zoom bars
	long* zoomvalp;
	int stat = PtGetResource(ABW_zoom_scrollbar, Pt_ARG_GAUGE_VALUE, &zoomvalp,0);	// get zoom exponent
	assert(stat == 0);															// must get
	m_zoom = 1<<(*zoomvalp-1);										// power of 2 zooming
	long* hscrollp;
	stat = PtGetResource(ABW_horiz_scrollbar, Pt_ARG_GAUGE_VALUE, &hscrollp,0);		// get zoom exponent
	assert(stat == 0);															// must get
	long* vscrollp;
	stat = PtGetResource(ABW_vert_scrollbar, Pt_ARG_GAUGE_VALUE, &vscrollp,0);		// get zoom exponent
	assert(stat == 0);															// must get
	long hscroll = *hscrollp;												// get horizontal scroll bar value
	long vscroll = *vscrollp;												// get vertical scroll bar value
	hscroll *= m_zoom;														// adjust scaling of scrollbar for zooming
	vscroll *= m_zoom;
	PtWidget_t* widget = ABW_video_port;							// get area for drawing
	PhRect_t bounds;															// bounds of widget
	PtCalcCanvas(widget, &bounds);									// get bounds of widget
	m_scrolloffset.x = hscroll - (bounds.ul.x + bounds.lr.x)/2;	// update offset of origin
	m_scrolloffset.y = vscroll - (bounds.ul.y + bounds.lr.y)/2;	// to place origin at center before scrolling
}
//
//	drawvehicle -- draw the vehicle on the grid
//
void NavRead::drawvehicle(PtWidget_t* widget, int zoom, const PhPoint_t& offset, double headingdeg)
{
	if (!m_headerread) return;													// no vehicle info if no header
	//	Draw vehicle polygon
	PgSetStrokeColor(Pg_BLACK);
	PgSetFillColor( Pg_BLUE);
	//	Draw the vehicle using the vehicle dimensions specified in the header item.
	PhPoint_t pos = {0,0}; 
	PhPoint_t vehoutline[k_vehoutlinen];
	mat3 rot(rotation2D(vec2(0,0),headingdeg));						// rotate around origin
	for (int i=0; i<k_vehoutlinen; i++)
	{	vec2 pt(k_vehoutline[i][0]*m_logheader.m_vehlength-m_logheader.m_vehcenteroffset,
		k_vehoutline[i][1]*m_logheader.m_vehwidth);	// scale point from outline
		pt = rot * pt;																		// rotate
		convertmappt(vehoutline[i],pt[0],pt[1]);							// convert to screen coords
	}
	PgDrawPolygon(vehoutline, k_vehoutlinen, &pos, Pg_DRAW_FILL_STROKE |  Pg_CLOSED );	// draw it
}
//
//	drawcross -- draw one horizontal and one vertical line as part of a grid
//
//	Returns true if any drawing possible
//
bool NavRead::drawcross(const PhRect_t& bounds, double x, double y)
{
		PhPoint_t p0,h1, h2,v1,v2;											// points of line
		convertmappt(p0, x, y);											// center of the intersection of horizontal and vertical
		bool inxbounds = (p0.x >= bounds.ul.x && p0.x <= bounds.lr.x);	// check bounds
		bool inybounds = (p0.y >= bounds.ul.y && p0.y <= bounds.lr.y);
		if (!(inxbounds || inybounds)) return(false);				// if outside BOTH bounds, done, nothing to draw
		if (inybounds)
		{	h1.x = bounds.ul.x;
			h2.x = bounds.lr.x;
			h1.y = h2.y = p0.y;
			PgDrawLine(&h1,&h2);												// draw horizontal line
		}
		if (inxbounds)
		{
			v1.y = bounds.ul.y;
			v2.y = bounds.lr.y;
			v1.x = v2.x = p0.x;
			PgDrawLine(&v1,&v2);												// draw vertical line
		}
		return(true);																// success
}
//
//	drawgrid  -- draw the grid 
//
//	A line is drawn every meter
//
//	
void NavRead::drawgrid()
{	const int k_bigzoom = 4;												// less zoom than this, 10m grid
	PtWidget_t* widget = ABW_video_port;							// get area for drawing
	PhRect_t bounds;															// bounds of widget
	PtCalcCanvas(widget, &bounds);									// get bounds of widget
	PgSetStrokeColor(Pg_GREY);											// draw in grey
	PgSetStrokeWidth(1);													// return to normal
	//	Draw grid at integer (1m) intervals
	double gridinterval = 1.0;
	if (m_zoom < k_bigzoom) return;									// no grid if too small
	double xstart, ystart;
	convertmapwinpt(xstart,ystart,bounds.ul);					// get upper left in vehicle coordinates
	xstart += m_logframe.m_x;											// get upper left in world coordinates
	ystart += m_logframe.m_y;
	xstart = floor(gridinterval*xstart)/gridinterval;				// round down to grid unit
	ystart = floor(gridinterval*ystart)/gridinterval;															
	xstart -= m_logframe.m_x;											// get upper left in world coordinates
	ystart -= m_logframe.m_y;
	for (int i=0; ; i++)															// draw grid, one cross at a time
	{	
		bool good = drawcross(bounds,xstart+i, ystart-i);	
		if (!good) break;
	}
}
//
//	drawwideline  -- draw a wide line.
//
//	Photon has a line width limitation
//
static void drawwideline(const PhPoint_t& p0, const PhPoint_t& p1, float halfwidth)
{	if (halfwidth < 2) halfwidth = 2;							// apply minimum drawing width
	if (halfwidth > 8192) halfwidth = 8192;				// keep safely in short range after rotations
	PhPoint_t radii = { int(halfwidth), int(halfwidth) };	// radius of end circles
	PgDrawEllipse(&p0, &radii, Pg_DRAW_FILL);			// draw circle around p0
	PgDrawEllipse(&p1, &radii, Pg_DRAW_FILL);			// draw circle around p1
	float dx = p1.x - p0.x;
	float dy = p1.y - p0.y;
	float len = sqrt(dx*dx + dy*dy);							// length
	if (len < 0.001) return;											// done
	dx /= len;																// normalize
	dy /= -len;
	//	dy, dx is now a unit vector perpendicular to the original vector
	PhPoint_t quad[4] =												// construct four points
		{	p0, p0, p1, p1 };
	quad[0].x -= short(dy*halfwidth);						// build rectangle from points
	quad[0].y -= short(dx*halfwidth); 
	quad[1].x += short(dy*halfwidth);
	quad[1].y += short(dx*halfwidth); 
	quad[3].x -= short(dy*halfwidth);
	quad[3].y -= short(dx*halfwidth); 
	quad[2].x += short(dy*halfwidth);
	quad[2].y += short(dx*halfwidth);
	const PhPoint_t offset = {0,0};
	PgDrawPolygon(quad,4,&offset,Pg_CLOSED | Pg_DRAW_FILL); 		// draw polygon
}
//
//	drawwaypoints  -- draw the waypoints currently visible
//
//	Assumes waypoints are in ascending order, but not all need be present.
//
void NavRead::drawwaypoints()
{
	if (!m_showwaypoints) return;									// do not draw if turned off
	if (m_waypoints.size() < 2) return;							// nothing to do
	const PgColor_t Pg_LIGHTBLUE	=	PgRGB( 128, 128, 255 );	// color for waypoint areas
	PgSetFillColor(Pg_LIGHTBLUE);
	PgSetStrokeColor(Pg_LIGHTBLUE);
	for (size_t i=0; i<m_waypoints.size()-1; i++)				// for active waypoint pairs
	{	const LogItemWaypoint& item0 = m_waypoints[i];
		const LogItemWaypoint& item1 = m_waypoints[i+1];	// next waypoint
		if ((item1.m_serial != item0.m_serial + 1)				// if not a sequential pair
		&& (item1.m_serial != 1))										// special case for closed course; waypoint 1 appears at end
		{	continue;	}														// don't try to connect them
		////printf("Waypoint %d: [%8.2f %8.2f]\n", item.m_serial, item.m_rx, item.m_ry);	// ***TEMP***
		PhPoint_t center0, radii0;
		PhPoint_t center1;
		bool onscreen = false;
		onscreen = convertmappt(center0,item0.m_rx, item0.m_ry);		// convert to screen coords
		onscreen &= convertmappt(center1,item1.m_rx, item1.m_ry);		// convert to screen coords
		//	***NEED TO CHECK FOR WAYPOINT FAR OFFSCREEN*** this is possible, and requires corrections.
		drawwideline(center0, center1, m_zoom*float(item0.m_halfwidth));
	}
	//	Set up font for waypoint labels
	//	Use Helvetica Bold. Size varies with zoom
	const int k_basetextsize = 8;									// 8 points in smallest scale
	int textsize = k_basetextsize*m_zoom;
	char font_name[MAX_FONT_TAG];
	if (PfGenerateFontName( "Helvetica", 
                        PF_STYLE_BOLD ,
                        textsize, font_name ) != NULL ) 
   {
		PgSetFont( font_name );
	}
	PgSetTextColor(Pg_BLACK);										// waypoint IDs in black
	//	Display waypoint number on top of waypoint
	for (size_t i=0; i<m_waypoints.size(); i++)				// for all active waypoints
	{	const LogItemWaypoint& item0 = m_waypoints[i];
		bool onscreen = false;
		PhPoint_t center0;
		onscreen = convertmappt(center0,item0.m_rx, item0.m_ry);		// convert to screen coords
		if (!onscreen) continue;										// ignore if off screen
		//	Draw text name of waypoint.
		char s[10];
		snprintf(s,sizeof(s),"%d",item0.m_serial);					// edit waypoint number as string
		PgDrawText(s,strlen(s),&center0, Pg_TEXT_CENTER | Pg_TEXT_MIDDLE);	// display the text
		////printf("Drew \"%s\" at (%d,%d)\n", s, center0.x, center0.y);	// ***TEMP***
	}
}
//
//	drawmiscitems  -- draw misc. items stored while reading a frame
//
void NavRead::drawmiscitems()
{
	for (size_t i=0; i<m_lines.size(); i++)						// draw all lines
	{	drawlineitem(m_lines[i]);	}
	for (size_t i=0; i<m_circles.size(); i++)						// draw all circles
	{	drawcircleitem(m_circles[i]);	}
	for (size_t i=0; i<m_arcs.size(); i++)							// draw all arcs
	{	drawarcitem(m_arcs[i]);	}
	for (size_t i=0; i<m_arrows.size(); i++)						// draw all arrows
	{	drawarrowitem(m_arrows[i]);	}
}
//
//	map_port_draw -- draw video into indicated port
//
void NavRead::map_port_draw(PtWidget_t *widget, PhTile_t *damage)
{	PhRect_t rect;																// clipping rectangle
   	PtCalcCanvas( widget, &rect );										// get clipping rectangle for this widget
    PtClipAdd( widget, &rect );											// add to clip
	//	Get current scaling parameters
	updategridparams();														// ***TEMP***
	drawwaypoints();															// draw waypoints under everything
	drawgrid();																	// draw the grid
	double heading = (m_logframe.m_heading)*(180/M_PI);	// get heading in degrees
	drawdata(widget, m_zoom, m_scrolloffset);					// draw the data
	drawmiscitems();															// draw any misc, items.
	m_goalhistory.Draw(m_logframe.m_framenumber);		// draw thie history
	m_poshistory.Draw(m_logframe.m_framenumber);		// draw thie history
	drawvehicle(widget, m_zoom, m_scrolloffset, heading);// draw the vehicle on top
	PtClipRemove();															// end clip region
}
//
//	slew -- go to a new frame
//
void NavRead::slew(int frame)
{	if (frame <=0) m_map.clearmap();								// erase map on rewind
	m_logfile.seektoframe(frame);										// go to indicated frame
	////printf("Slew to frame %d\n",frame);								// ***TEMP***
	m_playing = false;														// not playing
	forceredraw();																// force a redraw
}
//
//	play  -- set/clear play mode
//
void NavRead::play(bool playing)
{	m_playing = playing;													// set play mode
	forceredraw();																// start drawing process
}
//
//	obstacles_toggle_changed  --  set obstacles visible (menu toggle)
//
void NavRead::obstacles_toggle_changed(bool on)
{	m_showobstacles = on;
	forceredraw();
}
//
//	waypoint_toggle_changed  --  set waypoints visible (menu toggle)
//
void NavRead::waypoint_toggle_changed(bool on)
{	m_showwaypoints = on;
	forceredraw();
}
//
//	redrawtimer  -- redraw timer, time to redraw if play mode
//
void NavRead::redrawtimer()
{	if (!m_playing) return;													// if not playing, nothing to do
	if (m_online)																	// if online mode
	{	// ***MORE***
	} else {
		bool good = m_logfile.seektoframe(getframenumber()+1);		// advance one frame
		if (!good) m_playing = false;										// stop play if falied seek
		forceredraw();															// force redraw anyway
	}
}
//	
//	Callbacks from controls
//
void NavRead::horiz_scrollbar_moved( PtWidget_t *widget)
{	forceredraw();	}
void NavRead:: vert_scrollbar_moved( PtWidget_t *widget)
{	forceredraw();	}
void NavRead::zoom_scrollbar_moved( PtWidget_t *widget)
{	forceredraw();	}
void NavRead::run_button_pushed( PtWidget_t *widget)
{	forceredraw();	}

//
//	zeroscrollbar  -- center a scrollbar
//
static void zeroscrollbar(PtWidget_t *widget)
{
	PtSetResource(widget,Pt_ARG_GAUGE_VALUE, 0, 0);	// center the scroll bar
}
//
//	center_button_pushed  -- moves scroll bars back to center
//
void NavRead::center_button_pushed( PtWidget_t *widget)
{
	zeroscrollbar(ABW_vert_scrollbar);							// center both scrollbars
	zeroscrollbar(ABW_horiz_scrollbar);
	forceredraw();															// and force a redraw
}

//
//	forceredraw -- force a redraw cycle
//
void NavRead::forceredraw()
{
	int frame = m_logfile.getframenumber();						// get position	
	readframe(frame);														// read desired frame
	updatewidgets();															// update widgets accordingly
	PtDamageWidget(ABW_video_port);							// force a redraw
}
//
//	Class MoveHistory  -- recent movement of the vehicle
//
//
//	clear  -- clear the history list
//
void MoveHistory::clear()							
{	m_history.clear();														// clear list
	m_historysize = 0;													// size is zero
}
//
//	add -- add frame to history
//
//	Frame number must be one greater than end of list for add
//
void MoveHistory::add(int framenumber, double x, double y)
{
	if (!m_history.empty())												// if nonempty list
	{	const MoveHistoryItem& lastitem = m_history.back();	// get last item
		if (framenumber != (lastitem.m_framenumber+1))	// if not good case
		{	if ((framenumber < (lastitem.m_framenumber+1))	// if already stored
			&& (framenumber > m_history.front().m_framenumber))
			{	return;	}														// already have this one
			clear();																// in new region, start over
		}
	}
	MoveHistoryItem item;
	item.m_framenumber = framenumber;
	item.m_x = x;
	item.m_y = y;
	m_history.push_back(item);										// add to list
	if (m_historysize <= m_historymax)							// if not too many
	{	m_historysize++;													// add one
	} else {																	// if too many
		assert(!m_history.empty());									// must not be empty
		m_history.pop_front();											// remove oldest
	}
}
//
//	Draw -- draw entire history, up to frame number specified.
//
void MoveHistory::Draw(int maxframenumber)
{	if (m_history.empty()) return;									// nothing to do
	PgSetStrokeColor(m_color);										// set drawing color for history line
	std::list<MoveHistoryItem>::const_iterator prev = 0;	// prev item
	bool first = true;
	for (std::list<MoveHistoryItem>::const_iterator p = m_history.begin(); p != m_history.end(); p++)
	{	if (!first)
		{	if (p->m_framenumber > maxframenumber) break;		// reached current frame, stop
			DrawSegment(*prev, *p);												// draw one line segment
		}
		first = false;															// no longer first time through
		prev = p;																// prev point for drawing
	}
}
//
//	DrawSegment  -- draw one line segment of history
//
void MoveHistory::DrawSegment(const MoveHistoryItem& p1, const MoveHistoryItem& p2)
{	double vehx = m_owner.getLogItemFrame().m_x;				// get vehicle position
	double vehy = m_owner.getLogItemFrame().m_y;
	double x1 = p1.m_x - vehx;										// convert absolute position to vehicle-relative for draw
	double y1 = p1.m_y - vehy;
	double x2 = p2.m_x - vehx;
	double y2 = p2.m_y - vehy;
	PhPoint_t pt1, pt2;
	if (!m_owner.convertmappt(pt1, x1, y1)) return;						// ignore if way offscreen
	if (!m_owner.convertmappt(pt2, x2, y2)) return;
	if (pt1.x == pt2.x  && pt1.y == pt2.y) return;			// if zero length, ignore
	PgDrawLine(&pt1, &pt2);											// draw the line
}

