//
//	navread.h  --  support for reading from a FireWire nav using our driver
//
//	John Nagle
//	Team Overbot
//	October, 2003
//
#ifndef NAVREAD_H
#define NAVREAD_H

//	Need to get phAb to search more directories

#include "ablibs.h"
#include <memory>
#include <vector>
#include <list>
#include <string>
#include "messaging.h"
#include "algebra3aux.h"
#include "logitem.h"
#include <scrollablemap.h>
//
//	Globals
//
extern std::string g_current_directory;													// the current directory
//
//	Constants
//
const int k_fastforwardframes = 10;														// fast forward does this many frames

//
//	struct DisplayCellData  -- display data for one cell
//
struct DisplayCellData {
	PgColor_t	m_color;
	int m_framenumber;																			// frame on which we updated this
	float m_elev;																						// elevation
public:
	PgColor_t getcolor() const { return(m_color); }
	void clear() { m_color = Pg_GREY; m_framenumber = 0; m_elev = 0;}
}; 
class Dummy 																						// dummy, so we can instantiate ScrollableMap as child
{};
//
//	TerrainMap -- the map, on the player side
//
class TerrainMap: public ScrollableMap<DisplayCellData,Dummy>
{
public:
	TerrainMap(int dim, double cellspermeter)
	:	ScrollableMap<DisplayCellData,Dummy>(dim, cellspermeter)
	{}
};
//
//	MoveHistoryItem  -- where were we at frame N?
//
struct MoveHistoryItem {
	int m_framenumber;
	double m_x;
	double m_y;
};
class NavRead;												// forward
//
//	class MoveHistory  -- recent movement of the vehicle
//
class MoveHistory {
private:
	NavRead& m_owner;								// owning object
	size_t m_historymax;								// maximum history to store
	size_t m_historysize;									// currently stored
	std::list<MoveHistoryItem> m_history;		// the history
	PgColor_t m_color;										// draw color
public:
	MoveHistory(NavRead& owner, size_t historymax, PgColor_t color = Pg_BLACK)
	: m_owner(owner), m_historysize(0), m_historymax(historymax), m_color(color)
	{}
	void clear();												// clear the history
	void add(int framenumber, double x, double y);	// add to history
	void Draw(int maxframenumber);				// draw, up to this frame number
	PgColor_t getcolor() const  { return(m_color); }
private:
	void DrawSegment(const MoveHistoryItem& p1, const MoveHistoryItem& p2);
};

//	
//	class LogFileReader --  log file and related info
//
class LogFileReader {
private:
	LogFile m_logfile;												// the logfile
	int		m_framenumber;										// current frame number
	std::vector<fpos_t> m_framepositions;				// starting position for each frame, for slew
public:
	LogFileReader(): m_framenumber(0) {}			// constructor
	~LogFileReader() { close(); }							// destructor
	int getframenumber() const { return(m_framenumber); }	// access
	bool seektoframe(int framenumber);				// go to frame N, true if success
	bool getitem(LogItem& item, LogItem::Type_e& type);		// read next frame from file, true if success	
	int open(const char* filename);						// open log file 
	void close();														// close log file
	bool valid() { return(m_logfile.valid()); }			// true if log file valid and readable
};
//
//	class NavRead  --  read from the nav server
//
class NavRead {
private:
	TerrainMap m_map;											// the cell map
	////MsgClientPort m_navserver;							// define client port, no timeout
	LogFileReader m_logfile;									// the logfile
	LogItemFrame m_logframe;								// frame info for current frame
	LogItemHeader m_logheader;							// log header
	MoveHistory m_poshistory;								// recent movement history
	MoveHistory m_goalhistory;								// recent goal history
	std::vector<LogItemWaypoint> m_waypoints;	// waypoints in current frame
	std::vector<LogItemCircle> m_circles;				// circle items in current frame
	std::vector<LogItemArc> m_arcs;						// arc items in current frame
	std::vector<LogItemLine> m_lines;					// line items in current frame
	std::vector<LogItemArrow> m_arrows;				// arrow items in current frame
	bool m_headerread;											// saw a header item
	bool m_online;													// online mode - running under watchdog
	bool m_playing;												// true if in play mode
	bool m_showobstacles;									// show obstacles if set
	bool m_showwaypoints;									// show waypoints if set
	enum read_mode_e { mode_closed, mode_paused, mode_step_forward, mode_step_reverse, mode_run, mode_ff, mode_fr };
	read_mode_e m_readmode;								// current read mode (VCR-type model)
	fpos_t m_framestart;											// start position of frame, after LogItemFrame, for redraws
	int m_zoom;														// zoom factor
	PhPoint_t m_scrolloffset;									// scrollbar control 0,0 means vehicle at center of screen
	PhImage_t* 	m_image;										// working image for display
private:
public:
	int reset();
	void horiz_scrollbar_moved( PtWidget_t *widget);
	void run_button_pushed( PtWidget_t *widget);
	void vert_scrollbar_moved( PtWidget_t *widget);
	void zoom_scrollbar_moved( PtWidget_t *widget);
	void center_button_pushed( PtWidget_t *widget);
	void map_port_draw( PtWidget_t *widget, PhTile_t *damage );	// redraw
	int open(const char* filename);						// open log file 
	void close();														// close log file
	void slew(int framenumber);								// slew to frame and redraw
	void play(bool playing);									// set play/stop mode
	void obstacles_toggle_changed(bool on);		// set obstacles visible
	void waypoint_toggle_changed(bool on);			// set waypoints visible
	bool showobstacles() const { return(m_showobstacles); }
	bool showwaypoints() const { return(m_showwaypoints); }
	void redrawtimer();											// redraw timer for frame
	int getframenumber() const { return(m_logfile.getframenumber()); }
	const LogItemFrame& getLogItemFrame() const { return(m_logframe); }
	NavRead();														// constructor
	~NavRead();														// destructor
	bool convertmappt(PhPoint_t& pt, float x, float y);	// convert logged point to map window coordinates
private:
	int setup();														// initial setup
	void updategridparams();									// get zoom/scroll info
	void convertmapwinpt(double& x, double& y, const PhPoint_t& pt);
	void convertmapcolor(PgColor_t& mapcolor, LogColor logcolor);	// convert log color to Photon color
	void zoomdata(const std::vector<CellData>& obstacles, const PhDim_t& datadim, PhImage_t* image, const PhPoint_t& offset, int zoom);
	void drawframe();												// draw a frame
	void updatewidgets();										// update all widgets based on current frame info
	void drawdata(PtWidget_t* widget, int zoom, const PhPoint_t& scrolloffset);
	void drawvehicle(PtWidget_t* widget, int zoom, const PhPoint_t& scrolloffset, double headingdeg);
	bool drawcross(const PhRect_t& bounds, double x, double y);
	void 	drawgrid();												 // draw the grid
	void drawwaypoints();										// draw the waypoints
	void drawcell(int ix, int iy, const PgColor_t color, float elev);	// draw cell at indicated relative coordinates
	void drawlineitem(const LogItemLine& item);
	void drawcircleitem(const LogItemCircle& item);	// handle a draw-circle item
	void drawarcitem(const LogItemArc& item);	// handle a draw-arc item
	void drawarrowitem(const LogItemArrow& item);	// handle a draw-arrow item
	void drawframeitem(const LogItemFrame& item);
	PgColor_t getcolor(const CellData& cell);			// color that best represents a cell
	PgColor_t getcolor(const CellData::CellType type);
	void processwaypointitem(const LogItemWaypoint& item);	// handle a waypoint item
	void 	processheaderitem(const LogItemHeader& item);
	void processframeitem(const LogItemFrame& item); // handle a frame item
	void processcellitem(const LogItemCell& item);// handle a cell item
	void processlineitem(const LogItemLine& item);	// handle a draw-circle item
	void processcircleitem(const LogItemCircle& item);	// handle a draw-circle item
	void processarcitem(const LogItemArc& item);	// handle a draw-arc item
	void processarrowitem(const LogItemArrow& item);	// handle a draw-arrow item
#ifdef OBSOLETE
	void processlineitem(const LogItemLine& item); // handle a line item
	void processtriangleitem(const LogItemTriangle& item); // handle a line item
	void processquaditem(const LogItemQuad& item); // handle a line item
#endif // OBSOLETE
	void processitem(const LogItem& item, LogItem::Type_e type);		// process one item
	bool readframe();												// read next frame
	bool readframe(int frame);								// read one frame from file or msg, update widgets
	void drawmiscitems();
	void forceredraw();											// something has changed, redraw video port
};
#endif // NAVREAD_H