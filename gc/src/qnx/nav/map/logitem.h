//
//	LogItem.h  -- logging information for NewSteer / Overbot maps
//
//	John Nagle
//	Team Overbot
//	January, 2004
//
//
//
//	Copyright 2005 by John Nagle
//	999 Woodland Avenue
//	Menlo Park, CA  94025
//
//	This program is free software; you can redistribute it and/or modify
//	it under the terms of the GNU General Public License as published by
//	the Free Software Foundation; either version 2 of the License, or
//	(at your option) any later version.

//	This program is distributed in the hope that it will be useful,
//	but WITHOUT ANY WARRANTY; without even the implied warranty of
//	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//	GNU General Public License for more details.

//	You should have received a copy of the GNU General Public License
//	along with this program; if not, write to the Free Software
//	Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//
#ifndef LOGITEM_H
#define LOGITEM_H
#include <string>
#include <inttypes.h>
#include <stdio.h>
#include <assert.h>
#include "mapcell.h"
//
//
const int k_log_version = 2;				// log version, for comparison with reader programs
//
typedef uint8_t LogColor;					// ***TEMP*** will become an enum
//
//	Logging information is represented as sequences of LogItem.
//	These items are variable length, prefaced by a length byte
//	and a type byte.
//
//
//	struct LogItemHeader - one of these begins a log file
//
//	Contains primarily scaling parameters.
//
struct LogItemHeader {
	int m_logversion;							// version of the log file format
	float m_vehlength;						// length of vehicle
	float m_vehwidth;							// width of vehicle
	float m_vehcenteroffset;				// offset of vehicle rectangle from vehicle center (>0 means forward of center)
	double m_cellspermeter;				// number of grid cells per linear meter. Preferably an integer.
};
//
//	struct LogItemFrame - one of these begins each "frame", or steering cycle
//
struct LogItemFrame	
{	int m_framenumber;						// frame number since boot
	uint64_t m_timestamp;					// nanoseconds since epoch
	double m_x;									// current vehicle position (XY)
	double m_y;									// all positions for this frame are relative to this
	double m_latitude;						// current vehicle position (lat, long)
	double m_longitude;
	double m_heading;						// heading (radians	)
	double m_speed;							// speed in m/s, signed
	int		m_ix;										// location of vehicle in map, cell coords
	int		m_iy;										// location of vehicle in map, cell coords
	double	m_z;									// absolute elevation (up) for this frame (added at end for expansion)
	float	m_pitch;								// pitch, in radians
	float	m_roll;									// roll, in radians
};
//
//	struct LogItemLine -- represents a line to be drawn
//
struct LogItemLine
{	float m_x0;									// relative to frame
	float m_y0;
	float m_x1;
	float m_y1;
	LogColor m_color;							// "color", actually the meaning
};
//
//	struct LogItemTriangle -- represents a triangle to be drawn
//
struct LogItemTriangle
{	float m_x0;									// relative to frame
	float m_y0;
	float m_x1;
	float m_y1;
	float m_x2;
	float m_y2;
	bool m_fill;										// fill this triangle
	LogColor m_color;							// "color", actually the meaning
};
//
//	struct LogItemQuad -- represents a quad  to be drawn
//
struct LogItemQuad
{	float m_x0;									// relative to frame
	float m_y0;
	float m_x1;
	float m_y1;
	float m_x2;
	float m_y2;
	float m_x3;
	float m_y3;
	bool m_fill;										// fill this quad
	LogColor m_color;							// "color", actually the meaning
};
//
//	struct LogItemCircle -- represents a circle to be drawn
//
struct LogItemCircle
{	float m_rx;										// relative to frame
	float m_ry;
	float m_r;
	bool m_fill;										// fill the circle
	LogColor m_color;							// "color"
};
//
//	struct LogItemArc -- represents an arc to be drawn
//
struct LogItemArc
{	float m_r0x;									// relative to frame
	float m_r0y;
	float m_r1x;									// relative to frame
	float m_r1y;
	float m_curvature;							// 1/radius
	LogColor m_color;							// "color"
};
//
//	struct LogItemCell -- represents an update to one cell
//
struct LogItemCell
{	
	CellData::CellType	m_type : 3;		// new cell type
    bool m_valid : 1;							// there is valid data in this cell
	int 	m_irx: 12;								// position relative to origin for this frame
	int 	m_iry: 12;
	int		m_rz: 12;								// elevation relative to origin, in cm.
};
//
//	LogItemWaypoint -- waypoint of current interest
//
struct LogItemWaypoint {
	int 	m_serial;								// waypoint number
	float m_rx;										// position relative to frame
	float m_ry;
	float m_halfwidth;							// width at end
	float m_speedlimit;						// m/sec
};
//
//	LogItemArrow -- an arrow
//
struct LogItemArrow {
	float m_rx;										// relative to frame
	float m_ry;
	float m_dirx;									// direction, unit vector
	float m_diry;
	float m_length;								// arrow length
	LogColor m_color;							// "color"
};
struct LogItemFrameEnd {
//	Deliberately empty
};
//
//	struct LogItem
//
struct LogItem
{	//	Frame types - start at 99, so junk data will be rejected	
	enum Type_e { item_frameend = 99, item_frame, item_line, item_circle, item_cell, 
		item_scanline, item_header, item_triangle, item_quad, item_waypoint, item_arc,
		item_arrow };
	union {
		LogItemFrame m_frame;
		LogItemLine	m_line;
		LogItemCell m_cell;
		LogItemHeader m_header;
		LogItemWaypoint m_waypoint;
		LogItemCircle m_circle;										// circle type
		LogItemArc m_arc;											// an arc
		LogItemArrow m_arrow;									// an arrow
		LogItemFrameEnd m_frameend;						// end of frame marker, no content
	};
};
//
//	LogMarshall -- represents one block of log entries
//
//	This does marshalling and unmarshalling.
//
//	Subclass for file or message I/O 
//
class LogMarshall {
	bool m_verbose;										// true if debug
public:
	LogMarshall(): m_verbose(false) {}
	void setverbose(bool verbose) { m_verbose = verbose; }
	bool isverbose() const { return(m_verbose); }
	bool getitem(LogItem& item, LogItem::Type_e& type);	// get next item
	void add(const LogItemFrame& item)
	{	additem(item,  LogItem::item_frame); }
	void add(const LogItemLine& item)
	{	additem(item,  LogItem::item_line); }
	void add(const LogItemTriangle& item)
	{	additem(item,  LogItem::item_triangle); }
	void add(const LogItemQuad& item)
	{	additem(item,  LogItem::item_quad); }
	void add(const LogItemCircle& item)
	{	additem(item, LogItem::item_circle); }
	void add(const LogItemArc& item)
	{	additem(item, LogItem::item_arc); }
	void add(const LogItemArrow& item)
	{	additem(item, LogItem::item_arrow); }
	void add(const LogItemCell& item)
	{	additem(item, LogItem::item_cell); }
	void add(const LogItemWaypoint& item)
	{	additem(item, LogItem::item_waypoint); }
	void add(const LogItemHeader& item)
	{	additem(item, LogItem::item_header); }
	void add(const LogItemFrameEnd& item)
	{	additem(item, LogItem::item_frameend); }
	virtual bool valid()  = 0;							// true if no errors yet
	virtual bool iswriteable() const = 0;			// true if open for writing
	virtual void flush() {}								// flush output
private:
	template<class T>void additem(const T& item, LogItem::Type_e type);			// add a filled-in item
protected:
	virtual void put(uint8_t byte)					// write one byte to output
	{	write(&byte, sizeof(byte));	}				// write N bytes to output
	virtual void write(const uint8_t buf[], size_t size) = 0;	// write 
	virtual int get(uint8_t& byte)						// get one byte from input
	{	return(read(&byte,sizeof(byte))); }
	virtual int read(uint8_t buf[], size_t size) = 0;	// get N bytes from input
};
//
//	class LogFile -- a file of log items
//
class LogFile: public LogMarshall {
private:
	FILE* m_fd;												// file descriptor
	bool m_writeable;										// true if writeable
public:
	LogFile(): m_fd(0), m_writeable(false) {}	// constructor
	virtual ~LogFile() { close(); }
	int open(const char* filename, bool writing);
	void close();
	int getpos(fpos_t& pos)							// get position in file
	{	return(::fgetpos(m_fd,&pos)); }
	int setpos(const fpos_t& pos)					// set position in file
	{	return(::fsetpos(m_fd,&pos)); }
	bool iswriteable() const { return(m_writeable); } // true if nonnull
	bool valid();												// true if no errors yet
	bool isopen() const 
	{	return(m_fd != NULL); }						// true if open
	void flush();												// flush output
protected:
	void write(const uint8_t buf[], size_t size);//write requested bytes
	int read(uint8_t buf[], size_t size);				// read requested bytes

};
//
//	class LogMsg  -- log messages as a string, for QNX messaging
//
const size_t k_log_msg_default_size = 3000;			// 3K msg assumed
class LogMsg: public LogMarshall {
private:
	std::basic_string<uint8_t> m_block;			// the block
	size_t m_readpos;										// read position
	bool m_writeable;										// set writeable
protected:
	void write(const uint8_t buf[], size_t size)
	{	m_block.append(buf,size); }
	int read(uint8_t buf[], size_t size)
	{	int stat = m_block.copy(buf,size,m_readpos);
		m_readpos += size;
		return(stat);
	}
public:
	bool valid()												// only possible error is out of mem
	{	return(true); }
	bool iswriteable() const 							// true if writeable
	{	return(m_writeable); }
	void enablewrite(bool enb)						// enable and prepare for writing
	{	clear(); m_writeable = enb;
		if (enb)
		{	reserve(k_log_msg_default_size);	}	
	}
	void reserve(size_t size)							// preallocate to save time
	{	m_block.reserve(size); }
	LogMsg(): m_readpos(0), m_writeable(false) 
	{	 }		// constructor
	virtual ~LogMsg() {}								// virtual destructor
	void clear()												// clear to empty
	{	m_block.erase(m_block.begin(),m_block.end()); m_readpos = 0; }
	//	Access
	size_t getsize() const { return(m_block.size()); }
	bool getdata() const { return(m_block.data()); }
	//	Insert received message
	void assign(const uint8_t msg[], size_t size)
	{	m_block.assign(msg,size);
		m_readpos = 0;
	}
};
//
//	Implementation
//
#include "logitem_impl.h"

#endif // LOGITEM_H