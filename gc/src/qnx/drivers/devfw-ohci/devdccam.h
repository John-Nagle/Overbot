//
//	devdccam.h  --  device entry generator for hot-pluggable devices
//
//	Part of
//	devfw-ohci  -- FireWire (IEEE-1394) driver for OHCI-compatible devices
//
//	John Nagle
//	Animats
//	
//	January. 2003
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
#ifndef DEVDCCAM_H
#define DEVDCCAM_H
#include "mindreadyforcpp.h"								// WORKAROUND for sizeof(bool) problem
#include <limits.h>
#include <string>
#include <memory>
#include "mutexlock.h"
#include "businfo.h"
#include "devdccamdefs.h"
#include "devfw-camera.h"
//
//	Constants that should be in dc1394_control.h, but aren't.
//
//	If either of these registers is nonzero, the camera is in an invalid state.
//
#define REG_CAMERA_FEATURE_CONTROL_ERROR_STATUS_HI     0x640U 
#define REG_CAMERA_FEATURE_CONTROL_ERROR_STATUS_LO     0x644U 
//
//	Base addresses for camera format and mode capabilities
//
#define REG_CAMERA_V_MODE_INQ_0 											0x180U
#define REG_CAMERA_V_RATE_INQ_0_0											0x200U
//
//
//	Properties of camera modes
//
//	CameraColorMode  --  pixel format
//	
enum CameraColorMode { YUV444, YUV422, YUV411, RGB444, MONO1, MONO2  };
//
//	Properties of a camera mode
//
struct CameraModeProps {
	uint32_t						m_format;						// format (IEEE std)
	uint32_t						m_mode;						// mode (IEEE std)
	uint32_t						m_width;						// in pixels
	uint32_t						m_height;						// in pixels
	CameraColorMode		m_colormode;				// pixel format
};
//
//	Constant table lookup functions
//
extern const CameraModeProps* CameraModeSearch(uint32_t mode);
extern const size_t CameraModeFrameBytes(const CameraModeProps& props);
extern const uint32_t CameraModeRelMode(const CameraModeProps& props);
//
//	class Framequeuetem  -- one frame from a camera
//
class Framequeueitem {
public:
	dbuf_t*		m_buf;												// buffer chain, representing one frame
	timespec	m_timestamp;									// time at which frame was received
	int				m_status;											// associated status of read
public:
	Framequeueitem()
	: m_buf(0) {}													// no initial link
};
//
//	class Framequeue  -- incoming frame queue
//
class Framequeue {
private:
	ost::Mutex m_getlock;										// lock get side of queue.
	DbufPool  m_rxcopypool;									// copy (of buffer pointers, not actual data) of data in input queue. 
	ost::BoundedBuffer<Framequeueitem,4>	 m_queue;	// small input queue
	Framequeueitem m_current;								// the most recent framequeue item
public:
	Framequeue();
	~Framequeue();
	int reserve(uint32_t bufcount);							// number of buffers to reserve in queue
	void 	putcopy(dbuf_t* buffer, int stat);				// put copy into queue
	int get(float maxwait);										// get next buffer
	int getdata(size_t offset, uint8_t data[], size_t maxsize);
	void purge();													// clear queue
	size_t size();														// get size of data available
	void clear();														// clear current buffer
	int gettimestamp(struct timespec& timestamp)
	{	 if (m_current.m_buf) { timestamp = m_current.m_timestamp; return(EOK); } // success
		return(ESTALE);											// fails, no current buffer for timestamp
	}
	int getstatus()
	{	if (m_current.m_buf) { return(m_current.m_status); } // success
		return(EAGAIN);												// no data, try later
	}
};
//
//	class Devdccam  --  driver for a digital camera
//
class Devdccam: public Busentry {
private:																	// data buffer
	Framequeue m_framequeue;							// queue of incoming frames
	const CameraModeProps*  m_modeprops;		// current camera mode properties, null if unknown
	uint32_t m_framerate;										// selected frame rate
	bool	m_singleshot;											// true if in single-shot mode
	int m_isochannel;												// isochronous channel (-1 if unallocated)
	bool m_active;													// true if camera should be active
public:
	//	Initialization
	Devdccam(Businfo& bus, 
		uint32_t initialnodenumber, uint32_t vendor, uint32_t hiid, uint32_t lowid);
	~Devdccam();													// destructor
	int setidentity();												// get device name from device ROM
	//	Usual I/O operations
	int io_open(resmgr_context_t* ctp, io_open_t *msg, iofunc_attr_t *attr, void *extra);
	int io_close_ocb(resmgr_context_t *ctp, void* reserved, RESMGR_OCB_T* ocb);
	int io_read(resmgr_context_t *ctp, io_read_t* msg, RESMGR_OCB_T* ocb);
	int io_write(resmgr_context_t *ctp, io_write_t* msg, RESMGR_OCB_T* ocb);
	int io_devctl(resmgr_context_t *ctp, io_devctl_t* msg, RESMGR_OCB_T* ocb);
	//	Incoming isochronous events
	void isorcvpkt(const llaIsoHdr_t& hdr, dbuf_t*& buffer);
	void isorcvblock(uint32_t channel, dbuf_t* buffer, unsigned status);
	void isorcvpurge();
	//	Bus reset events
	void busresetbegin();										// a bus reset has started; stop everything
	void busresetdone();										// a bus reset has completed; restart what was stopped
	//	Access
	const char* getnameprefix() const {	return("dc");	}				// prefix names with this
private:
	int getcamerafeature(dc1394_feature_info& feature);		// get camera feature info
	int setcamerafeature(dc1394_feature_info& feature);		// set camera feature info
	int getcameracontrolregister(uint64_t offset, uint32_t& value);
	int setcameracontrolregister(uint64_t offset, uint32_t value);
	int getcameraromregister(uint64_t offset, uint32_t& value);
	int gettimestamp(struct timespec& timestamp);
	int takepicture();
	int checkstatus();
	int setvideomode(dc1394_video_mode_t& modeinfo);
	int getvideomode(dc1394_video_mode_t& modeinfo);
	int checkmodeprops(const CameraModeProps& props, uint32_t framerate);
	int getcameraident(uint64_t& euid, string& vendor, string& model);
	int appendstringfromrom(uint32_t offset, int len, string& s);

	int startcamera();												// start camera, using current params
	int setupcamera();											// set up, but do not start yet.
	int resetcamera();												// reset camera, leaving it off
};
#endif // DEVDCCAM_H