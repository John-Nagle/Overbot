//
//	devdccam.cpp  -- FireWire (IEEE-1394) driver for OHCI-compatible devices
//
//	Driver for DC-type cameras
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
#include <sys/neutrino.h>
#include <limits.h>
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <errno.h>
#include <devctl.h>
#include <valarray>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <sys/resmgr.h>
#include <string.h>
#include <assert.h>
#include "businfo.h"
#include "devdccam.h"
#include "dc1394constants.h"
#include "devfw-camera.h"
#include "utilities.h"
//
//	Constants
//
const float readtimeout = 0.2;												// wait no more than 200ms for frame
//
//	Class Devdccam implementation
//
//	Constructor
//
Devdccam::Devdccam(Businfo& bus,
		uint32_t initialnodenumber, uint32_t vendor, uint32_t hiid, uint32_t lowid)
		: Busentry(bus, initialnodenumber, vendor, hiid, lowid),
		m_modeprops(0),															// don't know camera mode yet
		m_framerate(0),															// no framerate yet
		m_singleshot(false),														// not in single shot mode
		m_isochannel(-1),															// no isochronous channel assigned
		m_active(false)																// camera inactive
{
	string s;
	builddefaultname(s);															// build default name
	setname(s);																		// overridden later if we can read the name string
}
//
//	Destructor
//
Devdccam::~Devdccam()
{
	getbus().getbuschannels().releasechannel(*this);			// end listening on any isochronous channel
}
//
//	io_open  -- open the device
//
//	***NEED EXCLUSIVE USE CHECK***
//	***NEED TO RESET DEVICE MODES***
//
int Devdccam::io_open(resmgr_context_t* ctp, io_open_t *msg, iofunc_attr_t *attr, void *extra)
{	int stat = Deventry::io_open(ctp,msg,attr,extra);				// handle at higher level
	if (stat != EOK) return(stat);												// open failed
	//	Successful open - do any necessary open processing
	////m_cameramode = Cameramode();									// reset camera mode to defaults
	return(stat);
}
//
//	io_close  -- final close of this open descriptor
//
int Devdccam::io_close_ocb(resmgr_context_t *ctp, void* reserved, RESMGR_OCB_T* ocb)
{
	resetcamera();																// reset camera, release isochronous resources
	return(Deventry::io_close_ocb(ctp,reserved,ocb));
}
//
//	io_read  -- normal read
//
//	Starts the camera in streaming mode if needed.
//	Waits for a frame to come in. If no frame appears within
//	a short period, a timeout error is returned.
//
//	Once a full frame is read, zero bytes are returned as an EOF indicator.
//	Further reads are allowed and will start reading the next frame.
//
//	Single-frame mode ("takepicture") is not currently supported.
//
//	With some work, we could remove an extra copy from input.
//
int Devdccam::io_read(resmgr_context_t *ctp, io_read_t* msg, RESMGR_OCB_T* ocb)
{
	int stat = iofunc_read_verify(ctp,msg, ocb,NULL);			// check that we have read access
	if (stat != EOK) return(stat);											// if not, fails
	if (!getbus().isup()) return(EAGAIN);								// if bus reset in progress, try again later
	//	Handle an XTYPE override if present
	int xtype = msg->i.xtype & _IO_XTYPE_MASK;				// extract xtype bit
	if (xtype != _IO_XTYPE_NONE)										// if an xtype is specified
	{	return(ENOSYS);	}													// fail, not meaningful for a sequential device.
	int off = ocb->offset;													// offset into I/O data 
	//	
	//	Take the picture
	//
	if (off == 0)																	// if first read of a frame
	{	int stat = startcamera();											// turn on camera
		if (stat != EOK) return(stat);										// fails
		stat = m_framequeue.get(readtimeout);					// get a new frame
		if (stat != EOK) return(stat);										// fails
		// 	Frame read successfully
	}
	ocb->attr->nbytes = m_framequeue.size();					// get size of data in buffer	
	int nleft = ocb->attr->nbytes - off;								// assume amount of data available is in ocb->attr
	int nbytes = min(nleft, msg->i.nbytes);							// number of bytes to transfer
	uint8_t buf[8192];															// ***TEMP*** for extra copy to simplify for test
	nbytes = min(nbytes,int(sizeof(buf)));							// must fit
	nbytes = m_framequeue.getdata(off,buf,nbytes);			// xfer indicated size
	//////logprintf("Read transferred %d bytes of %d available starting from offset %d.\n",nbytes,m_framequeue.size(),off);		// ***TEMP***
	//	Return data to caller
	if (nbytes)
	{	//	***INEFFICIENT*** need to use IOVs to avoid extra copy
		MsgReply(ctp->rcvid, nbytes, buf, nbytes);	// return data from frame buffer
		//	Update last time read
		ocb->attr->flags |= IOFUNC_ATTR_ATIME | IOFUNC_ATTR_DIRTY_TIME;
		//	Advance "seek pointer"
		ocb->offset += nbytes;											// advance into buffer
	}	else {
		if (msg->i.nbytes > 0)												// if trying to read a nonzero number of bytes, but got none
		{	m_framequeue.clear();											// done, ready to accept a new frame
			ocb->offset = 0;													// at beginnng of read again		
		}		
		MsgReply(ctp->rcvid, EOK, NULL, 0);							// not returning data, just unblock client
	}
	return(_RESMGR_NOREPLY);
}
//
//	io_write  -- normal write
//
int Devdccam::io_write(resmgr_context_t *ctp, io_write_t* msg, RESMGR_OCB_T* ocb)
{
	return(EIO);																	// ***UNIMPLEMENTED***
}
//
//	io_devctl  -- normal device control
//
//	***NEED READ PERMISSION CHECK***
//
int Devdccam::io_devctl(resmgr_context_t *ctp, io_devctl_t* msg, RESMGR_OCB_T* ocb)
{
	if (!getbus().isup()) return(EAGAIN);								// if bus reset in progress, try again later
	uint16_t requesttype = msg->i.type;							// get request type
	uint32_t dcmd = msg->i.dcmd;										// get request command
	logprintf("  Devctl request on \"%s\": request %02x, dcmd %02x\n",getname().c_str(),requesttype,dcmd);	// ***TEMP***
	uint32_t stat = iofunc_devctl_default(ctp,msg,ocb);		// try default action for standard devctl functions
	if (stat != _RESMGR_DEFAULT)										// if worked
	{	return(stat);	}															// done
	////stat = iofunc_read_verify(ctp,msg, ocb,NULL);				// check that we have read access
	////if (stat != EOK) return(stat);											// if not, fails
	//	Not a POSIX standard function; must be one of ours.
	switch (msg->i.dcmd)	{												// fan out on command
	case DCMD_MEDIA_DEVFW_CAMERA_FEATURE_READ:	// feature read
	{	dc1394_feature_info feature;									// current feature
		if (sizeof(feature) > size_t(msg->i.nbytes))				// if user didn't specify a big enough buffer
		{	return(EFAULT);	}												// fails
		void* user_datap = _DEVCTL_DATA(msg->i);			// incoming data (short enough that it will be in the msg)
		memcpy(&feature,user_datap,sizeof(feature));		// read in feature item
		//	Actually read the feature from the camera
    		int stat = getcamerafeature(feature);						// get the feature info
    		if (stat != EOK) return(stat);										// handle error
		memcpy(user_datap,&feature,min(sizeof(feature),size_t(msg->i.nbytes)));	// copy result back to user msg
		msg->o.ret_val = EOK;												// report success
	    msg->o.nbytes = sizeof(feature);								// indicate returned size
   		return(_RESMGR_PTR(ctp, &msg->o, sizeof(msg->o) +  msg->o.nbytes));	// return result
	}
	
	case DCMD_MEDIA_DEVFW_CAMERA_FEATURE_WRITE:	// feature write
	{	dc1394_feature_info feature;									// current feature
		if (sizeof(feature) > size_t(msg->i.nbytes))				// if user didn't specify a big enough buffer
		{	return(EFAULT);	}												// fails
		void* user_datap = _DEVCTL_DATA(msg->i);			// incoming data (short enough that it will be in the msg)
		memcpy(&feature,user_datap,sizeof(feature));		// read in feature item
		//	Actually read the feature from the camera
    		int stat = setcamerafeature(feature);						// get the feature info
    		if (stat != EOK) return(stat);										// handle error
		memcpy(user_datap,&feature,min(sizeof(feature),size_t(msg->i.nbytes)));	// copy result back to user msg
		msg->o.ret_val = EOK;												// report success
	   	msg->o.nbytes = sizeof(feature);								// indicate returned size
   		return(_RESMGR_PTR(ctp, &msg->o, sizeof(msg->o) +  msg->o.nbytes));	// return result
	}
	case DCMD_MEDIA_DEVFW_ROM_READ:							// read from camera ROM
	{	dc1394_devctl_quad_t quadio;									// quadlet to read/write, and value
		if (sizeof(dc1394_devctl_quad_t) != size_t(msg->i.nbytes))				// if user didn't specify buffer of right size
		{	return(EFAULT);	}												// fails
		void* user_datap = _DEVCTL_DATA(msg->i);			// incoming data (short enough that it will be in the msg)
		memcpy(&quadio,user_datap,sizeof(quadio));			// read in ROM address
		//	Actually read the feature from the camera
    		int stat = getcameraromregister(quadio.offset,quadio.value);			// get the requested quad
    		if (stat != EOK) return(stat);										// handle error
		memcpy(user_datap,&quadio,min(sizeof(quadio),size_t(msg->i.nbytes)));	// copy result back to user msg
		msg->o.ret_val = EOK;												// report success
	    msg->o.nbytes = sizeof(quadio);								// indicate returned size
   		return(_RESMGR_PTR(ctp, &msg->o, sizeof(msg->o) +  msg->o.nbytes));	// return result
	}
	
	case DCMD_MEDIA_DEVFW_CCR_READ:
	{	dc1394_devctl_quad_t quadio;									// quadlet to read/write, and value
		if (sizeof(dc1394_devctl_quad_t) != size_t(msg->i.nbytes))				// if user didn't specify buffer of right size
		{	return(EFAULT);	}												// fails
		void* user_datap = _DEVCTL_DATA(msg->i);			// incoming data (short enough that it will be in the msg)
		memcpy(&quadio,user_datap,sizeof(quadio));			// read in ROM address
		//	Actually read the data from the camera
    		int stat = getcameracontrolregister(quadio.offset,quadio.value);			// get the requested quad
    		if (stat != EOK) return(stat);										// handle error
		memcpy(user_datap,&quadio,min(sizeof(quadio),size_t(msg->i.nbytes)));	// copy result back to user msg
		msg->o.ret_val = EOK;												// report success
	    msg->o.nbytes = sizeof(quadio);								// indicate returned size
   		return(_RESMGR_PTR(ctp, &msg->o, sizeof(msg->o) +  msg->o.nbytes));	// return result
	}
	
	case DCMD_MEDIA_DEVFW_CCR_WRITE:
	{	dc1394_devctl_quad_t quadio;									// quadlet to read/write, and value
		if (sizeof(dc1394_devctl_quad_t) != size_t(msg->i.nbytes))				// if user didn't specify buffer of right size
		{	return(EFAULT);	}												// fails
		void* user_datap = _DEVCTL_DATA(msg->i);			// incoming data (short enough that it will be in the msg)
		memcpy(&quadio,user_datap,sizeof(quadio));			// read in ROM address
		//	Actually read the data from the camera
    		int stat = setcameracontrolregister(quadio.offset,quadio.value);			// get the requested quad
    		if (stat != EOK) return(stat);										// handle error
		msg->o.ret_val = EOK;												// report success
	    msg->o.nbytes = 0;													// indicate returned size
   		return(_RESMGR_PTR(ctp, &msg->o, sizeof(msg->o) +  msg->o.nbytes));	// return result
	}

	case DCMD_MEDIA_DEVFW_CAMERA_TAKEPICTURE:
	{	
		int stat = takepicture();												// take the picture
		if (stat != EOK) return(stat);										// handle error
		msg->o.ret_val = EOK;												// report success
	    msg->o.nbytes = 0;													// indicate returned size (zero)
   		return(_RESMGR_PTR(ctp, &msg->o, sizeof(msg->o) +  msg->o.nbytes));	// return result
	}
	
	//	Video mode read --  gets camera video mode
	case DCMD_MEDIA_DEVFW_CAMERA_VIDEO_MODE_READ:
	{	dc1394_video_mode_t modeinfo;								// mode info to set
		if (sizeof(modeinfo) != size_t(msg->i.nbytes))			// if user didn't specify buffer of right size
		{	return(EFAULT);	}												// fails
		void* user_datap = _DEVCTL_DATA(msg->i);			// incoming data (short enough that it will be in the msg)
		int stat = getvideomode(modeinfo);							// try to set mode info
		if (stat != EOK) return(stat);										// handle error
		memcpy(user_datap,&modeinfo,min(sizeof(modeinfo),size_t(msg->i.nbytes)));	// copy result back to user msg
		msg->o.ret_val = EOK;												// report success
	    msg->o.nbytes = sizeof(modeinfo);							// indicate returned size
   		return(_RESMGR_PTR(ctp, &msg->o, sizeof(msg->o) +  msg->o.nbytes));	// return result
	}		
	
	//	Video mode write -- sets camera video mode
	case DCMD_MEDIA_DEVFW_CAMERA_VIDEO_MODE_WRITE:
	{	dc1394_video_mode_t modeinfo;								// mode info to set
		if (sizeof(modeinfo) != size_t(msg->i.nbytes))			// if user didn't specify buffer of right size
		{	return(EFAULT);	}												// fails
		void* user_datap = _DEVCTL_DATA(msg->i);			// incoming data (short enough that it will be in the msg)
		memcpy(&modeinfo,user_datap,sizeof(modeinfo));	// read in mode info
		int stat = setvideomode(modeinfo);							// try to set mode info
		if (stat != EOK) return(stat);										// handle error
		memcpy(user_datap,&modeinfo,min(sizeof(modeinfo),size_t(msg->i.nbytes)));	// copy result back to user msg
		msg->o.ret_val = EOK;												// report success
	    msg->o.nbytes = sizeof(modeinfo);								// indicate returned size
   		return(_RESMGR_PTR(ctp, &msg->o, sizeof(msg->o) +  msg->o.nbytes));	// return result
	}
			
	//	Reset --  reset to initial conditions
	case DCMD_MEDIA_DEVFW_CAMERA_RESET:
	{
		int stat = resetcamera();											// take the picture
		if (stat != EOK) return(stat);										// handle error
		msg->o.ret_val = EOK;												// report success
	    msg->o.nbytes = 0;													// indicate returned size (zero)
   		return(_RESMGR_PTR(ctp, &msg->o, sizeof(msg->o) +  msg->o.nbytes));	// return result
	}

	case DCMD_MEDIA_DEVFW_CAMERA_TIMESTAMP_READ:	// read timestamp of frame currently being read
	{	struct timespec timestamp;										// timestamp to read
		if (sizeof(timestamp) != size_t(msg->i.nbytes))				// if user didn't specify buffer of right size
		{	return(EFAULT);	}												// fails
		void* user_datap = _DEVCTL_DATA(msg->i);			// incoming data (short enough that it will be in the msg)
		//	Actually read the feature from the camera
    		int stat = gettimestamp(timestamp);							// get the requested timestamp
    		if (stat != EOK) return(stat);										// handle error
		memcpy(user_datap,&timestamp,min(sizeof(timestamp),size_t(msg->i.nbytes)));	// copy result back to user msg
		msg->o.ret_val = EOK;												// report success
	    msg->o.nbytes = sizeof(timestamp);						// indicate returned size
   		return(_RESMGR_PTR(ctp, &msg->o, sizeof(msg->o) +  msg->o.nbytes));	// return result
	}

	default:																			// unknown command, fails
		break;		
	}
	//	Not one for which we have special handling; use default action.
	return(ENOSYS);															// We don't know what this is.
}
//
//	Support functions
//
//
//	setvideomode  --  set camera's video mode
//
//	This has the side effect of stopping the camera.
//
int Devdccam::setvideomode(dc1394_video_mode_t& modeinfo)
{	resetcamera();																// shut down
	const CameraModeProps* modeprops = CameraModeSearch(modeinfo.videomode);	// look up matching video mode
	if (!modeprops) return(EINVAL);									// no such mode
	int stat = checkmodeprops(*modeprops,modeinfo.framerate);	// check whether this combo allowed
	if (stat != EOK) return(stat);											// fails
	//	Properties OK, accept them.
	m_modeprops = modeprops;										// set mode properties to use.
	m_framerate = modeinfo.framerate;								// set framerate to use
	return(getvideomode(modeinfo));								// finally get the params for the caller
}
//
//	getvideomode  --  get camera's video mode
//
int Devdccam::getvideomode(dc1394_video_mode_t& modeinfo)
{	
	if (!m_modeprops) return(ESTALE);							// current mode unknown
	modeinfo.videomode = m_modeprops->m_mode;	// return current mode
	modeinfo.framerate = m_framerate;					// return current framerate
	modeinfo.height = m_modeprops->m_height;		// height in pixels
	modeinfo.width = m_modeprops->m_width;		// width in pixels
	modeinfo.bytesperframe = CameraModeFrameBytes(*m_modeprops);	// calc final mode properties
	return(EOK);
}
//
//	getcamerafeature  --  get features and options of a specific feature from the camera
//
int Devdccam::getcamerafeature(dc1394_feature_info& feature) 
{
    uint64_t offset;
    uint32_t value;

    unsigned int orig_fid= feature.feature_id;
    unsigned int updated_fid= feature.feature_id;
    FEATURE_TO_INQUIRY_OFFSET(updated_fid, offset);
	int stat = getcameracontrolregister(offset,value);
	if (stat != EOK) return(stat);
    feature.available= (value & 0x80000000UL) ? DC1394_TRUE : DC1394_FALSE;
    if (feature.available == DC1394_FALSE)
    {
        return(EOK);													// not present, which is OK
    }

    if (orig_fid != FEATURE_TRIGGER) 
    {
        feature.polarity_capable= DC1394_FALSE;
        feature.trigger_mode= 0;
        feature.one_push= (value & 0x10000000UL) ? DC1394_TRUE : DC1394_FALSE;
    }
    else 
    {
        feature.one_push= DC1394_FALSE;
        feature.polarity_capable=
            (value & 0x02000000UL) ? DC1394_TRUE : DC1394_FALSE;
        feature.trigger_mode_capable_mask= ((value >> 12) & 0x0f);
    }

    feature.absolute_capable=
        (value & 0x40000000UL) ? DC1394_TRUE : DC1394_FALSE;
    feature.readout_capable=
        (value & 0x08000000UL) ? DC1394_TRUE : DC1394_FALSE;
    feature.on_off_capable=
        (value & 0x04000000UL) ? DC1394_TRUE : DC1394_FALSE;

    if (orig_fid != FEATURE_TRIGGER) 
    {
        feature.auto_capable=
            (value & 0x02000000UL) ? DC1394_TRUE : DC1394_FALSE;
        feature.manual_capable=
            (value & 0x01000000UL) ? DC1394_TRUE : DC1394_FALSE;

        feature.min= (value & 0xFFF000UL) >> 12;
        feature.max= (value & 0xFFFUL);
    }
    else 
    {
        feature.auto_capable= DC1394_FALSE;
        feature.manual_capable= DC1394_FALSE;
    }

    updated_fid= orig_fid;
    FEATURE_TO_VALUE_OFFSET(updated_fid, offset);
	stat = getcameracontrolregister(offset,value);
	if (stat != EOK) 	return(stat);									// handle failure

    if (orig_fid != FEATURE_TRIGGER) 
    {
        feature.one_push_active=
            (value & 0x04000000UL) ? DC1394_TRUE : DC1394_FALSE;
    }
    else 
    {
        feature.one_push_active= DC1394_FALSE;
    }

    feature.is_on= (value & 0x02000000UL) ? DC1394_TRUE : DC1394_FALSE;

    if (orig_fid != FEATURE_TRIGGER)
    {
        feature.auto_active=
            (value & 0x01000000UL) ? DC1394_TRUE : DC1394_FALSE;
	feature.trigger_polarity= DC1394_FALSE;
    }
    else
    {
        feature.trigger_polarity=
            (value & 0x01000000UL) ? DC1394_TRUE : DC1394_FALSE;
        feature.trigger_mode= (int)((value >> 14) & 0xF);
        feature.auto_active= DC1394_FALSE;
    }

    if (orig_fid == FEATURE_WHITE_BALANCE) 
    {
        feature.BU_value= (value & 0xFFF000UL) >> 12;
        feature.RV_value= value & 0xFFFUL;
    }
    else
    { 
        feature.value= value & 0xFFFUL;
    }

    if (orig_fid == FEATURE_TEMPERATURE)
    {
	feature.target_value= value & 0xFFF000UL;
    }
#ifdef NOTYET
    if (feature.absolute_capable>0)
      {
	dc1394_query_absolute_feature_min_max(handle, node, orig_fid,
					      &feature.abs_min, &feature.abs_max);
	dc1394_query_absolute_feature_value(handle, node, orig_fid,
					    &feature.abs_value);
	dc1394_query_absolute_control(handle, node, orig_fid,
				      &feature.abs_control);
	//fprintf(stderr,"%d %d\n",feature.absolute_capable,feature.abs_control);
      }
#endif // NOTYET
    return (EOK);
}
//
//	setcamerafeature  --  set features and options of a specific feature from the camera
//
//	Two things can be set - the value of the feature, and the on/off value of auto mode.
//
int Devdccam::setcamerafeature(dc1394_feature_info& feature) 
{
    uint64_t offset;
    unsigned int updated_fid= feature.feature_id;
    FEATURE_TO_INQUIRY_OFFSET(updated_fid, offset);		// calc register address of feature
    int stat = setcameracontrolregister(offset,feature.value);	// set 
    if (stat != EOK) return(stat);											// report error
    stat = checkstatus();													// check that camera is in a valid state now
    if (stat != EOK) return(stat);											// fails
    return (getcamerafeature(feature));					// finally do a get, updating feature structure
}
//
//	getcameracontrolregister  --  get a value from a camera control register
//
//	Returns Posix status.
//
int Devdccam::getcameracontrolregister(uint64_t offset, uint32_t& value)
{
	if (offset > 0xFFFF) return(EINVAL);					// range check
	uint64_t address = CCR_BASE + offset;				// read/write from CCR register requested
    int stat = getbus().readquadlet(getbusnode(),address,value);	// do the operation
    return(stat);														// return status
}
//
//	setcameracontrolregister  --  set a value into a camera control register
//
//	***MAY NEED TO PROTECT SOME REGISTERS FROM USER***
//
//	Returns Posix status
//
int Devdccam::setcameracontrolregister(uint64_t offset, uint32_t value)
{	if (offset > 0xFFFF) return(EINVAL);					// range check
    uint64_t address = CCR_BASE + offset;				// read/write from CCR register requested
    return(getbus().writequadlet(getbusnode(),address,value));	// do the operation
}
int Devdccam::getcameraromregister(uint64_t offset, uint32_t& value)
{
	if (offset > 0xFFFF) return(EINVAL);					// range check
    uint64_t address = CONFIG_ROM_BASE + offset;// read/write from ROM register requested
    int stat = getbus().readquadlet(getbusnode(),address,value);	// do the operation
    return(stat);														// return status
}
//
//	gettimestamp  --  get timestamp of image currently being read
//
int Devdccam::gettimestamp(struct timespec& timestamp)
{
	return(m_framequeue.gettimestamp(timestamp));	// pass down
}
//	
//	Bus reset handling
//
//
//	busresetbegin  --  a bus reset has begun.
//
//	Shut down all I/O locally, but don't do any bus transactions.
//
void Devdccam::busresetbegin()
{
	Busentry::busresetbegin();										// do parent-level processing
	m_active = false;														// no longer active
	logprintf("  Camera stopped by bus reset.\n");			// ***TEMP***
	//	Stop listening.  This prevents any further events from coming in.
	getbus().getbuschannels().releasechannel(*this);	// end listening on any isochronous channel
	//	Note that we do not deallocate the isochronous channel, because the bus reset does that.
	m_isochannel = -1;													// no isochronous channel
#ifdef NOTYET
	Framequeueitem item;												// queue a dummy item that says "try again"
	item.m_status = EAGAIN;
	m_framequeue.purge();
	m_framequeue.tryput(item);
#endif // NOTYET
}
//
//	busresetdone --  the bus reset process has finished.  Restart device if active.
//
void Devdccam::busresetdone()
{
}
//
//	Incoming data handling
//
//
//	isorcvpacket  -- an incoming packet belongs to us.
//
//	***TEMP*** just grabs random frames from an incoming stream.
//	Misses every other frame.
//
void Devdccam::isorcvpkt(const llaIsoHdr_t& hdr, dbuf_t*& buffer)
{	
#ifdef OBSOLETE
	if (m_framebuffer.filled()) return;						// if buffer filled, nothing to do
	if (m_framebuffer.getstatus() != EOK) return;	// if error, don't do anything more
	if (hdr.synchro)													// if beginning of frame
	{	if (m_framebuffer.size() == 0)						// if frame buffer empty
		{	
			m_framebuffer.append(buffer,hdr.dataLength);	}	// put in buffer
		else																// if not empty
		{	logprintf("ERROR: Frame undersize.  %d bytes from %s.\n",m_framebuffer.size(),getname().c_str());		// ***TEMP***
			m_framebuffer.setstatus(EIO);					// bad
		}
	} else {															// if not beginning of frame
		if (m_framebuffer.size() == 0 )						// if buffer is empty
		{	}																// ignore, not at start of frame
		else																// if not empty buffer
		{	m_framebuffer.append(buffer,hdr.dataLength);	}	// add to buffer
	}
	if (m_framebuffer.getstatus() != EOK)				// if trouble
	{	//	***ACTIVATE WAITING READ***
		return;
	}
	if (m_framebuffer.size() == m_framebuffer.capacity())	// if exactly filled
	{	logprintf("  Frame complete.  %d bytes from %s.\n",m_framebuffer.size(),getname().c_str());		// ***TEMP***
		m_framebuffer.setfilled();								// note filled
		//	***ACTIVATE WAITING READ***
	}
#endif // OBSOLETE
}
//
//	isorcvblock  -- receive entire image frame as one buffer
//
//	The hardware has done all the header stripping and synchronization for us.
//
void Devdccam::isorcvblock(uint32_t channel, dbuf_t* buffer, unsigned status)
{	m_framequeue.putcopy(buffer,status);					// add to queue
}
//
//	checkmodeprops  --  check whether camera is capable of requested mode
//
//	There are read-only registers in the camera to provide this info.  This
//	does not affect the camera state.
//
//	See IEEE Digital Camera Specification for the definitions of the registers used here.
//
int Devdccam::checkmodeprops(const CameraModeProps& props, uint32_t framerate)
{
    //	Validate framerate range
    if ((framerate < FRAMERATE_MIN) 		// sanity check
    		|| (framerate > FRAMERATE_MAX))
    		return(EINVAL);
    	//	Get format, mode, and framerate, counting from zero, which is what the hardware needs
	const uint32_t relformat = props.m_format - FORMAT_MIN;
	const uint32_t relmode = CameraModeRelMode(props);
	const uint32_t relframerate = framerate - FRAMERATE_MIN;
	//	Check for supported format/mode combination.
	//	There is one inquiry register for each format, with a bit for each mode
	uint32_t modebits;														// supported mode bits
	int stat = getcameracontrolregister(REG_CAMERA_V_MODE_INQ_0 + relformat*4, modebits);
	////logprintf("Checking format %d, mode %d in REG_CAMERA_V_MODE_INQ_0 + %04x: %08x\n",
	////	relformat, relmode, relformat*4, modebits);				// ***TEMP***
	if (stat != EOK) return(stat);
	if ((modebits & bitmsb(relmode)) == 0)						// if appropriate bit not set, fails
	{	return(EINVAL);	}
	//	Check for supported format/mode/framerate combination.
	//	There is one inquiry register for each format/mode combination, with a bit for each speed.
	uint32_t speedbits;
	stat = getcameracontrolregister(REG_CAMERA_V_RATE_INQ_0_0 + relformat*0x20 + relmode*4, speedbits);
	////logprintf("Checking speed %d in REG_CAMERA_V_RATE_INQ_0_0 + %04x: %08x\n",
	////	relframerate, relformat*0x20 + relmode*4, speedbits);				// ***TEMP***
	if ((speedbits & bitmsb(relframerate)) == 0)						// if appropriate bit not set, fails
	{	return(EINVAL);	}
	return(EOK);
}
//
//	setupcamera  -- set up the camera from the ground state.
//
//	Powers up and sets the camera, but does not start it.
//
int Devdccam::setupcamera()
{
	if (!m_modeprops) return(EINVAL);									// no valid properties for start
	resetcamera();																	// clear the camera
	////m_framebuffer.reserve(CameraModeFrameBytes(*m_modeprops));		// set frame size
	////m_framebuffer.clear();														// clear the frame
	m_framequeue.reserve(4);												// ***TEMP*** reserve 4 buffers - canned
		//	Set up the camera.
	//	Power up the camera.
	int stat = setcameracontrolregister(REG_CAMERA_POWER, ON_VALUE);
    if (stat != EOK) return(stat);
    //	Reset to factory defaults.
	stat = setcameracontrolregister(REG_CAMERA_INITIALIZE, ON_VALUE);
    if (stat != EOK) return(stat);
#ifdef OBSOLETE
    //	***TEMP*** check initial conditions
    	stat = checkstatus();														// check the status
	if (stat != EOK){ logprintf("Feature register error after initialize!\n"); return(stat);	}											// fails
	//	***END TEMP***
#endif // OBSOLETE
    //	Turn off ISO mode if on.  Some cameras come up sending, which is a blatant violation of the spec.
    //	They're not supposed to send  until told what channel to send on.
	stat = setcameracontrolregister(REG_CAMERA_ISO_EN, OFF_VALUE);
    if (stat != EOK) return(stat);

	//	Set video format
	logprintf("Setting video format %d, mode %d, framerate %d\n",
		m_modeprops->m_format, CameraModeRelMode(*m_modeprops), m_framerate);	// ***TEMP***
	stat = setcameracontrolregister(REG_CAMERA_VIDEO_FORMAT,
                           (((m_modeprops->m_format - FORMAT_MIN) & 0x7UL) << 29));
    if (stat != EOK) return(stat);
    
	//	Set video mode.  Look up appropriate mode for this format
	stat = setcameracontrolregister(REG_CAMERA_VIDEO_MODE,
                                    (((CameraModeRelMode(*m_modeprops)) & 0x7UL) << 29));;
    if (stat != EOK) return(stat);
    
    //	Set video framerate
    if ((m_framerate < FRAMERATE_MIN) 		// sanity check
    		|| (m_framerate > FRAMERATE_MAX))
    		return(EINVAL);
    stat = setcameracontrolregister(REG_CAMERA_FRAME_RATE,
                     (((m_framerate - FRAMERATE_MIN) & 0x7UL) << 29));
    if (stat != EOK) return(stat);
#ifdef TEMPTURNOFF
	//	Check status of camera feature registers; did camera reject setting?
	stat = checkstatus();														// check the status
	if (stat != EOK) return(stat);												// fails
#endif // TEMPTURNOFF
	return(EOK);																		// success; camera set up
}
//
//	startcamera  --  start up the camera
//
//	Called on opens and on bus reset.
//	Starts up the camera in whatever mode is currently set.
//
int Devdccam::startcamera()
{	if (m_active) return(EOK);													// already running, skip
	int stat = setupcamera();													// reset camera, power up, set up hardware
	if (stat != EOK)																	// if fail
	{	resetcamera();																// clear
		return(stat);																	// fails
	}
	//	Allocate an isochronous channel and set it up.
	stat = getbus().isochannelallocate(m_isochannel);			// find a free isochronous channel
	if (stat) return(stat);															// if fail, no start
	getbus().getbuschannels().listenchannel(*this,m_isochannel,
		CameraModeFrameBytes(*m_modeprops),					// ***TEMP*** should be a real packet size
		CameraModeFrameBytes(*m_modeprops),true);		// attach us to this channel

    //	Set isochronous channel and speed.
    const uint32_t speed = SPEED_400;								// Assume 400Mb/s FireWire
    stat = setcameracontrolregister(REG_CAMERA_ISO_DATA,                                       
                                         (((m_isochannel & 0xFUL) << 28) |
                                           ((speed & 0x3UL) << 24) ));;
    if (stat != EOK) return(stat);
	if (!m_singleshot)   														// if not in single shot mode
	{ 
	    //	Enable continuous transmission from camera. Pictures will be taken at specified frame rate.
		stat = setcameracontrolregister(REG_CAMERA_ISO_EN, ON_VALUE);
    		if (stat != EOK) return(stat);
    	}
    m_active = true;															// camera now active
    return(EOK);																	// done
}
//
//	checkstatus  --  check that camera is in a valid state, ready to take a picture
//
//	The devctl calls shouldn't let the camera be set to an invalid value.
//
int Devdccam::checkstatus()
{	uint32_t statuslo, statushi;
	//	Read the feature status registers -- if nonzero, something is set to an invalid value.
	int stat = getcameracontrolregister(REG_CAMERA_FEATURE_CONTROL_ERROR_STATUS_HI, statushi);	// look at one status register
	if (stat != EOK) return(stat);										// if fail
	stat = getcameracontrolregister(REG_CAMERA_FEATURE_CONTROL_ERROR_STATUS_HI, statushi);	// other one
	if (stat != EOK) return(stat);										// if fail
	//	If any status is nonzero, error.
	statuslo &= 0xf0000000;											// only check camera modes, not Format 6 stuffPP
	if (statushi != 0 || statuslo != 0)
	{	logprintf("ERROR: invalid camera feature setting hi %08x  lo %08x\n",statushi, statuslo);
		return(EINVAL);
	} 
	return(EOK);
}
//
//	resetcamera  --  shut down the camera
//
//	Called on opens and after each bus reset.
//	Shuts down the camera,
//
int Devdccam::resetcamera()
{	m_active = false;														// no longer active
	m_singleshot = false;												// not in single shot mode
	logprintf("Stopping camera.\n");								// ***TEMP***
	//	Stop listening.  This prevents any further events from coming in.
	getbus().getbuschannels().releasechannel(*this);	// end listening on any isochronous channel
	//	Turn off as much as we can.
	int stat1 = setcameracontrolregister(REG_CAMERA_ISO_EN, OFF_VALUE);
	int stat2 = setcameracontrolregister(REG_CAMERA_POWER, OFF_VALUE);
	//	Release isochronous resources
	int stat0 = EOK;
	if (m_isochannel >= 0)
	{	stat0 = getbus().isochannelrelease(m_isochannel);			// release the channel
	}
    if (stat0 != EOK) return(stat0);
    if (stat1 != EOK) return(stat1);
    if (stat2 != EOK) return(stat2);
    return(EOK);
}
//
//	isorecvpurge  --  isochronous receive is being stopped.
//
//	Release all duplicate buffers here.
//
void Devdccam::isorcvpurge()
{	
	m_framequeue.purge();											// purge any pending buffers
}

//
//	takepicture  --  take a single picture
//
//	The next read will read the picture.
//
//	***NEEDS WORK***
//
int Devdccam::takepicture()
{	logprintf("Takepicture\n");											// ***TEMP***
	if (!m_singleshot)														// if not in single shot mode
	{	m_singleshot = true;												// set single shot mode
		if (m_active)															// if already active
		{	int stat = setcameracontrolregister(REG_CAMERA_ISO_EN,OFF_VALUE);	// turn off continuous mode
			if (stat != EOK) return(stat);								// handle error
			//	***DO WE NEED A DELAY HERE???***
		}
	}
	if (!m_active) 															// if camera not active			
	{	int stat = startcamera();										// start up camera
		if (stat != EOK) return(stat);									// handle failure
	}
	m_framequeue.purge();											// clear frame queue
	//	Check that camera status is valid.
	int stat = checkstatus();
	if (stat != EOK) return(stat);										// if fail
	//	Make sure camera isn't currently taking a picture.
	uint32_t oneshotreg;													// one shot register
	stat = getcameracontrolregister(REG_CAMERA_ONE_SHOT, oneshotreg);	// take one frame
	if (stat != EOK) return(stat);										// if fail
	if (oneshotreg & ON_VALUE)										// if one-shot in progress
	{	logprintf("  Camera busy taking picture.\n");			// user asked too soon
		return(EBUSY);
	}
	stat = setcameracontrolregister(REG_CAMERA_ONE_SHOT, ON_VALUE);	// take one frame
	if (stat != EOK) return(stat);										// if fail
	return(EOK);
}
//
//	appendstringfromrom  --  get a text string from device ROM
//
//	Assumes string is ASCII, not UNICODE, although the IEEE-1394 spec allows UNICODE device names.
//
//	Appends to string.
//
int Devdccam::appendstringfromrom(uint32_t offset, int len, string& s)
{	len = min(len,255);													// avoid insane lengths
    while (len > 0)															// until all quads read
    {	uint32_t quad;														// quadlet, in host order
    		int stat = getcameraromregister(offset, quad);		// get 4 chars from ROM
        if (stat != EOK) return(stat);									// if fail
		for (int i=0; i<4; i++)											// for bytes of quad
		{	char ch = quad >> 24;										// get next char
			if (ch == '\0') return(EOK);								// stop at null
			s += ch;															// append to string
			quad <<= 8;													// get next byte
		}
		offset+= 4;															// advance to next quad
        len-= 4;																// use up that quad
    }
    return(EOK);
}
//
//	getcameraident  -- get camera indentity info
//
int Devdccam::getcameraident(uint64_t& euid, string& vendor, string& model)
{
    int len;
    uint32_t offset, offset2;
    uint32_t value[2];
    unsigned int count;
	//	Hard-coded constants below are from dc1394,
    //	Get the 64-bit EUID 
    int stat;
    if ((stat = getcameraromregister(ROM_BUS_INFO_BLOCK+0x0C, value[0]))) return(stat);
    if ((stat = getcameraromregister(ROM_BUS_INFO_BLOCK+0x10, value[1]))) return(stat);
	euid = ((uint64_t)value[0] << 32) | (uint64_t)value[1];	// combine into one octlet

    //	Get the unit directory offset
    if ((stat = getcameraromregister((ROM_ROOT_DIRECTORY + 16), offset))) return(stat);
    offset<<= 2;																	// offset is in units of quads
    offset&= 0x00ffffff;
    offset+= (ROM_ROOT_DIRECTORY + 16);

    //	Get the unit-dependent directory offset.
    if ((stat = getcameraromregister(offset+12, offset2)))  return(stat);
    offset2<<= 2;																// offset is in units of quads
    offset2+= (offset + 12);
    offset2&= 0x00ffffff;

    //	Get the base address of the command registers
    if ((stat = getcameraromregister(offset2+4, offset)))  return(stat);
    offset<<= 2;																	// offset is in units of quads
    offset&= 0x00ffffff;

    //	Get the vendor name leaf offset
    if ((stat = getcameraromregister(offset2+8, offset))) return(stat);
    offset<<= 2;																	// offset is in units of quads
    offset&= 0x00ffffff;
    offset+= (offset2 + 8);

    //	Get the length of the vendor name
    if ((stat = getcameraromregister(offset, value[0]))) return(stat);
    len= (int)((value[0] >> 16) & 0xFFFFUL)*4-8;				// was in quads, not including length
    offset+= 12;																	// advance to name
    if ((stat = appendstringfromrom(offset,len,vendor))) return(stat);	// get vendor name
    //	Get the model name leaf offset
    if ((stat = getcameraromregister(offset2+12, offset))) return(stat);
    offset<<= 2;																// units are quads
    offset &=0x00ffffff;
    offset+= (offset2 + 12);

    //	Get length of model name
    if ((stat = getcameraromregister(offset, value[0]))) return(stat);
    len= (int)((value[0] >> 16) & 0xFFFFUL)*4-8;
    offset+= 12;
    count= 0;
    if ((stat = appendstringfromrom(offset,len,model))) return(stat);	// get model name
    return(EOK);
}
//
//	setidentity  -- set identity info for device
//
int Devdccam::setidentity()
{
	string vendor, model;
	uint64_t euid;
	int stat = getcameraident(euid,vendor,model);					// get camera info from camera ROM
	if (stat != EOK) return(stat);												// fails
	string s;																			// build up camera name
	s = getnameprefix();
	s += '-';
	s += vendor;
	s += '-';
	s += model;
	s += '-';
	uint64_t serial = euid & 0xffffffff;										// get serial number
    char buf[30];																	// enough space for decimal number, even 64-bit
    s+= itoa(serial,buf,10);														// end with serial number
    Deventry::canonizename(s);												// keep strange characters out of filenames
    setname(s);																		// set device name
    return(EOK);																		// success
}

//
//	Framequeue -- dbuf-based input queue
//
Framequeue::Framequeue()
{
}
Framequeue::~Framequeue()
{	purge();																		// release any queued buffers
}
//
//	reserve  -- reserve some number of buffer slots
//
//	This just reserves queue slots; it doesn't actually allocate large buffers.
//
//	Must NOT be called while input is active.
//	***ARE WE SURE THAT NEVER HAPPENS?***
//
int Framequeue::reserve(uint32_t bufcount)
{	ost::MutexLock lok(m_getlock);												// lock get side of queue
	purge();																				// free any pending buffers
	m_rxcopypool.destroy();														// destroy existing pool if any
	int stat = m_rxcopypool.create(0,bufcount,0);						// header entries only
	if (stat != EOK) return(stat);													// fails
	return(EOK);																			// success
}
//
//	putcopy  --  copy a buffer and add it to the queue
//
//	The "copy" is lazy; it just increments a counter in the LLA dbuf system. 
//
void Framequeue::putcopy(dbuf_t* buffer, int stat)
{
	dbuf_t* dupbuf = dbufDuplicate(m_rxcopypool.getpool(),buffer);	// duplicate the buffer
	if (!dupbuf)																			// if trouble
	{	////logprintf("ERROR: buffer overrun.\n");							// ***TEMP*** really a caller problem
		return;
	}
	Framequeueitem item;															// item to queue
	item.m_buf = dupbuf;															// put in buffer
	item.m_status = stat;																// put in status
	clock_gettime(CLOCK_REALTIME,&item.m_timestamp); 			// timestamp input
	bool success = m_queue.tryput(item);									// enqueue input frame
	if (!success)																			// if input queue full
	{	logprintf("Dropped frame.\n");											// ***TEMP*** too verbose
		dbufFree(item.m_buf);														// release the duplicate
		return;
	}
}
//
//	get -- get from input queue
//
//	We keep the buffer
//
int Framequeue::get(float maxwait)
{	ost::MutexLock lok(m_getlock);												// lock get side of queue
	clear();																					// release previously active buffer if any
	bool success = m_queue.get(m_current,maxwait);
	if (!success) return(ETIMEDOUT);											// timeout, no input
	//	We have a queue item
	if (m_current.m_status != EOK)												// if error status, don't keep the buffer
	{	if (m_current.m_buf) 
		{	dbufFree(m_current.m_buf); m_current.m_buf = 0; }
	}
	return(m_current.m_status);													// return the status
}
//
//	clear -- done with current buffer
//
void Framequeue::clear()
{	ost::MutexLock lok(m_getlock);												// lock get side of queue
	if (m_current.m_buf)																// free buf if needed
	{	dbufFree(m_current.m_buf);
		m_current.m_buf = 0;
	}
}
//
//	purge  -- purge input queue
//
//	Just discards all buffers
//
void Framequeue::purge()
{	ost::MutexLock lok(m_getlock);												// lock get side of queue
	clear();																					// release current buffer if any
	while (m_queue.tryget(m_current))										// get until empty
	{	clear();	}																			// free buffer
}
//
//	getdata -- transfer data to user buffer
//
//	Returns bytes transferred
//
int Framequeue::getdata(size_t offset, uint8_t data[], size_t maxsize)
{
	if (!m_current.m_buf) return(0);												// no buffer; can't transfer
	int count = dbufCopyDataFromBuf(m_current.m_buf,offset,data,maxsize);	// copy indicated number of bytes
	if (count < 0) count = 0;														// if err, no data
	return(count);
}
//
//	size  -- get amount of data available
//
size_t Framequeue::size()
{	if (!m_current.m_buf) return(0);												// no buffer; can't transfer
	int count = dbufGetLength(m_current.m_buf);						// copy indicated number of bytes
	if (count < 0) count = 0;														// if err, no data
	return(count);
}










