/*
	devfw_camera.h  -- FireWire camera driver, user view

	This is the application interface to the QNX FireWire camera driver.
	John Nagle
	Animats
	March, 2003

	Usable from C and C++
*/
/*
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
*/
#ifndef DEVFW_CAMERA_H
#define DEVFW_CAMERA_H
#include <devctl.h>
#include <time.h>
#include "devdccamdefs.h"

/*	Commands for this interface	*/

/*
	Normal usage is
	
	devfw_camera_feature camerafeature;
	int stat = devctl(fd, DCMD_MEDIA_DEVFW_CAMERA_FEATURE_READ, &dc1394_feature_info, sizeof(dc1394_feature_info), 0);
	etc...
	
	Feature write and video mode write, if successful, update the input structure with the new values, as if a read.
*/

/*	Read/write features	*/
const unsigned int DCMD_MEDIA_DEVFW_CAMERA_FEATURE_READ_CODE = 0x2;	
const unsigned int DCMD_MEDIA_DEVFW_CAMERA_FEATURE_WRITE_CODE = 0x3;	

const unsigned int DCMD_MEDIA_DEVFW_CAMERA_FEATURE_READ =
	__DIOTF(_DCMD_MEDIA,DCMD_MEDIA_DEVFW_CAMERA_FEATURE_READ_CODE,dc1394_feature_info);
const unsigned int DCMD_MEDIA_DEVFW_CAMERA_FEATURE_WRITE =
	__DIOTF(_DCMD_MEDIA,DCMD_MEDIA_DEVFW_CAMERA_FEATURE_WRITE_CODE,dc1394_feature_info);


/*
	Read/write camera control registers, read configuration ROM
	Very low-level interface. Deprecated.
*/
struct dc1394_devctl_quad_s {
	uint32_t	offset;									// offset address from base
	uint32_t	value;									// value in/out
};
typedef struct dc1394_devctl_quad_s dc1394_devctl_quad_t;	/* C compatibility */
	
/*	Read camera configuration ROM */
const unsigned int DCMD_MEDIA_DEVFW_ROM_CODE = 0x4;
const unsigned int DCMD_MEDIA_DEVFW_ROM_READ =
	__DIOTF(_DCMD_MEDIA,DCMD_MEDIA_DEVFW_ROM_CODE,dc1394_devctl_quad_t);

/*	Read/write camera control registers */
const unsigned int DCMD_MEDIA_DEVFW_CCR_CODE = 0x6;
const unsigned int DCMD_MEDIA_DEVFW_CCR_READ =
	__DIOTF(_DCMD_MEDIA,DCMD_MEDIA_DEVFW_CCR_CODE,dc1394_devctl_quad_t);
const unsigned int DCMD_MEDIA_DEVFW_CCR_WRITE =
	__DIOT(_DCMD_MEDIA,DCMD_MEDIA_DEVFW_CCR_CODE,dc1394_devctl_quad_t);
	
/*	Set/Get video mode  -- set desired video mode */

struct dc1394_video_mode_s {
	int 	videomode;					/*	MODE_160x120_YUV444, etc. */
	int		framerate;						/* FRAMERATE_30, etc.*/
	int		height;							/* in pixels */
	int		width;							/* in pixels */
	int		bytesperframe;				/* in bytes */
};
typedef struct dc1394_video_mode_s dc1394_video_mode_t;	/* C compatibility */

const unsigned int DCMD_MEDIA_DEVFW_CAMERA_VIDEO_MODE_CODE = 0x8;
const unsigned int DCMD_MEDIA_DEVFW_CAMERA_VIDEO_MODE_READ =
	__DIOF(_DCMD_MEDIA,DCMD_MEDIA_DEVFW_CAMERA_VIDEO_MODE_CODE,dc1394_video_mode_t);
const unsigned int DCMD_MEDIA_DEVFW_CAMERA_VIDEO_MODE_WRITE =
	__DIOTF(_DCMD_MEDIA,DCMD_MEDIA_DEVFW_CAMERA_VIDEO_MODE_CODE,dc1394_video_mode_t);

/*	Take single frame -- take one frame with the camera now */
const unsigned int DCMD_MEDIA_DEVFW_CAMERA_TAKEPICTURE_CODE = 0xa;
const unsigned int DCMD_MEDIA_DEVFW_CAMERA_TAKEPICTURE =
	__DION(_DCMD_MEDIA,DCMD_MEDIA_DEVFW_CAMERA_TAKEPICTURE_CODE);
	
/*	Reset --  reset to initial conditions */
const unsigned int DCMD_MEDIA_DEVFW_CAMERA_RESET_CODE = 0xc;
const unsigned int DCMD_MEDIA_DEVFW_CAMERA_RESET =
	__DION(_DCMD_MEDIA,DCMD_MEDIA_DEVFW_CAMERA_RESET_CODE);
	
const unsigned int DCMD_MEDIA_DEVFW_CAMERA_TIMESTAMP_CODE = 0xe;
const unsigned int DCMD_MEDIA_DEVFW_CAMERA_TIMESTAMP_READ =
	__DIOF(_DCMD_MEDIA,DCMD_MEDIA_DEVFW_CAMERA_TIMESTAMP_CODE,struct timespec);



#endif /* DEVFW_CAMERA_H */