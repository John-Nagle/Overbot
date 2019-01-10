//
//	devdccamtables.cpp  -- tables used by the FireWire camera driver
//
//	Driver for DC-type cameras
//
//	John Nagle
//	Animats
//	
//	March. 2003
//
#include <limits.h>
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <errno.h>
#include <devctl.h>
#include <string.h>
#include <assert.h>
#include "businfo.h"
#include "devdccam.h"
#include "dc1394constants.h"
#include "devfw-camera.h"
#include "utilities.h"
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
//
//	Sizes of camera color modes.  Tied to enum CameraColorMode.
//
const unsigned int CameraBytesPerFourPixels[] =
	{	4+4+4, 4+2+2, 4+1+1, 4+4+4, 4, 4*2, 0 };

//
//	Known digital camera formats.  These are standardized in the IEEE standard.
//
const CameraModeProps dccamProps[] = {
	{	FORMAT_VGA_NONCOMPRESSED,		MODE_160x120_YUV444,		160,	120,	YUV444		},
	{	FORMAT_VGA_NONCOMPRESSED,		MODE_320x240_YUV422,		320,	240,	YUV422		},
	{	FORMAT_VGA_NONCOMPRESSED,		MODE_640x480_YUV411,		640,	480,	YUV411		},
	{	FORMAT_VGA_NONCOMPRESSED,		MODE_640x480_YUV422,		640,	480,	YUV422		},
	{	FORMAT_VGA_NONCOMPRESSED,		MODE_640x480_RGB,			640,	480,	RGB444	},
	{	FORMAT_VGA_NONCOMPRESSED,		MODE_640x480_MONO,			640,	480,	MONO1		},
	{	FORMAT_VGA_NONCOMPRESSED,		MODE_640x480_MONO16,		640,	480,	MONO2		},
	
	{	FORMAT_SVGA_NONCOMPRESSED_1,	MODE_800x600_YUV422,		800,	600,	YUV422		},
	{	FORMAT_SVGA_NONCOMPRESSED_1,	MODE_800x600_RGB,			800,	600,	RGB444	},
	{	FORMAT_SVGA_NONCOMPRESSED_1,	MODE_800x600_MONO,			800,	600,	MONO1		},
	{	FORMAT_SVGA_NONCOMPRESSED_1,	MODE_800x600_MONO16,		800,	600,	MONO2		},
	
	{	FORMAT_SVGA_NONCOMPRESSED_1,	MODE_1024x768_YUV422,	1024,768,	YUV422		},
	{	FORMAT_SVGA_NONCOMPRESSED_1,	MODE_1024x768_RGB,			1024,768,	RGB444	},
	{	FORMAT_SVGA_NONCOMPRESSED_1,	MODE_1024x768_MONO,		1024,768,	MONO1		},
	{	FORMAT_SVGA_NONCOMPRESSED_1,	MODE_800x600_MONO16,		1024,768,	MONO2		},
	
	{	FORMAT_SVGA_NONCOMPRESSED_2,	MODE_1280x960_YUV422,	1280,960,	YUV422		},
	{	FORMAT_SVGA_NONCOMPRESSED_2,	MODE_1280x960_RGB,			1280,960,	RGB444	},
	{	FORMAT_SVGA_NONCOMPRESSED_2,	MODE_1280x960_MONO,		1280,960,	MONO1		},
	{	FORMAT_SVGA_NONCOMPRESSED_2,	MODE_1280x960_MONO16,	1280,960,	MONO2		},
	{ 0 } };	
	
//	
//	Lookup functions
//
//
//	CameraModeSearch -- look up camera mode properties by format and mode
//
const CameraModeProps* CameraModeSearch(uint32_t mode)
{
	for (size_t i=0; dccamProps[i].m_height != 0; i++)
	{	const CameraModeProps& p = dccamProps[i];				// get row of properties
		if (mode == p.m_mode)												// if match
		{	return(&p);	}															// success
	}
	return(0);																			// returns null if fail
}
//
//	CameraModeFrameBytes  -- calc camera mode frame bytes
//
const size_t CameraModeFrameBytes(const CameraModeProps& props)
{	size_t pixels = props.m_height * props.m_width;			// pixels per frame
	return((pixels / 4) * CameraBytesPerFourPixels[props.m_colormode]);		// calc bytes per frame
}
//
//	CameraModeRelMode -- calc mode value for device register
//
//	The hardware wants the mode relative to the starting value for that format.
//
const uint32_t CameraModeRelMode(const CameraModeProps& props)
{	
    switch (props.m_format) {
        case FORMAT_VGA_NONCOMPRESSED:			return(props.m_mode - MODE_FORMAT0_MIN);
        case FORMAT_SVGA_NONCOMPRESSED_1:		return(props.m_mode - MODE_FORMAT1_MIN);
        case FORMAT_SVGA_NONCOMPRESSED_2:		return(props.m_mode - MODE_FORMAT2_MIN);
        default: return(0);											// should not happen
    }
}
