#ifndef DC1394CONSTANTS_H
#define DC1394CONSTANTS_H
/*
 * 1394-Based Digital Camera Control Library
 * Copyright (C) 2000 SMART Technologies Inc.
 *
 * Written by Gord Peters <GordPeters@smarttech.com>
 * Additions by Chris Urmson <curmson@ri.cmu.edu>
 * Additions by Damien Douxchamps <douxchamps@ieee.org>
 *
 * Acknowledgments:
 * Per Dalgas Jakobsen <pdj@maridan.dk>
 *   - Added retries to ROM and CSR reads
 *   - Nicer endianness handling
 *
 * Robert Ficklin <rficklin@westengineering.com>
 *   - bug fixes
 * 
 * Julie Andreotti <JulieAndreotti@smarttech.com>
 *   - bug fixes
 * 
 * Ann Dang <AnnDang@smarttech.com>
 *   - bug fixes
 *
 * Dan Dennedy <dan@dennedy.org>
 *  - bug fixes
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <stdlib.h>
#include <string.h>
#include <netinet/in.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <errno.h>


/********************/
/* Base ROM offsets */
/********************/

#define ROM_BUS_INFO_BLOCK             0x400U
#define ROM_ROOT_DIRECTORY             0x414U

/**********************************/
/* Configuration Register Offsets */
/**********************************/

/* See the 1394-Based Digital Camera Spec. for definitions of these */
#define REG_CAMERA_INITIALIZE          0x000U
#define REG_CAMERA_V_FORMAT_INQ        0x100U
#define REG_CAMERA_V_MODE_INQ_BASE     0x180U
#define REG_CAMERA_V_RATE_INQ_BASE     0x200U
#define REG_CAMERA_V_REV_INQ_BASE      0x2C0U
#define REG_CAMERA_BASIC_FUNC_INQ      0x400U
#define REG_CAMERA_FEATURE_HI_INQ      0x404U
#define REG_CAMERA_FEATURE_LO_INQ      0x408U
#define REG_CAMERA_ADV_FEATURE_INQ     0x480U
#define REG_CAMERA_FEATURE_HI_BASE_INQ 0x500U
#define REG_CAMERA_FEATURE_LO_BASE_INQ 0x580U
#define REG_CAMERA_FRAME_RATE          0x600U
#define REG_CAMERA_VIDEO_MODE          0x604U
#define REG_CAMERA_VIDEO_FORMAT        0x608U
#define REG_CAMERA_ISO_DATA            0x60CU
#define REG_CAMERA_POWER               0x610U
#define REG_CAMERA_ISO_EN              0x614U
#define REG_CAMERA_MEMORY_SAVE         0x618U
#define REG_CAMERA_ONE_SHOT            0x61CU
#define REG_CAMERA_MEM_SAVE_CH         0x620U
#define REG_CAMERA_CUR_MEM_CH          0x624U

#define REG_CAMERA_FEATURE_HI_BASE     0x800U
#define REG_CAMERA_FEATURE_LO_BASE     0x880U

#define REG_CAMERA_BRIGHTNESS	       0x800U
#define REG_CAMERA_EXPOSURE	       0x804U
#define REG_CAMERA_SHARPNESS	       0x808U
#define REG_CAMERA_WHITE_BALANCE       0x80CU
#define REG_CAMERA_HUE		       0x810U
#define REG_CAMERA_SATURATION	       0x814U
#define REG_CAMERA_GAMMA	       0x818U
#define REG_CAMERA_SHUTTER	       0x81CU
#define REG_CAMERA_GAIN		       0x820U
#define REG_CAMERA_IRIS		       0x824U
#define REG_CAMERA_FOCUS	       0x828U
#define REG_CAMERA_TEMPERATURE	       0x82CU
#define REG_CAMERA_TRIGGER_MODE	       0x830U
#define REG_CAMERA_ZOOM		       0x880U
#define REG_CAMERA_PAN		       0x884U
#define REG_CAMERA_TILT		       0x888U
#define REG_CAMERA_OPTICAL_FILTER      0x88CU
#define REG_CAMERA_CAPTURE_SIZE	       0x8C0U
#define REG_CAMERA_CAPTURE_QUALITY     0x8C4U 


/***********************/
/*  Macro definitions  */
/***********************/

#define FEATURE_TO_VALUE_OFFSET(feature, offset)                      \
                                                                      \
    if ( (feature > FEATURE_MAX) || (feature < FEATURE_MIN) )         \
    {                                                                 \
		return(EINVAL);                                        \
    }                                                                 \
    else if (feature < FEATURE_ZOOM)                                  \
    {                                                                 \
	offset= REG_CAMERA_FEATURE_HI_BASE;                           \
        feature-= FEATURE_MIN;                                        \
    }                                                                 \
    else                                                              \
    {                                                                 \
	offset= REG_CAMERA_FEATURE_LO_BASE;                           \
	feature-= FEATURE_ZOOM;                                       \
                                                                      \
	if (feature >= FEATURE_CAPTURE_SIZE)                          \
	{                                                             \
	    feature+= 12;                                             \
	}                                                             \
                                                                      \
    }                                                                 \
                                                                      \
    offset+= feature * 0x04U;


#define FEATURE_TO_INQUIRY_OFFSET(feature, offset)                    \
                                                                      \
    if ( (feature > FEATURE_MAX) || (feature < FEATURE_MIN) )         \
    {                                                                 \
	return(EINVAL);                                        \
    }                                                                 \
    else if (feature < FEATURE_ZOOM)                                  \
    {                                                                 \
	offset= REG_CAMERA_FEATURE_HI_BASE_INQ;                       \
        feature-= FEATURE_MIN;                                        \
    }                                                                 \
    else                                                              \
    {                                                                 \
	offset= REG_CAMERA_FEATURE_LO_BASE_INQ;                       \
	feature-= FEATURE_ZOOM;                                       \
                                                                      \
	if (feature >= FEATURE_CAPTURE_SIZE)                          \
	{                                                             \
	    feature+= 12;                                             \
	}                                                             \
                                                                      \
    }                                                                 \
                                                                      \
    offset+= feature * 0x04U;

#ifdef OBSOLETE
/**************************/
/*  Constant definitions  */
/**************************/

const char *dc1394_feature_desc[NUM_FEATURES] =
{
    "Brightness",
    "Exposure",
    "Sharpness",
    "White Balance",
    "Hue",
    "Saturation",
    "Gamma",
    "Shutter",
    "Gain",
    "Iris",
    "Focus",
    "Temperature",
    "Trigger",
    "Zoom",
    "Pan",
    "Tilt",
    "Optical Filter",
    "Capture Size",
    "Capture Quality"
};

/*
  These arrays define how many image quadlets there
  are in a packet given a mode and a frame rate
  This is defined in the 1394 digital camera spec 
*/
const int quadlets_per_packet_format_0[42] = 
{
     -1,  -1,  15,  30,  60,  -1,
     -1,  20,  40,  80, 160,  -1,
     -1,  60, 120, 240, 480,  -1,
     -1,  80, 160, 320, 640,  -1,
     -1, 120, 240, 480, 960,  -1,
     -1,  40,  80, 160, 320, 640,
     -1,  80, 160, 320, 640,  -1
};

const int quadlets_per_packet_format_1[48] = 
{
     -1, 125, 250, 500, 1000,   -1,
     -1,  -1, 375, 750,   -1,   -1,
     -1,  -1, 125, 250,  500, 1000,
     96, 192, 384, 768,   -1,   -1,
    144, 288, 576,  -1,   -1,   -1,
     48,  96, 192, 384,  768,   -1,
     -1, 125, 250, 500, 1000,   -1,
     96, 192, 384, 768,   -1,   -1
};

const int quadlets_per_packet_format_2[48] = 
{
    160, 320,  640,   -1, -1, -1,
    240, 480,  960,   -1, -1, -1,
     80, 160,  320,  640, -1, -1,
    250, 500, 1000,   -1, -1, -1,
    375, 750,   -1,   -1, -1, -1,
    125, 250,  500, 1000, -1, -1,
    160, 320,  640,   -1, -1, -1,
    250, 500, 1000,   -1, -1, -1
};
#endif // OBSOLETE
//
//	More constants from internals.
//
/* Definitions which application developers shouldn't care about */
#define CONFIG_ROM_BASE             0xFFFFF0000000ULL
#define CCR_BASE                    0xFFFFF0F00000ULL

#define ON_VALUE                    0x80000000UL
#define OFF_VALUE                   0x00000000UL


#endif // DC1394CONSTANTS_H