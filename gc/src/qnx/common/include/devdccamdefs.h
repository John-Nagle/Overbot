/*
 * 1394-Based Digital Camera Control Library
 * Copyright (C) 2000 SMART Technologies Inc.
 *
 * Written by Gord Peters <GordPeters@smarttech.com>
 * Additions by Chris Urmson <curmson@ri.cmu.edu>
 * Additions by Damien Douxchamps <douxchamps@ieee.org>
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
#ifndef DEVDCCAMDEFS_H
#define DEVDCCAMDEFS_H
#include <sys/types.h>
/*
	Definitions for FireWire digital camera API.
	Part of devfw-ohci driver for QNX
	John Nagle
	Animats
	March, 2003
	
	Constants here are determined by the FireWire camera specification.
	
	This file is both C and C++ compatible, and is included by users of the driver's API
*/

/* Enumeration of data speeds on the FireWire bus */
enum
{
    SPEED_100= 0,
    SPEED_200,
    SPEED_400
};
#define SPEED_MIN                   SPEED_100
#define SPEED_MAX                   SPEED_400
#define NUM_SPEEDS                  (SPEED_MAX - SPEED_MIN + 1)

/* Enumeration of camera framerates */
enum 
{
    FRAMERATE_1_875= 32,
    FRAMERATE_3_75,
    FRAMERATE_7_5,
    FRAMERATE_15,
    FRAMERATE_30,
    FRAMERATE_60
};
#define FRAMERATE_MIN               FRAMERATE_1_875
#define FRAMERATE_MAX               FRAMERATE_60
#define NUM_FRAMERATES              (FRAMERATE_MAX - FRAMERATE_MIN + 1)

/* Enumeration of camera modes for Format_0 */
enum 
{
    MODE_160x120_YUV444= 64,
    MODE_320x240_YUV422,
    MODE_640x480_YUV411,
    MODE_640x480_YUV422,
    MODE_640x480_RGB,
    MODE_640x480_MONO,
    MODE_640x480_MONO16
};
#define MODE_FORMAT0_MIN	    MODE_160x120_YUV444
#define MODE_FORMAT0_MAX	    MODE_640x480_MONO16
#define NUM_FORMAT0_MODES	    (MODE_FORMAT0_MAX - MODE_FORMAT0_MIN + 1)

/* Enumeration of camera modes for Format_1 */
enum 
{
    MODE_800x600_YUV422= 96,
    MODE_800x600_RGB,
    MODE_800x600_MONO,
    MODE_1024x768_YUV422,
    MODE_1024x768_RGB,
    MODE_1024x768_MONO,
    MODE_800x600_MONO16,
    MODE_1024x768_MONO16
};
#define MODE_FORMAT1_MIN	    MODE_800x600_YUV422
#define MODE_FORMAT1_MAX	    MODE_1024x768_MONO16
#define NUM_FORMAT1_MODES	    (MODE_FORMAT1_MAX - MODE_FORMAT1_MIN + 1)


/* Enumeration of camera modes for Format_2 */
enum 
{
    MODE_1280x960_YUV422= 128,
    MODE_1280x960_RGB,
    MODE_1280x960_MONO,
    MODE_1600x1200_YUV422,
    MODE_1600x1200_RGB,
    MODE_1600x1200_MONO,
    MODE_1280x960_MONO16,
    MODE_1600x1200_MONO16
};
#define MODE_FORMAT2_MIN	    MODE_1280x960_YUV422
#define MODE_FORMAT2_MAX	    MODE_1600x1200_MONO16
#define NUM_FORMAT2_MODES	    (MODE_FORMAT2_MAX - MODE_FORMAT2_MIN + 1)

/* Enumeration of camera modes for Format_6 */
enum 
{
    MODE_EXIF= 256
};
#define MODE_FORMAT6_MIN            MODE_EXIF
#define MODE_FORMAT6_MAX            MODE_EXIF
#define NUM_FORMAT6_MODES           (MODE_FORMAT6_MAX - MODE_FORMAT6_MIN + 1)

/* Enumeration of camera modes for Format_7 */
enum {
    MODE_FORMAT7_0= 288,
    MODE_FORMAT7_1,
    MODE_FORMAT7_2,
    MODE_FORMAT7_3,
    MODE_FORMAT7_4,
    MODE_FORMAT7_5,
    MODE_FORMAT7_6,
    MODE_FORMAT7_7
};
#define MODE_FORMAT7_MIN            MODE_FORMAT7_0
#define MODE_FORMAT7_MAX            MODE_FORMAT7_7
#define NUM_MODE_FORMAT7            (MODE_FORMAT7_MAX - MODE_FORMAT7_MIN + 1)

/* Enumeration of Format_7 color modes */
enum {
    COLOR_FORMAT7_MONO8= 320,
    COLOR_FORMAT7_YUV411,
    COLOR_FORMAT7_YUV422,
    COLOR_FORMAT7_YUV444,
    COLOR_FORMAT7_RGB8,
    COLOR_FORMAT7_MONO16,
    COLOR_FORMAT7_RGB16
};
#define COLOR_FORMAT7_MIN           COLOR_FORMAT7_MONO8
#define COLOR_FORMAT7_MAX           COLOR_FORMAT7_RGB16
#define NUM_COLOR_FORMAT7           (COLOR_FORMAT7_MAX - COLOR_FORMAT7_MIN + 1)

/* Enumeration of trigger modes */
enum {
    TRIGGER_MODE_0= 352,
    TRIGGER_MODE_1,
    TRIGGER_MODE_2,
    TRIGGER_MODE_3
};
#define TRIGGER_MODE_MIN            TRIGGER_MODE_0
#define TRIGGER_MODE_MAX            TRIGGER_MODE_3
#define NUM_TRIGGER_MODE            (TRIGGER_MODE_3 - TRIGGER_MODE_0 + 1)

/* Enumeration of camera image formats */
enum {
    FORMAT_VGA_NONCOMPRESSED= 384,
    FORMAT_SVGA_NONCOMPRESSED_1,
    FORMAT_SVGA_NONCOMPRESSED_2,
    /* 3 reserved formats */
    FORMAT_STILL_IMAGE= 390,
    FORMAT_SCALABLE_IMAGE_SIZE
};
#define FORMAT_MIN                  FORMAT_VGA_NONCOMPRESSED
#define FORMAT_MAX                  FORMAT_SCALABLE_IMAGE_SIZE
#define NUM_FORMATS                 (FORMAT_MAX - FORMAT_MIN + 1)

/* Enumeration of camera features */
enum 
{
    FEATURE_BRIGHTNESS= 416,
    FEATURE_EXPOSURE,
    FEATURE_SHARPNESS,
    FEATURE_WHITE_BALANCE,
    FEATURE_HUE,
    FEATURE_SATURATION,
    FEATURE_GAMMA,
    FEATURE_SHUTTER,
    FEATURE_GAIN,
    FEATURE_IRIS,
    FEATURE_FOCUS,
    FEATURE_TEMPERATURE,
    FEATURE_TRIGGER,
    /* 19 reserved features */
    FEATURE_ZOOM,
    FEATURE_PAN,
    FEATURE_TILT,
    FEATURE_OPTICAL_FILTER,
    /* 12 reserved features */
    FEATURE_CAPTURE_SIZE,
    FEATURE_CAPTURE_QUALITY
    /* 14 reserved features */
};
#define FEATURE_MIN                 FEATURE_BRIGHTNESS
#define FEATURE_MAX                 FEATURE_CAPTURE_QUALITY
#define NUM_FEATURES                (FEATURE_MAX - FEATURE_MIN + 1)

/* Return values for visible functions*/
#define DC1394_SUCCESS               1
#define DC1394_FAILURE              -1
#define DC1394_NO_CAMERA            0xffff

/* Parameter flags for dc1394_setup_format7_capture() */
#define QUERY_FROM_CAMERA -1
#define USE_MAX_AVAIL     -2
#define USE_RECOMMENDED   -3

/* Yet another boolean data type */
typedef enum
{
    DC1394_FALSE= 0,
    DC1394_TRUE
} dc1394bool_t;
/*
	dc1394_feature_info  -- feature info for a specific camera feature
	
	Compatible with Linux DC1394 package.
*/
typedef struct __dc1394_feature_info_struct 
{
    unsigned int feature_id;
    dc1394bool_t available;
    dc1394bool_t one_push;
    dc1394bool_t absolute_capable;
    dc1394bool_t readout_capable;
    dc1394bool_t on_off_capable;
    dc1394bool_t auto_capable;
    dc1394bool_t manual_capable;
    dc1394bool_t polarity_capable;
    dc1394bool_t one_push_active;
    dc1394bool_t is_on;
    dc1394bool_t auto_active;
    char trigger_mode_capable_mask;
    int trigger_mode;
    dc1394bool_t trigger_polarity;
    int min;
    int max;
    int value;
    int BU_value;
    int RV_value;
    int target_value;

    dc1394bool_t abs_control;
    float abs_value;
    float abs_max;
    float abs_min;
} dc1394_feature_info;

#endif /* DEVDCCAMDEFS_H */
