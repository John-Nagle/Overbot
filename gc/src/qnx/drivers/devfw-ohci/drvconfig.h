//
//	drvconfig.h  --  low level driver for OHCI adapter
//
//	John Nagle
//	Animats
//	March, 2003
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
#ifndef DRVCONFIG_H
#define DRVCONFIG_H
//
//	Configuration constants
//
const unsigned int DEFAULT_GAP_COUNT = 63;		// IEEE 1394 gap count
const size_t NBR_DBUF_ASY = 4;								// async output bufs - small
const unsigned int MAX_ADAPTERS = 8;					// max number of adapters
#endif // DRVCONFIG_H