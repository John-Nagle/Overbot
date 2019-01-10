//
//	devfactory.cpp  -- FireWire (IEEE-1394) driver for OHCI-compatible devices
//
//	Creates deventry devices for specific devices.
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
#include <dirent.h>
#include <valarray>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <sys/resmgr.h>
#include <string.h>
#include "devfactory.h"
#include "businfo.h"
//	Includes for specific drivers go here
#include "devdccam.h"
//
//	Singleton for all device types
//
Devfactory devicefactory;						// the device factory
//
//	makedevice -- device factory
//
Busentry* Devfactory::makedevice(Businfo& bus,
		uint32_t initialnodenumber, uint32_t vendor, uint32_t hiid, uint32_t lowid)
{
	//	Dumb version - always generates a DC camera device for now.
	return(new Devdccam(bus,initialnodenumber,vendor,hiid,lowid));
}
