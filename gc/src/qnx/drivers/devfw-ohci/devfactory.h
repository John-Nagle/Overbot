//
//	devfactory.h  --  device entry generator for hot-pluggable devices
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
#ifndef DEVFACTORY_H
#define DEVFACTORY_H
#include "mindreadyforcpp.h"								// WORKAROUND for sizeof(bool) problem
#include <limits.h>
#include <string>
#include <vector>
#include "devindex.h"
#include "fwadapter.h"
//
class Devindex;
class Busentry;
class Businfo;
//
//	class Devfactory  --  generates a new device entry
//
class Devfactory {
public:
	Busentry* makedevice(Businfo& bus, 
		uint32_t nodenumber, uint32_t vendor, uint32_t hiid, uint32_t lowid);
};
//
//	Singleton for all device types
//
extern Devfactory devicefactory;						// the device factory
#endif // DEVFACTORY_H