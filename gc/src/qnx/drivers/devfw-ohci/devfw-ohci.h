//
//	QNX support for IEEE-1394 devices
//
//	John Nagle
//	Animats
//	January, 2003
//
//	This file is to be included by subdrivers of "ohci-1394".
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
#ifndef DEVFW_OHCI_H
#define DEVFW_OHCI_H
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <sys/resmgr.h>
typedef unsigned int quadlet_t;
typedef unsigned long long u_int64_t;

#endif // DEVFW_OHCI_H