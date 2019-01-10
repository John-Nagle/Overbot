//
//	devindex.cpp  --  device index for hot-pluggable devices
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
#include <sys/neutrino.h>
#include <limits.h>
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <errno.h>
#include <dirent.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <sys/resmgr.h>
#include <string.h>
#include <assert.h>
#include "devindex.h"
//
const bool verbose = 1;									// debug mode
//
//	Useful functions
//
#define ALIGN(x) (((x) +3) & ~3)						// align on 4-byte boundary
//
//	Class Deventry implementation
//
//	Constructor
//
Deventry::Deventry(Devindex& index)
	: m_index(index),								// owning object
	m_idcycle(0)										// cycle on which inventoried
{	
	initattrs();											// initialize default attributes
}
//
//	Destructor
//
Deventry::~Deventry()
{	assert(m_attr.count == 0);				// all opens must be closed on delete
}
//
//	setinode -- set the inode number
//
//	Must be done only once. 
//
void Deventry::setinode(ino_t inode)
{	m_attr.inode = inode;									// set in attr structure, so open and dir see it.
}
//
//	safetodelete  -- true if safe to delete this item
//
bool Deventry::safetodelete() const
{	return(m_attr.count == 0);	}						// OK to delete item if not in use at all.
//
//	openable  -- returns 0 if OK to open, otherwise error
//
int Deventry::checkopenable() const
{	//	Check for device not found on current bus reset, but not yet purged.
	if (getindex().getcycle() != getcycle()) return(ENODEV);		// no such device
	return(EOK);										// normal
}
//
//	stampcycle -- stamp with cycle number
//
void Deventry::stampcycle()
{	m_idcycle = getindex().getcycle(); }

//
//	purgeio  -- purge I/O activity on this device
//
bool Deventry::purgeio()
{	return(true);	}									// for now ***TEMP***
//
//	builddirent  --  build a directory entry, for directory reads
//
//	A directory entry consists of a fixed-length header plus
//	a variable-length string, padded out with one or more nulls
//	to a multiple of the alignment. Since this is not a standard
//	C or C++ structure, we have to do some marshalling here.
//	
//	Result returned as a string of bytes.
//
//	Allocates, can throw out out of memory exception, exception safe.
//
void Deventry::builddirent(basic_string<uint8_t>& s)
{	
	if (getinode() == 0) return;								// empty, ignore this slot
	struct dirent d;													// build directory entry here
	d.d_ino = getinode();										// inode number
	d.d_offset = getoffset();									// offset into directory
	d.d_namelen = m_name.size();							// length of name, without nulls
	const size_t dirheadersize = &d.d_name[0]-(char*)&d;	// length of dirent header
	size_t length = dirheadersize + d.d_namelen;	// length without filler
	size_t filler = ALIGN(length+1) - length;			// filler needed to achieve alignment - at least one null
	d.d_reclen = dirheadersize + d.d_namelen+filler;		// record length with filler
	//	Copy fields to string
	s.assign(d.d_reclen,0);										// create filled with zeroes
	unsigned int outpos = 0;									// position in output string
	uint8_t* p = (uint8_t*)(&d);								// treat string as bytes
	for (unsigned int i=0; i<dirheadersize; i++,outpos++)
	{	s[outpos] = p[i];	}										// fill into string
	for (unsigned int i=0; i<m_name.size(); i++,outpos++)
	{	s[outpos] = m_name[i];	}							// fill into string
}
//
//	Resource manager operations
//
//
//	initattrs  --  initialize the attribute structure
//
//	Subclass can override if desired.
//
void Deventry::initattrs()
{	iofunc_attr_init(&m_attr, S_IFREG | 0666, 0 , 0);	// mode 666, owned by root, group 0
	m_attr.inode = 0;												// no inode yet, unused slot
}
//
//	io_open  -- open of device
//
//	Note that this means an open is being tried, not that it succeeded.
//
int  Deventry::io_open(resmgr_context_t* ctp, io_open_t *msg, iofunc_attr_t *attr, void *extra)
{
	int stat = checkopenable();									// check whether device openable
	if (stat) return(stat);												// return stat if fail
	////if (verbose) printf("Opening \"%s\"\n",getname().c_str());				// debug output
	return(iofunc_open_default(ctp,msg,&m_attr,extra));	// let open default finish the job
}
//
//	io_close_ocb  --  last close of device file descriptor.
//
int Deventry::io_close_ocb(resmgr_context_t *ctp, void* reserved, RESMGR_OCB_T* ocb)
{	int stat = iofunc_close_ocb_default(ctp,reserved,ocb);	// do normal close actions.
	////if (verbose) printf("Closed \"%s\" (%d users left)\n",getname().c_str(),m_attr.count);				// ***TEMP***
	if ((checkopenable() != EOK) && safetodelete())	// if unplugged and no users left
	{	m_index.needremove();								// schedule a dead device purge
	}
	return(stat);														// normal
}
//
//	io_read  --  read
//
int Deventry:: io_read(resmgr_context_t *ctp, io_read_t* msg, RESMGR_OCB_T* ocb)
{	assert(ocb->attr == &m_attr);							// must be on right device
	int stat = iofunc_read_verify(ctp,msg,ocb,0);	// determine whether read is allowed
	if (stat) return(stat);											// if no read access, fails
	////printf("Reading %s\n",getname().c_str());			// ***TEMP***
	////printf("Use count: %d\n",m_attr.count);				// ***TEMP***
	return(ENODEV);												// ***TEMP***
}
//
//	io_write  -- write
//
int  Deventry:: io_write(resmgr_context_t *ctp, io_write_t* msg, RESMGR_OCB_T* ocb)
{
	int stat = iofunc_write_verify(ctp,msg,ocb,0);	// determine whether read is allowed
	if (stat) return(stat);											// if no read access, fails
	return(ENODEV);												// ***TEMP***
}
//
//	class Devindex implementation -- an index of devices
//
//	Owns the Deventry items
//
//	Constructor
//
Devindex::Devindex()
	: m_idcycle(0),													// init bus reset cycle number
	m_needremove(false)										// no need to remove old devices yet
{}
//
//	Destructor
//
Devindex::~Devindex()
{	//	Try to stop any pending I/O. 
	//	If we're here, we're exiting the driver, so QNX will finish any cleanup.
	purgeio();															// purge all pending I/O
}
//
//	purgeio -- purge all pending I/O
//
bool Devindex::purgeio()
{	bool success = true;
	for (unsigned int i = 0; i <m_devs.size(); i++)
	{	success &= m_devs[i]->purgeio();	}			// purge all pending I/O
	return(success);												// true if all purging succeeded
}
//
//	startidcycle -- start a new bus reset cycle
//
void Devindex::startidcycle()
{	m_idcycle++;	}												// advance count of bus resets
//
//	needremove -- set remove-needed flag
//
//	Set when a device close finds a dead device that can be removed.
//	Ordinarily, devices are purged at bus reset, but devices that are
//	still open can't be removed yet.
//
void Devindex::needremove()
{	m_needremove = true;	}
//
//	removeolddevs  --  purge old entries not found on the current reset cycle
//
//	This removes out all device entries for unplugged devices.
//	But they stay in the table, half-dead, if still open by something.
//	Called only from the main resource manager task.
//
void Devindex::removeolddevs()
{	if (!m_needremove) return;										// if not needed, skip
    ost::MutexLock lok(getlock());									// lock device index from here to end.
	m_needremove = false;											// used up need for remove
	//	Purge all devices not found on this cycle, and thus removeable.
	for (unsigned int i=0; i<m_devs.size(); i++)				// for all devices
	{	Deventry* dev = m_devs[i];									// this device
		if (!dev) continue;													// open slot
		if (dev->getcycle() == getcycle()) continue;		// if found on current cycle, don't purge
		if (!dev->safetodelete()) continue;						// if not yet deletable, don't purge
		//	OK to delete - do it
		delete(dev);
		m_devs[i] = 0;														// remove from table
	}
}
//
//	find  --  look up device entry by name
//
//	Linear search
//
Deventry* Devindex::find(const char* name)
{  ost::MutexLock lok(getlock());									// lock device index from here to end.
	for (unsigned int i=0; i<m_devs.size(); i++)				// for all devices
	{	Deventry* dev = m_devs[i];									// this device	
		if (!dev) continue;													// open slot
		if (dev->getname() == name)								// if find
		{	return(dev);	}													// return find
	}
	return(0);																	// no find
}
//
//	find -- alternate form, with C++ string
//
Deventry* Devindex::find(const string& name)
{	return(find(name.c_str()));	}

//
//	insert  --  insert new device entry
//
//	Does not check for duplicates.  Do a find first.
//
void Devindex::insert(Deventry* dev)
{	ost::MutexLock lok(getlock());						// may be a recursive lock, but that's allowed.
	dev->stampcycle();										// stamp with current cycle
	for (unsigned int i=0; i<m_devs.size(); i++)	// for all entries
	{	if (m_devs[i] == 0)									// if found an open slot
		{	dev->setinode(i+1);								// inode number is slot+1
			m_devs[i] = dev;									// insert it
			return;
		}
	}
	//	No space, need a new slot
	dev->setinode(m_devs.size()+1);				// 1+slot
	m_devs.push_back(dev);								// add at end
}
//	
//	getdevbyinode -- get device item by inode number
//
//	Index in table is inode-1.
//
Deventry* Devindex::getdevbyinode(unsigned int inode)
{	if (inode == 0) return(0);								// if zero inode, no dev
	unsigned int ix = inode-1;							// convert inode to index
	if (ix >= m_devs.size()) return(0);				// if inode out of range, 0
	return(getdev(ix));										// get dev
}
//
//	canonizename  --  convert name to canonical form
//
//	Converts to lower case, no spaces
void Deventry::canonizename(string& name)
{	for (unsigned int i=0; i<name.size(); i++)
	{	char ch = name[i];
		ch = tolower(ch);											// force lower case
		switch (ch) {												// avoid undesirable characters in device names
		case '\n':
		case '\0':
		case '/':																
		case '\\':
		case ' ': 
			ch = '_';														// convert to underscore
			break;
			continue;
		default:	
			break;
		}
		name[i] = ch;												// put back modified character
	}
}

//
//	io_devctl  -- device control
//
int  Deventry::io_devctl(resmgr_context_t *ctp, io_devctl_t* msg, RESMGR_OCB_T* ocb)
{
	return(ENODEV);												// ***TEMP***
}

