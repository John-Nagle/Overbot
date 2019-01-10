//
//	devindex.h  --  device index for hot-pluggable devices
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
#ifndef DEVINDEX_H
#define DEVINDEX_H
#include <limits.h>
#include <string>
#include <map>
#include <vector>
#include <dirent.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <sys/resmgr.h>
#include "mutexlock.h"

class Devindex;													// forward
//
//	class Deventry  --  an entry in a devindex
//
class Deventry {
private:
	Devindex& m_index;										// owning devindex
	string m_name;													// device name
	unsigned int m_idcycle;									// inventory during which found
	iofunc_attr_t m_attr;											// resource manager file attributes
public:
	Deventry(Devindex& index);							// constructor
	virtual ~Deventry();											// destructor
	const string& getname() const  {return(m_name); }
	void setname(const string& name) { m_name = name; }	// set the name
	unsigned int getcycle() const { return(m_idcycle); }
	Devindex& getindex() const { return(m_index); }		// get owning index
	ino_t getinode() const { return(m_attr.inode);	 }	// inode access
	int getoffset() const { return(getinode() - 1);	}	// offset is always inode number - 1
	void stampcycle(); 											// stamp with bus inventory cycle on which last seen
	void setinode(ino_t inode);
	virtual bool safetodelete() const;						// true if safe to delete
	virtual int checkopenable() const;					// returns error code if not openable
	virtual bool purgeio();										// kill any activity on this device
	void builddirent(basic_string<uint8_t>& s);		// fill dirent into byte string
protected:																// called by subclasses
	virtual void initattrs();										// initialize the default device attributes 
public:
	static void canonizename(string& name);		// convert name to canonical form, in place
	//	Resource manager operations
	virtual int io_open(resmgr_context_t* ctp, io_open_t *msg, iofunc_attr_t *attr, void *extra);
	virtual int io_close_ocb(resmgr_context_t *ctp, void* reserved, RESMGR_OCB_T* ocb);
	virtual int io_read(resmgr_context_t *ctp, io_read_t* msg, RESMGR_OCB_T* ocb);
	virtual int io_write(resmgr_context_t *ctp, io_write_t* msg, RESMGR_OCB_T* ocb);
	virtual int io_devctl(resmgr_context_t *ctp, io_devctl_t* msg, RESMGR_OCB_T* ocb);
};
//
//	class Devindex  -- an index of devices
//
//	Owns the Deventry items
//
class Devindex {
private:
	vector<Deventry*> m_devs;							// table of devices
	unsigned int m_idcycle;									// current inventory cycle
	bool m_needremove;										// need to remove old devices
protected:
	ost::Mutex m_lock;												// lock when updateing the table of active devices
public:
	Devindex();														// constructor
	~Devindex();													// destructor
	Deventry* find(const string& name);				// look up by name
	Deventry* find(const char* name);					// look up by name, C string
	void insert(Deventry* dev);								// insert by name, no dup check
	void startidcycle();											// begin a new ID cycle
	void needremove();											// set remove needed
	void removeolddevs();										// remove devs from older cycles
	unsigned int getcycle() const { return(m_idcycle); }
	unsigned int getdevcount() const { return(m_devs.size()); }
	ost::Mutex& getlock() { return(m_lock); }			// get our lock
	Deventry* getdev(unsigned int ix) { return(m_devs[ix]); }
	Deventry* getdevbyinode(unsigned int inode);// get by inode number
protected:
	const vector<Deventry*>& getdevs() const { return(m_devs); }	// get entire dev list

private:
	bool purgeio();													// purge any pending I/O operations
};
#endif // DEVINDEX_H

