//
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
#include <valarray>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <sys/resmgr.h>
#include <string.h>
#include "devindex.h"										// device index
#include "businfo.h"
//
//	TODO:
//	1.	Add try/catch clauses to catch out-of-memory conditions.
//
//	Constants
//	
const char* basepath = "/dev/fw0";					// our pathname space starts here
//
//	Useful functions
//
//
//	The FireWire bus, and its associated device index.
//	We currently support only one local adapter.
//
static Businfo devices(0);									// bus info for adapter 0
//
//	initdevices -- initialize device table
//
static void initdevices()
{
	int stat = devices.startup();							// try to start driver
	if (stat) 															// if fail
	{	perror("Unable to initialize IEEE-1394 driver");
		exit(EXIT_FAILURE);
	}
}
//
//	openrequest  -- process an open request
//
static int openrequest(resmgr_context_t* ctp, io_open_t *msg,
	iofunc_attr_t *attr, void *extra)
{
	//	Check for a request for the base directory.
	if (msg->connect.path[0] == '\0')					// if empty path, is base directory
	{	return(iofunc_open_default(ctp,msg,attr,extra));	}	// do default thing
	//	Request for an actual device open
	Deventry *dev = devices.find(msg->connect.path)	;	// look up 
	if (!dev)														// if no find
	{	return(ENOENT);	}									// no such entry
	//	Device name exists; check whether we have sufficient privileges to open it.
	int stat = dev->io_open(ctp,msg,attr,extra);		// do the open
	return(stat);
}
//
//	directoryreadrequest -- request to read a directory
//
//	This returns the devices we know about.
//	Since this is a hot-pluggable device type, the list can change.
//
static int directoryreadrequest(resmgr_context_t* ctp, io_read_t* msg,
	iofunc_ocb_t* ocb)
{	
	int sts = iofunc_read_verify(ctp,msg,ocb,NULL);	// standard checks
	if (sts != EOK)	{return(sts);	}								// fails
	//	The returned message is a packed sequence of variable-length dirent entries
	basic_string<uint8_t> reply_msg;							// reply constructed here, in byte string
	for (; ocb->offset < devices.getdevcount() ; ocb->offset++)	// fill in until full, or done
	{	Deventry* dev = devices.getdev(ocb->offset);				// get relevant device entry
		if (!dev) continue;												// skip if open slot
		basic_string<uint8_t> dirmsgpart;
		dev->builddirent(dirmsgpart);							// get dirent as byte string, with alignment padding
		if (reply_msg.size() + dirmsgpart.size() > size_t(msg->i.nbytes))	// if will not fit
		{	break;	}														// full, done
		reply_msg.append(dirmsgpart);						// not full, add to dirent block
	}
	//	Return directory entries to sender
	MsgReply(ctp->rcvid, reply_msg.size(), reply_msg.data(), reply_msg.size());	// size is reply code
	return(_RESMGR_NOREPLY);									// We have replied; done
}
//
//	readrequest --  process a read request
//
//	This includes requests to read the directory.
//
static int readrequest(resmgr_context_t* ctp, io_read_t* msg, iofunc_ocb_t* ocb)
{	//	Check for directory read
	if (S_ISDIR(ocb->attr->mode))								// handle directory read
	{	return(directoryreadrequest(ctp,msg,ocb)); }	// handle locally
	if (S_ISREG(ocb->attr->mode))								// handle regular read
	{	Deventry* dev = devices.getdevbyinode(ocb->attr->inode);	// look up device entry
		if (!dev) return(ENODEV);									// if no dev, fails
		return(dev->io_read(ctp,msg,ocb));					// pass to device level
	}
	return(EBADF);														// handle bogus request
}
//
//	writerequest --  process a read request
//
//	This includes requests to write the directory, although we don't allow that.
//
static int writerequest(resmgr_context_t* ctp, io_write_t* msg, iofunc_ocb_t* ocb)
{	//	Check for directory read
	if (S_ISDIR(ocb->attr->mode))								// handle directory read
	{	return(EINVAL); }												// reject it
	if (S_ISREG(ocb->attr->mode))								// handle regular read
	{	Deventry* dev = devices.getdevbyinode(ocb->attr->inode);	// look up device entry
		if (!dev) return(ENODEV);									// if no dev, fails
		return(dev->io_write(ctp,msg,ocb));					// pass to device level
	}
	return(EBADF);														// handle bogus request
}
//
//	devctlrequest --  process a read request
//
//	This includes requests to operate on the directory, although they're rejected.
//
static int devctlrequest(resmgr_context_t* ctp, io_devctl_t* msg, iofunc_ocb_t* ocb)
{	
	//	Check for directory devctl
	if (S_ISDIR(ocb->attr->mode))								// handle directory devctl
	{	return(iofunc_devctl_default(ctp,msg,ocb)); }	// standard action

	Deventry* dev = devices.getdevbyinode(ocb->attr->inode);	// look up device entry
	if (!dev) return(ENODEV);										// if no dev, fails
	return(dev->io_devctl(ctp,msg,ocb));					// pass to device level
}
//
//	closeocbrequest --  process a final close request
//
//	This is the last close on a file, not the final close.
//	This includes requests to close the directory.
//	No reply is required; all the opens are gone. This gives us a chance to clean up.
//
static int closeocbrequest(resmgr_context_t* ctp, void* reserved, iofunc_ocb_t* ocb)
{	//	Check for directory read
	if (S_ISDIR(ocb->attr->mode))								// handle directory close
	{	return(OK); }														// no action required
	if (S_ISREG(ocb->attr->mode))								// handle regular read
	{	Deventry* dev = devices.getdevbyinode(ocb->attr->inode);	// look up device entry
		if (!dev) return(ENODEV);									// if no dev, fails
		return(dev->io_close_ocb(ctp,reserved,ocb));	// pass to device level
	}
	return(EBADF);														// handle bogus request
}
//
//	
//	eventhandler  --  handles a resmgr event
//	
//	We do this instead of using resmgr_handler because we want to call our purge routine
//	after each I/O.
//
static int eventhandler(resmgr_context_t* ctp)
{
	int stat = resmgr_handler(ctp);						// handle the event
	devices.removeolddevs();							// purge anything now closed and unplugged
	return(stat);													// returns status
}

//	main  -- main program
//
int main(int argc, char* argv[])
{
	resmgr_attr_t			resmgr_attr;
	resmgr_io_funcs_t		io_func;							// callbacks for I/O functions
	resmgr_connect_funcs_t	connect_func;		// callbacks for connect functions
	iofunc_attr_t			attr;
	
	dispatch_t* dpp = dispatch_create();			// create the dispatch structure
	if (!dpp)														// if not created
	{	perror("dispatch_create failed");				// fails
		exit(EXIT_FAILURE);									// fail all
	}
	//	Set up thread pool
	thread_pool_attr_t pool_attr = { 0 };				// thread pool starts out as zeroes
	pool_attr.handle = dpp;								// set up pool attributes
	pool_attr.context_alloc = resmgr_context_alloc;
	pool_attr.block_func = resmgr_block;
	pool_attr.handler_func = eventhandler;
	pool_attr.context_free = resmgr_context_free;
	pool_attr.lo_water = 1;									// at least this many threads
	pool_attr.hi_water = 4;									// ???
	pool_attr.increment = 1;
	pool_attr.maximum = 32;								// this many threads max
	thread_pool_t* tpp = thread_pool_create(&pool_attr, POOL_FLAG_EXIT_SELF);
	if (!tpp)
	{	perror("Unable to create thread pool."); return(EXIT_FAILURE);	}
	//	Bind default functions into the outcall tables
	iofunc_func_init(_RESMGR_CONNECT_NFUNCS, &connect_func,
						_RESMGR_IO_NFUNCS, &io_func);
	//	Hook up our functions to the outcall tables
	io_func.read = readrequest;							// read operations
	io_func.write = writerequest;						// write operations
	io_func.devctl = devctlrequest;						// devctl operations
	io_func.close_ocb = closeocbrequest;			// close operations
	//	Hook up functions that use pathnames.
	connect_func.open = openrequest;				// open operations
	//	Initialize resource manager attributes
	memset(&resmgr_attr,0,sizeof(resmgr_attr));	// clear to zero
	resmgr_attr.nparts_max = 1;
	resmgr_attr.msg_max_size = 2048;
	//	Set the properties of this directory
	iofunc_attr_init(&attr, S_IFDIR | 0555,0,0);		// directory owned by root, group root, mode 0555
	attr.inode = 1;
	attr.nbytes = 0;
	//	Link ourself into the pathname space
	int stat = resmgr_attach(dpp,&resmgr_attr, basepath,
		_FTYPE_ANY, _RESMGR_FLAG_DIR,
		&connect_func, &io_func, &attr);
	if (stat ==-1)
	{	perror("Unable to attach resource manager to path");
		exit(EXIT_FAILURE);									// fail all
	}
	//	Allocate a context
	resmgr_context_t* ctp = resmgr_context_alloc(dpp);	// allocate a context
	if (!ctp)
	{	perror("Unable to allocate resource manager context.");
		exit(EXIT_FAILURE);
	}
	//	Initialize device table
	initdevices();													// initialize device table
	//	Event loop
	thread_pool_start(tpp);									// let the thread pool do it
#ifdef OBSOLETE
	for (;;)
	{	ctp = resmgr_block(ctp);							// get next event message
		if (!ctp)
		{	perror("Unable to resmgr_block");
			exit(EXIT_FAILURE);
		}
		resmgr_handler(ctp);								// handle the event
		devices.removeolddevs();						// purge anything now closed and unplugged
	}
#endif // OBSOLETE
	return(EXIT_SUCCESS);									// normally unreachable
}
