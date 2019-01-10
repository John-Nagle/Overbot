//
//	drvadapter.cpp  --  low level driver for OHCI adapter
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
#include <errno.h>
#include "mindreadyforcpp.h"
#include "llaError.h"
#include "drvadapter.h"
#include "utilities.h"

uint32_t Drvadapter::m_adaptersactive = 0;						// number of adapters currently active
//
//	Drvadapter --  low-level driver for one adapter
//
//	This is currently an encapsulation of the Mindready LLA, but could be converted to use a non-LLA approach.
//
//
//	llacheck  -- check return code from LLA, convert to POSIX value, print error message
//
int Drvadapter::llacheck(int rc, const char msg[])
{	if (rc == 0) return(EOK);												// no error, normal case
	logprintf("ERROR from low-level driver: %s (%08x) from function \"%s\"\n",Err2Str(rc), rc ,msg);
	switch (rc) {
	case SDERR_SYS_UNAVAILABLE:									// retryable errors
		return(EBUSY);			// must retry operation 

    case SDERR_INVALID_ADDR:											// bad parameter errors
    case SDERR_INVALID_ADP_INDEX:
    case SDERR_INVALID_CHAN_LIST:
    case SDERR_INVALID_CMD:
    case SDERR_INVALID_CTX_HANDLE:
    case SDERR_INVALID_DESC:
    case SDERR_INVALID_DEVDESC:
    case SDERR_INVALID_FLAGS:
    case SDERR_INVALID_GAP_COUNT:
    case SDERR_INVALID_GEN_COUNT:
    case SDERR_INVALID_HANDLE:
    case SDERR_INVALID_LICENCE:
    case SDERR_INVALID_PACKET:
    case SDERR_INVALID_PARAM	:
    		return(EINVAL);
    		
    	default:																			// all other errors
    		return(EIO);
	}
}
//
//	Destructor
//
Drvadapter::~Drvadapter()
{	cleanup();	}
//
//	cleanup
//
void Drvadapter::cleanup()
{	if (m_lla == 0) return;												// not open, cleanup not needed
	//	Clean up the four subsections
	m_asySendCtxt.cleanup();	
	m_asyRecvCtxt.cleanup();	
	m_isoSendCtxt.cleanup();	
	m_isoRecvCtxt.cleanup();
	//	Clean up at adapter level
	busResetRecvStop();												// no more bus reset events
	llaDetach(m_lla);
	m_lla = 0;																	// no more adapter
	m_adaptersactive--;													// decr. number of adapters active
	if (m_adaptersactive == 0)										// if last adapter is closing
	{	llaRelease();	}														// final close of LLA system
}
//
//	init -- initalization
//
int Drvadapter::init()
{	if (m_lla) return(EINVAL);											// already open, invalid request
	if (m_adaptersactive == 0)										// if first init
	{	int stat = llacheck(llaInit(),"initialization");				// initialize LLA
		if (stat != EOK) return(stat);									// fails
	}
	if (m_lla) return(EINVAL);											// already open, invalid request
	m_lla = llaAttach(m_adapterid);									// attach to adapter
	if (!m_lla) return(ENODEV);										// no such device
	m_adaptersactive++;												// increment active adapters
	return(EOK);																// done
}
//
//	All of these return POSIX error codes.
//	Isochronous functions
int Drvadapter::isoEnableCycleMaster()
{	return(llacheck(llaEnableCycleMaster(getlla()),"Enable self as isochronous cycle master"));	}
int Drvadapter::isoDisableCycleMaster()
{	return(llacheck(llaDisableCycleMaster(getlla()),"Disable self as isochronous cycle master"));	}
//
//	Node info for this node (the OHCI adapter)
//
//	getPresentStatus -- get present status for this adapter  from the LLA
//
int Drvadapter::getPresentStatus(llaPresentStatus_t& presentStatus)
{	return(llacheck(llaGetPresentStatus(getlla(),&presentStatus),"getPresentStatus"));	}
//
//	setForceRootFlag  -- set force-root mode. Next bus reset will make this node the root.
//
int Drvadapter::setForceRootFlag(bool force)
{	if (force)
	{	return(llacheck(llaSetForceRoot(getlla()),"force self to be root node")); }
	else
	{	return(llacheck(llaClearForceRoot(getlla()),"unset force self to be root node")); }
}
//
//	getPhyID  -- get physical ID of this node
//
int Drvadapter::getPhyID(llaPhyId_t&  phyID)
{	return(llacheck(llaGetPhyId(getlla(),&phyID),"getPhyID"));	}
//
//	getNodeID  -- get node ID info for this adapter
//
int Drvadapter::getNodeID(llaNodeId_t& nodeID)
{	return(llacheck(llaGetNodeId(getlla(),&nodeID),"getNodeID"));	}
//
//	getGenerationCount  -- get bus reset generation count
//
//	LLA access functions
//
//
//	GetGenerationCount  --  get bus reset generation count
//
int Drvadapter::getGenerationCount(uint32_t& gencount)
{ return(llacheck(llaGetGenerationCount (getlla(), &gencount),"getGenerationCount")); }
//
//	setConfigContender  -- set/clear want to be isochronous resource manager
//
int Drvadapter::setConfigContender(bool wantToBeIRM)					// want to be IRM?
{	if (wantToBeIRM)
	{ return(llacheck(llaSetContender(getlla()),"set 'want to be isochronous bus manager contender'")); }
	else
	{ return(llacheck(llaClearContender(getlla()),"clear 'want to be isochronous bus manager contender'")); }
}
//
//	setGapCount -- set isochronous gap count
//
int Drvadapter::setGapCount(uint32_t value)								
{ return(llacheck(llaSetGapCount(getlla(), value),"set gap count")); }
//	Control and status registers
//
//	readCSR  -- read control and status register, this node
//
int Drvadapter::readCSR(unsigned offset, uint32_t& quadletVal)
{ return(llacheck(llaCsrRead(getlla(), offset, &quadletVal),"read CSR")); }
//
//	writeCSR  -- write control and status register, this node
//
int Drvadapter::writeCSR(unsigned offset, uint32_t quadletVal)
{ return(llacheck(llaCsrWrite(getlla(), offset, quadletVal),"write CSR")); }
//
//	lockCSR  -- compare and swap control and status register, this node
//
int Drvadapter::lockCSR(unsigned offset, uint32_t& quadletVal, uint32_t&	newQuadletVal)
{ return(llacheck(llaCsrLock(getlla(), offset, quadletVal,&newQuadletVal),"lock CSR")); }
//
//	Isochronous support
//
//	isoRecvStart -- start isochronous reception
//
int Drvadapter::isoRecvStart(uint32_t channel, uint32_t framesize, uint32_t totalsize, bool waitforsync)
{	return(m_isoRecvCtxt.start(channel, framesize,totalsize,waitforsync)); }		// pass down a level
//
//	isoRecvStop  -- stop isochronous reception
//
int Drvadapter::isoRecvStop(uint32_t channel)
{	return(m_isoRecvCtxt.stop(channel));											// shut down
}