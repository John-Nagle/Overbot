//
//	fwadapter.cpp  --  FireWire adapter interface.
//
//	This talks to the hardware interface of the local adapter.
//
//	This version uses the Mindready library.
//	Try to keep all Mindready-specific code in this module.
//
//	John Nagle
//	Animats
//	February, 2002
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
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <assert.h>
#include "mindreadyforcpp.h"					// required to compile with cpp
#include "mutexlock.h"
#include "fwadapter.h"
#include "businfo.h"
#include "utilities.h"
//
//	Constants
//
//	Thread priorities for Mindready LLA
const char intHandlerPriority[] = "15";											
const char taskHandlerPriority[] = "14";

const bool verbose = true;								// enable debug
//
//	Static object variables
//
bool  Fwadapter:: m_terminate = false;                		  	    // thread should terminate																				// adapter info
pthread_t		 Fwadapter:: m_thread;                       	     	// thread that processes events from adapters
uint32_t       Fwadapter:: m_nbrOfAdp = 0;                   	    	// number of adapters on this machine
Fwadapter*  Fwadapter:: m_adapters[MAX_ADAPTERS];	// per-adapter info
ost::BoundedBuffer<Event_t, eventQueueSize> Fwadapter::m_eventqueue;	// the event queue


//
//	Constructor
//
Fwadapter::Fwadapter(Businfo& bus, uint32_t adapterid)
	: Drvadapter(adapterid),
	m_owner(bus),												//	tie us to the parent object
	m_adapterid(adapterid),
	m_used(false),
	m_iamroot(false)
{	assert(adapterid < MAX_ADAPTERS);				// must have valid adapter number
	m_adapters[m_adapterid] = this;						// enter into adapter table
	if (adapterid >= m_nbrOfAdp) m_nbrOfAdp = adapterid+1;	// increase number of adapters if needed
}
//
//	Destructor
//
Fwadapter::~Fwadapter()
{	
	m_adapters[m_adapterid] = 0;							// remove from adapter table
}
//
//	initadapter  -- general startup
//
int Fwadapter::initadapter()
{
    //	Set thread priorities
    // 	Key priorities must be set in environment variables for Mindready LLA.
    //	These can be overridden from the command line.
    setenv("OHCI_INT_HANDLER_PRIORITY",intHandlerPriority,0);
    setenv("OHCI_TASK_HANDLER_PRIORITY",taskHandlerPriority,0);
	//	Open the 1394 device
	int stat = init();											// init at driver level
	if (stat!=EOK)
	{	logprintf("ERROR: unable to initialize low-level driver\n");
		return(stat);
	}
	//	We are willing to be the isochronous resource manager
	stat = setConfigContender(true);
	if (stat != EOK) return(stat);
	//	Set "gap count"
	stat = setGapCount(DEFAULT_GAP_COUNT);
	if (stat != EOK) return(stat);
	//	Start bus reset event reception
	stat = busResetRecvStart();
	if (stat != EOK) return(stat);
	//	Create event thread.
   	stat = pthread_create(&m_thread, NULL, startwaitforevent, this);
	if (stat!=EOK)
   	{	logprintf("ERROR: Unable to create thread.\n");
   		return(stat);
   	}
////#ifdef OBSOLETE
    /* Immediately do a bus reset so we can get usefull information */
    /* via the self id packets (root node number, number of node on */
    /* the bus, etc...)                                             */
    stat = busReset(false);									// force a bus reset
////#endif // OBSOLETE
    return (EOK);
}



//
//	cleanupadapter  --  release all adapters and LLA
//
void Fwadapter::cleanupadapter()
{
	m_terminate = true;													// tell thread to terminate
    usleep(10000);
	cleanup();																	// shut down everything
}




//
//	processevent  -- process an event for this adapter
//
void Fwadapter::processevent(Event_t& event)
{	
	 switch(event.type)													// fan out on event type
     {
		 case EVENT_NONE:
			break;

		case EVENT_IN_ASY_PK:
			processasypkt(event.info.asyPk,event.buffer);
			break;

		case EVENT_IN_ISO_PK:
			m_owner.processisopkt(event.info.isoPk.hdr,event.buffer);
			break;

		case EVENT_IN_ISO_FRAME:
			m_owner.processisoframe(event.info.isoFr,event.buffer);
			break;

		case EVENT_BUS_RESET:
			processbusresetevent(event.info.busReset);
			break;
	
		default:
			logprintf("ERROR: Unknown event type (%d)\n", event.type);
        }
}
//
//	WaitForEvent  -- wait for the driver to send us an event.
//
//	This is the worker function of a thread.
//
//	The driver and this program share a memory area containing a
//	circular buffer. This thread reads events from that buffer and
//	calls appropriate action functions.
//
//	This appears to be sound unless the event buffer wraps around.
//
void Fwadapter::waitforevent()
{	SetThreadPriority(13,SCHED_RR);									// needs a high priority
	//	Event loop
	//	***IF WE NEED A TIMEOUT SYSTEM, THIS IS WHERE TO DO IT***
	while (!m_terminate)														// until program dies
	{	Event_t event;															// working event
		m_eventqueue.get(event);										// get an event
		processevent(event);												// handle it
        if (event.buffer)dbufFree(event.buffer);					// release associated dbuf, if needed
	}	
	//	Event loop has exited. Clean up.
    //	When we return, the thread exits automatically.
}
//
//	putevent -- post an event to the event queue
//
//	Never blocks; may lose events, but will tally them
//
bool Fwadapter::putevent(const Event_t& event)
{	bool success = m_eventqueue.tryput(event);								// put the event
	if (success) return(true);															// normal case
	if (event.buffer) dbufFree(event.buffer);									// if lost, free associated dbufs
	logprintf("Event queue full; event lost.\n");								// report
	return(false);																			// failed to puts
}
//
//	Event handlers
//
//
//	processasypkt  --  process one incoming async packet
//
//	Assumes the data fit in the short payload area of an event.
//	The dbuf is not examined. So only quadlet async packets are accepted.
//
void Fwadapter::processasypkt(const AsyPkInfo_t& pkInfo, const dbuf_t* buffer)
{	
	m_owner.asyncrecv(pkInfo.hdr,pkInfo.shortpayload,pkInfo.hdr.dataLength);	// handle the event
}
//
//	processbusresetevent  --  process a bus reset event.
//
//	This results in re-inventorying the bus.  But we can't inventory the
//	bus from this thread; inventory requires bus activity, which requires
//	events, which require this thread to receive them.  So all we do here
//	is what has to be done immediately.
//
//	NOTE: Called from event process. 
//
void Fwadapter::processbusresetevent(const BusResetInfo_t& busResetInfo)
{
    //	We are processing a bus reset of one adapter.
    //	Get new node ID after bus reset 
	llaNodeId_t nodeId;													// the node ID, in packet format
	int stat = getNodeID(nodeId);									// get the node ID
	if (stat != EOK) return;												// couldn't get node ID
	m_nodeinfo.nodeId = nodeId;									// save our new node ID 
    	m_owner.notebusreset();											// note inventory needed
}
//
//	updaterominfo  --  update ROM info for an adapter
//
int Fwadapter::updaterominfo()
{
    int  err = m_owner.readlocalrominfo(m_nodeinfo);
	if (err)
	{
		return(ENODEV);									// no such device
	}
	return(EOK);
}
//
//	isnodeouradapter --  is this node one of our adapters
//
//	If true, we can talk to it locally.
//
bool Fwadapter::isnodeouradapter(uint32_t node)
{	
	//	Is the current node one of our own adapters, on this machine?
	for (uint32_t j=0; j<m_nbrOfAdp; j++)					// for all adapters
	{	Fwadapter* adapter = getadapter(j);				// get indicated adapter
		if (!adapter) continue;										// if not present, ignore
		if(adapter->m_nodeinfo.nodeId.phyId == node)
		{
			return(true);
		}																		// no, it's not.
	}
	return(false);														// no find, not ours
}
//
//	storexmitdata  --  store data in xmit area for sending
//
int Fwadapter::storexmitdata(const uint8_t data[],size_t size)
{	
	if (size > sizeof(m_inData))
	{	logprintf("Oversize data (%d bytes, max is %d) to storexmitdata.\n",size,sizeof(m_inData));	
		return(ENOMEM);												// if not enough available
	}
	memcpy(m_inData,data,size);								// copy
	return(EOK);															// success
}
//
//	isocallback -- express callback for isochronous receives
//
//	Called at receive callback level, almost at interrupt level. Do not delay more than a few microseconds.
//
//	Per-object
//
void Fwadapter::isocallback(const  llaIsoHdr_t& pkt, dbuf_t*& buffer)
{	m_owner.processisopkt(pkt,buffer);
}
//
//	isocallback -- express callback for isochronous receives
//
//	Called at receive callback level, almost at interrupt level. Do not delay more than a few microseconds.
//
//	Static.
//
void Fwadapter::isocallback(uint32_t adapterid, const llaIsoHdr_t& pkt, dbuf_t*& buffer)
{	Fwadapter* adapter = getadapter(adapterid);		// get appropriate adapter
	if (adapter) adapter->isocallback(pkt,buffer);		// pass to appropriate adapter
	////logprintf("Iso recv on %d\n",adapterid);				// ***TEMP***
}
//
//	Callbacks from low-level driver
//
//	These are all at callback level -- don't block.
//
//	selfidcallback  --  self ID packets received
//
void Fwadapter::selfidcallback(const SelfIdPks_t& selfIdPks)
{
    //	Copy latest self ID info into shared area.
    {	ost::MutexLock lok(m_selfidlock);								// brief lock for copy only - max block a few us.
	    m_selfidpackets = selfIdPks;										// copy
	}
	Event_t event(EVENT_BUS_RESET);								// the event
	putevent(event);															// put the event on the event queue.
}
//
//	getselfidpackets  -- get latest self ID packets
//
void Fwadapter::getselfidpackets(SelfIdPks_t& selfIdPks)
{
    //	Copy latest self ID info from shared area.
    {	ost::MutexLock lok(m_selfidlock);								// brief lock for copy only - max block a few us.
	    selfIdPks = m_selfidpackets;										// copy
	}
}
//
//	asyrecvcallback  -- general async receive callback
//
void Fwadapter::asyrecvcallback(llaAsyHdr_t& hdr, dbuf_t* pData)
{
	Event_t event(EVENT_IN_ASY_PK);									// the event
	event.info.asyPk.hdr = hdr;											// copy the header to the event
	//	We have the header, and we may have data in a dbuf.
	//	Copy the data, if small enough to fit in the event.
	//	For longer events, we ought to copy the whole dbuf, but we never need that much data,
	//	so that isn't implemented.
	if (event.info.asyPk.hdr.dataLength != 0 && pData)		// if we have a dbuf with payload
	{	if (event.info.asyPk.hdr.dataLength > sizeof(event.info.asyPk.shortpayload))		// if we can't fit it in
		{	logprintf("ERROR: Oversize incoming async data of %d bytes\n",event.info.asyPk.hdr.dataLength);
			return;
		}
		size_t count = dbufCopyDataFromBuf (pData,         // src dynamic buffer 
                                            0,    // src offset in the dynamic buffer 
                                          	event.info.asyPk.shortpayload,           // dest buffer  
                                          	event.info.asyPk.hdr.dataLength); // nbr bytes to copy
      	if (count != event.info.asyPk.hdr.dataLength)
      	{	logprintf("ERROR: Unable to copy %d bytes of incoming async data from dbuf. Copied %d bytes.\n",
      			event.info.asyPk.hdr.dataLength,count);
      		return;
      	} 
	}
	putevent(event);															// put the event. Handles queue full
}
//
//	isorecvblockcallback  --  received finished block of isochronous data
//
//	This should contain the entire amount to be read in one chained buffer.
//	Headers have been stripped by the hardware.
//
void Fwadapter::isorecvblockcallback(uint32_t channel, dbuf_t* pData, int status)
{	m_owner.processisoblock(channel, pData,status);
}
//
//	isopurgecallback  --  iso channel is shutting down, purge	
//
void Fwadapter::isopurgecallback(uint32_t channel)
{	m_owner.processisopurge(channel);					// pass upwards
}

//


