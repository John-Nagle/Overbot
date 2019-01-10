//
//	businfo.cpp  --  get bus information
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
#include <string.h>
#include "mindreadyforcpp.h"
#include <sys/neutrino.h>
#include <limits.h>
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <errno.h>
#include <dirent.h>
#include <unistd.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <sys/resmgr.h>
#include <assert.h>
#include <arpa/inet.h>
#include "devindex.h"
#include "businfo.h"
#include "csr1394.h"
#include "utilities.h"
#include "devfactory.h"
using namespace ost;
//
//	Constants
//
bool verbose = true;												// enables debug output
//
//
//	class Businfo implementation
//
//
//	Constructor
//
Businfo::Businfo(uint32_t adapter)
	: m_adapter(adapter),
	m_initialized(false),
	m_terminate(false),
	m_needinventory(true),	
	m_transactionlabel(0),
	m_inventorywait(0),												// start out locked
	m_interface(*this,adapter),
	m_busresetinfo(*this),
	m_buschannels(*this)
{
}
//
//	startup -- start up the driver
//
//	Returns Posix status code.
//
int Businfo::startup()
{
    MutexLock lok(m_lock);											// lock during startup
    if (m_initialized)
        return(EOK);														// already done
    m_needinventory = true;										// must inventory next time
    int stat = m_interface.initadapter();						// start up link to FireWire driver
    if (stat)
    {
        logprintf("Unable to initialize FireWire interface.\n");
        return(ENODEV);												// if error, no device
    }
    //	Start asynchronous receive task
    stat = getinterface().asyRecvStart();					// start async recv.
    if (stat)
    {
        logprintf("Failed to start asychronous packet reception.\n");
        return(ENODEV);												// fails
    }
    //	This is actually our second bus reset.  Why do we need two?
    stat = busreset(false,false);									// force a bus reset
    if (stat)
    {
        logprintf("Failed to reset the bus.\n");
        return(stat);														// fails
    }
    //	Start inventory task to inventory the devices on the bus.
    stat = pthread_create(&m_inventorythread,NULL,&startinventorytask,this);	// start up the inventory task
    if (stat)
    {	logprintf("Failed to start bus inventory task.\n");
    		return(ENODEV);
    }    
    m_initialized = true;												// success
    return(EOK);															// success
}
//
//	shutdown  -  shutdown everything
//
void Businfo::shutdown()
{
    if (!m_initialized)
        return;																// already done
    MutexLock lok(m_lock);											// lock during shutdown
    //	Shut down our local tasks
    m_terminate = true;												// we are terminating
    m_inventorywait.post();										// wake up inventory task so it can die
    pthread_join(m_inventorythread,NULL);				// wait for it to die
    m_interface.cleanup();											// break link to driver
    m_initialized = false;
}
//
//	notebusreset  -- note that a bus reset has occured, and wake up inventory task
//
//	Locking not required.
//
void Businfo::notebusreset() 
{	m_needinventory = true; 										// need an inventory
	m_inventorywait.post();										// start the task that does it
}
//
//	handlebusreset  --  inventory the bus and update the device info, if needed
//
//	Returns a Posix error number.
//
//	All old node numbers are invalid at this point, and must be cleared.
//	New node numbers come from the bus reset info.
//
int Businfo::handlebusreset()
{
    MutexLock lok(m_lock);										// lock during inventory
    m_needinventory = true;									// inventory is not valid
    //	Get node ID of current IEEE-1394 adapter on local machine.
    int err = m_interface.getNodeID(m_nodeid);		// get our node number
    if (err) return(err);												// fails
    uint32_t startgeneration = m_nodeid.genCount;	// get generation from node ID
    //	Shut down all devices using this adapter.
    for (unsigned int i=0; i<getdevcount(); i++)
    {	Deventry* dev = getdev(i);							// get the device
    	Busentry* busdev = (Busentry*) dev;			// UNCHECKED DOWNCAST
    	if (busdev) busdev->busresetbegin();			// tell device that a bus reset has started
    }
    //	Process the bus reset info.
    err = m_busresetinfo.updateonbusreset1();		// do the first part of processing
    if (err) return(err);												// fails
    err = m_busresetinfo.updateonbusreset2();		// do the second part of processing
    if (err) return(err);												// fails
   	err = checkgeneration(startgeneration);			// check that generation has not changed
   	if (err) return(err);												// handle error
    //	Bus reset data has been collected. Now update our driver info.
    //
    //	Register all nodes as QNX devices.
    //
    startidcycle();															// starting a new bus inventory cycle
    //	Examine all nodes
    for (uint32_t node=0; node<m_busresetinfo.getnodecount(); node++)
    {
        //	Is the current node one of our own adapters, on this machine?
        bool externalNode = !m_interface.isnodeouradapter(node);	// is it one of ours?
        if (!externalNode)
            continue;															// ignore self nodes
        const NodeInfo_t* nodeinfo = m_busresetinfo.getnodeinfo(node);
        if (!nodeinfo) continue;											// out of range?
        if (!nodeinfo->respondingRom)
        {	logprintf("  Unable to get node info for node %d\n",node);
            continue;															// skip if didn't respond ***MAY CAUSE NODE TO BE LOST***
        }
        //	We have found a node - register it
        int stat = updatedev(node,nodeinfo->vendorID,
                             nodeinfo->chipIdHigh, nodeinfo->chipIdLow);
        if (stat)
            return(stat);														// if fail, end update
    }
    //	Final check - make sure bus generation didn't change again
   	err = checkgeneration(startgeneration);					// check that generation has not changed
   	if (err) return(err);														// handle error

    needremove();															// remove anything not found
    m_needinventory = false;											// done with inventory
    //	Restart devices using this adapter.
    for (unsigned int i=0; i<getdevcount(); i++)
    {	Deventry* dev = getdev(i);									// get the device
    	Busentry* busdev = (Busentry*) dev;					// UNCHECKED DOWNCAST
    	if (busdev) busdev->busresetdone();					// tell device that a bus reset has finished
    }
    //	One last check
   	err = checkgeneration(startgeneration);					// check that generation has not changed
   	if (err) return(err);														// handle error
    return(EOK);																// success
}
//
//	inventorytask  -- take inventory of the devices on the bus.
//
//	Triggered by a bus reset.
//
void Businfo::inventorytask()														
{	for (;;)
	{	m_inventorywait.wait();											// wait for work
		if (m_terminate) return;												// if terminating, exit
		if (!m_needinventory) continue;								// nothing to do
		handlebusreset();														// update the inventory
		if (m_needinventory)												// if trouble
		{	//	Trouble - inventory failed
			logprintf("  Unable to inventory FireWire devices. Retrying.\n");
			sleep(1);																// wait 1 sec to avoid loop
			m_inventorywait.post();										// go around again
			continue;
		}
		//	Success. Wait for more work.
	}
}
//
//	updatedev  -- update info for a single device
//
int Businfo::updatedev(uint32_t nodenumber, uint32_t vendor, uint32_t hiid, uint32_t lowid)
{
    ost::MutexLock lok(getlock());										// lock device index from here to end.
    Busentry* dev = find(vendor,hiid,lowid);						// Is it already in the index?
    if (dev)
    {	//	***MAY NEED TO PURGE I/O ON NODE NUMBER CHANGE*** or maybe we just purge all I/O on bus reset.
    		dev->setbusnode(nodenumber);								// set new node number
		dev->stampcycle();													// mark as active in current cycle
    } else {
	    //	No entry yet, add a new one
	    dev = devicefactory.makedevice(*this,nodenumber,vendor,hiid,lowid);	// get a suitable device object
	    assert(dev);																// must get something, even if a dummy
	    int stat = dev->setidentity();									// ask device for its identity info
	    if (stat != EOK)
	    {	logprintf("Unable to determine identity of node %d\n",nodenumber);
	    		////return(stat);															// fails ***TEMP*** keep unknown devices until dummy driver for them
	    	}
	   	insert(dev);																// put in node index
	}
	assert(dev);																	// will now have a device
    if (verbose)
        logprintf("  Updated node #%d: %s\n", nodenumber,dev->getname().c_str());
    return(EOK);
}
//
//	purgeio  -- purge any outstanding I/O
//
bool purgeio()
{
//	***MORE***
	return(true);																	// ***TEMP***
}
//
//	asyncrecv -- an async packet has been received
//
//	Locking is implicit in put to reply queue.
//
void Businfo::asyncrecv(const llaAsyHdr_t& hdr, const uint8_t data[], uint32_t datasize)
{
	if (hdr.destNodeId.phyId != getnodeid().phyId)				// if not addressed to us
	{	return;	}																	// ignore
	if (hdr.destNodeId.genCount != getnodeid().genCount)// if old packet (wrong bus reset generation)
	{	return;	}																	// ignore
	//	Test for message types that are replies to something we sent.
	switch (hdr.transactionCode) {										// fan out on transaction code
	case IEEE1394_QUAD_RDRESP:										// reply to quad read
	case IEEE1394_BLOCK_RDRESP:									// reply to block read
	case IEEE1394_WRRESP:												// reply to write (any)
	case IEEE1394_LOCK_RESP:											// lock response
		{	Shortbusmsg msg(hdr,data,datasize);					// got a message that replies to something
			m_replies.put(msg);												// put in reply queue
			return;																	// done
		}
	default:																			// something else
		break;										
	}
	//	Process requests directed at us.
	logprintf("Ignored unrequested incoming async message.\n");
}
//
//	Class Shortbusmsg  -- short bus messages
//
//
//	Constructor
//
Shortbusmsg::Shortbusmsg(const llaAsyHdr_t& hdr, const uint8_t data[], uint32_t size)
	:	m_header(hdr), m_size(size)							// copy header and size
{	for (unsigned int i=0; i<size && i<maxShortBusMsg; i++)
	{	m_data[i] = data[i];	}					// copy data
}
//
//	getquadlet  -- get 4 bytes as a 32-bit quantity
//
const uint32_t Shortbusmsg::getquadlet() const
{	
	return(ntohl(*((uint32_t*)m_data)));						// convert byte order			
}
//
//	BusReset  -- do a bus reset.
//
//	All nodes get new node IDs.
//
//	Returns Posix error code if fail
//
//	We don't normally try to become the bus root. It's only necessary to do that if the
//	bus root isn't able to become the isochronous cycle master.
//
int Businfo::busreset(bool becomeRoot, bool shortBusReset)
{	
    ost::MutexLock lok(getlock());									// lock device index from here to end.
    int stat = getinterface().setForceRootFlag(becomeRoot);			// set whether we want to become bus root
    if (stat) return(stat);													// if fail
	stat = getinterface().busReset(shortBusReset);		// do the bus reset
	if (stat) return(stat);													// if fail
	//	Wait for the bus to reset.
	logprintf("  Commanded bus reset on adapter %d. %s\n",getadapter(),becomeRoot ? "(trying to become root)":"");	// ***TEMP***
	usleep(10000);															// wait 10ms
    return(EOK);																// success
}
//
//	isrootnode  --  is current node the root
//
//	Returns true if we are the root, false if not, or we can't tell.
//
bool Businfo::isrootnode()
{
	llaPresentStatus_t presentStatus;
	int stat = getinterface().getPresentStatus(presentStatus);	// get present status
	if (stat) return(false);															// if fail, assume we're not the root.
    if (presentStatus.busRoot) return(true);								// we are the root
 	return(false);																		// we are not the root
}
//
//	readlocalrominfo  --  read info from local adapter ROM
//
//	Returns POSIX error status.
//
int Businfo::readlocalrominfo(NodeInfo_t& nodeinfo)
{	
    ost::MutexLock lok(getlock());										// lock device index from here to end.
	uint32_t quadletVal;
    int stat = getinterface().readCSR(CSR1394_ROM_BUS_OPTIONS_ADDR,quadletVal);
    if (stat != EOK) return(stat);
    uint32_t dataQuadlet = ntohl(quadletVal);
    nodeinfo.irmc = (dataQuadlet & 0X80000000) >> 31;
    nodeinfo.cmc  = (dataQuadlet & 0X80000000) >> 30;
    nodeinfo.isc  = (dataQuadlet & 0X80000000) >> 29;
    nodeinfo.bmc  = (dataQuadlet & 0X80000000) >> 28;


    // Get vendor ID from adapter ROM
    stat = getinterface().readCSR(CSR1394_ROM_GUID_HI_ADDR,quadletVal);
    if (stat != EOK) return(stat);
    dataQuadlet = ntohl(quadletVal);
    nodeinfo.vendorID = dataQuadlet >> 8;

    nodeinfo.chipIdHigh = dataQuadlet & 0x000000FF;

    /* Get ROM vendor ID */
    stat = getinterface().readCSR(CSR1394_ROM_GUID_LOW_ADDR,quadletVal);
    if (stat != EOK) return(stat);
    dataQuadlet = ntohl(quadletVal);
    nodeinfo.chipIdLow = dataQuadlet;

    nodeinfo.root = isrootnode();						// are we the root node?
    nodeinfo.respondingRom      = true;				// we got the info, good.
    nodeinfo.respondingNodeInfo = true;
    return (EOK);
}
//
//	AppPrintPacketHdr  -- print a packet header for debug.
//
//	No access to state; no locking needed.
//
void AppPrintPacketHdr(const llaAsyHdr_t& hdr,const char* heading)
{	logprintf("  %s packet:\n",heading);
	logprintf("     Source node: %d  Destination node: %d\n",
		hdr.srcNodeId.phyId,hdr.destNodeId.phyId);
	logprintf("	    Label: %d  Transaction code: %s\n", hdr.transactionLabel,
									TransactionCodeStr(hdr.transactionCode));
	logprintf("     Address %08x:%0x8x\n",hdr.destinationHigh,hdr.destinationLow);
}
//
//	checkrequestreplymatch  --  check that a packet reply matches a packet request.
//
static bool checkrequestreplymatch(const llaAsyHdr_t& request, const llaAsyHdr_t& reply)
{
	//	Is this message for us?
	if (request.srcNodeId.phyId != reply.destNodeId.phyId) 
	{	return(false); }
	//	Is this message in reply to our message?
	if (request.destNodeId.phyId != reply.srcNodeId.phyId)
	{	return(false); }
	//	Do the transaction IDs match?
	if (request.transactionLabel != reply.transactionLabel)
	{	return(false); }
	return(true);															// matches
}
//
//	asynctransaction  -- do an asynchronous transaction, with retry
//
int Businfo::asynctransaction(llaAsyHdr_t&hdr,  const uint8_t data[], const AsyTxCtrl_t& ctl, Shortbusmsg& replymsg, uint32_t retries)
{
	for (;;)																			// until retries run out
	{	int stat = asynctransactiontry(hdr,data,ctl, replymsg);	// try the transaction
		if (stat == EOK) return(EOK);									// success - done
		if (retries == 0) return(stat);										// failed
		logprintf("Retrying failed async bus transaction.\n");
		retries--;
	}
}
//
//	asynctransactiontry  -- do an asynchronous transaction
//
//	Only one async transaction can be in progress at a time.  This really should be locked per-device.
//
int Businfo::asynctransactiontry(llaAsyHdr_t&hdr,  const uint8_t data[], const AsyTxCtrl_t& ctl, Shortbusmsg& replymsg)
{	const float bustimeout = 0.1;										// 100ms bus timeout 	
	const unsigned int maxmsgs = 10;								// max messages to try
    ost::MutexLock lok(getlock());										// lock device index from here to end.
	//	First, drain off any queued replies that came in while we were doing something else
	while (m_replies.tryget(replymsg))								// remove anything in queue
	{	logprintf("Ignored stale or unwanted reply message.\n");}
	//	Send the query to the bus
	int stat = getinterface().asySend(hdr,data,ctl);				// send to the bus
	if (stat) return(stat);														// handle error
	//	Wait for a reply.
	for (unsigned int i=0; i<maxmsgs; i++)						// until max try count
	{	bool got = m_replies.get(replymsg,bustimeout);		// wait for a message
		if (!got)
		{	logprintf("No reply to bus transaction.\n");
			return(EIO);															// if timeout, fail
		}
		//	Check that message matches query. If not, get next message.
		if (!checkrequestreplymatch(hdr,replymsg.getheader()))
       {	logprintf("Bad Response: query packet and reply packet did not match.\n");
            	AppPrintPacketHdr(hdr,"Sent");
            AppPrintPacketHdr(replymsg.getheader(),"Received");
          	continue;																// try again
       }
		if (0)																			// dump transaction
       	{	logprintf("Async transaction successful:\n");
            	AppPrintPacketHdr(hdr,"Sent");
            AppPrintPacketHdr(replymsg.getheader(),"Received");
        }
		return(EOK);																// success
	}
	return(EIO);																	// got many bogus replies
}
//	
//	readquadlet  -- read quadlet from another node on the bus.
//
//	We have to send an async message and wait for the reply.
//	Reorders bytes from network order. (see getquadlet)
//
int Businfo::readquadlet(uint32_t destNode, uint64_t destAddr, uint32_t& quadval)
{
	//	Build the bus transaction.
	//	Generation number changes only when a bus reset is processed, so if a bus reset
	//	occurs, all async transactions fail until bus reset processing has occured.
    ost::MutexLock lok(getlock());								// lock device index from here to end.
    if (destNode > maxBusNode) return(EIO);			// invalid bus address, device offline
	AsyTxCtrl_t ctl;
	ctl.computeTimeStamp = true;								// we're not specifying a time stamp
	llaAsyHdr_t hdr;
    hdr.srcNodeId = getnodeid();								// our (source) node ID
    hdr.destNodeId = getnodeid();								// init adapter and generation number of node ID, from last bus reset
    hdr.destNodeId.phyId = destNode;						// set actual destination node
    hdr.destinationHigh  = destAddr >> 32;				// split 64-bit address into two 32-bit parts
    hdr.destinationLow   = destAddr & 0xffffffff;
    hdr.transactionCode  = IEEE1394_QUAD_RDREQ;
    hdr.extendedTransactionCode = 0;
    hdr.transactionLabel = 0;
    hdr.retryCode        = IEEE1394_RETRY_X;
    hdr.responseCode     = 0;
    hdr.dataLength       = 0;
    hdr.speed            = IEEE1394_100MBITS_SPEED;
    //	Do the bus transaction. This blocks.
    Shortbusmsg replymsg;															// get reply here
    int stat = asynctransaction(hdr,0,ctl,replymsg);						// do it
    if (stat)
    {	logprintf("ReadQuadlet to node %d failed.\n",destNode);		// fails
    	return(stat);																	// fails
    }
	quadval = replymsg.getquadlet();									// get return value
 	return(EOK);																		// success
}
//	
//	writequadlet  -- write quadlet to another node on the bus.
//
//	We have to send an async message and wait for the reply.
//	Reorders bytes to network order
//
int Businfo::writequadlet(uint32_t destNode, uint64_t destAddr, uint32_t quadVal)
{
    ost::MutexLock lok(getlock());								// lock device index from here to end.
    if (destNode > maxBusNode) return(EIO);			// invalid bus address, device offline
	quadVal = htonl(quadVal);									// reorder bytes
	//	Build the bus transaction
	AsyTxCtrl_t ctl;
	ctl.computeTimeStamp = true;								// we're not specifying a time stamp
	llaAsyHdr_t hdr;
    hdr.srcNodeId = getnodeid();								// our (source) node ID
    hdr.destNodeId = getnodeid();								// init adapter and generation number of node ID, from last bus reset
	//	Build the bus transaction
    hdr.destNodeId.phyId = destNode;
    hdr.destinationHigh  = destAddr >> 32;				// split 64-bit address into two 32-bit parts
    hdr.destinationLow   = destAddr & 0xffffffff;
	hdr.transactionCode  = IEEE1394_QUAD_WRREQ;
	hdr.extendedTransactionCode = 0;
	hdr.transactionLabel = gettransactionlabel();
	hdr.retryCode        = IEEE1394_RETRY_X;
	hdr.responseCode     = 0;
	hdr.dataLength       = 4;
	hdr.speed            = IEEE1394_100MBITS_SPEED;
    //	Do the bus transaction. This blocks.
    Shortbusmsg replymsg;																// get reply here
    int stat = asynctransaction(hdr,(uint8_t*)&quadVal,ctl,replymsg);// do it
    if (stat)
    {	logprintf("WriteQuadlet to node %d failed.\n",destNode);			// fails
    		return(stat);																				// fails
    	}
 	return(EOK);								
}
//
//	compareswapquadlet  -- compare and swap a quadlet from another node on the bus
//
//	Used to update the register that stores available isochronous channels.
//
//	We have to send an async message and wait for the reply.
//	Reorders bytes to network order
//
int Businfo::compareandswapquadlet(uint32_t destNode, uint64_t destAddr, uint32_t oldquadval, uint32_t newquadval)
{	
    ost::MutexLock lok(getlock());								// lock device index from here to end.
    if (destNode > maxBusNode) return(EIO);			// invalid bus address, device offline
	//	Build data portion of lock request.
	struct lockreq32 {												// 32-bit compare and swap request
		uint32_t m_old;													// before value
		uint32_t m_new;												// after value
	};
	lockreq32 req = { htonl(oldquadval), htonl(newquadval) };	// build compare and swap request
	llaNodeId_t nodeId;												// the node ID, in packet format
	//	Get the source node ID, and set it in both source and destination.
	//	The real destination ID goes in phyID.
	int stat = getinterface().getNodeID(nodeId);			// get the node ID
	if (stat != EOK) return(stat);									// if fail
	//	Build the bus transaction
	AsyTxCtrl_t ctl;
	ctl.computeTimeStamp = true;								// we're not specifying a time stamp
	llaAsyHdr_t hdr;
    hdr.srcNodeId = nodeId;
    hdr.destNodeId = nodeId;					// destination node ID
	//	Build the bus transaction
    hdr.destNodeId.phyId = destNode;
    hdr.destinationHigh  = destAddr >> 32;				// split 64-bit address into two 32-bit parts
    hdr.destinationLow   = destAddr & 0xffffffff;
	hdr.transactionCode  =  IEEE1394_LOCK_REQ;		// request for lock operation
	hdr.extendedTransactionCode = IEEE1394_COMPARE_SWAP;	// lock op is compare and swap
	hdr.transactionLabel = gettransactionlabel();
	hdr.retryCode        = IEEE1394_RETRY_X;
	hdr.responseCode     = 0;
	hdr.dataLength       = sizeof(req);						// data area
	hdr.speed            = IEEE1394_100MBITS_SPEED;
    //	Do the bus transaction. This blocks.
    Shortbusmsg replymsg;																// get reply here
    stat = asynctransaction(hdr,(uint8_t*)&req,ctl,replymsg);			// do it
    if (stat)
    {	logprintf("Lock to node %d failed.\n",destNode);					// fails
    	return(stat);																				// fails
    }
    uint32_t returnquadval = replymsg.getquadlet();							// get return value. should match OLD value.
    if (oldquadval != returnquadval) 													// if not same, compare and swap failed.
    {	logprintf("Lock to node %d failed, busy. Setting %08x, expected %08x, got %08x\n",
    			destNode,newquadval,oldquadval,returnquadval);										
    	return(EBUSY);																			// lock unsuccessful, write did not take place
    }
 	return(EOK);																					// success
}
//
//	readcsr  -- read control/status register from our own, or another, node on the bus.
//
int Businfo::readcsr(uint32_t destNode, uint32_t offset, uint32_t& quadVal)
{	if (destNode == getnodeid().phyId)					// if this request is for our own local node
	{  
		return(getinterface().readCSR(offset,quadVal));
	}
	else 																	// if command is to remote node, do it.
    	{	////logprintf("Read CSR %d from node %d (we are node %d)\n",offset,destNode,getnodeid());	// ***TEMP***
    		return(readquadlet(destNode,csraddr(offset),quadVal));
    	}
}
//
//	readcsr, 32-bit version
//
int Businfo::readcsr(uint32_t destNode, uint32_t offset, uint64_t& octletVal)
{
    if (destNode > maxBusNode) return(EIO);			// invalid bus address, device offline
	uint32_t hi, lo;												// low and high quads
	int stat = readcsr(destNode,offset, hi);			// read low quad
	if (stat != EOK) return(stat);
	stat = readcsr(destNode,offset+4, lo);			// read high quads
	if (stat != EOK) return(stat);
	octletVal = uint64_t(hi)<<32 | lo;					// construct octlet
	return(EOK);													// success
}
//
//	compareandswapcsr  -- lock/read/swap control/status register from local node
//
int Businfo::compareandswapcsr(uint32_t destNode, uint32_t offset, uint32_t oldQuadVal, uint32_t newQuadVal)
{	
    if (destNode > maxBusNode) return(EIO);			// invalid bus address, device offline
	if (destNode == getnodeid().phyId)					// if this request is for our own local node
	{	uint32_t returnedQuadVal = newQuadVal;	// will get back value after swap. 
		int stat = getinterface().lockCSR(offset,oldQuadVal,returnedQuadVal);
	    if (stat)
	    {
	        logprintf("Failed to compare and swap CSR.\n");
	        return(stat);												// I/O error
	    }
	    if (returnedQuadVal != oldQuadVal)		// if old value wasn't the one expected
	    {	logprintf("Busy when comparing and swapping CSR locally.\n");
	    		return(EBUSY);
	    	}
    	} else {															// remote node, do bus transaction
    		return(compareandswapquadlet(destNode, csraddr(offset),oldQuadVal,newQuadVal));
    }
    return(EOK);
}
//
//	Isochronous support
//
//	isochannelallocate  --  try to get a free isochronous channel
//
//	All this really does is find a free bit in the 64-bit register that tracks which isochronous channels are in use.
//
//	Currently, only looks at the first 32 channels, so if more than 32 isochronous channels are in use,
//	this will not find a free channel.
//
int Businfo::isochannelallocate(int& chan)
{	
    ost::MutexLock lok(getlock());											// lock device index from here to end.
	chan = -1;																		// no channel yet
	for (unsigned int tries = 0; tries < 20; tries++)					// try repeatedly for ISO allocation
	{	uint32_t isomanager = m_busresetinfo.m_IRM;				// bus reset manager
		if (isomanager == NO_IRM) return(ENETDOWN);			// no isochronous resource manager on bus
		uint32_t alloc;																// first allocation register
		int stat = readcsr(isomanager,CSR1394_CHANNELS_AVAILABLE_HI_ADDR, alloc);
		if (stat != EOK) return(stat);
		////logprintf("Available channel register before allocate: %08x\n",alloc);	// ***TEMP***
		int workchan =  findfirstbitmsb(alloc);							// find first one bit, counting from MSB end.
		if (workchan < 0) return(EMFILE);									// all ISO channels in use
		uint32_t allocmask = alloc & (~bitmsb(workchan));		// clear bit, counting from MSB end.	
		//	Clear the requested bit. 0 means allocated.  Note that channel zero is
		//	represented n this register by the MSB (most significant bit), which is
		//	backwards from most other usages.
		//	(Table 18-3 in "FireWire System Architecture" has this wrong. But the Linux developers, IEEE
		//	spec, and testing against WIndows 2000 indicate this is right.)
		stat = compareandswapcsr(isomanager, CSR1394_CHANNELS_AVAILABLE_HI_ADDR,alloc,allocmask);
		if (stat == EOK) 															// if success
		{	chan = workchan;													// use selected channel
			logprintf("Allocated iso. channel %d\n",chan);			// ***TEMP***
			return(EOK);																// success
		}
		//	Failed, someone else was trying channel allocation at the same time.
		//	The odds of this are very low, but nonzero.
		logprintf("  Iso channel allocation of channel %d failed, retrying.\n",workchan);	
		usleep(100*tries+1);													// linear backoff
	}
	return(EBUSY);
}
//
//	isochannelrelease  --  release an isochronous channel
//
int Businfo::isochannelrelease(int chan)
{																	
    ost::MutexLock lok(getlock());											// lock device index from here to end.
	for (unsigned int tries = 0; tries < 20; tries++)					// try repeatedly for ISO allocation
	{	uint32_t isomanager = m_busresetinfo.m_IRM;				// bus reset manager
		if (isomanager == NO_IRM) return(ENETDOWN);			// no isochronous resource manager on bus
		uint32_t alloc;																// allocation register
		int stat = readcsr(isomanager,CSR1394_CHANNELS_AVAILABLE_HI_ADDR, alloc);
		if (stat != EOK) return(stat);
		//////logprintf("Available channel register: %08x\n",alloc);	// ***TEMP***
		uint32_t allocmask = alloc | (bitmsb(chan));					// set bit, counting from MSB end.	
		//	Set the requested bit. 1 means unallocated.  Note that channel zero is
		//	represented n this register by the MSB (most significant bit), which is
		//	backwards from most other usages.
		stat = compareandswapcsr(isomanager, CSR1394_CHANNELS_AVAILABLE_HI_ADDR,alloc,allocmask);
		if (stat == EOK) 															// if success
		{	
			return(EOK);																// success
		}
		//	Failed, someone else was trying channel allocation at the same time.
		//	The odds of this are very low, but nonzero.
		logprintf("  Iso channel release of channel %d failed, retrying.\n",chan);	
		usleep(100*tries+1);													// linear backoff
	}
	return(EBUSY);
}
void  Businfo::processisoframe(const IsoFrInfo_t& frame, dbuf_t*& buffer)
{
	//	Try to avoid this approach, which is for packing multiple ISO packets into one buffer and is Mindready-specific.
}
//
//	dumpisohdr  --  dump an isochronous packet header
//
static void dumpisohdr(const  llaIsoHdr_t& hdr)
{	
	logprintf("  Iso. pkt %u bytes, tag %u, channel %u, syncro %u , speed %u , timestamp %u , skip %x, cycle %x\n",
		hdr.dataLength,				// IEEE 1394-1995 Std 6.2.4.8
		hdr.tag,								// IEEE 1394-1995 Std 6.2.4.12 
		hdr.channel,						// IEEE 1394-1995 Std 6.2.4.13 
		hdr.synchro,						// IEEE 1394-1995 Std 6.2.4.14 
		hdr.speed,						// IEEE 1394-1995 Std Table 4-2 
		hdr.timeStamp,					// cycle timer compare value 
		hdr.skip,							// Skip packet enable flag 
		hdr.cycleMatch);				// bus reset cycle number?
}
//
//	processisopkt  --  process an incoming isochronous packet
//
//	The packet is sent to whatever device, if any, is monitoring that isochronous channel.
//
//	Called from the isochronous callback, so delays can cause loss of data.
//
void  Businfo::processisopkt(const  llaIsoHdr_t& hdr, dbuf_t*& buffer)
{	if (0) dumpisohdr(hdr);											// very verbose debug print
	Busentry* dev = findisolistener(hdr.channel);// find who wants this channel
	if (!dev) return;														// ignore, no listener for this channel
	dev->isorcvpkt(hdr,buffer);									// send data to device
}
//
//	processisoblock  --  process an incoming isochronous block of data, without headers
//
//	The block is sent to whatever device, if any, is monitoring that isochronous channel.
//
//	Called from the isochronous callback, so delays can cause loss of data.
//
//	No locking needed; device cannot disappear while open.
//
void  Businfo::processisoblock(uint32_t channel, dbuf_t* buffer, unsigned status)
{	Busentry* dev = findisolistener(channel);			// find who wants this channel
	if (!dev) return;														// ignore, no listener for this channel
	dev->isorcvblock(channel, buffer,status);			// send data to device
}
//
//	processisopurge  --  process an isochronous purge request
//
//	All duplicate buffers must be released
//
void  Businfo::processisopurge(uint32_t channel)
{	
    ost::MutexLock lok(getlock());								// lock device index from here to end.
	Busentry* dev = findisolistener(channel);			// find who wants this channel
	if (!dev) return;														// ignore, no listener for this channel
	dev->isorcvpurge();												// tell device to release buffers
}
//
//	getgeneration  -- get bus generation number
//
//	Incremented on every bus reset.  If it changes during an inventory, a multiple bus
//	reset has occured and we have to re-inventory.
//
int Businfo::getgeneration(uint32_t& gen)
{	
	return(getinterface().getGenerationCount(gen));	
}
//
//	checkgeneration -- check that generation number has not changed
//
int Businfo::checkgeneration(uint32_t gen)
{	uint32_t curgen;													// current bus generation
	int stat = getgeneration(curgen);							// get current generation
	if (stat != EOK) return(stat);									// fails
	if (gen == curgen) return(EOK);							// normal case
	logprintf("  A bus reset has occured.  Bus re-inventory required.\n");
	m_needinventory = true;										// note this, even though a bus reset event is expected.
	return(EAGAIN);
}
//
//	isoenable  --  isochronous receive enable/disable
//
//	Initializes the isochronous receive hardware for this channel
//
int Businfo::isoenable(uint32_t channel, uint32_t framesize, uint32_t totalsize, bool waitforsync)
{	return(getinterface().isoRecvStart(channel,framesize,totalsize,waitforsync));	}	// adapter does this
//
//	isodisable  --  isochronous receive enable/disable
//
//	Shuts down
//
int Businfo::isodisable(uint32_t channel)
{	return(getinterface().isoRecvStop(channel));	}	// adapter does this

//
//	busreset1  --  process a bus reset event.
//
//	First part of bus reset - gathers data from the bus adapter.
//
int Busresetinfo::updateonbusreset1()
{
	//	Get the "self ID" packets from all the nodes on the bus.
    //	Get the most recent set of self ID packets.
	SelfIdPks_t selfidpkts;
   	m_owner.getinterface().getselfidpackets(selfidpkts);
    //	Sanity check self ID packets.
    if (selfidpkts.numOfBlock > IEEE1394_MAX_NB_NODE)		// probably junk from adapter
    {	logprintf("Too many nodes on bus.\n");
    		m_nbrOfNode = 0;							
    		return(EIO);														// reset the bus and try again
    	}
    // Save info about the number of nodes in the bus 
    m_nbrOfNode  = selfidpkts.numOfBlock;
    m_rootNode   = selfidpkts.numOfBlock-1;				// Root node is always last node to self-ID
    //	Save the self ID packets
    m_selfIdPks = selfidpkts;

    /* Get adapter node id info */
	if (verbose)															// debug output
	{ 	logprintf("  BUS RESET:  We are node %d on adapter %d%s.   %d nodes on bus.\n",
           m_owner.getnodeid().phyId, m_owner.getadapter(),
           (m_rootNode == m_owner.getnodeid().phyId ? " (we are root)" : ""),
           m_nbrOfNode);
	}
    // Reset Iso Resource Manager and Bus Manager 
    m_IRM = NO_IRM;
    m_BM  = DEFAULT_BUS_MNG_NUMBER;
    return(EOK);
}
//
//	busreset2  --  second part of bus reset
//
//	This is about figuring out which nodes are in charge of what.
//
//	The players:
//
//		The root node: last node to check in after a bus reset.
//
//		The isochronous resource manager: keeps track of ISO channels and bandwidth.
//		Not all nodes know how to be an IRM, but any capable node can do the job.
//		
//		The isochronous cycle manager - sends clock cycles.   Only the root node can generate
//		isochronous clock cycles, but it's possible for a node to be selected for that job that
//		can't do it.
//
//		The bus manager - decides who's doing what on this bus reset. Only nodes that volunteer
//		can become bus manager. Generally, host computers will offer to be bus manager, but
//		peripherals will not.
//
int Busresetinfo::updateonbusreset2()
{

    /* Decode IRM node number(Isochronous Resource Manager)*/
    /* from info in the self id packets.                   */
    //	The last node that wants to be isochronous resource manager gets to be it.
    m_IRM = NO_IRM;
    for (uint32_t node=0; node<m_nbrOfNode; node++)
    {
        if (m_selfIdPks.block[node].linkActive &&
                m_selfIdPks.block[node].contender)
        {
            m_IRM = node;
        }
    }
	//	The bus manager is chosen by the isochronous resource manager.

    /* Get the BM node number (Bus Manager) */
    m_BM = DEFAULT_BUS_MNG_NUMBER;
    if (m_IRM != NO_IRM)
    {	logprintf("  Iso. resource manager is node %d%s.\n",
    			m_IRM, m_IRM == m_owner.getnodeid().phyId ? " (we are IRM)" : "");
        /* Are we already IRM ? */
        if (m_IRM == m_owner.getnodeid().phyId)
        {	uint32_t quadletVal;
            int stat = m_owner.getinterface().readCSR(CSR1394_BUS_MANAGER_ID_ADDR,quadletVal);
            if (stat != EOK) return(stat);
            m_BM = quadletVal;
        }
        else /* Ask the IRM for BM */
        {	//	This asks another node for info, requiring bus traffic and an event.
    			uint32_t            dataQuadlet;              /* When reading quadlet     */
            int err = m_owner.readcsr (m_IRM, CSR1394_BUS_MANAGER_ID_ADDR, dataQuadlet);
            if (err)
                m_BM = DEFAULT_BUS_MNG_NUMBER;
            else
                m_BM = dataQuadlet & BUS_MANAGER_MASK;
        }
    } else {															// there is no IRM
    		logprintf("  ERROR: No isochronous resource manager found - cannot do isochronous I/O.\n");
    	}
    /* Read ROM info of each node on the bus */
    for (uint32_t node=0; node<m_nbrOfNode; node++)
    {
        uint32_t err = 0;
        m_nodeInfo[node].respondingRom      = false;
        m_nodeInfo[node].respondingNodeInfo = false;

        /* Do not send packet to current adapter */
        if (node == m_owner.getnodeid().phyId)
        {	err = m_owner.getinterface().updaterominfo();
            if (err)
            {
                logprintf("Failed to get node-specific information.\n");
                return(ENODEV);									// no such device
            }
            /* Backup this info in the node info list */
            m_nodeInfo[node] =
            		m_owner.getinterface().getnodeinfo();
            continue;
        }
		////logprintf("Reading ROM options for node %d\n",node);	// ***TEMP***
        /* Get ROM bus options at addr offset 0x0408*/
        uint64_t destAddr = 0XFFFFF0000408llu;			// ROM bus options at 0x0408
        NodeInfo_t& nodeinfo = m_nodeInfo[node];
    		uint32_t            dataQuadlet;              /* When reading quadlet     */
    		err = m_owner.readquadlet(node,destAddr,dataQuadlet);
        if (err)
        {
            nodeinfo.respondingRom = false;
            continue;
        }
        else
        {
            nodeinfo.respondingRom = true;
        }

        nodeinfo.irmc = (dataQuadlet & 0X80000000) >> 31;
        nodeinfo.cmc  = (dataQuadlet & 0X40000000) >> 30;
        nodeinfo.isc  = (dataQuadlet & 0X20000000) >> 29;
        nodeinfo.bmc  = (dataQuadlet & 0X10000000) >> 28;
        usleep(10000);

        /* Get Vendor ID and UID-hi*/
        const uint64_t idhiaddr = 0xFFFFF000040Cllu;		// read from ID Hi
        err = m_owner.readquadlet(node, idhiaddr, dataQuadlet);
        if (err)
        {	logprintf("Unable to read Vendor ID from node on bus.\n");
            nodeinfo.respondingRom = false;
            continue;
        }

        nodeinfo.vendorID   = dataQuadlet >> 8;
        nodeinfo.chipIdHigh = dataQuadlet & 0x000000FF;
        usleep(10000);																	// ***TEMP** probably unneeded

        /* Get Vendor UID-low*/
        const uint64_t idloaddr = 0xFFFFF0000410llu;		// ID Lo bus address
        err = m_owner.readquadlet(node, idloaddr, dataQuadlet);
		if (err)
        {
            nodeinfo.respondingRom = false;
            continue;
        }
        nodeinfo.chipIdLow = dataQuadlet;
    }
    /* Wait for all node response */
    usleep(100000);																	// a full 100ms
    if (m_nbrOfNode < 2)															// if alone on the network
    {	logprintf("  Alone on the network.\n");								// we are very alone
    		return(EOK);																		// cannot start up isochronous cycles on 1-node bus
    	}
    	//	There are at least two nodes on the bus.
	//	Are we the root node?
	//	Set up isochronous cycle master
    if (m_owner.isrootnode())													// if we are the root node
    {	int stat = m_owner.getinterface().isoEnableCycleMaster();				// turn on cycle generation
		if (stat != EOK) return(stat);										// fails
		logprintf("  Root node - generating isochronous cycles.\n");
   	} else {
   		//	We are not the root. Is the root generating isochronous cycles?
        const NodeInfo_t& nodeinfo = m_nodeInfo[m_rootNode];	// get node info for root node
        if (nodeinfo.cmc)															// if other node can generate isochronous cycles
        {	logprintf("  Root node %d is a valid isochronous cycle manager.\n",m_rootNode);	} // let them do it
        else 																				// if root node is clueless
        {	int stat = m_owner.busreset(true,false);					// force a bus reset that makes us root
        		if (stat != EOK) return(stat);										// fails
        		logprintf("  Root node %d is unable to generate isochronous cycles. Trying to become root.\n",m_rootNode);
        		return(EAGAIN);															// try again after the bus reset
        	}
    }
    return(EOK);
}
//
//	findisolistener  --  find a device that wants packets for this ISO channel
//
Busentry* Businfo::findisolistener(uint32_t channel)
{
	return(m_buschannels.findisolistener(channel));	// return indicated channel
}
//
//	builddefaultname  --  build default name for a device
//
//	Just "deviceclass-vendorid-hiid-lowid".
//	Only used when we can't get the name from ROM
//
void Busentry::builddefaultname(string& s)
{
    s+= getnameprefix();													// based on driver being used
    s+= '-';																			// separator
    char buf[sizeof(uint32_t)*2+1];									// enough space for hex number
    s+= itoa(m_vendor,buf,16);											// vendor number
    s+= '-';																			// separator
    s+= itoa(m_hiid,buf,16);												// high part of ID (model number)
    s+= '-';																			// separator
    s+= itoa(m_lowid,buf,16);												// low part of ID (serial number)
    assert(s.c_str()[s.length()] == '\0');								// make sure string properly terminated
    Deventry::canonizename(s);											// convert to canonical form
    assert(s.c_str()[s.length()] == '\0');								// make sure string properly terminated
} 
//
//	find -- look up by bus info
//
//	Used after a bus reset
//
Busentry* Businfo::find(uint32_t vendor, uint32_t hiid, uint32_t lowid)
{
	const vector<Deventry*>& devs = getdevs();		// get entire device list
	for (vector<Deventry*>::const_iterator p = devs.begin(); p != devs.end(); p++)
	{	Busentry* busent = dynamic_cast<Busentry*>(*p);	// checked downcast; will return null if wrong type
		if (!busent) continue;											// not one of ours
		if (busent->compare(vendor,hiid,lowid))				// if match
		{	return(busent);			}										// found
	}
	return(0);																	// no find
}
//
//	Class Buschannels  
//
Buschannels::Buschannels(Businfo& bus)
 	:m_owner(bus)															// set backlink
{	for (unsigned int i=0; i<MAX_ISO_CHANNELS; i++)	// clear all channel slots
 	{	m_receivers[i] = 0; }
}
//
//	listenchannel -- start listening on a specified channel
//
int Buschannels::listenchannel(Busentry& busdev, uint32_t channel, uint32_t framesize, uint32_t totalsize, bool waitforsync)
{
	if (channel >= MAX_ISO_CHANNELS) return(EBUSY);	// out of range
	if (m_receivers[channel])											// if somebody already using this channel
	{	return(EBUSY);	}													// we can't use it
	m_receivers[channel] = &busdev;							// we have it
	return(m_owner.isoenable(channel,framesize,totalsize,waitforsync));				// enable listening on this channel
}
//
//	releasechannel  -- release any channel owned by a device
//
int Buschannels::releasechannel(Busentry& busdev)
{	int finalstat = EOK;
	for (unsigned int i=0; i<MAX_ISO_CHANNELS; i++)	// search for channel pointing to device
	{	if (m_receivers[i] == &busdev)							// if find
		{	
			int stat = m_owner.isodisable(i);						// turn off listening on this channel
			if (stat != EOK) finalstat = stat;							// accumulate bad status if any
			m_receivers[i] = 0;											// not using this channel
		}
	}
	return(finalstat);
}
//
//	releaseall  -- release all channels
//
void  Buschannels::releaseall()
{
	for (unsigned int i=0; i<MAX_ISO_CHANNELS; i++)	// search for channel pointing to device
	{	if (m_receivers[i])													// if in use
		{	m_owner.isodisable(i);										// turn off listening on this channel
			m_receivers[i] = 0;											// not using this channel
		}
	}
}




