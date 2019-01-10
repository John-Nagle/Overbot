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
#include "drvadapter.h"
#include "drvadapterintern.h"
#include "utilities.h"


//
//	Drvadapter --  low-level driver for one adapter
//
//	This is currently an encapsulation of the Mindready LLA, but could be converted to use a non-LLA approach.
//
//	All of these return POSIX error codes.
//	Asychronous functions

//
//	checkasyheader --  check an async request header for sanity
//
//	Returns POSIX error code.
//
static int checkasyheader(const llaAsyHdr_t& hdr)
{	//	Check speed field
    if (hdr.speed  != IEEE1394_100MBITS_SPEED &&
            hdr.speed  != IEEE1394_200MBITS_SPEED &&
            hdr.speed  != IEEE1394_400MBITS_SPEED )
    {
        logprintf("ERROR: Bad outgoing packet header: Invalid speed.\n");
        return (EINVAL);
    }
    //	Check transaction code vs. data length
    if (hdr.transactionCode == IEEE1394_LOCK_REQ)
    {
        if (hdr.dataLength > 16)									//  a send of 16 is allowed, but most receivers only support 8
        {	logprintf("ERROR: Bad outgoing packet header: oversize lock request of %d bytes\n", hdr.dataLength);
            return (EINVAL);
        }
    }

    if (hdr.transactionCode == IEEE1394_LOCK_RESP)
    {
        if (hdr.dataLength > 8)
        {	logprintf("ERROR: Bad outgoing packet header: oversize lock response of %d bytes\n", hdr.dataLength);
            return (EINVAL);
        }
    }
	//	Check for sending a packet to yourself, which isn't allowed.
    if (hdr.srcNodeId.phyId == hdr.destNodeId.phyId)
    {
        logprintf("ERROR: Bad outgoing packet header: sent packet to self.\n");
        return (EINVAL);
    }
    return (EOK);
}

//
//	asySend  --  send async packet on bus
//
int Drvadapter::asySend(llaAsyHdr_t&hdr,  const uint8_t data[], const AsyTxCtrl_t& ctl)
{	return(m_asySendCtxt.asySend(hdr,data,ctl));	}			// pass down a level

//
//	asySend  --  send async packet on bus
//
int DrvAsySend::asySend(llaAsyHdr_t&hdr,  const uint8_t data[], const AsyTxCtrl_t& ctl)
{
    unsigned     tCode   = hdr.transactionCode;
    unsigned     timeout = 0;
    dbuf_t      *pDbuf   = NULL;
    uint32_t     quadlet = 0;
    if (checkasyheader(hdr) != EOK) return(EINVAL);			// sanity check packet

    //	Put generation count in header if not already present.
    if (hdr.srcNodeId.genCount == 0)
    {
        //	Get the current bus reset generation count
        int stat = Drvadapter::llacheck(llaGetGenerationCount (m_owner.getlla(), &hdr.destNodeId.genCount),"getGenerationCount");
        if (stat != EOK) return(stat);
        hdr.srcNodeId = hdr.destNodeId;
    }
	//	Create buffer pool if needed
    if (!m_txpool.initialized())												// if not initialized
    {	int stat = m_txpool.create(MEM_9K, NBR_DBUF_ASY, DBUF_PHYSICAL_MEMORY | DBUF_QUADLET_ALIGN);
    		if (stat != EOK) return(stat);										// no memory error
    	}

    //	Fill in remaining fields of packet header
    hdr.priority = IEEE1394_PRIORITY_MIN;							// priority
    //	Data items 4 bytes long go in "quadlet".  Bigger items allocate a dbuf. 
    /* Copy application data to the driver dBuf or quadlet */
    if ((tCode == IEEE1394_QUAD_WRREQ) || (tCode == IEEE1394_QUAD_RDRESP))
    {
        if (hdr.dataLength != 4)
        {
            logprintf("ERROR: Data length must be 4 bytes for quadlet request.\n");
            hdr.dataLength = 4;
        }
        quadlet = *(uint32_t*)data;											// get data now
        pDbuf = NULL;
    }
    else if ((tCode == IEEE1394_BLOCK_WRREQ) 
    		|| (tCode == IEEE1394_BLOCK_RDRESP)
    		|| (tCode == IEEE1394_LOCK_REQ))
    {
        /* We need to get a dbuf for Tx only when the packet to send */
        /* contains a payload.  Wait until we get dbuf               */
        //	***THIS IS TERRIBLE*** waits if no buffer
        for(;;)
        {
            pDbuf = dbufAlloc (m_txpool.getpool(), MEM_9K, 0);
            if (pDbuf)
                break;
            logprintf("Waiting for buffer for async output.\n");
            usleep(1000);														// wait 1MS
        }
    }
    if (pDbuf && hdr.dataLength)							// if have a dbuf and data
    {	//	Copy data to the dbuf.
       	size_t count = dbufCopyDataToBuf (
                                data, 							// from here
                               pDbuf,										// to this buf
                               0,												// no offset
                               hdr.dataLength);						// length
        if (count != hdr.dataLength)
        {
            logprintf("dbufCopyDataToBuf failed\n");
            dbufFree(pDbuf);											// release dbuf after fail.
            return (EINVAL);
        }
    }

    //	Transmit, retrying if adapter busy.
    for (;;)
    {
        int stat = asynctransmit(
                    hdr,
                    pDbuf,
                    ctl,
                    quadlet,
                    timeout);
        if (stat == EOK)
            break;				// sent successfully

        if (stat == EBUSY)
       	{	usleep(1000);												// wait 1ms
       		logprintf("Waiting for busy controller.\n");		// warn
       		continue;														// no send, must handle
       	}
        //	Sending failed.
        return(stat);
    }
    //	Wait for callback from transmission completion.
    bool success = sem.wait(0.002);										// wait, but not more than 2ms
    if (!success)																		// if timed out, very bad
    {	logprintf("ERROR: Asynchronous send timed out. Hardware problem.\n");
    		return(EIO);																	// bad send
    }
    return (EOK);
}
//
//	asynctransmit  -- actually does an async transmit
//
//	Internal
//
//	Returns a POSIX error code.
//	May return EBUSY, which indicates a need for an immediate retry.
//
int DrvAsySend::asynctransmit(llaAsyHdr_t& hdr, dbuf_t* pData, const AsyTxCtrl_t& ctl, uint32_t quadlet,  unsigned timeout)
{
    //	Timestamp the header if needed
    if (ctl.computeTimeStamp)
    {
   		uint32_t timestamp;
        int stat = Drvadapter::llacheck(llaReadLinkRegister(m_owner.getlla(), TSB12LV23_ISO_CYCLE_TIMER, &timestamp),"readLinkRegister");
        if (stat != EOK) return(stat);
        hdr.timeStamp = ((timestamp >> TSB12LV23_CYCLE_COUNT_SHFT)) & TSB12LV23_TIME_STAMP_MASK;
    }

    //	Fan out on header transaction code.
    switch (hdr.transactionCode)
    {
    case IEEE1394_QUAD_WRREQ:
        if (hdr.dataLength != 4) return(EINVAL);						// already checked elsewhere, but important.
        return(Drvadapter::llacheck(llaSendQWriteReqPkt (m_owner.getlla(),
                                  &hdr,
                                  quadlet,
                                  asyncsendcallbackstatic,
                                  this),"send of quadlet write request"));

    case IEEE1394_BLOCK_WRREQ:
        return(Drvadapter::llacheck(llaSendBWriteReqPkt (m_owner.getlla(),
                                  &hdr,
                                  pData,
                                  asyncsendcallbackstatic,
                                  this),"send of block write"));
                        
    case IEEE1394_QUAD_RDREQ:
        return(Drvadapter::llacheck(llaSendQReadReqPkt (m_owner.getlla(),
                                 &hdr,
                                 asyncsendcallbackstatic,
                                 this),"send of quadlet read request"));

    case IEEE1394_BLOCK_RDREQ:
        return(Drvadapter::llacheck(llaSendBReadReqPkt (m_owner.getlla(),
                                 &hdr,
                                 asyncsendcallbackstatic,
                                 this),"send of block read request"));

    case IEEE1394_QUAD_RDRESP:
        if (hdr.dataLength != 4) return(EINVAL);						// already checked elsewhere, but important.
        return(Drvadapter::llacheck(llaSendQReadRespPkt (m_owner.getlla(),
                                  &hdr,
                                  quadlet,
                                  asyncsendcallbackstatic,
                                  this,
                                  timeout),"send quadlet read response"));

    case IEEE1394_BLOCK_RDRESP:
        return(Drvadapter::llacheck(llaSendBReadRespPkt (m_owner.getlla(),
                                  &hdr,
                                  pData,
                                  asyncsendcallbackstatic,
                                  this,
                                  timeout),"send block read response"));

    case IEEE1394_WRRESP:

	 	if (hdr.dataLength != 0) return(EINVAL);						// already checked elsewhere, but important.
        return(Drvadapter::llacheck(llaSendWriteRespPkt (m_owner.getlla(),
                                  &hdr,
                                  asyncsendcallbackstatic,
                                  this,
                                  timeout),"send write response"));
        
    case IEEE1394_LOCK_REQ:
            return(Drvadapter::llacheck(llaSendLockReqPkt(m_owner.getlla(),
                                  &hdr,
                                  pData,
                                  asyncsendcallbackstatic,
                                  this),"send lock request"));
   		
    default:
       	break;
    }
    return(EINVAL);
}
//
//	asyncsendcallbackstatic  -- callback from LLA on send completion
//
//	Just passes to the appropriate object.
//
void DrvAsySend::asyncsendcallbackstatic(unsigned status, unsigned acknowledge, dbuf_t* pdata, void* pContext)
{
    	DrvAsySend* obj = (DrvAsySend*) pContext;					// get relevant object
	obj->asyncsendcallback(status,acknowledge,pdata);
}
//
//	asyncsendcallback  --  an async send has completed.
//
//	Wakes up the waiting sender.
//
void DrvAsySend::asyncsendcallback(unsigned status, unsigned acknowledge, dbuf_t* pdata)
{
    dbufFree(pdata);															// free dbuf if any
    sem.post();																	// wake up the sender.
}

//
//	asyRecvStart -- start asynchronous receive
//	
int Drvadapter::asyRecvStart()
{	return(m_asyRecvCtxt.start());}
//
//	asyRecvStop  --  close and free memory
//
int Drvadapter::asyRecvStop()
{	m_asyRecvCtxt.cleanup();								// shut down and clean up
	return(EOK);
}
//
//	class DrvAsySend
//
//	Cleanup  -- release any resources left attached
//
void DrvAsySend::cleanup()
{
	m_txpool.destroy();														// get rid of txpool if allocated
}
//
//	class DrvAsyRecv
//
//	start  -- start asynchronous receive
//
//	Just enables reception.
//
//	Returns POSIX error code.
//
int DrvAsyRecv::start()
{
    unsigned fCount = 1;
    /* check if asy reception is already ON */
    if (m_started) return(EOK);											// already on
    cleanup();																		// clean up any leftover buffers from last time
	//	Create buffer pool if needed.
	//	This really should be part of the asy. recv object.
    if (!m_rxpool.initialized())		// if not initialized
    {	int stat = m_rxpool.create(MEM_9K,
        		NBR_DBUF_ASY,DBUF_PHYSICAL_MEMORY | DBUF_QUADLET_ALIGN);
    		if (stat != EOK) return(stat);										// no memory error
    		// Allocate a linked list of dbuf for receiving requests packets
    		pReqDBuf = dbufMultiAlloc(m_rxpool.getpool(), (NBR_DBUF_ASY/2));
    		if (pReqDBuf == NULL)
		{
			logprintf("dbufMultiAlloc failed\n");
			cleanup();
			return (ENOMEM);
    		}
    		// Allocate a linked list of dbuf for receiving response packets
    		pRespDBuf = dbufMultiAlloc (m_rxpool.getpool(), (NBR_DBUF_ASY/2));
    		if (pRespDBuf == NULL)		// if failed
    		{
        		logprintf("dbufMultiAlloc failed\n");
        		cleanup();																// clean up buffers
			return (ENOMEM);
       	}     
    }
	//	Install the callback for packet receives.
    int stat = Drvadapter::llacheck(llaSetAsyInCb (m_owner.getlla(),
                        staticrecvcallback,
                        (void *) this,
                        LLA_ASY_REQ | LLA_ASY_RESP),"set asychronous callback");
    if (stat != EOK) { cleanup();  return(stat);		}							// fails

    //	Enable reception for async callbacks.
    stat = Drvadapter::llacheck(llaStartAsyRecv(m_owner.getlla(),													// fails
                          pReqDBuf,
                          LLA_ASY_REQ,
                          fCount,
                          staticreqrecyclecallback,
                          (void *) this)," start async receive");
    if (stat != EOK) { cleanup();  return(stat);		}							// fails


    /* Enable the reception for asynchronous RESPONSE packets */
    stat = Drvadapter::llacheck(llaStartAsyRecv (m_owner.getlla(),
                          pRespDBuf,
                          LLA_ASY_RESP,
                          fCount,
                          staticresprecyclecallback,
                          (void *) this),"start async receive (responses)");
    if (stat != EOK) { cleanup();  return(stat);		}							// fails
    m_started = true;																		// set as open
    return(EOK);																				// done
}
//
//	Cleanup  -- release any resources left attached
//
//	This can be called in an arbitrary state. It leaves the context stopped and all buffers released.
//
//	Called from a destructor.
//
void DrvAsyRecv::cleanup()
{
    dbuf_t *pBuf1 = NULL;
	//	Return buffers allocate for requests.
    int status = llaStopAsyRecv (m_owner.getlla(), &pBuf1, LLA_ASY_REQ);
    if (status != OK)
    {
        logprintf("llaStopAsyRecv (LLA_ASY_REQ) failed\n");
    }
    if (pBuf1) dbufFree (pBuf1);
    pReqDBuf = NULL;
    //	Return buffers allocated for responses
    dbuf_t *pBuf2 = NULL;
    status = llaStopAsyRecv(m_owner.getlla(), &pBuf2, LLA_ASY_RESP);
    if (status != OK)
    {
        logprintf("llaStopAsyRecv (LLA_ASY_RESP) failed\n");
    }
    if (pBuf2) dbufFree (pBuf2);
    pRespDBuf = NULL;
	//	Destroy buffer pool
	m_rxpool.destroy();													// release the pool
	m_started = false;														// no longer running
}
//
//	recvcallback  --  called for each incoming packet
//
//	CALLBACK PRIORITY - MUST NOT BLOCK
//
//	Called for each received dbuf with async data.
//
//	Note that the dbuf passed is only valid for this callback.
//
void DrvAsyRecv::recvcallback(dbuf_t* pData, unsigned status)
{
    llaAsyHdr_t    header;
    //	Fan out on packet status
    switch (status)
    {
    case IEEE1394_COMPLETE:  /*means ack complete has been sent*/
    case IEEE1394_GOOD: /*means ack pending has been sent*/
        break;																// normal
        
    case IEEE1394_BROADCAST:
        break; /* OK */
        
    case IEEE1394_DATA_CRC_ERROR:
        logprintf("ERROR: IEEE1394_DATA_CRC_ERROR\n");
        return;
      
    case IEEE1394_FORMAT_ERROR:
        logprintf("ERROR: IEEE1394_FORMAT_ERROR\n");
        return;

    default:
        logprintf("ERROR: Unknown status (%X) in asy recv callback\n", status);
        return;
    }
    //	Extract header from incoming packet
    int stat = Drvadapter::llacheck(llaDecodeAsyPkt (m_owner.getlla(), &header, pData),"decode async packet");
    if (stat != EOK) return;
    //	We have valid header and data -- pass it upwards
    m_owner.asyrecvcallback(header,pData);							// pass upwards
}
//
//	Buffer recycling
//
/*******************************************************************************
*
* NAME:   AsyReqRecycleCb()
*
* DESC:   Dynamic Buffers is a shared resource between the driver and the lla.
*         This callback called if the driver runs out of dBuf elements
*         (reception buffer) for Response packets.  At this moment, it will
*         automatically call this function to get some new dBuf.
*
* INPUT:  context: driver context
*
* OUTPUT: None
*
* RETURN: None
*
*/
void DrvAsyRecv::reqrecyclecallback(dbuf_t* pdata)
{
	//	Return buffer to pool
    Drvadapter::llacheck(llaStartAsyRecv (m_owner.getlla(), pdata,
                          LLA_ASY_REQ, 1,
                          staticreqrecyclecallback, this),"start async recv buffer recycling");

}
/*******************************************************************************
*
* NAME:   AsyRespRecycleCb()
*
* DESC:   Dynamic Buffers is a shared resource between the driver and the lla.
*         This callback called if the driver runs out of dBuf elements
*         (reception buffer) for Response packets.  At this moment, it will
*         automatically call this function to get some new dBuf.
*
* INPUT:  context: driver context
*
* OUTPUT: None
*
* RETURN: None
*
*/
void DrvAsyRecv::resprecyclecallback(dbuf_t* pdata)
{	//	return buffers to pool
    Drvadapter::llacheck(llaStartAsyRecv (m_owner.getlla(), pdata,
                          LLA_ASY_RESP, 1,
                          staticresprecyclecallback, this),"async response buffer recycle");
}

	
