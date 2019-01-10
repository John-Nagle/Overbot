//
//	drviso.cpp  --  low level driver for OHCI adapter, isochronous part
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
//	Isochronous receive
//
//	Uses buffer-filling mode, which receives a fixed-size multi-packet frame without
//	software intervention. This is necessary to handle uncompressed video without
//	losing packets.
//
//	(A note from Mindready's documentation follows.)
//
/*******************************************************************************
*
*           When you are using BUFFER-FILLING mode, you must considere three things:
*            1- The data frame size you want
*            2- The data segment size of each dbuf
*            3- The number of dbuf for a frame
*
*           You must try respect these two formulas to get the best performances and not
*           having packet split over two frames.
*
*               fCount  x  data_segment_size  =  nbOfBytes   =   frame_size
*
*                                     and
*
*               frame_size    MODULUS   data_segment_size  =  0

*/

////#define NBR_DBUF                 32



/* You will optimize the buffer filling mode by modifying */
/* those three values                                     */

////#define DATA_SEG_SIZE            MEM_10K
////#define FRAME_SIZE               MEM_1M
const uint32_t RECYCLE_FREQUENCY_COUNT = 1;							// DO NOT set to 0 - system will hang
const uint32_t ISO_DBUF_COUNT = 2;											// 2 = double buffering
//
//	start  --  start isochronous reception on the given channel.
//
int DrvIsoRecv::start(uint32_t channel, uint32_t framesize, uint32_t totalsize, bool waitforsync)	// start reception
{
	if (channel >= MAX_ISO_CHANNELS) return(EINVAL);// channel out of range
	return(m_channels[channel].start(this,channel,framesize,totalsize,waitforsync));
}
//
//	stop -- stop isochronous reception
//	
int DrvIsoRecv::stop(uint32_t channel)
{	logprintf("DrvIsoRecv::stop channel %d\n",channel);				// ***TEMP***
	if (channel >= MAX_ISO_CHANNELS) return(EINVAL);
	return(m_channels[channel].stop());										// stop requested channel
}
//
//	cleanup  -- shut down isochronous reception if running and clean up.
//
void DrvIsoRecv::cleanup()
{	// clean up all channels
	for (unsigned int i=0; i<MAX_ISO_CHANNELS; i++)
	{	m_channels[i].cleanup();	}
}

//
//	Class DrvIsoRecvChan  -- one channel for isochronous receives
//
//	start  -- start up this channel
//
int DrvIsoRecvChan::start(DrvIsoRecv* owner,  uint32_t channel, uint32_t framesize, uint32_t totalsize, bool waitforsync)
{	if (channel >= MAX_ISO_CHANNELS) return(EINVAL);// channel out of range
	if (m_ctxHandle >= 0) return(EBUSY);						// channel in use
	m_owner = owner;													// save owner, which is always the enclosing object.
	m_channel = channel;												// receiving on this channel
	m_frameSize = framesize;											// with this packet payload size

    /* Configure a rx DMA for isochronous reception          */
    /* DMA context handle is returned via ctxHandle variable.*/

    llaChanListConfig_t   chanListConfig;              /* To config iso RX  */
    chanListConfig.configFlags = ISO_CONT_CREATE_CTX;
    if (totalsize > 0)											// if fixed size, use buffer fill mode
    {	chanListConfig.configFlags |= ( ISO_CONT_BUFFER_FILL | ISO_CONT_STRIP_HDR); }
    if (waitforsync)																		// if wait for sync, set 
    {	chanListConfig.configFlags |= ISO_CONT_SYNCH_MATCH; }
   	chanListConfig.ctxHandle   = &m_ctxHandle;						// one per channel
    chanListConfig.tagMatch    = 0;
    chanListConfig.syncMatch   = waitforsync ? 1 : 0;					// set to wait for sync frame ???
    chanListConfig.cycleDelay  = 0;
    chanListConfig.chanList.list[0]    = m_channel;						// listen on one channel
    chanListConfig.chanList.listLength = 1;

	//	Configure isochronous reception for this channel and mode set.
    int stat = Drvadapter::llacheck(llaSetChannelList (getowner().getowner().getlla(), &chanListConfig),"set isochronous channel list");
    if (stat != EOK) 
    {	cleanup(); return(stat); }							// handle error
	//	Register the isochronous receive callback.  This is called when the requested data has been received.
    stat = Drvadapter::llacheck(llaSetIsoInCb (getowner().getowner().getlla(),
                        m_ctxHandle,
                        staticrecvcallback,
                        this),"set isochronous callback");
    if (stat != EOK) 
    	{	 cleanup(); return(stat); }							// handle error

    //	Create a receive buffer pool, and a buffer chain for receive.
	m_nbBytesInFillMode = totalsize;
	//	Just allocate one huge buffer of the requested size. Why get fancy?
	///size_t datasegsize = totalsize;
	///uint32_t nbrdbuf = 1;
	m_frameSize = m_nbBytesInFillMode;			// force big buffer
	//	Try one buffer per packet
	size_t datasegsize = m_frameSize;
	uint32_t nbrdbuf = m_nbBytesInFillMode / datasegsize;
	if (nbrdbuf * datasegsize != m_nbBytesInFillMode)
	{	logprintf("ERROR: total size %d bytes not a multiple of packet payload size %d bytes.\n",
			m_nbBytesInFillMode,datasegsize);
			return(EINVAL);
	}
	nbrdbuf *= ISO_DBUF_COUNT;						// double buffer, or whatever
    stat = m_rxbufpool.create(datasegsize, nbrdbuf, DBUF_PHYSICAL_MEMORY | DBUF_QUADLET_ALIGN);
    if (stat != EOK)
    {	cleanup(); return(ENOMEM); }
	//	Allocate a buffer chain big enough for whatever we're receiving.
    dbuf_t* pDbuf  = dbufMultiAlloc (m_rxbufpool.getpool(), nbrdbuf);
    if (pDbuf == NULL)
   	{	cleanup(); return(ENOMEM);	}
	//	Actually start isochronous reception.
    stat = Drvadapter::llacheck(llaStartIsoRecv (getowner().getowner().getlla(),
                          pDbuf,                          /* Rx dbuf                      */
                          m_ctxHandle,              /* DMA context handle           */
                          m_nbBytesInFillMode,               /* nbrOfByte in buff-fill mode  */
                          0,                              /* configFlag: for futur use    */
                          RECYCLE_FREQUENCY_COUNT,        /* nbr dbuf full before recycle cb*/
                          staticrecyclecallback,                      /* Recycle callback             */
                          this),"start isochronous reception");                         /* User context                 */
    if (stat != EOK) 
    {	dbufFree(pDbuf); cleanup(); return(stat); }							// handle error
	//	***TEMP*** debug print
    logprintf ("Iso channel %d started\n",m_channel);
    logprintf ("  Frame size:             %7d\n", m_frameSize);
    logprintf ("  Data Segment Size:      %7d\n", m_nbBytesInFillMode);
    logprintf ("  Recycle Freqency Count: %7d\n", RECYCLE_FREQUENCY_COUNT);
    return(EOK);	
}																			// success
//
//	stop -- stop iso recv for this channel
//
//	This releases the entire buffer pool, which seems a bit drastic.
//	But we have no idea what size buffers we need next time.
//
int DrvIsoRecvChan::stop()
{	if (m_ctxHandle >= 0)																// if isochronous reception started
	{	dbuf_t* pDbuf;																		// dbuf returned after start
		int stat = Drvadapter::llacheck(llaStopIsoRecv(getowner().getowner().getlla(), &pDbuf, m_ctxHandle),"stop isochronous reception");	
		if (stat == EOK)																	// if stopped OK
		{	if (pDbuf) dbufFree(pDbuf);	}											// return dbuf if needed
		//	Remove isochronous reception callback
   		stat = Drvadapter::llacheck(llaSetIsoInCb (getowner().getowner().getlla(), m_ctxHandle, NULL, NULL),
    			"remove isochronous callback");

   		//	Delete the isochronous reception context.
    		llaChanListConfig_t   chanListConfig;            							// just to turn off
    		chanListConfig.configFlags = ISO_CONT_DELETE_CTX;
    		chanListConfig.ctxHandle   = &m_ctxHandle;
    		stat = Drvadapter::llacheck(llaSetChannelList (getowner().getowner().getlla(), &chanListConfig),"clear isochronous channel list");
		//	Release any stored dbuf copies being used by the recipient of packets
		getowner().getowner().isopurgecallback(m_channel);	// purges at higher level
		//	Release dbuf pool
		m_rxbufpool.destroy();															// release
		m_ctxHandle = -1;																// no longer running
	}
	return(EOK);
}
void DrvIsoRecvChan::cleanup()
{	stop();	}
//
//	recvcallback  -- isochronous receive callback
//
/*
* DESC:   This callback is invoked when FRAME_SIZE bytes of isochronous data
*         have been received (including header+trailer+payload.)
*
*         The dbuf you get is not a dbuf you passed to the LLA.  The pData is a 
*         spawned dbuf that belongs to the LLA (you could not use dbufFree() with it)
*         note: the pData is valid only during the reception callback.
*         
*         The pData ptr might points to more than one dbuf.  Especialy if
*         FRAME_SIZE > data_segment_size.
*
*         Note: Try to exit the function ASAP because the same function is use 
*         for all incoming packets.  (Do not use sem_wait() like functions.)
*
*/
void DrvIsoRecvChan::recvcallback(dbuf_t* pData, unsigned status)
{
    if (status != IEEE1394_GOOD)
    {
        logprintf ("ERROR Bad isochronous packet status\n");
        getowner().getowner().isorecvblockcallback(m_channel,0,EIO);		// report I/O error
        return;
    }
    getowner().getowner().isorecvblockcallback(m_channel, pData,status);	// pass upwards
} 


//
//	recyclecallback  --  called by LLA when it wants to return a buffer to the pool for this device
//
void DrvIsoRecvChan::recyclecallback(dbuf_t* pData)
{
    if (!(m_ctxHandle >= 0))											// if called after shutdown
    {
        dbufFree(pData);													// already stopped; just free dbuf.
    }
    else
    {
		//	Return dbuf to pool so the driver can use it for incoming data.
        int stat = Drvadapter::llacheck(llaStartIsoRecv (getowner().getowner().getlla(),
                              pData,												// buffer being returned
                              m_ctxHandle,									// low-level open
                              m_nbBytesInFillMode,                     // number of bytes to wait for next time
                              0,                             						// unused
                              RECYCLE_FREQUENCY_COUNT,        // low water mark for next call
                              staticrecyclecallback,                     // this callback, at non-object level
                              this),"restart isochronous receive in callback");                      /* User context                 */
        if (stat != EOK)
        {	
            //	***NEED TO MARK I/O AS FAILURE***
        }
    }
}
//
//	Isochronous send support
//
//	Currently unimplemented
//
//
//	cleanup  -- shut down isochronous reception if running and clean up.
//
void DrvIsoSend::cleanup()
{}

