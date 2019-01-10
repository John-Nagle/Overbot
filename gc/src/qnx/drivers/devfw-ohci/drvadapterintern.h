//
//	drvadapterintern.h  --  low level driver for OHCI adapter
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
#ifndef DRVADAPTERINTERN_H
#define DRVADAPTERINTERN_H

#include "mindreadyforcpp.h"
#include "tsb12lv23.h"                      							
#include "tsb41lv3.h"                     
#include "mutexlock.h"
#include "drvconfig.h"
#include "utilities.h"

class AsyTxCtrl_t;
class DrvIsoRecv;
class Drvadapter;															// forward
//
//	class DrvAsySend  -- the asychronous send component of the low-level part of the driver.
//
class DrvAsySend
{
	Drvadapter&				m_owner;								// owning adapter							
	DbufPool m_txpool;													// For outgoing data 
	ost::Semaphore sem;												// sender waits for adapter to finish
public:
	
	DrvAsySend(Drvadapter& owner)				//	Constructor
		: m_owner(owner)
	{}
	void cleanup();															// release buffers, etc.
	~DrvAsySend()											// destructor
	{	cleanup();	}
	int asySend(llaAsyHdr_t& hdr,  const uint8_t data[], const AsyTxCtrl_t& ctl);
private:
	int asynctransmit (llaAsyHdr_t& hdr, dbuf_t* pData, const AsyTxCtrl_t& ctl, uint32_t quadlet,  unsigned timeout);
	void asyncsendcallback(unsigned status, unsigned acknowledge, dbuf_t* pdata);
	static void asyncsendcallbackstatic(unsigned status, unsigned acknowledge, dbuf_t* pdata, void* pContext);
};
//
//	class DrvIsoSend  -- isochronous send support
//
class DrvIsoSend
{
	Drvadapter&				m_owner;							// owning adapter							
	DbufPool								m_txpool;
	dbuf_t *pDbuf; 
	uint32_t generationCount;
	ost::Semaphore sem;
public:
	void cleanup();
	~DrvIsoSend()
	{	cleanup();	}
	DrvIsoSend(Drvadapter& owner)				//	Constructor
	: m_owner(owner),pDbuf(0),generationCount(0)
	{}
};
//
//		class DrvAsyRecv  -- asynchronous receives
//
class DrvAsyRecv
{	Drvadapter&				m_owner;							// owning adapter							
	bool							m_started;								// true if started
    uint32_t					generationCount;
	uint32_t					rxTaskStarted;
	uint8_t						tstLvl;
	uint32_t					byteCntr;
	uint32_t					rxMode;
	DbufPool					m_rxpool;
	dbuf_t*						pReqDBuf;
	dbuf_t*						pRespDBuf;
	ost::Semaphore sem;
public:
	DrvAsyRecv(Drvadapter& owner)											//	Constructor
	: m_owner(owner),m_started(false),generationCount(0),rxTaskStarted(0),tstLvl(0),byteCntr(0),rxMode(0),
	pReqDBuf(0),pRespDBuf(0)
	{}
	void cleanup();														// release buffers, etc.
	int start();																// start receive
	~DrvAsyRecv()											// destructor
	{	cleanup();	}
private:
	//	Callbacks from LLA
	void recvcallback(dbuf_t* pdata, unsigned status);
	static void staticrecvcallback(dbuf_t* pData, unsigned status, void* context)
	{	DrvAsyRecv* obj = (DrvAsyRecv*) context;	// get relevant object
		obj->recvcallback(pData,status);						// call object callback
	}
	void reqrecyclecallback(dbuf_t* pdata);
	static void staticreqrecyclecallback(dbuf_t* pData, void* context)
	{	DrvAsyRecv* obj = (DrvAsyRecv*) context;	// get relevant object
		obj->reqrecyclecallback(pData);			// call object callback
	}
	void resprecyclecallback(dbuf_t* pdata);
	static void staticresprecyclecallback(dbuf_t* pData, void* context)
	{	DrvAsyRecv* obj = (DrvAsyRecv*) context;	// get relevant object
		obj->resprecyclecallback(pData);			// call object callback
	}

};
//
//	class DrvIsoRecvChan  --  receive for a specific channel
//
class DrvIsoRecvChan {
	DrvIsoRecv*						m_owner;										// owning iso recv state
	uint32_t							m_channel;									// isochronous channel
	int										m_ctxHandle;								// relevant context handle
	DbufPool							m_rxbufpool;									// receive buffer pool for this channel
	uint32_t							m_nbBytesInFillMode;
	uint32_t							m_frameSize;								// size of each frame
public:
	DrvIsoRecv& getowner() const { return(*m_owner); }			
	DrvIsoRecvChan()
	: m_owner(0),m_ctxHandle(-1)
	{}
	int start(DrvIsoRecv* owner, uint32_t channel, uint32_t framesize, uint32_t totalsize, bool waitforsync);		// start reception
	int stop();
	void cleanup();
	~DrvIsoRecvChan()
	{	cleanup(); 	}
	//	Callbacks from LLA
	void recvcallback(dbuf_t* pdata, unsigned status);
	static void staticrecvcallback(dbuf_t* pData, unsigned status, void* context)
	{	DrvIsoRecvChan* obj = (DrvIsoRecvChan*) context;		// get relevant object
		obj->recvcallback(pData,status);						// call object callback
	}
	void recyclecallback(dbuf_t* pdata);
	static void staticrecyclecallback(dbuf_t* pData, void* context)
	{	DrvIsoRecvChan* obj = (DrvIsoRecvChan*) context;		// get relevant object
		obj->recyclecallback(pData);							// call object callback
	}
};
const uint32_t MAX_ISO_CHANNELS = 64;						// maximum number of isochronous channels

//
//	class DrvIsoRecv  -- isochronous receive state
//
class DrvIsoRecv
{	Drvadapter&					m_owner;										// owning adapter							
	DrvIsoRecvChan				m_channels[MAX_ISO_CHANNELS];	// the channels
public:
	DrvIsoRecv(Drvadapter& owner)											//	Constructor
	: m_owner(owner)
	{}
	Drvadapter& getowner() const { return(m_owner); }			
	int start(uint32_t channel, uint32_t framesize, uint32_t totalsize, bool waitforsync);		// start reception
	int stop(uint32_t channel);
	void cleanup();
	~DrvIsoRecv()
	{	cleanup(); }
private:
#ifdef OBSOLETE
	//	Callbacks from LLA
	void recvcallback(dbuf_t* pdata, unsigned status);
	static void staticrecvcallback(dbuf_t* pData, unsigned status, void* context)
	{	DrvIsoRecv* obj = (DrvIsoRecv*) context;		// get relevant object
		obj->recvcallback(pData,status);						// call object callback
	}
	void recyclecallback(dbuf_t* pdata);
	static void staticrecyclecallback(dbuf_t* pData, void* context)
	{	DrvIsoRecv* obj = (DrvIsoRecv*) context;		// get relevant object
		obj->recyclecallback(pData);							// call object callback
	}
#endif // OBSOLETE
};
//
//	Misc. constants
//
//
//	Memory size definitions
//
const size_t  MEM_1K  =  1024;
const size_t  MEM_2K  =     2*MEM_1K;
const size_t  MEM_3K  =     3*MEM_1K;
const size_t  MEM_4K   =    4*MEM_1K;
const size_t  MEM_5K   =    5*MEM_1K;
const size_t  MEM_6K   =    6*MEM_1K;
const size_t  MEM_7K   =    7*MEM_1K;
const size_t  MEM_8K    =   8*MEM_1K;
const size_t  MEM_9K   =    9*MEM_1K;
const size_t  MEM_10K  =   10*MEM_1K;
const size_t  MEM_16K   =  16*MEM_1K;

const size_t  MEM_1M  =  1048576; /* 0x00200000 */
const size_t  MEM_2M   = 2*MEM_1M;
const size_t  MEM_4M  =  4*MEM_1M; 
const size_t  MEM_8M  =  8*MEM_1M;
const size_t  MEM_16M =16*MEM_1M;
const size_t  MEM_32M = 32*MEM_1M;

#endif // DRVADAPTERINTERN_H