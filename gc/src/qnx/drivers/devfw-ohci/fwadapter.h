//
//	fwadapter.h  --  FireWire adapter and its info
//
//	Uses Mindready interface.
//
//	John Nagle
//	Animats
//	January, 2002
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
#ifndef FWADAPTER_H
#define FWADAPTER_H
#include "mindreadyforcpp.h"
#include "mutexlock.h"
#include "drvadapter.h"
class Businfo;													// forward

/* Defines */

#define MEM_SIZE_ASY_RX         MEM_1M/4
#define MEM_SIZE_ISO_RX         MEM_1M
#define MEM_SIZE_AUTORESPONSE   MEM_16K

const size_t inDataSize = 16;								// max bytes we ever send (biggest is lock request)
const unsigned int eventQueueSize = 64;			// store this many events

//
//	Types of packets in queue from low-level driver to event handler.
//
    enum EventType_e                               /* Determine which kind of pk have*/
    {                                           /* been received by the driver     */

        EVENT_NONE             = 0,                       /* not event at all     */
        EVENT_IN_ASY_PK        = 1,                       /* Asy pk received      */
        EVENT_IN_ISO_PK        = 2,                       /* Iso pk received      */
        EVENT_IN_ISO_FRAME     = 3,                       /* Iso frame received   */
        EVENT_BUS_RESET        = 4,                       /* Bus reset detected   */
    };
   	struct AsyPkInfo_t            					     /* Info about Asy Packet received */
    {
        llaAsyHdr_t hdr;                                  /* 1394 Asy header      */
        uint8_t shortpayload[4];					/* payload, if it will fit */
    };

    	struct IsoPkInfo_t        					          /* Info about Iso Packet received */
    {
        llaIsoHdr_t hdr;                                  /* 1394 Iso header      */
    };


#define     MAX_NBR_SEG 16                  /* Max data ptr to Iso Frame data */
    struct IsoFrInfo_t             					    /* Info about iso Frame received  */
    {
        uint8_t     channel;                              /* Channel of Iso frame */
        uint8_t     nbrSeg;                               /* Nbr of data segments */
        uint32_t    totalSize;                            /* Iso frame size       */

        uint32_t    segSize  [MAX_NBR_SEG];               /* Size of each segment */
        void       *pSegData [MAX_NBR_SEG];               /* Ptr to data segment  */

    };

    struct BusResetInfo_t            			      /* Info about received seld-id pk*/
    {
        uint8_t     adpIdx;                               /* Adapter Index number */
    };

//
//	class Event_t  --  an event in the event queue
//
class Event_t
{
public:
    uint8_t       status;
    EventType_e   type;
    dbuf_t*	buffer;										// associated dbuf, if any 
    union
        {
            AsyPkInfo_t     asyPk;                    // Asy pk info         
            IsoPkInfo_t     isoPk;                      // Iso pk info           
            IsoFrInfo_t     isoFr;                      	// Iso Fr info           
            BusResetInfo_t  busReset;            // Bus reset notification
        } info;
public:
	Event_t(EventType_e etype)				// constructor with type
	: type(etype), buffer(0) {}
	Event_t()												// empty constructor
	: type(EVENT_NONE), buffer(0) {}
};
//
//	class Fwadapter  --  one FireWire adapter
//
//	But all adapters are indexed in a static table.
//
//	Interface to the local hardware.
//	Information about adapters only.
//	Node info is in Businfo
//
class Fwadapter: public Drvadapter {
private:
	Businfo&		m_owner;											// owning businfo object
	uint8_t m_inData[inDataSize];								// fixed buffer for async sends // ***TEMP***
	uint32_t m_adapterid;											// which adapter we are
	bool m_used;														// this adapter is used
	bool m_iamroot;													// this adapter is the root
	NodeInfo_t	m_nodeinfo;										// per-node info
	SelfIdPks_t m_selfidpackets;									// latest self ID packets
	ost::Mutex	m_selfidlock;											// lock on self ID data (very short)
private:
    static bool     m_terminate;                            		// thread should terminate																				// adapter info
	static pthread_t		m_thread;                            	// thread that processes events from adapters
	static uint32_t      m_nbrOfAdp;                          	// number of adapters on this machine
	static Fwadapter* m_adapters[MAX_ADAPTERS];	// index of all adapters
	static ost::BoundedBuffer<Event_t, eventQueueSize> m_eventqueue;	// the event queue
public:
	Fwadapter(Businfo& bus,  uint32_t adapterid);
	~Fwadapter();
	int initadapter();													// initialize this adapter
	void cleanupadapter();											// release, for this adapter
	//	Static functions
	static void isocallback(uint32_t adapterid, const  llaIsoHdr_t& pkt, dbuf_t*& buffer);		// isochronous receive event for all adapters
	static Fwadapter* getadapter(uint32_t adapterid)
	{	return((adapterid<MAX_ADAPTERS) ? m_adapters[adapterid] : 0); }
	bool putevent(const Event_t& event);					// low-level driver puts events here
	static bool isnodeouradapter(uint32_t node);		// is this node one of our adapters?
	//	Per-object functions  
	void getselfidpackets(SelfIdPks_t& pkts);				// get self ID packets
	const NodeInfo_t&  getnodeinfo() const { return(m_nodeinfo); } 
	int updaterominfo();												// update info from adapter ROM
	int storexmitdata(const uint8_t data[],size_t size);	// store data for next transmission
	uint8_t* getxmitdatabuf() 										// get address of buffer for send
	{	return(m_inData);	}											// return buffer address
	void processevent(Event_t& event);					// process an event for this object
	void isocallback(const  llaIsoHdr_t& pkt, dbuf_t*& buffer);		// isochronous receive event for this adapters
private:																		// our local thread
	void processbusresetevent(const BusResetInfo_t& busResetInfo);
	void processasypkt(const AsyPkInfo_t& pPkInfo,  const dbuf_t* buffer);
	void  waitforevent();												// worker for the thread
	static void* startwaitforevent(void* param) 		// object of thread fork
	{	Fwadapter* obj = (Fwadapter*)param;			// coerce to object
		obj->waitforevent();											// run
		return(param);													// done
	}
protected:
	//	Callbacks from low-level driver
	void selfidcallback(const SelfIdPks_t& selfIdPks);					// subclass to get incoming bus resets
	void asyrecvcallback(llaAsyHdr_t& hdr, dbuf_t* pData);		// subclass to get incoming async packets
	////virtual void isorecvpktcallback(llaIsoHdr_t& hdr, dbuf_t* pData)  {}// subclass to get incoming single  iso. packet
	void isorecvblockcallback(uint32_t channel, dbuf_t* pData, int status);			// subclass to get incoming iso. packet block
	virtual void isopurgecallback(uint32_t channel);
};
#endif // FWADAPTER_H