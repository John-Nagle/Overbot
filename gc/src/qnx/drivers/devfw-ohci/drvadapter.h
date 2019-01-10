//
//	drvadapter.h  --  low level driver for OHCI adapter
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
#ifndef DRVADAPTER_H
#define DRVADAPTER_H

#include "mindreadyforcpp.h"
#include "drvadapterintern.h"
#include <ieee1394.h>
//
//	Constants
//
const uint32_t NO_IRM  =                      0x3F;  /* IRM number when no IRM on bus  */
const uint32_t DEFAULT_BUS_MNG_NUMBER  =      0x3F;  /* BM  number when no BM  on bus  */
const uint32_t BUS_MANAGER_MASK =       0x0000003F;  /* Bus manager mask for ROM       */

//
//	AsyTxCtrl_t  -- control info for async transmission
//
struct AsyTxCtrl_t             					  		/* Control the asy transmission   */
{
	bool     computeTimeStamp;              	/* Time stamp flag      */
	uint32_t transactionTimeuSec;            	/* Time used to send    */
};
//
//	IsoRxDC_t   -- control for async reception
//
struct IsoRxDC_t                   						
{
	unsigned channel;                                 /* Channel to listen    */
	unsigned tagMatch;                               /* Tag match            */
	unsigned cycleDelay;                            /* Cycle delay          */

	bool bufferFillingMode;                          /* BFM flag             */
	unsigned nbBytesInFillMode;                 /* Nbr byte in BFM      */

	bool stripHeader;                               	  /* Strip header flag    */
	unsigned synchMatch;                           /* Synch Match          */
	unsigned timeStamp;                              /* Time stamp           */
};
//
//	SelfIdHeaderHdr_t  -- self ID packet information
//
struct SelfIdHeaderHdr_t            /* Self id paket information      */
{
	uint8_t phyId;                                /* Phy id number        */
	uint8_t gapCount;                             /* Gap count active     */
	uint8_t linkActive;                           /* link active flag     */
	uint8_t phySpeed;                             /* Max physical speed   */
	uint8_t phyDelay;                             /* Phy Delay            */
	uint8_t contender;                            /* Contender flag       */
	uint8_t powerClass;                           /* Device power class   */
	uint8_t numberOfPorts;                        /* Number of ports      */
	uint8_t initiatedReset;                       /* Init the bus reset   */
	uint8_t port[IEEE1394_NPORT_MAX];             /* Port                 */
};

struct SelfIdPks_t 
{
	uint8_t     numOfBlock;                          //	number of nodes found on the bus
	uint32_t    generationCount;                 // generation count of bus reset
	SelfIdHeaderHdr_t block [IEEE1394_NODE_NUMBER_MAX + 1];  //	 Block of self ID packet headers
};
//
//		PhyConfigDC  --  send physical configuration packets
//
struct PhyConfigDC_t               
{
        unsigned   generationCount;                      	 /* Generation count     */
        bool       setForceRoot;                         			 /* Become root flag     */
        llaPhyId_t phyId;                                			 /* local phy id         */
        bool       setGapCount;                           		/* Gap count flag       */
        unsigned   gapCount;                             		 /* Gap count value      */
};
//
//	Drvadapter --  low-level driver for one adapter
//
//	This is basically an encapsulation of the Mindready LLA, but could be converted to use a non-LLA approach.
//
class Drvadapter {
private:
	llaAdapter_t*	 m_lla; 									// LLA-level handle
	uint32_t     m_adapterid;  							// adapter number
	DrvAsySend    m_asySendCtxt;	
	DrvAsyRecv    m_asyRecvCtxt;
	DrvIsoSend    m_isoSendCtxt;
	DrvIsoRecv    m_isoRecvCtxt;
private:
	static uint32_t m_adaptersactive;												// number of adapters currently active
public:
	explicit Drvadapter(uint32_t adapterid)									// constructor
	: m_lla(0),m_adapterid(adapterid),
		m_asySendCtxt(*this),m_asyRecvCtxt(*this),m_isoSendCtxt(*this),m_isoRecvCtxt(*this){}	
	virtual ~Drvadapter();																// destructor
	virtual void cleanup();																// cleanup
	int init();																					// initialize
	//	Access
	llaAdapter_t* getlla() { return(m_lla); }										// get the LLA adapter 
	uint32_t getadapterid()	 { return(m_adapterid); }						// get adapter number
	static int llacheck(int status, const char* fnname);	// convert LLA errors to POSIX errors
	//	All of these return POSIX error codes.
	//	Asychronous functions
	int asySend(llaAsyHdr_t&hdr,  const uint8_t data[], const AsyTxCtrl_t& ctl);
	int asyRecvStart();
	int asyRecvStop();
	//	Isochronous functions
	int isoSend(const llaIsoHdr_t& hdr, const uint8_t data[]);
	int isoRecvStart(uint32_t channel, uint32_t framesize, uint32_t totalsize, bool waitforsync);	// start block mode
	int isoRecvStop(uint32_t channel);											// stop reception
	int isoEnableCycleMaster();
	int isoDisableCycleMaster();
	//	Bus reset functions
	int busReset(bool shortBusReset);
	int busResetRecvStart();
	int busResetRecvStop();
	//	Node info for this node (the OHCI adapter)
	int getPresentStatus(llaPresentStatus_t& presentStatus);
	int setForceRootFlag(bool force);
	int getPhyID(llaPhyId_t&  phyId); 
	int getNodeID(llaNodeId_t& nodeId);
	int getGenerationCount(uint32_t& gencount);
	int setConfigContender(bool wantToBeIRM);					// want to be IRM?
	int setGapCount(uint32_t value);									// set desired gap count
	//	Control and status registers
	int readCSR(unsigned offset, uint32_t& quadletVal);
	int writeCSR(unsigned offset, uint32_t quadletVal);
	int lockCSR(unsigned offset, uint32_t& quadletVal, uint32_t&	newQuadletVal);
	//	Link registers
	////int actionLinkRegister(ActionDC_e  action, uint32_t& quadletVal);
private:
	int asynctransmit (llaAsyHdr_t& hdr, dbuf_t* pData, const AsyTxCtrl_t& ctl, uint32_t quadlet,  unsigned timeout);
	void selfidcallback(const llaConfigIn_t* pConfigIn);						// self ID callback, object level
	static void selfidcallbackstatic(llaConfigIn_t* pConfigIn, void* pcontext)
	{	Drvadapter* obj = (Drvadapter*) pcontext;								// coerce to object
		obj->selfidcallback(pConfigIn);													// call at object level
	}
public:
	//	These must be defined in a subclass.
	//	These are all called at callback level - callee must not block.
	virtual void selfidcallback(const SelfIdPks_t& selfIdPks) = 0;				// subclass to get incoming bus resets
	virtual void asyrecvcallback(llaAsyHdr_t& hdr, dbuf_t* pData) = 0;	// subclass to get incoming async packets
	virtual void isorecvpktcallback(llaIsoHdr_t& hdr, dbuf_t* pData)  {}// subclass to get incoming single  iso. packet
	virtual void isorecvblockcallback(uint32_t channel, dbuf_t* pData, int status) {};		// subclass to get incoming iso. packet block
	virtual void isopurgecallback(uint32_t channel) {}
};
//
//	Unrelated stuff to be moved elsewhere.
//
//
//	NodeInfo_t -- general info about a node, local or remote
//
   struct NodeInfo_t       			            /* The General node  Information  */
    {
        bool        root;                                 /* is node the root     */
        llaNodeId_t nodeId;                               /* node, bus id gencount*/

        bool        respondingRom;                        /* Respond its ROM info */
        bool        respondingNodeInfo;                   /* Repsond its nodeinfo */

        /* From Bus Info Block */
        uint32_t    vendorID;                             /* Vendor ID number     */
        uint8_t     chipIdHigh;                           /* Chip Unique ID High  */
        uint32_t    chipIdLow;                            /* Chip Unique ID Low   */
        bool        irmc;                                 /* Iso Ress Manager Cap */
        bool        cmc;                                  /* Cycle Master Capable */
        bool        isc;                                  /* Isochronous  Capable */
        bool        bmc;                                  /* Bus Manager  Capable */
    };

#endif // DRVADAPTER_H