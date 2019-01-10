//
//	businfo.h  --  device index for hot-pluggable devices
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
#ifndef BUSINFO_H
#define BUSINFO_H
#include "mindreadyforcpp.h"								// WORKAROUND for sizeof(bool) problem
#include <limits.h>
#include <string>
#include <vector>
#include <dirent.h>
#include "mutexlock.h"
#include "devindex.h"
#include "fwadapter.h"
//
class Devindex;
class Businfo;
class Busentry;

const uint32_t maxBusNode =63;				// max allowed bus node number
const uint32_t invalidBusNode = 0xffffffff;	// invalid bus node number
//
//	class Shortbusmsg -- a short message on the bus - 0, 4 or 8 bytes of payload
//
const uint32_t maxShortBusMsg = 8;		// max bytes in a short bus message
class Shortbusmsg {
private:
	 llaAsyHdr_t	m_header;						// message header
	 uint8_t 	m_data[maxShortBusMsg];	// associated data
	 uint32_t	m_size;									// size of associated data
public:
	Shortbusmsg(): m_size(0) {}				// clear to 0
	Shortbusmsg(const llaAsyHdr_t& hdr, const uint8_t data[], uint32_t size);
	const llaAsyHdr_t& getheader() const { return(m_header); }
	const uint32_t getquadlet() const;		// get 4 bytes as a 32-bit value
};
//
//	class Busresetinfo  --  info obtained on a bus reset
//
class Busresetinfo {
private:
	Businfo& m_owner;								// owning object backlink
public:
	SelfIdPks_t	m_selfIdPks;                     	//	latest set of self ID packets
	uint32_t		m_rootNode;                   	//  Root Node number  
	uint32_t		m_IRM;                           	//	Isochronous resource manager node number
	uint32_t		m_BM;                             	// bus manager node number
private:
	uint32_t		m_nbrOfNode;                 	//	number of nodes on the bus
	NodeInfo_t	m_nodeInfo [IEEE1394_MAX_NB_NODE];    //	Node information
public:
	Busresetinfo(Businfo& bus):m_owner(bus) {}
	int updateonbusreset1();						// update on a bus reset, part 1
	int updateonbusreset2();						// update on a bus reset, part 2
	unsigned int getnodecount() const { return(m_nbrOfNode); }
	NodeInfo_t*	getnodeinfo(unsigned int i)
	{	if (i >= getnodecount()) return(0);	// out of range
		return(&m_nodeInfo[i]);					// get node
	}
};
//
//	class Buschannels  --  isochronous channels on the bus
//
class Buschannels {
private:
	Businfo& m_owner;								// owning object backlink
	Busentry* m_receivers[MAX_ISO_CHANNELS];	// all possible channels
public:
	Buschannels(Businfo& owner);			// constructor
	int listenchannel(Busentry& busdev, uint32_t channel, uint32_t framesize, uint32_t totalsize, bool waitforsync);	// start listening on this channel
	int releasechannel(Busentry& busdev);	// release channels pointing to this device
	void releaseall(); 									// release all channels
	Busentry* findisolistener(uint32_t channel)	// get iso listener for requested channel
	{	if (channel >= MAX_ISO_CHANNELS) return(0);	// fails if out of range
		return(m_receivers[channel]);			// return indicated channel
	}
};
//
//	class Businfo -- one 1394 bus adapter and information about it
//
class Businfo: public Devindex {
private:
	unsigned int m_adapter;						// adapter number
	bool m_initialized;									// driver has been initialized
	bool m_terminate;									// terminate if true
	bool m_needinventory;							// true if need to inventory the bus
	llaNodeId_t m_nodeid;							// our node number, including gen count and adapter number
	uint32_t m_transactionlabel;					// cyclic serial number for async transactions
	ost::BoundedBuffer<Shortbusmsg,1> m_replies; // incoming messages that look like replies
	pthread_t m_inventorythread;				// inventory thread
	ost::Semaphore m_inventorywait;			// inventory task waits for work
	Fwadapter	m_interface;						// interface to the adapter
	//	Basic bus info, updated on a bus reset.
	Busresetinfo m_busresetinfo;				// the bus reset info
	//	Bus channel allocation
	Buschannels m_buschannels;				// isochronous channel info
public:
	Businfo(unsigned int adapter);
	virtual ~Businfo()  { shutdown(); }
	int startup();
	void shutdown();
	void notebusreset();								// bus reset noted, must re-inventory
	uint32_t getadapter() const { return(m_adapter); } // which adapter on local machine
	const llaNodeId_t& getnodeid() const { return(m_nodeid); }
	bool isrootnode();
	bool getterminate() { return(m_terminate);	}// true if shutting down
	Fwadapter& getinterface() { return(m_interface); }	// Note - assumes only one adapter
	Buschannels& getbuschannels() { return(m_buschannels);	}	
	bool isup() const { return(!m_needinventory); }	// true if bus up for real work
public:														// for now
	void asyncrecv(const llaAsyHdr_t& hdr, const uint8_t data[], uint32_t datasize);	// incoming async packet
	//	Short bus I/O transactions.
	int readquadlet(uint32_t destNode, uint64_t destAddr, uint32_t& quadVal);
	int writequadlet(uint32_t destNode, uint64_t destAddr, uint32_t quadVal);
	int compareandswapquadlet(uint32_t destNode, uint64_t destAddr, uint32_t oldquadVal, uint32_t newquadval);
	int readcsr(uint32_t destNode, uint32_t offset, uint32_t& quadVal);
	int readcsr(uint32_t destNode, uint32_t offset, uint64_t& octletVal);
	int compareandswapcsr(uint32_t destNode, uint32_t offset, uint32_t oldQuadVal, uint32_t newQuadVal);
	int readlocalrominfo(NodeInfo_t& pNodeInfo);
	//	General bus operations
	int asynctransaction(llaAsyHdr_t&hdr,  const uint8_t data[], const AsyTxCtrl_t& ctl, Shortbusmsg& reply, uint32_t retries=5);
	void processbusresetevent(const BusResetInfo_t& busResetInfo);
	int busreset(bool becomeRoot, bool shortBusReset);
	//	Isochronous mode
	void processisoframe(const IsoFrInfo_t& frame, dbuf_t*& buffer);
	void processisopkt(const  llaIsoHdr_t& pkt, dbuf_t*& buffer);
	void processisoblock(uint32_t channel, dbuf_t* buffer, unsigned status);
	void processisopurge(uint32_t channel);
	int isoenable(uint32_t channel, uint32_t framesize, uint32_t totalsize, bool waitforsync);	// start block mode
	int isodisable(uint32_t channel);					// disable this channel
	int isochannelallocate(int& chan);
	int isochannelrelease(int chan);
	//	Naming
	Deventry* find(const string& name)				// pass to parent
	{	return(Devindex::find(name));	}
	Deventry* find(const char* name)				// pass to parent
	{	return(Devindex::find(name));	}
protected:
	bool purgeio();												// cancel all I/O
private:
	int getgeneration(uint32_t& gen);					// get generation number
	int checkgeneration(uint32_t gen);				// check generation number
	Busentry* find(uint32_t vendor, uint32_t hiid, uint32_t lowid);	// find by hardware ID
	uint32_t gettransactionlabel() {m_transactionlabel = (m_transactionlabel +1 )% 0x40; return(m_transactionlabel); }
	int makemerootnode();
	int asynctransactiontry(llaAsyHdr_t&hdr,  const uint8_t data[], const AsyTxCtrl_t& ctl, Shortbusmsg& reply);
	int handlebusreset();									// handle bus reset
	void inventorytask();									// inventories the bus
	static void* startinventorytask(void* param) // object of thread fork
	{	Businfo* obj = (Businfo*)(param);			// cast into object
		obj->inventorytask();								// run
		return(param);											// unused
	}
	int updatedev(uint32_t nodenumber, uint32_t vendor, uint32_t hiid, uint32_t lowid);
	Busentry* findisolistener(uint32_t channel);	// find who wants to listen to this channel
	uint64_t csraddr(uint32_t offset)					// control/status register address from offset
	{    return(0xFFFFF0000000llu | offset);	}	// build destination bus address

};
//
//	class Busentry  -- one node on the 1394 bus
//
//	Abstract class - subclassed for each driver.
//
class Busentry: public Deventry {
private:
	uint32_t m_busnode;								// node on the bus; changes at each bus reset
	uint32_t m_vendor;								// vendor ID
	uint32_t m_hiid;										// high ID (usually model number)
	uint32_t m_lowid;									// low ID (usually serial number)
public:
	Busentry(Businfo& bus,
		uint32_t initialnodenumber, uint32_t vendor, uint32_t hiid, uint32_t lowid)
	:	Deventry(bus),									// for parent
		m_busnode(initialnodenumber),		// set initial node number (changes on bus reset)
		m_vendor(vendor), m_hiid(hiid), m_lowid(lowid)
	{}
	bool valid() { return(m_busnode <= maxBusNode); }	// invalid when device goes offline
	bool compare( uint32_t vendor, uint32_t hiid, uint32_t lowid)
	{	return(vendor == m_vendor && hiid == m_hiid && lowid == m_lowid);	}
	Businfo& getbus() { return((Businfo&)getindex());}	// NOTE unchecked downcast
	uint32_t getbusnode() const { return(m_busnode); }
	void setbusnode(uint32_t busnode)
	{	m_busnode = busnode;	}				// set node on bus
	virtual void isorcvpkt(const llaIsoHdr_t& hdr, dbuf_t*& buffer) {}
	virtual void isorcvblock(uint32_t channel, dbuf_t* buffer, unsigned status) {}
	virtual void isorcvpurge() {}
	//	Bus reset activity
	virtual void busresetbegin() { m_busnode = invalidBusNode; };	// a bus reset has started; stop everything
	virtual void busresetdone() {};			// a bus reset has completed; restart what was stopped
	//	Access
	virtual const char* getnameprefix() const = 0;	// prefix names with this
	virtual int setidentity() = 0;					// get identity info from device
	virtual void builddefaultname(string& s);	// build default device name 
};
#endif // BUSINFO_H

