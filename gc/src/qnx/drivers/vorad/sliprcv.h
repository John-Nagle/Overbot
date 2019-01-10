//
//	SLIP protocol, receive side
//
//	John Nagle
//	Team Overbot
//	April, 2002.
//
//	Eaton VORAD radars frame a fixed-format structure for
//	transmission over an async link using the old SLIP
//	protocol defined in RFC 1055.
//
#ifndef SLIPRECV_H
#define SLIPRECV_H
//
#include <inttypes.h>
//
//	SLIP special characters
//	
const uint8_t kSlipEND = 0300;
const uint8_t kSlipESC = 0333;
const uint32_t kMaxSlipPacketSize = 1006;				// Berkeley standard
//
//	class SlipRecv  --  receives bytes, delivers blocks
//
class SlipRecv {
private:
	uint8_t	m_buffer[kMaxSlipPacketSize];	// max size
	uint32_t	m_count;										// chars in buffer
	bool	m_escape;											// in escape mode	
public:
	SlipRecv();
	//	Callback - subclass to get packets
	void accept(uint8_t);										// accept one char
	virtual void deliver(const uint8_t msg[], uint32_t len) = 0;
};
//
//	Implementation - inline
//
inline SlipRecv::SlipRecv()
: m_count(kMaxSlipPacketSize+1),			// forces errors until sync
m_escape(false) {}									// not in escape mode
//	
//
//	accept --  called for each incoming character
//
inline void SlipRecv::accept(uint8_t ch)
{	if (m_escape)										// if last char was escape
	{	m_escape = false;	}						// accept this char
	else														// normal case
	{	switch (ch) {									// fan out on symbol
		case kSlipEND:									// end symbol
			if (m_count <= kMaxSlipPacketSize)	// if valid size, not in overflow mode
			{	deliver(m_buffer,m_count);	}	// deliver 
			m_count = 0;								// clear buffer
			return;											// done
		case kSlipESC:									// escape character
			m_escape = true;							// escape next char
			return;											// and ignore
		default:												// normal case
			break;											// handle below
		}
	}
	//	Normal case -- add char to buffer, if it fits
	if (m_count >= kMaxSlipPacketSize)		// if full, or no END yet
	{	deliver(m_buffer,0);							// indicate bad packet
		return;												// ignore oversize char
	}
	m_buffer[m_count++] = ch;					// store char
}
#endif // SLIPRECV_H

