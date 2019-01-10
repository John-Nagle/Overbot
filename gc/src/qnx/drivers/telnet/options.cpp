//
//	Options table for Telnet option negotiation
//
//	John Nagle
//	Team Overbot
//	December, 2003.
//
//	
//	For suboption negotiation, receive side.
//
#include <inttypes.h>
#include "options.h"
#include "protocolrx.h"
//
//	Telnet suboptions which we understand.
//	
//	These are preceded by IAC SB. We then match the pattern here.
//
//	Every pattern must end with "k_endsuboption". Zero is a valid pattern item.
//
const uint32_t	p0[] = { TOPT_COMPORTCONTROL, COM_PORT_SIGNATURE_RECV, k_readescapedstring, TELNET_IAC, TELNET_SE, k_endsuboption };
const uint32_t	p1[] = { TOPT_COMPORTCONTROL, COM_PORT_SIGNATURE, k_readescapedstring, TELNET_IAC, TELNET_SE, k_endsuboption };
const uint32_t 	p2[] = { TOPT_COMPORTCONTROL, COM_PORT_SET_BAUDRATE, k_readanybyte, k_readanybyte, k_readanybyte, k_readanybyte , TELNET_IAC, TELNET_SE, k_endsuboption };
const uint32_t 	p3[] = { TOPT_COMPORTCONTROL, COM_PORT_SET_DATASIZE, k_readanybyte, TELNET_IAC, TELNET_SE, k_endsuboption };
const uint32_t 	p4[] = { TOPT_COMPORTCONTROL, COM_PORT_SET_PARITY, k_readanybyte, TELNET_IAC, TELNET_SE, k_endsuboption };
const uint32_t 	p5[] = { TOPT_COMPORTCONTROL, COM_PORT_SET_STOPSIZE, k_readanybyte, TELNET_IAC, TELNET_SE, k_endsuboption };
const uint32_t 	p6[] = { TOPT_COMPORTCONTROL, COM_PORT_SET_CONTROL, k_readanybyte, TELNET_IAC, TELNET_SE, k_endsuboption };
const uint32_t 	p7[] = { TOPT_COMPORTCONTROL, COM_PORT_SET_LINESTATE_MASK, k_readanybyte, TELNET_IAC, TELNET_SE, k_endsuboption };
const uint32_t 	p8[] = { TOPT_COMPORTCONTROL, COM_PORT_SET_MODEMSTATE_MASK, k_readanybyte, TELNET_IAC, TELNET_SE, k_endsuboption };
const uint32_t 	p9[] = { TOPT_COMPORTCONTROL, COM_PORT_PURGE_DATA, k_readanybyte, TELNET_IAC, TELNET_SE, k_endsuboption };
const uint32_t 	p10[] = { TOPT_COMPORTCONTROL, COM_PORT_NOTIFY_LINESTATE, k_readanybyte, TELNET_IAC, TELNET_SE, k_endsuboption };
const uint32_t 	p11[] = { TOPT_COMPORTCONTROL, COM_PORT_NOTIFY_MODEMSTATE, k_readanybyte, TELNET_IAC, TELNET_SE, k_endsuboption };
const uint32_t 	p12[] = { TOPT_COMPORTCONTROL, COM_PORT_FLOWCONTROL_SUSPEND, TELNET_IAC, TELNET_SE, k_endsuboption };
const uint32_t 	p13[] = { TOPT_COMPORTCONTROL, COM_PORT_FLOWCONTROL_RESUME, TELNET_IAC, TELNET_SE, k_endsuboption };
//
//	The table
//
const uint32_t* telnetsuboptions[] =
	{p0,p1,p2,p3,p4,p5,p6,p7,p8,p9,p10,p11,p12,0 };

