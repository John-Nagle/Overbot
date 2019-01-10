// ProtocolRx.h: interface for the CProtocolRx class.
//
//////////////////////////////////////////////////////////////////////

#ifndef PROTOCOLRX_H
#define PROTOCOLRX_H

#include <inttypes.h>
#include <string>

typedef int SOCKET;												// for BSD-type networking
const SOCKET INVALID_SOCKET = -1;					// for BSD-style networking
const SOCKET SOCKET_ERROR	 = -1;					// for BSD-style networking

enum verb_e
{	verb_none = 0,
    verb_sb   = 250,
    verb_will = 251,
    verb_wont = 252,
    verb_do   = 253,
    verb_dont = 254
};

enum state_e
{
    state_data,  			// we expect a data byte
    state_code,  			// we expect a code
    state_option,  		// we expect an option
};
enum option_e
{
    TOPT_BIN = 0,   // Binary Transmission
    TOPT_ECHO = 1,  // Echo
    TOPT_RECN = 2,  // Reconnection
    TOPT_SUPP = 3,  // Suppress Go Ahead
    TOPT_APRX = 4,  // Approx Message Size Negotiation
    TOPT_STAT = 5,  // Status
    TOPT_TIM = 6,   // Timing Mark
    TOPT_REM = 7,   // Remote Controlled Trans and Echo
    TOPT_OLW = 8,   // Output Line Width
    TOPT_OPS = 9,   // Output Page Size
    TOPT_OCRD = 10, // Output Carriage-Return Disposition
    TOPT_OHT = 11,  // Output Horizontal Tabstops
    TOPT_OHTD = 12, // Output Horizontal Tab Disposition
    TOPT_OFD = 13,  // Output Formfeed Disposition
    TOPT_OVT = 14,  // Output Vertical Tabstops
    TOPT_OVTD = 15, // Output Vertical Tab Disposition
    TOPT_OLD = 16,  // Output Linefeed Disposition
    TOPT_EXT = 17,  // Extended ASCII
    TOPT_LOGO = 18, // Logout
    TOPT_BYTE = 19, // Byte Macro
    TOPT_DATA = 20, // Data Entry Terminal
    TOPT_SUP = 21,  // SUPDUP
    TOPT_SUPO = 22, // SUPDUP Output
    TOPT_SNDL = 23, // Send Location
    TOPT_TERM = 24, // Terminal Type
    TOPT_EOR = 25,  // End of Record
    TOPT_TACACS = 26, // TACACS User Identification
    TOPT_OM = 27,   // Output Marking
    TOPT_TLN = 28,  // Terminal Location Number
    TOPT_3270 = 29, // Telnet 3270 Regime
    TOPT_X3 = 30,  // X.3 PAD
    TOPT_NAWS = 31, // Negotiate About Window Size
    TOPT_TS = 32,   // Terminal Speed
    TOPT_RFC = 33,  // Remote Flow Control
    TOPT_LINE = 34, // Linemode
    TOPT_XDL = 35,  // X Display Location
    TOPT_ENVIR = 36,// Telnet Environment Option
    TOPT_AUTH = 37, // Telnet Authentication Option
    TOPT_NENVIR = 39,// Telnet Environment Option
    TOPT_COMPORTCONTROL = 44, // Com Port Control Option (see RFC 2217)
    TOPT_EXTOP = 255, // Extended-Options-List
    TOPT_ERROR = 256  // Magic number
};
//
//	The Telnet codes. These appear in Telnet streams, but only after TELNET_IAC
//
enum telnet_code_e {
	TELNET_SE  =  240,
    TELNET_NOP = 241,
    TELNET_DM  =  242,
    TELNET_BRK  = 243,
    TELNET_IP  = 244,
    TELNET_AO  =  245,
    TELNET_AYT  = 246,
    TELNET_EC =   247,
    TELNET_EL  =  248,
    TELNET_GA  =  249,
    TELNET_SB =  250,
    TELNET_WILL= 251,
    TELNET_WONT = 252,
    TELNET_DO  =  253,
    TELNET_DONT = 254,
   	TELNET_IAC = 255
};

//
//	Suboptions of com port option
//
enum com_port_option_e {
	COM_PORT_SIGNATURE = 0,												// note typo in RFC 2217 re this.
	COM_PORT_SET_BAUDRATE  =    1,        
    COM_PORT_SET_DATASIZE   =    2,           
    COM_PORT_SET_PARITY   =  3,      
	COM_PORT_SET_STOPSIZE  =  4,     
	COM_PORT_SET_CONTROL  =  5,      
	COM_PORT_NOTIFY_LINESTATE = 6,      
	COM_PORT_NOTIFY_MODEMSTATE = 7,      
	COM_PORT_FLOWCONTROL_SUSPEND = 8,     
	COM_PORT_FLOWCONTROL_RESUME =  9,      
	COM_PORT_SET_LINESTATE_MASK = 10,      
	COM_PORT_SET_MODEMSTATE_MASK = 11,     
	COM_PORT_PURGE_DATA  =  12,
	COM_PORT_SIGNATURE_RECV = 100									// not sure about this, but Sealink sends it   
};
//
//	Table commands - must be greater than 255.
//
const uint32_t k_readescapedstring = 1001;
const uint32_t k_readanybyte = 1002;
const uint32_t k_endsuboption = 1003;
//
//	The option table
//
extern const uint32_t* telnetsuboptions[];				// external table
//
//	class SuboptionEngine  -- table driven suboption recognition engine
//
class SuboptionEngine {
private:
	std::basic_string<uint8_t> m_stringarg;					// option accumulated here
	bool m_stringargesc;											// last char was terminating char, may be in escape
	int m_optionnum;													// number of option being parsed (-1 if none)
	int m_optionpos;													// position within the option 
public:
	SuboptionEngine();
	virtual ~SuboptionEngine();
	void resetoption();												// reset to ground state
	bool acceptoption(uint8_t code);							// input a char in option mode. Returns false if no more option
protected:
	//	Derived class must redefine to receive option content
	virtual void deliveroption(const uint8_t data[], size_t datalen) = 0;
	virtual void protocolerror();									// report an error
private:
	int matchoption();													// match string against patterns >=0 pat, -1 is fail, -2 is ambiguous.
	void addtooption(uint8_t code);
};
//
//	Telnet protcol engine
//
class CProtocolRx: public SuboptionEngine
{
private:
	state_e m_state;																	// incoming Telnet option character stream state
	verb_e m_verb;
	int m_verbose;																		// >0 if verbose mode desired
	//	Information we get from options
	int m_baudrate;																		// the baud rate
public:
    CProtocolRx();
    virtual ~CProtocolRx();
    bool TelnetProtocol(SOCKET server, unsigned char code);
    void setverbose(int verbose) { m_verbose = verbose; }
    uint32_t getbaudrate() { return(m_baudrate); }
    void clearbaudrate() { m_baudrate = 0; }								// clear after a change
private:
    void yesreply(SOCKET server, verb_e verb, option_e option);
   	void noreply(SOCKET server, verb_e verb, option_e option);
    void askfor(SOCKET server, verb_e verb, option_e option);
protected:
	virtual void deliveroption(const uint8_t data[], size_t datalen);	// suboption, without the IAC SB
	virtual void deliversimpleoption(SOCKET socket, verb_e verb, option_e opt);		// simple option
private:
	void statechange(state_e state, verb_e verb);							// change state of protocol engine
	void statechange(state_e state);												// change state of protocol engine
	int sendoption(SOCKET server, const uint8_t msg[], uint32_t len, int flags);		// send an option
    bool TelnetProtocolImpl(SOCKET server,unsigned char code);	// long implementation for non-trivial cases
};
//
//	Inlines
//
inline void CProtocolRx::statechange(state_e state)						// quick, non-verbose state change
{	m_state = state; }
//
//	TelnetProtocol  --  called for every character
//
//	Returns true if caller should deliver byte to as data
//
inline bool CProtocolRx::TelnetProtocol(SOCKET server,unsigned char code)
{	if (m_state == state_data && code != TELNET_IAC) return(true);		// fast path for normal data
	return(TelnetProtocolImpl(server, code));								// general case
}

#endif //