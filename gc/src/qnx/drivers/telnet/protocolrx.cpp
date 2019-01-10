//
//	protocolrx.cpp  --  receive side of Telnet protocol for "com ports".
//
//	Understands the Telnet "COM PORT" option (RFC 2217) but no others.
//
//	John Nagle
//	Team Overbot
//	December, 2003
//
#include <stdio.h>
#include <assert.h>
#include <sys/socket.h>
#include "protocolrx.h"

//
//	Debug utilities
//
//
//	verbname -- text name for "verb" state enumeration
//
static const char* verbname(verb_e verb)
{	switch (verb) {
	case verb_none: return("NONE");
	case verb_sb: return("SB");
	case verb_will: return("WILL");
	case verb_wont: return("WONT");
	case verb_do: return("DO");
	case verb_dont: return("DONT");
	default: return("verb ???"); 
	}
};
//
//	statename -- text name for "state" state enumeration
//
static const char* statename(state_e state)
{	switch (state) {
	case state_data: return("DATA");
	case state_code: return("CODE");
	case state_option: return("OPTION");
	default: return("state ???");
	}
}
//
//	The suboption engine, for table-drive parsing of suboptions
//
//
//	class SuboptionEngine  -- table driven suboption recognition engine
//
//
//	Constructor
//
SuboptionEngine::SuboptionEngine()
:	m_stringargesc(false), m_optionnum(-1), m_optionpos(0)
{}
//	
//	Destructor
SuboptionEngine::~SuboptionEngine()
{}
//
//	resetoption -- reset option engine to ground state
//
void SuboptionEngine::resetoption()
{	m_stringarg.erase();
	m_stringargesc = false;
	m_optionnum = -1;
	m_optionpos = 0;
}
//
//	acceptoption -- input a char in option mode. Returns false if no more option
//
//	This is the actual parsing engine.
//	Returns false if no more option, or if error
//
bool SuboptionEngine::acceptoption(uint8_t code)
{	////printf("acceptoption: code %d, pattern %d, position %d\n", code, m_optionnum, m_optionpos);	// ***TEMP***
	if (m_optionnum < 0)													// if no option yet
	{	
		addtooption(code);													// always accumulate new byte
		m_optionnum = matchoption();									// try to match
		switch (m_optionnum) {											// fan out on option number state
		case -2:	return(true);												// still ambiguous, get more bytes
		case -1:																	// failed
			protocolerror();														// report error
			resetoption();														// reset engine
			return(false);														// done with this bad option
		default:																		// found the beginning of a valid option
			m_optionpos = m_stringarg.length();					// and are this many bytes into it
			return(true);															// get more
		}
		return(true);
	}
	//	We are locked into a specific option. Continue match
	const uint32_t* pat = telnetsuboptions[m_optionnum];	// option being tried
	uint32_t patbyte = pat[m_optionpos];							// get current pattern byte
	switch (patbyte) {														// fan out on pattern byte
		case k_endsuboption:												// end of option
			protocolerror();														// should never be here.
			resetoption();														// reset engine
			return(false);														// no more of this option	
			
		case k_readanybyte:												// maches any byte
			addtooption(code);												// accumulate new byte
			m_optionpos++;													// advance pattern
			break;																	
			
		case k_readescapedstring:										// escaped text - anything but pattern-end character																						// two pattern ends escape a pattern end character
		{	const uint32_t patternend = pat[m_optionpos+1];	// character that stops pattern
			if (code == patternend)										// if escape character 
			{	if (m_stringargesc)											// if in escape, this is one escaped character (IAC IAC -> IAC)
				{	m_stringargesc = false; 								// ignore, we got it already
				} else {															// if escape char, but not in escape (xxx IAC)
					addtooption(code);										// accumulate new byte, part of same pattern match
					m_stringargesc = true;									// now in an escape, must look at next char
				}
			} else {																// if not escape character 
				if (m_stringargesc)											// if in escape mode (IAC xxx)
				{	m_stringargesc = false;								// no longer in escape mode
					m_optionpos++;											// no longer in text escape
					m_optionpos++;											// use up patternend char
					return(acceptoption(code));							// recurse because we have looked ahead one char
				} else {															// if not escape, and not in escape mode (xxx xxx)
					//	normal case, do nothing
					addtooption(code);										// accumulate new byte
				}
			}
			break;																
		}
			
		default:																		// ordinary character
			addtooption(code);												// accumulate new byte
			if (code != patbyte)												// if pattern not matched
			{	protocolerror();													// report error
				resetoption();													// reset
				return(false);													// no more
			}
			m_optionpos++;													// advance pattern
			break;
	}

	//	Are we done?
	assert(m_optionnum >= 0);											// must be matching a pettern if here
	if (telnetsuboptions[m_optionnum][m_optionpos] == k_endsuboption)	// if entire pattern has been used up
	{	deliveroption(m_stringarg.data(), m_stringarg.length());	// deliver the option
		resetoption();															// reset engine
		return(false);															// no more of this option	
	}
	return(true);																	// get more
}
//
//	matchoption  -- match accumulated option against pattern
//
//	Returns:	>0	valid pattern ID
//					-1		no match
//					-2 	ambiguous, keep trying
//
int SuboptionEngine::matchoption()
{	int matchpat = -1;																	// matched pattern, if any
	for (int patnum = 0; telnetsuboptions[patnum]; patnum++)	// for all patterns
	{	const uint32_t* pat = telnetsuboptions[patnum];				// this pattern
		assert(pat);
		bool match = true;															// if this pattern matches
		for (unsigned int pos = 0; pos < m_stringarg.length(); pos++)		// for length of string
		{	uint32_t patbyte = m_stringarg[pos];							// get pattern byte
			switch (patbyte) {
			case k_readanybyte: continue;										// wildcard, matches anything
			default:																			// must match exactly
				if (patbyte != pat[pos])											// if no match
				{	match = false; break;	}										// fails
			}
		}
		if (match)																			// if we have a match with this pattern
		{	if (matchpat >= 0)														// if it's not the first match
			{	return(-2);	}																// ambiguous, need more bytes to decide
			matchpat = patnum;														// first, save
		} 
	}
	return(matchpat);																	// return matching pattern ID, or -1 if no match
}
//
//	addtooption -- append to option
//
void SuboptionEngine::addtooption(uint8_t code)
{	m_stringarg += code;	}
//
//	protocolerror -- report a protocol error
//
void SuboptionEngine::protocolerror()
{	printf("\n[PROTOCOL ERROR in suboption: ");
	for (unsigned int i=0; i<m_stringarg.length(); i++)
	{	printf("%d ",m_stringarg[i]); }
	printf("]\n");
}
//
//	deliveroption -- the parent has delivered suboption data
//
//	We understand a few suboptions.
//
//
//	Format here is everything after the IAC SB up to the end.
//
//	Escapes within variable-length text have been removed, but the length here gives us the needed framing info.
//
void CProtocolRx::deliveroption(const uint8_t data[], size_t datalen)
{
	//	Handle option
	if (datalen < 2) return;														// not meaningful
	const uint8_t option = data[0];											// get option byte
	switch (option) {																// handle option
	case TOPT_COMPORTCONTROL:
		switch (data[1]) {															// fan out on suboption
		case COM_PORT_SET_CONTROL: 
			if (m_verbose)
			{	printf("[COM_PORT_SET_CONTROL: %d]\n", data[2]); }	// display control byte 
			break;
			
		case COM_PORT_NOTIFY_MODEMSTATE:
			if (m_verbose)
			{	printf("[COM_PORT_NOTIFY_MODEMSTATE: 0x%02x]\n", data[2]); }	// display control byte 
			break;
			
		case COM_PORT_SIGNATURE_RECV:								// not sure about this, but Sealink sends it
			if (m_verbose)															// if verbose
			{	printf("[COM_PORT_SIGNATURE_RECV: ");				// print device signature
				for (unsigned int i=2; i<datalen-2; i++) putchar(data[i]);	
				printf("]\n");
			}
			break;
			
		case COM_PORT_SET_BAUDRATE:
		{	const uint32_t b1 = data[2];
			const uint32_t b2 = data[3];
			const uint32_t b3 = data[4];
			const uint32_t b4 = data[5];
			m_baudrate = b4 | (b3 << 8) | (b2 << 16) | (b1 << 24); 	// baud rate
		
			if (m_verbose)															// if verbose
			{	printf("\[COM_PORT_SET_BAUDRATE: %d]\n",m_baudrate);	// display baudrate
			}
			break;
		}
			
		default: 
			if (m_verbose)
			{	printf("[Unknown TOPT_COMPORTCONTROL suboption of %d bytes: ",datalen);	
				for (unsigned int i=0; i<datalen; i++)
				{	printf(" %d",data[i]); }
					printf("]\n");
				}
				break;
		}
		return;																			// handled this option
		
	default:
		if (m_verbose)
		{	printf("\n[Unknown suboption of %d bytes: ",datalen);	
			for (unsigned int i=0; i<datalen; i++)
			{	printf(" %d",data[i]); }
				printf("]\n");
			}

		break;
	}
}
//
//	deliversimpleoption -- one of the one-byte options
//
//	We accept com port control, and reject everything else.
//
//	The Sealevel unit actually sends suboptions whether we do this or not, which is wrong.
//
void CProtocolRx::deliversimpleoption(SOCKET socket, verb_e verb, option_e opt)
{	if (m_verbose)
	{	printf("[Option %s %d]\n", verbname(verb), opt); }
	switch (verb) {
		case verb_do:
		case verb_dont:
			switch (opt) {															// do what?
				case TOPT_COMPORTCONTROL:								// Do com port control
					yesreply(socket, verb, opt);								// will cooperate
					break;
				default:
					noreply(socket, verb, opt);								// will not cooperate
				break;
			}
		default:
			break;
		}
}
//
//	statechange  --  protocol engine state change, with debug output
//
void CProtocolRx::statechange(state_e state, verb_e verb)
{
	if (m_verbose>1 && (m_state != state || m_verb != verb))		// if verbose mode and state change
	{	printf("State change: %s to %s, verb %s to %s\n",
			statename(m_state), statename(state),
			verbname(m_verb), verbname(verb));
	}
	m_state = state;
	m_verb = verb;
}

CProtocolRx::CProtocolRx()
: m_state(state_data), m_verb(verb_none), m_baudrate(0)						// initialize state machine
{
}

CProtocolRx::~CProtocolRx()
{
}

//
//	sendoption -- send an option
//
//	Provides debug output
//
int CProtocolRx::sendoption(SOCKET server, const uint8_t msg[], uint32_t len, int flags)
{
	if (m_verbose)															// if verbose mode
	{	printf(" [Sending option: ");
		for (unsigned int i=0; i<len; i++)
		{	printf("0x%02x ",msg[i]);	}
		printf("]\n");
	}
	return(send(server,msg,len,flags));
}
//
//	yesreply -- reply to option negotiation
//
void CProtocolRx::yesreply(SOCKET server, verb_e verb, option_e option)
{
    unsigned char buf[3];
    buf[0] = TELNET_IAC;
    buf[1] = (verb==verb_do)?TELNET_WILL:(verb==verb_dont)?TELNET_WONT:(verb==verb_will)?TELNET_DO:TELNET_DONT;
    buf[2] = (unsigned char)option;
    sendoption(server,buf,3,0);
}

void CProtocolRx::noreply(SOCKET server, verb_e verb, option_e option)
{
    unsigned char buf[3];
    buf[0] = TELNET_IAC;
    buf[1] = (verb==verb_do)?TELNET_WONT:(verb==verb_dont)?TELNET_WILL:(verb==verb_will)?TELNET_DONT:TELNET_DO;
    buf[2] = (unsigned char)option;
    sendoption(server,buf,3,0);
}

void CProtocolRx::askfor(SOCKET server, verb_e verb, option_e option)
{
    unsigned char buf[3];
    buf[0] = TELNET_IAC;
    buf[1] = (unsigned char)verb;
    buf[2] = (unsigned char)option;
    sendoption(server,buf,3,0);
}


//
//	TelnetProtocolImpl -- implementation of the Telnet protocol engine
//
//	See RFC 854.
//
//	This is a very limited implementation of the protocol.
//
//	We can reject any option we don't like, and only
//	enter suboption negotiations for the few that we know about.
//
//
bool CProtocolRx::TelnetProtocolImpl(SOCKET server,unsigned char code)
{
    //	Decide what to do (state based)
    switch(m_state)
    {
    case state_data:																// we are in DATA state
        switch(code)
        {
        case TELNET_IAC:															// if IAC
        	statechange(state_code);											// go to CODE state
            break;
        default:
           return(true);																// normal data
        }
        break;
        
    case state_code:																// in CODE state
        switch(code)
        {
        case TELNET_IAC:															// IAC IAC - escapes an IAC in ordinary data
        	statechange(state_data);											// go back to DATA state
       		return(true);
            
        // Codes which we are ignoring
        case TELNET_SE:
        	statechange(state_data);											// go back to DATA state
            break;
        case TELNET_NOP:
        	statechange(state_data);											// go back to DATA state
            break;
        case TELNET_DM:
        	statechange(state_data);											// go back to DATA state
            break;
        case TELNET_BRK:
        	statechange(state_data);											// go back to DATA state
            break;
        case TELNET_IP:
        	statechange(state_data);											// go back to DATA state
            break;
        case TELNET_AO:
        	statechange(state_data);											// go back to DATA state
            break;
        case TELNET_AYT:
        	statechange(state_data);											// go back to DATA state
            break;
        case TELNET_EC:
        	statechange(state_data);											// go back to DATA state
            break;
        case TELNET_EL:
        	statechange(state_data);											// go back to DATA state
            break;
        case TELNET_GA:
        	statechange(state_data);											// go back to DATA state
            break;
            
            //	Codes we actually do something about.
            //	Transitions to option state, and set a verb. More option data will follow.
        case TELNET_SB:
        	statechange(state_option,verb_sb);
            break;
        case TELNET_WILL:
        	statechange(state_option, verb_will);
            break;
        case TELNET_WONT:
        	statechange(state_option, verb_wont);
            break;
        case TELNET_DO:
        	statechange(state_option, verb_do);
            break;
        case TELNET_DONT:
        	statechange(state_option, verb_dont);
            break; 
        default:																			// unknown control, ignore
        	statechange(state_data);											// go back to DATA state
			break;
        }
        break;
        
    case state_option:															// state: IAC verb ...
    	switch (m_verb) {
 		   	case verb_sb:															// suboption state
 	    	{	bool moreoption = acceptoption(code);				// send byte to suboption engine
   		 		if (!moreoption)													// if no more suboption
   		 		{	statechange(state_data, verb_none);	}			// back to data state
    			break;
    		}
    		case verb_will:
    		case verb_wont:
    		case verb_do:
    		case verb_dont:
    			{	option_e opt((option_e)code);							// code is an option - coerce
    				deliversimpleoption(server, m_verb, opt);			// deliver simple one-byte option
   					statechange(state_data, verb_none);				// return to data state
   					break;
   				}
   				
   		default:
    			statechange(state_data, verb_none);					// error, return to start state
    	}
        break;
    }
    return(false);																// not data, do not keep
}


