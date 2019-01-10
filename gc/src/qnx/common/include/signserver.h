/////////////////////////////////////////////////////////////////////////////
//
//    File: signserver.h
//
//    Usage:
//        In the watchdog startup file (startfile.txt):
//				ID=SIGN signserver /dev/ser1 -v 
///
//    Description:
//       The Sign Server displays messages on a large display sign.
//       
//
//    Written By:
//        John Nagle
//        Team Overbot
//        November, 2003
//
/////////////////////////////////////////////////////////////////////////////

#ifndef SIGNSERVER_H
#define SIGNSERVER_H
#include <strings.h>
#include "messaging.h"

//
//  SignServer - Sign messages
//
//
//  SignServerMsgSNTX - SNTX: Sign Text
//
//  Sends text to sign.
//	Reply is an error code only.
//	Does not block.
//
struct SignServerMsgSNTX: public MsgBase {
	static const uint32_t k_msgtype = char4('S','N','T','X');
	static const int k_maxtextsize = 128;		// allow for long message
	enum SignPriority { Routine, Priority, Urgent };
	enum SignColor { DefaultColor=0, Red, Green, Yellow, Amber };
	SignPriority	m_priority;
	SignColor	m_color;
	char m_text[k_maxtextsize];					// the text
};

//
//  SignServerMsg - all MVP Server messages as a union
//
//  Used as argument to MsgReceive. Size of union is size of largest 
//  acceptable message
//
union SignServerMsg {
	SignServerMsgSNTX m_sntx;
};
//
//	Quick messaging functions
//
//
//	SignDisplay  -- general form -- defaults to routine priority
//
inline int SignDisplay(const char* msg, SignServerMsgSNTX::SignPriority pri = SignServerMsgSNTX::Routine, 
	SignServerMsgSNTX::SignColor color = SignServerMsgSNTX::DefaultColor)
{	
	const char* SignServerName = "SIGN";							// name of sign server, for quick use
	static MsgClientPort clientport(SignServerName,0.1);		// define client port, 0.1 sec timeout
	SignServerMsgSNTX outmsg;
	outmsg.m_msgtype = SignServerMsgSNTX::k_msgtype;	// build message to send to server
	outmsg.m_priority = pri;
	outmsg.m_color = color;
	strncpy(outmsg.m_text,msg,sizeof(outmsg.m_text));			// copy text of message
	return(clientport.MsgSend(outmsg));								// send msg, return stat only
}
//
//	Short forms for higher priorities
//	
inline int SignDisplayPriority(const char* msg)
{	return(SignDisplay(msg,SignServerMsgSNTX::Priority));	}
inline int SignDisplayUrgent(const char* msg)
{	return(SignDisplay(msg,SignServerMsgSNTX::Urgent));	}
#endif // SIGNSERVER_H
