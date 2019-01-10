/////////////////////////////////////////////////////////////////////////////
//
//    File: mvpserver.h
//
//    Usage:
//        In a watchdog startup file:
//				ID=MVPC mvpserver [-n <portName>] [-b <portBaud>] 
//                                [-t <portTimeout>] [-m]
//
//        See #define's below for default port settings.
//
//    Description:
//        An MVP Server serves as the gate keeper to multiple MVP units
//        on the same RS485 serial port.
//
//        Create MVP objects (see mvp.h, mvp.cc in libgccontrol) in your code to 
//        access an MVP Server easily.
//
//    See also:
//        mvp.h, mvp.cc, libgccontrol.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        October, 2003
//
/////////////////////////////////////////////////////////////////////////////

#ifndef MVPSERVER_H
#define MVPSERVER_H

#include "messaging.h"

// default serial port settings
#define MVPSERVER_PORTNAME      ("/dev/ser1")
#define MVPSERVER_PORTBAUD		(19200)
#define MVPSERVER_PORTCONTROL	("8N1")
#define MVPSERVER_PORTTIMEOUT	(15000)		// 15 msec (best for 1st inst)

//
//  MVPServer - MVP Server commands and errors
//
//  example usage:
//      MyClass::MyFunc(MVPServer::Err err)
//      {
//          if ( err == MVPServer::ERR_FAILED_CHECK ) {
//              ...
//          }
//      }
//
class MVPServer {
public:
	enum Cmd {
    	// enable/disable/reset
    	CMD_EN, CMD_DI, CMD_RN,
    	// target profile
    	CMD_LA, CMD_LR, CMD_SP, CMD_AC, CMD_DC,
  		// constant velocity
    	CMD_V,
    	// actual
    	CMD_POS, CMD_EP, CMD_AS, CMD_ERR,
    	// limit - position range
    	CMD_N, CMD_LL, CMD_LLM, CMD_SA,
    	// limit - current
    	CMD_SM, CMD_ANO,
    	// home
    	CMD_HO, CMD_HA, CMD_HP, CMD_HS, CMD_HF, CMD_AE,
    	// motion
    	CMD_M,
    	// motion abort
    	CMD_AB, CMD_AD, CMD_AA,
    	// errors
    	CMD_FD, CMD_FDT, CMD_FA,
    	// gains
    	CMD_POR, CMD_I, CMD_DER,
    	// sample period
    	CMD_SR,
    	// analog input/output
    	CMD_ANI, CMD_EAI, CMD_ANM,
    	// digital input/output
    	CMD_DACA, CMD_PI, CMD_PO, CMD_XST,
    	// macros
    	CMD_ME, CMD_MS,
    	// communication
    	CMD_SD, CMD_OK, CMD_CK, CMD_CX,
    	// status
    	CMD_ST, CMD_FV,
    	// EEprom
    	CMD_EEPSAV, CMD_EEBOOT
	};
	enum Err {
		ERR_OK,							// INST
		ERR_MSGSEND_FAILED,				// INST
		ERR_COMMAND_STRING_TOO_LONG,	// INST
		ERR_PARAMETER_OUT_OF_RANGE,		// INST
		ERR_WRITE_BAD,					// INST
		ERR_READ_BAD					// INST
	};
	static char *ErrMsg(Err err)
	{
		switch ( err ) {
			case ERR_MSGSEND_FAILED:
				return "MsgSend failed";
				break;
			case ERR_COMMAND_STRING_TOO_LONG:
				return "command string too long";
				break;
			case ERR_PARAMETER_OUT_OF_RANGE:
				return "parameter out of range";
				break;
			case ERR_WRITE_BAD:
				return "bad write";
				break;
			case ERR_READ_BAD:
				return "bad read";
				break;
			default:
				return "unknown error";
				break;
		}
	}
};

//
//  MVPServerMsgINST - INST: MVP Server instruction
//
struct MVPServerMsgINST: public MsgBase {
	static const uint32_t k_msgtype = char4('I','N','S','T');
	MVPServer::Err err;		// returned, ERR_0K=no error, otherwise error
	int node;				// MVP node number
	MVPServer::Cmd cmd;		// MVP command
	int param;				// MVP parameter
	bool paramActive;		// whether parameter should be used
	int value;				// returned, value from MVP node
};

//
//  MVPServerMsgVERB - VERB: MVP Server verbose mode
//
struct MVPServerMsgVERB: public MsgBase {
	static const uint32_t k_msgtype = char4('V','E','R','B');
	MVPServer::Err err;		// returned, ERR_0K=no error, otherwise error
	bool get;				// true=get, false=set
	bool verbose;			// true=on, false=off
};

//
//  MVPServerMsg - all MVP Server messages as a union
//
//  Used as argument to MsgReceive. Size of union is size of largest 
//  acceptable message
//
union MVPServerMsg {
	MVPServerMsgINST m_inst;
	MVPServerMsgVERB m_verb;
};

#endif // MVPSERVER_H
