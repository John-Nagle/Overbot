////////////////////////////////////////////////////////////////////////////
//
//    File: mvpserver_.h
//
//    Usage:
//        See mvpserver.h.
//
//    Description:
//        See mvpserver.h.
//
//        This is the internal header file (mvpserver.h is the external one).
//
//        Originally an MVP Server would be part of another program/process.
//        i.e., a compiled version of this file would be linked to other
//        objects.  It is now a stand-alone process that accepts messages.
//
//        The primary purpose of an MVP Server is to send instructions out
//        to a set of MVP units over an RS485 serial port.  The MVP Server
//        checks the integrity of the instruction based on its database
//        of commands.  It then sends the instruction out the RS485 port, and
//        returns values if neccessary.
//
//    See also:
//        mvpserver.h, main.cc, cmddb.cc, checksum.cc, 
//        mvp.h, mvp.cc, libgccontrol
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        August, 2003
//
/////////////////////////////////////////////////////////////////////////////

#ifndef MVPSERVER__H
#define MVPSERVER__H

#include <vector>
#include <pthread.h>
#include "serial.h"
#include "serialmenu.h"

#include "mvpserver.h"

using namespace std;

#define MVPSERVER_PORTNAME_LEN	(82)// max port name length

#define MVPSERVER_NODE_MAX	(63)	// highest node number

#define MVPSERVER_CMD_LEN    (8)
#define MVPSERVER_INSTR_LEN (82)
#define MVPSERVER_REPLY_LEN (82)

#define MVPSERVER_TRIES_NUM	(4)		// number of tries to write/read instruction

#define MVPSERVER_ID		(82)	// watchdog ID of server; used with menu

// command properties record
typedef struct {
    char cmdStr[MVPSERVER_CMD_LEN]; // command text string (e.g., "LA")
    bool paramOK;                   // command can take parameters
    bool paramRange;                // command parameters must be within range
    int paramMin;                   // minimum parameter value
    int paramMax;                   // maximum parameter value
    bool valueRtn;                  // command w/o parameter returns a value
    bool valueNeg;					// negative values allowed
    int valueNibbles;				// # nibbles in return val (for neg transl)
    bool valueCksum;				// whether return value includes checksum
} cmdRec;

// internal MVPServer class
class MVPServer_ {
public:
	MVPServer_();				// constructor
	~MVPServer_();				// destructor
	void MenuThread();
	void SerialPortInit(char *portName, int portBaud, int portTimeout);
	void MessageServer();
	bool CmdDBStr2Enum(char *str, MVPServer::Cmd *cmd);
private:
	// server message handling
	MsgServerPort *serverport;	// message port
	int rcvid;
	MVPServerMsg msg;			// area for incoming and outgoing message
	
	void *menuThread();
    // need static function loopStart() for pthread_create
    // loopStart calls the instance-specific function loop()
    static void* menuThreadStart(void* arg)
	    { return(reinterpret_cast<MVPServer_*>(arg)->menuThread()); }
	
	void messageHandle();

	// server state
	bool verbose;
	
	// server resources
	Serial *s;				// serial port used to communicate with MVP unit(s)
	
	// database of command properties and related routines
	vector<cmdRec> cmdDB;
	vector<cmdRec>::iterator cmdDBi;
	
    void cmdDBLoad(MVPServer::Cmd cmd, char *cmdStr, bool paramOK, 
                  bool paramRange, int paramMin, int paramMax,
                  bool valueRtn, bool valueNeg, int valueNibbles,
                  bool valueCksum);
    void cmdDBInit();
                  
    // database of whether node has been initialized
    vector<bool> *nodeInit;

	// checksum-related routines
	static void checksumCalc(char *instr, char *nibbleL, char *nibbleR);	
	static void checksumAppend(char *instr);
	static bool checksumOK(char *reply);
	
	// instruction-related routines
	MVPServer::Err instructionInit();
	MVPServer::Err instructionSend(MVPServerMsg *msgSend);
	void instructionErrMsgTryAgain(MVPServerMsg *msgSend);
	void instructionErrMsgTryLast(MVPServerMsg *msgSend);
};

#endif // MVPSERVER__H