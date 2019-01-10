//
//	signserver.cpp  -- server for BetaBrite sign
//
//	Our simplest server
//
//	John Nagle
//	Team Overbot
//	November, 2003
//
//
//	roadserver.cpp  --  The road follower, as a QNX server
//
//	John Nagle
//	Team Overbot
//	November, 2003
//
#include <assert.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "mutexlock.h"
#include "betabrite.h"
#include "signserver.h"
#include "logprint.h"
//
//	Constants
//	
const int k_holdtime = 60;										// message hold time, seconds
//
static bool verbose = false;									// true if verbose mode
//
//
//	class SignServer  --  read from a FireWire camera, do road following
//
class SignServer: private BetaBrite {
private:
	int m_holdtime;													// hold current message this long unless preeempted
	SignServerMsgSNTX::SignPriority m_holdpri;	// priority of message currently being displayed	
	pthread_t m_displaythread;								// the display thread	
private:
	const char* calccolor(const SignServerMsgSNTX& msg);	// msg color
	// Display thread
	void* displaythread();										// display thread
	static void* startdisplaythread(void* threaddata) { return(reinterpret_cast<SignServer*>(threaddata)->displaythread()); }
	//	Shared data
	ost::BoundedBuffer<SignServerMsgSNTX,4> m_queue;	// next message to display, if any
public:
	SignServer();												// constructor
	virtual ~SignServer();									// destructor
	int init(const char* signport);						// initialization
	void serverTX(int rcvid, SignServerMsgSNTX& msg);
	void idle();
	void clear();													// clear the display
};
//
//	Constructor
//
SignServer::SignServer()
: m_holdtime(0), m_holdpri(SignServerMsgSNTX::Routine)
{
}
//	
//	Destructor
//
SignServer::~SignServer()
{
	clear();																					// clear the display
}
//
//	init -- startup
//
int SignServer::init(const char* signport)
{
	int stat = open(signport);											// try to open
	if (stat) return(stat);													// if fail
	stat = pthread_create(&m_displaythread,0,startdisplaythread,this);	// start the display thread
	if (stat) return(stat);
	return(EOK);																// success
}
//
//	idle -- idle for 1 second, update display
//
void SignServer::idle()
{
	if (m_holdtime <= 0) return;													// nothing to do
	m_holdtime--;																		// count down
	if (m_holdtime > 0) return;													// not time yet
	clear();																					// clear the display
	m_holdpri = SignServerMsgSNTX::Routine;							// mark as routine priority
}
//
//	calccolor  -- get appropriate color for message
//
const char*  SignServer::calccolor(const SignServerMsgSNTX& msg)
{
	//	Decide on color, based on specified color and priority
	switch(msg.m_color) {
		case SignServerMsgSNTX::Red: return(BETABRITE_COLOR_RED);
		case SignServerMsgSNTX::Green: return(BETABRITE_COLOR_GREEN);
		case SignServerMsgSNTX::Amber: return(BETABRITE_COLOR_AMBER);
		case SignServerMsgSNTX::Yellow: return(BETABRITE_COLOR_YELLOW);
		default: break;													// other
	}
	//	No specified color, use priority
	switch(msg.m_priority) {
		case SignServerMsgSNTX::Urgent: return(BETABRITE_COLOR_RED);
		case SignServerMsgSNTX::Routine: return(BETABRITE_COLOR_GREEN);
		case SignServerMsgSNTX::Priority: return(BETABRITE_COLOR_AMBER);
		default: break;													// other
	}
	return(BETABRITE_COLOR_GREEN);						// if nothing specified, use green
}
//	
//	serverTX  --  accept text line from client
//
void SignServer::serverTX(int rcvid, SignServerMsgSNTX& msg)
{	msg.m_text[sizeof(msg.m_text)-1] = '\0';						// avoid possible string overrun of buffer if junk data
	if (verbose)
	{	printf("%s\n",msg.m_text); fflush(stdout);		}			// log message
	if (msg.m_priority < m_holdpri)										// priority lower than what is being displayed.
	{	MsgError(rcvid,EBUSY);	return; }								// sign is busy
	//	Can display msg. Enqueue for display thread, which does the serial port work
	m_holdpri = msg.m_priority;											// save priority
	if (strlen(msg.m_text) > 1)											// if non-blank message
	{	m_holdtime = k_holdtime;		}									// hold for this long
	else																				// sending a blank message clears any hold
	{	m_holdtime = 0; }														// otherwise no hold
	bool put = m_queue.tryput(msg);									// put on the display queue
	if (!put)																			// if unable to put
	{	MsgError(rcvid,EBUSY); }											// can't do it, queue full
	MsgError(rcvid,EOK);														// only reply is a status
}
//
//	The display thread
//
//	Reads from a queue of messages and displays them.
//	This keeps the server from blocking its clients while actually sending the data out the serial port
//
void* SignServer::displaythread()
{
	clear();																			// clear the display
	SignServerMsgSNTX prevmsg;										// previous message
	prevmsg.m_text[0] = '\0';												// no text yet
	for (;;)																			// forever
	{	SignServerMsgSNTX msg;											// working message
		m_queue.get(msg);													// wait, get a message
		if (strncmp(prevmsg.m_text, msg.m_text, sizeof(msg.m_text)) == 0) continue;	// ignore if unchanged
		setcolor(calccolor(msg));											// set sign color
		int stat = display(msg.m_text);									// display the text
		if (stat)
		{	perror("Error displaying message");	}					// log, but ignore
		prevmsg = msg;														// save msg
	}
}
//
//	clear -- clear the display
//
//	Leaves a green dash
//
void SignServer::clear()
{
	setcolor(BETABRITE_COLOR_GREEN);							// initialize sign to one green dash
	display("-");
}
//
//	runserver -- run as a server
//
//	This server accepts request messages, does some computation, and returns the result.
//
void runserver(const char* signport)															
{	SignServer sign;														// the sign
	//	Initialize sign
	logprintf("Opening sign serial port: \"%s\"\n",signport);						// Log sign opened
	fflush(stdout);
	int stat = sign.init(signport);										// try to open sign
	if (stat != EOK)															// if unsuccessful open
	{																				
		perror("Unable to open sign port");
		exit(1);																	// fails
	}

	//	The server normally has the name given the program by the watchdog file. This is in the env. variable "ID"
	MsgServerPort serverport(1.0);								// define message port, 1 second timeout
	if (verbose) serverport.setverbose();						// more talkative
	stat = serverport.ChannelCreate();							// create a channel, tell watchdog about it
	if (stat) 
	{	perror("ChannelCreate failed in server"); 
		serverport.Dump();
		exit(1);																	// fails
	}
	fflush(stdout);															// force out any startup message
	for (;;)																		// forever
	{	SignServerMsg msgin;											// area for incoming msg
		_msg_info msginfo;												// aux info
		int rcvid = serverport.MsgReceive(msgin,&msginfo);	// get msg
		if (rcvid < 0)															// if error
		{	if (errno == ETIMEDOUT)									// if timeout
			{	sign.idle();														// do idle actions
				continue;
			}
			fflush(stdout); perror("MsgReceive failed in server");				// fails
			sleep(1);															// avoid tight loop if repeated trouble
			continue;															// fails
		}
		if (rcvid == 0)														// pulse
		{	logprintf("Server received a pulse.\n");					// pulses don't require a reply
			continue;
		}
		//	We have received a message
		switch (msgin.m_sntx.m_msgtype) {						// fan out on type
		case SignServerMsgSNTX::k_msgtype:					// request with text
		{	if (msginfo.msglen != sizeof(msgin.m_sntx)) { MsgError(rcvid,EBADRPC); break; }	// msg size check
			sign.serverTX(rcvid,msgin.m_sntx);					// handle msg
			break;
		}
			
		default:																	// unknown, fails
			MsgError(rcvid,EBADRPC);									// reply with result code only
			break;
		}
		////serverport.watchdogreset();								// tell the watchdog we are still alive
	}
}
//
//	runclient -- run as a client
//
//	usage  -- print usage and exit
//
static void usage()
{	printf("Usage: signserver [options] signport\n");
	printf("  Options:  -v verbose\n");
	exit(1);																		// fails
}
//
//	Main program
//
//	Usage: signserver [options] signname
//
int main(int argc, const char* argv[])
{	const char* signport = 0;											// no sign name yet
	//	Parse input arguments
	for (int i=1; i<argc; i++)											// for all args
	{	const char* arg= argv[i];										// this arg
		if (arg[0] == '-')													// if flag argument
		{	switch(arg[1])	{												// interpret flags
			case 'v': verbose = true;	 break;						// set verbose mode
			default: usage();												// bad call, fails
			}
			continue;															// next arg
		}
		//	Not flag, must be file arg
		if (signport) usage();
		signport = arg;
	}
	if (!signport) usage();												// must have camera name
	//	go
	runserver(signport);													// run  sign server
	return(0);																	// success
}
