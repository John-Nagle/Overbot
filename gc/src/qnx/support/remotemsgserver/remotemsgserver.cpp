//
//	General server testing server
//
//	Allows remote nodes to send messages to programs running under the watchdog, for test only.
//
//	TEST USE ONLY
//
//	John Nagle
//	Team Overbot
//	November, 2004
//
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <errno.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <logprint.h>
#include "messaging.h"
#include "remotemessaging.h"
#include "timeutil.h"
//
//	Configuration constants
//
const size_t k_max_messsage_length = 640*480*3+100;	// longest allowed message
const size_t k_max_reply_length =  k_max_messsage_length;	// longest allowed reply
const double k_timeout = 1.0;								// once a second, cycle
//
//
//	Incoming message, which may be a pulse we have to interpret.
//	
union  MsgData {
    _pulse m_pulse;												// if it's a pulse
   	char m_servermsg[k_max_messsage_length];	// if it's a message
};
static char reply[k_max_reply_length];									// reply buffer
static MsgData msg;																// dummy message


//	
//	handledatamsg  -- handle a test message
//
static void handledatamsg(MsgClientPort& dest, int rcvid, const char* msg, size_t len)
{	
	int cnt = dest.MsgSend(msg, len, reply, sizeof(reply));// send to local server
	if (cnt < 0)																	// if local failure
	{	perror("Remote msg pass failed"); fflush(stderr);		// report
		MsgError(rcvid,errno);												// return error code to remote
		return;
	}
	if (cnt > int(k_max_reply_length))									// if oversize message
	{	printf("Remote msg of %d bytes is too big.  Limit is %d.\n", cnt, k_max_reply_length); fflush(stdout); 
		MsgError(rcvid, ENOMEM);										// return too big error
		return;
	}
	//	Send reply. 
	//	We can't actually tell how long the reply is, but we assume it is the value of "cnt", which is conventional
	int stat = MsgReply(rcvid, cnt, reply, cnt);									// success, send reply
	if (stat < 0)																	// if local failure
	{	perror("Remote msg reply failed"); fflush(stderr);	}	// report
}
//
//	The test server -- runs as a separate thread.
//
//	Only run for test operations.
//
//	Checks messages and forwards them to the real server.
//
static void testserver(const char* myname, const char* destname)
{	MsgClientPort dest(destname, k_timeout);							// create a connection to the destination
	name_attach_t* attach = name_attach(0,myname,0);		// create a path
	if (!attach)
	{	printf("Failed to name_attach to \"%s\"\n", myname);		// two copies are probably running
		exit(1);
	}
	logprintf("Remote msg server: connecting \"%s\" to \"%s\"\n",myname,destname);		// server is ready
	//	Receive loop
	uint64_t lastwatchdog = gettimenowns();							// last write to watchdog
    while (1) {
		uint64_t now = gettimenowns();									// keep alive logic
		uint64_t elapsed = now - lastwatchdog;						// time since last msg
		{	if (elapsed > uint64_t(k_timeout*1000000000))
			{	logprintf("\n");														// keep output alive
				lastwatchdog = now;											// note that we did it
			}
		}	
    	_msg_info info;													// additional message info
        int rcvid = MsgReceive(attach->chid, &msg, sizeof(msg), &info);
        if (rcvid < 0) 
        {	//	Trouble
        	if (errno == ETIMEDOUT)								// normal timeout
        	{	logprintf("\n");
        		lastwatchdog = now;
        		continue;
        	}
        	perror("MsgReceive failed");							// should not happen
            break;
        }

        if (rcvid == 0) {/* Pulse received */
            switch (msg.m_pulse.code) {
            case _PULSE_CODE_DISCONNECT:
                /*
                 * A client disconnected all its connections (called
                 * name_close() for each name_open() of our name) or
                 * terminated
                 */
                ConnectDetach(msg.m_pulse.scoid);
                break;
                
            case _PULSE_CODE_UNBLOCK:
                /*
                 * REPLY blocked client wants to unblock (was hit by
                 * a signal or timed out).  It's up to you if you
                 * reply now or later.
                 */
                break;
                
            default:
                /*
                 * A pulse sent by one of your processes or a
                 * _PULSE_CODE_COIDDEATH or _PULSE_CODE_THREADDEATH 
                 * from the kernel?
                 */
                 break;
            }
            continue;
        }
#ifdef OBSOLETE
        /* A QNX IO message received, reject */
        if (msg.hdr.type >= _IO_BASE && msg.hdr.type <= _IO_MAX) {
        	printf("QNX I/O msg type %d received.\n",msg.hdr.type);	
            MsgError(rcvid, ENOSYS);
            continue;
        }
#endif // OBSOLETE
        /* A message (presumable ours) received, handle */
        handledatamsg(dest, rcvid, msg.m_servermsg, info.msglen);								// do it
    }

    /* Remove the name from the space */
    name_detach(attach, 0);
}
//	
//	handlereversedatamsg  -- handle a test message
//
static void handlereversedatamsg(RemoteMsgClientPort& dest, int rcvid, const char* msg, size_t len)
{	
	int cnt = dest.MsgSend(msg, len, reply, sizeof(reply));// send to local server
	if (cnt < 0)																	// if local failure
	{	perror("Remote msg pass failed"); fflush(stderr);		// report
		MsgError(rcvid,errno);												// return error code to remote
		return;
	}
	if (cnt > int(k_max_reply_length))									// if oversize message
	{	printf("Remote msg of %d bytes is too big.  Limit is %d.\n", cnt, k_max_reply_length); fflush(stdout); 
		MsgError(rcvid, ENOMEM);										// return too big error
		return;
	}
	//	Send reply.  Reply will be truncated to size of receive buffer.
	//	We can't actually tell how long the reply is, but we assume it is the value of "cnt"
	int stat = MsgReply(rcvid, cnt, reply, cnt);						// success, send reply
	if (stat < 0)																	// if local failure
	{	perror("Remote msg reply failed"); fflush(stderr);	}	// report
}
//
//	The reverse test server -- runs as a separate thread.
//
//	Only run for test operations.
//
//	Pretends to be a real server and forwards messages to a test server.
//
static void reversetestserver(const char* destnode, const char* destname)
{	MsgServerPort serverport(0.0);										// create our server port
	RemoteMsgClientPort dest(destnode,destname,0.0);			// create our dest port
	//
	logprintf("Reverse remote msg server: simulating \"%s\" by connecting to \"%s\" on \"%s\"\n",getenv("ID"),destname,destnode);		// server is ready
    // create a channel, tell watchdog
    int stat = serverport.ChannelCreate();	
    if  (stat ) {
		perror("ChannelCreate failed.\n");
		serverport.Dump();
		abort();																						// should never happen
    }    
	//	Receive loop
	uint64_t lastwatchdog = gettimenowns();							// last write to watchdog
    while (1) {
		uint64_t now = gettimenowns();									// keep alive logic
		uint64_t elapsed = now - lastwatchdog;						// time since last msg
		{	if (elapsed > uint64_t(k_timeout*1000000000))
			{	logprintf("\n");														// keep output alive
				lastwatchdog = now;											// note that we did it
			}
		}	
    	_msg_info info;													// additional message info
        int rcvid = serverport.MsgReceive(&msg, sizeof(msg), &info);

        if (rcvid < 0) 
        {	//	Trouble
        	if (errno == ETIMEDOUT)								// normal timeout
        	{	logprintf("\n");
        		lastwatchdog = now;
        		continue;
        	}
        	perror("MsgReceive failed");							// should not happen
            break;
        }

        if (rcvid == 0) {/* Pulse received */
            switch (msg.m_pulse.code) {
            case _PULSE_CODE_DISCONNECT:
                /*
                 * A client disconnected all its connections (called
                 * name_close() for each name_open() of our name) or
                 * terminated
                 */
                ConnectDetach(msg.m_pulse.scoid);
                break;
                
            case _PULSE_CODE_UNBLOCK:
                /*
                 * REPLY blocked client wants to unblock (was hit by
                 * a signal or timed out).  It's up to you if you
                 * reply now or later.
                 */
                break;
                
            default:
                /*
                 * A pulse sent by one of your processes or a
                 * _PULSE_CODE_COIDDEATH or _PULSE_CODE_THREADDEATH 
                 * from the kernel?
                 */
                 break;
            }
            continue;
        }
#ifdef OBSOLETE
        /* A QNX IO message received, reject */
        if (msg.hdr.type >= _IO_BASE && msg.hdr.type <= _IO_MAX) {
        	printf("QNX I/O msg type %d received.\n",msg.hdr.type);	
            MsgError(rcvid, ENOSYS);
            continue;
        }
#endif // OBSOLETE
        /* A message (presumable ours) received, handle */
        handlereversedatamsg(dest, rcvid, msg.m_servermsg, info.msglen);								// do it
    }
}
//
//	getmynodename  -- get name of this node
//
static const char* getmynodename()
{
	static char buf[511];													// my node name
	int stat = netmgr_ndtostr(ND2S_LOCAL_STR | ND2S_QOS_HIDE, 0, buf, sizeof(buf));	// node 0 is always oneself
	if (stat < 0) return("QNX NATIVE NETWORKING NOT RUNNING");				// native networking not running
	return(buf);																// return node name, short form
}
//
//	usage -- usual usage 
//
static void usage()
{	printf("Usage: remotemsgserver [-v] [-r] TARGETSERVER [TARGETNODE]\n");	
	printf(" -v verbose\n");
	printf(" -r reverse mode, pretends to be watchdog server but talks to remote.\n");			
	printf("  Must run under the Overbot watchdog.\n");		
	exit(1);
}
//
//	main program
//
int main(int argc, const char* argv[])
{	bool verbose = false;														// verbose flag
	bool reverse = false;														// reverse mode
	const char* targetserver = 0;											// target server
	const char* targetnode = 0;
	for (int i=1; i<argc; i++)													// parse args
	{	const char* arg = argv[i];											// this arg
		if (arg[0] == '-')
		{	switch(arg[1]) {
			case 'v': 	verbose = true; break;
			case 'r':		reverse = true; break;
			default:	usage();
			}
		} else {
			if (targetnode) usage();											// one arg only
			if (!targetserver) 
			{ targetserver = arg; }
			else 
			{ targetnode = arg; }
		}		
	}
	if (!targetserver) usage();													// exactly one arg
	const char* id = getenv("ID");											// my ID
	if (!id) usage();																	// require ID
	if (!reverse)
	{	testserver(id,targetserver);											// run the server until killed
	} else {
		if (!targetnode) targetnode = getmynodename();		// use own node name
		reversetestserver(targetnode, targetserver); 				// run reverse server until killed
	}
	return(0);																			// done
}
