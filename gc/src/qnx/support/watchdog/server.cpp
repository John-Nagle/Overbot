//
//	Watchdog  -- top-level watchdog program for QNX real-time applications
//
//	File server.cpp --  looks up processes by ID, in response to queries
//
//	J.	Nagle
//	August, 2002
//
#include <stdio.h>
#include "messaging.h"
#include "watchdog.h"
#include "logprint.h"
//
//	server -- waits for requests, looks up current PID of program, returns it.
//
//	Returns only if sent a kill-type signal
//
//	Query: a text string
//	Reply: a pid_t
//
void* Watchdog::server()
{
    try
    {
        for (;;)																			// message loop
        {	struct _msg_info msginfo;											// received info about msg
            WatchdogMsg msg;													// incoming message area
            const int rcvid = MsgReceive(m_serverchid, &msg,sizeof(msg), &msginfo);	// wait for msg
            if (aborting()) throw("Abort in progress");					// bail out if aborting
            if (rcvid < 0)																// if server failed
            {	char msg[255];														// build message here
                snprintf(msg,sizeof(msg), "Watchdog server failure: %s", strerror(errno));
                panic(msg);															// fatal
                return(0);																// fails
            }
            if (rcvid == 0)															// if pulse, ignore
            {	if (verbose())
                {
                    logprintf("Watchdog server received a pulse. Ignored.\n");
                }
                continue;
            }
            const uint32_t len = uint32_t(msginfo.msglen);					// length of msg
            if (len < sizeof(msg.m_ok.m_msgtype))	// if msg too short
            {	MsgError(rcvid,EBADRPC);
                continue;
            }					// report as bad RPC request.
            //
            //	Fan out on server requests, based on code at beginning of message
            //
            switch (msg.m_ok.m_msgtype)
            {								// fan out on type
            case WatchdogMsgOK::k_msgtype:							// OK msg
                {	if (len != sizeof(msg.m_ok))
                    {
                        MsgError(rcvid,EBADRPC);
                        break;
                    }	// msg size check
                    int stat = serverOK(msg.m_ok, msginfo);				// handle msg
                    int err = MsgError(rcvid,stat);								// reply with result code only
                    if (err)
                        perror("Watchdog server MsgError failed.");
                    break;
                }

            case WatchdogMsgID::k_msgtype:							// ID msg - look up ID of service
                {	//	Look up server by program ID name
                    if (len != sizeof(msg.m_id))
                    {
                        MsgError(rcvid,EBADRPC);
                        break;
                    }	// msg size check
                    int stat = serverID(msg.m_id, msginfo);					// handle msg
                    if (stat != EOK)
                    {
                        MsgError(rcvid, stat);
                        break;
                    }							// handle error
                    int err = MsgReply(rcvid,msg.m_id);						// reply with result filled into original msg
                    if (err)
                        perror("Watchdog server MsgReply failed.");
                    break;
                }

            case WatchdogMsgSV::k_msgtype:							// SV msg - identifying self as server
                {	if (len != sizeof(msg.m_sv))
                    {
                        MsgError(rcvid,EBADRPC);
                        break;
                    }	// msg size check
                    int stat = serverSV(msg.m_sv, msginfo);				// handle msg
                    int err = MsgError(rcvid,stat);								// reply with result code only
                    if (err)
                        perror("Watchdog server MsgError failed.");
                    break;
                }

            default:
                {
                    MsgError(rcvid,EPROCUNAVAIL);							// no such message type
                }
            }																				// end switch
        }
    }
    catch (const char* msg)												// during abort, other threads throw this
    {
        logprintf("Server thread exception: %s\n", msg);
    }
    return(0);																		// exit thread
}
//
//	serverOK  --  reset watchdog timer
//
int Watchdog::serverOK(WatchdogMsgOK& msg, const struct _msg_info& msginfo)
{
    WatchedProgram* pgm = findbymsginfo(msginfo);		// look up by who sent msg
    if (!pgm)
        return(ESRCH);												// no find, fails
    if (verbose())
    {
        logprintf("[%s]: reset watchdog OK.\n",pgm->getid().c_str());
    }
    //	***NEED TO ACTUALLY RESET WATCHDOG***
    //	***NEED TO GET SYSTEM READY/NOT READY STATUS***
    return(EOK);																	// success
}
//
//	serverID  --  handle ID message - look up a server
//
int Watchdog::serverID(WatchdogMsgID& msg, const struct _msg_info& msginfo)
{
    if (verbose())																// debug
    {	logprintf("Watchdog request: find server with ID=%s\n",msg.m_progid);
    }	// print request
    WatchedProgram* pgm = findbyid(msg.m_progid,sizeof(msg.m_progid));	// look up by name (may not be null-terminated)
    if (!pgm)
        return(ESRCH);												// no find, fails
    msg.m_portid = pgm->getmsgportid()	;							// return port ID
    if (msg.m_portid.m_pid < 0)											// if null PID
    {	return(ENOTCONN);
    }												// not connected
    //	Convert node ID from relative to us to relative to calling node.
    msg.m_portid.m_nodeid = netmgr_remote_nd(msginfo.nd, msg.m_portid.m_nodeid);
    if (verbose())																// debug
    {	logprintf("Watchdog request: found server with ID=%s: node %d, pid %d, chid %d\n",msg.m_progid, msg.m_portid.m_nodeid, msg.m_portid.m_pid, msg.m_portid.m_chid);
    }	// print request
    return(EOK);																	// success
}
//
//	serverSV  -- register a process as having a server
//
//	***NEED TO CLEAR m_portid WHEN PROGRAM EXITS***
//
int Watchdog::serverSV(WatchdogMsgSV& msg, const struct _msg_info& msginfo)
{
    if (verbose())																// debug
    {	logprintf("Watchdog request: register server for node %d, pid %d\n",msginfo.nd, msginfo.pid);
    }	// print request

    //khianhao
    //we will use findbyid to find the owner of this mesg
    //WatchdogMsgSV now contains the name as seen by the server program
    WatchedProgram * pgm = findbyid(msg.m_name, strlen(msg.m_name));
#ifdef OBSOLETE	// can't register two servers to one process

    if (!pgm)																		// if lookup by name didn't work
    {	pgm = findbymsginfo(msginfo);								// look up by who sent msg
    }
#endif // OBSOLETE
    if (!pgm)
    {
        logprintf("[%s]: Node %d, process %d tried to register as a server but could not be found as a process under watchdog control.\n",
                  "???",msginfo.nd,msginfo.pid);
        return(ESRCH);															// no such process
    }
    ////if (pgm->getchid() >= 0)											// if not null PID
    ////{	return(EISCONN);	}												// already known
#ifdef OBSOLETE
    MsgPortID serverport;													// build server port info
    serverport.m_chid = msg.m_chid;
    serverport.m_nodeid = msginfo.nd;								// other node in local terms
    serverport.m_pid = msginfo.pid;
#endif // OBSOLETE

    pgm->setserverpidandchid(msginfo.pid,msg.m_chid);	// set process and channel ID
    if (verbose())
    {
        logprintf("[%s]: registered server at node %d, pid %d,  chid %d.\n",pgm->getid().c_str(),msginfo.nd, msginfo.pid, msg.m_chid);
    }
    return(EOK);																	// success
}
