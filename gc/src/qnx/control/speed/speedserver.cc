////////////////////////////////////////////////////////////////////////////////
//
//    File:
//      speedserver.cc
//
//    Usage:
//
//	In the watchdog startup file (startfile.txt):
//		ID=SPEED speedserver
//
//    Description:
//
//	Takes speed commands from the Move server and executes them by issuing
//	lower level commands to the Brake and Throttle servers.
//
//
//      Simulation modes are provided for testing purposes
//		(see speedservermenu.h)
//
//	The Speed Server consists of the following threads:
//          	- a main thread, which accepts messages
//     		- a menu thread
//
//    Messages received:
//
//	SpeedServerMsg::SetSpeed
//
//    Messages sent:
//
//
//    Written By:
//
//      Achut Reddy
//      Team Overbot
//      January 2004
//
////////////////////////////////////////////////////////////////////////////////


#include <stdlib.h>
#include <unistd.h>

#include "logprint.h"
#include "speedserver.h"
#include "timeutil.h"
//
//	constants
//
const float k_msg_timeout = 0.120;										// 120ms timeout
																							// just slightly shorter than hardware watchdog


//
// Class member functions
//
//
//	Constructor
//
SpeedServer::SpeedServer()
	: m_serverport(k_msg_timeout), m_verbose(false)
{
    // initialize server messaging
    int stat = m_serverport.ChannelCreate();	// create a channel, tell watchdog
    if  (stat )
    {
        perror("SpeedServer::SpeedServer - ChannelCreate failed.\n");
        m_serverport.Dump();
        throw("ChannelCreate failed");				// fails
    }

    //
    // initialize server data
    //
};
//
// destructor
//
SpeedServer::~SpeedServer()
{
}
//
//	messageThread  --  the server thread
//
//	Accepts messages and processes them.
//	Will block for short periods only.	
//	Replies to message before performing the requested action, so as not to block the
//	move server unduly.
//
void SpeedServer::messageThread(bool verbose)
{	m_verbose = verbose;												// set verbosity
    SpeedServerMsg	msg;												// area for incoming msg

    // loop collecting messages forever
    while (true)
    {
		//	Update system state
		update();																// read controllers and update system state
        // wait for message
        int rcvid = m_serverport.MsgReceive(msg);
        // handle errors
        if ( rcvid < 0 )
        {
        	if (errno == ETIMEDOUT)									// if timeout
        	{	handleTimeout();											// handle a timeout (this causes an e-stop)
       			continue;
        	}
            logprintf("MsgReceive failed: %s\n",strerror(errno));
            usleep(10000);													// 10ms wait to avoid tight loop if repeated trouble 
            continue;
        }
        else if ( rcvid == 0 )
        {
            logprintf("SpeedServer::messageThread - received a pulse.\n");
            // pulses don't require a reply
            continue;
        }
        // handle message
        handleMessage(rcvid, msg);

        // tell the watchdog we are still alive
        ////m_serverport.watchdogreset();										// avg 100ms, max about 120ms.
    }
    abort();																					// unreachable
}
//
//	Handle messages
//
void
SpeedServer::handleMessage(int rcvid, const SpeedServerMsg& msg)
{
    // Dispatch on message type
    switch ( msg.m_speedset.m_msgtype )
    {

    case MsgSpeedSet::k_msgtype:											// SetSpeed msg
        handleSpeedSet(rcvid,msg.m_speedset);							// handle the message
        break;
        
    case MsgSpeedStop::k_msgtype:											// SetStop msg
    	handleSpeedStop(rcvid,msg.m_speedstop);						// handle the message
    	break;

    default:
        logprintf("SpeedServer::handleMessage - unknown message type: %x\n",
                msg.m_speedset.m_msgtype);
        break;
    }
}
//
//	handleTimeout -- handle a timeout, generic
//
//	This will cause a stop if the vehicle is in a run state
//
void SpeedServer::handleTimeout()
{
	if (GetState() != MsgSpeedSet::state_run) return;						// ignore timeouts unless in RUN mode
	handleFault(Fault::movetimeout);												// cause a fault condition
}
//
//	handleSpeedStop -- causes a fault on request of the move server.
//	
void SpeedServer::handleSpeedStop(int rcvid, const MsgSpeedStop& msg)
{
	handleFault(msg.m_fault);										// force fault
	MsgError(rcvid, EOK);											// reply OK, no data
}
//
//	handleSpeedSet -- Handle MsgSpeedSet
//
//	Sets current message.
//
void
SpeedServer::handleSpeedSet(int rcvid, const MsgSpeedSet& msg)
{	bool good = true;																			// no problems yet
	bool busy = false;
	MsgSpeedSetReply::Hint hint = MsgSpeedSetReply::hint_none;	// no hint yet
	//	Get current state from chassis and return it.
	//	Check for state change.  if we have a state change, we must check its validity
	//	in current circumstances.
	MsgSpeedSet::State newstate = msg.m_state;								// desired state
	if (newstate != GetState())															// if a state change is requested
	{	if ((msg.m_gear != GetGear()) && (msg.m_state == MsgSpeedSet::state_run))	// if asking for a gear change while moving
		{	newstate = MsgSpeedSet::state_shifting;	}							// ask for shifting state
		good = handleStateChange(msg.m_state, busy);						// try to do it
		if (!good)																					// if failed to change state
		{	handleFault(Fault::eventsequenceerror);								// caller did something bad in run state - serious
		} else {																					// state change occured, but did we get what we wanted?
			if (busy)																				// not yet, but in progress
			{	hint = MsgSpeedSetReply::hint_busy;								// but it is happening
			}
		}
	}
	//	Check for a gear change.  Only allowed in some states.
	if (good && !busy)																			// only if good
	{	if (msg.m_gear != GetGear())														// if gear change
		{	good = handleGearChange(msg.m_gear, busy);						// try to do it
			if (good)																					// if failed to change state
			{	if (busy)																				// not yet, but in progress
				{	hint = MsgSpeedSetReply::hint_busy;								// but it is happening
				}
			}
		} else {																						// in correct gear
			if (GetState() == MsgSpeedSet::state_shifting && msg.m_state == MsgSpeedSet::state_run)	// if was shifting
			good &= handleStateChange(MsgSpeedSet::state_run, busy);	// 
			if (good)																					// if failed to change state
			{	if (busy)																				// not yet, but in progress
				{	hint = MsgSpeedSetReply::hint_busy;								// but it is happening
				}
			}
		}
	}
	if (!good)
	{	logprintf("ERROR: state or gear change rejected (%s to %s) (%s to %s).\n",
			 ErrMsg(GetState()), ErrMsg(msg.m_state), ErrMsg(GetGear()), ErrMsg(msg.m_gear)); 
		handleFault(Fault::eventsequenceerror);										// caller did something bad - fault down
	}
	//	Build reply data
	MsgSpeedSetReply replymsg;														// blank reply message
	buildMsgReply(replymsg);															// fill in state of chassis
	if (hint != MsgSpeedSetReply::hint_none) replymsg.m_hint = hint;	// use hint if necessary
	//	Reply to request
	int stat = MsgReply(rcvid, replymsg);											// normal reply
	if (stat)																							// trouble
	{	logprintf("MsgReply failed: %s\n", strerror(errno)); 
		return;																						// don't do speed change if problem
	} 
	if (replymsg.m_lastfault != Fault::none) faultReported();				// fault has been reported and can be cleared.
	//	We've now replied to the request. Now do any speed changes needed.
	//	This applies only in run mode.
	if (GetState() != MsgSpeedSet::state_run) return;							// no action if not in run mode
	handleSpeedChange(msg.m_acceleration, msg.m_speed, msg.m_curvature);			// adjust speed
}
