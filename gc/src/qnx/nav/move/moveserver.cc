///////////////////////////////////////////////////////////////////////////////
//
//    File:
//      moveserver.cc
//
//    Usage:
//
//	In the watchdog startup file (startfile.txt):
//		ID=MOVE moveserver
//
//    Description:
//
//	Takes high-level move commands from mapserver and implements them
//	by translating them into low-level commands to the vehicle actuator
//	servers (steering, throttle, brake, trasmission).
//
//      Performs sanity checks on the move commands.  It contains a simple
//      dynamics model and will refuse to move in any manner which threatens
//      vehicle safety.  If necessary it will reduce the specified speed until
//      it is deemed safe.
//
//      Also checks for potential collisions by querying the vorad server;
//	if a collision appears imminent, it will halt the vehicle in spite
//	of any move commands.
//
//	Keeps track of predicted position vs. actual position error so the
//	algorithm can adapt and adjust for it.
//
//
//      Simulation modes are provided for testing purposes
//		(see moveservermenu.h)
//
//
//	The Move Server consists of the following threads:
//          	- a main thread, which accepts messages
//              - a command thread
//     		- a menu thread
//
//    Messages received:
//
//	MoveServerMsg::Move
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
#include <strings.h>										// only for bzero
#include <algorithm>										// min, max
#include "logprint.h"
#include "timeutil.h"
#include "moveserver.h"
//
//	Constants
//
const double k_moveservertimeout = 0.100;	// 10Hz cycle
const double k_speedservertimeout = 0.150;	// speed server stuck?
const double k_gpsinsport = 0.300;					// GPS/INS server stuck?
//
// Class member functions
//
//
//	Constructor
//
MoveServer::MoveServer()
:	m_serverport(0.0),																// us, as server, no timeout
	m_speedserverport("SPEED",k_speedservertimeout),			// to speed server
	m_gpsinsport("GPSINS",k_speedservertimeout),					// to GPS/INS server
	m_verbose(false),
	m_lastdistance(0),
	m_lastfault(Fault::none),														// no fault yet
	m_timedout(false)	,																// not timed out
	m_lastlogtime(0),
	m_avgvibration(0)																// filtered vibration
{
	bzero(&m_lastgpsinsreply,sizeof(m_lastgpsinsreply));			// zero out state until we get some
	m_lastgpsinsreply.err = GPSINS_MSG::INITIALIZATION;			// note not initialized
	bzero(&m_lastspeedreply,sizeof(m_lastspeedreply));			// last state
    // initialize server messaging
    int stat = m_serverport.ChannelCreate();	// create a channel, tell watchdog
    if  (stat )
    {
        perror("SpeedServer::SpeedServer - ChannelCreate failed.\n");
        m_serverport.Dump();
        throw("ChannelCreate failed");				// fails
    }
};
//
//	destructor
//
MoveServer::~MoveServer()
{}
//
//	messageThread - the actual server
//
//
//	diagnoseTimeout -- diagnose a move server timeout
//
//	For some reason, probably priority-related, we don't always get a move through every 100ms, as required.
//
static void diagnoseTimeout(const uint64_t waitstart, const uint64_t waitend, 
	const uint64_t handlestart, uint64_t handleend, const uint64_t prevwaitend)
{
	uint64_t handleelapsed = handleend - handlestart;		// handle time, elapsed nanoseconds
	uint64_t waitelapsed = waitend - waitstart;					// wait time, elapsed nanoseconds
	uint64_t cycleelapsed = waitend - prevwaitend;				// cycle time of entire last cycle
	double handletime = (handleelapsed * 0.000000001);
	double waittime = (waitelapsed * 0.000000001);
	double cycletime = (cycleelapsed * 0.000000001);
	logprintf("Last request executed in %1.4f secs. Waited %1.4fs for command. Last cycle %1.4fs.\n",
		handletime, waittime, cycletime);
}
//
//	messageThread  --  the server thread
//
//	Accepts messages and processes them.
//	Will block for short periods only.	
//	Replies to message before performing the requested action, so as not to block the
//	move server unduly.
//
void MoveServer::messageThread(bool verbose)
{	m_verbose = verbose;												// set verbosity
    MoveServerMsg::MsgUnion	msg;								// area for incoming msg

    // loop collecting messages forever
    uint64_t lasthandlestart = 0;										// timestamp info
    uint64_t lasthandleend = 0;
    uint64_t prevrcvwaitend = 0;
    uint64_t rcvwaitend = 0;
    while (true)
    {	
        // wait for message
        uint64_t rcvwaitstart = gettimenowns();			// timestamp before wait
        int rcvid = m_serverport.MsgReceive(msg);
        prevrcvwaitend = rcvwaitend;							// save previous for cycle time
        rcvwaitend = gettimenowns();							// timestamp before wait
        // handle errors
        if ( rcvid < 0 )
        {
        	if (errno == ETIMEDOUT)									// if timeout
        	{	handleTimeout();											// handle a timeout (this causes an e-stop)
        		diagnoseTimeout(rcvwaitstart, rcvwaitend, lasthandlestart, lasthandleend, prevrcvwaitend);
        		continue;
        	}
            logprintf("MsgReceive failed: %s\n",strerror(errno));
            usleep(10000);													// 10ms wait to avoid tight loop if repeated trouble 
            continue;
        }
        else if ( rcvid == 0 )
        {
            logprintf("MoveServer::messageThread - received a pulse.\n");
            // pulses don't require a reply
            continue;
        }
        // handle message
        lasthandlestart = gettimenowns();						// timestamp before action
        handleMessage(rcvid, msg);
       	lasthandleend = gettimenowns();							// timestamp before action
       log();																					// do periodic logging
    }
    abort();																					// unreachable
}
//
//	handleMessage -- handle all messages
//
void MoveServer::handleMessage(int rcvid, const MoveServerMsg::MsgUnion& msg)
{
    // Dispatch on message type
    switch ( msg.m_header.m_msgtype )
    {
    case MoveServerMsg::MsgMove::k_msgtype:		// MOVE msg
        handleMove(rcvid, msg.m_move);					// handle it
        break;
        
	case MoveServerMsg::MsgMoveStop::k_msgtype: // Emergency stop
		handleStop(rcvid,msg.m_stop);							// handle it
		break;
	
	case MoveServerMsg::MsgMoveQuery::k_msgtype:	// status query only
		handleQuery(rcvid);											// handle it
		break;

    default:
        logprintf( "MoveServer::handleMessage - unknown message type: 0x%8x\n",
                msg.m_header.m_msgtype);
        MsgError(rcvid, EINVAL);									// invalid request
        break;
    }
}

//
//	handleMove -- handle Move message
//
void MoveServer::handleMove(int rcvid, const MoveServerMsg::MsgMove& msg)
{
	MsgSpeedSetReply speedreply;						// reply from speed server
	int stat = sendSpeedMessage(msg, speedreply);		// send request to speed server
	if (stat < 0)
	{	MsgError(rcvid, errno);									// handle error
		return;
	}
	MoveServerMsg::MsgMoveReply reply;				// our reply
	reply.m_speedreply = speedreply;					// return reply from speed server
	reply.m_lastdistance = msg.m_distance;			// return distance moved
	MsgReply(rcvid, reply);										// return a reply
	m_lastspeedreply = speedreply;						// save last speed reply
	m_lastdistance = msg.m_distance;					// save last distance to move
	m_timedout = false;											// not timed out, good
	sendGPSINSMessage();										// update positional info
}
//
//	handleQuery -- handle a status query request
//
//	Doesn't do anything, just queries.
//	Used for remote user interface, not actual operation.
//
void MoveServer::handleQuery(int rcvid)
{
	MoveServerMsg::MsgMoveReply reply;				// the reply
	reply.m_speedreply = m_lastspeedreply;			// return reply from speed server
	reply.m_lastdistance = m_lastdistance;				// return last distance moved
	//	Timeout handling.  If no SPEED requests are being sent, the SPEED server
	//	isn't being queried.  So we need to return the last fault condition.
	//	We really should query the speed server for status, but we don't yet
	//	have a way to do that without resetting its timer. 
	if (m_timedout && reply.m_speedreply.m_lastfault == Fault::none)	// if timed out
	{	reply.m_speedreply.m_lastfault = m_lastfault;						// use fault code if available
		if (reply.m_speedreply.m_lastfault == Fault::none)				// if no fault code available
		{	reply.m_speedreply.m_lastfault = Fault::movetimeout;	}	// report a timeout state
	}
	MsgReply(rcvid, reply);										// return a reply
}
//
//	handleTimeout  --  a timeout has occured
//
//	For now, this is treated as an emergency stop
//
void MoveServer::handleTimeout()
{	setFault(Fault::movetimeout);
	m_timedout = true;											// now in a timed out state
	sendGPSINSMessage();										// update positional info
}
//
//	handleStop -- handle Stop message
//
void MoveServer::handleStop(int rcvid, const MoveServerMsg::MsgMoveStop& msg)
{	logprintf("E-stop requested by client.\n");			// higher level is unhappy.
	int stat = setFault(msg.m_fault);						// set fault in speed server
	if (stat < 0)														// if trouble	
	{	MsgError(rcvid, errno); }								// pass to caller
	MsgError(rcvid, EOK);										// accept emergency stop
	sendGPSINSMessage();										// update positional info
}
//
//	setFault  -- set a fault condition
//
//	Causes an emergency stop
//
int MoveServer::setFault(Fault::Faultcode faultid)
{
	logprintf("Fault detected: %s\n", Fault::ErrMsg(faultid));	
	MsgSpeedStop stopmsg;
	stopmsg.m_msgtype = MsgSpeedStop::k_msgtype;				// set message type
	stopmsg.m_fault = faultid;														// use provided fault code
	m_lastfault = faultid;																// save last fault sent downward
	int sink;
	int stat = m_speedserverport.MsgSend(stopmsg, sink);		// send emergency stop to speed server
	if (stat < 0)
	{	logprintf("Error sending emergency stop to speed server: %s\n", strerror(errno));	}
	return(stat);														// return nonzero if fail
}
//
//	sendSpeedMessage  -- send request to SPEED server
//
int MoveServer::sendSpeedMessage(const MoveServerMsg::MsgMove& msg, MsgSpeedSetReply& speedreply)
{
	//	Build request to speed server
	MsgSpeedSet speedmsg;									// request to speed server
	speedmsg.m_msgtype = MsgSpeedSet::k_msgtype;	// set message type
	speedmsg.m_curvature = msg.m_curvature;	// set steering per request
	speedmsg.m_gear = msg.m_gear;					// requested gear
	speedmsg.m_state = MsgSpeedSet::state_run;	// request run state
	speedmsg.m_speed = 0;									// assume zero speed
	const float k_temp_accel = 0.25;						// ***TEMP*** temporary canned accel (g)
	const float k_g = 9.8;										// acceleration of gravity
	speedmsg.m_acceleration = k_temp_accel*k_g;			// temporary accel
	if (m_lastspeedreply.m_lastfault != Fault::none)// if we are in a faulted state
	{	
		speedmsg.m_state = MsgSpeedSet::state_idle;		// have to go back to idle state to clear it
		logprintf("Clearing fault: %s\n", Fault::ErrMsg(m_lastspeedreply.m_lastfault));
	} else if (msg.m_gear == MsgSpeedSet::gear_neutral)		// if request for neutral, go to idle
	{	speedmsg.m_state = MsgSpeedSet::state_idle;		// usually for mission completed
	} else if ((m_lastspeedreply.m_state != MsgSpeedSet::state_run)
	||	(m_lastspeedreply.m_hint == MsgSpeedSetReply::hint_busy) // if cranking, or waiting, or
	|| (m_lastspeedreply.m_gear != msg.m_gear))	// or shifting
	{	if (m_verbose)																// debug output
		{	logprintf("Busy: %s%s%s\n",										// busy, waiting
				((m_lastspeedreply.m_hint == MsgSpeedSetReply::hint_busy) ? "(from speed server)" : ""),	// why
				((m_lastspeedreply.m_gear != msg.m_gear) ? "(shifting)" : ""),
	 			((m_lastspeedreply.m_state != MsgSpeedSet::state_run)	 ? "(not in run state)" : ""));
	 	}
	}
	else																	// OK to go
	{	float speed = msg.m_speed;							// requested speed
		if (!getINSValid())											// if INS not valid
		{	speed = 0;												// force zero speed
			logprintf("Speed reduced from %3.2fm/s to %3.2fm/s due to INS failure.\n", msg.m_speed, speed);
		}
		speed = std::min(speed, safeSpeedForDistance(msg.m_distance, getPitch()));	// apply stopping distance check
		float speedlimit = safeSpeedLimit();				// configured speed limit
		if (speed > speedlimit)
		{
			logprintf("Speed reduced from %3.2fm/s to %3.2fm/s due to configured speed limit.\n", speed, speedlimit);
			speed = speedlimit;
		}
		float safecurvespeed = safeSpeedForCurvature(msg.m_curvature, getRoll());	// apply skid load check
		if (speed > safecurvespeed)
		{	
			logprintf("Speed reduced from %3.2fm/s to %3.2fm/s due to tight turn.\n", speed, safecurvespeed);
			speed = safecurvespeed;
		}
		float safeterrainspeed = safeSpeedForRoughness(m_lastspeedreply. m_speed, getVibration());	// apply vibration check
		if (speed > safeterrainspeed)
		{	
			logprintf("Speed reduced from %3.2fm/s to %3.2fm/s due to excessive vibration.\n", speed, safeterrainspeed);
			speed = safeterrainspeed;		
		}
		speedmsg.m_speed = speed;							// desired speed
	}
	int stat = m_speedserverport.MsgSend(speedmsg, speedreply);	// pass to speed server
	if (stat < 0)														// if trouble
	{	
		setFault(Fault::networkerror);						// report problem	
		return(stat);
	}
	m_lastfault = speedreply.m_lastfault;				// save last fault, if any
	m_timedout = false;											// not timed out, good
	if (m_verbose)
	{	logprintf("Move: %4.2fm  %4.2fm/s -> %4.2fm/s,  curvature %6.2f\n",
			msg.m_distance, msg.m_speed, speedmsg.m_speed, msg.m_curvature);
	}
	return(0);
}
//
//	sendGPSINSMessage  -- get positional data from GPS/INS
//
int MoveServer::sendGPSINSMessage()
{
	GPSINSMsgReq gpsinsmsg;											// request to GPS/INS server
	gpsinsmsg.m_msgtype = GPSINSMsgReq::k_msgtype;	// set message type
	int stat = m_gpsinsport.MsgSend(gpsinsmsg, m_lastgpsinsreply);
	if (stat < 0)																	// if trouble
	{	
		setFault(Fault::networkerror);									// report problem
		bzero(&m_lastgpsinsreply, sizeof(m_lastgpsinsreply));	// fails
		m_lastgpsinsreply.err = GPSINS_MSG::INITIALIZATION;		// note not initialized
		return(stat);
	}
	return(0);																		// success
}
