//
//	automodes.cpp  --  automatic modes 
//
//	John Nagle
//	Team Overbot
//	January. 2004
//
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include "drivingcontrolmode.h"
#include "manualdrive.h"
#include "logprint.h"
#include "mapservermsg.h"
#include "remotemessaging.h"
#include "tuneable.h"
#include "proto.h"													// so we can raise dialog boxes
#include "ablibs.h"
#include "abimport.h"

//
//	Constants
//
const char* k_map_server_name = "DUMMYMAP";	
const char* k_map_server_node = "gcrear0";
const double k_map_server_timeout = 0.5;				// slow timeout, we're just observing
//
//	Constructor
//
AutoModes::AutoModes(Manualdrive& owner, const char* nodename, const char* servername) 
	: DrivingControlMode(owner),
	m_runmode(false),
	m_mapservernode(nodename),
	m_mapservername(servername),
	m_mapserverport(m_mapservernode, m_mapservername,k_map_server_timeout)
{	
}

//
//	AutoModes -- just observes what the automatic system is doing.
//
bool AutoModes::takecontrol()
{	ost::MutexLock lok(m_lock);							// lock
	setrunmode(false);																// not in run mode
	getowner().getmoveinfodisplay().enable(true);					// enable
	return(true);																			// take control
}
//
//	releasecontrol  -- mode change coming
//
//	Implies an emergency stop
//
void AutoModes::releasecontrol()
{	ost::MutexLock lok(m_lock);													// lock
	setrunmode(false);																// not in run mode
	emergencystopactivated("Mode change","");						// send a final stop message
	m_mapserverport.ConnectDetach();										// break connection to map servers
	getowner().getmoveinfodisplay().enable(false);					// disable
}
//
//	emergencystopactivated -- an emergency stop has occured
//
//	Actually do the stop
//
void AutoModes::emergencystopactivated(const char* who, const char* why)
{	ost::MutexLock lok(m_lock);													// lock
	logprintf("Emergency stop activated: %s  %s\n", who, why);	// ***TEMP***
	////if (!m_runmode) return;															// only in run mode
	////setrunmode(false);																// not in run mode
	//	Always send emergency stop message, even if not in run mode.
	//	Caller turns run mode off first.
	//	Build and send emergency stop message
	MoveServerMsg::MsgMoveStop stopmsg;
	stopmsg.m_msgtype = MoveServerMsg::MsgMoveStop::k_msgtype;	// set type
	stopmsg.m_fault = Fault::manualkill;										// reason for stop
	int stat = m_mapserverport.MsgSend(stopmsg);					// send the message
	if (stat < 0)																			// if trouble
	{	logprintf("Unable to send emergency stop command: %s\n",strerror(errno));		// not good, but move server will time out
		return;
	}
}
//
//	getrunmode  -- true if in run mode
//
bool AutoModes::getrunmode()
{	
	return(m_runmode);																			// ***TEMP***
}
//
//	updatedrive  -- called periodically if caller is happy
//
void AutoModes::updatedrive()
{
	ost::MutexLock lok(m_lock);																// lock
	MoveServerMsg::MsgMoveQuery msg;												// actually to map server, but same code
	msg.m_msgtype = MoveServerMsg::MsgMoveQuery::k_msgtype;		// set message type
	MapServerMsg::MsgMapQueryReply reply;
	int stat = m_mapserverport.MsgSend(msg, reply);							// query server
	if (stat < 0)																						// if can't reach mao server
	{	bzero(&reply, sizeof(reply));														// blank the output
		setrunmode(false);																		// map server is down
	} else {																							// if talking to map server
		setrunmode(true);																		// we are up
	}
	updatemovedisplay(reply);																// update display
}

//
//	joystick_move  -- joystick/pedals have moved
//
//	All this does is trigger E-stop if you press the brake pedal.
//
void AutoModes::joystick_move(float x, float y)
{	//	Joystick has moved.  Update controllers.
	ost::MutexLock lok(m_lock);							// lock
	if (!m_runmode)											// if not in run mode
	{	return;	}													// do nothing
	//	Compute speed
	if (y > 0.5)													// brake pedal pushed
	{	float brake =  (y - 0.5)*2;							// upper half of range is brake
		//	Brake is now in range 0..1
		if (brake > 0.80)										// if big brake
		{	getowner().emergencystop("Brake pedal pushed hard.");	// trip emergency stop
			return;													
		}
	}
}
//
//	setrunmode -- change to run mode
//
bool AutoModes::setrunmode(bool runmodeon)
{	
	if (!runmodeon)																			// if turning off
	{	m_runmode = false;	}																// stop immediately, even if locking problem
	ost::MutexLock lok(m_lock);															// lock
	m_runmode = runmodeon;															// set run mode
	//	***MORE***
	return(true);
}

//
//	shift_button_pressed  -- manual shift request
//
//	Request low, high, neutral, or reverse.
//
void AutoModes::shift_button_pressed(MsgSpeedSet::Gear newgearin)
{	
	ost::MutexLock lok(m_lock);							// lock
}
//
//	updatemovedisplay  --  Process a reply from the map server.
//
//	Updates the display.
//
void AutoModes::updatemovedisplay(const MapServerMsg::MsgMapQueryReply& reply)
{
	MoveInfoDisplay& display = getowner().getmoveinfodisplay();	// object to update
	const MoveServerMsg::MsgMoveReply& movereply = reply.m_movereply;
	display.setstate(ErrMsg(movereply.m_speedreply.m_state));				// update state on screen
	display.sethint(ErrMsg(movereply.m_speedreply.m_hint));					// update hint
	display.setmovedistance(movereply.m_lastdistance);							// update distance
	display.setwaypointnumber(reply.m_waypointserial);							// update waypoint serial
	////printf("Waypoint serial: %d\n", reply.m_waypointserial);	// ***TEMP DEBUG***
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
//	Submodes
//
AutoControlMode::AutoControlMode(Manualdrive& owner) 
	: AutoModes(owner, (getenv("TESTMOVENODE") ? getenv("TESTMOVENODE") : k_map_server_node), k_map_server_name)
{
}
//
//	getmodename  -- get name of this mode
//
const char* AutoControlMode::getmodename() const
{	return("Automatic driving");	}						// at least for now

//	Offline mode - always look on local node for node
OfflineAutoControlMode::OfflineAutoControlMode(Manualdrive& owner) 
	: AutoModes(owner, getmynodename(), k_map_server_name)
{
}
//
//	getmodename  -- get name of this mode
//
const char*  OfflineAutoControlMode::getmodename() const
{	return("OFFLINE TEST of automatic driving");	}						// at least for now
//
//	updatemovedisplay  --  Process a reply from the move server.
//
//	Updates the display.
//
void OfflineAutoControlMode::updatemovedisplay(const MapServerMsg::MsgMapQueryReply& reply)
{
	AutoModes::updatemovedisplay(reply);									// do parent
	////MoveInfoDisplay& display = getowner().getmoveinfodisplay();	// object to update
	double odometer = reply.m_movereply.m_speedreply.m_odometer;											// get odometer
	PtSetResource (ABW_odometer_value, Pt_ARG_NUMERIC_VALUE, &odometer, 0 );
	getowner().getspeedmeasuring().SetUnscaledValue(reply.m_movereply.m_speedreply.m_speed);			// set speedometer
	getowner().updateGearButtons(reply.m_movereply.m_speedreply.m_gear, reply.m_movereply.m_speedreply.m_gear);	// set gear buttons
}

