//
//  speedserverchassis.cc  --  speed server interface to chassis object
//
//	Used when controlling the real chassis.  There is also a dummy version for
//	debugging the move server. 
//
//	John Nagle
//	Team Overbot
//	November, 2004
//
#include <stdlib.h>
#include <unistd.h>

#include "logprint.h"
#include "speedserver.h"
//
//	handleStateChange  -- request for a state change
//
//	Returns true if success
//
bool SpeedServerChassis::handleStateChange(MsgSpeedSet::State newstate, bool& busy)
{	return(m_chassis.SetState(newstate, busy));							// try to change state
}
//
//	handleSpeedChange  -- request for a speed change
//
//	Returns true if success
//
bool SpeedServerChassis::handleSpeedChange(float accel, float speed, float steering)
{	m_chassis.SetSpeed(accel,speed, steering);								// set new control goals.
	return(true);																					// success in setting
}
//
//	handleGearChange  -- request for a gear change
//
//	Returns true if request accepted.
//
//	Must be in correct state before shifting.
//
bool SpeedServerChassis::handleGearChange(MsgSpeedSet::Gear newgear, bool& busy)
{	busy = false;
	if (newgear == m_chassis.GetGear()) return(true);						// nothing to do
	logprintf("Requested shift from %s to %s\n", ErrMsg(m_chassis.GetGear()), ErrMsg(newgear)); 
	switch (m_chassis.GetState()) {													// what to do based on state
	case MsgSpeedSet::state_parked:												// OK to shift
	case MsgSpeedSet::state_idle:														// OK to shift
	case MsgSpeedSet::state_shifting:												// shift in progress
	case MsgSpeedSet::state_paused:												// paused
		break;		
		
	case MsgSpeedSet::state_runwait:												// don't shift in runwait
		busy = true;
		return(true);			
		
	case MsgSpeedSet::state_run:														// we are in run state
		{	logprintf("Shift requested in run mode - stopping.\n");			
			bool good = handleStateChange(MsgSpeedSet::state_shifting, busy);			// go to shifting state
			if (!good) return(false); 														// sequence error
		}
		break;										
	
	default:
		return(false);																			// caller error
	}
	if (m_chassis.GetState() >= MsgSpeedSet::state_run) 					// if still in run state
	{	return(false);	} 																		// sequencing error
	//	OK, state allows gear change.
	if (!m_chassis.GetStopped()) 
	{	busy = true;
		return(true);																				// vehicle in motion, try again later
	}
	return(m_chassis.SetGear(newgear, busy));									// request gear change
}

//
//	handleFault  --  handle a fault
//
void SpeedServerChassis::handleFault(Fault::Faultcode faultid)
{	m_chassis.SetFault(faultid);
}
//
//	faultReported -- the fault has been reported.
//	
//	We will reset it if the vehicle has stopped.
//
void SpeedServerChassis::faultReported()
{	m_chassis.ResetFault();															// try to reset fault
}
//
//	Update  -- update the chassis object
//
//	Should happen every 100ms, never more than 120ms between updates.
//
void SpeedServerChassis::update()
{	m_chassis.SetVerbose(m_verbose);											// done too often, but cheap
	m_chassis.Update();																	// actually do the update
}
//
//	buildMsgReply -- fill in reply data about what chassis is doing
//
void
SpeedServerChassis::buildMsgReply(MsgSpeedSetReply& replymsg)
{
	//	Build reply data
	replymsg.m_odometer = m_chassis.GetOdometer();
	replymsg.m_speed = m_chassis.GetSpeed();
	replymsg.m_curvature = m_chassis.GetCurvature();
	replymsg.m_state = m_chassis.GetState();
	replymsg.m_gear = m_chassis.GetGear();
	replymsg.m_lastfault = m_chassis.GetFault();
	replymsg.m_hint  = m_chassis.GetHint();
}
//
//	GetState --  access to chassis state
//
MsgSpeedSet::State SpeedServerChassis::GetState()
{	return(m_chassis.GetState()); }
//
//	GetGear --  access to chassis state
//
MsgSpeedSet::Gear SpeedServerChassis::GetGear()
{	return(m_chassis.GetGear()); }


#ifdef OBSOLETE
        // get curent vehicle speed
 
        // Controller equations based on the following Transfer function:
        //
        // K(s) = k * (1 + 1/(Ti * s) + (Td * s) / (1 + Td*s/N))
        //
        // After discretization using modified MPZ method:
        //
        // K(z) = k * (1 + (T/Ti) * (1/(z - 1)) + (Td/T) * ((z - 1) / (z - a)))
        //
        // Vi(k) = Vi(k-1) + Ki * e(k)
        // Vd(k) = a * Vd(k-1) + Kd * (e(k) - e(k-1))
        // u(k) = Kp * (e(k) + Vi(k) + Vd(k))

        e = deltaV;
        vi = vi1 + Ki * e1;
        vd = a*vd1 + Kd * (e - e1);
        u = Kp * (e + vi + vd);

        vi1 = vi;
        vd1 = vd;
        e1 = e;

#endif // OBSOLETE
