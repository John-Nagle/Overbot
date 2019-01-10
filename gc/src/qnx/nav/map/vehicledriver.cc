//
//	vehicledriver.cc  --  the main driving task
//
//	This is the main routine that drives the vehicle.
//
//	J. Nagle
//	Team Overbot
//	December, 2004.
//
//
//	Copyright 2005 by John Nagle
//	999 Woodland Avenue
//	Menlo Park, CA  94025
//
//	This program is free software; you can redistribute it and/or modify
//	it under the terms of the GNU General Public License as published by
//	the Free Software Foundation; either version 2 of the License, or
//	(at your option) any later version.

//	This program is distributed in the hope that it will be useful,
//	but WITHOUT ANY WARRANTY; without even the implied warranty of
//	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//	GNU General Public License for more details.

//	You should have received a copy of the GNU General Public License
//	along with this program; if not, write to the Free Software
//	Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//
#include <exception>
#include "vehicledriver.h"
#include "mapserver.h"
#include "logprint.h"
#include "gpsins_messaging.h"
#include "moveservermsg.h"
#include "algebra3.h"
#include "eulerangle.h"
#include "tuneable.h"
#include "geocoords.h"
#include "timeutil.h"
#include "geomutil.h"
#include "signserver.h"
//
//	Constants
//
const double k_period = 0.100;							// 100ms timed loop
const int k_priority = 14;										// at this priority
//	NOTE -- this steering angle info is also in the move/speed servers and must match.
const Tuneable k_max_steering_angle_deg("STEERINGANGLELIM",25,60,33.0,"Maximum steering angle, degrees");
const float k_max_steering_angle = k_max_steering_angle_deg*(M_PI/180.0);		// convert to radians
const Tuneable k_wheelbase("WHEELBASE",1,3,1.7,"Effective wheelbase, m");	// measured to point between rear axles
const float k_invturnradius = sin(k_max_steering_angle) / k_wheelbase;				// max curvature
//	NOTE - duplicate from mapupdate
const Tuneable k_nogo_cell_roughness_limit("NOGOCELLROUGHNESS",1,30,20,"Minimum roughness of no-go area, cm");
const Tuneable k_veh_length("VEHLENGTH", 2, 4, 3, "Vehicle length, m");				// vehicle length
const Tuneable k_veh_width("VEHLENGTH", 2, 4, 2, "Vehicle width, m");				// vehicle length
const Tuneable k_steering_error_ratio("STEERINGERRORRATIO", 0.01, 1, 0.10, "Steering error, ratio");		// cross track error expected per unit move
const Tuneable k_safe_fusednav_cep("SAFEFUSEDNAVCEP", 0.00, 3.0, 2.0, "Safe fusednav circular error, m");		// cross track error expected per unit move
//
//	Compute timing control - maximum time that drive computation should take.
//
//	If too short, and overrun, we will get timing jitter and move server timeouts.
//	If too long (close to 100ms), reaction time suffers.  
//	Must not exceed 100 ms minus the time required to issue a move command, which is short.
//
const uint64_t k_drive_step_compute_period = 70* (1000000);					// 70ms per cycle 

//
//	Misc. support functions
//
//	halfcirclearclen  -- Arc length of a half circle of a given curvature
//
//	But never more than maxdist.
//	Used for graphics
//
static float halfcirclearclen(float curvature, float maxdist)
{
	if (fabs(curvature) < 0.0001) return(maxdist);													// straight line
	return(std::min(maxdist, float(M_PI/fabs(curvature))));										// half a circle
}

//	
//	Public methods  -- called from outside thread
//
//
//	Constructor
//
VehicleDriver::VehicleDriver(MapServer& owner)	 // constructor
	: TimedLoop(k_period, k_priority),
	m_owner(owner),
	m_moveClientPort("MOVE",k_period),						// client port for sending out move commands
    m_gpsinsClientPort("GPSINS",k_period),					// client port for querying GPS/INS	    
	m_steertimestamp(0),
	m_busy(true),
	m_initialized(false),
	m_distancetoback(0.0),
	m_lastrecoveryaction(recovery_normal),				// not recovering from anything yet
	m_lastrecoverypos(0,0),										// last recovery was here
	m_missioncompleted(false),									// mission completed
	m_lastgpsinserrorstatus(GPSINS_MSG::INITIALIZATION),		// last error status from GPS/INS
	m_timedloopoverruns(0)										// number of timed loop overruns
{
	bzero(&m_lastmovereply, sizeof(m_lastmovereply));	// clear last move reply
}
//
//	Destructor
//
VehicleDriver::~VehicleDriver()
{
}
//
//	initDriving -- initialize NewSteer
//
void VehicleDriver::initDriving()
{
	ost::MutexLock lok(getOwner().getMapLock());	// lock map during steering calc
	//	Set vehicle parameters in steering level's vehicle model.
  	m_VehicleDriver.setVehicleTrackingError(k_steering_error_ratio);
    m_VehicleDriver.setVehicleProperties(k_veh_width, k_veh_length, k_invturnradius);
    m_VehicleDriver.setTerrainRoughnessThreshold(k_nogo_cell_roughness_limit);
	m_steertimestamp = gettimenow();					// start the clock
	m_VehicleDriver.init();										// initialize steering level
	logprintf("Vehicle properties: %1.2fm long, %1.2fm wide, %1.3f 1/r (%1.3fm radius), %1.3f steering error ratio.\n",
		double(k_veh_length), double(k_veh_width), double(k_invturnradius), 
		double(1.0/k_invturnradius), double(k_steering_error_ratio));
	m_lastspeed = 0;												// initial conditions for speed and steering
	m_lastcurvature = 0;
	m_distancetoback = 0;										// not backing up
	m_lastrecoveryaction = recovery_normal;		// not recovering
	m_busy = false;												// not busy
	m_missioncompleted = false;							// not mission complete
}
//
//	stopDriving  -- stop active driving
//
//	Called from outside the thread, in case we want to stop everything. 
//	Currently unused.
//
void VehicleDriver::stopDriving()
{
	Stop();																// stop the timed loop
	////SetFault(Fault::missionstopped);						// stop move server
	while (Looping())												// until timed loop stops
	{	logprintf("Waiting for driving to stop.\n");		
		usleep(100000);											// wait one cycle
	}
	//	No longer running the driving thread
	updateMoveStatus();										// final status update
}
//
//	startDriving --  start active driving
//
//	Called once, at initialization
//
bool VehicleDriver::startDriving()
{
	getOwner().getMap().clearmap();						// NOTE - all map data is lost here.
	initDriving();
	Start();
	return(true);
}
//
//	getStatus  -- get last move status
//
//	This is used primarily for external debug clients that want to find out what's going on.
//	Status retrieved while not driving isn't too useful until we fix the speed server to
//	have a query function, too.
//
void VehicleDriver::getStatus(MapServerMsg::MsgMapQueryReply& status)
{	
	ost::MutexLock lok(m_lock);								// lock map during to protect reply
	//	Get driving status
	status.m_waypointserial = m_VehicleDriver.getWaypointSerial();	// get the waypoint serial number
	//	Get move server status
	if (Looping())													// if the driving thread is active
	{	status.m_movereply = m_lastmovereply;		// copy reply to caller
		return;
	}
	//	Not driving, must ask move server for info
	MoveServerMsg::MsgMoveQuery msg;				// request
	MoveServerMsg::MsgMoveReply movereply;	// reply
	msg.m_msgtype = MoveServerMsg::MsgMoveQuery::k_msgtype;	// set message type
	int stat = m_moveClientPort.MsgSend(msg, movereply);		// send query to move server
	if (stat < 0)														// clear reply if fail
	{	bzero(&movereply, sizeof(movereply)); }
	m_lastmovereply = movereply;
	status.m_movereply = movereply;					// return final move reply
	bzero(&status, sizeof(status));							// all zero when not driving ***TEMP***
}
//
//	Private methods -- called from within thread
//
//
//	code  -- the code executed on each timed loop
//
void VehicleDriver::code()
{	try {
		bool good = driveStep();									// advance one step
		if (!good)
		{	Stop();															// stop the timed loop
			updateMoveStatus();									// final status update
			throw("DRIVING FAILURE - must force shutdown.");	// This is bad, but it's all we have time for.
		}
	}
	catch (const std::exception& except)							// catch exceptions
	{	logprintf("EXCEPTION in driveStep: %s\n", except.what());			// print exception
		sleep(1);															// allow time for output
		throw;
	}
	catch (const char* msg)										// catch exceptions
	{	logprintf("EXCEPTION in driveStep: %s\n", msg);					// print exception
		sleep(1);															// allow time for output
		throw;
	}
	catch (...)
	{	logprintf("UNKNOWN EXCEPTION in driveStep.\n");
		sleep(1);															// allow time for output
		throw;
	}
}
//
//	updateBlindSpot -- update blind spot under the vehicle after a restart
//
//	The LIDAR can see down to the front bumper, but we have to cover the area between the
//	vehicle midpoint and the nearest LIDAR scan line.
//
//	Need to fill a rectangle the size of the vehicle.
//
void VehicleDriver::updateBlindSpot(const vec3& pos, const vec2& fwd, bool fullfill)
{	const vec3 forward(fwd[0], fwd[1], 0);															// forward vector in 3D
	const vec3 right(forward[1], -forward[0],0);													// rightward pointing vector
	float minrange = 2.0;																					// minimum range at which seen
	const float halfwidth = (k_veh_width*0.5)*4.0;												// vehicle halfwidth, halflength
	const float halflength = k_veh_length*0.5*2.5;												// ***TEMP*** bogus oversize dims
	const vec3 p(pos + forward*(k_veh_length*0.5));										// move test rectangle forward slightly
	const vec3 p1 = p + forward*halflength - right*halfwidth;							// left front corner
	const vec3 p2 = p + forward*halflength + right*halfwidth;							// right front corner
	vec3 p4, p3;																									// back of fill area
	if (fullfill)																										// big fill, under vehicle
	{	//	Corners of the rectangle, clockwise.
		p3 = p - forward*halflength + right*halfwidth;											// right rear corner
		p4 = p - forward*halflength - right*halfwidth;											// left rear corner
	} else {																										// small fill, from center of vehicle to beginning of sweep
		const vec3 pback(forward*k_veh_length*0.25);										// move back by this much
		p3 = pos + right*halfwidth - pback;
		p4 = pos - right*halfwidth - pback;
	}
	//	Strict area under vehicle.  if we're on it, it must be OK
	const float qhalfwidth = k_veh_width*0.6;													// vehicle halfwidth, halflength
	const float qhalflength = k_veh_length*0.5;													// actual dimensions
	const vec3 q1 = pos + forward*qhalflength - right*qhalfwidth;						// left front corner
	const vec3 q2 = pos + forward*qhalflength + right*qhalfwidth;						// right front corner
	const vec3 q3 = pos - forward*qhalflength + right*qhalfwidth;						// left front corner
	const vec3 q4 = pos - forward*qhalflength - right*qhalfwidth;						// right front corner
	
	uint32_t cyclestamp = m_owner.getMap().getcyclestamp();							// get current cycle stamp
	m_owner.getMap().setancientstamp(cyclestamp);										// we override any older data
	m_owner.updateMapRectangle(p1, p2, p3, p4, minrange, cyclestamp, false);		// fill unknown area near vehicle as good
	m_owner.updateMapRectangle(q1, q2, q3, q4, minrange, cyclestamp, true); // fill area under vehicle as good
	m_owner.getMap().setancientstamp(cyclestamp);										// anything newer overrides us
}
//
//	requestRunMode  -- get into run mode
//
//	Sends move requests for zero motion until we get run mode.
//
bool VehicleDriver::requestRunMode(bool& busy)
{	
	//	Check for mission completion
	if (m_missioncompleted)																				// if all done
	{	busy = true;
		Fault::Faultcode faultid;																				// fault id
		bool good = commandMove(0, 0, 0, MsgSpeedSet::gear_neutral, busy, faultid);		// command a zero move, go to neutral
		good = good;																							// ignore status
		busy = true;																								// don't drive
		return(true);																								// will try again
	}	
	//	Check that GPS has initialized
	double startspeed, cep;
	vec3 startpos;
	vec2 startforward;
	float pitch, roll;
	bool hasgps = getPosition(startpos, startforward, startspeed, cep, pitch, roll);	// get current vehicle situation
	if (!hasgps)
	{	logprintf("Waiting for good GPS/INS position.\n");	
		////SignDisplayPriority("Waiting for good GPS fix.");										// note problem
		busy = true;
		Fault::Faultcode faultid;																				// fault id
		bool good = commandMove(0, 0, 0, MsgSpeedSet::gear_neutral, busy, faultid);		// command a zero move, go to neutral
		if (!good)
		{	good = handleDrivingFault(faultid);														// try to handle problem
			if (!good) return(false);																			// failed
		}
		busy = true;																								// don't drive now
		return(true);																								// will try again
	}
	//	Start engine if necessary, and shift into gear.
	Fault::Faultcode faultid;																					// fault id
	//	***TEMP*** fix to get proper gear info and set it here.
	MsgSpeedSet::Gear newgear = MsgSpeedSet::gear_low;								// assume low gear (for now)
	if (m_distancetoback > 0) 
	{	logprintf("REVERSE GEAR in request run mode,\n");
		newgear = MsgSpeedSet::gear_reverse;													// if reverse
	}
	bool good = commandMove(0, 0, 0, newgear, busy, faultid);			// command a zero move
	if (!good)
	{	good = handleDrivingFault(faultid);															// try to handle problem
		if (!good) return(false);																				// failed
	}
	//	Check that sensors have initialized
	ost::MutexLock lok(getOwner().getMapLock());												// lock map during steering calc
	LMSmapUpdater& lmssensor(getOwner().getLMSupdater());						// get LMS
	if ((!lmssensor.LMSvalid()) || (!lmssensor.gazeistracking()))						// if LMS not ready
	{	logprintf("Waiting for LMS initialization.\n");												// check that LMS is up
		//	***NEEDS WORK***  doesn't really check that LMS lines are coming in.
		busy = true;																								// waiting
		return(true);	
	}
	//	Once only initialization, after GPS is up and we have a location - fill blind spot under vehicle out to just in front of front bumper.
	if (!m_initialized)
	{	updateBlindSpot(startpos, startforward, false);											// update the blind spot.
		m_initialized = true;																					// first initialization
		////getOwner().getLMSupdater().requestSweep();											// request an initial LIDAR sweep
	}
	if (busy)
	{	logprintf("Waiting for engine start or shifting.\n");										// OK, but not yet ready
		return(true);
	}
	busy = false;																								// success
	return(true);																									// success, but may be busy
}
//
//	driveLog  -- do logging of steering results
//
void VehicleDriver::driveLog(const vec3& startpos3d, const vec2& startforward)
{	vec2 startpos(startpos3d[0], startpos3d[1]);									// use 2D version here
	//	Debug logging support
	const vec2 toright(startforward[1], -startforward[0]);						// toright			
	//	Log goal point as green circle.
	const vec2 goal = m_VehicleDriver.getGoal();									// get current steering goal
	getOwner().getLog().logSteeringGoal(goal);
	//	Log the four corners of the curved wedge in white, for debug
	for (size_t i=0; i < m_VehicleDriver.getWedgePointCount(); i++)
	{	getOwner().getLog().logMiscPoint(m_VehicleDriver.getWedgePoint(i),7,3);	}
	//	Show planned path
	const int k_plan_segments_logged = 15;											// log with some reasonable number of segments
	const float displaypathlength = m_VehicleDriver.getPathLength();	// get length of path
	if (displaypathlength > 0)																	// if valid path
	{	vec2 prevpt = m_VehicleDriver.getPointAlongPath(0);					// get starting point of path
		for (int i=1; i<k_plan_segments_logged; i++)
		{	const float s = float(i) / (k_plan_segments_logged - 1);				// fraction along path
			vec2 pt = m_VehicleDriver.getPointAlongPath(s*displaypathlength);	// end of segment
			getOwner().getLog().logLine(prevpt, pt, 4);								// line segment
			prevpt = pt;																				// for next time	
		}
	}
	//	Log curvature limit arcs
	const float k_limit_arc_display_dist = 20*2;										// draw out to reasonable stopping dist. (m) 
	const uint8_t arccolor = 1;																// blue
	vec2 sink;
	//	Construct two arcs, showing the limits to which we can steer at the current speed.
	float mincurv = parallelcurve(m_VehicleDriver.getmincurv(), -k_veh_width*0.5);
	float maxcurv = parallelcurve(m_VehicleDriver.getmaxcurv(), -k_veh_width*0.5);
	float dist0 = halfcirclearclen(mincurv, k_limit_arc_display_dist)*0.5;
	vec2 ptleftside = startpos-toright*(k_veh_width*0.5);
	vec2 ptrightside = startpos+toright*(k_veh_width*0.5);
	vec2 ptl0 = pointalongarc(ptleftside, startforward, mincurv, dist0, sink);
	getOwner().getLog().logArc(ptleftside, ptl0, mincurv, arccolor);
	float dist1 = halfcirclearclen(maxcurv, k_limit_arc_display_dist)*0.5;
	vec2 ptl1 = pointalongarc(ptrightside, startforward, maxcurv, dist1, sink);
	getOwner().getLog().logArc(ptrightside, ptl1, maxcurv, arccolor);
	//	Log waypoints tried
	const std::vector<PathEndpoint>& endpoints = m_VehicleDriver.getpathendpoints();	// get the path endpoints
	for (size_t i=0;  i< endpoints.size(); i++)											// for endpoints stored for logging
	{	const PathEndpoint& endpoint = endpoints[i];								// this endpoint
			getOwner().getLog().logArrow(endpoint.m_pos, endpoint.m_dir, endpoint.m_color);	// log endpoint
	}
	//	Log road-follower information
	const RoadFollowInfo& road = getOwner().getMap().getroadfollowinfo();		// get road follower info
	if (road.m_confidence > 0)																// if valid
	{	const float k_roadarcdist = 15;														// arc distance for road follower arc
		const float k_roadarccolor = 7;														// white
		vec2 ptl1 = pointalongarc(startpos, startforward, road.m_curvature, k_roadarcdist, sink);
		getOwner().getLog().logArc(startpos, ptl1, road.m_curvature, k_roadarccolor);
	}
}
//
//	driveStep  -- one iteration of the driving cycle
//
//	Runs regularly, every 100ms, controlled by a timed loop. 
//
//	Returning false from this means that driving has failed unrecoverably.
//
//	TODO:	Uncertainty in position is not handled.
//
bool VehicleDriver::driveStep()
{	uint64_t stepstarttimens = gettimenowns();													// time step started
	//	Get into run mode if necessary
	if (m_busy)																									// if busy
	{	bool good = requestRunMode(m_busy);													// try to get run mode
		if (!good) return(false);																				// fails
		if (m_missioncompleted) return(true);														// done, don't do any more driving
	}
	//	Get coordinates from GPS system.  If we can't get a position, we must fault, and can't drive at all now.
	double startspeed, cep;
	float roll, pitch;
	vec3 startpos;																								// starting position
	vec2 startforward;																						// unit vector in forward dir
	bool good = getPosition(startpos, startforward, startspeed, cep, roll, pitch);	// get current vehicle situation
	if (!good)
	{	if (!m_busy)
		{	handleDrivingFault(Fault::noposition);		}											// trouble
		return(true);																								// fails, but recoverable
	}
	//	Check for CPU overrun.  Fault if necessary. We must do this to prevent an infinite loop
	good = checkOverrun();																				// check for overrun
	if (!good)																										// if CPU overrun 
	{	if (!m_busy)
		{	handleDrivingFault(Fault::cpuoverrun);		}											// trouble
		return(true);																								// fails, but recoverable
	}
	float commandedmovedistance = 0;															// begin constructing move
	float recommendedspeed = 0;
	float commandedcurvature = 0;
	int requesteddir = 1;																						// 1=fwd, -1=reverse
	//	Locked section - map and waypoints protected.
	{	ost::MutexLock lok(getOwner().getMapLock());											// lock map during steering calc
		TerrainMap& map(getOwner().getMap());													// access to now-locked map
		//	Get relevant waypoints
		if (getActiveWaypoints().size() == 0)														// if no waypoints
		{	SetFault(Fault::offcourse);
			logprintf("No valid waypoints - can't continue.\n");
			return(false);
		}
		getOwner().getLog().logWaypoints(getActiveWaypoints());						// log the waypoints
		//	Update current info from road follower
		map.updateroadfollowinfo();																		// bring up to date for this cycle
		//	Update fault recovery state
		updateDrivingFault(startpos);																	// tell fault recovery where we are
		//	Call NewSteer to get the next steering command
		double now = gettimenow();																	// get time
		float elapsedtime = now - m_steertimestamp;											// update time
		//	Backing-up hack. Backs up short distances when in trouble.
		if (m_distancetoback > 0) requesteddir = -1;											// request reverse
		logprintf("Calling steer: %spos (%1.2f, %1.2f) headed (%1.3f, %1.3f) at %1.1f m/s 1/r %1.5f  %1.3f secs %d wpts\n",
			(requesteddir < 0 ? "(REVERSE) " : ""),
			startpos[0], startpos[1], startforward[0], startforward[1], m_lastspeed, m_lastcurvature, elapsedtime, getActiveWaypoints().size());	// ***TEMP***
		good = m_VehicleDriver.steer(startpos, startforward, m_lastspeed, m_lastcurvature, pitch, roll, requesteddir, elapsedtime,
			map, getActiveWaypoints(),
			commandedmovedistance, commandedcurvature, recommendedspeed);
		m_steertimestamp = now;				
		// update last steer cycle
		if (!good)
		{	logprintf("Driving control fault: %s.\n",Fault::ErrMsg(m_VehicleDriver.getFault()));	// can't proceed
			commandedmovedistance = 0;															// force a normal stop
			recommendedspeed = 0;
			good = handleDrivingFault(m_VehicleDriver.getFault());						// attempt recovery
			if (!good)
			{	SetFault(m_VehicleDriver.getFault());	}
		}
		//	If "busy", force commanded move to zero, but keep steering during the stop.
		if (m_busy)																								// if busy, force move dist to zero
		{	commandedmovedistance = 0;
			recommendedspeed = 0;
		}
		if (good)
		{	logprintf("Steer: go %1.2f m at %1.2f m/s with curvature %1.5f  %s\n",
				commandedmovedistance, recommendedspeed, commandedcurvature, (m_busy? "BUSY - STOPPING" : ""));	// ***TEMP***
			driveLog(startpos, startforward);															// log the results
		}
		getOwner().getLog().logFrameEnd();														// end of a log frame
		if (!good)
		{	getOwner().getLog().logFlush();															// get good log to very end if trouble
			return(false);																						// fails if bad
		}
	}
	//	Synchronization point -- delay here until specified time. This insures that move commands
	//	are consistently issued 100ms apart. Too much jitter will cause speed server timeouts
	//	or hardware watchdog timeouts.
	uint64_t waituntilns = stepstarttimens + k_drive_step_compute_period;		// wait until this time (ns)
	struct timespec waituntil;																				// wait until this time (as timespec)
	nsec2timespec(&waituntil, waituntilns);														// convert format
	uint64_t stependtimens = gettimenowns();													// get time now
	if (waituntilns < stependtimens)																	// if CPU overrun (took too long to get here)
	{	uint64_t overrunns = stependtimens - waituntilns;									// note length of overrun
		logprintf("CPU OVERRUN in DriveStep. Exceeded %1.4f s limit by %1.4f s.\n",
			k_drive_step_compute_period * 0.000000001, overrunns*0.000000001);		
	}
	clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &waituntil, NULL);			// WAIT until scheduled time, precisely
	//	Finally command the move.
	bool busy;																									// true if busy (starting, paused, etc.)
	MsgSpeedSet::Gear newgear = MsgSpeedSet::gear_low;								// assume low gear (for now)
	if (requesteddir < 0) 
	{	logprintf("REVERSE GEAR\n");
		newgear = MsgSpeedSet::gear_reverse;													// if reverse
	}
	Fault::Faultcode faultid;																					// fault ID returned
	good = commandMove(commandedmovedistance, commandedcurvature, recommendedspeed, newgear, busy, faultid);
	if (!good)
	{	good = handleDrivingFault(faultid);															// attempt recovery
		if (!good)
		{	return(false);	}																					// unable to recover
	}
	if (busy)																										// busy at lower level
	{	m_busy = true;	}																						// must wait for lower level
	return(true);																									// completed one step
}
//
//	getPosition -- get position from GPS/INS server
//
bool VehicleDriver::getPosition(vec3& startpos, vec2& startforward, double& startspeed, double &cep, float& roll, float& pitch)
{	
	GPSINSMsgReq msg;																						// message to send
	msg.m_msgtype = GPSINSMsgReq::k_msgtype;												// set type
	GPSINSMsgRep reply;																					// the reply
	int stat = m_gpsinsClientPort.MsgSend(msg,reply);										// request position fix
	if (stat < 0)																									// if trouble
	{	logprintf("Error from GPSINS server: %s\n", strerror(errno));						// failed
		return(false);																							// fails
	}
	//	Analyze reply. Get basic fields, except position.
	cep = std::max(reply.unc[0], reply.unc[1]);													// circular error 
	float vx = reply.vel[0];
	float vy = reply.vel[1];
	startspeed = sqrt(vx*vx + vy*vy);																// actual speed from GPSINS
	roll = deg2radians(reply.rpy[0]);																	// roll angle (radians)
	pitch = deg2radians(reply.rpy[1]);																// pitch angle
	float yaw = deg2radians(reply.rpy[2]);														// yaw angle (heading)
	startforward = vec2(sin(yaw), cos(yaw));													// as vector
	//	If just came out of initialization, must clear map. Our position has moved and map data is invalid
	if (m_lastgpsinserrorstatus == GPSINS_MSG::INITIALIZATION && reply.err !=  GPSINS_MSG::INITIALIZATION)	
	{	//	Must clear the entire map.
		ost::MutexLock lok(getOwner().getMapLock());											// lock map during steering calc
		TerrainMap& map(getOwner().getMap());													// access to now-locked map
		map.clearmap();																						// clear the entire map, losing all data
		SetFault(Fault::mapcleared);																		// note fault situation
		m_lastgpsinserrorstatus = reply.err;															// update error status from GPSINS/Fusednav
		return(false);																							// force an "obstacle" error
	}
	m_lastgpsinserrorstatus = reply.err;																// update error status from GPSINS/Fusednav
	//	Insist on high-quality Omnistar HP until Fusednav works. Set k_safe_fusednav_cep to about 2 meters when it works.
	bool good = reply.err == GPSINS_MSG::OK;													// GPSINS/Fusednav happy
	bool goodgps =  good && (reply.posType == GPSINS_MSG::OMNISTAR_HP)		// must have Omnistar HP lock
		&& (reply.posStat == GPSINS_MSG::SOL_COMPUTED);								// true if high quality GPS
	if (k_safe_fusednav_cep < 0.01)																	// if we are requiring Omnistar HP
	{
		good = good && goodgps;																		// gps must be good
	} else {																										// if trusting Fusednav position
		good = good && (cep <= k_safe_fusednav_cep);									// good if Fusednav says it is
		if (good && !goodgps)																				// if not good GPS
		{	if (getVerboseLevel() >= 1)	logprintf("DEAD RECKONING in progress.\n");	}	// so note 
	}
	if (!good)
	{	
		logprintf("Lost good GPS lock: %s (%s) (%s) cep %1.2f m.\n", 
			decodeErr(reply.err),																			// OK vs not OK
			decodePosType(reply.posType), decodePosStat(reply.posStat), cep);
		return(false);																							// fails
	}
	//	We have good data. Update.
	//	Convert coords.  Trivial, but has to be done right.
	startpos = XYZfromNED(reply.pos);																// convert coords
#define CROSSCHECK
#ifdef CROSSCHECK	// ***TEMP***
	Vector<3> llhrad(deg2radians(reply.llh[0]), deg2radians(reply.llh[1]), reply.llh[2]);						// convert Lat and Long to radians
	vec3 xyzposcheck = XYZfromLLH(llhrad, getOwner().getAllWaypoints().getOrigin());		// recalculate position
	if ((xyzposcheck-startpos).length() > 1.0)														// if out of sync
	{	logprintf("LLH to XYZ error: At Lat. %1.7f Long. %1.7f, GPSINS XYZ=(%1.2f %1.2f %1.2f)  Check XYZ=(%1.2f %1.2f %1.2f)\n",
			reply.llh[0], reply.llh[1], startpos[0], startpos[1], startpos[2], 
			xyzposcheck[0], xyzposcheck[1], xyzposcheck[2]);
	}
#endif // CROSSCHECK
	//	Compute fix for log
	ost::MutexLock lok(getOwner().getMapLock());												// lock map during update
	//	Update center position of map
	int centerix = getOwner().getMap().coordtocell(startpos[0]);						// compute new center position
	int centeriy = getOwner().getMap().coordtocell(startpos[1]);
	mat4 vehpose;
	posefromfix(reply,vehpose);																		// compute a pose matrix
	//	Must output log message before scrolling map, or map updates will be out of synch.
	getOwner().getLog().logVehiclePosition(vehpose, centerix,centeriy, reply.timestamp, reply.llh[0], reply.llh[1], startspeed);	// log it
	getOwner().getMap().setmapcentercell(centerix, centeriy);							// scroll map to keep vehicle at center
	getOwner().getPoses().addPose(vehpose, cep, reply. timestamp);				// report pose to higher level
	return(true);																									// good fix
}
//
//	initPosition -- call position update once to set initial position of vehicle and center the map.
//
bool VehicleDriver::initPosition()
{	double startspeed, cep;
	float pitch, roll;
	vec3 startpos; 
	vec2 startforward;
	return(getPosition(startpos, startforward, startspeed, cep, pitch, roll));		// get current vehicle situation
}
//
//	commandMove  -- tell the vehicle to move
//
//	Sends a message to the move server.
//
//	If this returns false, there's a problem with the vehicle and "faultid" is set.
//
bool VehicleDriver::commandMove(float dist, float curvature, float speed, MsgSpeedSet::Gear desiredgear, bool& busy,
	Fault::Faultcode& faultid)
{	//	Check for OpenSteer bug (obsolete since NewSteer replaced OpenSteer)
	faultid = Fault::none;																					// no fault yet
	if (!finite(curvature) || !finite(dist) || !finite(speed))									// if invalid values
	{	logprintf("commandMove given invalid values: dist=%1.2f m  curvature=%1.2f  speed=%1.2f m/s.\n",
			dist, curvature, speed);
		SetFault(Fault::internalerror);																// steering bug  - bogus result
		return(false);
	}
	MoveServerMsg::MsgMove msg;																// build the move
	msg.m_msgtype = MoveServerMsg::MsgMove::k_msgtype;						// set type
	msg.m_distance = dist;																				// distance to advance
	msg.m_curvature = curvature;																	// 1/radius of curvature
	msg.m_speed = speed;																			// max allowed speed
	msg.m_gear = desiredgear;																		// desired gear
	MoveServerMsg::MsgMoveReply reply;														// reply area
	int stat = m_moveClientPort.MsgSend(msg,reply);										// send the move
	if (stat < 0)																								// if can't communicate
	{	logprintf("Error from move server: %s\n", strerror(errno));					// failed
		SetFault(Fault::networkerror);																// network problem, probably
		return(false);																						// fails
	}
	//	Valid reply, analyze reply
	{	ost::MutexLock lok(m_lock);																	// briefly lock during update
		m_lastmovereply = reply;																		// save for remote access
	}
	busy = reply.m_speedreply.m_hint == MsgSpeedSetReply::hint_busy;		// test if busy
	faultid = reply.m_speedreply.m_lastfault;							// get fault code
	if (faultid != Fault::none)																			// if fault
	{	logprintf("Fault reported: %s\n", Fault::ErrMsg(faultid));							// report
		return(false);																						// fails
	}
	//	***MAY NEED MORE CHECKS***
	m_lastspeed = reply.m_speedreply.m_speed;											// save last speed
	m_lastcurvature = reply.m_speedreply.m_curvature;								// save last curvature
	return(true);																								// success
}
//
//	SetFault  -- set and report a fault.
//
//	This will stop the vehicle, but there is recovery.
//
void VehicleDriver::SetFault(Fault::Faultcode newfault)
{
	MoveServerMsg::MsgMoveStop stopmsg;
	stopmsg.m_msgtype = MoveServerMsg::MsgMoveStop::k_msgtype;			// set type
	stopmsg.m_fault = newfault;																		// set the fault code
	logprintf("FAULT: %s\n", Fault::ErrMsg(newfault));										// display msg
	int stat = m_moveClientPort.MsgSend(stopmsg);										// send it
	if (stat < 0)																								// if trouble
	{	logprintf("Error from move server: %s\n", strerror(errno));					// failed
	}
	//	Read back final status of move server.
	updateMoveStatus();
}
//
//	updateMoveStatus  -- updates status read by user interface program
//
bool VehicleDriver::updateMoveStatus()
{	
	//	Read back status of move server.
	MoveServerMsg::MsgMoveQuery msg;				// request
	msg.m_msgtype = MoveServerMsg::MsgMoveQuery::k_msgtype;	// set message type
	int stat = m_moveClientPort.MsgSend(msg, m_lastmovereply);		// send query to move server
	if (stat < 0)														// clear reply if fail
	{	bzero(&m_lastmovereply, sizeof(m_lastmovereply));
		m_lastmovereply.m_speedreply.m_lastfault = Fault::none;		// no good fault info
	}
	if (m_lastmovereply.m_speedreply.m_lastfault == Fault::none)	// if no fault stored
	{	m_lastmovereply.m_speedreply.m_lastfault = m_VehicleDriver.getFault(); }		// use fault from this level
	return(stat >0);													// if success
}
//
//	getVerboseLevel  -- true if verbose mode
//
int VehicleDriver::getVerboseLevel()
{	return(getOwner().getVerboseLevel()); }				// if verbose mode
//
//	getActiveWaypoints  -- parent object owns these. Locked by map lock.
//
const ActiveWaypoints& VehicleDriver::getActiveWaypoints() const
{ return(m_owner.getMap().getActiveWaypoints()); }
//
//	getCurvature  -- get latest turning curvature
//
float VehicleDriver::getCurvature()
{	ost::MutexLock lok(m_lock);
	return(m_lastcurvature);								// return last curvature value
}
//
//	setVerboseLevel  -- set verbosity level
//
void VehicleDriver::setVerboseLevel(int lev)
{	m_VehicleDriver.setVerboseLevel(lev);	}
//
//	checkOverrun  -- check for CPU overrun
//
//	If we take too long making it through the timed loop, we will go compute-bound and lock up the system. 
//	So this is serious, and we have to fault. This is recoverable.
//
bool VehicleDriver::checkOverrun()
{	if (OverrunCountGet() != m_timedloopoverruns)		// if timed loop CPU overrun
	{	m_timedloopoverruns = OverrunCountGet();		// reset count
		return(false);														// trouble
	}
	return(true);																// no overrun
}



