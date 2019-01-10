//
//	newsteer.cpp  -- steering controller.
//
//	Replacement for OpenSteer
//
//	John Nagle
//	Team Overbot
//	February, 2005
//
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
//
//	class NewSteer  -- one steering controller
//
//
//	Waypoint following with obstacle avoidance
//
//
#include "geomutil.h"
#include "algebra3.h"
#include "newsteer.h"
#include "tuneable.h"
#include "waypoints.h"
#include "waypointutil.h"
#include "curvedwedge.h"
#include "dynamicsutil.h"
#include "timeutil.h"
#include "arcpath.h"
#include "splinepath.h"
#include "logprint.h"
//
//	Constants
//
const Tuneable k_min_speed("MINSPEED", 0.1, 0.5, 0.3, "Minimum forward speed, even if trouble (m/sec)");
const Tuneable k_max_reverse_speed("MAXREVERSESPEED", 0.2, 1.0, 0.5, "Maximum speed in reverse (m/sec)");
const Tuneable k_max_speed_increase("MAXSPEEDINCREASE", 0.1, 5.0, 2.0, "Maximum speed increase per update (m/sec)");
const Tuneable k_min_move("MINMOVE",0.1, 3.0, 2.0, "Minimum move dist (m)");
const Tuneable k_goal_dist("GOALDIST",1, 40, 15, "Distance to goal point (m)");
const Tuneable k_stopping_distance_multiplier("STOPPINGDISTMULT", 1.0, 2.5, 1.75, "Multiply stopping distance by this as safety margin");
const Tuneable k_curvature_multiplier("CURVATUREMULT", 1.0, 3.0, 2.0, "Multiply curvature by this when computing speed limit");
const Tuneable k_offcourse_tolerance("OFFCOURSETOLERANCE", 0.0, 3.5, 0.0, "Allowed off-course tolerance before shutdown.");
const Tuneable k_initial_offcourse_tolerance("INITIALOFFCOURSETOLERANCE", 0.0, 25, 5.0, "Allowed off-course tolerance for first waypoint.");

//
//	Constructor
//
NewSteer::NewSteer()
        : m_outpathendpos(vec2(k_NaN,k_NaN)),										// used for hysteresys
        m_verboselevel(0)
{
	init();																									// clear state
}
//
//	Destructor
//
NewSteer::~NewSteer()
{
}
//
//	init -- clear state
//
void NewSteer::init()
{	m_currentwaypoint.m_serial = -1;														// dummy previous waypoint
	m_currentwaypoint.m_turnnum = 0;
	m_outwaypoint = -1;
}
//
//	calcSafeSpeedForCurvature  -- calculate safe speed for a given curvature
//
//	Used to slow down for turns ahead, not to brake in turns.
//
float NewSteer::calcSafeSpeedForCurvature(float curvature)
{
	return(::calcSafeSpeedForCurvature(m_inpitch, m_inroll, curvature));
}
//
//	steer  -- one steering update cycle
//
bool NewSteer::steer (const vec3& startPos, const vec2& startForward,	// start position, direction
        const float startSpeed,									// current vehicle speed in m/sec
        const float startCurvature,							// current vehicle steering position, as 1/turn radius
        const float startPitch, const float startRoll,		// current pitch, roll
        const int startDir,											// 1=fwd, -1=reverse
        // time since last steer update (in seconds)
        const float elapsedTime,

        // map of nearby terrain traversability
        const TerrainMap& map,

        // relevant waypoints (only the nearby ones)
        const ActiveWaypoints& waypoints,

        // output: OK to move this distance
        float& safeMoveDistance,

        // output: with this 1/turning radius
        // (right turn > 0)
        float& recommendedCurvature,

        // output: desired speed
        float& recommendedSpeed)
{
    //	Update system state
    updatestate(startPos, startForward, startSpeed, startCurvature, elapsedTime, startPitch, startRoll, startDir);
    //	Do the driving
    bool good = dodriving(waypoints, map, safeMoveDistance, recommendedCurvature, recommendedSpeed);
    recommendedCurvature *= startDir;		// invert steering if moving backwards
	return(good);
}
//
//	updatestate -- set state, for later access
//
void NewSteer::updatestate(const vec3& startPos, const vec2& startForward,
                           float startspeed, float startCurvature, float elapsedTime, float pitchradians, float rollradians, int dir)
{
    m_inposition = vec2(startPos[0], startPos[1]);	// incoming position
    m_inelev = startPos[2];								// incoming elevation
    m_inforward = startForward*dir;				// incoming direction
    m_inspeed = startspeed;							// incoming speed
    m_incurvature = startCurvature*dir;			// incoming curvature
    m_intimestep = elapsedTime;					// incoming elapsed time
    m_outfault = Fault::none;							// no fault yet
    m_intimestep = std::max(m_intimestep,0.010f);	// avoid problems with zero timestep on first call.
    m_inpitch = pitchradians;							// incoming vehicle pitch
    m_inroll = rollradians;								// incoming vehicle roll
    m_indir = dir;
    m_outcollisionavoidcurv = 0;					// no collision avoidance arc yet
    m_outcollisionavoiddist = 0;						// no collision avoidance arc yet (this is a debug output)
    m_outpathendpoints.clear();						// clear path end points
    m_outwaypoint = -1;								// no waypoint number yet
    //	Calculated values
    // calculate allowed curvature limits in present situation
	calcSafeCurvatureLimits(m_inspeed, m_inpitch, m_inroll, m_maxcurvature, m_outmincurv, m_outmaxcurv);
	calcStoppingDistance(m_inspeed, m_inpitch, m_inroll, m_outstoppingdist);		// calculate stopping distance for current speed
	m_outstoppingdist *= k_stopping_distance_multiplier;										// apply stopping distance safety margin
	if (getVerboseLevel() >= 1)												// somewhat verbose
	{	logprintf("Allowed curvature %1.4f .. %1.4f, stopping distance %1.2f m.\n",
		getmincurv(), getmaxcurv(), getstoppingdist());	
	}
}
//
//	setFault -- report a fault
//
//	Caller can read this
//
void NewSteer::setFault(Fault::Faultcode faultid, const char* msg)
{
	if (m_outfault == Fault::none)								// if first fault
	{	m_outfault = faultid;			}								// save fault id
	logprintf("Driving fault: %s %s\n",
		Fault::ErrMsg(faultid), msg);
}

//
//	getGoalPointOnCenterline  --  generate a goal point on the centerline
//
//	This is the initial goal point we're chasing.  It may be modified
//	later for obstacle avoidance.
//
bool NewSteer::getGoalPointOnCenterline(const WaypointTriple& wp,
                                        bool& inturn,
                                        double distahead, vec2& goalpt)
{
	distahead = getMinGoalDist(distahead);									// apply minimum goal distance check
	vec2 p0(wp[0].m_x, wp[0].m_y);												// first point
	vec2 p1(wp[1].m_x, wp[1].m_y);												// second point
 	double U;																					// fraction along waypoint
	vec2 intersect;
	inturn = false;																			// assume not in turn
	if ((p1-p0).length2() < 0.00001)												// identical points, fails
	{	return(false);	}
	pointtolinesegmentdistance(m_inposition, p0, p1, U, intersect);// calc nearest point on first waypoint segment centerline
    goalpt = pointalongline(intersect, p0, p1, distahead); 				// project ahead on line
    float width = std::min(wp[0].m_width,wp[1].m_width);				// smallest width involved in turn
    vec2 arcp0(pointalongline(p1,p0,p1,-width*0.5));						// entry point of turn arc near point p1
    double goaldistpastend = (goalpt-p0).length() - (arcp0-p0).length() ;	// goal distance past beginning of turn arc
    if (goaldistpastend < 0) return(true);										// if before turn arc, done.
    //	Within turn. Follow curve, unless goal is past end of turn section.
    //	BUG: The goal distance past the beginning of the turn is not correct when the vehicle itself is in the
    //	turn.
    inturn = true;																			// in a turn section
    if (nullwaypoint(wp[2])) 															// if null waypoint
    {																								// no third waypoint, presumably end of path
    	if (distanceoutsidewaypoint(goalpt, wp[0], wp[1]) < 0)		// if inside waypoint
    	{	return(true);		}																// still inside waypoint, keep going to end
    	goalpt = p1;																			// Near end of course, go for last waypoint center
      	if ((distanceoutsidewaypoint(goalpt, wp[0], wp[1]) < 0)		// if inside waypoint
      	&& ((m_inposition - goalpt).length() > m_vehicledim[1]))		// and haven't reached goal
    	{	return(true);		}																// still inside waypoint, keep going to end  	
    	return(false);																		// end of mission - done.
    }
    vec2 p2(wp[2].m_x, wp[2].m_y);												// third point
	vec2 arcp1(pointalongline(p1,p1,p2,width*0.5));						// exit point of turn arc
	//	Compute position along arc in turn
	float arccurvature;
	vec2 entryforward(p1-p0);														// forward at turn entry
	entryforward.normalize();															// must normalize
	bool good = tangentarcthroughpoints(arcp0, entryforward, arcp1, arccurvature);
	if (!good)
	{	logprintf("Can't find a suitable arc for the turn.\n");					// should not happen unless very narrow
		return(false);
	}
	////logprintf("Turn arc: curvature= %1.4f arcp0=(%1.2f.%1.2f)  arcp1=(%1.2f,%1.2f)  entryforward=(%1.2f,%1.2f)\n",
	////	arccurvature, arcp0[0], arcp0[1], arcp1[0], arcp1[1], entryforward[0], entryforward[1]);	// ***TEMP***
	double turnlength = arclength(arcp0, arcp1, arccurvature);		// length through turn
	double goaldistpastarc = goaldistpastend - turnlength;			// distance past turn arc
    if (goaldistpastarc > 0)																// we are into next segment
    { 	goaldistpastarc = std::min(goaldistpastarc, (arcp1-p2).length());	// 	 running through very short segments
    	goalpt = pointalongline(arcp1,arcp1,p2, goaldistpastarc);	// project along next segment
   		return(true);																			// success
   	}
	//	Goal point is in arc.  Need circular interpolation
	vec2 endforward;																		// end direction at arc end
	goalpt = pointalongarc(arcp0, entryforward, arccurvature, goaldistpastend, endforward);	// project along arc
	////logprintf("Turn goal: (%1.2f, %1.2f) dist %1.2fm\n", goalpt[0], goalpt[1], goaldistpastend);	// ***TEMP***
    return(true);
}
//
//	adjustGoalPointForBoundaries  --  generate a goal point on the centerline
//
//	This is the initial goal point we're chasing.  It may be modified
//	later for obstacle avoidance.
//
//	Maxspeed comes in at the speed we'd like to go, and may be reduced.
//
bool NewSteer::adjustGoalPointForBoundaries(const WaypointTriple& wp, double distahead, vec2& goalpt, float& maxspeed)
{
    uint64_t starttime = gettimenowns();									// for performance check
	//	First try to get to the centerline.  Usually this works.
	bool good = getGoodGoalPointOnCenterline(wp, distahead, goalpt, maxspeed);
	uint64_t elapsed = gettimenowns() - starttime;				// elapsed time, ns
	if (elapsed > 2000000)													// if greater than 2ms
	{	logprintf("getGoodGoalPointOnCenterline took too long: %1.6f seconds.\n", elapsed*0.000000001);	}
	if (good) 
	{		return(good);
	}
	logprintf("Can't get to centerline in one move.\n");
	//	Backup algorithm. We generally don't need this unless we have an understeer problem.
	good = getGoodGoalPointNearCenterline(wp, distahead, goalpt, maxspeed);
	if (!good)
	{	logprintf("Can't get to centerline at all.\n");	}
	return(good);
}
//
//	steerToGoalPoint  -- generate curvature to get us to the goal point
//
//	This is pure geometry.  The output curvature is the inverse of the
//	radius of the circle tangent to the current direction and passing
//	through the goal point.
//
//	We're always chasing the goal point, but we never get there.
//	Old version - becoming obsolete
//
bool NewSteer::steerToGoalPoint(const vec2& goalpt, float& curvature)
{
	//	Desired path is an arc through the current vehicle position, tangent to the current
	//	direction of travel, and through the goal point. Calculate curvature.
	return(tangentarcthroughpoints(m_inposition, m_inforward, goalpt, curvature));
}
//
//	steerToPath  -- steer to follow indicated path
//
//	Currently, we always take the initial curvature of the generated path.
//	This is meaningful for arcs and dual arcs.
//	But we need to be careful that the path doesn't have some wierd initial curvature due to
//	a boundary condition.
//
//	Tries to compute the curvature that will hit a point one vehicle length out. This compensates for
//	initial S-curve issues.
//
//	Must return a best-effort curvature value even if it returns false, so that steering during fault stops is sane.
//
bool NewSteer::steerToPath(const CurvedPath& path, float& curvature)
{
	const float k_steer_to_path_vehicle_lengths = 2.0;			// steer to this many vehicle lengths out
	double curvdist = getSteeringDwellDistance() + k_steer_to_path_vehicle_lengths*m_vehicledim[1];	// get curvature from this far out
	curvdist = std::min(curvdist, path.getlength());					// don't go off end of path
#ifdef OBSOLETE
	float simplecurvature = path.getsteeringcurvature();						// return curvature at a reasonable place to get steering
	curvature = path.getcurvatureat(curvdist);						// get curvature further out
	if (curvature != simplecurvature)										// trouble
	{	logprintf("steerToPath: curved path at %1.2f m has curvature %1.4f vs simple curv of %1.4f\n",
			curvdist, curvature,simplecurvature);
	}
#endif // OBSOLETE
	//	Desired path is an arc through the current vehicle position, tangent to the current
	//	direction of travel, and through the specified point on the path. Calculate curvature.
	vec2 goalpt, endforward;													// get goal point we're trying to steer to
	bool good = path.pointalongpath(curvdist, goalpt, endforward);	// get point along path
	if (!good)																			// probably has U-turn or some such stupidity
	{	logprintf("Can't get point %1.2f m along path.\n",curvdist);	// should not happen
		return(false);
	}
	good = tangentarcthroughpoints(m_inposition, m_inforward, goalpt, curvature);
	if (!good)																			// probably has U-turn or some such stupidity
	{	logprintf("Can't construct a steering angle for this path.\n");
		return(false);
	}
	if (curvature > getmaxcurv() || curvature < getmincurv())	// limit to safe limits
	{	logprintf("ERROR: commanded curvature %1.4f outside safe limits [%1.4f .. %1.4f].\n",
			curvature, getmincurv(), getmaxcurv());
		curvature = std::max(std::min(curvature, getmaxcurv()), getmincurv());		// bound during fault stops
		////return(false);															// check curvature limits
	}
	return(true);
}
//
//	checkOnCourse -- check that vehicle is on course
//
//	Check that we are still on the course.  Also patches course slightly for recovery and start.
//
//	Sets a fault and returns false if any problem.
//
bool NewSteer::checkOnCourse(WaypointTriple& wp, bool& offcourserecovery)
{	offcourserecovery = false;											// not doing off-course recovery
    double outsidedist = distanceoutsidewaypoints(m_inposition, wp);	// how far are we inside the waypoint?
    ////logprintf("Outside waypoint #%d by %1.2f m.\n", wp[0].m_serial, outsidedist);	// ***TEMP***
    //	Outside distance is negative when inside. Negative is good
    float offcourse = outsidedist + m_vehicledim[0]*0.5;	// distance off course
    if (offcourse <= 0) 														// completely on course, no problem
    {	//	Check for mission completion.
    	if (!nullwaypoint(wp[2])) return(true);						// more waypoints, not done yet
    	if (m_indir < 0) return(true);										// can't be done if in reverse.
    	vec2 pend(wp[1].m_x, wp[1].m_y);							// the last waypoint
    	double disttoend = (m_inposition-pend).length();		// distance to the last waypoint
    	if (disttoend < wp[0].m_width)									// if we're inside the final circle
    	{	setFault(Fault::done,"Mission completed");			// mission complete. Done.
    		return(false);														// done
    	}
    	return(true);
    }
    //	Check for special cases - just slightly off course, and headed for first waypoint.
    if ((offcourse < k_offcourse_tolerance)							// off course but within tolerance
    	|| (wp[0].m_serial == 1 && offcourse < k_initial_offcourse_tolerance)) // first waypoint case
    {	
    	double extrawidth = offcourse*2.0 + m_vehicledim[0];	// allow extra waypoint width
    	logprintf("Outside waypoint #%d by %1.2f m but recoverable.  Adding %1.2f m to waypoint width.\n", 
    		wp[0].m_serial, offcourse, extrawidth);					// note problem
    	wp[0].m_width += extrawidth;									// add extra width
    	if (wp[0].m_serial != 1)												// if not special case for starting outside first waypoint
    	{	offcourserecovery = true;	}								// note off-course recovery mode (very slow)
		return(true);																// go for it
	}
	//	There's a false alarm problem here.  If the vehicle is very near an acute corner, it can be reported
    //	as off course when it isn't.  So we check eight points around the edges of the vehicle.
    logprintf("Very near edge of course. Checking corners of vehicle.\n");
    offcourse = 0;
    vec2 side(m_inforward[1], -m_inforward[0]);				// sideward pointing vector
    //	Test all four corners and the middle of each face of the vehicle rectangle.
    //	Ignore trailing half of vehicle. to avoid some problems at tight waypoint transitions
    //	and so we can back out of an off-course problem.
    //	Note that "m_inforward" points in the direction of travel, which may be reverse.
    for (int i=-1; i<=1; i++)													// left, middle, right
    {	for (int j=0; j<=1; j++)												// not rear, but front and middle
    	{	////if (j != 0)																// if front or rear test
    		////{	if (j == m_indir) continue; }								// ignore trailing end of vehicle
    		vec2 corneroffset(side*(m_vehicledim[0]*0.5*i)+m_inforward*(m_vehicledim[1]*0.5*j));
    		float outdist = distanceoutsidewaypoints(m_inposition+corneroffset, wp);
    		offcourse = std::max(offcourse, outdist);				// get worst distance
    	}
    }
    if (offcourse <= k_offcourse_tolerance)									// check passed
    {	logprintf("Leading corners all on course within tolerance of %1.2fm; closest distance %1.2f m.\n", 
    		double(k_offcourse_tolerance),-offcourse); 
    	return(true);																// on course, no problem
    }
	char s[100];																	// for msg
    snprintf(s,sizeof(s),"by %1.2f m", offcourse);
    setFault(Fault::offcourse,s);											// report fault
    return(false);																// fails
}
//
//	calcSafeLimits  -- calculate speed safe for this condition
//
bool NewSteer::calcSafeLimits(const WaypointTriple& wp,
                              double distin, float inspeed, bool inturn, double curvature, float& distance, float& speed)
{
	distance = distin;																		// proposed distance to move
    if (distance < k_min_move)														// if move too short
    {	setFault(Fault::obstacle, "Move too small");							// stop BEFORE hitting obstacle. Allows useful rescan.
    	logprintf("Move of %1.2f m is too short.\n", distance);			
		return(false);
	}
   	float waypointspeed = wp[0].m_speedlimit;								// speed limit from waypoint
   		//	***Get speed from proper waypoint, which may not be the first one.
  	float distspeed = calcSafeSpeedForDistance(m_inpitch, distance / k_stopping_distance_multiplier);	// apply speed limit based on clear move ahead
  	float prevspeed = m_inspeed + k_max_speed_increase;			// avoid jackrabbit starts
  	float curvspeed = calcSafeSpeedForCurvature(curvature*k_curvature_multiplier);	// we need to get down to this speed

  	//	Desired speed is the mininum of those five limits
  	speed = std::min(std::min(std::min(std::min(waypointspeed, distspeed) , inspeed), prevspeed), curvspeed);
  	if (m_indir < 0) speed = std::min(speed, float(k_max_reverse_speed));		// reverse is restricted to slow speed operation
  	speed = std::max(speed, float(k_min_speed));							// insure some forward progress
  	if (getVerboseLevel() >= 1)														// somewhat verbose
  	{	if (m_indir < 0) 																		// if in reverse gear
  		{	logprintf("Speed limited to %1.2f m/s in reverse.\n", speed); }
  		else
  		{	logprintf("Speed limits: in %1.2f  waypoint %1.2f   stoppingdist %1.2f  prevspeed %1.2f curvature %1.2f --> %1.2f\n",
		  		inspeed, waypointspeed, distspeed, prevspeed, curvspeed, speed);		
		 }
  	}
    return(true);																				// success
}
//
//	dodriving  -- main driving operation
//
bool NewSteer::dodriving(const ActiveWaypoints& waypoints, const TerrainMap& map,
                         float& safeMoveDistance, float& recommendedCurvature, float& recommendedSpeed)
{
	//	Defautl curvature and speed, in case of faults.
	recommendedCurvature = m_incurvature;									// stay with last steering command
	recommendedSpeed = 0;																// zero speed
	safeMoveDistance = 0;																	// zero move
    //	Get waypoints of interest - current and next waypoint segment
    WaypointTriple wp;																		// relevant waypoints
    bool good = getRelevantSegmentPair(waypoints, wp);				// get waypoints
    if (!good)
    {
		setFault(Fault::offcourse, "Unable to get waypoint segment pair");
        return(false);						// off course, or done.
    }
    m_currentwaypoint = wp[0];														// save current waypoint hint 
    //	Check that we are still on the course.  Also patches course slightly for recovery and start
    bool offcourserecovery = false;
    good = checkOnCourse(wp, offcourserecovery);							// check on course. If recovering, offcourserecovery is set
    if (!good)
    {	return(false);																			// fails
    }
    //	Get the base goal point, which is on the centerline.
    //	In future, we will treat the goal point as a particle and displace
    //	it from this point if we have to.
    bool inturn;								// true if in turn.
    good = getGoalPointOnCenterline(wp, inturn, k_goal_dist, m_outgoal);
    if (!good)
    {
		setFault(Fault::offcourse,"Unable to get goal point");
        return(false);						// off course, or done.
    }
	//	Construct a path based on obstacles and boundaries
    float curvspeed = wp[0].m_speedlimit;											// requested speed, from waypoints
    float movedist;																				// final move distance
	bool goodpath = constructPath(wp, map, m_outgoal, m_outpath, movedist, curvspeed);		// build path around obstacles
    //	Steer based on the path, if it is at all valid.
    //	Path generation tries to generate the best possible path, even if calling for an emergency stop.
    //	If there's no valid path, we fault and continue with the last good steering command.
    //	We have to do this or steering gets stupid during emergencies.
    if (m_outpath.getvalid())																// if usable path, even if in fault stop
    {	good = steerToPath(m_outpath, recommendedCurvature);		// compute desired steering command
	    if (!good)
	    {
			setFault(Fault::drivingfault,"Unable to steer to path");
        	return(false);																		// off course, or done.
        }
    }
	if (!goodpath)																				// if constructed a good path
    {	setFault(Fault::drivingfault,"Unable to find a usable path");
        return(false);																			// trouble
    }
    m_outpathendpos = m_outpath.getendpos();								// save output position for next time
    //	Speed and distance safety limits.
    good = calcSafeLimits(wp, movedist, curvspeed, inturn, recommendedCurvature,	// calculate speed and stopping distance
                          safeMoveDistance, recommendedSpeed);
    if (!good)
    {
        setFault(Fault::drivingfault,"Unable to generate safe speed/distance");
        return(false);						// off course, or done.
    }
    if (offcourserecovery)																	// if recoverying from off course problem
    {	recommendedSpeed = std::min(recommendedSpeed, float(k_min_speed));	} // cut speed way down
    return(true);							// success
}
