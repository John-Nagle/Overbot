//
//	newsteer.h  -- steering controller.
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
#ifndef NEWSTEER_H
#define NEWSTEER_H
#include <vector>
#include "algebra3.h"
#include "waypoints.h"
#include "faultcodes.h"
#include "splinepath.h"
#include "scurvepath.h"
#include "nan.h"
//
//	Forward declarations
//
class TerrainMap;
class CurvedPath;
class ImpingementInfo;
class ImpingementGroup;

class WaypointTriple;										// forward, used internally only
//
const size_t k_wedgepoint_count = 6;			// this many wedge point markers saved for debug
//
//
//	class PathEndpoint  -- endpoint of a path
//
//	Used for debug output
//
struct PathEndpoint {
	vec2 m_pos;																	// position
	vec2 m_dir;																	// direction
	uint8_t	m_color;															// 3-bit RGB
};
//
//	class NewSteer  -- one steering controller
//
//
class NewSteer
{
private:
    //	Permanent state.
    vec2	m_vehicledim;					// width, length
    float	m_maxcurvature;				// tightest curvature vehicle can do
    float	m_trackingerrratio;			// how good is steering?
    float	m_steeringrate;					// max curvature change per unit time
    float	m_terrainthreshold;			// allowed terrain roughness
    //	State at last entry.  Updated every time.
    vec2	m_inposition;						// current vehicle position
    vec2	m_inforward;						// vehicle direction (unit vector)
    float m_inelev;							// current vehicle elevation
    float	m_inspeed;						// vehicle speed
    float	m_incurvature;					// input curvature
    float	m_inpitch;							// input pitch (radians, + is uphill)
    float	m_inroll;							// input roll (radians, + is right)
    int m_indir;								// 1=forward, -1= reverse
    float	m_intimestep;					// time since last update
    //	Hints.  May be wrong.
   	Waypoint	m_currentwaypoint;	// where we are now, maybe
    //	Output state from last cycle
    vec2	m_outgoal;						// goal at end of last cycle
    float m_outmincurv;					// minimum allowed curvature at this speed
   	float m_outmaxcurv;					// maximum allowed curvature at this speed
   	float m_outstoppingdist;			// stopping distance at current speed
   	vec2 m_outpathendpos;			// endpoint of last path taken
   	///SplinePath m_outpath;				// last path generated and driven
   	SCurvePath m_outpath;				// last path generated and driven
    Fault::Faultcode m_outfault;		// fault status at end of last cycle
    vec2	m_outwedgepoints[k_wedgepoint_count];	// wedge points last examined, for debug
    std::vector<PathEndpoint> m_outpathendpoints;	// trial endpoints last examined, for debug
    float m_outcollisionavoidcurv;	// arc to collision avoidance goal point, for debug
    float	m_outcollisionavoiddist;	// distance to above, zero if none.
    int		m_outwaypoint;				// waypoint number we are currently in, for debug
    //	Precaulculate data
   	std::vector<float>	m_curvatureschedule;	// list of curvatures to try, relative to current position
   	//	Debug support
   	int m_verboselevel;					// 0=quiet, 1=per-cycle messages, 2=within-cycle messages
public:

    NewSteer();								// constructor
    virtual ~NewSteer();					// destructor

    // cross-track error < forward distance*ratio
    void setVehicleTrackingError (float ratio)
    {
        m_trackingerrratio = ratio;
    }
    void setTerrainRoughnessThreshold(float roughness)				// allowed roughness, checked for marginal terrain (yellow)
    {	m_terrainthreshold = roughness; }

	void setVerboseLevel(int lev) {	m_verboselevel = lev; }		// message control
	int getVerboseLevel() const { return(m_verboselevel); }
    void init ();							// reinitialize any cached state

	//	Vehicle properties needed to make steering decisions
	//	More may be added
	void setVehicleProperties (float x, float y, float invturnradius, float locktolocksecs = 5.0)
    {
        m_vehicledim = vec2(x,y);					// vehicle dimensions
        m_maxcurvature = invturnradius;		// how tight can we turn?
        m_steeringrate = 2.0*m_maxcurvature / locktolocksecs;	// how fast can we turn
    }
    
    // Execute one STEER update: given current vehicle state (global
    // position, heading, speed), elapsed time since last steer update,
    // current terrain traversability map, and nearby waypoints -- this
    // function returns:
    //
    //   (0) a bool indicating successful driving, returns false if stuck
    //   (1) distance vehicle can safely move along its (curved) path
    //   (2) the steering controller's recommended path curvature
    //   (3) the steering controller's recommended speed
   	//
	bool steer (const vec3& startPos, const vec2& startForward,	// start position, direction
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
        float& recommendedSpeed);

    //	Debug output
    vec2& getGoal()
    {
        return(m_outgoal);
    }			// get goal, for debug
    
    Fault::Faultcode getFault()															// why we returned false
    {	return(m_outfault);	}
    
    const vec2 getWedgePoint(unsigned int n) const 						// access to debug wedge info
    {	assert(n<k_wedgepoint_count);	 return(m_outwedgepoints[n]);	}
    const size_t getWedgePointCount() const { return(k_wedgepoint_count); }
    //	Debug access into collision avoidance
    const float getCollisionAvoidCurvature() const { return(m_outcollisionavoidcurv); }
    const float getCollisionAvoidDistance() const { return(m_outcollisionavoiddist); }
	const float getPathLength() const;	
	const vec2 getPointAlongPath(float dist) const;			
 	const std::vector<PathEndpoint>& getpathendpoints() const { return(m_outpathendpoints); }
 	const int getWaypointSerial() const { return(m_outwaypoint); }
    //	Misc. access
	float getmincurv() const { return(m_outmincurv);	}					// safe curvature limits on last cycle
	float getmaxcurv() const { return(m_outmaxcurv);	}
	float getstoppingdist() const { return(m_outstoppingdist); }
private:
	float calcSafeSpeedForCurvature(float curvature);						// speed limit check re curvature
	void setFault(Fault::Faultcode faultid, const char* msg = "");			// report a fault
	bool testCurvatureRange(const WaypointTriple& wp,
		float dist, int side, float mincurv, float maxcurv, float& curvature, vec2& furthestimpingement);
	void updatestate(const vec3& startPos, const vec2& startForward,
                           float startspeed, float startCurvature, float elapsedTime, float pitchradians, float rollradians, int dir);
    bool getRelevantSegmentPair(const ActiveWaypoints& waypoints, WaypointTriple& wpts);
    bool getRelevantSegmentPairFwd(const ActiveWaypoints& waypoints, WaypointTriple& wpts);
    bool getRelevantSegmentPairRev(const ActiveWaypoints& waypoints, WaypointTriple& wpts);
    bool getGoalPointOnCenterline(const WaypointTriple& wp, bool& inturn,
                                  double distahead, vec2& goalpt);
	bool getGoodGoalPointOnCenterline(const WaypointTriple& wp, double distahead, vec2& goalpt, float& maxspeed);
	bool getGoodGoalPointNearCenterline(const WaypointTriple& wp, double distahead, vec2& goalpt, float& maxspeed);
	float getShoulderWidth(float dist);
	float getBaseWidth(bool forobstacles);
	float getMinGoalDist(float dist);			
	bool testGoalPointAgainstBoundaries(const WaypointTriple& wp, float basewidth, const vec2& goalpt);
	bool testCurvatureAgainstBoundaries(const WaypointTriple& wp, float basewidth, float curvature, float arclen,
		vec2& impingement, int& sideout);
	bool testCurvatureGoal(const WaypointTriple& wp, const TerrainMap& map, bool safemode, float shoulderwidth, float curvature, float arclen,
		float& arclenout, ImpingementGroup& impingements);
	bool adjustGoalPointForBoundaries(const WaypointTriple& wp, double distahead, vec2& goalpt, float& maxspeed);
	bool adjustGoalPoint(const WaypointTriple& wp, const TerrainMap& map, double distahead, vec2& goalpt, float& maxspeed);
	bool adjustGoalPoint2(const WaypointTriple& wp, const TerrainMap& map, double& arclen, float& curvature, bool& brakehard);
	void improveGoalPoint(const WaypointTriple& wp, const TerrainMap& map, double& arclen, float& curvature);
	bool searchCurvatureRange(const WaypointTriple& wp, const TerrainMap& map, 
		bool testinsidebounds, double inarclen, float incurv, 
		double& bestarclen, float& bestcurv);
	bool testShouldersAgainstBoundaries(const WaypointTriple& wp, float basewidth, float shoulderwidth, float curvature, float arclen, 
		ImpingementGroup& impingements);
	void combineImpingements(float curvature, ImpingementInfo&  imp1, const ImpingementInfo& imp2);
	float calcImprovedCurvature(const float curvature, const float arclen, ImpingementGroup& impingements, float& curvatureout);
	bool calcTiltPenalty(const ImpingementGroup& impingements, float& tiltpenalty);
	float distanceToImpingement(float curvature, const vec2& impingement);
	float distanceToImpingement(float curvature, const ImpingementInfo& impingement);
	float getSteeringDwellDistance() const;
	float getSteeringLookaheadDistance() const;
	bool testPath(const WaypointTriple& wp, const TerrainMap& map, bool safemode, float shoulderwidth, const CurvedPath& path, 
		const float pathlenin, float& pathlenout, ImpingementGroup& impingements);
	bool constructPathFromCurvature(const WaypointTriple& wp, const TerrainMap& map, float curv, float pathlen, CurvedPath& path, 
			ImpingementGroup& impingements);
	bool tryPathCurvature(const WaypointTriple& wp, const TerrainMap& map, const float testcurv,  
		const vec2& ingoalpt,	CurvedPath& path,	float& metric);																			
	bool constructPath2(const WaypointTriple& wp, const TerrainMap& map, const vec2& ingoal, CurvedPath& path, bool& brakehard);
	bool constructPath(const WaypointTriple& wp, const TerrainMap& map, const vec2& goalpt, CurvedPath& path, float& movedist, float& maxspeed);
	bool searchPathRange(const WaypointTriple& wp, const TerrainMap& map,  bool testinsidebounds, const vec2& ingoal, CurvedPath& path);														// path to work on, will be changed
	void improvePath(const WaypointTriple& wp, const TerrainMap& map, CurvedPath& path, float shoulderwidth, bool& tightspot);
	const std::vector<float>& getCurveTestSchedule();
	void logPathEndpoint(const CurvedPath& path, uint8_t color = 4);
	float adjustMetricForRoad(const TerrainMap& map, float curv, float metric);
    bool steerToGoalPoint(const vec2& goalpt, float& curvature);
	bool steerToPath(const CurvedPath& path, float& curvature);	// steer to follow this path
	bool calcSafeLimits(const WaypointTriple& wp,
                              double distin, float inspeed, bool inturn, double curvature, float& distance, float& speed);
	bool checkOnCourse(WaypointTriple& wp, bool& goveryslow);
    bool dodriving(const ActiveWaypoints& waypoints, const TerrainMap& map,
                   float& safeMoveDistance, float& recommendedCurvature, float& recommendedSpeed);
	//	Test only
	void testImproveGoalPoint(const WaypointTriple& wp, const TerrainMap& map, float shoulderwidth, double& arclen, float& curvature);
};

#endif // NEWSTEER_H
