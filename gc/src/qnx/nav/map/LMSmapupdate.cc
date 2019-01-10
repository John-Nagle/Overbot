//
//	LMSmapupdate.cc  --  map updating based on sensor data
//
//	Functions specific to SICK LMS 221 Laser Measurement System.
//
//	John Nagle
//	Team Overbot
//	December, 2004
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
#include "LMSmapupdate.h"
#include "mapserver.h"
#include "logprint.h"
#include "tuneable.h"
#include "algebra3.h"
#include "geocoords.h"
//
//	Configurable constants
//
const Tuneable k_maxangle_param("SCANWIDTH",60,180,120,"Scan width, degrees");
const Tuneable k_maxgazeangle_param("MAXGAZEANGLE", 5,  120, 20, "Scan width which controls gaze, degrees");
//	***RECALCULATE THREAT ARC RADII***
const Tuneable k_threat_arc_radius_red("VERTTHREATARCRED", 5,100, 10, "Vertical threat arc radius, red (m)");
const Tuneable k_threat_arc_radius_yellow("VERTTHREATARCYELLOW", 5,100, 10, "Vertical threat arc radius, yellow (m)");
const Tuneable k_scan_limit_angle_deg("SCANLIMITANGLE", 60,120, 80, "Scan limit angle (deg)");
const Tuneable k_scan_range_factor("SCANRANGEFACTOR", 0.1, 10, 0.75, "Max scan range x turn radius factor");
const double k_scan_limit_angle =  deg2radians(k_scan_limit_angle_deg);	// in radians
const double k_scan_interval = 1.0;										// scans are 1 degree apart
const int k_maxangle = int(k_maxangle_param/(2*k_scan_interval));				// half angle
const int k_maxgazeangle = int(k_maxgazeangle_param/(2*k_scan_interval));	// half angle
//
//// #define SINGLEPOINTTEST	//	worked badly in practice.  Needs work.

//
//	Mounting position of scanner
//
//	Really should be parameters of the object, in case we have multiple scanners.
//
//	Gaze limits. What we can see over the vehicle body.
//									uplimit, upsafelimit, downlimit, sign
//	***THESE NEED CHECKING***
GazeParams k_frontlook = { deg2radians(91), deg2radians(85), deg2radians(55), 1 };
GazeParams k_rearlook = { deg2radians(-91), deg2radians(-85), deg2radians(-75), -1 };
const Tuneable k_default_scan_range("DEFAULTSCANRANGE", 4, 50, 12, "Default scan range (m)");
const char* k_tiltserver = "LMS";											// gets tiilt commands

const float k_scannerheight = 2.04;										// scanner height from ground
const vec3 k_scannertiltaxis(0,1,0);										// tilt axis of scanner
const size_t k_lidar_queue_size  = 25;									// max number of scan lines to queue before processing

inline bool nonrangevalue(uint16_t range) { return(range > 8182); }				// true if special value from SICK unit
//
//	dump  -- debug support
//
static void dump(const vec3& v, const char* msg)
{	logprintf("%s: [%7.4f %7.4f %7.4f]\n", msg, v[0],v[1],v[2]);	}
//
//	dump  -- debug support
//
static void dump(const vec4& v, const char* msg)
{	logprintf("%s: [%7.4f %7.4f %7.4f %7.4f]\n", msg, v[0],v[1],v[2],v[3]);	}
static void dump(const mat4& m, const char* msg)
{	logprintf("%s:\n",msg);
	dump(m[0],"");
	dump(m[1],"");
	dump(m[2],"");
	dump(m[3],"");
}
//
//	pointoutsidecutline  -- is point outside the "cut line"?  
//
//	This gets rid of the artifacts where the same point on the ground is rescanned many times on the inside of turns
//
static bool pointoutsidecutline(const vec3& p, const vec2& cutcenter, const vec2& cutdir, bool cuttoleft)
{	vec2 toright(cutdir[1], -cutdir[0]);										// unit vector to right of line
	vec2 pt(p,VZ);																	// P in 2D
	double dir = toright * (pt - cutcenter);								// dot product, vector to point to right of line
	return((dir < 0) == cuttoleft);											// true if on "cut side"					
}																				
//
//	Constructor
//
LMSmapUpdater::LMSmapUpdater(MapServer& owner, const vec3& scanneroffset)
	: m_owner(owner), 
	m_gazecontrol(*this, k_tiltserver, k_frontlook, k_rearlook),	// gaze initialization
	m_scanneroffset(scanneroffset), 
	m_prevvalid(false),
	m_tiltcorrector(*this)
{
	m_scanneroffsetransform = translation3D(m_scanneroffset);	// construct matrix for scanner offset from GPS pos
	//	Allocate queue items for LIDAR queue. Use "new" only at startup.
	for (size_t i=0;  i < k_lidar_queue_size; i++)
	{	LidarScanLine* blank = new LidarScanLine;				// get a blank line
		m_emptyqueue.push(blank);										// fill queue with empty scan lines
	}
	//	Set gaze control to a constant value (TEMP)
	m_gazecontrol.setrangegoal(k_default_scan_range);		// set range goal, in m, for auto tilt management
}
	
//
//
//	handleLidarData  --  handle an incoming LMS LIDAR scan line
//
//	GPS/INS synchronization requires a queue of LIDAR scan lines and of GPS poses,
//	which have to be matched and interpolated.
//
void LMSmapUpdater::LMShandleLidarData(const LidarScanLine& lp)
{
	ost::MutexLock lok(m_owner.getMapLock());						// lock map during update
	//	Buffered line processing
    if (getVerboseLevel() >= 3)
    {
        logprintf("handleLidarData: time=%lld. count=%d, status=%x\n",
                  lp.m_header.m_timestamp, lp.m_header.m_valueCount, lp.m_header.m_statusByte);
    }
    //	Put new line on queue
    //
    if (m_emptyqueue.empty())												// if no available buffers
    {	logprintf("LIDAR queue stuck. Flushing.\n");					// should not happen
    	while (!m_linequeue.empty())										// flush queue
    	{	LidarScanLine* p =  m_linequeue.front();					// get first entry
    		m_linequeue.pop();													// pop from line queue
    		assert(p);																	// must be nonempty
    		m_emptyqueue.push(p);											// push on empty queue, ignoring
    	}
    }
    assert(!m_emptyqueue.empty());										// must have a working buffer
    LidarScanLine* work = m_emptyqueue.front();				// get a working buffer
    m_emptyqueue.pop();														// remove it from the queue
    assert(work);
    *work = lp;																		// save new line
    m_linequeue.push(work);													// push onto work queue
  	//	Process the queue, in order
  	while (!m_linequeue.empty())											// while lines to process
  	{	LidarScanLine* first = m_linequeue.front();					// get first item
  		assert(first);																	// must get it
		VehiclePose vehpose;													// get vehicle pose
		bool toolate;			
		//	Try to get a relevant vehicle pose from interpolation								
		bool good = m_owner.getPoses().getposeattime(vehpose, first->m_header.m_timestamp, toolate);	// get pose at time of timestamp
		if ((!good) && (!toolate))												// if not ready to process the first line
		{	break;	}																	// try again later
		//	We will use up this line
		if (good)																		// we have GPS data
		{	LMShandlePosedLidarData(*first, vehpose.m_vehpose);	// process line with vehicle position
		} else {
			logprintf("No valid, current GPS data. LIDAR data ignored.\n");	
			m_prevvalid = false;												// drop previous line as obsolete
		}
		m_linequeue.pop();													// remove from work queue
		m_emptyqueue.push(first);										// move to empty queue
	}
}
//
//	LMShandlePosedLidarData  -- process scan line for which we have a vehicle position
//
void LMSmapUpdater::LMShandlePosedLidarData(const LidarScanLine& inlp, const mat4& invehpose)
{
	//	Stamp the scan line
	uint32_t cyclestamp = m_owner.getMap().incrementcyclestamp();	// increment scan line serial number
	float avgrange;																// average range
	bool goodavgrange = LMScalcAverageRange(inlp, avgrange);			// calc average range
	LidarScanLine lp;  // scan line data with corrected tilt
	mat4 vehpose=invehpose; 
	bool good = m_tiltcorrector.correctTilt(inlp,goodavgrange, avgrange, vehpose, cyclestamp, lp);	// correct tilt
	if (!good)																		// trouble
	{	m_prevvalid = false;													// no good scan line pair
		return;																		// reject
	}
	float tilt = lp.m_header.m_tilt;										// get corrected tilt
	//	tilt angle, 0 is down, pi/2 is forward
	//	Tilt correction based on average range to target for area near center of scan

	good = LMSgazeCheck(tilt, avgrange, goodavgrange);			// check gaze direction, dazzle, tilt, etc.
	if (!good)																		// trouble
	{	m_prevvalid = false;													// no good scan line pair
		return;																		// reject
	}
	//	Compute scanner position from vehicle position
	const mat4 scannerpose(vehpose*LMStiltPose(tilt));	// get the scanner transform in world space
    if (m_prevvalid)															// if previous line is valid
	{	 //	Process the pair of scan lines
    	LMSupdateScanlinePair(m_prevscanline, m_prevscannerpose, lp, scannerpose, cyclestamp);
    }
    m_prevscannerpose = scannerpose;							// save for next pair of scan lines
    m_prevscanline = lp;
    m_prevvalid = true;
}
//
//	class LMSLidarScanVectors  -- represents a vector of LIDAR scan lines
//
//	This is just precomputation of constants.
//
class LMSLidarScanVectors {
private:
	int m_entries;											// number of entries
	int m_center;												// center of the entries
	std::vector<vec3> m_oddscan;				// 3D vectors for direction from LIDAR for each scan point
	std::vector<vec3> m_evenscan;
public:
	LMSLidarScanVectors(size_t entries, double interval = M_PI/180)
	: m_entries(entries), m_center(entries/2)
	{	assert(entries % 2 == 1);						// must have odd number of entries
		m_oddscan.resize(entries);
		m_evenscan.resize(entries-1);
		//	Calculate vectors for "odd" scan line (-2, -1, 0, 1, 2)
		for (size_t i=0; i<entries/2; i++)
		{	double angle = i*interval;
			vec3 v(cos(angle),sin(angle),0);
			m_oddscan[m_center+i] = v;
			v[1] = -v[1];
			m_oddscan[m_center-i] = v;
		}
		//	Calculate vectors for "even" scan line (-1.5, -0.5, 0.5, 1.5)
		for (size_t i=0; i<(entries/2)-1; i++)
		{	double angle = i*interval + (interval*0.5);
			vec3 v(cos(angle),sin(angle),0);
			m_evenscan[m_center+i] = v;
			v[1] = -v[1];
			m_evenscan[m_center-i-1] = v;
		}
	}
	int size() const { return(m_entries);	}						// get entry count
	const vec3& scanVector(int index, bool odd) const		// counting from center
	{
#ifdef OBSOLETE	
		const vec3& result = odd ? m_oddscan[m_center+index] : m_evenscan[m_center+index];
		logprintf("scanVector for %d (%s): [%7.3f %7.3f %7.3f]\n", 						// ***TEMP***
			index, (odd ? "odd" : "even"), 
			result[0],result[1],result[2]);
#endif // OBSOLETE
		return(odd ? m_oddscan[m_center+index] : m_evenscan[m_center+index]);	
	}
};
const LMSLidarScanVectors k_scan_table(181);				// build table
//
//	LMSrangeConvert  --  convert range from LIDAR value to meters
//
//	Returns zero for invalid values.
//
inline float LMSrangeConvert(uint16_t range)
{	if (nonrangevalue(range)) return(0.0);							// handle special value
	return(range*0.01);														// convert to meters
}
//
//	LMScalcAverageRange  -- check scan data for average range of center section of scan
//
//	Returns true if narrow range is all good
//
bool 	LMSmapUpdater::LMScalcAverageRange(const LidarScanLine& lp, float& avgrange)
{	const int center = k_scan_table.size()/2;							// index of center point
	const int k_max_bad_points = 4;										// more than this stops scan
	const float k_default_avgrange = 999;								// huge bogus average range value
	//	Scan for gaze management purposes. Narrow scan range only.
	//	If we can't see all the pixels in the narrow scan range, we have to tilt the scanner down
	{	uint32_t rangetotal = 0;												// total of ranges examined
		int rangecount = 0;														// count
		int badpoints = 0;															// bad points seen
		for (int i= -k_maxgazeangle; i<= k_maxgazeangle; i++)
		{	uint16_t range = lp.m_range[center+i];					// get a range point
			if (!nonrangevalue(range))										// if valid
			{	rangetotal += range;	rangecount ++;}				// total
			else																			// bad values
			{	badpoints++;														// trouble
			}
		}
		avgrange = rangecount ? (rangetotal*0.01/rangecount) : k_default_avgrange;			// range seen, avoid divide by 0
		return(badpoints <= k_max_bad_points);						// valid if no bad points
	}
}
//
//	LMSgazeCheck  -- check scan data for gaze adjustment
//
bool 	LMSmapUpdater::LMSgazeCheck(float tilt, float avgrange, bool gooddata)
{
	bool good = m_gazecontrol.reportranges(tilt, avgrange, !gooddata);	// update gaze management
	if (!good && getVerboseLevel() >= 2)					// debug
	{	logprintf("Gaze control rejected line: tilt %1.2f deg, range %1.2f m %s\n",
			radians2deg(tilt), avgrange, (gooddata ? "" : "(BAD)"));
	}
	return(good);														// don't update bad lines
}
//
//	LMSupdateScanlinePair  -- update map using a pair of scan lines
//
//	This is where all the real work gets done.
//
void LMSmapUpdater::LMSupdateScanlinePair(const LidarScanLine& lp1, const mat4& scannerpose1,
					const LidarScanLine& lp2, const mat4& scannerpose2, uint32_t cyclestamp, bool playback)
{
	ost::MutexLock lok(m_owner.getMapLock());					// lock map during update; others are reading it
	bool lp1odd = (lp1.m_header.m_valueCount & 1);		// an "odd" line contains 181 entries, degrees -180,-179....179..180
	bool lp2odd = (lp2.m_header.m_valueCount & 1);		// an "even" line contains 180 entries, -179.5,-178.5 ... 178.5, 179.5
	if (!(lp1odd ^ lp2odd))
	{	logprintf("Scan lines not interlaced.\n"); return;	}		// ignore, bad data
	//	Check scan line lengths.  If these are wrong, we have a scanner problem.
	if ((lp1.m_header.m_valueCount | 1) != k_scan_table.size())
	{	logprintf("Scan line length error: expected %d, received %d\n", k_scan_table.size(), lp1.m_header.m_valueCount);
		return;
	}
	if ((lp2.m_header.m_valueCount | 1) != k_scan_table.size())
	{	logprintf("Scan line length error: expected %d, received %d\n", k_scan_table.size(), lp2.m_header.m_valueCount);
		return;
	}
	if (getVerboseLevel() >= 3)																// dump scanner pose matrices
	{	dump(scannerpose1,"scanner pose 1");						
		dump(scannerpose2,"scanner pose 2");
	}
	//	If "sweeping", we're not moving, and quality is better
	bool sweeping = gazeissweeping();							// true if sweeping
	bool tracking = gazeistracking();									// true if tracking the ground
	if (!(sweeping || tracking || playback)) return;				// don't update unless sweeping or tracking
	//	Calculate "cut limits", to avoid junk when the vehicle is turning and the lines overlap
	vec2 cutcenter, cutdir;
	bool cuttoleft, needcut;
	LMScalcScanLimits(scannerpose1, cutcenter, cutdir, cuttoleft, needcut);
	//	Calculate minimum ranges at which a point is a serious threat.
	//	This catches above-the-ground obvious obstacles
#ifdef SINGLEPOINTTEST	// didn't work out well in practice
	float redthreatrange = LMScalcThreatRange(scannerpose1, m_owner.getNogoCellRoughnessLimit(), k_threat_arc_radius_red);			// definite no-go
	float yellowthreatrange = LMScalcThreatRange(scannerpose1, m_owner.getClearCellRoughnessLimit(), k_threat_arc_radius_yellow);		// eval in detail later
#endif // SINGLEPOINTTEST
	const int center = k_scan_table.size()/2;						// index of center point
	//	Scan across the two scan lines, updating triangles of three scan points.
	for (int i = -k_maxangle; i<= k_maxangle; i++)			// scan across the line
	{	
		//	Get relevant ranges and convert to meters.
		//	Invalid values become zero.
		float r1a = LMSrangeConvert(lp1.m_range[center+i-1]);
		float r1b = LMSrangeConvert(lp1.m_range[center+i]);
		float r2a = LMSrangeConvert(lp2.m_range[center+i-1]);
		float r2b = LMSrangeConvert(lp2.m_range[center+i]);
		//	Calculate minimum range for each triangle.  This is used only so that near updates override far ones.
		float minrange1 = std::min(std::min(r1a,r1b),r2a);
		float minrange2 = std::min(std::min(r1b,r2a),r2b);
		//	***TEMP TEST*** discard triangles where second scan line is nearer than first
		if (!sweeping)
		{	if (r1a > r2a && r1b > r2b) 									// if reversed order of scan lines
			{	continue;	}
		}
		//	***END TEMP TEST***

		if (minrange1 <= 0 || minrange2 <= 0)
		{	//	***Check for "pollution" value.
		}
		//	Compute the three points of each triangle.	Note that this may take place with zero (bogus) ranges.
		//	These points are in scanner space.
		vec3 p1a(k_scan_table.scanVector(i-1, lp1odd)*r1a);
		vec3 p1b(k_scan_table.scanVector(i, lp1odd)*r1b);
		vec3 p2a(k_scan_table.scanVector(i-1, lp2odd)*r2a);
		vec3 p2b(k_scan_table.scanVector(i, lp2odd)*r2b);
		if (getVerboseLevel() >= 4)													// really, really verbose
		{	logprintf("Pos %3d: ranges %6.2f %6.2f %6.2f %6.2f\n",
				i,
				r1a,r1b,r2a,r2b);
			dump(p1a,"p1a");														// ***TEMP*** very verbose
			dump(p1b,"p1b");
			dump(p2a,"p2a");
			dump(p2b,"p2b");
		}
#ifdef SINGLEPOINTTEST // didn't work out in practice
		//	Single point check for threatening points.
		//	We only look at one point of each group, so we examine each point exactly once.
		float threatrange = p1a*vec3(1,0,0);							// range from scanner, forward, in scan plane
#endif // SINGLEPOINTTEST
		//	Transform points into world space
		p1a = scannerpose1*p1a;
		p1b = scannerpose1*p1b;
		p2a = scannerpose2*p2a;
		p2b = scannerpose2*p2b;
		if (getVerboseLevel() >= 4)
		{
			dump(p1a,"p1a world");														// really, really verbose
			dump(p1b,"p1b world");
			dump(p2a,"p2a world");
			dump(p2b,"p2b world");
		}
		//	Finish single-point test
#ifdef SINGLEPOINTTEST
		if (threatrange > 0)														// if not a no-find point
		{	if (threatrange < redthreatrange)								// if red or yellow threat
			{	float elev = p1a[2];												// elevation
				m_owner.updateCell(p1a, CellData::NOGO, minrange1, 0, elev, cyclestamp);		// mark cell as red
			}
			else if (threatrange < yellowthreatrange)				// if yellow threat
			{	float elev = p1a[2];												// elevation
				m_owner.updateCell(p1a, CellData::POSSIBLE, minrange1, 0, elev,cyclestamp);	 // mark cell as yellow
			}
		}
#endif // SINGLEPOINTTEST
		//	Check whether point should be ignored because the vehicle is turning too hard.
		if (needcut && pointoutsidecutline(p1a, cutcenter, cutdir, cuttoleft))				// check against cut line				
		{	continue;	}
		//	Update map by triangles.
		m_owner.updateMapTriangle(p1a, p1b,p2a, sweeping, minrange1, cyclestamp);			// add a triangle into the map
		m_owner.updateMapTriangle(p2a, p2b, p1b, sweeping, minrange2, cyclestamp);			// add a triangle into the map
#ifdef OBSOLETE // replaced by single-point test
		//	Update map by edges.  This catches big vertical changes, which may not form valid triangles.
		//	We only do this for three of the five edges, to avoid duplication with the next triangle
		if (r1a <= 0 || r2a <= 0 || r1b <= 0 || r2b <= 0)											// if some triangle not valid
		{	m_owner.updateMapEdge(p1a,p2a,r1a,r2a,sweeping,cyclestamp);			// add three edges into the map
			m_owner.updateMapEdge(p1a,p2b,r1a,r2b,sweeping,cyclestamp);
			m_owner.updateMapEdge(p2a,p2b,r2a,r2b,sweeping,cyclestamp);
		}
#endif // OBSOLETE
	}
}
//
//	LMScalcThreatRange -- calculate minimum range at which a return is a threat
//
//	Returned range is in meters
//
//	Checks the range against a cylinder starting minheight above the ground, with radius rad
//
float LMSmapUpdater::LMScalcThreatRange(const mat4& scannerpose, float minheight, float radius)
{	//	Find center of cylinder relative to scanner
	float scannertominheight = k_scannerheight - minheight;								// minheight to scanner dist
	vec3 scannerray(scannerpose*vec3(1,0,0));													// ray outward from scanner on centerline
	scannerray[0] = fabs(scannerray[0]);																// handle backwards pointing
	float ang = atan2(scannerray[2], scannerray[0]);											// scanner angle (0=horiz, >0=down)
	//	Calculate intersection between scannerray and cylinder 
	ang += (M_PI/2);																								// add quarter circle to ang
	float a = radius-scannertominheight;																// vertical side of triangle
	float c = radius;
	//	Triangle with sides a ,b, c  Angle ab is "ang". We have a,c. Need b
	//	Law of cosines: c*c = a*a + b*b -2*a*b*cos(ang);
	//	Solving for b, we have a quadratic:
	//	(1)*b*b + (-2*a*cos(ang))*b + (a*a -c*c) = 0;
	//	So
	const float aterm = 1;
	const float bterm = (-2*a*cos(ang));
	const float cterm = a*a - c*c;
	float b = (-bterm + sqrt(bterm*bterm - 4*aterm*cterm))/(2*aterm);						// by quadratic formula
	////logprintf("Scanner ray: (%1.4f %1.4f %1.4f) ang %1.4f a %1.2f  c %1.2f  b %1.2f\n",
	////	 scannerray[0], scannerray[1], scannerray[2],ang,a,c,b);	// ***TEMP***
	return(b);
}
//
//	LMStiltPose  -- transformation from vehicle coords to scanner coords
//
//	Remember, Z is upward.
//
//	***CHECK SIGN OF TILT***
//
mat4 LMSmapUpdater::LMStiltPose(float tilt)
{	////float tiltdeg = tilt*(180/M_PI) - 90;																	// tilt in degrees 0 is horizontal, up is +
	////float tiltdeg = tilt*(-180/M_PI) - 90;																	// tilt in degrees 0 is horizontal, up is +
	float tiltdeg = tilt*(-180/M_PI) + 90;																	// tiltdeg 0 is horizontal, up is -
	if (getVerboseLevel() >= 3)
	{	logprintf("Tilt below horizontal. degrees: %6.3f\n",tiltdeg);								
		dump(rotation3D(k_scannertiltaxis, tiltdeg),"tilt pose");									
	}
	return(m_scanneroffsetransform*rotation3D(k_scannertiltaxis, tiltdeg));				// compute scanner transform
}
//
//	Gaze-related functions
//
//	requestNod -- request a "nod" -- look closer
//
void LMSmapUpdater::requestNod()
{
	//	***MORE***
}
//
//	requestSweep -- request a full sweep
//
//	Usually called when in trouble
//
void LMSmapUpdater::requestSweep()
{
	m_gazecontrol.requestsweep();																			// request a LIDAR sweep
}
//
//	gazeistracking -- true if up and in tracking mode
//
bool LMSmapUpdater::gazeistracking()
{
	return(m_gazecontrol.gazeistracking());																// from gaze
}
//
//	gazeissweeping  -- true if gaze is in sweep mode
//
bool LMSmapUpdater::gazeissweeping()
{
	return(m_gazecontrol.gazeissweeping());																// from gaze
}
//
//	getVerbose  -- true if verbose mode
//
int LMSmapUpdater::getVerboseLevel() const
{	return(m_owner.getVerboseLevel()); }																			// get from parent
//
//	resetAncientStamp  --  reset ancient stamp so new data overrides old
//
void LMSmapUpdater::resetAncientStamp()
{
	TerrainMap& map = m_owner.getMap();				// get the map
	ost::MutexLock lok(m_owner.getMapLock());			// lock the map
	map.setancientstamp(map.getcyclestamp());		// data from here on will override old data
}
//
//	LMScalcScanLimits  --  calculate limits of allowed scan
//
//	In turns, we don't want to look beyond a cutoff line.
//
//	There is a drawing to illustrate this in the doc directory, under "nav".  See "Map updating in tight turns".
//
void LMSmapUpdater::LMScalcScanLimits(const mat4& scannerpose, vec2& center, vec2& dir, bool& cuttoleft, bool& needcut)
{	cuttoleft = false;													// avoid returning junk
	needcut = false;													// no cut necessary yet
	float scanrange = k_default_scan_range;				// set desired scan range
	float curvature = m_owner.getCurvature();			// get current turning curvature	
	center = vec2(0,0);
	dir = vec2(0,0);													// avoid returning junk
	if (!gazeistracking()) curvature = 0;						// ignore during sweeps
	if (fabs(curvature) > 0.02)									// if significant curvature (radius < 50m)
	{	needcut = true;												// indicate cut line needed
		cuttoleft = (curvature < 0);								// cut to left if curving to left
		float radius = 1.0 / curvature;							// current turning radius (signed)
		vec3 pos3d = ExtractTranslation(scannerpose);	// position of scanner, 3D.
		vec2 pos(pos3d, VZ);										// position of scanner, 2D
		vec3 forward3d = scannerpose * vec3(1,0,0);	// point one unit in direction scanner is pointing
		vec2 forward(forward3d, VZ);							// drop to 2D
		forward = forward - pos;									// vector in forward dir
		forward.normalize();											// make a normal vector
		vec2 toright(forward[1], -forward[0]);				// vector to right
		center = vec2(toright*radius);							// center of turn, relative to vehicle
		////logprintf("LMS: pos (%1.2f %1.2f %1.2f)  dir (%1.2f %1.2f %1.2f  forward (%1.2f %1.2f)\n",
		////	pos3d[0], pos3d[1], pos3d[2], forward3d[0], forward3d[1], forward3d[2], forward[0], forward[1]);	// ***TEMP***
		center += pos;													// center of turn, absolute, 2d
		//	Now construct limiting unit direction vector from center
		double sinscanlimitangle = sin(k_scan_limit_angle);
		double cosscanlimitangle = cos(k_scan_limit_angle);
		if (curvature > 0) cosscanlimitangle = -cosscanlimitangle;		// always inward for positive value
		dir = vec2(cosscanlimitangle * toright + sinscanlimitangle * forward);	// direction to limit in world space
	////	logprintf("LMS: tight turn, ignore points %s of line from (%1.2f, %1.2f) along (%1.2f, %1.2f)\n",
		////	(cuttoleft) ? "left": "right", center[0], center[1], dir[0], dir[1]);	// ***TEMP***
		//	Finally, adjust tilt during turns to avoid blind spot at end of turn
		float safescanrange = fabs(radius)*k_scan_range_factor;	// calculate a safe scan range
		scanrange = std::min(safescanrange, float(k_default_scan_range));	// set scan range
	}
	m_gazecontrol.setrangegoal(scanrange);			// set range goal, in m, for auto tilt management
}





