//
//	playbacktest.cc  --  map updating based on canned file data
//
//	Test only,
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
#include <stdio.h>
#include <vector>
#include "mapserver.h"
#include "logprint.h"
#include "tuneable.h"
#include "algebra3.h"
#include "maplog.h"
#include "gpsins_messaging.h"
#include "eulerangle.h"
#include "timejitter.h"
#include "waypoints.h"
#include "geocoords.h"
#include "interpolatelib.h"

//
const float k_min_tilt = (M_PI/180)*40;						// must have at least 40 degrees of tilt from straight down to be useful.
const float k_ancienttime = 15.0;								// data older than this can be overriden by new. Secs.
//
//	dumptimestamp  --  dump a 64-bit timestamp
//
//	Input is nanoseconds since epoch.
//
static void dumptimestamp(const char* msg, const uint64_t timestamp)
{
	timespec ts;														// as timespec
	nsec2timespec(&ts,timestamp);						// convert to seconds
	time_t tstime = ts.tv_sec;									// into time type
	tm localtm;
	localtime_r(&tstime,&localtm);							// convert time
	char s[100];
	const char* format = "%F %T";							// yyyy-mm-dd hh:mm:ss 
	strftime(s,sizeof(s),format,&localtm);					// edit time
	logprintf("%s %s (%lld ns.)\n",msg,s,timestamp);	// print
}
//
//	interpolateangle  -- interpolate between two angles in radians
//
inline float interpolateangle(float prevang, float nextang, float fract)
{
	vec2 prevdir(cos(prevang),sin(prevang));
	vec2 nextdir(cos(nextang),sin(nextang));
	vec2 dir(nextdir*fract + prevdir*(1.0-fract));					// interpolate yaw by weighted vector addition
	return(atan2(dir[1], dir[0]));												// convert vector to angle
}
//
//	interpolatepose  -- interpolate a pose matrix given two fixes
//
//	Preliminary version - interprets angles, but individually, not as a set
//
static bool interpolatepose(const GPSINSMsgRep& prevfix, const GPSINSMsgRep& nextfix, 
	uint64_t timewanted, mat4& vehpose)
{		mat4 prevpose, nextpose;
	posefromfix(prevfix, prevpose);											// make pose matrices from GPS/INS fix records
	posefromfix(nextfix, nextpose);
	
	
	#define LERPINTERPOLATION
#ifdef LERPINTERPOLATION
	//	Use LERP interpolation code
	return(interpolateLERP(prevpose, prevfix.timestamp, 
		nextpose, nextfix.timestamp,
		timewanted, vehpose));	
#endif
	
	
	
	
//#define NEWINTERPOLATION
#ifdef NEWINTERPOLATION
	//	Use common interpolation code
	return(interpolateposedumb(prevpose, prevfix.timestamp, 
		nextpose, nextfix.timestamp,
		timewanted, vehpose));	
#else
	uint64_t dtperiod = nextfix.timestamp - prevfix.timestamp;	// delta between fixes
	if (timewanted < prevfix.timestamp) return(false);	// not in this block
	uint64_t dt = timewanted - prevfix.timestamp;	// delta from start to end
	float fract = (dt/float(dtperiod));						// fraction into period
	//	Interpolate angles and position separately. Should do a full SLERP interpolation.
	vec3 pprev(ExtractTranslation(prevpose));							// position at start of interval
	vec3 pnext(ExtractTranslation(nextpose));							// position at end of interval
	vec3 position = pnext*fract + pprev*(1.0-fract);					// linearly interpolate position
	float prevroll = deg2radians(prevfix.rpy[0]);						// roll
	float nextroll = deg2radians(nextfix.rpy[0]);						// roll
	float prevpitch = deg2radians(prevfix.rpy[1]);						// roll
	float nextpitch = deg2radians(nextfix.rpy[1]);						// roll
	float prevyaw = (M_PI*0.5)-deg2radians(prevfix.rpy[2]);		// yaw on previous cycle
	float nextyaw = (M_PI*0.5)-deg2radians(nextfix.rpy[2]);		// yaw on next cycle
	float roll = interpolateangle(prevroll, nextroll, fract);
	float pitch = interpolateangle(prevpitch, nextpitch, fract);
	float yaw = interpolateangle(prevyaw, nextyaw, fract);
	const EulerAngles angs(roll, pitch, yaw);								// Roll, pitch, yaw (input is relative to N, CW).
	Eul_ToHMatrix(angs,vehpose,EulOrdXYZr);							// Order is roll, pitch, yaw, sequentially, per AHRS 400 manual.
	vehpose = translation3D(position)*vehpose;						// apply translation
#endif // NEWINTERPOLATION
#ifdef OBSOLETE
	vec3 trans(ExtractTranslation(vehpose));								// ***TEMP***
	printf("interpolatepose: fract %6.2f  pos [%6.2f %6.2f %6.2f]  trans  [%6.2f %6.2f %6.2f]\n",
		fract, prevfix.pos[0],prevfix.pos[1],prevfix.pos[2],trans[0],trans[1],trans[2]);		// ***TEMP***
	vec3 trans2(ExtractTranslation(vehpose));								// ***TEMP***
	printf("interpolatepose:  trans2  [%6.2f %6.2f %6.2f]\n",
		trans2[0],trans2[1],trans2[2]);		// ***TEMP***
#endif // OBSOLETE
	return(true);
}
//
//	getfixbytime --  get the fix pair that brackets the requested time.
//
static bool getfixbytime(const vector<GPSINSMsgRep>& fixes, const uint64_t timewanted, int& fixix)
{	static size_t hint = 0;											// where to try from last time
	if (fixes[hint].timestamp > timewanted)			// if advanced too far
	{	hint = 0;	}													// restart search
	while ((hint < fixes.size()-1)
		&& (fixes[hint+1].timestamp < timewanted))	// not there yet
		{	hint++;	}
	//	We're there, or the data isn't there.
	if (fixes[hint].timestamp <= timewanted)			// if successive values bracket the desired reading
	{	fixix = hint; return(true);	}							// success
	hint = 0;															// failure, start from beginning
	return(false);
}
//
//	getposebytime -- get pose from GPS data, given time
//
static bool getposebytime(const vector<GPSINSMsgRep>& fixes, const uint64_t timewanted, mat4& vehpose, 
	double& latitude, double& longitude, float& speed)
{	if (fixes.size() == 0)											// if no GPS data
	{	vehpose = identity3D();								// consider vehicle to be at origin
		return(true);
	}
	int ix = 0;
	bool good = getfixbytime(fixes, timewanted, ix);	// get appropriate fix
	if (!good) return(false);										// GPS data not in sync with LIDAR data
	const GPSINSMsgRep& prevfix = fixes[ix];
	const GPSINSMsgRep& nextfix = fixes[ix+1];
	good = interpolatepose(prevfix, nextfix, timewanted, vehpose);	// interpolate pose from two fixes
	latitude = prevfix.llh[0];									// latitude
	longitude = prevfix.llh[1];									// longitude
	speed = vec2(prevfix.vel[0], prevfix.vel[1]).length();	// speed, absolute
	return(good);													// return good fix
}
//
//	readgpsins  -- read GPS/INS data records into a list
//
static void readgpsins(FILE* gpsin, vector<GPSINSMsgRep>& fixes)
{	GPSINSMsgRep infix;
	while (fread(&infix,sizeof(infix),1,gpsin) > 0)		// until EOF
	{	fixes.push_back(infix);										// add to list
		////printf("GPSINS: at %lld ns, [%6.2f %6.2f %6.2f]\n",infix.timestamp,infix.pos[0],infix.pos[1],infix.pos[2]);		// ***TEMP***
	}
	//	Fix validation
	for (size_t i=1; i<fixes.size(); i++)						// for all fixes after the first
	{	if (fixes[i-1].timestamp >= fixes[i].timestamp)	// if timestamps out of sequence
		{	logprintf("GPSINS fix %d: timestamps out of sequence.\n",i);	
			exit(1);
		}
	}
	if (fixes.size() < 2)
	{	logprintf("Only %d records in GPSINS file. Cannot use file.\n",fixes.size());	exit(1);}
	dumptimestamp("GPS file start:",(*fixes.begin()).timestamp);
	////	dumptimestamp("GPS file end:  ",(*fixes.back()).timestamp);
	dumptimestamp("GPS file end:  ",fixes[fixes.size()-1].timestamp);
}
//
//	playbackTest -- play back test data
//
//	"dummylidarin" contains LMS LIDAR scan lines, and must be present.
//	"dummygpsinsin" contains GPSINS messages, and is optional. 
//	If GPS/INS data is not present, we assume that we're testing with data from a stationary
//	scanner which can tilt.
//
//	Non real time code.
//
void MapServer::playbackTest(const char* dummylidarin, const char* dummygpsinsin, const char* waypointin, const char* logdirout)
{
	assert(dummylidarin);											// must have LIDAR data
	FILE* lidarin = fopen(dummylidarin,"r");				// open for reading
	if (!lidarin)
	{	logprintf("Unable to open LIDAR data file \'%s\".\n",
			dummylidarin);
		exit(1);																// fails
	}
	FILE* gpsin = NULL;												// GPS file
	vector<GPSINSMsgRep> fixes;								// list of fixes
	if (dummygpsinsin)
	{	gpsin = fopen(dummygpsinsin,"r");					// open for reading
		if (!gpsin)															// if fail
		{	logprintf("Unable to open GPSINS data file \"%s\".\n",
			dummygpsinsin);
			exit(1);															// fails
		}
		readgpsins(gpsin,fixes);									// read list of GPS fixes
	}
	//	Waypoint support
	WaypointSet wpts;												// waypoint list object
	wpts.setVerbose(getVerbose());								// set verboseness
	if (waypointin)
	{	int stat = wpts.readWaypoints(waypointin);		// read the file
		if (stat < 0)
		{	logprintf("Unable to read waypoint file  \"%s\".\n",waypointin);
			exit(1);															// fails
		}
	}
	//	Log file support
	if (logdirout)
	{	m_log.openlogfile(logdirout);										// create a log file
		m_log.logHeader(k_vehlength, k_vehwidth, m_map.getcellspermeter());		// log file header info
	}
	mat4 vehpose(identity3D());									// vehicle pose is at origin looking east.
	mat4 scannerpose1;												// scanner pose of line 1
	mat4 scannerpose2;												// scanner pose of line 2
	LidarScanLine line1, line2;									// our two scan lines
	ActiveWaypoints activewaypoints;						// active waypoint set
	int rejects = 0;														// rejected messages
	bool first = true;
	uint64_t lasttimestamp = 0;									// last time stamp
	for (uint32_t cyclestamp = 0; ;cyclestamp++)
	{	int stat = fread(&line2, sizeof(line2), 1, lidarin);	// get scan line
		if (stat <= 0) break;											// EOF
		lasttimestamp = line2.m_header.m_timestamp;	// last timestamp read
		if (first)
		{		dumptimestamp("First LIDAR line timestamp: ", line2.m_header.m_timestamp);
		}
		//	Update "ancient stamp". Data older than this is overridden by new, even if better
		const int k_scanspersec = 75;											// normal scan rate
		uint32_t ancientdiff = uint32_t(k_ancienttime*k_scanspersec);		// how old
		if (cyclestamp > ancientdiff)										// avoid negative unsigned value
		{	m_map.setancientstamp(cyclestamp - ancientdiff);	}	// older than this is ancient
		double latitude, longitude;											// from last GPS fix, uninterpolated
		float speed;
		bool good = getposebytime(fixes, line2.m_header.m_timestamp, vehpose, latitude, longitude, speed);	// get vehicle pose
		if (!good)
		{	if (rejects++ > 10) continue;								// give up after 10 rejects
			logprintf("Unable to get vehicle pose from GPS data for scan line #%d.\n", cyclestamp);
			dumptimestamp("LIDAR line timestamp: ", line2.m_header.m_timestamp);
			continue;														// otherwise continue
		}
		float avgrange;
		bool goodavgrange = getLMSupdater().LMScalcAverageRange(line2, avgrange);
		float intilt = line2.m_header.m_tilt;
		float tilt;	
		uint32_t  originalCycleStamp = cyclestamp;
		bool goodtilt =  getLMSupdater().getLMStiltCorrector().correctTilt(line2,goodavgrange, avgrange, vehpose, cyclestamp, line2);	// correct tilt
      if (!goodavgrange || !goodtilt) {
          tilt = intilt;
      } else {
      	tilt = line2.m_header.m_tilt;
      }
		scannerpose2 = vehpose*getLMSupdater().LMStiltPose(tilt);	// compute scanner pose from tilt info
		if (getVerbose())
		{	vec3 trans2 = ExtractTranslation(scannerpose2);						// get position
			vec3 trans1 = ExtractTranslation(scannerpose1);						// get position
			printf("Scanner location: [%6.2f %6.2f %6.2f] -  [%6.2f %6.2f %6.2f]\n",
				trans1[0], trans1[1], trans1[2], trans2[0], trans2[1], trans2[2]);
		}
		if (first)																// first time, wait for second scan line
		{	first = false; 
		} else {
			//	Update vehicle position in map
			vec3 trans = ExtractTranslation(vehpose);									// get vehicle position
			m_map.setmapcenter(trans[0], trans[1]);										// scroll the map
			////logprintf("Map center set to (%3.2f, %3.2f)\n", trans[0], trans[1]);	// ***TEMP***
			//	Log vehicle position
		 	m_log.logVehiclePosition(vehpose, m_map.getix(), m_map.getiy(), line2.m_header.m_timestamp,latitude, longitude, speed);
		 	//	Log active waypoint set - inefficient
		 	const float maphalfwidth = m_map.celltocoord(m_map.getdimincells()+1)*0.5;	// halfwidth of map
		 	const float x0 = m_map.celltocoord(m_map.getix()) - maphalfwidth;
		 	const float y0 = m_map.celltocoord(m_map.getiy()) - maphalfwidth;
		 	const float x1 = m_map.celltocoord(m_map.getix()) + maphalfwidth;
		 	const float y1 = m_map.celltocoord(m_map.getiy()) + maphalfwidth;
		 	wpts.waypointsInRect(activewaypoints, x0,y0,x1,y1);	// get relevant waypoints
		 	m_log.logWaypoints(activewaypoints);				// log the set
		 	//	Process pair of scan lines.
		 	if (fabs(line2.m_header.m_tilt) > k_min_tilt)	// update if not looking at tilt head frame.
		 	{	getLMSupdater().LMSupdateScanlinePair(line1, scannerpose1, line2, scannerpose2, cyclestamp, true);
		 	}
			if (logdirout)
			{	////logMap(log,cyclestamp);						// log the map
				m_log.logFrameEnd();								// end of a log frame
			}
		}
		line1 = line2;													// pair for next time
		scannerpose1 = scannerpose2;						// scanner pose for next time
		cyclestamp = originalCycleStamp;
	}
	dumptimestamp("Last LIDAR line timestamp: ", lasttimestamp);	// final timestamp for debug
	fclose(lidarin);
}
