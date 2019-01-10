//
//	Dummy VORAD client,.  Used ONLY for debug
//
//	Generates dummy "obstacle" data
//
//	J.	Nagle
//	Team Overbot
//	February, 2005
//
#include <strings.h>
#include <vector>
#include "algebra3.h"
#include "logprint.h"
#include "timeutil.h"
#include "speedserver.h"
#include "voradserver.h"
//
//	Constants
//
const float k_obstacle_size = 2.0;														// car-sized obstacles
const float k_max_range = 50;															// 50m radar range
////const float k_maxscananglerad =  (M_PI/180)*15;						// +-15 degree scan range
const float k_maxscananglerad =  (M_PI/180)*60;							// +-60 degree scan range, to emulate LIDAR range
const float k_mincosoffset = cos(M_PI*0.5 - k_maxscananglerad);	// range of scan
const vec3 k_scanneroffset(2, 0, -0.25);											// offset between GPS antenna and VORAD scanner, meters

//
//	addtarget -- add a target to a VORAD message 
//
static bool addtarget(VoradServerMsgVDTG& msg, const vec2& targvector)
{
	if (msg.m_targetcount >= VoradServerMsgVDTG::k_maxtargets) return(false);	// full
	VoradTargetItem& target = msg.m_targets[msg.m_targetcount++];			// get appropriate target slot
	target.m_movingtarget = false;														// fill in dummy target data									
	target.m_collisionthreatlev = 0;
	target.m_targ_rrange = 0;
	target.m_targ_x = targvector[0];													// set relative target coords.
	target.m_targ_y = targvector[1];
	return(true);
}
//
//	simulatevorad  -- simulate one VORAD cycle
//
void SpeedServerDummy::simulatevorad()
{
	vec2 pos(m_dummyx, m_dummyy);												// where we are now
	vec2 dir(sin(m_dummyheading), cos(m_dummyheading));			// direction (relative to X axis)
	VoradServerMsgVDTG msg;															// message we are generating
	msg.m_msgtype = VoradServerMsgVDTG::k_msgtype;					// set message type
	msg.m_timestamp = gettimenowns();											// timestamp event
	msg.m_targetcount = 0;																// no targets yet
	for (size_t i=0; i<m_dummyobstacles.size(); i++)							// scan all dummy obstacles
	{	const vec2& ob = m_dummyobstacles[i];									// this obstacle
		vec2 vectoob(ob-pos);																// vector from vehicle position to obstacle
		double dist = vectoob.length();												// distance to obstacle
		if (dist < 0.01 || dist > k_max_range) continue;						// not a valid obstacle
		vec2 unitvectoob(vectoob*(1/dist));											// unit vector to obstacle
		double cosoffset = unitvectoob*dir;											// cos of angle obstacle is off axis
		if (fabs(cosoffset) < k_mincosoffset) continue;							// if too far off, ignore
		//	Rotate target into frame of vehicle.
		//	(If only algebra3.h had "mat2d")
		mat3 rot(rotation2D(vec2(0,0),-(90-(180/M_PI)*m_dummyheading)));	// remember, heading is 0=north
		vec3 targ3(rot*vectoob);															// rotate vector into vehicle coords
		vec2 targ(targ3[0],targ3[1]);													// extract 2D translation
		targ -= vec2(k_scanneroffset[0], k_scanneroffset[1]);				// make relative to VORAD
		bool seen = addtarget(msg, targ);											// add target to message
		if (seen)
		{	logprintf("Dummy VORAD target at (%1.2f, %1.2f) -> (%1.2f, %1.2f)\n", ob[0], ob[1], targ[0], targ[1]);	// note target in log
		}
	}
	//	Target message built, send it
	int stat = m_dummymapclientport.MsgSend(msg);					// send it
	if (stat < 0)
	{	logprintf("Error sending VORAD data to map server: %s\n", strerror(errno));	}
}
//
//	loaddummyobstacles -- load dummy obstacles from file
//
//	A dummy obstacle file is just a set of lines of X,Y values.
//
int SpeedServerDummy::loaddummyobstacles(const char* filename)
{
	FILE* infile = fopen(filename,"r");												// open for reading
	if (infile == NULL) return(-1);													// fails
	for (int lineno =1; ; lineno++)													// read input lines
	{	char s[200];
		char* ins = fgets(s,sizeof(s),infile);										// read a line
		if (!ins) break;																		// EOF
		double x=0, y=0;
		int cnt = sscanf(s,"%lf %lf", &x, &y);										// read X and Y
		if (cnt != 2)
		{	logprintf("Line %d: \"%s\" in dummy obstacle file %s is bad.\n", lineno, s, filename);
			fclose(infile); 
			return(-1);
		}
		vec2 pt(x,y);																			// make point
		logprintf("Dummy VORAD target at (%1.2f, %1.2f)\n", x,y);	// list point
		m_dummyobstacles.push_back(pt);										// add point
	}
	fclose(infile);																				// finally close file
	return(0);																					// success
}
