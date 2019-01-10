//
//	Dummy GPS/INS server. Used ONLY for debug
//
//	Outputs the position from the dummy move server
//
//	J.	Nagle
//	Team Overbot
//	February, 2005
//
#include <pthread.h>
#include <strings.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include "logprint.h"
#include "speedserver.h"
#include "Nav.h"
//
//	Misc. support
//
const double k_earth_circumference = 40076000;						// rough value
const double k_earth_radius = k_earth_circumference / (2*M_PI) - 6866;	// radius of earth (rough) ***TEMP***
//
//	convert degrees to radians
inline double deg2radians(double s) { return(s*(M_PI/180.0));	}
//	convert radians to degrees
inline double radians2deg(double s) { return(s*(180.0/M_PI));	}
#ifdef NOTYET	// Math is off by 20Km in north dir.
//
//	calc_lat_long  --  convert current position in X,Y to lat, long in degrees
//
//	For fake data only. Elevation basis is arbitrary.
//
void SpeedServerDummy::calc_lat_long(double llh[3])
{
	Vector<3> LocalPos;																	// position in north, east, elev (?)
	////double elev = 0;																			// no simulated elevation yet
	LocalPos[0] = m_dummyy;															// local north east, up (?)
	LocalPos[1] = m_dummyx;
	LocalPos[2] = -k_earth_radius;
	//	Compute position in earth-centered, earth fixed.
	Vector<3> ECEFpos = Tangent2ECEF(LocalPos, m_dummyllhbasepoint[0], m_dummyllhbasepoint[1]);
	Vector<3> LLHpos = ECEF2llh(ECEFpos);										// convert from ECEF to lat, long, height
	llh[0] = radians2deg(LLHpos[0]);													// convert to degrees for output
	llh[1] = radians2deg(LLHpos[1]);
	llh[2] = LLHpos[2];																		// elev is meters above geoid
	logprintf("NED pos: (%1.2f, %1.2f, %1.2f) -> ECEF pos  (%1.2f, %1.2f, %1.2f) -> LLH (%1.6f, %1.6f, %1.2f m)\n",
		LocalPos[0], LocalPos[1], LocalPos[2], ECEFpos[0], ECEFpos[1], ECEFpos[2], LLHpos[0], LLHpos[1], LLHpos[2]);	
}
#endif // NOTYET
//
//	handle_request --  answer a GPS request with fake data from the dummy speed server
//
void SpeedServerDummy::handle_request(int rcvid)
{
	struct GPSINSMsgRep rep;																// build reply here
	bzero(&rep,sizeof(rep));																// clear object
	rep.err = GPSINS_MSG::OK;															// GPS OK
	struct timespec nowtime;
    clock_gettime(CLOCK_REALTIME, &nowtime);									// get now										
	rep.timestamp = timespec2nsec(&nowtime);								// timestamp  
    rep.posStat =  GPSINS_MSG::SOL_COMPUTED;          						// position status
    rep.posType = GPSINS_MSG::OMNISTAR_HP;									// good quality data
    rep.pos[1] = m_dummyx;																// send back dummy position in North, East, Down coords
    rep.pos[0] = m_dummyy;
    rep.pos[2] = 0;																				// no Z
    rep.rpy[0] = 0;																				// no roll
    rep.rpy[1] = 0;																				// no pitch
    rep.rpy[2] = m_dummyheading*(180/M_PI);									// heading, in degrees
    rep.vel[0] = sin(m_dummyheading)*m_dummyspeed;					// X speed
    rep.vel[1] = cos(m_dummyheading)*m_dummyspeed;				// Y speed
    rep.vel[2] = 0;			
 ////#ifdef OBSOLETE																	// no Z speed
    //	Our origin is at latitude 0, longitude 0.
    rep.llh[0] = m_dummyy * (360/k_earth_circumference);				// fake latitude
    rep.llh[1] = m_dummyx * (360/k_earth_circumference);				// fake longitude
    rep.llh[2] = 0;																				// fake height
//// #endif // OBSOLETE
////	calc_lat_long(rep.llh);																	// calculate latitude, longitude from X,Y
    //	All other fields are zero for now.
	int err = MsgReply(rcvid, rep);
	if (err) {
		perror("server-thread: MsgReply in handle_request");
	}
}

//
//	handle_basepoint -- handle a request to change the basepoint
//
void SpeedServerDummy::handle_basepoint(int rcvid, const GPSINSBasepoint& msg)
{
	//	Sets X=0. Y=0 for  latitude, longitude, height
	m_dummyllhbasepoint[0] = deg2radians(msg.llh[0]);
	m_dummyllhbasepoint[1] = deg2radians(msg.llh[1]);
	m_dummyllhbasepoint[2] = msg.llh[2];
	MsgError(rcvid, EOK);									// reply OK, no data
}
//
//	handle_getbasepoint -- handle a request to change the basepoint
//
void SpeedServerDummy::handle_getbasepoint(int rcvid, const GPSINSGetBasepoint& msg)
{	// 	In the dummy world, the origin is whatever we set it to
	GPSINSGetBasepointRep reply;
	reply.llh[0] = radians2deg(m_dummyllhbasepoint[0]);
	reply.llh[1] = radians2deg(m_dummyllhbasepoint[1]);
	reply.llh[2] = m_dummyllhbasepoint[2];								
	MsgReply(rcvid, reply);									// reply
}
//
//	server_thread  -- the dummy speed server thread
//
void* SpeedServerDummy::server_thread()
{
	logprintf("Starting dummy GPS/INS server. OFFLINE USE ONLY.\n");	
	// define message port, no timeout
	//	Create a dummy server on a name_attach port.  TEST ONLY
	const char* myname = "DUMMYGPS";										// create a dummy GPS server
	name_attach_t* attach = name_attach(0,myname,0);				// create a path
	if (!attach)
	{	printf("Failed to name_attach to \"%s\"\n", myname);			// two copies are probably running
		exit(1);
	}
	logprintf("Dummy GPS server at \"%s\" \n",myname);				// server is ready	
	for (;;) {
		GPSINSMsg  msgin;															// union of all allowed messages

		int rcvid = MsgReceive(attach->chid, &msgin, sizeof(msgin),0);

		if (rcvid <= 0) {
			printf("MsgReceive failed in server\n");
			sleep (1);
			continue;
		}
		if (rcvid == 0) {
			printf("Received a pulse; shouldn't be receiving it\n");
			sleep(1);
			continue;
		}

		//	rcvid ok
		//	Fan out on message type
		switch (msgin.m_req.m_msgtype) {
		case GPSINSMsgReq::k_msgtype:											// request fix
			handle_request(rcvid);
			break;
				
		case GPSINSBasepoint::k_msgtype:										// basepoint change
			handle_basepoint(rcvid, msgin.m_basepoint);					// do a basepoint change
			break;
				
		case GPSINSGetBasepoint::k_msgtype:									// get basepoint
			handle_getbasepoint(rcvid, msgin.m_getbasepoint);		// just retrieves current basepoint
			break;
			
		default:
			MsgError(rcvid, EINVAL);													// some invalid request
			break;
		}
	}
	return(0);																						// unreachable
}
//
//	startdummygpsinsserver  -- start the dummy GPS/INS server
//
void SpeedServerDummy::startdummygpsinsserver()
{	pthread_create(0,0,serverthreadstart,this);									// start the thread
}

