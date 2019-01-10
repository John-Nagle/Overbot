//
//	voradserver.cpp  --  The VORAD radar server
//
//	John Nagle
//	Team Overbot
//	November, 2003
//
#include <assert.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include "mutexlock.h"
#include "voradserver.h"
#include "voradradar.h"
#include "speedservermsg.h"
#include "tuneable.h"
#include "timeutil.h"
#include "logprint.h"
//
//	Constants
//
//	threatsecs -- must stop if within this many seconds to a collision
//
//	Value is usual DMV recommendation from driver's handbook, hence conservative.
//
const Tuneable k_threatsecs("THREATSECS",1.0, 10.0, 4.0,"Time to collision before e-stop (secs).");	// E-stop if collision expected sooner
//
//	moveminmps  -- minimum meters/sec to consider target moving
//
const float k_moveminmps = 0.25;							// min speed differential for moving target	
const char* k_speedservername = "SPEED";			// speed server name
const char* k_mapservername = "MAP";					// map server name
const double k_clienttimeout = 0.5;							// Client timeout for sending messages						

static bool verbose = false;										// true if verbose mode
static bool sendtargetreports = true;						// true if sending target reports
//
//	class VoradRadarData -- the latest data from the VORAD radar, including its thread
//
class VoradRadarData: public VoradRadar
{
private:
	pthread_t m_commthread;								// communications thread
	int m_fd;															// file descriptor for device
	ost::Mutex m_lock;												// lock against access
	ost::Semaphore m_dataavail;							// wake up waiting program
	bool m_valid;													// data below is valid
	bool m_estop;													// trouble - demand an E-stop
	int m_targetcount;												// fields in last message from VORAD
	uint64_t	m_timestamp;									// timestamp of last VORAD receive
	VoradTargetItem m_targets[VoradServerMsgVDTG::k_maxtargets];	// VORAD target data
	MsgClientPort m_speedclientport;						// for sending E-stop to speed server
	MsgClientPort m_mapclientport;							// for sending target data to map server
private:
	void* commthread();										// display thread
	static void* startcommthread(void* threaddata) { return(reinterpret_cast<VoradRadarData*>(threaddata)->commthread()); }
	virtual void delivererror(int stat, const char* msg);	// deliver error message
	virtual void deliverstartup();										// unit has reset - clear target info
	virtual void delivertargetreport(uint8_t sequence, uint8_t fieldcount,VoradField fields[]);	// must override
	bool converttarget(const VoradTarget& targ, VoradTargetItem& item);	// convert target data, return TRUE if E-stop needed
	float secondstocollision(const VoradTarget& targ);		// seconds to collision
	void sendestop(Fault::Faultcode faultid);			// send emergency stop command
	void sendtargetreport();									// send target report to map server
public:
	int init(const char* voradport);							// initialization
	int getupdate(VoradServerMsgVDTG& repmsg);	// update this message
	VoradRadarData() 
	: m_fd(-1), m_estop(false), m_targetcount(0), m_timestamp(0),
	m_speedclientport(k_speedservername, k_clienttimeout),
	m_mapclientport(k_mapservername, k_clienttimeout)
	{}
	~VoradRadarData()
	{	if (m_fd >=0) ::close(m_fd); }
};
//
//
//	class VoradServer  --  read from an Eaton VORAD radar, process info
//
class VoradServer {
private:
	VoradRadarData m_radar;							// the radar
private:

public:
	VoradServer();													// constructor
	virtual ~VoradServer();									// destructor
	int init(const char* voradport);							// initialization
	void serverRQ(int rcvid, const VoradServerMsgVDRQ& reqmsg);
	void idle();
};
//
//	Class VoradRadarData implementation
//
//	commthread -- reads from VORAD, updates latest target data
//
void* VoradRadarData::commthread()
{	assert(m_fd >= 0);																// file should be open
	for (;;)
	{	int stat = waitforinput(m_fd);												// this gets us callbacks
		if (stat)
		{	perror("VORAD serial port failed");								// fails
			exit(1);
		}
	}
}			
//	
//	
void VoradRadarData::delivererror(int stat, const char* msg)	// deliver error message
{	if (verbose) { logprintf("VORAD error: %s\n",msg); } // note error
	ost::MutexLock lok(m_lock);	
	m_valid = false;																	// no target, but not valid either
}
//
//	deliverstartup  -- callback indicating unit has reset, clear target info
//
void VoradRadarData::deliverstartup() 								
{	ost::MutexLock lok(m_lock);													// lock
	m_valid = false;																// no target, but not valid either
}

//	
//	delivertargetreport -- a new target report has arrived.
//
//	Find the most threatening target and set it for the next request.
//
void VoradRadarData::delivertargetreport(uint8_t sequence, uint8_t fieldcount,VoradField fields[])
{	ost::MutexLock lok(m_lock);													// lock
	m_targetcount = 0;
	m_estop = false;																	// so far, so good
	for (int i=0; i<fieldcount; i++)												// for all targets
	{	const VoradField& field = fields[i];									// get field
		if (	field.m_targetid == 0) continue;									// if 0, is a drop message. Ignore
		if (m_targetcount >= VoradServerMsgVDTG::k_maxtargets)		// too many targets
		{	m_valid = false;															// VORAD error. Should not happen
			logprintf("Too many targets. VORAD error.\n"); 
			sendestop(Fault::radar);												// definitely a good reason to stop
			return;
		}
		//	Evaluate target.
		m_estop |= converttarget(field.m_targetinfo, m_targets[m_targetcount++]);	// convert target info, detect trouble
	}
	m_timestamp = gettimenowns();											// timestamp target data
	m_valid = true;																		// target data is valid
	if (sendtargetreports)															// if sending to map server enabled enabled
	{	sendtargetreport();	}														// tell map server about targets
}
//
//	converttarget  --  convert target from VORAD internal format to message format
//
//	Also trip E-stop if really threatening
//
bool VoradRadarData::converttarget(const VoradTarget& targ, VoradTargetItem& item)
{
	//	Calculate target info
	float rangem = targ.rangem();												// range in meters
	float velocitymps = targ.velocitymps();									// range rate in meters
	float bearingrad = targ.azimuthrad();									// azimuth in radians (positive is right)
	float sinbearingrad = sin(bearingrad);
	float cosbearingrad = cos(bearingrad);
	item.m_targ_y = -sinbearingrad * rangem;							// compute vector to target
	item.m_targ_x = cosbearingrad * rangem;							// X is ahead, Y is left
	item.m_targ_rrange = velocitymps;										// range to target
	//	Decide whether target is moving
	item.m_movingtarget = false;												// assume non-moving target here.
	//	***MORE*** need moving target calculation
	////float approachrate = cosbearingrad * velocitymps;				// approach rate
	////item.m_movingtarget = fabs (approachrate + item.m_speed) > k_moveminmps;	// if moving, so note
	//	Decide if serious trouble.
	float secstocollision = secondstocollision(targ);					// calc seconds to collision
	if (secstocollision < 0.1) secstocollision = 0.1;						// prevent divide by zero
	item.m_collisionthreatlev = k_threatsecs / secstocollision;	// bigger threat levels are worse
	if (item.m_collisionthreatlev < 1.0) return(false);					// OK, no E-stop
	float bearingdeg = bearingrad*(180/M_PI);							// bearing in degrees
	logprintf("COLLISION IMMINENT in %1.1fs: target at %1.2f m, speed %1.2f m/s, bearing %1.1f deg. Trigger E-stop.\n",
		secstocollision, rangem, velocitymps, bearingdeg); 			// Big trouble. Slam on brakes.
	sendestop(Fault::nearcollision);											// trip E-stop
	return(true);
}
//
//	init -- startup
//
int VoradRadarData::init(const char* voradport)
{	if (m_fd >= 0)
	{	::close(m_fd); m_fd = -1; }													// close if already open
	m_fd = ::open(voradport,O_RDWR);										// try to open
	if (m_fd < 0) return(errno);													// if fail
	int stat = setupserial(m_fd);													// set serial modes
	if (stat) return(stat);																// fails if error
	stat = pthread_create(&m_commthread,0,startcommthread,this);	// start the display thread
	if (stat) return(errno);
	return(EOK);																			// success
}
//
//	getupdate  -- update a server request with the latest data
//
//	Just copies whatever the VORAD last reported, if anything.
//
int VoradRadarData::getupdate(VoradServerMsgVDTG& replymsg)
{	ost::MutexLock lok(m_lock);													// lock
	if (!isup()) return(EIO);															// fails if VORAD not ready
	if (!m_valid) return(EIO);														// fails if VORAD returned bad data
	replymsg.m_estop = m_estop;												// do we need an E-stop?
	replymsg.m_timestamp = m_timestamp;								// copy timestamp
	replymsg.m_targetcount = m_targetcount;							// copy target count
	assert(m_targetcount <= VoradServerMsgVDTG::k_maxtargets);		// check target count
	for (int i=0; i<m_targetcount; i++)										// for all targets
	{	replymsg.m_targets[i] = m_targets[i]; }							// copy target info
	return(EOK);																			// success
}
//
//	secondstocollision -- calculate seconds to collision for this target
//
//	Smaller values are more threatening.
//
//	Value should be seconds to collision.
//
//
float VoradRadarData::secondstocollision(const VoradTarget& target)
{
	//	Get range and closing rate for this target
	float rangem = target.rangem();							// range in meters
	float closingmps = target.velocitymps(); 				// closing rate in meters/sec
	if (rangem < 0.1) rangem = 0.1;							// very close range
	if (fabs(closingmps) < 0.1) closingmps = 0.1;		// assume 10cm/sec if no range rate read
	if (closingmps < 0) closingmps = 0.1;					// assume 10cm/sec closing if actually going away
	float secstohit = rangem / closingmps;					// secs to collision
	return(secstohit);													// returns seconds to collision
}
//
//	sendestop -- send emergency stop message to speed server
//
//	This is a backup to the higher level systems.
//
void VoradRadarData::sendestop(Fault::Faultcode faultid)	// send emergency stop command
{
	MsgSpeedStop msg;														// stop message
	msg.m_msgtype = MsgSpeedStop::k_msgtype;				// this is a stop message
	msg.m_fault = faultid;													// report type of fault
	int stat = m_speedclientport.MsgSend(msg);				// send to speed server
	if (stat < 0)																	// report error, but can't do much
	{	logprintf("ERROR sending E-stop command to speed server: %s\n",strerror(errno));
	}
}
//
//	sendtargetreport -- send ordinary target report to map server
//
void VoradRadarData::sendtargetreport()
{	VoradServerMsgVDTG msg;											// target message
	msg.m_msgtype = VoradServerMsgVDTG::k_msgtype;	// this is a target
	int err = getupdate(msg);												// get latest info
	if (err)																			// if we have a VORAD problem
	{	sendestop(Fault::radar);											// report a radar fault
		return;
	}
	int stat = m_mapclientport.MsgSend(msg);					// send to speed server
	if (stat < 0)																	// report error, but can't do much
	{	logprintf("ERROR sending target info to map server: %s\n",strerror(errno));
	}
}
//
//	Constructor
//
VoradServer::VoradServer()
{
}
//	
//	Destructor
//
VoradServer::~VoradServer()
{
}
//
//	init -- startup
//
int VoradServer::init(const char* voradport)
{	return(m_radar.init(voradport));
}

//
//	idle -- idle for 1 second, update display
//
void VoradServer::idle()
{
}
//	
//	serverTG  --  request from VORAD server
//
//	Replies with a VDTG message.
//
void VoradServer::serverRQ(int rcvid, const VoradServerMsgVDRQ& rqmsg)
{	// ***MORE*** need to block for next update if none available.
	VoradServerMsgVDTG replymsg;									// reply message built here
	int stat = m_radar.getupdate(replymsg);						// update message
	if (stat) 
	{	MsgError(rcvid, stat);												// return error
		return;
	}
	MsgReply(rcvid,replymsg);											// otherwise reply
}

//
//	runserver -- run as a server
//
//	This server accepts request messages, does some computation, and returns the result.
//
void runserver(const char* voradport)															
{	VoradServer vorad;														// the vorad
	//	Initialize vorad
	logprintf("Opening Vorad \"%s\"\n",voradport);						// Log vorad opened
	int stat = vorad.init(voradport);										// try to open vorad
	if (stat != EOK)															// if unsuccessful open
	{																				
		perror("Unable to open vorad port");
		exit(1);																	// fails
	}

	//	The server normally has the name given the program by the watchdog file. This is in the env. variable "ID"
	MsgServerPort serverport(1.0);								// define message port, 1 second timeout
	if (verbose) serverport.setverbose();						// more talkative
	stat = serverport.ChannelCreate();							// create a channel, tell watchdog about it
	if (stat) 
	{	perror("ChannelCreate failed in server"); 
		serverport.Dump();
		exit(1);																	// fails
	}
	logprintf("Server started.\n");
	for (;;)																		// forever
	{	VoradServerMsg msgin;										// area for incoming msg
		_msg_info msginfo;												// aux info
		int rcvid = serverport.MsgReceive(msgin,&msginfo);	// get msg
		if (rcvid < 0)															// if error
		{	if (errno == ETIMEDOUT)									// if timeout
			{	vorad.idle();													// do idle actions
				continue;
			}
			fflush(stdout); perror("MsgReceive failed in server");				// fails
			sleep(1);															// avoid tight loop if repeated trouble
			continue;															// fails
		}
		if (rcvid == 0)														// pulse
		{	printf("Server received a pulse.\n");					// pulses don't require a reply
			continue;
		}
		//	We have received a message
		switch (msgin.m_vdrq.m_msgtype) {					// fan out on type
		case VoradServerMsgVDRQ::k_msgtype:				// request with text
		{	if (msginfo.msglen != sizeof(msgin.m_vdrq)) { MsgError(rcvid,EBADRPC); break; }	// msg size check
			vorad.serverRQ(rcvid,msgin.m_vdrq);				// handle msg
			break;
		}
			
		default:																	// unknown, fails
			MsgError(rcvid,EBADRPC);									// reply with result code only
			break;
		}
		////serverport.watchdogreset();								// tell the watchdog we are still alive
	}
}
//
//	usage  -- print usage and exit
//
static void usage()
{	printf("Usage: voradserver [options] voradport\n");
	printf("  Options:  -v verbose\n");
	printf("					-n disable sending target reports to map server.\n");
	exit(1);																		// fails
}
//
//	Main program
//
//	Usage: voradserver [options] voradname
//
int main(int argc, const char* argv[])
{	const char* voradport = 0;											// no vorad name yet
	//	Parse input arguments
	for (int i=1; i<argc; i++)											// for all args
	{	const char* arg= argv[i];										// this arg
		if (arg[0] == '-')													// if flag argument
		{	switch(arg[1])	{												// interpret flags
			case 'v': verbose = true;	 break;						// set verbose mode
			case 'n':  sendtargetreports = false; break;		// turn off sending target reports to map server
			default: usage();												// bad call, fails
			}
			continue;															// next arg
		}
		//	Not flag, must be file arg
		if (voradport) usage();
		voradport = arg;
	}
	if (!voradport) usage();												// must have radar serial port name
	//	go
	if (!sendtargetreports)
	{	logprintf("VORAD reports to map server disabled.\n");	}	// note suppression
	runserver(voradport);												// run  vorad server
	return(0);																	// success
}