//
//	simplesonar.cpp  --  The simple sonar server
//
//	John Nagle
//	Team Overbot
//	August, 2004
//
#include <assert.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include "mutexlock.h"
#include "speedservermsg.h"
#include "controller.h"
#include "tuneable.h"
#include "logprint.h"
#include "simplesonar.h"
//
//	Configuration constants - TEMP values
//
const char k_speedservername[] = "SPEED";									// the speed server
const char k_mapservername[] = "MAP";											// the map server 
const float k_clienttimeout = 0.100;	
const char k_forwardcontrollername[] = "gctransmission";				// controller that has inputs for front sonars
const char k_reversecontrollername[] = "gctransmission";				// controller that has inputs for rear sonars
const int k_forwardport = 1;
const int k_reverseport = 2;
//
//	class SimpleSonar -- the latest data from the VORAD radar, including its thread
//
class SimpleSonar
{
private:
	Controller m_fwdsonar;										// for reading forward sonar	
	Controller m_revsonar;										// for reading reverse sonar
	MsgClientPort m_speedclientport;						// for sending E-stop to speed server
	MsgClientPort m_mapclientport;							// for sending target data to map server
	bool m_verbose;												// true if verbose
private:
	void sonarfault(const char* msg);					// report a sonar fault
	void sonardata(Fault::Faultcode fault, bool fwdobstacle, bool revobstacle);	// report sonar data
	void sendestop(Fault::Faultcode faultid);			// send emergency stop command
	void sendtargetreport();									// send target report to map server
public:
	SimpleSonar() 
	: m_fwdsonar(k_forwardcontrollername, true),	// read only controller access
	m_revsonar(k_reversecontrollername, true),
	m_speedclientport(k_speedservername, k_clienttimeout),
	m_mapclientport(k_mapservername, k_clienttimeout)
	{}
	~SimpleSonar()
	{	 }
	void runclient(bool verbose);										// run as a client
};
//
//	sendestop -- send emergency stop message to speed server
//
//	This is a backup to the higher level systems.
//
void SimpleSonar::sendestop(Fault::Faultcode faultid)	// send emergency stop command
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
//	sonarfault -- report a sonar fault
//
void SimpleSonar::sonarfault(const char* msg)
{
	logprintf("Sonar fault: %s\n", msg);								// log problem
	sonardata(Fault::sonar, false, false);								// report to map server
}
void SimpleSonar::sonardata(Fault::Faultcode faultid, bool fwdobstacle, bool revobstacle)
{
	SonarObstacleMsgReq msg;											// stop message
	msg.m_msgtype = SonarObstacleMsgReq::k_msgtype;	// this is a sonar data message
	msg.m_fault = faultid;													// report type of fault
	msg.m_frontobstacle = fwdobstacle;							// report front and rear obstacle status
	msg.m_rearobstacle = revobstacle;
	int stat = m_mapclientport.MsgSend(msg);					// send to speed server
	if (stat < 0)																	// report error, but can't do much
	{	logprintf("ERROR sending sonar data command to map server: %s\n",strerror(errno));
	}
	if (m_verbose || fwdobstacle || revobstacle)
	{	logprintf("Near obstacles: %s %s\n", fwdobstacle ? "(FRONT) " : "", revobstacle ? "(REAR)" : ""); }
}
//
//	runclient -- run as a client
//
//	Runs forever
//
void SimpleSonar::runclient(bool verbose)
{	m_verbose = verbose;															// set verbosity
	for (;;)
	{	//	Try to stay connected
		usleep(100000);																// wait 100ms.
		if (k_forwardport > 0)														// if forward sonar configured
		{	bool connected = m_fwdsonar.Connected();
			if (!connected)
			{	m_fwdsonar.Connect();	}
		}
		if (k_reverseport > 0)														// if reverse sonar configured
		{	bool connected = m_revsonar.Connected();
			if (!connected)
			{	m_revsonar.Connect();	}
		}
		//	Read foward and reverse sonars from controllers
		bool fwdobstacle = false;													// no obstacles yet
		bool revobstacle = false;
		if (k_forwardport > 0)														// if forward sonar configured
		{	Controller::Err err = m_fwdsonar.DigitalInputGet(k_forwardport, &fwdobstacle);
			if (err != Controller::ERR_OK)											// check for controller fault
			{	sonarfault(Controller::ErrMsg(err));							// handle fault	
				continue;
			}
		}
		if (k_reverseport > 0)														// if reverse sonar configured
		{	Controller::Err err = m_revsonar.DigitalInputGet(k_reverseport, &revobstacle);
			if (err != Controller::ERR_OK)											// check for controller fault
			{	sonarfault(Controller::ErrMsg(err));							// handle fault	
				continue;
			}
		}	
		sonardata(Fault::none, fwdobstacle, revobstacle);			// handle data
	}
}
//
//	usage  -- print usage and exit
//
static void usage()
{	printf("Usage: simplesonar [options]\n");
	printf("  Options:  -v verbose\n");
	exit(1);																		// fails
}
//
//	Main program
//
//	Usage: simplesonar [options] 
//
int main(int argc, const char* argv[])
{	bool verbose = false;
	//	Parse input arguments
	for (int i=1; i<argc; i++)											// for all args
	{	const char* arg= argv[i];										// this arg
		if (arg[0] == '-')													// if flag argument
		{	switch(arg[1])	{												// interpret flags
			case 'v': verbose = true;	 break;						// set verbose mode
			default: usage();												// bad call, fails
			}
			continue;															// next arg
		}
		//	Not flag, must be file arg
		usage();
	}
	SimpleSonar sonars;													// the sonars
	sonars.runclient(verbose);										// run as client
	return(1);																	// not reached, we hope
}