/* -*- indent-tabs-mode:T; c-	basic-offset:8; tab-width:8; -*- vi: set ts=8:
 *
 * Frank Zhang 4/12/05 added contents in handle_request()
 * Khian Hao Lim
 * Oct 6 03
 * Overbot
 *
 * Entry point for server thread for GPSINS Server
 * Sits in a qnx specific receive, reply loop
 */

#include "thread.h"
#include "fusednav.h"

static void handle_request(int rcvid, Kalman_Filter* kf)
{

	struct GPSINSMsgRep rep;
	kf->mFusedNav->copyState(rep);

	int err = MsgReply(rcvid, rep);
	if (err) {
		perror("server-thread: MsgReply in handle_request");
	}
	
}

//
//	handle_basepoint -- handle a request to change the basepoint
//
static void handle_basepoint(int rcvid, const GPSINSBasepoint& msg, Kalman_Filter* kf)
{
	//	Changing the basepoint means the filter has to be reset, so this isn't done yet.
	Vector<3> bpllh;
	bpllh[0] = msg.llh[0];
	bpllh[1] = msg.llh[1];
	bpllh[2] = msg.llh[2];
	kf->mGPS.setBasePoint(bpllh);
	MsgError(rcvid, EOK);
}
//
//	handle_getbasepoint -- handle a request to change the basepoint
//
static void handle_getbasepoint(int rcvid, const GPSINSGetBasepoint& msg, Kalman_Filter* kf)
{
	const Vector<3>& basept = kf->mGPS.mLLHZero;	// get basepoint (lat(rad), long(rad), height) XYZ coord system
	GPSINSGetBasepointRep reply;
	reply.llh[0] = C_RAD2DEG*basept[0];		// convert to degrees for message
	reply.llh[1] = C_RAD2DEG*basept[1];
	reply.llh[2] = basept[2];			// height in meters, no conversion
	MsgReply(rcvid, reply);
}

//TODO
//we really would need a lock on the data structs
void *
server_thread(void * args_v)		
{
	
	printf("Starting server-thread.\n");

	Kalman_Filter*         kf    = (Kalman_Filter*) args_v;

	
	// define message port, no timeout
	MsgServerPort serverport(0.0);
	// create a channel, tell watchdog about it
	int stat = serverport.ChannelCreate();
	if (stat)
	{   
		perror("ChannelCreate failed in server");
		exit(1); 																				// fatal
		return(0);
	}

	else {
		for (;;) {
			GPSINSMsg  msgin;															// union of all allowed messages

			int rcvid = serverport.MsgReceive(msgin);
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

			if (verbose) {
				printf("Server received a message\n");
			}
			//	Fan out on message type
			switch (msgin.m_req.m_msgtype) {
			case GPSINSMsgReq::k_msgtype:											// request fix
				handle_request(rcvid, kf);
				break;
				
			case GPSINSBasepoint::k_msgtype:										// basepoint change
				handle_basepoint(rcvid, msgin.m_basepoint, kf);		// do a basepoint change
				break;
				
			case GPSINSGetBasepoint::k_msgtype:									// get basepoint
				handle_getbasepoint(rcvid, msgin.m_getbasepoint,kf);	// just retrieves current basepoint
				break;
				
			default:
				MsgError(rcvid, EINVAL);													// some invalid request
				break;
			}
		}
	}	

}
