/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

/* Standard headers */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

/* Local headers */
#include "ablibs.h"
#include "global.h"
#include "abimport.h"
#include "proto.h"
#include "../../../common/include/messaging.h"
#include "../../../common/include/remotemessaging.h"
#include "../../../common/include/gpsins_messaging.h"

#include <iostream>
using namespace std;
//	Connects to GPSINS server using an instance of remotemsgserver.
//	Must be running both GPS and an instance of remotemsgserver started like this:
//
//	ID=RGPSINS remotemsgserver GPSINS
//
const char *servername = "RGPSINS";
const char* servernode = "gcrear0";		// which machine
static RemoteMsgClientPort clientport(servernode,servername, 0.2);			// connects to remote server

void
updateAnnounce()
{
	struct timeval nowTime;
	gettimeofday(&nowTime, NULL);
	double secs = getNumSecsPassed(startTime, nowTime);

	if (poses.size() < 2) {
		return;
	}
	GPSINS_MSG::Err errOneBack = poses[poses.size() -2].err;
	GPSINS_MSG::Err err = poses[poses.size() -1].err;

	GPSINS_MSG::SolStatusEnum  posStatOneBack = poses[poses.size() -2].posStat;
	GPSINS_MSG::SolStatusEnum  posStat = poses[poses.size() -1].posStat;
	
	GPSINS_MSG::PosVelTypeEnum  posTypeOneBack = poses[poses.size() -2].posType;
	GPSINS_MSG::PosVelTypeEnum  posType = poses[poses.size() -1].posType;

	if (errOneBack != err) {
		char text[256];
		snprintf(text, sizeof(text),"%f secs: Error changed from %s to %s\n",
					secs, decodeErr(errOneBack), decodeErr(err));
		
		//add text to the end of the text screen
		PtTextModifyText(ABW_textAnnounce, 0, 0, -1, text, strlen(text));
		
	}	

	if (posStatOneBack != posStat) {
		char text[256];
		snprintf(text, sizeof(text),"%f secs: Position Status changed from %s to %s\n",
					secs, decodePosStat(posStatOneBack), decodePosStat(posStat));
		
		//add text to the end of the text screen
		PtTextModifyText(ABW_textAnnounce, 0, 0, -1, text, strlen(text));
		
	}	

	if (posTypeOneBack != posType) {
		char text[256];
		snprintf(text, sizeof(text),"%f secs: Position Type changed from %s to %s\n",
					secs, decodePosType(posTypeOneBack), decodePosType(posType));
		
		//add text to the end of the text screen
		PtTextModifyText(ABW_textAnnounce, 0, 0, -1, text, strlen(text));
		
	}	


}

void
updateTrendUnc()
{
	//we will display as number of cm
	short unc[3];
	
	int posSize = poses.size();
	
	if (posSize > 0) {
		unc[0] = (short)(poses.back().unc[0] * 100);
		unc[1] = (short)(poses.back().unc[1] * 100);
		unc[2] = (short)(poses.back().unc[2] * 100);

		PtArg_t args;

		PtSetArg(&args, Pt_ARG_TREND_DATA, unc, 3);
		PtSetResources(ABW_trendUnc, 1, &args);
	}

}

void
updateGUI()
{
		if(poses.size() == 0) { return; }

		struct GPSINSMsgRep & rep = poses.back();

		//TODO
		//i really don't understand why we must do this
		double llh[3] = {rep.llh[0], rep.llh[1], rep.llh[2]};
		double vel[3] = {rep.vel[0], rep.vel[1], rep.vel[2]};
		double pos[3] = {rep.pos[0], rep.pos[1], rep.pos[2]};
		double unc[3] = {rep.unc[0], rep.unc[1], rep.unc[2]};
		double acc[3] = {rep.acc[0], rep.acc[1], rep.acc[2]};
		double pqr[3] = {rep.pqr[0], rep.pqr[1], rep.pqr[2]};
		double rpy[3] = {rep.rpy[0], rep.rpy[1], rep.rpy[2]};

		//update values on valuators
		PtSetResource (ABW_valLat, Pt_ARG_NUMERIC_VALUE, &llh[0], 0);
		PtSetResource (ABW_valLong, Pt_ARG_NUMERIC_VALUE, &llh[1], 0);
		PtSetResource (ABW_valHeight, Pt_ARG_NUMERIC_VALUE, &llh[2], 0);
		
		PtSetResource (ABW_valPosX, Pt_ARG_NUMERIC_VALUE, &pos[0], 0);
		PtSetResource (ABW_valPosY, Pt_ARG_NUMERIC_VALUE, &pos[1], 0);
		PtSetResource (ABW_valPosZ, Pt_ARG_NUMERIC_VALUE, &pos[2], 0);
		
		PtSetResource (ABW_valVelX, Pt_ARG_NUMERIC_VALUE, &vel[0], 0);
		PtSetResource (ABW_valVelY, Pt_ARG_NUMERIC_VALUE, &vel[1], 0);
		PtSetResource (ABW_valVelZ, Pt_ARG_NUMERIC_VALUE, &vel[2], 0);		
		
		PtSetResource (ABW_valUncX, Pt_ARG_NUMERIC_VALUE, &unc[0], 0);
		PtSetResource (ABW_valUncY, Pt_ARG_NUMERIC_VALUE, &unc[1], 0);
		PtSetResource (ABW_valUncZ, Pt_ARG_NUMERIC_VALUE, &unc[2], 0);

		PtSetResource (ABW_valAccX, Pt_ARG_NUMERIC_VALUE, &acc[0], 0);
		PtSetResource (ABW_valAccY, Pt_ARG_NUMERIC_VALUE, &acc[1], 0);
		PtSetResource (ABW_valAccZ, Pt_ARG_NUMERIC_VALUE, &acc[2], 0);

		PtSetResource (ABW_valRollRate, Pt_ARG_NUMERIC_VALUE, &pqr[0], 0);
		PtSetResource (ABW_valPitchRate, Pt_ARG_NUMERIC_VALUE, &pqr[1], 0);
		PtSetResource (ABW_valYawRate, Pt_ARG_NUMERIC_VALUE, &pqr[2], 0);

		PtSetResource (ABW_valRoll, Pt_ARG_NUMERIC_VALUE, &rpy[0], 0);
		PtSetResource (ABW_valPitch, Pt_ARG_NUMERIC_VALUE, &rpy[1], 0);
		PtSetResource (ABW_valYaw, Pt_ARG_NUMERIC_VALUE, &rpy[2], 0);
		
		//update error status, solstat and postype
		PtSetResource(ABW_textErr, Pt_ARG_TEXT_STRING, decodeErr(rep.err), 0);
		PtSetResource(ABW_textPosStat, Pt_ARG_TEXT_STRING, decodePosStat(rep.posStat), 0);
		PtSetResource(ABW_textPosType, Pt_ARG_TEXT_STRING, decodePosType(rep.posType), 0);
}

void
debugGui()
{
	//for testing, we fake some readings;
	static struct GPSINSMsgRep fakeRep;
	fakeRep.llh[0] += 0.01;
	fakeRep.llh[1] += 0.02;
	fakeRep.llh[2] += 0.03;
	fakeRep.vel[0] += 0.01;
	fakeRep.vel[1] += 0.02;
	fakeRep.vel[2] += 0.03;
	fakeRep.pos[0] += 0.01;
	fakeRep.pos[1] += 0.02;
	fakeRep.pos[2] += 0.03;
	fakeRep.unc[0]+= 0.01;
	fakeRep.unc[1] += 0.02;
	fakeRep.unc[2] += 0.03;
	fakeRep.acc[0]+= 0.01;
	fakeRep.acc[1] += 0.02;
	fakeRep.acc[2] += 0.03;
	fakeRep.pqr[0] += 0.01;
	fakeRep.pqr[1] += 0.02;
	fakeRep.pqr[2] += 0.03;
	fakeRep.rpy[0] += 0.1;
	fakeRep.rpy[1] += 0.2;
	fakeRep.rpy[2] += 0.3;

	fakeRep.posType = fakeRep.posType == GPSINS_MSG::NONE ?
									GPSINS_MSG::OMNISTAR_HP:
									GPSINS_MSG::NONE;
	fakeRep.posStat = fakeRep.posStat == GPSINS_MSG::SOL_COMPUTED? GPSINS_MSG::INSUFFICIENT_OBS : 
								GPSINS_MSG::SOL_COMPUTED;
	
	poses.push_back(fakeRep);	

	unsigned int * bufSize;
	PtGetResource(ABW_valBufferSize, Pt_ARG_NUMERIC_VALUE, &bufSize, 0);
	if (poses.size() > *bufSize && *bufSize > 0) {
		poses.erase(poses.begin());
	}

}

double
getNumSecsPassed(struct timeval start, struct timeval now)
{
	double secs = 0;
	secs += now.tv_sec - start.tv_sec;
	secs += (now.tv_usec - start.tv_usec) / 1000000.0;
	return secs;
}

int
myTimerActivate( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{
		
	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

   	struct GPSINSMsgReq req;
   	req.m_msgtype = GPSINSMsgReq::k_msgtype;		// set request type
	struct GPSINSMsgRep rep;

	//send request to gpsins server and update pos
	int err = clientport.MsgSend(req, rep);

	//comment out later
	//debugGui();
		
	if (err < 0) {																	// if fail
		//	Trouble. Really should put this message into the GUI.
		char s[200];
		snprintf(s,sizeof(s),"Error connecting to server \"%s\" on node \"%s\": %s\n", 
			servername, servernode, strerror(errno));
		//	Add text to the end of the text screen
		PtTextModifyText(ABW_textAnnounce, 0, 0, -1, s, strlen(s));
		PtSetResource(ABW_textErr, Pt_ARG_TEXT_STRING, "No connection", 0);
	}
	else {
	
		poses.push_back(rep);		

		unsigned int * bufSize;
		PtGetResource(ABW_valBufferSize, Pt_ARG_NUMERIC_VALUE, &bufSize, 0);
		if (poses.size() > *bufSize && *bufSize > 0) {
			poses.erase(poses.begin());
		}
	}

	//comment out later
	//err = 0;
	
	if (mPause || (err < 0)) {
		return Pt_CONTINUE;
	}
	
	updateGUI();
	updateTrendUnc();
	updateAnnounce();
	
	//we will need to damage the widget for redraw to happen			
	if (ABW_positionView) {
		PtDamageWidget(ABW_positionView);
	}
	if (ABW_heading) {
		PtDamageWidget(ABW_heading);
	}
	if (ABW_artHorizon) {
		PtDamageWidget(ABW_artHorizon);
	}
	
	PtFlush();
	return( Pt_CONTINUE );

	}

