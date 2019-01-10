/////////////////////////////////////////////////////////////////////////////
//
//    File: controller.cc
//
//    Usage:
//        See controller.h.
//
//    Description:
//        See controller.h.
//
//        The velocities and accelerations returned from the controller
//        depend on the sample period.  If the sample period is 1000 usec,
//        the velocity is in quadrature counts/sec, and the accelerations
//        (and decelerations) are in quadrature counts/sec^2.  If the sample
//        period is set to a value different than 1000 usec, the velocities
//        and accelerations scale appropriately.  For example, if the sample
//        period was set to 2000 usec, the velocities will be cut in half
//        and the accelerations in fourths.
//
//        For sample period in usec,
//            actual_vel   = recv_vel   / (sample period/1000);
//            actual_accel = recv_accel / (sample period/1000)^2;
//
//        (This is all infinitely simpler than the MVP conversions!)
//
//        Based on mvp.cc and mvpserver.cc code of August 2003.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        January, 2004
//
/////////////////////////////////////////////////////////////////////////////

#include "controller.h"
#include "logprint.h"
using namespace std;
//
//	constants
//
const int k_downloadtimeus = 3000000;			// allow up to 3 seconds for download and list commands

//	Default timeout value for controller requests
//	Must be short enough to tolerate 2 or 3 lost packets before the watchdog kills everything.
//	Must be long enough that none of the common commands take longer.
const int  CONTROLLER_PORTTIMEOUT = 50000;	// usec

//
//	class Controller
//
//	Constructor
//
//	Will throw if hostname is invalid.
//
//	Does not make the connection. "Connect" must be called.
//
Controller::Controller(const char *hostname, bool isreadonly)
: e(hostname, 0, CONTROLLER_PORTTIMEOUT),		// intialize Ethernet socket
	verbose(false), 												// not verbose
	readonly(isreadonly)										// not read only
{ 
    // set sample period to default
    samplePeriod = CONTROLLER_DEF_SAMPPER;
    
    // initialize conversion factors (default slope=1.0, offset=0.0)
    convSet(&encoder, "quad counts", 1.0, 0.0);
    convSet(&encoderAux, "quad counts", 1.0, 0.0);  
    convSet(&analogInput1, "volts", 1.0, 0.0);
    convSet(&analogInput2, "volts", 1.0, 0.0);
    convSet(&torque, "volts", 1.0, 0.0);
    
    // initialize command database
    cmdDBInit();
}
//
//	Connect  -- connect Ethernet connection
//
int Controller::Connect(double timeout)
{	if (Connected()) return(0);								// OK if already connected
	return(e.Connect(timeout));								// do the connection
}
//
//	Shutdown  -- disconnect Ethernet connection
//
//	Note that this does NOT presently shut down motion from the controller.
//
int Controller::Shutdown()
{	if (!Connected()) return(0);								// OK if already disconnected
	return(e.Shutdown());										// do the disconnection
}
//
//	Connected -- true if connected
//
bool Controller::Connected() const
{	return(e.Connected());	}									// pass down to Ethernet level
//
//	Datagram mode -- sets system to use UDP instead of TCP
//
//	UDP is useful over wireless links, because our timeouts can be better matched to the real time problem.
//
//	Passed through to Ethernet level
//
bool Controller::DatagramMode()
{	return(e.DatagramMode()); 	}
void Controller::DatagramMode(bool iDatagram)
{	e.DatagramMode(iDatagram); }

//
//	SetReadOnly -- set/clear read only mode
//
//	In read only mode, only operations that don't affect what the controller is doing
//	are allowed. This is for debug programs that observe only.
//
Controller::Err Controller::SetReadOnly(bool isreadonly)
{	readonly = isreadonly;										// set read only mode
	return(ERR_OK);
}

Controller::Err Controller::MotorOff()
{	
	return instructionSend(Controller::CMD_MO, 0, (float *) NULL);
}

Controller::Err Controller::DefinePosnSet(float posn)
{
	int quadCounts;

    // convert position into encoder quad counts
    quadCounts = roundf (posn * encoder.slope + encoder.offset);

	return instructionSend(Controller::CMD_DP, quadCounts, (float *) NULL);
}

Controller::Err Controller::DefinePosnAuxSet(float posn)
{
	int quadCounts;

    // convert position into encoder quad counts
    quadCounts = roundf (posn * encoderAux.slope + encoderAux.offset);

	return instructionSend(Controller::CMD_DE, quadCounts, (float *) NULL);
}

Controller::Err Controller::LimitPosnMinSet(float posnMin)
{
    int quadCounts;

    // convert position into encoder quad counts
    quadCounts = roundf (posnMin * encoder.slope + encoder.offset);

    return instructionSend(Controller::CMD_BL, quadCounts, (float *) NULL);
}

Controller::Err Controller::LimitPosnMinGet(float *posnMin)
{
    int quadCounts;
    Controller::Err stat;

    stat = instructionSend(Controller::CMD_BL, 0, &quadCounts);

    // convert encoder quad counts into position
    *posnMin = (float(quadCounts) - encoder.offset) / encoder.slope;

    return stat;
}

Controller::Err Controller::LimitPosnMaxSet(float posnMax)
{
    int quadCounts;

    // convert position into encoder quad counts
    quadCounts = roundf (posnMax * encoder.slope + encoder.offset);

    return instructionSend(Controller::CMD_FL, quadCounts, (float *) NULL);
}

Controller::Err Controller::LimitPosnMaxGet(float *posnMax)
{
    int quadCounts;
    Controller::Err stat;

    stat = instructionSend(Controller::CMD_FL, 0, &quadCounts);

    // convert encoder quad counts into position
    *posnMax = (float(quadCounts) - encoder.offset) / encoder.slope;

    return stat;
}

Controller::Err Controller::LimitPosnErrSet(float posnErr)
{
    int quadCounts;

    // convert position into encoder quad counts
    quadCounts = roundf (posnErr * encoder.slope + encoder.offset);

    return instructionSend(Controller::CMD_ER, quadCounts, (float *) NULL);
}

Controller::Err Controller::LimitPosnErrGet(float *posnErr)
{
    int quadCounts;
    Controller::Err stat;

    stat = instructionSend(Controller::CMD_ER, 0, &quadCounts);

    // convert encoder quad counts into position
    *posnErr = (float(quadCounts) - encoder.offset) / encoder.slope;

    return stat;
}

Controller::Err Controller::ActualPosnGet(float *posn)
{
    int quadCounts;
    Controller::Err stat;

    stat = instructionSend(Controller::CMD_TP, 0, &quadCounts);
    
    // convert encoder quad counts into position
    *posn = (float(quadCounts) - encoder.offset) / encoder.slope;
    
    return stat;
}

Controller::Err Controller::ActualPosnRefGet(float *posnRef)
{
    int quadCounts;
    Controller::Err stat;

    stat = instructionSend(Controller::CMD_RP, 0, &quadCounts);

    // convert encoder quad counts into position
    *posnRef = (float(quadCounts) - encoder.offset) / encoder.slope;

    return stat;
}

Controller::Err Controller::ActualPosnErrorGet(float *posnErr)
{
	int quadCounts;
    Controller::Err stat;

    stat = instructionSend(Controller::CMD_TE, 0, &quadCounts);

    // convert encoder quad counts into position
    *posnErr = (float(quadCounts) - encoder.offset) / encoder.slope;

    return stat;
}
//
//	ActualPosnAuxGetUnscaled  -- get position, unscaled
//
//	Used for odometer applications where the encoder value wraps around.
//
Controller::Err Controller::ActualPosnAuxGetUnscaled(int *posnAux)		
{
    return(instructionSend(Controller::CMD_TD, 0, posnAux));
}
Controller::Err Controller::ActualPosnAuxGet(float *posnAux)
{
	int quadCounts;
    Controller::Err stat = ActualPosnAuxGetUnscaled(&quadCounts);
    // convert encoder quad counts into position
    *posnAux = (float(quadCounts) - encoderAux.offset) / encoderAux.slope;
    return stat;
}

Controller::Err Controller::ActualVelAvgGet(float *vel)
{
    int quadCountsPerSec;
    Controller::Err stat;

    stat = instructionSend(Controller::CMD_TV, 0, &quadCountsPerSec);

    // convert encoder quad counts/sec into velocity
    *vel = float(quadCountsPerSec) * (samplePeriod/CONTROLLER_DEF_SAMPPER) 
           / encoder.slope;

    return stat;
}
 
Controller::Err Controller::ActualTorque(float *torq)
{
	float volts;
    Controller::Err stat;

    stat = instructionSend(Controller::CMD_TT, 0, &volts);

    // convert volts into torque units
    *torq = (volts - torque.offset) / torque.slope;

    return stat;
}

Controller::Err Controller::LoopAnalogSet(bool active)
{
	return instructionSend(Controller::CMD_AF, active, (float *) NULL);
}

Controller::Err Controller::LoopAnalogGet(bool *active)
{
	return instructionSend(Controller::CMD_AF, 0, active);
}

Controller::Err Controller::LoopDualSet(bool active)
{
	return instructionSend(Controller::CMD_DV, active, (float *) NULL);
}

Controller::Err Controller::LoopDualGet(bool *active)
{
	return instructionSend(Controller::CMD_DV, 0, active);
}

// no controller gain units for now
Controller::Err Controller::GainProportionalSet(float prop)
{  
	return instructionSend(Controller::CMD_KP, prop, (float *) NULL);
}

Controller::Err Controller::GainProportionalGet(float *prop)
{
	return instructionSend(Controller::CMD_KP, 0, prop);
}

Controller::Err Controller::GainIntegralSet(float integr)
{
	return instructionSend(Controller::CMD_KI, integr, (float *) NULL);
}

Controller::Err Controller::GainIntegralGet(float *integr)
{
	return instructionSend(Controller::CMD_KI, 0, integr);
}

Controller::Err Controller::GainDerivativeSet(float deriv)
{
	return instructionSend(Controller::CMD_KD, deriv, (float *) NULL);
}

Controller::Err Controller::GainDerivativeGet(float *deriv)
{
	return instructionSend(Controller::CMD_KD, 0, deriv);
}

// in volts	
Controller::Err Controller::LimitIntegratorSet(float limit)
{
	return instructionSend(Controller::CMD_IL, limit, (float *) NULL);
}

Controller::Err Controller::LimitIntegratorGet(float *limit)
{
	return instructionSend(Controller::CMD_IL, 0, limit);
}
	
Controller::Err Controller::LimitTorqueSet(float torq)
{
	float volts;

	volts = torq * torque.slope + torque.offset;
	
	return instructionSend(Controller::CMD_TL, volts, (float *) NULL);
}

Controller::Err Controller::LimitTorqueGet(float *torq)
{
	float volts;
    Controller::Err stat;

    stat = instructionSend(Controller::CMD_TL, 0, &volts);

    // convert volts into torque units
    *torq = (volts - torque.offset) / torque.slope;

    return stat;
}
	
Controller::Err Controller::OffsetTorqueSet(float offset)
{
	float volts;

	volts = offset * torque.slope + torque.offset;
	
	return instructionSend(Controller::CMD_OF, volts, (float *) NULL);
}

Controller::Err Controller::OffsetTorqueGet(float *offset)
{
	float volts;
    Controller::Err stat;

    stat = instructionSend(Controller::CMD_OF, 0, &volts);

    // convert volts into torque units
    *offset = (volts - torque.offset) / torque.slope;

    return stat;
}

Controller::Err Controller::SamplePeriodSet(float periodSec)
{
    int usec;
	
	// store new sample period for velocity conversion
	samplePeriod = periodSec;
	
    // convert seconds into usec
    usec = roundf (periodSec * 1000000.0);

    return instructionSend(Controller::CMD_TM, usec, (float *) NULL);
}

Controller::Err Controller::SamplePeriodGet(float *periodSec)
{
    int usec;
    Controller::Err stat;

	stat = instructionSend(Controller::CMD_TM, 0, &usec);

	// convert usec into seconds
	*periodSec = float(usec) / 1000000.0;

	return stat;
}

Controller::Err Controller::TargetPosnSet(float posn)
{
	return instructionSend(Controller::CMD_TARGPOSN, posn, (float *) NULL);
}

Controller::Err Controller::TargetPosnGet(float *posn)
{
	return instructionSend(Controller::CMD_TARGPOSN, 0, posn);
}

Controller::Err Controller::TargetVelSet(float vel)
{
	return instructionSend(Controller::CMD_TARGVEL, vel, (float *) NULL);
}

Controller::Err Controller::TargetVelGet(float *vel)
{
	return instructionSend(Controller::CMD_TARGVEL, 0, vel);
}

Controller::Err Controller::TargetVelEpsSet(float veps)
{
	return instructionSend(Controller::CMD_TARGVEPS, veps, (float *) NULL);
}

Controller::Err Controller::TargetVelEpsGet(float *veps)
{
	return instructionSend(Controller::CMD_TARGVEPS, 0, veps);
}

Controller::Err Controller::TargetAccelSet(float accel)
{
	return instructionSend(Controller::CMD_TARGACC, accel, (float *) NULL);
}

Controller::Err Controller::TargetAccelGet(float *accel)
{
	return instructionSend(Controller::CMD_TARGACC, 0, accel);
}

Controller::Err Controller::TargetDecelSet(float decel)
{
	return instructionSend(Controller::CMD_TARGDEC, decel, (float *) NULL);
}

Controller::Err Controller::TargetDecelGet(float *decel)
{
	return instructionSend(Controller::CMD_TARGDEC, 0, decel);
}

Controller::Err Controller::TargetNew()
{
	return instructionSend(Controller::CMD_TARGNEW, 0, (float *) NULL);
}

Controller::Err Controller::TargetStart()
{
	return ProgramExecute("#Targets");
}

Controller::Err Controller::TargetRunning(bool *running)
{
	return ProgramRunning(running);
}

Controller::Err Controller::TargetStop()
{
	return ProgramHalt();
}
 
Controller::Err Controller::ProfilePosnAbsSet(float posn)
{
    int quadCounts;

    // convert position into encoder quad counts
    quadCounts = roundf (posn * encoder.slope + encoder.offset);

	return instructionSend(Controller::CMD_PA, quadCounts, (float *) NULL);
}

Controller::Err Controller::ProfilePosnAbsGet(float *posn)
{
    int quadCounts;
    Controller::Err stat;

    stat = instructionSend(Controller::CMD_PA, 0, &quadCounts);

    // convert encoder quad counts into position
    *posn = (float(quadCounts) - encoder.offset) / encoder.slope;

    return stat;
}
    
Controller::Err Controller::ProfilePosnRelSet(float posn)
{
    int quadCounts;

    // convert position into encoder quad counts
    quadCounts = roundf (posn * encoder.slope + encoder.offset);

	return instructionSend(Controller::CMD_PR, quadCounts, (float *) NULL);
}

Controller::Err Controller::ProfilePosnRelGet(float *posn)
{
    int quadCounts;
    Controller::Err stat;

    stat = instructionSend(Controller::CMD_PR, 0, &quadCounts);

    // convert encoder quad counts into position
    *posn = (float(quadCounts) - encoder.offset) / encoder.slope;

    return stat;
}
 
Controller::Err Controller::ProfileVelMaxSet(float vel)
{
    int quadCountsPerSec;

    // convert velocity into quad counts/sec
    quadCountsPerSec = roundf(vel * encoder.slope 
                       / (samplePeriod/CONTROLLER_DEF_SAMPPER));
    
    return instructionSend(Controller::CMD_SP, quadCountsPerSec, (float *) NULL);
}

Controller::Err Controller::ProfileVelMaxGet(float *vel)
{
    int quadCountsPerSec;
    Controller::Err stat;

    stat = instructionSend(Controller::CMD_SP, 0, &quadCountsPerSec);

    // convert encoder quad counts/sec into velocity
    *vel = float(quadCountsPerSec) * (samplePeriod/CONTROLLER_DEF_SAMPPER)
            / encoder.slope;

    return stat;
}

Controller::Err Controller::ProfileAccelSet(float accel)
{
    int quadCountsPerSec2;

    // convert acceleration into encoder quad counts per sec^2
    quadCountsPerSec2 = roundf (accel*encoder.slope 
                        / ((samplePeriod/CONTROLLER_DEF_SAMPPER)
                           *(samplePeriod/CONTROLLER_DEF_SAMPPER)));

    return instructionSend(Controller::CMD_AC, quadCountsPerSec2, (float *) NULL);
}

Controller::Err Controller::ProfileAccelGet(float *accel)
{
    int quadCountsPerSec2;
    Controller::Err stat;

    stat = instructionSend(Controller::CMD_AC, 0, &quadCountsPerSec2);

    // convert encoder quad counts per sec^2 into acceleration
    *accel = float(quadCountsPerSec2)*((samplePeriod/CONTROLLER_DEF_SAMPPER)
                                       *(samplePeriod/CONTROLLER_DEF_SAMPPER))
                   / encoder.slope;

    return stat;
}
   
Controller::Err Controller::ProfileDecelSet(float decel)
{
    int quadCountsPerSec2;

    // convert acceleration into encoder quad counts per sec^2
    quadCountsPerSec2 = roundf (decel*encoder.slope 
                        / ((samplePeriod/CONTROLLER_DEF_SAMPPER)
                           *(samplePeriod/CONTROLLER_DEF_SAMPPER)));

    return instructionSend(Controller::CMD_DC, quadCountsPerSec2, (float *) NULL);
}

Controller::Err Controller::ProfileDecelGet(float *decel)
{
    int quadCountsPerSec2;
    Controller::Err stat;

    stat = instructionSend(Controller::CMD_DC, 0, &quadCountsPerSec2);

    // convert encoder quad counts per sec^2 into acceleration
    *decel = float(quadCountsPerSec2)*((samplePeriod/CONTROLLER_DEF_SAMPPER)
                                       *(samplePeriod/CONTROLLER_DEF_SAMPPER))
                   / encoder.slope;

    return stat;
}

Controller::Err Controller::JogVelSet(float vel)
{
    int quadCountsPerSec;

    // convert velocity into quad counts/sec
    quadCountsPerSec = roundf(vel * encoder.slope 
                       / (samplePeriod/CONTROLLER_DEF_SAMPPER));
    
    return instructionSend(Controller::CMD_JG, quadCountsPerSec, (float *) NULL);
}

Controller::Err Controller::JogVelGet(float *vel)
{
    int quadCountsPerSec;
    Controller::Err stat;

    stat = instructionSend(Controller::CMD_JG, 0, &quadCountsPerSec);

    // convert encoder quad counts/sec into velocity
    *vel = float(quadCountsPerSec) * (samplePeriod/CONTROLLER_DEF_SAMPPER) 
           / encoder.slope;

    return stat;
}

// limit=true, switch input active high, =false active low
// home=true, forward direction when high, =false, reverse direction when high
Controller::Err Controller::SwitchConfigure(bool limit, bool home)
{
	char str[8];
	
	sprintf(str, "%s,%s,1", limit ? "1" : "-1", home ? "1" : "-1");
	
	return instructionSend(Controller::CMD_CN, str, (float *) NULL);
}

Controller::Err Controller::SwitchFindEdge()
{
	return instructionSend(Controller::CMD_FE, 0, (float *) NULL);
}

Controller::Err Controller::SwitchForwardGet(bool *state)
{
	return instructionSend(Controller::CMD_LF, 0, state);
}

Controller::Err Controller::SwitchReverseGet(bool *state)
{
	return instructionSend(Controller::CMD_LR, 0, state);
}

Controller::Err Controller::MotionBegin()
{
	return instructionSend(Controller::CMD_BG, 0, (float *) NULL);
}

Controller::Err Controller::MotionDone(bool *done)
{
	Controller::Err stat;
	
	stat = instructionSend(Controller::CMD_BG, 0, done);
	*done = !*done; // in controller 1 = commanded motion, 0=otherwise
	
	return stat;
}

Controller::Err Controller::MotionServoHere()
{
	return instructionSend(Controller::CMD_SH, 0, (float *) NULL);
}

Controller::Err Controller::MotionStop()
{
	return instructionSend(Controller::CMD_ST, 0, (float *) NULL);
}

Controller::Err Controller::MotionAbort()
{
	return instructionSend(Controller::CMD_AB, 0, (float *) NULL);
}

Controller::Err Controller::ErrorActionSet(bool action)
{
	return instructionSend(Controller::CMD_OE, action, (float *) NULL);
}

Controller::Err Controller::ErrorActionGet(bool *action)
{
	return instructionSend(Controller::CMD_OE, 0, action);
}

// msg must be able to hold CONTROLLER_MSG_LEN characters
Controller::Err Controller::ErrorCodeCommand(int *code, char *msg, size_t msglen)
{
	Controller::Err stat;
	//	Check buffer length.
	stat = instructionSend(Controller::CMD_TC, 0, msg, msglen);
	sscanf(msg, "%d", code);		// scan numeric code from beginning of msg	
	return stat;
}

//
//	MotorStopMsg  -- msg decode for motor stop reasons
//
const char* MotorStopMsg(int code)
{
	switch (code) {
		case 0:		return("Motors are running, independent mode");
		case 1:		return("Motors stopped at commanded independent position");
		case 2:		return("Decelerating or stopped by FWD limit switch or software limit, FL");
		case 3:		return("Decelerating or stopped by REV limit switch or software limit, BL");
		case 4:		return("Decelerating or stopped by Stop Command (ST)");
		case 6:		return("Stopped by Abort input");
		case 7:		return("Stopped by Abort command (AB)");
		case 8:		return("Decelerating or stopped by Off-on-Error (OE1)");
		case 9:		return("Stopped after Finding Edge (FE)");
		case 10:	return("Stopped after Homing (HM)");
		case 11:	return("Stopped by selective Abort input");
		case 50:	return("Contour running");
		case 51:	return("Contour Stop");
		case 99:	return("MC timeout");
		case 100:	return("Motors are running, vector sequence");
		case 101:	return("Motors stopped at commanded vector");
		default:		return("Unknown stop code");
		}
		return("???");																							// unreachable
}

// msg must be able to hold CONTROLLER_MSG_LEN characters
Controller::Err Controller::ErrorCodeMotorStop(int *code, char *msg,size_t msglen)
{
	assert(code);																			// null pointer not allowed
	assert(msg);
	Controller::Err stat = instructionSend(Controller::CMD_SC, 0, code);
	strncpy(msg, MotorStopMsg(*code), msglen);	// return msg
	return stat;
}
                                       
Controller::Err Controller::AnalogInputGet(int chan, float *value)
{
    float volts;
    Controller::Err stat;

	switch ( chan ) {
		case 1:
			stat = instructionSend(Controller::CMD_AN1, 0, &volts);
   			// convert volts into a/d value
    		*value = (volts - analogInput1.offset) / analogInput1.slope;
			break;
		case 2:
			stat = instructionSend(Controller::CMD_AN2, 0, &volts);
   			// convert volts into a/d value
    		*value = (volts - analogInput2.offset) / analogInput2.slope;
			break;
		default:
			return Controller::ERR_PARAMETER_OUT_OF_RANGE;
			break;
	}
	
	return stat;
}

Controller::Err Controller::DigitalInputGet(int chan, bool *state)
{
    switch ( chan ) {
		case 1:
			return instructionSend(Controller::CMD_IN1, 0, state);
			break;
		case 2:
			return instructionSend(Controller::CMD_IN2, 0, state);
			break;
		case 3:
			return instructionSend(Controller::CMD_IN3, 0, state);
			break;
		case 4:
			return instructionSend(Controller::CMD_IN4, 0, state);
			break;
		case 5:
			return instructionSend(Controller::CMD_IN5, 0, state);
			break;
		case 6:
			return instructionSend(Controller::CMD_IN6, 0, state);
			break;
		case 7:
			return instructionSend(Controller::CMD_IN7, 0, state);
			break;
		default:
			return Controller::ERR_PARAMETER_OUT_OF_RANGE;
			break;
	}
}

Controller::Err Controller::DigitalInputGetAll(int *states)
{
	return instructionSend(Controller::CMD_TI, 0, states);
}

Controller::Err Controller::DigitalOutputSet(int chan, bool state)
{
	if (state) {
		return instructionSend(Controller::CMD_SB, chan, (float *) NULL);
	} else {
		return instructionSend(Controller::CMD_CB, chan, (float *) NULL);
	}
}

Controller::Err Controller::DigitalOutputGet(int chan, bool *state)
{
	int states;
	Controller::Err stat;
	
	stat = instructionSend(Controller::CMD_OP, 0, &states);
	*state = (bool)(states & (1<<(chan-1)));
	
	return stat;
}

Controller::Err Controller::DigitalOutputSetAll(int states)
{
	return instructionSend(Controller::CMD_OP, states, (float *) NULL);
}

Controller::Err Controller::DigitalOutputGetAll(int *states)
{
	return instructionSend(Controller::CMD_OP, 0, states);
}
//	
//	Download/upload support
//
const char k_enddownload[] = "\r\n\\";					// ends a download command
//
//	beginsWith  -- true if string begins with given string
//
static bool beginsWith(const std::string& s, const char* key)
{
	for (std::size_t i=0; i<s.size() && key[i]; i++)		// up to end of either string
	{	if (s[i] != key[i]) return(false); }					// if different, no match
	return(true);														// match
}
//
//	isComment -- true if line is a Galil controller comment
//
//	We don't download comments.  Galil deletes comments in their
//	download program, so we have to do so as well.
//
static bool isComment(const std::string& s)
{	const char k_comment1[] = "REM";					// comment if begins with REM
	const char k_comment2[] = "NO";						// comment if begins with NO
	const char k_comment3[] = "'";							// comment if begins with "'"
	if (s.length() < 1) return(true);							// a blank line is a comment
	if (beginsWith(s,k_comment1)) return(true);
	if (beginsWith(s,k_comment2)) return(true);
	if (beginsWith(s,k_comment3)) return(true);
	return(false);													// not a comment
}
//
//	ProgramClear  -- clear stored program
//
Controller::Err Controller::ProgramClear()
{	// download null program, clearing program memory
	return(instructionSend(Controller::CMD_DL,k_enddownload,NULL,k_downloadtimeus)); // do the command
}
//
//	ProgramLineAppend  -- download one line of a program
//
Controller::Err Controller::ProgramLineAppend(const std::string& line)
{	std::string cmd = line;										// make program into command
	cmd += k_enddownload;									// last line must end with end of line, then backslash
	return(instructionSend(Controller::CMD_DLAPPEND,cmd.c_str(),NULL,k_downloadtimeus));  // do the command
}
//
//	ProgramLineRead  -- reads one line of program
//
//	Line is returned with the line number and a space at the beginning
//
Controller::Err Controller::ProgramLineRead(int lineno, std::string& line)
{
	char linenostr[CONTROLLER_REPLY_LEN];
	snprintf(linenostr, sizeof(linenostr),"%d,%d",lineno, lineno);	// edit line numbers into command
	std::string param = linenostr;												// sending LS nnn,nnn
	Controller::Err stat = instructionSend(Controller::CMD_LS,false,param,line);
	if (stat != ERR_OK) return(stat);												// fails
	//	We should now have one line of the program, preceded by a line number and a space.
	return(Controller::ERR_OK);													// success
}
//
//	ProgramLineCheck  -- checks that program matches specified line
//
//	Line given should NOT have a line number.
//
Controller::Err Controller::ProgramLineCheck(int lineno, const std::string& line)
{	char linenostr[12];
	snprintf(linenostr,sizeof(linenostr),"%d ",lineno);				// line number as string
	std::string downloadedline = linenostr;								// this really should be provided by the STL
	downloadedline += line;													// construct expected line
	std::string uploadedline;													// line for uploaded line, which should match
	Controller::Err stat = ProgramLineRead(lineno, uploadedline);
	if (stat) return(stat);															// fails
	//	Remove any trailing CR LF
	while (uploadedline.length() > 0 &&
		((uploadedline[uploadedline.length()-1] == '\r') || (uploadedline[uploadedline.length()-1] == '\n')))
	{	uploadedline.erase(uploadedline.begin() + uploadedline.length()-1); 	}
	if (uploadedline == downloadedline) return(Controller::ERR_OK);		// matches, good
	//	Report failed match
	logprintf("Downloaded program does not match at line %d:\n  Expected \"%s\"\nReceived: \"%s\"\n",
		lineno, downloadedline.c_str(), uploadedline.c_str());
	return(ERR_RESPONSE_BAD);											// fails
}
//
//	ProgramErrLine  -- return line at which program halted
//
Controller::Err Controller::ProgramErrLine(int & lineno)
{	float linenofloat = 0;
	Controller::Err stat =  instructionSend(Controller::CMD_ED, 0, &linenofloat);
	if (stat != ERR_OK) return(stat);							
	lineno = int(linenofloat);								// convert to integer
	return(ERR_OK);											// return 
}
//
//
//	ProgramDownloadCheck -- from filename
//
//	Read back and check downloaded program
//
Controller::Err Controller::ProgramDownloadCheck(const char *filename)
{
	FILE *fileptr = fopen(filename, "r");				// open for reading
	if (!fileptr) return(Controller::ERR_FILE_OPEN_BAD);	// unable to open
	std::string s;													// working line
	int lineno = 0;												// at line 0
	Controller::Err stat = ERR_OK;						// OK so far
	//	Begin downloading program.
	for (;;)															// for all chars in input file
	{	int inchar = fgetc(fileptr);							// get input file char
		if (inchar == EOF || inchar == '\n' || inchar == '\r')				// if end of line
		{	if (!isComment(s))									// ignore comment lines
			{	Err stat = ProgramLineCheck(++lineno,s);		// download this line
				if (stat != ERR_OK) 
				{	logprintf( "Controller::ProgramDownloadCheck error on line: %d. %s\n", lineno, s.c_str());
					break;											// fails
				}
			}
			s.erase();												// clear input line
			if (inchar == EOF) break;						// normal eof
		} else {
			s += char(inchar);								// append char
		}
	}
	fclose(fileptr);												// done with file
	return(stat);													// download the program
}
//
//	ProgramDownload -- from filename
//
Controller::Err Controller::ProgramDownload(const char *filename)
{
	FILE *fileptr = fopen(filename, "r");				// open for reading
	if (!fileptr) return(Controller::ERR_FILE_OPEN_BAD);	// unable to open
	std::string s;													// working line
	Err stat = ProgramClear();								// download a blank line, erasing stored program
	if (stat != ERR_OK)
	{	logprintf( "Controller::ProgramDownload unable to clear program.\n");
		fclose(fileptr); return(stat);						// fails
	}
	int lineno = 0;												// at line 0
	//	Begin downloading program.
	for (;;)															// for all chars in input file
	{	int inchar = fgetc(fileptr);							// get input file char
		if (inchar == EOF || inchar == '\n' || inchar == '\r')				// if end of line
		{	if (!isComment(s))									// ignore comment lines
			{	lineno++;											// count downloaded lines
				Err stat = ProgramLineAppend(s);	// download this line
				if (stat != ERR_OK) 
				{	logprintf( "Controller::ProgramDownload error on line: %d. %s\n", lineno, s.c_str());
					break;											// fails
				}
			}
			s.erase();												// clear input line
			if (inchar == EOF) break;						// normal eof
		} else {
			s += char(inchar);								// append char
		}
	}
	fclose(fileptr);												// done with file
	if (stat == Controller::ERR_OK)						// if OK so far
	{	stat = ProgramDownloadCheck(filename);	}	// check download validity
	return(stat);													// return status
}
//
//	ProgramList -- returns entire stored program from controller as a string.
//
//	***NEEDS WORK*** doesn't really understand how listings end.
//
// to list the longest possible program, str needs to be at least 500x82
Controller::Err Controller::ProgramList(char *str, size_t strlen)
{
	return instructionSend(Controller::CMD_LS, 0, str, strlen,k_downloadtimeus);
}

// only works for thread 0 right now
// for some reason, DL starts at line 1 (not 0) so need to XQ1
Controller::Err Controller::ProgramExecute(const char* label)
{	
	if ( (label == NULL) || (strcmp(label, " ") == 0) ) {
		return instructionSend(Controller::CMD_XQ, "1", (float *) NULL);
	} else {
		return instructionSend(Controller::CMD_XQ, label, (float *) NULL);
	}
}

// only works for thread 0 right now
Controller::Err Controller::ProgramRunning(bool *running)
{
	int lineNum;
	Controller::Err stat;
	
	stat = instructionSend(Controller::CMD_XQ, 0, &lineNum);
	*running = (lineNum != -1);
	
	return stat;
}

Controller::Err Controller::ProgramHalt()
{
	return instructionSend(Controller::CMD_HX, 0, (float *) NULL);
}
//
//	Variable access
//
//
//	VariableSet -- set variable value
//
//	Not general command syntax, so we must call instrSend directly.
//
Controller::Err Controller::VariableSet(const char* name, float value)
{
	if (readonly) return(ERR_READ_ONLY_MODE);						// command not allowed in read only mode
	char instr[CONTROLLER_INSTR_LEN];
	snprintf(instr,CONTROLLER_INSTR_LEN, "%s=%f\r\n",name,value);	// format "VARNAME=VALUE"
    char reply[CONTROLLER_REPLY_LEN];	// reply from controller
	return(instrSend(instr,false,reply,CONTROLLER_REPLY_LEN,0));				// send the command directly - no command code
}
//
//	VariableGet  -- get variable value
//
//
//	Not general command syntax, so we must call instrSend directly.
//
Controller::Err Controller::VariableGet(const char* name, float* rvalue)
{	assert(rvalue);
	*rvalue=0.0;																		// avoid returning junk
	char instr[CONTROLLER_INSTR_LEN];
	snprintf(instr,CONTROLLER_INSTR_LEN, "MG %s\r\n",name);	// format "MG VARNAME"
	char reply[CONTROLLER_REPLY_LEN];
	Controller::Err err = instrSend(instr,true,reply,CONTROLLER_REPLY_LEN,0);		// send the command directly - no command code
	if (err != Controller::ERR_OK) return(err);							// fails
	int cnt = sscanf(reply,"%f", rvalue);									// get value
	if (cnt != 1)																		// if bad value format
	{	logprintf("Controller::VariableGet - reading variable \"%s\", received bad value %f\n",
			name, *rvalue);
		return(Controller::ERR_RESPONSE_BAD);						// bad result
	}
	return(Controller::ERR_OK);												// success
}

Controller::Err Controller::StatusAxisInMotion(bool *state)
{
    int sbits;
	Controller::Err stat;
	
	stat = instructionSend(Controller::CMD_TS, 0, &sbits);

	// extract out bit of interest
	*state = sbits & 0x0080;

	return stat;
}

Controller::Err Controller::StatusErrorLimitExceeded(bool *state)
{
    int sbits;
	Controller::Err stat;
	
	stat = instructionSend(Controller::CMD_TS, 0, &sbits);

	// extract out bit of interest
	*state = sbits & 0x0040;

	return stat;
}

Controller::Err Controller::StatusMotorOff(bool *state)
{
    int sbits;
	Controller::Err stat;
	
	stat = instructionSend(Controller::CMD_TS, 0, &sbits);

	// extract out bit of interest
	*state = sbits & 0x0020;

	return stat;
}

Controller::Err Controller::StatusForwardLimitInactive(bool *state)
{
    int sbits;
	Controller::Err stat;
	
	stat = instructionSend(Controller::CMD_TS, 0, &sbits);

	// extract out bit of interest
	*state = sbits & 0x0008;

	return stat;
}

Controller::Err Controller::StatusReverseLimitInactive(bool *state)
{
    int sbits;
	Controller::Err stat;
	
	stat = instructionSend(Controller::CMD_TS, 0, &sbits);

	// extract out bit of interest
	*state = sbits & 0x0004;

	return stat;
}

Controller::Err Controller::StatusHomeSwitch(bool *state)
{
    int sbits;
	Controller::Err stat;
	
	stat = instructionSend(Controller::CMD_TS, 0, &sbits);

	// extract out bit of interest
	*state = sbits & 0x0002;

	return stat;
}

Controller::Err Controller::StatusLatchNotArmed(bool *state)
{
    int sbits;
	Controller::Err stat;
	
	stat = instructionSend(Controller::CMD_TS, 0, &sbits);

	// extract out bit of interest
	*state = sbits & 0x0001;

	return stat;
}

// msg must be able to hold 7*CONTROLLER_MSG_LEN characters
Controller::Err Controller::StatusHandle(char *msg, size_t msglen)
{
	return instructionSend(Controller::CMD_TH, 0, msg, msglen);
}

// msg must be able to hold CONTROLLER_MSG_LEN characters
Controller::Err Controller::StatusFirmwareVersionGet(char *msg, size_t msglen)
{
	return instructionSend(Controller::CMD_RV, 0, msg, msglen);
}

// will need 1 sec timeout
Controller::Err Controller::BurnParameters()
{
	return instructionSend(Controller::CMD_BN, 0, (float *) NULL);
}

// will need 10 sec timeout
Controller::Err Controller::BurnProgram()
{
	return instructionSend(Controller::CMD_BP, 0, (float *) NULL);
}

Controller::Err Controller::ResetToPowerUp()
{
	return instructionSend(Controller::CMD_RS, 0, (float *) NULL);
}
//
//	Functions with conversion.  Not currently used.
//
void Controller::ConvEncoderSet(char *iUnits, float iSlope, float iOffset)
{
	convSet(&encoder, iUnits, iSlope, iOffset);
}

void Controller::ConvEncoderGet(char *oUnits, float *oSlope, float *oOffset)
{
	convGet(&encoder, oUnits, oSlope, oOffset);
}

char *Controller::UnitsEncoder()
{	
	return encoder.units;
}

void Controller::ConvEncoderAuxSet(char *iUnits, float iSlope, float iOffset)
{
	convSet(&encoderAux, iUnits, iSlope, iOffset);
}

void Controller::ConvEncoderAuxGet(char *oUnits, float *oSlope, float *oOffset)
{
	convGet(&encoderAux, oUnits, oSlope, oOffset);
}

char *Controller::UnitsEncoderAux()
{
	return encoderAux.units;
}

void Controller::ConvTorqueSet(char *iUnits, float iSlope, float iOffset)
{
	convSet(&torque, iUnits, iSlope, iOffset);
}

void Controller::ConvTorqueGet(char *oUnits, float *oSlope, float *oOffset)
{
	convGet(&torque, oUnits, oSlope, oOffset);
}

char *Controller::UnitsTorque()
{
	return torque.units;
}

void Controller::ConvAnalogInputSet(char *iUnits, float iSlope, float iOffset)
{
	convSet(&analogInput1, iUnits, iSlope, iOffset);
}

void Controller::ConvAnalogInputGet(char *oUnits, float *oSlope, float *oOffset)
{
	convGet(&analogInput1, oUnits, oSlope, oOffset);
}                          

char *Controller::UnitsAnalogInput()
{
	return analogInput1.units;
}

void Controller::ConvAnalogInputAuxSet(char *iUnits, float iSlope, float iOffset)
{
	convSet(&analogInput2, iUnits, iSlope, iOffset);
}

void Controller::ConvAnalogInputAuxGet(char *oUnits, float *oSlope, float *oOffset)
{
	convGet(&analogInput2, oUnits, oSlope, oOffset);
}

char *Controller::UnitsAnalogInputAux()
{
	return analogInput2.units;
}

void Controller::convSet(Controller::Conversion *c, char *iUnits, float iSlope, float iOffset)
{
	strncpy(c->units, iUnits, CONTROLLER_UNITS_LEN);
	c->slope = iSlope;
	c->offset = iOffset;
}

void Controller::convGet(Conversion *c, char *oUnits, float *oSlope, float *oOffset)
{
	strncpy(oUnits, c->units, CONTROLLER_UNITS_LEN);
	*oSlope  = c->slope;
	*oOffset = c->offset;
}

int Controller::roundf(float x)
{
	return int (x + (x<0 ? -0.5 : 0.5));
}
