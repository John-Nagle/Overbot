/////////////////////////////////////////////////////////////////////////////
//
//    File: controller.h
//
//    Usage:
//        #include "controller.h"
//
//        Controller c("gcsteer");	// "gcsteer" is the hostname
//
//    Description:
//        Controller objects allow you to command Galil motion controller units 
//        via the Ethernet with a more user-friendly interface.  Commands 
//        can be issued using user units.
//
//        Create a Controller object for each Galil unit you need to access.
//
//        Based on mvp.h and mvpserver.h code of August 2003.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        January, 2004
//
/////////////////////////////////////////////////////////////////////////////

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdio.h>		// messaging.h needs to include this
#include <vector>
#include <string>
#include "messaging.h"
#include "ethernet.h"
#include "mutexlock.h"

using namespace std;	// for vector class

// specify INT_MAX and INT_MIN that are smaller than actual values so don't
// have to deal with round off error from conversion factors (changing
// all the variables over to "double" doesn't help either (sscanf))
#define CONTROLLER_INT_MAX		(2147480000)	// close enough, no round errs
#define CONTROLLER_INT_MIN		(-2147480000)	// close enough, no round errs

#define CONTROLLER_UNITS_LEN	(20)	// max length of description of units
#define CONTROLLER_CMD_LEN		(82)	// max length of command string
#define CONTROLLER_INSTR_LEN	(82)	// max instruction length
#define CONTROLLER_REPLY_LEN	(500*82)// max reply length
#define CONTROLLER_QUERY_LEN	(82)	// max length of query string
#define CONTROLLER_MSG_LEN		(82)	// max length of controller error msg
#define CONTROLLER_FILENAME_LEN	(82)	// max filename length

#define CONTROLLER_DEF_SAMPPER	(0.001)	// default sample period in sec



class Controller {
public:
    // default constructor
    Controller(const char *hostname, bool isreadonly=false);			// hostname can be name in /etc/hosts or
    																// IP address
	// errors
	enum Err {
		ERR_OK,
		ERR_COMMAND_STRING_TOO_LONG,
		ERR_PARAMETER_OUT_OF_RANGE,
		ERR_SEND_BAD,
		ERR_RECV_BAD,
		ERR_FILE_OPEN_BAD,
		ERR_FILE_CLOSE_BAD,
		ERR_NOT_CONNECTED,
		ERR_RESPONSE_BAD,
		ERR_FILE_FORMAT_BAD,
		ERR_READ_ONLY_MODE	,
		ERR_PROGRAM_NOT_RUNNING,
		ERR_PROGRAM_NOT_READY,
		ERR_PROGRAM_IN_MANUAL
	};
	//	ErrMsg  -- convert error code to error message text.
	static const char* ErrMsg(Err err)
	{
		switch ( err ) {
			case ERR_OK:
				return "no error";
			case ERR_COMMAND_STRING_TOO_LONG:
				return "command string too long";
			case ERR_PARAMETER_OUT_OF_RANGE:
				return "parameter out of range";
			case ERR_SEND_BAD:
				return "bad send";
			case ERR_RECV_BAD:
				return "bad receive";
			case ERR_FILE_OPEN_BAD:
				return "problem opening file";
			case ERR_FILE_CLOSE_BAD:
				return "problem closing file";
			case ERR_FILE_FORMAT_BAD:
				return "bad data in download file";
			case ERR_NOT_CONNECTED:
				return "not connected to controller"; 
			case ERR_RESPONSE_BAD:
				return "unexpected message from controller";
			case ERR_READ_ONLY_MODE:
				return "operation not allowed in read-only mode";
			case ERR_PROGRAM_NOT_RUNNING:
				return "controller program not running";
			case ERR_PROGRAM_NOT_READY:
				return "controller program not ready";
			case ERR_PROGRAM_IN_MANUAL:
				return "controller program in manual mode";
			default:
				return "unknown error";
			}
			//	Unreachable
	}
	//	Connect/disconnect Ethernet connection
	int Connect(double timeout = 0.0);
	int Shutdown();
	//	Connection status
	bool Connected() const;
	bool DatagramMode();
	void DatagramMode(bool iDatagram);

	const char* Hostname() const { return(e.Hostname()); }
	const char* IPAddress() const { return (e.IPAddress()); }
	//	Misc. access
	bool ReadOnly() const { return(readonly); }
	Controller::Err SetReadOnly(bool isreadonly);			// set/clear read only mode



    // motor off
	Controller::Err MotorOff();								// MO | :

    // define position
    Controller::Err DefinePosnSet(float posn);				// DP int (-2147483648 to 2147483647) | :
    Controller::Err DefinePosnAuxSet(float posn);			// DE int (-2147483648 to 2147483647) | :

    // limits - position range
    Controller::Err LimitPosnMinSet(float posnMin);			// BL int (-2147483648 to 2147483647) | :
    Controller::Err LimitPosnMinGet(float *posnMin);		// BL? | int (-2147483648 to 2147483647)

    Controller::Err LimitPosnMaxSet(float posnMax);			// FL int (-2147483648 to 2147483647) | :
    Controller::Err LimitPosnMaxGet(float *posnMax);		// FL? | int (-2147483648 to 2147483647)

	Controller::Err LimitPosnErrSet(float posnIn);			// ER int (1 to 32767)
 	Controller::Err LimitPosnErrGet(float *posnIn);			// ER? | int (1 to 32767)
 
    // actuals
	Controller::Err ActualPosnGet(float *posn);				// TP | int 
	Controller::Err ActualPosnRefGet(float *posnCmd);		// RPX | int 
	Controller::Err ActualPosnErrorGet(float *posnErr); 	// TE | int (-2147483647 to 2147483647)
	Controller::Err ActualPosnAuxGet(float *posnAux);		// TD | int 
	Controller::Err ActualPosnAuxGetUnscaled(int *posnAux);		// TD | int - no scaling, for odometers which wrap at 2^24
	Controller::Err ActualVelAvgGet(float *vel);			// TV | int 
	Controller::Err ActualTorque(float *torq);				// TT | float (-9.998 to 9.998)

    // control loop parameters
	Controller::Err LoopAnalogSet(bool active);				// AF int (0 or 1) |::
	Controller::Err LoopAnalogGet(bool *active);			// AF? | int (0 or 1)
	
	Controller::Err LoopDualSet(bool active);				// DV int (0 or 1) | :
	Controller::Err LoopDualGet(bool *active);				// DV? | int (0 or 1)
	
	Controller::Err GainProportionalSet(float prop);		// KP float (0 to 1023.875 incr 0.125) | :
	Controller::Err GainProportionalGet(float *prop);		// KP? | float (0 to 1023.875 incr 0.125)

	Controller::Err GainIntegralSet(float integr);			// KI float (0 to 2047.875 incr 1/128) | :
	Controller::Err GainIntegralGet(float *integr);			// KI? | float (0 to 2047.875 incr 1/128)

	Controller::Err GainDerivativeSet(float deriv);			// KD float (0 to 4095.875 incr 0.125) | :
	Controller::Err GainDerivativeGet(float *deriv);		// KD? | float (0 to 4095.875 incr 0.125)
	
	Controller::Err LimitIntegratorSet(float limit);		// IL float (0 to 9.9988 incr 0.0003) | :
	Controller::Err LimitIntegratorGet(float *limit);		// IL? | float (0 to 9.9988 incr 0.0003)
	
	Controller::Err LimitTorqueSet(float torq);				// TL float (0 to 9.998 incr 0.0003) | :
	Controller::Err LimitTorqueGet(float *torq);			// TL? | float (0 to 9.998 incr 0.0003)
	
	Controller::Err OffsetTorqueSet(float offset);			// OF float (-9.998 to 9.998 incr 0.0003) | :
	Controller::Err OffsetTorqueGet(float *offset);			// OF? | float (-9.998 to 9.998 incr 0.0003)

	Controller::Err SamplePeriodSet(float periodSec);		// TM int (250 to 20000 incr 125) | :
	Controller::Err SamplePeriodGet(float *periodSec);		// TM? | int (250 to 20000 incr 125)

	// absolute targets (application program)
	// need to download Targets program before calling start
	// need to set posn, vel, veleps, acc, dec before calling start
	Controller::Err TargetPosnSet(float posn);				// TargPosn=<value>; TargNew=1 | :
	Controller::Err TargetPosnGet(float *posn);				// TargPosn= | <value>
	Controller::Err TargetVelSet(float vel);				// TargVel=<value>; TargNew=1 | :
	Controller::Err TargetVelGet(float *vel);				// TargVel= | <value>
	Controller::Err TargetVelEpsSet(float veps);			// TargVeps=<value>; TargNew=1 | :
	Controller::Err TargetVelEpsGet(float *veps);			// TargVeps= | <value>
	Controller::Err TargetAccelSet(float accel);			// TargAcc=<value>; TargNew=1 | :
	Controller::Err TargetAccelGet(float *accel);			// TargAcc= | <value>
	Controller::Err TargetDecelSet(float decel);			// TargDec=<value>; TargNew=1 | :
	Controller::Err TargetDecelGet(float *decel);			// TargDec= | <value>
	Controller::Err TargetNew();							// TargetNew=1 | :
	Controller::Err TargetStart();							// XQ#Targets | :
	Controller::Err TargetRunning(bool *running);			// MG _XQ | int (-1 or line # 0 to 500)
	Controller::Err TargetStop();							// HX | :
	
    // absolute/relative profile (indep axis posn)   
    Controller::Err ProfilePosnAbsSet(float posn);			// PA int (-2147483647 to 2147483648) | :
    Controller::Err ProfilePosnAbsGet(float *posn);			// PA? | int (-2147483647 to 2147483648)
    
    Controller::Err ProfilePosnRelSet(float posn);			// PR int (-2147483648 to 2147483647) | :
    Controller::Err ProfilePosnRelGet(float *posn);			// PR? | int (-2147483648 to 2147483647)
 
    Controller::Err ProfileVelMaxSet(float vel);			// SP int (0 to 12000000) | :
    Controller::Err ProfileVelMaxGet(float *vel);			// SP? | int (0 to 12000000)

    Controller::Err ProfileAccelSet(float accel);			// AC int (1024 to 67107840 incr 1024) | :
    Controller::Err ProfileAccelGet(float *accel);			// AC? | int (1024 to 67107840 incr 1024)
    
    Controller::Err ProfileDecelSet(float decel);			// DC int (1024 to 67107840 incr 1024) | :
    Controller::Err ProfileDecelGet(float *decel);			// DC? |  int (1024 to 67107840 incr 1024)
    
    // constant velocity (indep jogging)
    Controller::Err JogVelSet(float vel);					// JG int (-12000000 to 12000000) | :
    Controller::Err JogVelGet(float *vel);					// JG? | abs(int (-12000000 to 12000000))

    // switches
    Controller::Err SwitchConfigure(bool limit, bool home);	// CN 1 or -1, 1 or -1, 1 or -1 | :
    Controller::Err SwitchFindEdge();						// FE | :
    Controller::Err SwitchForwardGet(bool *state);			// MG _LF | float (0 or 1)
    Controller::Err SwitchReverseGet(bool *state);			// MG _LR | float (0 or 1)

    // motion
	Controller::Err MotionBegin();							// BG | :
	Controller::Err MotionDone(bool *done);					// MG _BG | float (0 or 1)
	Controller::Err MotionServoHere();						// SH | :

    // motion abort
	Controller::Err MotionStop();							// ST | :
	Controller::Err MotionAbort();							// AB | :

    // errors
    Controller::Err ErrorActionSet(bool action);			// OE int (0 or 1) | :
    Controller::Err ErrorActionGet(bool *action);			// OE? | int (0 or 1)
    
    Controller::Err ErrorCodeCommand(int *code, char *msg, size_t msglen);	// TC1 | int (1 to 127) msg
    Controller::Err ErrorCodeMotorStop(int *code, char *msg, size_t msglen);// SC | int (0 to 101)

    // analog inputs
	Controller::Err AnalogInputGet(int chan, float *value);	// MG @AN[chan=1 or 2] | float (-10 to 10)

    // digital inputs/outputs
    Controller::Err DigitalInputGet(int chan, bool *state);	// MG @IN[chan 1 to 7] | float (0 or 1)
    Controller::Err DigitalInputGetAll(int *states);		// TI | int (0 to 127)
    Controller::Err DigitalOutputSet(int chan, bool state);	// SB/CB int (1 to 3) | :
    Controller::Err DigitalOutputGet(int chan, bool *state);// OP? | int (0 to 7)
    Controller::Err DigitalOutputSetAll(int states);		// OP int (0 to 7) | :
    Controller::Err DigitalOutputGetAll(int *states);		// OP? | int (0 to 7)

    // application programming 
    Controller::Err ProgramDownload(const char *filename);		// Download file, one line at a time
    Controller::Err ProgramDownloadCheck(const char *filename);		// Read and check file
    Controller::Err ProgramList(char *str, size_t strlen);		// LS
    Controller::Err ProgramExecute(const char *label);		// XQ (could add param & thread later)
	Controller::Err ProgramRunning(bool *running);			// MG _XQ
    Controller::Err ProgramHalt();							// HX
	Controller::Err ProgramLineRead(int lineno, std::string& line);	// reads one line of program
	Controller::Err ProgramErrLine(int & lineno);							// get line on which program halted
	
	//	Variable access
	Controller::Err VariableSet(const char* name, float value);
	Controller::Err VariableGet(const char* name, float* value);

    // status
    Controller::Err StatusAxisInMotion(bool *state);		// TS | int (0 to 255) bit 7
    Controller::Err StatusErrorLimitExceeded(bool *state);	// TS | int (0 to 255) bit 6
    Controller::Err StatusMotorOff(bool *state);			// TS | int (0 to 255) bit 5
    Controller::Err StatusForwardLimitInactive(bool *state);// TS | int (0 to 255) bit 3
    Controller::Err StatusReverseLimitInactive(bool *state);// TS | int (0 to 255) bit 2
    Controller::Err StatusHomeSwitch(bool *state);			// TS | int (0 to 255) bit 1
    Controller::Err StatusLatchNotArmed(bool *state);		// TS | int (0 to 255) bit 0

	Controller::Err StatusHandle(char *msg, size_t msglen);	// TH | 7 lines of text
    Controller::Err StatusFirmwareVersionGet(char *msg, size_t msglen);	// ^R^V (does not work, nonstandard reply format)

    // EEprom
	Controller::Err BurnParameters();						// BN | : (1 sec)
	Controller::Err BurnProgram();							// BP | : (10 sec)
	Controller::Err ResetToPowerUp();						// RS | :
	// (no ResetToDefault since will forget IP address; do under Windows)

	// conversion routines
	// can only set units, slope, and offset; slopeVel is computed
	void ConvEncoderSet(char *iUnits, float iSlope, float iOffset);
	void ConvEncoderGet(char *oUnits, float *oSlope, float *oOffset);
	char *UnitsEncoder();
	
	void ConvEncoderAuxSet(char *iUnits, float iSlope, float iOffset);
	void ConvEncoderAuxGet(char *oUnits, float *oSlope, float *oOffset);
	char *UnitsEncoderAux();

	void ConvTorqueSet(char *iUnits, float iSlope, float iOffset);
	void ConvTorqueGet(char *oUnits, float *oSlope, float *oOffset);
	char *UnitsTorque();
	
	void ConvAnalogInputSet(char *iUnits, float iSlope, float iOffset);
	void ConvAnalogInputGet(char *oUnits, float *oSlope, float *oOffset);
	char *UnitsAnalogInput();
	                              
	void ConvAnalogInputAuxSet(char *iUnits, float iSlope, float iOffset);
	void ConvAnalogInputAuxGet(char *oUnits, float *oSlope, float *oOffset);
	char *UnitsAnalogInputAux();
	
	void Verbose(bool state) { verbose = state; }
	bool Verbose() { return verbose; }

private:
    // commands
	enum Cmd {
		// motor off
		CMD_MO,
		// define position
		CMD_DP, CMD_DE,
    	// limits - position range	
		CMD_BL, CMD_FL, CMD_ER,
    	// actuals
		CMD_TP, CMD_RP, CMD_TE, CMD_TD, CMD_TV, CMD_TT,
    	// control loop parameters	
		CMD_AF, CMD_DV, CMD_KP, CMD_KI, CMD_KD, CMD_IL, CMD_TL, CMD_OF, CMD_TM, 
		// absolute targets (application program)	
		CMD_TARGPOSN, CMD_TARGVEL, CMD_TARGVEPS,
		CMD_TARGACC, CMD_TARGDEC, CMD_TARGNEW,
    	// absolute/relative profile (indep axis posn)
		CMD_PA, CMD_PR, CMD_SP, CMD_AC, CMD_DC,
    	// constant velocity (indep jogging)
		CMD_JG,
    	// switches
		CMD_CN, CMD_FE, CMD_LF, CMD_LR,
    	// motion	 
		CMD_BG, CMD_SH,
    	// motion abort	
		CMD_ST, CMD_AB, 
    	// errors	 
		CMD_OE, CMD_TC, CMD_SC,
    	// analog inputs	
		CMD_AN1, CMD_AN2,
		// digital inputs/outputs	
		CMD_IN1, CMD_IN2, CMD_IN3, CMD_IN4, CMD_IN5, CMD_IN6, CMD_IN7,
		CMD_TI, CMD_SB, CMD_CB, CMD_OP, 
    	// application programming	
		CMD_DL, CMD_DLAPPEND,  CMD_ED, CMD_LS, CMD_XQ, CMD_HX, 
    	// status	
		CMD_TS, CMD_TH, CMD_RV,
    	// EEprom	
		CMD_BN, CMD_BP,CMD_RS,
	};

	// command properties record
	typedef struct {
    	char cmdStr[CONTROLLER_CMD_LEN];	// command text string (e.g., "MO")
	    bool paramOK;                   	// can take parameters
	    int paramType;						// parameter type
    	bool paramRange;                	// parameters must be within range
	    float paramMin;                   	// minimum parameter value
    	float paramMax;                   	// maximum parameter value
    	bool queryOK;						// ok to query this command
	    char queryStr[CONTROLLER_QUERY_LEN];// query text string
	} cmdRec;
  
	// database of command properties and related routines
	vector<cmdRec> cmdDB;
	vector<cmdRec>::iterator cmdDBi;

    void cmdDBInit();
	void cmdDBLoad(Controller::Cmd cmdNum, char *cmdStr, bool paramOK,
                   int paramType, bool paramRange, float paramMin, 
                   float paramMax, bool queryOK, char *queryStr);

	// command parameters can be of various types
//	union ParamType {
//		int paramInt;
//		float paramFloat;
//		char *paramString;
//	};
	
	//  for encoder: set slope = cpr * 4 * gearRatio / unitsPerTurn
	struct Conversion {
		char units[CONTROLLER_UNITS_LEN];
	    float slope;
	    float offset;
	};
	
    // sample period in sec (need to store for velocity conversion)
    float samplePeriod;

    // linear conversion factors for Controller inputs & outputs
    Conversion encoder;
    Conversion encoderAux;
    Conversion torque;
    Conversion analogInput1;
    Conversion analogInput2;
    
    // generic conversion routines
    void convSet(Conversion *c, char *iUnits, float iSlope, float iOffset);
    void convGet(Conversion *c, char *oUnits, float *oSlope, float *oOffset);

    // round to nearest integer
    int roundf(float x);

	// Ethernet TCP/IP socket communication with Controller unit
	Ethernet e;
	
	//	Uploading/downloading support
	Controller::Err ProgramLineAppend(const std::string& line);	// append one line to program
	Controller::Err ProgramClear();												// clear stored program
	Controller::Err ProgramLineCheck(int lineno, const std::string& line);	// checks
 
    // routines to send messages to and receive replies from the Controller
    Controller::Err instructionSend(Controller::Cmd cmd, int param, int *rtn);
    Controller::Err instructionSend(Controller::Cmd cmd, int param, float *rtn);
    Controller::Err instructionSend(Controller::Cmd cmd, int param, bool *rtn);
    Controller::Err instructionSend(Controller::Cmd cmd, float param, float *rtn);
    Controller::Err instructionSend(Controller::Cmd cmd, int param, char* rtn, size_t rtnlen, int timeoutus = 0);
    Controller::Err instructionSend(Controller::Cmd cmd, bool query, const std::string& param, std::string& reply, int timeoutus = 0);
    Controller::Err instructionSend(Controller::Cmd cmd, const char* param, float* rtn, int timeoutus = 0);
	Controller::Err instrSend(const char *instr, bool query, char *reply, size_t replylen, int timeoutus = 0); // with retry and recovery
	Controller::Err instrSendOnce(const char *instr, bool query, char *reply, size_t replylen, int timeoutus); // one try
	bool parameterOutOfRange(Controller::Cmd cmd, float param);
	void instructionErrMsgTryAgain(const char *instr);
	void instructionErrMsgTryLast(const char *instr);
	Controller::Err instructionErrMsgResync();
    
	// verbose mode
	bool verbose;
	//	Read-only mode - write-type operations not allowed
	bool readonly;
	//	Lock against interfering accesses to same controller
	ost::Mutex m_lock;																		// lock access to the controller
};

#endif // CONTROLLER_H
