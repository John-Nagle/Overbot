/////////////////////////////////////////////////////////////////////////////
//
//    File: mvp.h
//
//    Usage:
//        #include "mvp.h"
//
//        MVP m(4);
//
//    Description:
//        MVP objects allow you to command MVP units via an MVP Server with 
//        a more user-friendly interface.  Commands can be issued using user
//        units.
//
//        Create an MVP object for each MVP unit you need to access.
//
//        FIX - need to handle valueFV
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        August, 2003
//
/////////////////////////////////////////////////////////////////////////////

#ifndef MVP_H
#define MVP_H

#include <stdio.h>		// messaging.h needs to include this
#include "messaging.h"
#include "mvpserver.h"

#define MVP_UNITS_LEN (20)		// max length of description of units

class MVP {
public:
    // default constructor
    MVP(char *serverID, int timeout, int nodeNum);

    // enable/disable/reset
	MVPServer::Err NodeEnable();							// EN
	MVPServer::Err NodeDisable();							// DI
	MVPServer::Err NodeReset();								// RN

    // absolute/realtive target profile
    MVPServer::Err TargetPosnAbsSet(float posn);			// LA
    MVPServer::Err TargetPosnAbsGet(float *posn);			// LA
    
    MVPServer::Err TargetPosnRelSet(float posn);			// LR
    MVPServer::Err TargetPosnRelGet(float *posn);			// LR

    MVPServer::Err TargetVelMaxSet(float vel);				// SP
    MVPServer::Err TargetVelMaxGet(float *vel);				// SP

    MVPServer::Err TargetAccelRateSet(float accel);			// AC
    MVPServer::Err TargetAccelRateGet(float *accel);		// AC
    
    MVPServer::Err TargetDecelRateSet(float decel);			// DC
    MVPServer::Err TargetDecelRateGet(float *decel);		// DC
    
    // constant velocity
    MVPServer::Err TargetVelConstantSet(float vel);			// V
    MVPServer::Err TargetVelConstantGet(float *vel);		// V

    // actual
	MVPServer::Err ActualPosnGet(float *posn);				// POS
	MVPServer::Err ActualPosnErrorGet(float *posnErr); 		// ERR
	MVPServer::Err ActualPosnMPMGet(float *posnMPM);		// EP
	MVPServer::Err ActualVelGet(float *vel);				// AS

    // limits - position range
	MVPServer::Err TargetPosnInRangeSet(float posnIn);		// N
 	MVPServer::Err TargetPosnInRangeGet(float *posnIn);		// N

    MVPServer::Err TargetPosnLimitMinSet(float posnMin);	// LL
    MVPServer::Err TargetPosnLimitMinGet(float *posnMin);	// LLM (made up)

    MVPServer::Err TargetPosnLimitMaxSet(float posnMax);	// LL
    MVPServer::Err TargetPosnLimitMaxGet(float *posnMax);	// LL

    MVPServer::Err ActionPosnOutOfLimitSet(int code);		// SA
    MVPServer::Err ActionPosnOutOfLimitGet(int *code);		// SA

	// limits - current
	MVPServer::Err PowerLimitPWMOutputSet(float percent);	// SM
	MVPServer::Err PowerLimitPWMOutputGet(float *percent);	// SM

    MVPServer::Err PowerLimitCurrentSet(float amps);		// ANO (program current limit)
    MVPServer::Err PowerLimitCurrentGet(float *amps);		// ANO (program current limit)

    // home
    MVPServer::Err Home();									// HO (zero)
    MVPServer::Err Home(float posn);						// HO
	MVPServer::Err HomeArmSet(bool setting);				// HA
	MVPServer::Err HomeArmGet(bool *setting); 				// HS
	MVPServer::Err HomeArmPolaritySet(int setting);			// HP
	MVPServer::Err HomeArmPolarityGet(int *setting);		// HS

	MVPServer::Err ActionHomeSequenceSet(int code);			// HF
	MVPServer::Err ActionHomeSequenceGet(int *code);		// HF

	MVPServer::Err HomeEnableOnBootSet(bool setting);		// AE
	MVPServer::Err HomeEnableOnBootGet(bool *setting);		// AE

    // motion
	MVPServer::Err MotionMove();							// M

    // motion abort
	MVPServer::Err MotionAbort();							// AB
	MVPServer::Err MotionAbortDecelRateSet(float decel);	// AD
	MVPServer::Err MotionAbortDecelRateGet(float *decel);	// AD
	MVPServer::Err ActionMotionAbortSet(int code);			// AA
	MVPServer::Err ActionMotionAbortGet(int *code);			// AA

    // errors
    MVPServer::Err ErrorPosnMaxSet(float posnErr);			// FD
    MVPServer::Err ErrorPosnMaxGet(float *posnErr);			// FD

	MVPServer::Err ErrorPosnFlagDelaySet(float delaySec);	// FDT
	MVPServer::Err ErrorPosnFlagDelayGet(float *delaySec);	// FDT

    MVPServer::Err ActionErrorPosnSet(int code);			// FA
    MVPServer::Err ActionErrorPosnGet(int *code);			// FA

    // gains
	MVPServer::Err GainProportionalSet(float prop);			// POR
	MVPServer::Err GainProportionalGet(float *prop);		// POR

	MVPServer::Err GainIntegralSet(float integr);			// I
	MVPServer::Err GainIntegralGet(float *integr);			// I

	MVPServer::Err GainDerivativeSet(float deriv);			// DER
	MVPServer::Err GainDerivativeGet(float *deriv);			// DER

    // sample period
	MVPServer::Err SamplePeriodSet(float periodSec);		// SR
	MVPServer::Err SamplePeriodGet(float *periodSec);		// SR

    // analog input/output
	MVPServer::Err AnalogInputValueGet(float *value);		// ANI
	MVPServer::Err AnalogInputValueGet(bool *state);		// ANI (analog input mode dig)
    MVPServer::Err AnalogInputValueMPMGet(float *valueMPM);	// EAI

	MVPServer::Err AnalogInputModeSet(int mode);			// ANM
	MVPServer::Err AnalogInputModeGet(int *mode);			// ANM

    MVPServer::Err AnalogOutputSet(float value);			// ANO (fixed current limit)

    // digital input/output
    MVPServer::Err DigitalOutputSet(bool state);			// DACA
	MVPServer::Err DigitalInputMPMGet(int chan, bool *state);	// PI
    MVPServer::Err DigitalOutputMPMSet(int chan, bool state);	// PO
    MVPServer::Err DigitalInputAllMPMGet(int *state);			// XST

    // macros
	MVPServer::Err MacroExecute(int macro);						// ME
    MVPServer::Err MacroStatusGet(int *macro);					// MS

    // communication
    MVPServer::Err SerialResponseDelaySet(float delaySec);		// SD
    MVPServer::Err SerialResponseDelayGet(float *delaySec);		// SD
    
    MVPServer::Err SerialResponseOKSet(bool sayOK);				// OK
    MVPServer::Err SerialResponseOKGet(bool *sayOK);			// OK

	MVPServer::Err SerialChecksumSet(bool state);				// CK
	MVPServer::Err SerialChecksumGet(bool *state);				// CK
	MVPServer::Err SerialChecksumOff();							// CX

    // status
    MVPServer::Err StatusMoveInProgress(bool *moving);			// ST
    MVPServer::Err StatusMotorInPosn(bool *there);				// ST
    MVPServer::Err StatusVelocityMode(bool *mode);				// ST
    MVPServer::Err StatusAnalogInput(bool *input);				// ST

    MVPServer::Err StatusTrajPercComplete(bool *complete);		// ST
    MVPServer::Err StatusDeviceNet(bool *connected);			// ST
    MVPServer::Err StatusDeviceNetError(bool *err);				// ST
    MVPServer::Err StatusMoveError(bool *offcourse);			// ST

    MVPServer::Err StatusMotorDisabled(bool *disabled);			// ST
    MVPServer::Err StatusRangeLimit(bool *outof);				// ST
    MVPServer::Err StatusLocalMode(bool *active);				// ST
    MVPServer::Err StatusEmergencyStop(bool *active);			// ST

    MVPServer::Err StatusExternalEvent1(bool *active);			// ST
    MVPServer::Err StatusPositiveLimitFlag(bool *active);		// ST
    MVPServer::Err StatusExternalEvent2(bool *active);			// ST
    MVPServer::Err StatusNegativeLimitFlag(bool *active);		// ST

    MVPServer::Err StatusFirmwareVersionGet(char *str);			// FV

    // EEprom
	MVPServer::Err EEpromSave();								// EEPSAV

	MVPServer::Err EEpromBootSet(bool setting);					// EEBOOT
	MVPServer::Err EEpromBootGet(bool *setting);				// EEBOOT

	// conversion routines
	// can only set units, slope, and offset; slopeVel is computed
	void ConvEncoderSet(char *iUnits, float iSlope, float iOffset);
	void ConvEncoderGet(char *oUnits, float *oSlope, float *oOffset);
	char *UnitsEncoder();
	
	void ConvEncoderMPMSet(char *iUnits, float iSlope, float iOffset);
	void ConvEncoderMPMGet(char *oUnits, float *oSlope, float *oOffset);
	char *UnitsEncoderMPM();
		
	void ConvAnalogInputSet(char *iUnits, float iSlope, float iOffset);
	void ConvAnalogInputGet(char *oUnits, float *oSlope, float *oOffset);
	char *UnitsAnalogInput();
	                              
	void ConvAnalogInputMPMSet(char *iUnits, float iSlope, float iOffset);
	void ConvAnalogInputMPMGet(char *oUnits, float *oSlope, float *oOffset);
	char *UnitsAnalogInputMPM();

	void ConvAnalogOutputSet(char *iUnits, float iSlope, float iOffset);
	void ConvAnalogOutputGet(char *oUnits, float *oSlope, float *oOffset);
	char *UnitsAnalogOutput();
private:
    // client port for instructions
    MsgClientPort *clientport;

    // node number of MVP unit
    int node;
    
    // routine to send messages to and receive replies from the MVP Server
    MVPServer::Err instruction(int node, MVPServer::Cmd cmd, int param, 
                               bool paramActive, int *value);
    
    // sample period in sec (need to store for velocity conversion)
    float samplePeriod;

	//  for encoder: set slope = cpr * 4 * gearRatio / unitsPerTurn
	struct Conversion {
		char units[MVP_UNITS_LEN];
	    float slope;
	    float offset;
	};

    // linear conversion factors for MVP and MPM inputs & outputs
    Conversion encoder;
    Conversion encoderMPM;
    Conversion analogInput;
    Conversion analogInputMPM;
    Conversion analogOutput;
    
    // generic conversion routines
    void convSet(Conversion *c, char *iUnits, float iSlope, float iOffset);
    void convGet(Conversion *c, char *oUnits, float *oSlope, float *oOffset);
    
    // need velocity conversion for encoder only
    float encoderSlopeAccel;
    float encoderSlopePGain;
    float encoderSlopeIGain;
    float encoderSlopeDGain;
    
    // round to nearest integer
    int roundf(float x);
};

#endif // MVP_H
