/////////////////////////////////////////////////////////////////////////////
//
//    File: mvp.cc
//
//    Usage:
//        See mvp.h.
//
//    Description:
//        See mvp.h.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        August, 2003
//
/////////////////////////////////////////////////////////////////////////////

#include "mvp.h"

//
// MVP_CONV_VEL - velocity conversion constant (for encoder)
//
// The MVP velocity values depend on the encoder counts per revolution and
// the sample period, and are in units of rpm.  Namely,
//     true_rpm = commanded_rpm * 500cpr/E cpr * 500usec/S usec
//     where E = number of cpr for encoder, and S = sample period in usec
//     i.e., for E=500 and S=500, true_rpm=commanded_rpm
//
// But we want velocity to be in user-defined units/sec, and have the program
// calculate commanded_rpm.  To convert units/sec into commanded_rpm:
//     commanded_rpm = true_rpm * E cpr/500cpr * S usec/500usec
//                   = N units/sec * A quad_counts/unit * 60 sec/min
//                     * 1/(4*E) rev/quad_counts * E cpr/500cpr * S usec/500usec
//     where N = true velocity in units/sec, A = slope in quad_counts/unit
// So,
//     commanded_rpm = N * A * 60 * 1/4 * 1/500 * S/500
// If the sample period is in seconds,
//     commanded_rpm = vel * slope * samplePeriod * 60*1/4*1/500*1/0.0005
//                   = vel * slope * samplePeriod * 60
//
// Could calculate slope*samplePeriod*60 each time slope or samplePeriod
// changes, but not that costly to compute on the fly.
//
#define MVP_CONV_VEL (60.0)

MVP::MVP(char *serverID, int timeout, int nodeNum)
{
	// define client port and MVP node number
	clientport = new MsgClientPort(serverID, timeout);
    node = nodeNum;
    
    // set sample period in seconds, default is 500 usec
    samplePeriod = 0.0005;
    
    // initialize conversion factors (default slope=1.0, offset=0.0)
    convSet(&encoder, "quad counts", 1.0, 0.0);
    
    encoderSlopeAccel = 1.0;	// until figure out these
    encoderSlopePGain = 1.0;
    encoderSlopeIGain = 1.0;
    encoderSlopeDGain = 1.0;
  
    convSet(&encoderMPM, "quad counts", 1.0, 0.0);  
    convSet(&analogInput, "a/d counts", 1.0, 0.0);
    convSet(&analogInputMPM, "a/d counts", 1.0, 0.0);
    convSet(&analogOutput, "d/a counts", 1.0, 0.0);
}

MVPServer::Err MVP::NodeEnable()
{
    return instruction(node, MVPServer::CMD_EN, NULL, false, NULL);
}

MVPServer::Err MVP::NodeDisable()
{
    return instruction(node, MVPServer::CMD_DI, NULL, false, NULL);
}

MVPServer::Err MVP::NodeReset()
{
    return instruction(node, MVPServer::CMD_RN, NULL, false, NULL);
}

MVPServer::Err MVP::TargetPosnAbsSet(float posn)
{
    int quadCounts;

    // convert position into encoder quad counts
    quadCounts = roundf (posn * encoder.slope + encoder.offset);

    return instruction(node, MVPServer::CMD_LA, quadCounts, true, NULL);
}

MVPServer::Err MVP::TargetPosnAbsGet(float *posn)
{
    int quadCounts;
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_LA, 0, false, &quadCounts);

    // convert encoder quad counts into position
    *posn = (float(quadCounts) - encoder.offset) / encoder.slope;

    return stat;
}

MVPServer::Err MVP::TargetPosnRelSet(float posn)
{
    int quadCounts;

    // convert position into encoder quad counts
    quadCounts = roundf(posn * encoder.slope + encoder.offset);

    return instruction(node, MVPServer::CMD_LR, quadCounts, true, NULL);
}

MVPServer::Err MVP::TargetPosnRelGet(float *posn)
{
    int quadCounts;
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_LR, 0, false, &quadCounts);

    // convert encoder quad counts into position
    *posn = (float(quadCounts) - encoder.offset) / encoder.slope;

    return stat;
}

MVPServer::Err MVP::TargetVelMaxSet(float vel)
{
    int rpm;

    // convert velocity into encoder rpm
    rpm = roundf(vel * encoder.slope * samplePeriod * MVP_CONV_VEL);
    
    return instruction(node, MVPServer::CMD_SP, rpm, true, NULL);
}

MVPServer::Err MVP::TargetVelMaxGet(float *vel)
{
    int rpm;
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_SP, 0, false, &rpm);
    
    // convert encoder rpm into velocity
    *vel = float(rpm) / (encoder.slope * samplePeriod * MVP_CONV_VEL);

    return stat;
}

MVPServer::Err MVP::TargetAccelRateSet(float accel)
{
    int quadCountsPerSec2;

    // convert acceleration into encoder quad counts per sec^2
    quadCountsPerSec2 = roundf (accel * encoderSlopeAccel);

    return instruction(node, MVPServer::CMD_AC, quadCountsPerSec2, true, NULL);
}

MVPServer::Err MVP::TargetAccelRateGet(float *accel)
{
    int quadCountsPerSec2;
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_AC, 0, false, &quadCountsPerSec2);

    // convert encoder quad counts per sec^2 into acceleration
    *accel = float(quadCountsPerSec2) / encoderSlopeAccel;

    return stat;
}


MVPServer::Err MVP::TargetDecelRateSet(float decel)
{
    int quadCountsPerSec2;

    // convert deceleration into encoder quad counts per sec^2
    quadCountsPerSec2 = roundf (decel * encoderSlopeAccel);

    return instruction(node, MVPServer::CMD_DC, quadCountsPerSec2, true, NULL);
}

MVPServer::Err MVP::TargetDecelRateGet(float *decel)
{
    int quadCountsPerSec2;
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_DC, 0, false, &quadCountsPerSec2);

    // convert encoder quad counts per sec^2 into deceleration
    *decel = float(quadCountsPerSec2) / encoderSlopeAccel;

    return stat;
}

MVPServer::Err MVP::TargetVelConstantSet(float vel)
{
    int rpm;

    // convert velocity into encoder rpm
    rpm = roundf (vel * encoder.slope * samplePeriod * MVP_CONV_VEL);

    return instruction(node, MVPServer::CMD_V, rpm, true, NULL);
}

MVPServer::Err MVP::TargetVelConstantGet(float *vel)
{
    int rpm;
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_V, 0, false, &rpm);

    // convert encoder rpm into velocity
    *vel = float(rpm) / (encoder.slope * samplePeriod * MVP_CONV_VEL);

    return stat;
}

MVPServer::Err MVP::ActualPosnGet(float *posn)
{
    int quadCounts;
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_POS, 0, false, &quadCounts);

    // convert encoder quad counts into position
    *posn = (float(quadCounts) - encoder.offset) / encoder.slope;

    return stat;
}

MVPServer::Err MVP::ActualPosnErrorGet(float *posnErr)
{
    int quadCountsErr;
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_ERR, 0, false, &quadCountsErr);

    // convert encoder error quad counts into position error
    *posnErr = float(quadCountsErr) / encoder.slope;

    return stat;
}

MVPServer::Err MVP::ActualPosnMPMGet(float *posnMPM)
{
    int quadCounts;
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_EP, 0, false, &quadCounts);

    // convert encoder quad counts into position
    *posnMPM = (float(quadCounts) - encoderMPM.offset) / encoderMPM.slope;

    return stat;
}

MVPServer::Err MVP::ActualVelGet(float *vel)
{
    int rpm;
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_AS, 0, false, &rpm);

    // convert encoder rpm into velocity
    *vel = float(rpm) / (encoder.slope * samplePeriod * MVP_CONV_VEL);

    return stat;
}

MVPServer::Err MVP::TargetPosnInRangeSet(float posnIn)
{
    int quadCounts;

    // convert position into encoder quad counts
    quadCounts = roundf (posnIn * encoder.slope + encoder.offset);

    return instruction(node, MVPServer::CMD_N, quadCounts, true, NULL);
}

MVPServer::Err MVP::TargetPosnInRangeGet(float *posnIn)
{
    int quadCounts;
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_N, 0, false, &quadCounts);

    // convert encoder quad counts into position
    *posnIn = (float(quadCounts) - encoder.offset) / encoder.slope;

    return stat;
}

MVPServer::Err MVP::TargetPosnLimitMinSet(float posnMin)
{
    int quadCounts;

    // convert position into encoder quad counts
    quadCounts = roundf (posnMin * encoder.slope + encoder.offset);
    if ( quadCounts >= 0 ) {
        // needed to specify negative position
        return MVPServer::ERR_PARAMETER_OUT_OF_RANGE;
    }

    return instruction(node, MVPServer::CMD_LL, quadCounts, true, NULL);
}

MVPServer::Err MVP::TargetPosnLimitMinGet(float *posnMax)
{
    int quadCounts;
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_LLM, 0, false, &quadCounts);

    // convert encoder quad counts into position
    *posnMax = (float(quadCounts) - encoder.offset) / encoder.slope;

    return stat;
}

MVPServer::Err MVP::TargetPosnLimitMaxSet(float posnMax)
{
    int quadCounts;

    // convert position into encoder quad counts
    quadCounts = roundf (posnMax * encoder.slope + encoder.offset);

    return instruction(node, MVPServer::CMD_LL, quadCounts, true, NULL);
}

MVPServer::Err MVP::TargetPosnLimitMaxGet(float *posnMax)
{
    int quadCounts;
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_LL, 0, false, &quadCounts);

    // convert encoder quad counts into position
    *posnMax = (float(quadCounts) - encoder.offset) / encoder.slope;

    return stat;
}

MVPServer::Err MVP::ActionPosnOutOfLimitSet(int code)
{
    return instruction(node, MVPServer::CMD_SA, code, true, NULL);
}

MVPServer::Err MVP::ActionPosnOutOfLimitGet(int *code)
{
    return instruction(node, MVPServer::CMD_SA, 0, false, code);
}

MVPServer::Err MVP::PowerLimitPWMOutputSet(float percent)
{
    return instruction(node, MVPServer::CMD_SM, roundf (percent*10), true, NULL);
}

MVPServer::Err MVP::PowerLimitPWMOutputGet(float *percent)
{
    int per10;
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_SM, 0, false, &per10);

	// convert from integer percent * 10 to percent
	*percent = float(per10) / 10;
	
	return stat;
}

// sorry to hard code numbers, but confirm on HPD chart
MVPServer::Err MVP::PowerLimitCurrentSet(float amps)
{
    int anoValue;
    
    // convert amps into ano value (0.0 <= amps <= 18.2)
    if ( amps < 0.0 ) {
    	return MVPServer::ERR_PARAMETER_OUT_OF_RANGE;
    } else if ( (amps >= 0.0) && (amps < 0.4) ) {
    	anoValue = roundf(amps * 500.0 + 2000.0);
    } else if ( (amps >= 0.4) && (amps < 1.7) ) {
    	anoValue = roundf(amps * 226.8 + 2111.9);
    } else if ( (amps >= 1.7) && (amps < 3.5) ) {
    	anoValue = roundf(amps * 165.7 + 2227.3);
    } else if ( (amps >= 3.5) && (amps < 6.2) ) {
    	anoValue = roundf(amps * 112.1 + 2403.2);
    } else if ( (amps >= 6.2) && (amps < 9.7) ) {
    	anoValue = roundf(amps * 86.76 + 2569.0);
    } else if ( (amps >= 9.7) && (amps < 14.8) ) {
    	anoValue = roundf(amps * 59.28 + 2829.7);
    } else if ( (amps >= 14.8) && (amps <= 18.2) ) {
    	anoValue = roundf(amps * 90.00 + 2374.0);
    } else { // amps > 18.2
    	return MVPServer::ERR_PARAMETER_OUT_OF_RANGE;
    }

    return instruction(node, MVPServer::CMD_ANO, anoValue, true, NULL);
}

MVPServer::Err MVP::PowerLimitCurrentGet(float *amps)
{
    int anoValue;
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_ANO, 0, false, &anoValue);

    // convert anoValue into amps
    if ( anoValue < 2000 ) {
    	return MVPServer::ERR_PARAMETER_OUT_OF_RANGE;
    } else if ( (anoValue >= 0) && (anoValue < 2200) ) {
    	*amps = (float(anoValue) - 2000.0) / 500.0;
    } else if ( (anoValue >= 2200) && (anoValue < 2500) ) {
    	*amps = (float(anoValue) - 2111.9) / 226.8;
    } else if ( (anoValue >= 2500) && (anoValue < 2800) ) {
    	*amps = (float(anoValue) - 2227.3) / 165.7;
    } else if ( (anoValue >= 2800) && (anoValue < 3100) ) {
    	*amps = (float(anoValue) - 2403.2) / 112.1;
    } else if ( (anoValue >= 3100) && (anoValue < 3400) ) {
    	*amps = (float(anoValue) - 2569.0) / 86.76;
    } else if ( (anoValue >= 3400) && (anoValue < 3700) ) {
    	*amps = (float(anoValue) - 2829.7) / 59.28;
    } else if ( (anoValue >= 3700) && (anoValue < 4000) ) {
    	*amps = (float(anoValue) - 2374.0) / 90.00;
    } else if ( (anoValue >= 4000) && (anoValue <= 4095) ) {
    	*amps = 18.2;
    } else { // anoValue > 4095
    	return MVPServer::ERR_PARAMETER_OUT_OF_RANGE;
    }

    return stat;
}

MVPServer::Err MVP::Home()
{
    return MVP::Home(0.0);
}

MVPServer::Err MVP::Home(float posn)
{
    int quadCounts;

    // convert position into encoder quad counts
    quadCounts = roundf (posn * encoder.slope + encoder.offset);
   
    return instruction(node, MVPServer::CMD_HO, quadCounts, true, NULL);
}

MVPServer::Err MVP::HomeArmSet(bool setting)
{
    return instruction(node, MVPServer::CMD_HA, int(setting), true, NULL);
}

MVPServer::Err MVP::HomeArmGet(bool *setting)
{
    int result;
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_HS, 0, false, &result);

    // home arm status is bit 1
    *setting = result & 0x01;

    return stat;
}

MVPServer::Err MVP::HomeArmPolaritySet(int setting)
{
    return instruction(node, MVPServer::CMD_HP, setting, true, NULL);
}

MVPServer::Err MVP::HomeArmPolarityGet(int *setting)
{
    int result;
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_HS, 0, false, &result);

    // home arm polarity status is bit 2
    *setting = result & 0x02;

    return stat;
}

MVPServer::Err MVP::ActionHomeSequenceSet(int code)
{
    return instruction(node, MVPServer::CMD_HF, code, true, NULL);
}

MVPServer::Err MVP::ActionHomeSequenceGet(int *code)
{
    return instruction(node, MVPServer::CMD_HF, 0, false, code);
}

MVPServer::Err MVP::HomeEnableOnBootSet(bool setting)
{
    return instruction(node, MVPServer::CMD_AE, int(setting), true, NULL);
}

MVPServer::Err MVP::HomeEnableOnBootGet(bool *setting)
{
    return instruction(node, MVPServer::CMD_AE, 0, false, (int *) setting);
}

MVPServer::Err MVP::MotionMove()
{
    return instruction(node, MVPServer::CMD_M, 0, false, NULL);
}

MVPServer::Err MVP::MotionAbort()
{
    return instruction(node, MVPServer::CMD_AB, 0, false, NULL);
}

MVPServer::Err MVP::MotionAbortDecelRateSet(float decel)
{
    int quadCountsPerSec2;

    // convert deceleration into encoder quad counts per sec^2
    quadCountsPerSec2 = roundf (decel * encoderSlopeAccel);

    return instruction(node, MVPServer::CMD_AD, quadCountsPerSec2, true, NULL);
}

MVPServer::Err MVP::MotionAbortDecelRateGet(float *decel)
{
    int quadCountsPerSec2;
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_AD, 0, false, &quadCountsPerSec2);

    // convert encoder quad counts per sec^2 into deceleration
    *decel = float(quadCountsPerSec2) / encoderSlopeAccel;

    return stat;
}

MVPServer::Err MVP::ActionMotionAbortSet(int code)
{
    return instruction(node, MVPServer::CMD_AA, code, true, NULL);
}

MVPServer::Err MVP::ActionMotionAbortGet(int *code)
{
    return instruction(node, MVPServer::CMD_AA, 0, false, code);
}

MVPServer::Err MVP::ErrorPosnMaxSet(float posnErr)
{
    int quadCountsErr;

    // convert position into encoder quad counts
    quadCountsErr = roundf (posnErr * encoder.slope);

    return instruction(node, MVPServer::CMD_FD, quadCountsErr, true, NULL);
}

MVPServer::Err MVP::ErrorPosnMaxGet(float *posnErr)
{
    int quadCountsErr;
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_FD, 0, false, &quadCountsErr);

    // convert encoder error quad counts into position error
    *posnErr = float(quadCountsErr) / encoder.slope;

    return stat;
}

MVPServer::Err MVP::ErrorPosnFlagDelaySet(float delaySec)
{
    int units;

    if ( delaySec < 0 ) {
        return MVPServer::ERR_PARAMETER_OUT_OF_RANGE;
    } else if ( delaySec == 0 ) {
        units = 0;
    } else {
		// each unit is 2 ms
	    units = roundf (delaySec * 500);
	}

    return instruction(node, MVPServer::CMD_FDT, units, true, NULL);
}

MVPServer::Err MVP::ErrorPosnFlagDelayGet(float *delaySec)
{
    int units;
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_FDT, 0, false, &units);

	// each unit is 2 ms
	*delaySec = float(units) / 500; 

	return stat;
}

MVPServer::Err MVP::ActionErrorPosnSet(int code)
{
    return instruction(node, MVPServer::CMD_FA, code, true, NULL);
}

MVPServer::Err MVP::ActionErrorPosnGet(int *code)
{
    return instruction(node, MVPServer::CMD_FA, 0, false, code);
}

// assume units of gain are current/quad counts
MVPServer::Err MVP::GainProportionalSet(float prop)
{
    int propEnc;

    // convert position into encoder quad counts
    propEnc = roundf(prop / encoderSlopePGain);

    return instruction(node, MVPServer::CMD_POR, propEnc, true, NULL);
}

MVPServer::Err MVP::GainProportionalGet(float *prop)
{
    int propEnc;
    MVPServer::Err stat;

	stat = instruction(node, MVPServer::CMD_POR, 0, false, &propEnc);

	// convert encoder quad counts into position
	*prop = float(propEnc) * encoderSlopePGain;

	return stat;
}

// assume units of gain are current / (position * sec)
MVPServer::Err MVP::GainIntegralSet(float integr)
{
    int integrEnc;

    // convert position into encoder quad counts
    integrEnc = roundf (integr / encoderSlopeIGain);

    return instruction(node, MVPServer::CMD_I, integrEnc, true, NULL);
}

MVPServer::Err MVP::GainIntegralGet(float *integr)
{
    int integrEnc;
    MVPServer::Err stat;

	stat = instruction(node, MVPServer::CMD_I, 0, false, &integrEnc);

	// convert encoder quad counts into position
	*integr = float(integrEnc) * encoderSlopeIGain;

	return stat;
}

// assume units of gain are current / (position / sec)
MVPServer::Err MVP::GainDerivativeSet(float deriv)
{
    int derivEnc;

    // convert position into encoder quad counts
    derivEnc = roundf (deriv / encoderSlopeDGain);

    return instruction(node, MVPServer::CMD_DER, derivEnc, true, NULL);
}

MVPServer::Err MVP::GainDerivativeGet(float *deriv)
{
    int derivEnc;
    MVPServer::Err stat;

	stat = instruction(node, MVPServer::CMD_DER, 0, false, &derivEnc);

	// convert encoder quad counts into position
	*deriv = float(derivEnc) * encoderSlopeDGain;

	return stat;
}

MVPServer::Err MVP::SamplePeriodSet(float periodSec)
{
    int usec;

    // need period to be greater than 0
    if ( periodSec <= 0.0 ) {
	    return MVPServer::ERR_PARAMETER_OUT_OF_RANGE;
	}
	
	// store new sample period for velocity conversion
	samplePeriod = periodSec;
	
    // convert seconds into usec
    usec = roundf (periodSec * 1000000.0);

    return instruction(node, MVPServer::CMD_SR, usec, true, NULL);
}

MVPServer::Err MVP::SamplePeriodGet(float *periodSec)
{
    int usec;
    MVPServer::Err stat;

	stat = instruction(node, MVPServer::CMD_SR, 0, false, &usec);

	// convert usec into seconds
	*periodSec = float(usec) / 1000000.0;

	return stat;
}

MVPServer::Err MVP::AnalogInputValueGet(float *value)
{
    int adCount; // will be value between 0 and 1024
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_ANI, 0, false, &adCount);

    // convert adCount into value
    *value = (float(adCount) - analogInput.offset) / analogInput.slope;

    return stat;
}

// use when analog input mode is digital
MVPServer::Err MVP::AnalogInputValueGet(bool *state)
{
    return instruction(node, MVPServer::CMD_ANI, 0, false, (int *) state);
}

MVPServer::Err MVP::AnalogInputValueMPMGet(float *valueMPM)
{
    int adCount; // will be value between 0 and 5000
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_EAI, 0, false, &adCount);

    // convert adCount into value
    *valueMPM = (float(adCount) - analogInputMPM.offset) / analogInputMPM.slope;

    return stat;
}

MVPServer::Err MVP::AnalogInputModeSet(int mode)
{
    return instruction(node, MVPServer::CMD_ANM, mode, true, NULL);
}

MVPServer::Err MVP::AnalogInputModeGet(int *mode)
{
    return instruction(node, MVPServer::CMD_ANM, 0, false, mode);
}

// available with fixed current limit
MVPServer::Err MVP::AnalogOutputSet(float value)
{
    int daCount;

    // convert value into daCount
    daCount = roundf (value * analogOutput.slope + analogOutput.offset);

    return instruction(node, MVPServer::CMD_ANO, daCount, true, NULL);
}

MVPServer::Err MVP::DigitalOutputSet(bool state)
{
    return instruction(node, MVPServer::CMD_DACA, int(state), true, NULL);
}

MVPServer::Err MVP::DigitalInputMPMGet(int chan, bool *state)
{
    // only have 4 channels available, 1-4
    if ( (chan < 0) || (chan > 4) ) {
        return MVPServer::ERR_PARAMETER_OUT_OF_RANGE;
    }

    return instruction(node, MVPServer::CMD_PI, chan, true, (int *) state);
}

MVPServer::Err MVP::DigitalOutputMPMSet(int chan, bool state)
{
    int param;

	// form channel+state int parameter (add comma in Command())
	param = chan*10 + int(state);

    return instruction(node, MVPServer::CMD_PO, param, true, NULL);
}

MVPServer::Err MVP::DigitalInputAllMPMGet(int *state)
{
	return instruction(node, MVPServer::CMD_XST, 0, false, state);
}

MVPServer::Err MVP::MacroExecute(int macro)
{
    // need positive macro number
	if ( macro <= 0 ) { // don't know if macro number 0 is allowed; check
	    return MVPServer::ERR_PARAMETER_OUT_OF_RANGE;
	}

    return instruction(node, MVPServer::CMD_ME, macro, true, NULL);
}

MVPServer::Err MVP::MacroStatusGet(int *macro)
{
    return instruction(node, MVPServer::CMD_MS, 0, false, macro);
}

MVPServer::Err MVP::SerialResponseDelaySet(float delaySec)
{
    int units;

    if ( delaySec < 0 ) {
        return MVPServer::ERR_PARAMETER_OUT_OF_RANGE;
    } else if ( delaySec == 0 ) {
        units = 0;
    } else {
		// each unit is 500 usec 
	    units = roundf (delaySec * 2000);
	}

    return instruction(node, MVPServer::CMD_SD, units, true, NULL);
}

MVPServer::Err MVP::SerialResponseDelayGet(float *delaySec)
{
    int units;
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_FDT, 0, false, &units);

	// each unit is 500 usec
	*delaySec = float(units) / 2000; 

	return stat;
}

MVPServer::Err MVP::SerialResponseOKSet(bool sayOK)
{
    return instruction(node, MVPServer::CMD_SD, int(sayOK), true, NULL);
}

MVPServer::Err MVP::SerialResponseOKGet(bool *sayOK)
{
    return instruction(node, MVPServer::CMD_SD, 0, false, (int *) sayOK);
}

MVPServer::Err MVP::SerialChecksumSet(bool state)
{
	return instruction(node, MVPServer::CMD_CK, int(state), true, NULL);
}

MVPServer::Err MVP::SerialChecksumGet(bool *state)
{
	return instruction(node, MVPServer::CMD_CK, 0, false, (int *) state);
}

MVPServer::Err MVP::SerialChecksumOff()
{
	return instruction(node, MVPServer::CMD_CX, 0, false, NULL);
}
	
	
// probably should do something fancier to handle status bits,
// but this will probably work, and be easy on user
MVPServer::Err MVP::StatusMoveInProgress(bool *moving)
{
    int sbits;
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_ST, 0, false, &sbits);

	// extract out bit of interest
	*moving = sbits & 0x0001;

	return stat;
}

MVPServer::Err MVP::StatusMotorInPosn(bool *there)
{
    int sbits;
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_ST, 0, false, &sbits);

	// extract out bit of interest
	*there = sbits & 0x0002;

	return stat;
}

MVPServer::Err MVP::StatusVelocityMode(bool *mode)
{
    int sbits;
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_ST, 0, false, &sbits);

	// extract out bit of interest
	*mode = sbits & 0x0004;

	return stat;
}

MVPServer::Err MVP::StatusAnalogInput(bool *input)
{
    int sbits;
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_ST, 0, false, &sbits);

	// extract out bit of interest
	*input = sbits & 0x0008;

	return stat;
}

MVPServer::Err MVP::StatusTrajPercComplete(bool *complete)
{
    int sbits;
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_ST, 0, false, &sbits);

	// extract out bit of interest
	*complete = sbits & 0x0010;

	return stat;
}

MVPServer::Err MVP::StatusDeviceNet(bool *connected)
{
    int sbits;
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_ST, 0, false, &sbits);

	// extract out bit of interest
	*connected = sbits & 0x0020;

	return stat;
}

MVPServer::Err MVP::StatusDeviceNetError(bool *err)
{
    int sbits;
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_ST, 0, false, &sbits);

	// extract out bit of interest
	*err = sbits & 0x0040;

	return stat;
}

MVPServer::Err MVP::StatusMoveError(bool *offcourse)
{
    int sbits;
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_ST, 0, false, &sbits);

	// extract out bit of interest
	*offcourse = sbits & 0x0080;

	return stat;
}

MVPServer::Err MVP::StatusMotorDisabled(bool *disabled)
{
    int sbits;
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_ST, 0, false, &sbits);

	// extract out bit of interest
	*disabled = sbits & 0x0100;

	return stat;
}

MVPServer::Err MVP::StatusRangeLimit(bool *outof)
{
    int sbits;
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_ST, 0, false, &sbits);

	// extract out bit of interest
	*outof = sbits & 0x0200;

	return stat;
}

MVPServer::Err MVP::StatusLocalMode(bool *active)
{
    int sbits;
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_ST, 0, false, &sbits);

	// extract out bit of interest
	*active = sbits & 0x0400;

	return stat;
}

MVPServer::Err MVP::StatusEmergencyStop(bool *active)
{
    int sbits;
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_ST, 0, false, &sbits);

	// extract out bit of interest
	*active = sbits & 0x0800;

	return stat;
}

MVPServer::Err MVP::StatusExternalEvent1(bool *active)
{
    int sbits;
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_ST, 0, false, &sbits);

	// extract out bit of interest
	*active = sbits & 0x1000;

	return stat;
}

MVPServer::Err MVP::StatusPositiveLimitFlag(bool *active)
{
    int sbits;
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_ST, 0, false, &sbits);

	// extract out bit of interest
	*active = sbits & 0x2000;

	return stat;
}

MVPServer::Err MVP::StatusExternalEvent2(bool *active)
{
    int sbits;
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_ST, 0, false, &sbits);

	// extract out bit of interest
	*active = sbits & 0x4000;

	return stat;
}

MVPServer::Err MVP::StatusNegativeLimitFlag(bool *active)
{
    int sbits;
    MVPServer::Err stat;

    stat = instruction(node, MVPServer::CMD_ST, 0, false, &sbits);

	// extract out bit of interest
	*active = sbits & 0x8000;

	return stat;
}

MVPServer::Err MVP::StatusFirmwareVersionGet(char *str)
{
    return instruction(node, MVPServer::CMD_FV, 0, false, (int *) str);
}

MVPServer::Err MVP::EEpromSave()
{
    return instruction(node, MVPServer::CMD_EEPSAV, 0, false, NULL);
}

MVPServer::Err MVP::EEpromBootSet(bool setting)
{
    return instruction(node, MVPServer::CMD_EEBOOT, (int) setting, true, NULL);
}

MVPServer::Err MVP::EEpromBootGet(bool *setting)
{
    return instruction(node, MVPServer::CMD_EEBOOT, 0, false, 
                         (int *) setting);
}

void MVP::ConvEncoderSet(char *iUnits, float iSlope, float iOffset)
{
	convSet(&encoder, iUnits, iSlope, iOffset);
}

void MVP::ConvEncoderGet(char *oUnits, float *oSlope, float *oOffset)
{
	convGet(&encoder, oUnits, oSlope, oOffset);
}

char *MVP::UnitsEncoder()
{	
	return encoder.units;
}

void MVP::ConvEncoderMPMSet(char *iUnits, float iSlope, float iOffset)
{
	convSet(&encoderMPM, iUnits, iSlope, iOffset);
}

void MVP::ConvEncoderMPMGet(char *oUnits, float *oSlope, float *oOffset)
{
	convGet(&encoderMPM, oUnits, oSlope, oOffset);
}

char *MVP::UnitsEncoderMPM()
{
	return encoderMPM.units;
}

void MVP::ConvAnalogInputSet(char *iUnits, float iSlope, float iOffset)
{
	convSet(&analogInput, iUnits, iSlope, iOffset);
}

void MVP::ConvAnalogInputGet(char *oUnits, float *oSlope, float *oOffset)
{
	convGet(&analogInput, oUnits, oSlope, oOffset);
}                          

char *MVP::UnitsAnalogInput()
{
	return analogInput.units;
}

void MVP::ConvAnalogInputMPMSet(char *iUnits, float iSlope, float iOffset)
{
	convSet(&analogInputMPM, iUnits, iSlope, iOffset);
}

void MVP::ConvAnalogInputMPMGet(char *oUnits, float *oSlope, float *oOffset)
{
	convGet(&analogInputMPM, oUnits, oSlope, oOffset);
}

char *MVP::UnitsAnalogInputMPM()
{
	return analogInputMPM.units;
}

void MVP::ConvAnalogOutputSet(char *iUnits, float iSlope, float iOffset)
{
	convSet(&analogOutput, iUnits, iSlope, iOffset);
}

void MVP::ConvAnalogOutputGet(char *oUnits, float *oSlope, float *oOffset)
{
	convGet(&analogOutput, oUnits, oSlope, oOffset);
}

char *MVP::UnitsAnalogOutput()
{
	return analogOutput.units;
}

MVPServer::Err MVP::instruction(int node, MVPServer::Cmd cmd, int param, 
                           bool paramActive, int *value)
{
	MVPServerMsg msg;
	int err;

	msg.m_inst.m_msgtype = char4('I','N','S','T');
	msg.m_inst.node = node;
	msg.m_inst.cmd = cmd;
	msg.m_inst.param = param;
	msg.m_inst.paramActive = paramActive;

	err = clientport->MsgSend(msg, msg);
	if ( err ) {
		perror("MVP::instruction: MsgSend failed");
		return MVPServer::ERR_MSGSEND_FAILED;
	} else {
		// if value is NULL, don't care about return value
		if ( value != NULL ) {
			*value = msg.m_inst.value;
		}
		return msg.m_inst.err;
	}
}

void MVP::convSet(MVP::Conversion *c, char *iUnits, float iSlope, float iOffset)
{
	strncpy(c->units, iUnits, MVP_UNITS_LEN);
	c->slope = iSlope;
	c->offset = iOffset;
}

void MVP::convGet(Conversion *c, char *oUnits, float *oSlope, float *oOffset)
{
	strncpy(oUnits, c->units, MVP_UNITS_LEN);
	*oSlope  = c->slope;
	*oOffset = c->offset;
}

int MVP::roundf(float x)
{
	return int (x + (x<0 ? -0.5 : 0.5));
}
