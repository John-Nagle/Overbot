/////////////////////////////////////////////////////////////////////////////
//
//    File: mvpmenu.cc
//
//    Usage:
//        see mvpmenu.h
//
//    Description:
//        see mvpmenu.h
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        August, 2003
//
/////////////////////////////////////////////////////////////////////////////

#include "mvpmenu.h"

//
//    MVPMenu::Display - display options of mvp menu
//
//    Returns nothing
//
void MVPMenu::Display()
{
    printf("  h - home/enable node         g - pid gains\n");
    printf("  d - disable node             p - sample period\n");
    printf("  a - actual posn, error, vel  A - analog input mode\n");
    printf("  t - target profile           i - analog inputs\n");
    printf("  v - constant velocity        o - analog output\n");
    printf("  e - position error handling  I - digital inputs - each/all\n");
	printf("  l - position limits          O - digital outputs\n");
	printf("  L - current/PWM limits       r - macro status and execute\n");
	printf("  m - motion                   c - communication parameters\n");
    printf("  b - abort motion             E - EEprom\n");
    printf("  B - abort motion parameters  s - status bits\n");
    printf("  x - arm home                 S - status all\n");
    printf("  H - home parameters          C - conversion factors\n");
    printf("  V - view inputs continuously R - reset recover\n");
}

//
//    MVPMenu::Process - process options for MVP menu
//
//    Returns true if menu option processed, false if not
//
bool MVPMenu::Process(char c)
{
    switch (c) {
	    case 'h':
		    nodeHomeEnable();
		    break;
	    case 'd':
		    nodeDisable();
		    break;
	    case 'a':
		    actualPosnVelError();
		    break;
	    case 't':
		    targetProfile();
		    break;
	    case 'v':
		    constantVelocity();
		    break;
	    case 'e':
		    errorHandling();
		    break;
	    case 'l':
		    limitsPosn();
		    break;
	    case 'L':
		    limitsCurrentPWM();
		    break;
	    case 'm':
		    motion();
		    break;
	    case 'b':
		    motionAbort();
		    break;
	    case 'B':
		    motionAbortParameters();
		    break;
	    case 'x':
		    homeArm();
		    break;
	    case 'H':
		    homeParameters();
		    break;
	    case 'V':
		    viewInputs();
		    break;
	    case 'g':
		    gains();
		    break;
	    case 'p':
		    samplePeriod();
		    break;
	    case 'A':
		    analogInputMode();
		    break;
	    case 'i':
		    analogInputs();
		    break;
	    case 'o':
		    analogOutput();
		    break;
	    case 'I':
		    digitalInputs();
		    break;
	    case 'O':
		    digitalOutputs();
		    break;
	    case 'r':
		    macroStatusExecute();
		    break;
	    case 'c':
		    communicationParameters();
		    break;
	    case 'E':
		    eeprom();
		    break;
	    case 's':
		    statusBits();
		    break;
	    case 'S':
		    statusAll();
		    break;
	    case 'C':
		    convFactors();
		    break;
	    case 'R':
		    resetRecover();
		    break;
		default:
		    return false;
			break;
	}

	return true;
}

//
//    MVPMenu::nodeHomeEnable - home and enable menu option
//
//    Default home position is current position.  This allows this command
//    to be used to just enable the motor.
//
//    Returns nothing
//
void MVPMenu::nodeHomeEnable()
{
    float posn;
    char prompt[MVPMENU_PROMPT_LEN];
    MVPServer::Err err;

    // home encoders
    if ( Ask::YesNo("Home encoder(s)", true) ) {
    	if ( (err = m->ActualPosnGet(&posn)) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
    		        "MVPMenu::nodeHomeEnable - not able to get position (%s)\n", 
    		        MVPServer::ErrMsg(err));
	    	posn = 0.0;
		}
		snprintf(prompt, MVPMENU_PROMPT_LEN, "Home position (%s)",
		         m->UnitsEncoder());
		posn = Ask::Float(prompt, posn);
	    if ( (err = m->Home(posn)) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
    		        "MVPMenu::nodeHomeEnable - not able to home encoders (%s)\n", 
    		        MVPServer::ErrMsg(err));
		}
	}
	
    // enable node
    if ( Ask::YesNo("Enable node", true) ) {
		if ( (err = m->NodeEnable()) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
    		        "MVPMenu::nodeHomeEnable - not able to enable node (%s)\n", 
    		        MVPServer::ErrMsg(err));
		}
	}

    Ask::Pause();
}

void MVPMenu::nodeDisable()
{
	MVPServer::Err err;
	
    // disable node
    if ( Ask::YesNo("Disable node", true) ) {
		if ( (err = m->NodeDisable()) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
    		        "MVPMenu::nodeDisable - not able to disable node (%s)\n", 
    		        MVPServer::ErrMsg(err));
		}
	}

    Ask::Pause();
}

void MVPMenu::actualPosnVelError()
{
    float posn, posnErr, posnMPM, vel;
    MVPServer::Err err;

    // actual position
    if ( (err = m->ActualPosnGet(&posn)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
    	        "MVPMenu::actualPosnVelError - not able to get position (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else {
		printf("Actual position (%s): %f\n", m->UnitsEncoder(), posn);
	}

    // actual position error
    if ( (err = m->ActualPosnErrorGet(&posnErr)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
    	        "MVPMenu::actualPosnVelError - not able to get position error (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else {
		printf("Actual position error (%s): %f\n", m->UnitsEncoder(), posnErr);
	}

    // actual velocity
    if ( (err = m->ActualVelGet(&vel)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
    	        "MVPMenu::actualPosnVelError - not able to get velocity %s)\n", 
    	        MVPServer::ErrMsg(err));
	} else {
		printf("Actual velocity (%s/sec): %f\n", m->UnitsEncoder(), vel);
	}

    // actual MPM position
    if ( (err = m->ActualPosnMPMGet(&posnMPM)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
    	        "MVPMenu::actualPosnVelError - not able to get MPM position (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else {
		printf("Actual position MPM (%s): %f\n", m->UnitsEncoderMPM(), 
		       posnMPM);
	}

	Ask::Pause();
}

void MVPMenu::targetProfile()
{
    static char absRel = 'a';
    float posn;
    float vel;
    float accel;
    float decel;
    char prompt[MVPMENU_PROMPT_LEN];
    MVPServer::Err err;

    // target position
    if ( (absRel = Ask::Char("Absolute (a) or Relative (r) position", absRel,
	                         "ar")) == 'a' ) {
		// absolute
		if ( (err = m->TargetPosnAbsGet(&posn)) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
    	      "MVPMenu::targetProfile - not able to get default position (%s)\n", 
    	      MVPServer::ErrMsg(err));
		} else {
			snprintf(prompt, MVPMENU_PROMPT_LEN, 
					 "Target absolute position (%s)",
			         m->UnitsEncoder());
			posn = Ask::Float(prompt, posn);
			if ( (err = m->TargetPosnAbsSet(posn)) != MVPServer::ERR_OK ) {
    			fprintf(stderr, 
    	        "MVPMenu::targetProfile - not able to set target position (%s)\n", 
    	        MVPServer::ErrMsg(err));
			}
		}
	} else {
	    // relative
		if ( (err = m->TargetPosnRelGet(&posn)) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
    	      "MVPMenu::targetProfile - not able to get default position (%s)\n", 
    	      MVPServer::ErrMsg(err));
		} else {
			snprintf(prompt, MVPMENU_PROMPT_LEN, 
					 "Target relative position (%s)",
			         m->UnitsEncoder());
			posn = Ask::Float(prompt, posn);
			if ( (err = m->TargetPosnRelSet(posn)) != MVPServer::ERR_OK ) {
    			fprintf(stderr, 
    	        "MVPMenu::targetProfile - not able to set target position (%s)\n", 
    	        MVPServer::ErrMsg(err));
			}
		}
	}

	// target maximum velocity
    if ( (err = m->TargetVelMaxGet(&vel)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
      "MVPMenu::targetProfile - not able to get default maximum velocity (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else {
		snprintf(prompt, MVPMENU_PROMPT_LEN, "Target maximum velocity (%s/sec)",
		         m->UnitsEncoder());
	    vel = Ask::Float(prompt, vel);
		if ( (err = m->TargetVelMaxSet(vel)) != MVPServer::ERR_OK ) {
    			fprintf(stderr, 
      "MVPMenu::targetProfile - not able to set target maximum velocity (%s)\n", 
    	                MVPServer::ErrMsg(err));
		}
	}

	// target acceleration rate
    if ( (err = m->TargetAccelRateGet(&accel)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
      "MVPMenu::targetProfile - not able to get default acceleration rate (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else {
	    accel = Ask::Float("Target acceleration rate (UNITS)", accel);
		if ( (err = m->TargetAccelRateSet(accel)) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
      "MVPMenu::targetProfile - not able to set target acceleration rate (%s)\n", 
    	            MVPServer::ErrMsg(err));
		}
	}

	// target deceleration rate
    if ( (err = m->TargetDecelRateGet(&decel)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
      "MVPMenu::targetProfile - not able to get default deceleration rate (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else {
	    decel = Ask::Float("Target deceleration rate (UNITS)", decel);
		if ( (err = m->TargetDecelRateSet(decel)) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
      "MVPMenu::targetProfile - not able to set default deceleration rate (%s)\n", 
    	            MVPServer::ErrMsg(err));
		}
	}

	printf("Need to select 'm - motion' to begin profile.\n");

	Ask::Pause();
}

void MVPMenu::constantVelocity()
{
    float vel;
    char prompt[MVPMENU_PROMPT_LEN];
    MVPServer::Err err;

	// target constant velocity
    if ( (err = m->TargetVelConstantGet(&vel)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
   "MVPMenu::constantVelocity - not able to get default constant velocity (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else {
		snprintf(prompt, MVPMENU_PROMPT_LEN, 
		         "Target constant velocity (%s/sec)",
		         m->UnitsEncoder());
	    vel = Ask::Float(prompt, vel, -1000000, 1000000);
		if ( (err = m->TargetVelConstantSet(vel)) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
   "MVPMenu::constantVelocity - not able to set target constant velocity (%s)\n", 
    	            MVPServer::ErrMsg(err));
		}
	}

	Ask::Pause();
}

void MVPMenu::errorHandling()
{
    float posnErr;
	float sec;
	int action;
    char prompt[MVPMENU_PROMPT_LEN];
    MVPServer::Err err;

	// max position error
    if ( (err = m->ErrorPosnMaxGet(&posnErr)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
		"MVPMenu::errorHandling - not able to get maximum position error (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else {
		snprintf(prompt, MVPMENU_PROMPT_LEN, 
		       "Maximum position error to take action (%s)\n", m->UnitsEncoder());
	    posnErr = Ask::Float(prompt, posnErr);
		if ( (err = m->ErrorPosnMaxSet(posnErr)) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
		"MVPMenu::errorHandling - not able to set maximum position error (%s)\n", 
    	        	MVPServer::ErrMsg(err));
		}
	}

	// following error delay
    if ( (err = m->ErrorPosnFlagDelayGet(&sec)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
		"MVPMenu::errorHandling - not able to get following error delay (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else {
	    sec = Ask::Float("Delay before following error flag (sec)", sec);
		if ( (err = m->ErrorPosnFlagDelaySet(sec)) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
		"MVPMenu::errorHandling - not able to set following error delay (%s)\n", 
    	    	    MVPServer::ErrMsg(err));
		}
	}

	// position error action
    if ( (err = m->ActionErrorPosnGet(&action)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
	"MVPMenu::errorHandling - not able to get following error action code (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else {
		action = Ask::Int(
	"Following error action code: disable (0), servo (1), recover error (2)", 
			action, 0, 2);
		if ( (err = m->ActionErrorPosnSet(action)) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
	"MVPMenu::errorHandling - not able to set following error action code (%s)\n", 
    	        	MVPServer::ErrMsg(err));
		} else if ( action == 0 ) {
			printf("Will disable MVP when following error reached.\n");
		} else if ( action == 1 ) {
			printf("Will attempt to servo in present position.\n");
		} else {
			printf("Will attempt to recover accumulated error.\n");
		}
	}

    Ask::Pause();
}

void MVPMenu::limitsPosn()
{
    float posnIn, posnMin, posnMax;
	int actionOutOfRange;
    char prompt[MVPMENU_PROMPT_LEN];
    MVPServer::Err err;

	// in range position
    if ( (err = m->TargetPosnInRangeGet(&posnIn)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			    "MVPMenu::limitsPosn - not able to get in range position (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else {
		snprintf(prompt, MVPMENU_PROMPT_LEN, 
		         "In Range Position (%s)\n", m->UnitsEncoder());
	    posnIn = Ask::Float(prompt, posnIn);
		if ( (err = m->TargetPosnInRangeSet(posnIn)) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
				"MVPMenu::limitsPosn - not able to set in range position (%s)\n", 
    	        	MVPServer::ErrMsg(err));
		}
	}

	// position minimum (not able to get previous value)
	snprintf(prompt, MVPMENU_PROMPT_LEN, 
		     "Position minimum - can't get previous (%s)\n", m->UnitsEncoder());
	posnMin = Ask::Float(prompt, -1000000, -0.000001, -1000000); // (UNITS) - need to change range per units
	if ( (err = m->TargetPosnLimitMinSet(posnMin)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			    "MVPMenu::limitsPosn - not able to set position minimum (%s)\n", 
    	        MVPServer::ErrMsg(err));
	}

	// position maximum
    if ( (err = m->TargetPosnLimitMaxGet(&posnMax)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			    "MVPMenu::limitsPosn - not able to get position maximum (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else {
		snprintf(prompt, MVPMENU_PROMPT_LEN, 
		         "Position maximum (%s)\n", m->UnitsEncoder());
	    posnMax = Ask::Float(prompt, posnMax, 0, 1000000); // (UNITS) - need to change range per units
		if ( (err = m->TargetPosnLimitMaxSet(posnMax)) != MVPServer::ERR_OK ) {	
    		fprintf(stderr, 
			    "MVPMenu::limitsPosn - not able to set position maximum (%s)\n", 
    	        	MVPServer::ErrMsg(err));
		}
	}

	// action for position out of range
    if ( (err = m->ActionPosnOutOfLimitGet(&actionOutOfRange)) != 
         MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			    "MVPMenu::limitsPosn - not able to get action code (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else {
	    actionOutOfRange = Ask::Int(
		"Action out of range (0=disable, 1=servo okay, 2=move to range limit)", 
			actionOutOfRange, 0, 2);
		if ( (err = m->ActionPosnOutOfLimitSet(actionOutOfRange)) != 
		     MVPServer::ERR_OK ) {
    		fprintf(stderr, 
			   		"MVPMenu::limitsPosn - not able to set action code (%s)\n", 
    	        	MVPServer::ErrMsg(err));
		}
	}

	Ask::Pause();
}

void MVPMenu::limitsCurrentPWM()
{
	float percent, amps;
	MVPServer::Err err;

	// current limit
    if ( (err = m->PowerLimitCurrentGet(&amps)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			"MVPMenu::limitsCurrentPWM - not able to get current limit (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else {
	    amps = Ask::Float("Current Limit (amps)", amps, 0, 18.19);
		if ( (err = m->PowerLimitCurrentSet(amps)) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
			"MVPMenu::limitsCurrentPWM - not able to set current limit (%s)\n", 
    	    	    MVPServer::ErrMsg(err));
		}
	}

	// pwm output maximum
    if ( (err = m->PowerLimitPWMOutputGet(&percent)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
		"MVPMenu::limitsCurrentPWM - not able to get PWM output maximum (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else {
	    percent = Ask::Float("PWM Output Maximum (percent)", percent, 0, 100);
		if ( (err = m->PowerLimitPWMOutputSet(percent)) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
		"MVPMenu::limitsCurrentPWM - not able to set PWM output maximum (%s)\n", 
    	    	    MVPServer::ErrMsg(err));
		}
	}

	Ask::Pause();
}

void MVPMenu::motion()
{
	MVPServer::Err err;
	
    if ( Ask::YesNo("Move motor", true) ) {
	    if ( (err = m->MotionMove()) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
				    "MVPMenu::motion - not able to move motor (%s)\n", 
   	 	    	    MVPServer::ErrMsg(err));
		}
	}

    Ask::Pause();
}

void MVPMenu::motionAbort()
{
	MVPServer::Err err;
	
	// abort motion
    if ( Ask::YesNo("Abort motion", true) ) {
	    if ( (err = m->MotionAbort()) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
				    "MVPMenu::motionAbort - not able to abort motion (%s)\n", 
    	        	MVPServer::ErrMsg(err));
		}
	}

    Ask::Pause();
}

void MVPMenu::motionAbortParameters()
{
    float decel;
	int action;
	MVPServer::Err err;

	// abort deceleration rate
    if ( (err = m->MotionAbortDecelRateGet(&decel)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
"MVPMenu::motionAbortParameters - not able to get abort motion deceleration rate (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else {
	    decel = Ask::Float("Abort motion deceleration rate (UNITS)", decel);
		if ( (err = m->MotionAbortDecelRateSet(decel)) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
"MVPMenu::motionAbortParameters - not able to set abort motion deceleration rate (%s)\n", 
    	        	MVPServer::ErrMsg(err));
		}
	}

	// abort action
    if ( (err = m->ActionMotionAbortGet(&action)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
"MVPMenu::motionAbortParameters - not able to get abort motion action code (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else {
		action = Ask::Int(
	"Abort motion action code: disable (0), hard stop (1), soft stop (2)", 
			action, 0, 2);
		if ( (err = m->ActionMotionAbortSet(action)) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
"MVPMenu::motionAbortParameters - not able to set abort motion action code (%s)\n", 
    	    	    MVPServer::ErrMsg(err));
		} else if ( action == 0 ) {
			printf("Will disable MVP when abort motion.\n");
		} else if ( action == 1 ) {
			printf("Will execute hard stop when abort motion.\n");
		} else {
			printf("Will execute soft stop when abort motion.\n");
			printf("Will use motion abort deceleration rate.\n");
		}
	}

    Ask::Pause();
}

void MVPMenu::homeArm()
{
	bool arm;
	MVPServer::Err err;

	// arm the home input
    if ( (err = m->HomeArmGet(&arm)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			    "MVPMenu::homeArm - not able to get arm home setting (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else {
		arm = Ask::YesNo("Arm the home input", arm);
		if ( (err = m->HomeArmSet(arm)) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
				    "MVPMenu::homeArm - not able to set arm home setting (%s)\n", 
    	        	MVPServer::ErrMsg(err));
		}
	}

    Ask::Pause();
}

void MVPMenu::homeParameters()
{
	int polarity;
	int action;
    bool onboot;
    MVPServer::Err err;

	// define the home arming polarity
    if ( (err = m->HomeArmPolarityGet(&polarity)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
		"MVPMenu::homeParameters - not able to get home arming polarity (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else {
		polarity = Ask::Int(
		    "Use pos. (1) or neg. (0) polarity to trigger the home input", 
			polarity, 0, 1);
		if ( (err = m->HomeArmPolaritySet(polarity)) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
		"MVPMenu::homeParameters - not able to set home arming polarity (%s)\n", 
    	        	MVPServer::ErrMsg(err));
		}
	}

    // homing sequence action code
    if ( (err = m->ActionHomeSequenceGet(&action)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
"MVPMenu::homeParameters - not able to get homing sequence action code (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else {
		action = Ask::Int(
	"Homing sequence action code: disable (0), hard stop (1), soft stop (2)", 
			action, 0, 2);
		if ( (err = m->ActionHomeSequenceSet(action)) != MVPServer::ERR_OK ) {
	    	fprintf(stderr, 
"MVPMenu::homeParameters - not able to set homing sequence action code (%s)\n", 
    		        MVPServer::ErrMsg(err));
		} else if ( action == 0 ) {
			printf("Will disable MVP when home is triggered.\n");
		} else if ( action == 1 ) {
			printf("Will execute hard stop and assign home when triggered.\n");
		} else {
			printf("Will execute soft stop and assign home when triggered.\n");
			printf("Will use profile deceleration rate.\n");
		}
	}

	// enable and home next time it boots
    if ( (err = m->HomeEnableOnBootGet(&onboot)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
"MVPMenu::homeParameters - not able to get home and enable boot setting (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else {
		onboot = Ask::YesNo("Enable/home next time it boots", onboot);
		if ( (err = m->HomeEnableOnBootSet(onboot)) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
"MVPMenu::homeParameters - not able to enable/home next time it boots (%s)\n", 
    	    	    MVPServer::ErrMsg(err));
		}
	}

    Ask::Pause();
}

void MVPMenu::viewInputs()
{
    float posnMVP, posnMPM, aiMVP, aiMPM;
	bool aiMVPstate; // when configured as digital input
	int mode;
	bool di[4];
    char header[] = "posnMVP posnMPM | aiMVP aiMPM | di1 di2 di3 di4\n";// FIX - UNITS
	char format[] = "%.2f %.2f | %.2f %.2f | %i %i %i %i\n";
	int i;
	MVPServer::Err err;

    printf("Type any key to stop...\n\n");

    i = 0;
    while ( Ask::CharReady() == 0 ) {
	    if ( (i % 20) == 0 ) {
		    printf("%s", header);
		}
		// actual - posn, MPM posn
		if ( (err = m->ActualPosnGet(&posnMVP)) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
			    "MVPMenu::viewInputs - not able to get MVP position (%s)\n", 
    	    	    MVPServer::ErrMsg(err));
		}
		if ( (err = m->ActualPosnMPMGet(&posnMPM)) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
			    "MVPMenu::viewInputs - not able to set MPM position (%s)\n", 
    	    	    MVPServer::ErrMsg(err));
		}
		// analog input on MVP
		if ( (err = m->AnalogInputModeGet(&mode)) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
			"MVPMenu::viewInputs - not able to get analog input mode (%s)\n", 
    	    	    MVPServer::ErrMsg(err));
		} else if ( mode == 0 ) {
			if ( (err = m->AnalogInputValueGet(&aiMVP)) != MVPServer::ERR_OK ) {
    			fprintf(stderr, 
			    "MVPMenu::viewInputs - not able to get MVP analog input (%s)\n", 
    	        		MVPServer::ErrMsg(err));
			}
		} else { // mode == 1
			if ( (err = m->AnalogInputValueGet(&aiMVPstate)) != MVPServer::ERR_OK ) {
    			fprintf(stderr, 
	"MVPMenu::viewInputs - not able to get MVP analog (digital) input (%s)\n", 
    	        		MVPServer::ErrMsg(err));
			}
		}
		// analog input on MPM
		if ( (err = m->AnalogInputValueMPMGet(&aiMPM)) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
			    "MVPMenu::viewInputs - not able to MPM analog input (%s)\n", 
    	    	    MVPServer::ErrMsg(err));
		}
		// digital inputs on MPM
		for ( i = 1 ; i < 5 ; i++ ) {
			if ( (err = m->DigitalInputMPMGet(i, &(di[i-1]))) != MVPServer::ERR_OK ) {
    			fprintf(stderr, 
		"MVPMenu::viewInputs - not able to MPM digital input channel %d (%s)\n", 
    	        		i, MVPServer::ErrMsg(err));
			}
		}
		printf(format,
		       posnMVP, posnMPM, mode ? (float) aiMVPstate : aiMVP, aiMPM, 
			   di[0], di[1], di[2], di[3]);
		i++;
	}
	Ask::CharFlush(); // flush characters entered to stop printing

    Ask::Pause();
}

void MVPMenu::gains()
{
    float prop, integr, deriv;
    MVPServer::Err err;

	// proportional gain
    if ( (err = m->GainProportionalGet(&prop)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
		    	"MVPMenu::gains - not able to get proportional gain (%s)\n", 
    	   	    MVPServer::ErrMsg(err));
	} else {
		// FIX - could add limits like this on other commands
		// scale relative to UNITS
	    prop = Ask::Float("Proportional gain (UNITS)", prop, 0, 0x7FFF);
		if ( (err = m->GainProportionalSet(prop)) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
				    "MVPMenu::gains - not able to set proportional gain (%s)\n", 
    	        	MVPServer::ErrMsg(err));
		}
	}

	// integral gain
    if ( (err = m->GainIntegralGet(&integr)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			    "MVPMenu::gains - not able to get integral gain (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else {
	    integr = Ask::Float("Integral gain (UNITS)", integr);
		if ( (err = m->GainIntegralSet(integr)) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
			    	"MVPMenu::gains - not able to set integral gain (%s)\n", 
    	    	    MVPServer::ErrMsg(err));
		}
	}

	// derivative gain
    if ( (err = m->GainDerivativeGet(&deriv)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			    "MVPMenu::gains - not able to get derivative gain (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else {
	    deriv = Ask::Float("Derivative gain (UNITS)", deriv);
		if ( (err = m->GainDerivativeSet(deriv)) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
				    "MVPMenu::gains - not able to set derivative gain (%s)\n", 
    	        	MVPServer::ErrMsg(err));
		}
	}

    Ask::Pause();
}

void MVPMenu::samplePeriod()
{
	float sec;
	MVPServer::Err err;

	// sample period
    if ( (err = m->SamplePeriodGet(&sec)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			    "MVPMenu::samplePeriod - not able to get sample period (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else {
	    sec = Ask::Float("Sample period (sec)", sec);
		if ( (err = m->SamplePeriodSet(sec)) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
			    "MVPMenu::samplePeriod - not able to set sample period (%s)\n", 
    	    	    MVPServer::ErrMsg(err));
		}
	}

    Ask::Pause();
}

void MVPMenu::analogInputMode()
{
    int mode;
    MVPServer::Err err;

    // analog input mode
    if ( (err = m->AnalogInputModeGet(&mode)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
		"MVPMenu::analogInputMode - not able to get analog input mode (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else {
		mode = Ask::Int("Analog input mode: analog (0) or digital (1)", 
			            mode, 0, 1);
		if ( (err = m->AnalogInputModeSet(mode)) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
		"MVPMenu::analogInputMode - not able to set analog input mode (%s)\n", 
    	    	    MVPServer::ErrMsg(err));
		}
	}

    Ask::Pause();
}

void MVPMenu::analogInputs()
{
    int mode;
    float value;
    MVPServer::Err err;

    // analog input on MVP
    if ( (err = m->AnalogInputModeGet(&mode)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			"MVPMenu::analogInputs - not able to get analog input mode (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else if ( mode == 0 ) {
		if ( (err = m->AnalogInputValueGet(&value)) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
			"MVPMenu::analogInputs - not able to get MVP analog input (%s)\n", 
    	    	    MVPServer::ErrMsg(err));
		} else {
			printf("Analog input (MVP) = %f (%s)\n", value, 
			       m->UnitsAnalogInput());
		}
	} else { // mode == 1
	    printf("Analog input (MVP) configured as digital input\n");
	}

    // analog input on MPM
	if ( (err = m->AnalogInputValueMPMGet(&value)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			    "MVPMenu::analogInputs - not able to get MPM analog input (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else {
	    printf("Analog input (MPM) = %f (%s)\n", value, 
	           m->UnitsAnalogInputMPM());
	}

    Ask::Pause();
}

void MVPMenu::analogOutput()
{
    float value = 0.0;
    char prompt[MVPMENU_PROMPT_LEN];
    MVPServer::Err err;

	// analog output (if fixed current limit)
	if ( Ask::YesNo("Set analog output (need CURRENT LIMIT = FIXED)", false) ) {
		snprintf(prompt, MVPMENU_PROMPT_LEN, 
		         "Analog output (%s)\n", m->UnitsAnalogOutput());
		value = Ask::Float(prompt, value);
		if ( (err = m->AnalogOutputSet(value)) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
			    "MVPMenu::analogOutput - not able to set analog output (%s)\n", 
    	    	    MVPServer::ErrMsg(err));
		}
	}

    Ask::Pause();
}

void MVPMenu::digitalInputs()
{
    int mode;
    bool state;
    int stateInt;
	int i;
	MVPServer::Err err;

    // analog input on MVP (when configured as digital input)
    if ( (err = m->AnalogInputModeGet(&mode)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			"MVPMenu::digitalInputs - not able to get analog input mode (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else if ( mode == 1 ) {
		if ( (err = m->AnalogInputValueGet(&state)) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
			"MVPMenu::digitalInputs - not able to get MVP analod input (%s)\n", 
    	    	    MVPServer::ErrMsg(err));
		} else {
			//printf("Digital input (MVP) = %i (units?)\n", state ? 1 : 0);
			printf("Digital input (MVP) = %i\n", state);
		}
	}

	// digital inputs on MPM
	for ( i = 1 ; i < 5 ; i++ ) {
		if ( (err = m->DigitalInputMPMGet(i, &state)) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
"MVPMenu::digitalInputs - not able to get MPM digital input channel %d (%s)\n", 
	    	        i, MVPServer::ErrMsg(err));
		} else {
			printf("Digital input (MPM, Ch %i) = %i\n", i, state ? 1 : 0);
		}
	}
	
	// all digital inputs on MPM
	if ( (err = m->DigitalInputAllMPMGet(&stateInt)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			    "MVPMenu::digitalInputs - not able to get all digital inputs (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else {
		printf("Digital input (MPM, all)  = %d\n", stateInt);
	}

    Ask::Pause();
}

void MVPMenu::digitalOutputs()
{
    bool state;
	int i;
	char prompt[82];
	MVPServer::Err err;

    // digital output on MVP
	if ( Ask::YesNo("Set digital output (MVP, pin J2-1)", false) ) {
		state = (bool) Ask::Int("Digital output: high (1) or low (0) ",
		                        0, 0, 1);
		if ( (err = m->DigitalOutputSet(state)) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
			"MVPMenu::digitalOutputs - not able to set digital output (%s)\n", 
    	    	    MVPServer::ErrMsg(err));
		}
	}

	// digital outputs on MPM
	for ( i = 0 ; i < 4 ; i++ ) {
	    sprintf(prompt, "Set digital output (MPM, Ch %i)", i);
		if ( Ask::YesNo(prompt, false) ) {
			state = (bool) Ask::Int("Digital output: high (1) or low (0) ",
									0, 0, 1);
			if ( (err = m->DigitalOutputMPMSet(i, state)) != MVPServer::ERR_OK ) {
    			fprintf(stderr, 
			"MVPMenu::digitalOutputs - not able to set digital output (%s)\n", 
    	    		    MVPServer::ErrMsg(err));
			}
		}
	}

    Ask::Pause();
}

void MVPMenu::macroStatusExecute()
{
    int macro;
    MVPServer::Err err;

	// macro status
	if ( (err = m->MacroStatusGet(&macro)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			    "MVPMenu::macroStatusExecute - not able to get macro status (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else if ( macro == 0 ) {
	    printf("No macro is executing.\n");
	} else {
		printf("Executing macro number %i.\n", macro);
	}

    // execute macro
	if ( Ask::YesNo("Execute macro", false) ) {
		macro = Ask::Int("Macro number to execute", 1, 0, 1000);
		if ( (err = m->MacroExecute(macro)) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
			"MVPMenu::macroStatusExecute - not able to execute macro (%s)\n", 
    	    	    MVPServer::ErrMsg(err));
		}
	}

	Ask::Pause();
}

void MVPMenu::communicationParameters()
{
	float sec;
	bool ok;
	MVPServer::Err err;

	// serial response delay
    if ( (err = m->SerialResponseDelayGet(&sec)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
"MVPMenu::communicationParameters - not able to get serial response delay (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else {
	    sec = Ask::Float("Serial response delay (sec)", sec);
		if ( (err = m->SerialResponseDelaySet(sec)) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
"MVPMenu::communicationParameters - not able to set serial response delay (%s)\n", 
    	    	    MVPServer::ErrMsg(err));
		}
	}

    // serial OK response
    if ( (err = m->SerialResponseOKGet(&ok)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
"MVPMenu::communicationParameters - not able to get serial OK response (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else {
		ok = Ask::YesNo("Respond with 'ok' upon successful receipt", ok);
		if ( (err = m->SerialResponseOKSet(ok)) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
"MVPMenu::communicationParameters - not able to set serial OK response (%s)\n", 
	    	        MVPServer::ErrMsg(err));
		}
	}

    Ask::Pause();
}

void MVPMenu::eeprom()
{
    bool boot;
    MVPServer::Err err;

	// save all parameters to EEPROM
	if ( Ask::YesNo("Save all parameters to EEPROM", false) ) {
		if ( (err = m->EEpromSave()) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
		"MVPMenu::eeprom - not able to save all parameters to EEPROM (%s)\n", 
    	    	    MVPServer::ErrMsg(err));
		} else {
			printf("Saved all parameters to EEPROM.\n");
			printf("\n");
			printf("To manually disable configurations to EEPROM:\n");
			printf("  - turn all of the dip switches in the front of\n");
			printf("    the MVP to the OFF position\n");
			printf("  - power up the MVP\n");
			printf("  - disconnect power\n");
			printf("  - reset the dip switches to their original config\n");
			printf("  - reboot the MVP\n");
			printf("All the settings will return to factory defaults.\n");
			printf("\n");
		}
	}

	// use configuration from EEPROM on boot
    if ( (err = m->EEpromBootGet(&boot)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
		"MVPMenu::eeprom - not able to get EEPROM on boot configuration (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else {
		boot = Ask::YesNo("Use configuration from EEPROM on boot", boot);
		if ( (err = m->EEpromBootSet(boot)) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
		"MVPMenu::eeprom - not able to set EEPROM on boot configuration (%s)\n", 
    	        	MVPServer::ErrMsg(err));
		}
	}

    Ask::Pause();
}

void MVPMenu::statusBits()
{
    bool state;
	char ver[82];
	MVPServer::Err err;

    // move in progress
	if ( (err = m->StatusMoveInProgress(&state)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
		"MVPMenu::statusBits - not able to get move in progress status (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else if ( state == 0 ) {
	    printf(" 0: Not commanded to move.\n");
	} else {
		printf(" 0: Move in progress.\n");
	}

    // motor in position
	if ( (err = m->StatusMotorInPosn(&state)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
		"MVPMenu::statusBits - not able to get motor in position status (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else if ( state == 0 ) {
	    printf(" 1: Motor is not in position.\n");
	} else {
		printf(" 1: Motor is in position.\n");
	}

    // velocity mode
	if ( (err = m->StatusVelocityMode(&state)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			"MVPMenu::statusBits - not able to get velocity mode status (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else if ( state == 0 ) {
	    printf(" 2: MVP is in Position mode.\n");
	} else {
		printf(" 2: MVP is in Velocity mode.\n");
	}

    // analog input
	if ( (err = m->StatusAnalogInput(&state)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			"MVPMenu::statusBits - not able to get analog input status (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else if ( state == 0 ) {
	    printf(" 3: Analog input logic state = 0.\n");
	} else {
		printf(" 3: Analog input logic state = 1.\n");
	}

    // trajectory percent complete
	if ( (err = m->StatusTrajPercComplete(&state)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
"MVPMenu::statusBits - not able to get trajectory percent complete status (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else if ( state == 0 ) {
	    printf(" 4: Trajectory percentage is not complete.\n");
	} else {
		printf(" 4: Trajectory percentage is complete.\n");
	}

    // device net
	if ( (err = m->StatusDeviceNet(&state)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			    "MVPMenu::statusBits - not able to device net status (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else if ( state == 0 ) {
	    printf(" 5: DeviceNet connection is not active.\n");
	} else {
		printf(" 5: DeviceNet connection is active.\n");
	}

    // device net error
	if ( (err = m->StatusDeviceNetError(&state)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			    "MVPMenu::statusBits - not able to get device net error status (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else if ( state == 0 ) {
	    printf(" 6: DeviceNet message packets are ok.\n");
	} else {
		printf(" 6: DeviceNet message error in one or more packets.\n");
	}

    // move error
	if ( (err = m->StatusMoveError(&state)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			    "MVPMenu::statusBits - not able to get move error status (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else if ( state == 0 ) {
	    printf(" 7: Current move is going ok.\n");
	} else {
		printf(" 7: Current move is off trajectory more than allowed amount.\n");
	}

    // motor disabled
	if ( (err = m->StatusMotorDisabled(&state)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
		"MVPMenu::statusBits - not able to get motor disabled status (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else if ( state == 0 ) {
	    printf(" 8: Motor is enabled.\n");
	} else {
		printf(" 8: Motor is not enabled or has been disabled by error.\n");
	}

    // range limit
	if ( (err = m->StatusRangeLimit(&state)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			"MVPMenu::statusBits - not able to get range limit status (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else if ( state == 0 ) {
	    printf(" 9: Move is within range limits.\n");
	} else {
		printf(" 9: Reached the program range limits.\n");
	}

    // local mode
	if ( (err = m->StatusLocalMode(&state)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			"MVPMenu::statusBits - not able to get local mode status (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else if ( state == 0 ) {
	    printf("10: Remote mode is active.\n");
	} else {
		printf("10: Local mode is active.\n");
	}

    // emergency stop
	if ( (err = m->StatusEmergencyStop(&state)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
		"MVPMenu::statusBits - not able to get emergency stop status (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else if ( state == 0 ) {
	    printf("11: Emergency stop is not active.\n");
	} else {
		printf("11: Emergency stop is active.\n");
	}

    // external event 1
	if ( (err = m->StatusExternalEvent1(&state)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
		"MVPMenu::statusBits - not able to get external event 1 statu (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else if ( state == 0 ) {
	    printf("12: External event #1 is not active.\n");
	} else {
	    printf("12: External event #1 is active.\n");
	}

    // positive limit flag
	if ( (err = m->StatusPositiveLimitFlag(&state)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
	"MVPMenu::statusBits - not able to get positive limit flag status (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else if ( state == 0 ) {
	    printf("13: Positive limit flag is not active.\n");
	} else {
	    printf("13: Positive limit flag is active.\n");
	}

    // external event 2
	if ( (err = m->StatusExternalEvent2(&state)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
		"MVPMenu::statusBits - not able to get external event 2 status (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else if ( state == 0 ) {
	    printf("14: External event #2 is not active.\n");
	} else {
	    printf("14: External event #2 is active.\n");
	}

    // negative limit flag
	if ( (err = m->StatusNegativeLimitFlag(&state)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
	"MVPMenu::statusBits - not able to get negative limit flag status (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else if ( state == 0 ) {
	    printf("15: Negative limit flag is not active.\n");
	} else {
	    printf("15: Negative limit flag is active.\n");
	}

    // firmware version
	if ( (err = m->StatusFirmwareVersionGet(ver)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			    "MVPMenu::statusBits - not able to get firmware version (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else {
	    //printf("Firmware version: %s\n", ver);
	    printf("Firmware version: (not implemented yet)\n");
	}

    Ask::Pause();
}

void MVPMenu::statusAll()
{
    float posn, posnErr, vel, posnMPM;
    float posnAbs, posnRel, accel, decel;
    float posnMin, posnMax;
    float sec;
	float percent, amps;
    float prop, integr, deriv;
    float value;
	int polarity;
	int action;
	char actionStr[82];
    int mode;
	int i;
	int macro;
    bool onboot;
	bool state;
	bool ok;
	MVPServer::Err err;

    // actual - posn, err, vel, ext posn
    if ( (err = m->ActualPosnGet(&posn)) != MVPServer::ERR_OK ) {
       	fprintf(stderr, 
			    "MVPMenu::statusAll - not able to get position (%s)\n", 
    	        MVPServer::ErrMsg(err));
	}
    if ( (err = m->ActualPosnErrorGet(&posnErr)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			    "MVPMenu::statusAll - not able to get position error (%s)\n", 
    	        MVPServer::ErrMsg(err));
	}
    if ( (err = m->ActualVelGet(&vel)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			    "MVPMenu::statusAll - not able to get velocity (%s)\n", 
    	        MVPServer::ErrMsg(err));
	}
    if ( (err = m->ActualPosnMPMGet(&posnMPM)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			    "MVPMenu::statusAll - not able to get MPM position (%s)\n", 
    	        MVPServer::ErrMsg(err));
	}
	printf("actual (posn, err, vel, posnMPM) =\n");
	printf("%.2f UNITS, %.2f UNITS, %.2f UNITS, %.2f UNITS\n", 
	       posn, posnErr, vel, posnMPM);

    // target - abs posn, rel posn, max vel, accel, decel
	// (no way to tell if mext move absolute or relative?
	if ( (err = m->TargetPosnAbsGet(&posnAbs)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
		"MVPMenu::statusAll - not able to get target absolute position (%s)\n", 
    	        MVPServer::ErrMsg(err));
	}
	if ( (err = m->TargetPosnRelGet(&posnRel)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
		"MVPMenu::statusAll - not able to get target relative position (%s)\n", 
    	        MVPServer::ErrMsg(err));
	}
    if ( (err = m->TargetVelMaxGet(&vel)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
		"MVPMenu::statusAll - not able to get target maximum velocity (%s)\n", 
    	        MVPServer::ErrMsg(err));
	}
    if ( (err = m->TargetAccelRateGet(&accel)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
		"MVPMenu::statusAll - not able to get target acceleration rate (%s)\n", 
    	        MVPServer::ErrMsg(err));
	}
    if ( (err = m->TargetDecelRateGet(&decel)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
		"MVPMenu::statusAll - not able to get target deceleration rate (%s)\n", 
    	        MVPServer::ErrMsg(err));
	}
	printf("target (abs posn, rel posn, max vel, accel, decel) = \n");
	printf("%.2f UNITS, %.2f UNITS, %.2f UNITS, %.2f UNITS, %.2f UNITS\n", 
	       posnAbs, posnRel, vel, accel, decel);

	// error handling - max posn, delay, action
    if ( (err = m->ErrorPosnMaxGet(&posnErr)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
		"MVPMenu::statusAll - not able to get maximum position error (%s)\n", 
    	        MVPServer::ErrMsg(err));
	}
    if ( (err = m->ErrorPosnFlagDelayGet(&sec)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
		   "MVPMenu::statusAll - not able to get following error delay (%s)\n", 
    	        MVPServer::ErrMsg(err));
	}
    if ( (err = m->ActionErrorPosnGet(&action)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
	"MVPMenu::statusAll - not able to get following error action code (%s)\n", 
    	        MVPServer::ErrMsg(err));
	}
    switch (action) {
	    case 0:
		    sprintf(actionStr, "0=disable");
		    break;
	    case 1:
		    sprintf(actionStr, "1=servo");
		    break;
		case 2:
		    sprintf(actionStr, "2=recover");
			break;
		default:
		    break;
	}
	printf("error handling (max posn, delay, act) = ");
	printf("%.2f UNITS, %.6f sec, %s\n", posnErr, sec, actionStr);

	// position limits
    if ( (err = m->TargetPosnLimitMinGet(&posnMin)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			    "MVPMenu::statusAll - not able to get position maximum (%s)\n", 
    	        MVPServer::ErrMsg(err));
	}
    if ( (err = m->TargetPosnLimitMaxGet(&posnMax)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			    "MVPMenu::statusAll - not able to get position maximum (%s)\n", 
    	        MVPServer::ErrMsg(err));
	}
    if ( (err = m->ActionPosnOutOfLimitGet(&action)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			    "MVPMenu::statusAll - not able to get action code (%s)\n", 
    	        MVPServer::ErrMsg(err));
	}
    switch (action) {
	    case 0:
		    sprintf(actionStr, "0=disable");
		    break;
	    case 1:
		    sprintf(actionStr, "1=servo");
		    break;
		case 2:
		    sprintf(actionStr, "2=limit");
			break;
		default:
		    break;
	}
	printf("position limits (min posn, max posn) = %.2f UNITS, %.2f UNITS\n", 
	       posnMin, posnMax);

	// current/PWM limits
    if ( (err = m->PowerLimitCurrentGet(&amps)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			    "MVPMenu::statusAll - not able to get current limit (%s)\n", 
    	        MVPServer::ErrMsg(err));
	}
    if ( (err = m->PowerLimitPWMOutputGet(&percent)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			"MVPMenu::statusAll - not able to get PWM output maximum (%s)\n", 
    	        MVPServer::ErrMsg(err));
	}
	printf("current/PWM limits = %.2f amps, %.2f percent\n", amps, percent);

    // abort motion - decel, action
    if ( (err = m->MotionAbortDecelRateGet(&decel)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
"MVPMenu::statusAll - not able to get abort motion deceleration rate (%s)\n", 
    	        MVPServer::ErrMsg(err));
	}
    if ( (err = m->ActionMotionAbortGet(&action)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
		"MVPMenu::statusAll - not able to get abort motion action code (%s)\n", 
    	        MVPServer::ErrMsg(err));
	}
    switch (action) {
	    case 0:
		    sprintf(actionStr, "0=disable");
		    break;
	    case 1:
		    sprintf(actionStr, "1=hard stop");
		    break;
		case 2:
		    sprintf(actionStr, "2=soft stop");
			break;
		default:
		    break;
	}
	printf("abort motion (decel, act) = ");
	printf("%.2f UNITS, %s\n",decel, actionStr);

    // home parameters - polarity, action, onboot
    if ( (err = m->HomeArmPolarityGet(&polarity)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			"MVPMenu::statusAll - not able to get home arming polarity (%s)\n", 
    	        MVPServer::ErrMsg(err));
	}
    if ( (err = m->ActionHomeSequenceGet(&action)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
	"MVPMenu::statusAll - not able to get homing sequence action code (%s)\n", 
    	        MVPServer::ErrMsg(err));
	}
    if ( (err = m->HomeEnableOnBootGet(&onboot)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
	"MVPMenu::statusAll - not able to get home and enable boot setting (%s)\n", 
    	        MVPServer::ErrMsg(err));
	}
    switch (action) {
	    case 0:
		    sprintf(actionStr, "0=disable");
		    break;
	    case 1:
		    sprintf(actionStr, "1=hard stop");
		    break;
		case 2:
		    sprintf(actionStr, "2=soft stop");
			break;
		default:
		    break;
	}
	printf("home parameters (polarity, act, h/e onboot) = %i, %s, %i\n",
	       polarity, actionStr, onboot);

	// pid gains
    if ( (err = m->GainProportionalGet(&prop)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			    "MVPMenu::statusAll - not able to get proportional gain (%s)\n", 
    	        MVPServer::ErrMsg(err));
	}
    if ( (err = m->GainIntegralGet(&integr)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			    "MVPMenu::statusAll - not able to get integral gain (%s)\n", 
    	        MVPServer::ErrMsg(err));
	}
    if ( (err = m->GainDerivativeGet(&deriv)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			    "MVPMenu::statusAll - not able to get derivative gain (%s)\n", 
    	        MVPServer::ErrMsg(err));
	}
	printf("pid gains = %f UNITS, %f UNITS, %f UNITS\n", prop, integr, deriv);

	// sample period
    if ( (err = m->SamplePeriodGet(&sec)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			    "MVPMenu::statusAll - not able to get sample period (%s)\n", 
    	        MVPServer::ErrMsg(err));
	}
	printf("sample period = %f sec\n", sec);

	// analog input mode
    if ( (err = m->AnalogInputModeGet(&mode)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			    "MVPMenu::statusAll - not able to get analog input mode (%s)\n", 
    	        MVPServer::ErrMsg(err));
	}
	printf("analog input mode = %s\n", mode ? "1=digital" : "0=analog");

	// analog inputs
	printf("analog inputs = MVP ");
    if ( (err = m->AnalogInputModeGet(&mode)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			    "MVPMenu::statusAll - not able to get analog input mode (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else if ( mode == 0 ) {
		if ( (err = m->AnalogInputValueGet(&value)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			    "MVPMenu::statusAll - not able to get MVP analog input (%s)\n", 
    	        MVPServer::ErrMsg(err));
		} else {
			printf("%f UNITS, ", value);
		}
	} else { // mode == 1
	    printf("digital, ");
	}
	if ( (err = m->AnalogInputValueMPMGet(&value)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			    "MVPMenu::statusAll - not able to get MPM analog input (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else {
	    printf("MPM %f UNITS\n", value);
	}

	// digital inputs
	printf("digital inputs = ");
    if ( (err = m->AnalogInputModeGet(&mode)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			    "MVPMenu::statusAll - not able to get analog input mode (%s)\n", 
    	        MVPServer::ErrMsg(err));
	} else if ( mode == 1 ) {
		if ( (err = m->AnalogInputValueGet(&state)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			    "MVPMenu::statusAll - not able to get MVP analog input (%s)\n", 
    	        MVPServer::ErrMsg(err));
		} else {
			printf("MVP %i UNITS,", state ? 1 : 0);
		}
	}
	printf("MPM ");
	for ( i = 1 ; i < 5 ; i++ ) {
		if ( (err = m->DigitalInputMPMGet(i, &state)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
	"MVPMenu::statusAll - not able to get MPM digital input channel %d (%s)\n", 
    	        i, MVPServer::ErrMsg(err));
		} else {
			printf("%i, ", state ? 1 : 0);
		}
	}
	printf("\n");

	// macro status
	if ( (err = m->MacroStatusGet(&macro)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			    "MVPMenu::statusAll - not able to get macro status (%s)\n", 
    	        MVPServer::ErrMsg(err));
	}
	printf("macro executing = %i\n", macro);

	// communication parameters
    if ( (err = m->SerialResponseDelayGet(&sec)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
		    "MVPMenu::statusAll - not able to get serial response delay (%s)\n", 
    	        MVPServer::ErrMsg(err));
	}
    if ( (err = m->SerialResponseOKGet(&ok)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
		    "MVPMenu::statusAll - not able to get serial OK response (%s)\n", 
    	        MVPServer::ErrMsg(err));
	}
	if ( (err = m->SerialChecksumGet(&state)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			    "MVPMenu::statusAll - not able to get checksum state (%s)\n", 
    	        MVPServer::ErrMsg(err));
	}
	printf("communication parameters (delay, ok, checksum) = %f sec, %s, %d\n",
	       sec, ok ? "1=ok" : "0=no ok", int(state));


	// EEprom
    if ( (err = m->EEpromBootGet(&onboot)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
	"MVPMenu::statusAll - not able to get EEPROM on boot configuration (%s)\n", 
    	        MVPServer::ErrMsg(err));
	}
	printf("eeprom config used on boot = %s\n", onboot ? "1=yes" : "0=no");

    Ask::Pause();
}

void MVPMenu::convFactors()
{
	char units[MVP_UNITS_LEN];
	float slope;
	float offset;
	char prompt[MVPMENU_PROMPT_LEN];
	
	// encoder
	m->ConvEncoderGet(units, &slope, &offset);
	Ask::String("Encoder conversion units", units, units, MVP_UNITS_LEN);
	snprintf(prompt, MVPMENU_PROMPT_LEN,
	         "Encoder conversion slope (quad counts/%s)", units); 
	slope = Ask::Float(prompt, slope);
	offset = Ask::Float("Encoder conversion offset (quad counts)", offset);
	printf("\n");
	m->ConvEncoderSet(units, slope, offset);
	
	// encoderMPM
	m->ConvEncoderMPMGet(units, &slope, &offset);
	Ask::String("Encoder MPM conversion units", units, units, MVP_UNITS_LEN);
	snprintf(prompt, MVPMENU_PROMPT_LEN,
	         "Encoder MPM conversion slope (quad counts/%s)", units);
	slope = Ask::Float(prompt, slope);
	offset = Ask::Float("Encoder MPM conversion offset (quad counts)", offset);
	printf("\n");
	m->ConvEncoderMPMSet(units, slope, offset);
		
	// analogInput
	m->ConvAnalogInputGet(units, &slope, &offset);
	Ask::String("Analog Input conversion units", units, units, MVP_UNITS_LEN);
	snprintf(prompt, MVPMENU_PROMPT_LEN,
	         "Analog Input conversion slope (bits/%s)", units);
	slope = Ask::Float(prompt, slope);
	offset = Ask::Float("Analog Input conversion offset (bits)", offset);
	printf("\n");
	m->ConvAnalogInputSet(units, slope, offset);
		
	// analogInputMPM
	m->ConvAnalogInputMPMGet(units, &slope, &offset);
	Ask::String("Analog Input MPM conversion units", units, units, MVP_UNITS_LEN);
	snprintf(prompt, MVPMENU_PROMPT_LEN,
	         "Analog Input MPM conversion slope (bits/%s)", units);
	slope = Ask::Float(prompt, slope);
	offset = Ask::Float("Analog Input MPM conversion offset (bits)", offset);
	printf("\n");
	m->ConvAnalogInputMPMSet(units, slope, offset);
	
	// analogOutput
	m->ConvAnalogOutputGet(units, &slope, &offset);
	Ask::String("Analog Output conversion units", units, units, MVP_UNITS_LEN);
	snprintf(prompt, MVPMENU_PROMPT_LEN,
	         "Analog Output conversion slope (bits/%s)", units);
	slope = Ask::Float(prompt, slope);
	offset = Ask::Float("Analog Output conversion offset (bits)", offset);
	printf("\n");
	m->ConvAnalogOutputSet(units, slope, offset);
	
    Ask::Pause();
}

void MVPMenu::resetRecover()
{
	MVPServer::Err err;
	
	printf("Use this command to reset a node gracefully, or to recover from\n");
	printf("a forced node reset.\n\n");
	
	// reset node
    if ( Ask::YesNo("Reset node", false) ) {
		if ( (err = m->NodeReset()) != MVPServer::ERR_OK ) {
    		fprintf(stderr, 
				    "MVPMenu::resetRecover - not able to reset node (%s)\n", 
    	        	MVPServer::ErrMsg(err));
		}
	}
	
	// wait until node reset
	Ask::Pause("Type 'Enter' when node reset fully (single blinking top green light)");
	// need to flush serial buffer here, could add flush command to MVPServer?
	// will handle okay for now, will need to try a second time
	
	// set up checksum
	if ( (err = m->SerialChecksumSet(true)) != MVPServer::ERR_OK ) {
    	fprintf(stderr, 
			    "MVPMenu::resetRecover - not able to turn on checksum (%s)\n", 
    	        MVPServer::ErrMsg(err));
	}

    Ask::Pause();
}