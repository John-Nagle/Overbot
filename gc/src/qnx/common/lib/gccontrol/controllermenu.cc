/////////////////////////////////////////////////////////////////////////////
//
//    File: controllermenu.cc
//
//    Usage:
//        see controllermenu.h
//
//    Description:
//        see controllermenu.h
//
//        Based on mvpmenu.h code of August 2003.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        February, 2003
//
/////////////////////////////////////////////////////////////////////////////

#include "controllermenu.h"
//
//	Constants
//
const double k_connectwait = 0.5;							// wait no more than this for a network connection
//
//    ControllerMenu::Display - display options of controller menu
//
//    Returns nothing
//
void ControllerMenu::Display()
{
    printf("  x - motor off                e - error action\n");
    printf("  d - define positions         c - error command/motor stop\n");
	printf("  l - position limits          i - analog inputs\n"); 
    printf("  a - actual values            I - digital & switch inputs\n");
    printf("  f - feedback loop type       O - digital outputs\n");    
    printf("  g - PID gains                W - view inputs continuously\n");
    printf("  L - integ/torq limits/off    P - program download/list\n");
    printf("  s - sample period            E - program execute/halt\n");
    printf("  t - target motion            S - status bits\n");
    printf("  p - profile motion           R - burn parameters/program\n");
    printf("  j - jog motion               r - reset controller\n");
    printf("  w - limit sw config/find     v - verbose\n");
    printf("  h - servo here               C - conversion factors\n");
    printf("  b - stop/abort motion        V - variable get/set \n");
}
//
//	Reconnect  -- try to reconnect to device if necessary
//
bool ControllerMenu::Reconnect()
{	if (c->Connected()) return(true);							// already connected, no action
	printf("Connecting to \"%s\"...\n",c->Hostname());	fflush(stdout);
	int stat = c->Connect(k_connectwait);					// try to connect
	if (stat)																	// if not successful
	{	perror("ControllerMenu::Reconnect: unable to connect to controller");
	}
	return(c->Connected());										// return true if connected
}
//
//    ControllerMenu::Process - process options for controller menu
//
//    Returns true if menu option processed, false if not
//
bool ControllerMenu::Process(char c)
{
	Reconnect();															// try to reconnect to device if needed
    switch (c) {
	    case 'x':
		    motorOff();
		    break;
	    case 'd':
		    definePositions();
		    break;
	    case 'l':
		    positionLimits();
		    break;
	    case 'a':
		    actualValues();
		    break;
	    case 'f':
		    feedbackLoopType();
		    break;
	    case 'g':
		    pidGains();
		    break;
	    case 'L':
		    integratorTorqueInfo();
		    break;
	    case 's':
		    samplePeriod();
		    break;
	    case 't':
		    targetMotion();
		    break;
	    case 'p':
		    profileMotion();
		    break;
	    case 'j':
		    jogMotion();
		    break;
	    case 'w':
		    switchConfigFind();
		    break;
	    case 'h':
		    servoHere();
		    break;
	    case 'b':
		    stopAbortMotion();
		    break;
	    case 'e':
		    errorAction();
		    break;
	    case 'c':
		    errorCommandMotor();
		    break;
	    case 'i':
		    analogInputs();
		    break;
	    case 'I':
		    digitalInputs();
		    break;
	    case 'O':
		    digitalOutputs();
		    break;
	    case 'W':
		    viewInputs();
		    break;
	    case 'P':
		    programLoadList();
		    break;
	    case 'E':
		    programExecuteHalt();
		    break;
	    case 'S':
		    statusBits();
		    break;
	    case 'R':
		    burnParametersProgram();
		    break;
	    case 'r':
		    resetController();
		    break;
	    case 'v':
		    verbose();
		    break;
	    case 'C':
		    convFactors();
		    break;
		case 'V':
			variableGetSet();
			break;
		default:
		    return false;
			break;
	}

	return true;
}

//
//    ControllerMenu::motorOff - motor off menu option
//
//    Unconditionally turns motor off. No prompt first; useful for emergency stop.
//
//    Returns nothing
//
void ControllerMenu::motorOff()
{
    Controller::Err err;
    //	Unconditionally turn motor off, right now.
    if ( (err = c->MotorOff()) != Controller::ERR_OK ) {
    		fprintf(stderr, 
    		    "ControllerMenu::motorOff - not able to turn motor off (%s).\n", 
    		        Controller::ErrMsg(err));
	} else {
		printf("Motor turned off.\n");
    }
 	Ask::Pause();
}

void ControllerMenu::definePositions()
{
	float posn;
    char prompt[CONTROLLERMENU_PROMPT_LEN];
	Controller::Err err;

    // define encoder positions
    if ( Ask::YesNo("Define encoder positions", true) ) {
    	// main encoder
    	if ( (err = c->ActualPosnGet(&posn)) != Controller::ERR_OK ) {
    		fprintf(stderr, 
    		"ControllerMenu::definePositions - not able to get main encoder position (%s)\n", 
    		        Controller::ErrMsg(err));
	    	posn = 0.0;
		}
		snprintf(prompt, CONTROLLERMENU_PROMPT_LEN, "Main encoder position (%s)",
		         c->UnitsEncoder());
		posn = Ask::Float(prompt, posn);
	    if ( (err = c->DefinePosnSet(posn)) != Controller::ERR_OK ) {
    		fprintf(stderr, 
    		"ControllerMenu::definePositions - not able to set main encoder position (%s)\n", 
    		        Controller::ErrMsg(err));
		}
		
		// auxillary encoder
    	if ( (err = c->ActualPosnAuxGet(&posn)) != Controller::ERR_OK ) {
    		fprintf(stderr, 
    		"ControllerMenu::definePositions - not able to get aux. encoder position (%s)\n", 
    		        Controller::ErrMsg(err));
	    	posn = 0.0;
		}
		snprintf(prompt, CONTROLLERMENU_PROMPT_LEN, "Aux encoder position (%s)",
		         c->UnitsEncoderAux());
		posn = Ask::Float(prompt, posn);
	    if ( (err = c->DefinePosnAuxSet(posn)) != Controller::ERR_OK ) {
    		fprintf(stderr, 
    		"ControllerMenu::definePositions - not able to set aux. encoder position (%s)\n", 
    		        Controller::ErrMsg(err));
		}
	}

    Ask::Pause();
}

void ControllerMenu::positionLimits()
{
    float posn;
    char prompt[CONTROLLERMENU_PROMPT_LEN];
    Controller::Err err;
 	
	// position minimum
    if ( (err = c->LimitPosnMinGet(&posn)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
		"ControllerMenu::positionLimits - not able to get position min (%s)\n", 
    	        Controller::ErrMsg(err));
	} else {
		snprintf(prompt, CONTROLLERMENU_PROMPT_LEN, 
		         "Position minimum (%s)", c->UnitsEncoder());
	    posn = Ask::Float(prompt, posn, -2147483648.0, 2147483647.0);
	    // (UNITS) - need to change range per units
		if ( (err = c->LimitPosnMinSet(posn)) != Controller::ERR_OK ) {
    		fprintf(stderr, 
		"ControllerMenu::positionLimits - not able to set position min (%s)\n", 
    	        	Controller::ErrMsg(err));
		}
	}

	// position maximum
    if ( (err = c->LimitPosnMaxGet(&posn)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
		"ControllerMenu::positionLimits - not able to get position max (%s)\n", 
    	        Controller::ErrMsg(err));
	} else {
		snprintf(prompt, CONTROLLERMENU_PROMPT_LEN, 
		         "Position maximum (%s)", c->UnitsEncoder());
	    posn = Ask::Double(prompt, posn, -2147483648.0, 2147483647.0);
	    // (UNITS) - need to change range per units
		if ( (err = c->LimitPosnMaxSet(posn)) != Controller::ERR_OK ) {
    		fprintf(stderr, 
		"ControllerMenu::positionLimits - not able to set position max (%s)\n", 
    	        	Controller::ErrMsg(err));
		}
	}

	// position error
    if ( (err = c->LimitPosnErrGet(&posn)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
		"ControllerMenu::positionLimits - not able to get position err (%s)\n", 
    	        Controller::ErrMsg(err));
	} else {
		snprintf(prompt, CONTROLLERMENU_PROMPT_LEN, 
		         "Position error allowed (%s)", c->UnitsEncoder());
	    posn = Ask::Float(prompt, posn, 0, 1000000);
	    // (UNITS) - need to change range per units
		if ( (err = c->LimitPosnErrSet(posn)) != Controller::ERR_OK ) {
    		fprintf(stderr, 
		"ControllerMenu::positionLimits - not able to set position err (%s)\n", 
    	        	Controller::ErrMsg(err));
		}
	}

	Ask::Pause();
}

void ControllerMenu::actualValues()
{
    float value;
    Controller::Err err;

    // actual position
    if ( (err = c->ActualPosnGet(&value)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
    	    "ControllerMenu::actualValues - not able to get position (%s)\n", 
    	        Controller::ErrMsg(err));
	} else {
		printf("Actual position (%s): %f\n", c->UnitsEncoder(), value);
	}

    // reference position
    if ( (err = c->ActualPosnRefGet(&value)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
	"ControllerMenu::actualValues - not able to get position reference (%s)\n", 
    	        Controller::ErrMsg(err));
	} else {
		printf("Actual position reference (%s): %f\n", c->UnitsEncoder(), value);
	}

    // actual position error
    if ( (err = c->ActualPosnErrorGet(&value)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
    	        "ControllerMenu::actualValues - not able to get position error (%s)\n", 
    	        Controller::ErrMsg(err));
	} else {
		printf("Actual position error (%s): %f\n", c->UnitsEncoder(), value);
	}
	
    // actual auxillary position
    if ( (err = c->ActualPosnAuxGet(&value)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
    "ControllerMenu::actualValues - not able to get position auxillary (%s)\n", 
    	        Controller::ErrMsg(err));
	} else {
		printf("Actual position auxillary (%s): %f\n", c->UnitsEncoderAux(), value);
    }

    // actual velocity
    if ( (err = c->ActualVelAvgGet(&value)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
    	        "ControllerMenu::actualValues - not able to get velocity (%s)\n", 
    	        Controller::ErrMsg(err));
	} else {
		printf("Actual velocity (%s/sec): %f\n", c->UnitsEncoder(), value);
	}

    // actual torque
    if ( (err = c->ActualTorque(&value)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
    	        "ControllerMenu::actualValues - not able to get torque (%s)\n", 
    	        Controller::ErrMsg(err));
	} else {
		printf("Actual torque (%s): %f\n", c->UnitsTorque(), value);
	}
	
	Ask::Pause();
}

void ControllerMenu::feedbackLoopType()
{
    bool active;
    Controller::Err err;

	// analog feedback
    if ( (err = c->LoopAnalogGet(&active)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
   "ControllerMenu::feedbackLoopType - not able to get analog feedback setting (%s)\n", 
    	        Controller::ErrMsg(err));
	} else {
		active = Ask::YesNo("Analog Feedback active", active);
    	if ( (err = c->LoopAnalogSet(active)) != Controller::ERR_OK ) {
    		fprintf(stderr, 
    "ControllerMenu::feedbackLoopType - not able to set analog feedback setting (%s)\n", 
    		        Controller::ErrMsg(err));
		}
	}

	// dual loop
    if ( (err = c->LoopDualGet(&active)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
   "ControllerMenu::feedbackLoopType - not able to get dual loop setting (%s)\n", 
    	        Controller::ErrMsg(err));
	} else {
		active = Ask::YesNo("Dual Loop active", active);
    	if ( (err = c->LoopDualSet(active)) != Controller::ERR_OK ) {
    		fprintf(stderr, 
    "ControllerMenu::feedbackLoopType - not able to set dual loop setting (%s)\n", 
    		        Controller::ErrMsg(err));
		}
	}

	Ask::Pause();
}

void ControllerMenu::pidGains()
{	
    float gain;
    Controller::Err err;

	// proportional gain
    if ( (err = c->GainProportionalGet(&gain)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
		"ControllerMenu::pidGains - not able to get proportional gain (%s)\n", 
    	   	    Controller::ErrMsg(err));
	} else {
		// FIX - could add limits like this on other commands
		// scale relative to UNITS
	    gain = Ask::Float("Proportional gain (UNITS)", gain, 0, 1023.875);
		if ( (err = c->GainProportionalSet(gain)) != Controller::ERR_OK ) {
    		fprintf(stderr, 
		"ControllerMenu::pidGains - not able to set proportional gain (%s)\n", 
    	        	Controller::ErrMsg(err));
		}
	}

	// integral gain
    if ( (err = c->GainIntegralGet(&gain)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
		"ControllerMenu::pidGains - not able to get integral gain (%s)\n", 
    	        Controller::ErrMsg(err));
	} else {
	    gain = Ask::Float("Integral gain (UNITS)", gain, 0, 2047.875);
		if ( (err = c->GainIntegralSet(gain)) != Controller::ERR_OK ) {
    		fprintf(stderr, 
		"ControllerMenu::pidGains - not able to set integral gain (%s)\n", 
    	    	    Controller::ErrMsg(err));
		}
	}

	// derivative gain
    if ( (err = c->GainDerivativeGet(&gain)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
		"ControllerMenu::pidGains - not able to get derivative gain (%s)\n", 
    	        Controller::ErrMsg(err));
	} else {
	    gain = Ask::Float("Derivative gain (UNITS)", gain, 0, 4095.875);
		if ( (err = c->GainDerivativeSet(gain)) != Controller::ERR_OK ) {
    		fprintf(stderr, 
		"ControllerMenu::pidGains - not able to set derivative gain (%s)\n", 
    	        	Controller::ErrMsg(err));
		}
	}

    Ask::Pause();
}

void ControllerMenu::integratorTorqueInfo()
{
    float limit, torque, offset;
    //float m, b;
    char prompt[CONTROLLERMENU_PROMPT_LEN];
    Controller::Err err;

	// integrator limit
    if ( (err = c->LimitIntegratorGet(&limit)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
		"ControllerMenu::torqueIntegratorInfo - not able to get integrator limit (%s)\n", 
    	        Controller::ErrMsg(err));
	} else {
	    limit = Ask::Float("Integrator limit (volts)", limit, 0, 9.9988);
		if ( (err = c->LimitIntegratorSet(limit)) != Controller::ERR_OK ) {
    		fprintf(stderr, 
		"ControllerMenu::torqueIntegratorInfo - not able to set integrator limit (%s)\n", 
    	        	Controller::ErrMsg(err));
		}
	}

	// torque limit
    if ( (err = c->LimitTorqueGet(&torque)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
		"ControllerMenu::torqueIntegratorInfo - not able to get torque limit (%s)\n", 
    	        Controller::ErrMsg(err));
	} else {
		snprintf(prompt, CONTROLLERMENU_PROMPT_LEN, 
		         "Torque limit (%s)", c->UnitsTorque());
		//c->ConvTorqueGet(NULL, &m, &b);
	    //torque = Ask::Float(prompt, torque, 0, (9.998-b)/m);
	    torque = Ask::Float(prompt, torque, 0, 9.998);
		if ( (err = c->LimitTorqueSet(torque)) != Controller::ERR_OK ) {
    		fprintf(stderr, 
		"ControllerMenu::torqueIntegratorInfo - not able to set torque limit (%s)\n", 
    	        	Controller::ErrMsg(err));
		}
	}
		
	// torque offset
    if ( (err = c->OffsetTorqueGet(&offset)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
		"ControllerMenu::torqueIntegratorInfo - not able to get torque offset (%s)\n", 
    	        Controller::ErrMsg(err));
	} else {
		snprintf(prompt, CONTROLLERMENU_PROMPT_LEN, 
		         "Torque offset (%s)", c->UnitsTorque());
		//c->ConvTorqueGet(NULL, &m, &b);
	    //offset = Ask::Float(prompt, offset, 0, (9.998-b)/m);
	    offset = Ask::Float(prompt, offset, 0, 9.998);
		if ( (err = c->OffsetTorqueSet(offset)) != Controller::ERR_OK ) {
    		fprintf(stderr, 
		"ControllerMenu::torqueIntegratorInfo - not able to set torque offset (%s)\n", 
    	        	Controller::ErrMsg(err));
		}
	}
	
	Ask::Pause();
}

void ControllerMenu::samplePeriod()
{
	float sec;
	Controller::Err err;

	// sample period
    if ( (err = c->SamplePeriodGet(&sec)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
		"ControllerMenu::samplePeriod - not able to get sample period (%s)\n", 
    	        Controller::ErrMsg(err));
	} else {
	    sec = Ask::Float("Sample period (sec)", sec, 0.000250, 0.020000);
		if ( (err = c->SamplePeriodSet(sec)) != Controller::ERR_OK ) {
    		fprintf(stderr, 
		"ControllerMenu::samplePeriod - not able to set sample period (%s)\n", 
    	    	    Controller::ErrMsg(err));
		}
	}

    Ask::Pause();
}

// assumes Targets program already loaded in controller memory
void ControllerMenu::targetMotion()
{	
	bool running = false, targetsNew = false;
    float posn, vel, veps, accel, decel;
    char prompt[CONTROLLERMENU_PROMPT_LEN];
    Controller::Err err;

	// if program executing, see if want to specify new target or stop execution
	if ( (err = c->TargetRunning(&running)) != Controller::ERR_OK ) {
    		fprintf(stderr, 
    	 "Controller::targetMotion - not able to get execution status (%s)\n", 
    	            Controller::ErrMsg(err));
    	    return;
	} else if ( running ) {
		if ( !(targetsNew = Ask::YesNo("Enter new target parameters", true)) ) {
			if ( Ask::YesNo("Stop execution of #Targets", true) ) {
				if ( (err = c->TargetStop()) != Controller::ERR_OK ) {
    				fprintf(stderr, 
    	      "Controller::targetMotion - not able to stop execution (%s)\n", 
    	      		Controller::ErrMsg(err));
				} else {
					printf("Execution of #Targets stopped\n");
					return;
				}
			}
		}
	}
	
	// get new target parameters if appropriate
	if ( !running || (running && targetsNew) ) {
		// target position
		if ( (err = c->TargetPosnGet(&posn)) != Controller::ERR_OK ) {
    		fprintf(stderr, 
    	"ControllerMenu::targetMotion - not able to get target position (%s)\n", 
    	      	    Controller::ErrMsg(err));
		} else {
			snprintf(prompt, CONTROLLERMENU_PROMPT_LEN, "Target position (%s)",
			         c->UnitsEncoder());
			posn = Ask::Float(prompt, posn);
			if ( (err = c->TargetPosnSet(posn)) != Controller::ERR_OK ) {
    			fprintf(stderr, 
    	"ControllerMenu::targetMotion - not able to set target position (%s)\n", 
    	        Controller::ErrMsg(err));
			}
		}

		// target maximum velocity
    	if ( (err = c->TargetVelGet(&vel)) != Controller::ERR_OK ) {
    		fprintf(stderr, 
	    "ControllerMenu::targetMotion - not able to get target velocity (%s)\n", 
    		        Controller::ErrMsg(err));
		} else {
			snprintf(prompt, CONTROLLERMENU_PROMPT_LEN, "Target velocity (%s/sec)",
			         c->UnitsEncoder());
		    vel = Ask::Float(prompt, vel);
			if ( (err = c->TargetVelSet(vel)) != Controller::ERR_OK ) {
    				fprintf(stderr, 
	    "ControllerMenu::targetMotion - not able to set target velocity (%s)\n", 
    		                Controller::ErrMsg(err));
			}
		}

		// target velocity epsilon (small enough velocity to consider = 0)
    	if ( (err = c->TargetVelEpsGet(&veps)) != Controller::ERR_OK ) {
    		fprintf(stderr, 
	"ControllerMenu::targetMotion - not able to get target velocity eps (%s)\n", 
    		        Controller::ErrMsg(err));
		} else {
			snprintf(prompt, CONTROLLERMENU_PROMPT_LEN, 
			         "Target velocity epsilon, approx zero velocity (%s/sec)",
			         c->UnitsEncoder());
		    veps = Ask::Float(prompt, veps);
			if ( (err = c->TargetVelEpsSet(veps)) != Controller::ERR_OK ) {
    				fprintf(stderr, 
	"ControllerMenu::targetMotion - not able to set target velocity eps (%s)\n", 
    		                Controller::ErrMsg(err));
			}
		}

		// target acceleration
	    if ( (err = c->TargetAccelGet(&accel)) != Controller::ERR_OK ) {
    		fprintf(stderr, 
	"ControllerMenu::targetMotion - not able to get target acceleration (%s)\n", 
    		        Controller::ErrMsg(err));
		} else {
			snprintf(prompt, CONTROLLERMENU_PROMPT_LEN, 
				     "Target acceleration (%s/sec^2)",
				     c->UnitsEncoder());
		    accel = Ask::Float(prompt, accel);
			if ( (err = c->TargetAccelSet(accel)) != Controller::ERR_OK ) {
    			fprintf(stderr, 
	"ControllerMenu::targetMotion - not able to set target acceleration (%s)\n", 
    		            Controller::ErrMsg(err));
			}
		}

		// target deceleration
	    if ( (err = c->TargetDecelGet(&decel)) != Controller::ERR_OK ) {
    		fprintf(stderr, 
	"ControllerMenu::targetMotion - not able to get target deceleration (%s)\n", 
    		        Controller::ErrMsg(err));
		} else {
			snprintf(prompt, CONTROLLERMENU_PROMPT_LEN, 
				     "Target deceleration (%s/sec^2)",
				     c->UnitsEncoder());
		    decel = Ask::Float(prompt, decel);
			if ( (err = c->TargetDecelSet(decel)) != Controller::ERR_OK ) {
    			fprintf(stderr, 
	"ControllerMenu::targetMotion - not able to set target deceleration (%s)\n", 
    		            Controller::ErrMsg(err));
			}
		}

		// activate new target parameters if desired
		if ( !running ) {
			if ( Ask::YesNo("Start executing #Targets", true) ) {
				if ( (err = c->TargetStart()) != Controller::ERR_OK ) {
    				fprintf(stderr, 
    	      "Controller::targetMotion - not able to start execution (%s)\n", 
    	      		Controller::ErrMsg(err));
    	      		fprintf(stderr,
    	      "Controller::targetMotion - may need to load program\n");    	      		        
				} else {
					printf("Execution of #Targets started\n");
				}
			}
		} else if ( Ask::YesNo("Issue new target parameters", true) ) {
			if ( (err = c->TargetNew()) != Controller::ERR_OK ) {
   				fprintf(stderr, 
   	      "Controller::targetMotion - not able to issue new targets (%s)\n", 
   	      		Controller::ErrMsg(err));
			} else {
				printf("New target parameters issued\n");
			}
		}
	}

    Ask::Pause();
}

void ControllerMenu::profileMotion()
{
	bool running = false, done = false, motorOff = false;
    static char absRel = 'a';
    float posn, vel, accel, decel;
    char prompt[CONTROLLERMENU_PROMPT_LEN];
    Controller::Err err;

	// if #Targets executing, need to stop before entering profile parameters
	if ( (err = c->TargetRunning(&running)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
    	 "Controller::profileMotion - not able to get execution status (%s)\n", 
    	        Controller::ErrMsg(err));
    	return;
	} else if ( running ) {
		printf("#Targets program running on controller.  Will need to stop\n");
		printf("so can run individual profile motion.\n");
		if ( Ask::YesNo("Stop execution of #Targets program", true) ) {
			if ( (err = c->TargetStop()) != Controller::ERR_OK ) {
   				fprintf(stderr, 
   	      "Controller::profileMotion - not able to stop execution (%s)\n", 
   	      		Controller::ErrMsg(err));
   	      		return;
			} else {
				printf("Execution of #Targets stopped\n");
			}
		} else {
			printf("Execution of #Targets not stopped\n");
			return;
		}
	}
	
	// if profile executing, need to wait until it's done
	if ( (err = c->MotionDone(&done)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
	"Controller::profileMotion - not able to determine if profile done (%s)\n", 
    	        Controller::ErrMsg(err));
    	return;
	} else if ( !done ) {
		printf("Need to wait until current profile is complete\n");
		return;
	}
	 
    // profile position
    if ( (absRel = Ask::Char("Absolute (a) or Relative (r) position", absRel,
	                         "ar")) == 'a' ) {
		// absolute
		if ( (err = c->ProfilePosnAbsGet(&posn)) != Controller::ERR_OK ) {
    		fprintf(stderr, 
	"ControllerMenu::profileMotion - not able to get profile position (%s)\n", 
    	      Controller::ErrMsg(err));
		} else {
			snprintf(prompt, CONTROLLERMENU_PROMPT_LEN, 
					 "Profile absolute position (%s)",
			         c->UnitsEncoder());
			posn = Ask::Float(prompt, posn);
			if ( (err = c->ProfilePosnAbsSet(posn)) != Controller::ERR_OK ) {
    			fprintf(stderr, 
	"ControllerMenu::targetProfile - not able to set profile position (%s)\n", 
    	        Controller::ErrMsg(err));
			}
		}
	} else {
	    // relative
		if ( (err = c->ProfilePosnRelGet(&posn)) != Controller::ERR_OK ) {
    		fprintf(stderr, 
	"ControllerMenu::profileMotion - not able to get profile position (%s)\n", 
    	      Controller::ErrMsg(err));
		} else {
			snprintf(prompt, CONTROLLERMENU_PROMPT_LEN, 
					 "Profile relative position (%s)",
			         c->UnitsEncoder());
			posn = Ask::Float(prompt, posn);
			if ( (err = c->ProfilePosnRelSet(posn)) != Controller::ERR_OK ) {
    			fprintf(stderr, 
	"ControllerMenu::profileMotion - not able to set profile position (%s)\n", 
    	        Controller::ErrMsg(err));
			}
		}
	}

	// profile maximum velocity
    if ( (err = c->ProfileVelMaxGet(&vel)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
	"ControllerMenu::profileMotion - not able to get profile velocity (%s)\n", 
    	        Controller::ErrMsg(err));
	} else {
		snprintf(prompt, CONTROLLERMENU_PROMPT_LEN, "Profile max velocity (%s/sec)",
		         c->UnitsEncoder());
	    vel = Ask::Float(prompt, vel);
		if ( (err = c->ProfileVelMaxSet(vel)) != Controller::ERR_OK ) {
    			fprintf(stderr, 
	"ControllerMenu::profileMotion - not able to set profile velocity (%s)\n", 
    	                Controller::ErrMsg(err));
		}
	}

	// profile acceleration
    if ( (err = c->ProfileAccelGet(&accel)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
	"ControllerMenu::profileMotion - not able to get profile acceleration (%s)\n", 
    	        Controller::ErrMsg(err));
	} else {
		snprintf(prompt, CONTROLLERMENU_PROMPT_LEN, 
		         "Profile acceleration (%s/sec^2)",
		         c->UnitsEncoder());
	    accel = Ask::Float(prompt, accel);
		if ( (err = c->ProfileAccelSet(accel)) != Controller::ERR_OK ) {
    		fprintf(stderr, 
	"ControllerMenu::profileMotion - not able to set profile acceleration (%s)\n", 
    	            Controller::ErrMsg(err));
		}
	}

	// profile deceleration
    if ( (err = c->ProfileDecelGet(&decel)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
	"ControllerMenu::profileMotion - not able to get profile deceleration (%s)\n", 
    	        Controller::ErrMsg(err));
	} else {
		snprintf(prompt, CONTROLLERMENU_PROMPT_LEN, 
		         "Profile deceleration (%s/sec^2)",
		         c->UnitsEncoder());
	    decel = Ask::Float(prompt, decel);
		if ( (err = c->ProfileDecelSet(decel)) != Controller::ERR_OK ) {
    		fprintf(stderr, 
	"ControllerMenu::profileMotion - not able to set profile deceleration (%s)\n", 
    	            Controller::ErrMsg(err));
		}
	}
	
	// can't begin if motor is off
	if ( (err = c->StatusMotorOff(&motorOff)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
	"ControllerMenu::profileMotion - not able to get motor off status (%s)\n", 
    	        Controller::ErrMsg(err));
	} else {
		if ( motorOff ) {
			printf("Need to turn motor on with Servo Here before beginning motion.\n");
		} else {	
			if ( Ask::YesNo("Begin profile motion", true) ) {
				if ( (err = c->MotionBegin()) != Controller::ERR_OK ) {
					fprintf(stderr, 
  	      "Controller::profileMotion - not able to begin profile motion (%s)\n", 
			      		    Controller::ErrMsg(err));
      				return;
				} else {
					printf("Profile motion started\n");
				}
			}
		}	
	}

	Ask::Pause();
}

void ControllerMenu::jogMotion()
{
    float vel;
    bool motorOff = false;
    char prompt[CONTROLLERMENU_PROMPT_LEN];
    Controller::Err err;

	// jog velocity
    if ( (err = c->JogVelGet(&vel)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
		"ControllerMenu::jogMotion - not able to get jog velocity (%s)\n", 
    	        Controller::ErrMsg(err));
	} else {
		snprintf(prompt, CONTROLLERMENU_PROMPT_LEN, "Jog velocity (%s/sec)",
		         c->UnitsEncoder());
	    vel = Ask::Float(prompt, vel);
		if ( (err = c->JogVelSet(vel)) != Controller::ERR_OK ) {
    		fprintf(stderr, 
		"ControllerMenu::jogMotion - not able to set jog velocity (%s)\n", 
    	            Controller::ErrMsg(err));
		}
	}

	// can't begin if motor is off
	if ( (err = c->StatusMotorOff(&motorOff)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
	"ControllerMenu::jogMotion - not able to get motor off status (%s)\n", 
    	        Controller::ErrMsg(err));
	} else {
		if ( motorOff ) {
			printf("Need to turn motor on with Servo Here before beginning motion.\n");
		} else {	
			if ( Ask::YesNo("Begin jog motion", true) ) {
				if ( (err = c->MotionBegin()) != Controller::ERR_OK ) {
					fprintf(stderr, 
  	      "Controller::jogMotion - not able to begin jog motion (%s)\n", 
			      		    Controller::ErrMsg(err));
      				return;
				} else {
					printf("Jog motion started\n");
				}
			}
		}	
	}

    Ask::Pause();
}

void ControllerMenu::switchConfigFind()
{
	bool limit, home;
    char prompt[CONTROLLERMENU_PROMPT_LEN];
	Controller::Err err;
	
	// configure switches
	limit = Ask::YesNo("Limit switches active high", false);
	home = Ask::YesNo("Go forward when home input is high (otherwise reverse)", 
	                  false);
    if ( (err = c->SwitchConfigure(limit, home)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
		"ControllerMenu::switchConfig - not able to configure switches (%s)\n", 
    	        Controller::ErrMsg(err));
	} else {
		// find edge, if desired
		snprintf(prompt, CONTROLLERMENU_PROMPT_LEN, "Find edge in %s direction",
		         home ? "forward" : "reverse");
		if ( Ask::YesNo(prompt, true) ) {
    		if ( (err = c->SwitchFindEdge()) != Controller::ERR_OK ) {
    			fprintf(stderr, 
		"ControllerMenu::switchConfig - not able to find edge (%s)\n", 
    	        		Controller::ErrMsg(err));
    		}
		}
    }

    Ask::Pause();
}

void ControllerMenu::servoHere()
{
	bool running = false, done = false;
    Controller::Err err;
    
	// if #Targets executing, need to stop before servoing here
	if ( (err = c->TargetRunning(&running)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
    	 "Controller::servoHere - not able to get execution status (%s)\n", 
    	        Controller::ErrMsg(err));
    	return;
	} else if ( running ) {
		printf("#Targets program running on controller.  Will need to stop\n");
		printf("so can servo here.\n");
		if ( Ask::YesNo("Stop execution of #Targets program", true) ) {
			if ( (err = c->TargetStop()) != Controller::ERR_OK ) {
   				fprintf(stderr, 
   	      "Controller::servoHere - not able to stop execution (%s)\n", 
   	      		Controller::ErrMsg(err));
   	      		return;
			} else {
				printf("Execution of #Targets stopped\n");
			}
		} else {
			printf("Execution of #Targets not stopped\n");
			return;
		}
	}
	
	// if profile executing, need to wait until it's done
	if ( (err = c->MotionDone(&done)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
	"Controller::servoHere - not able to determine if profile done (%s)\n", 
    	        Controller::ErrMsg(err));
    	return;
	} else if ( !done ) {
		printf("Need to wait until current profile is complete\n");
		return;
	}

	// servo here
	if ( Ask::YesNo("Servo here", true) ) {
		if ( (err = c->MotionServoHere()) != Controller::ERR_OK ) {
    		fprintf(stderr, 
		"ControllerMenu::servoHere - not able to servo here (%s)\n", 
    	            Controller::ErrMsg(err));
		}
	}

    Ask::Pause();
}

void ControllerMenu::stopAbortMotion()
{
    char choice;
	Controller::Err err;
	
    choice = Ask::Char("Stop (s), abort (a), or don't change (d) motion", 
                       's', "sad");
    
    switch ( choice ) {
    	case 's':
    		if ( (err = c->MotionStop()) != Controller::ERR_OK ) {
    			fprintf(stderr, 
		   	"ControllerMenu::stopAbortMotion - not able to stop motion (%s)\n", 
    	   	    	    Controller::ErrMsg(err));
    		} else {
    			printf("Stopping motion\n");
    		}
    		break;
    	case 'a':
    		if ( (err = c->MotionAbort()) != Controller::ERR_OK ) {
    			fprintf(stderr, 
		   	"ControllerMenu::stopAbortMotion - not able to abort motion (%s)\n", 
    	   	    	    Controller::ErrMsg(err));
    		} else {
    			printf("Aborting motion\n");
    		}
    		break;
    	case 'd':
    	default:
    		break;
    }

    Ask::Pause();
}

void ControllerMenu::convFactors()
{
	char units[CONTROLLER_UNITS_LEN];
	float slope;
	float offset;
	char prompt[CONTROLLERMENU_PROMPT_LEN];
	
	// encoder
	c->ConvEncoderGet(units, &slope, &offset);
	Ask::String("Encoder conversion units", units, units, CONTROLLER_UNITS_LEN);
	snprintf(prompt, CONTROLLERMENU_PROMPT_LEN,
	         "Encoder conversion slope (quad counts/%s)", units); 
	slope = Ask::Float(prompt, slope);
	offset = Ask::Float("Encoder conversion offset (quad counts)", offset);
	printf("\n");
	c->ConvEncoderSet(units, slope, offset);
	
	// encoderAux
	c->ConvEncoderAuxGet(units, &slope, &offset);
	Ask::String("Encoder Aux conversion units", units, units, CONTROLLER_UNITS_LEN);
	snprintf(prompt, CONTROLLERMENU_PROMPT_LEN,
	         "Encoder Aux conversion slope (quad counts/%s)", units);
	slope = Ask::Float(prompt, slope);
	offset = Ask::Float("Encoder Aux conversion offset (quad counts)", offset);
	printf("\n");
	c->ConvEncoderAuxSet(units, slope, offset);

	// torque
	c->ConvTorqueGet(units, &slope, &offset);
	Ask::String("Torque conversion units", units, units, CONTROLLER_UNITS_LEN);
	snprintf(prompt, CONTROLLERMENU_PROMPT_LEN,
	         "Torque conversion slope (volts/%s)", units);
	slope = Ask::Float(prompt, slope);
	offset = Ask::Float("Torque conversion offset (volts)", offset);
	printf("\n");
	c->ConvTorqueSet(units, slope, offset);
		
	// analogInput
	c->ConvAnalogInputGet(units, &slope, &offset);
	Ask::String("Analog Input conversion units", units, units, CONTROLLER_UNITS_LEN);
	snprintf(prompt, CONTROLLERMENU_PROMPT_LEN,
	         "Analog Input conversion slope (volts/%s)", units);
	slope = Ask::Float(prompt, slope);
	offset = Ask::Float("Analog Input conversion offset (volts)", offset);
	printf("\n");
	c->ConvAnalogInputSet(units, slope, offset);
		
	// analogInputAux
	c->ConvAnalogInputAuxGet(units, &slope, &offset);
	Ask::String("Analog Input Aux conversion units", units, units, CONTROLLER_UNITS_LEN);
	snprintf(prompt, CONTROLLERMENU_PROMPT_LEN,
	         "Analog Input Aux conversion slope (volts/%s)", units);
	slope = Ask::Float(prompt, slope);
	offset = Ask::Float("Analog Input Aux conversion offset (volts)", offset);
	printf("\n");
	c->ConvAnalogInputAuxSet(units, slope, offset);

    Ask::Pause();
}

void ControllerMenu::errorAction()
{
    bool action;
    Controller::Err err;

	// off-on-error
    if ( (err = c->ErrorActionGet(&action)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
	"ControllerMenu::errorAction - not able to get off-on-error state (%s)\n", 
    	   	    Controller::ErrMsg(err));
	} else {
	    action = Ask::YesNo("Off-on-Error active", action);
		if ( (err = c->ErrorActionSet(action)) != Controller::ERR_OK ) {
    		fprintf(stderr, 
	"ControllerMenu::errorAction - not able to set off-on-error state (%s)\n", 
    	        	Controller::ErrMsg(err));
		}
	}

    Ask::Pause();
}

void ControllerMenu::errorCommandMotor()
{
	int code;
	char msg[CONTROLLER_MSG_LEN];
	Controller::Err err;

	// why command error
    if ( (err = c->ErrorCodeCommand(&code, msg, sizeof(msg))) != 
         Controller::ERR_OK ) {
    	fprintf(stderr, 
	"ControllerMenu::errorCommandMotor - not able to get command error (%s)\n", 
    	        Controller::ErrMsg(err));
	} else {
	    printf("Command error code:\n");
	    printf("  %s\n", msg);
	}
	
	// why motor stopped
    if ( (err = c->ErrorCodeMotorStop(&code, msg,sizeof(msg))) != 
         Controller::ERR_OK ) {
    	fprintf(stderr, 
	"ControllerMenu::errorCommandMotor - not able to get motor stop code (%s)\n", 
    	        Controller::ErrMsg(err));
	} else {
	    printf("Motor stop code:\n");
	    printf("  %d %s\n", code, msg);
	}
	//	Print offending program line if a line number is available.
	int lineno = 0;
	if ( (err = c->ProgramErrLine(lineno)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
	"ControllerMenu::statusBits - not able to get error line number (%s)\n", 
    	        Controller::ErrMsg(err));
	}
	
	if (lineno > 0)											// if got a line number
	{	std::string s;											// read offending line
		if ( (err = c->ProgramLineRead(lineno,s)) != Controller::ERR_OK ) {
		    	fprintf(stderr, 
					"ControllerMenu::statusBits - not able to get error line (%s)\n", 
    		        Controller::ErrMsg(err));
   		} else {
   			printf("Program stopped at line %d:  %s\n",lineno,s.c_str());	// print offending line
   		}
	}
    Ask::Pause();
}

void ControllerMenu::analogInputs()
{
    float value;
    Controller::Err err;

	Controller::Err AnalogInputGet(int chan, float *value);	// MG @AN[chan=1 or 2] | float (-10 to 10)
	
    // analog inputs
    for ( int i = 1 ; i < 3 ; i++ ) {
		if ( (err = c->AnalogInputGet(i, &value)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
		"ControllerMenu::analogInputs - not able to get analog input %d (%s)\n", 
    	        i, Controller::ErrMsg(err));
		} else {
	    	printf("Analog input %d = %f (%s)\n", i, value, 
	    	       c->UnitsAnalogInput());
		}
    }

    Ask::Pause();
}

void ControllerMenu::digitalInputs()
{
    bool state;
    int stateInt;
	Controller::Err err;
    	
	// digital inputs
	for ( int i = 1 ; i < 8 ; i++ ) {
		if ( (err = c->DigitalInputGet(i, &state)) != Controller::ERR_OK ) {
    		fprintf(stderr, 
"ControllerMenu::digitalInputs - not able to get digital input channel %d (%s)\n", 
	    	        i, Controller::ErrMsg(err));
		} else {
			printf("Digital input   %d = %d\n", i, state ? 1 : 0);
		}
	}
	
	// all digital inputs
	if ( (err = c->DigitalInputGetAll(&stateInt)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
	"ControllerMenu::digitalInputs - not able to get all digital inputs (%s)\n", 
    	        Controller::ErrMsg(err));
	} else {
		printf("Digital input all = %d\n", stateInt);
	}
	printf("\n");

    // switches
	if ( (err = c->SwitchForwardGet(&state)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
	"ControllerMenu::digitalInputs - not able to get forward switch state (%s)\n", 
    	        Controller::ErrMsg(err));
	} else {
		printf("Forward switch = %d\n", state ? 1 : 0);
	}

	if ( (err = c->SwitchReverseGet(&state)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
	"ControllerMenu::digitalInputs - not able to get reverse switch state (%s)\n", 
    	        Controller::ErrMsg(err));
	} else {
		printf("Reverse switch = %d\n", state ? 1 : 0);
	}

    Ask::Pause();
}

void ControllerMenu::digitalOutputs()
{
    bool state;
	int i;
	char prompt[82];
	Controller::Err err;
	
	// digital outputs
	for ( i = 1 ; i < 4 ; i++ ) {
	    sprintf(prompt, "Set digital output %d", i);
		if ( Ask::YesNo(prompt, false) ) {
			if ( (err = c->DigitalOutputGet(i, &state)) != Controller::ERR_OK ) {
    			fprintf(stderr, 
	"ControllerMenu::digitalOutputs - not able to get digital output %d (%s)\n", 
    	    		    i, Controller::ErrMsg(err));
			}
			state = (bool) Ask::Int("High (1) or low (0)", state ? 1 : 0, 0, 1);
			if ( (err = c->DigitalOutputSet(i, state)) != Controller::ERR_OK ) {
    			fprintf(stderr,
	"ControllerMenu::digitalOutputs - not able to set digital output %d (%s)\n", 
    	    		    i, Controller::ErrMsg(err));
			}
		}
	}

    Ask::Pause();
}

void ControllerMenu::viewInputs()
{
    float enc, encAux;
    float ai[2];
	bool di[7];
	bool fsw, rsw;
    char header[] = "enc encAux | ai aiAux | di1 di2 di3 di4 di5 di6 di7 | Fsw Rsw\n";
	char format[] = "%.2f %.2f | %.2f %.2f | %d %d %d %d %d %d %d | %d %d\n";
	int line;
	Controller::Err err;

    printf("Type any key to stop...\n\n");

    line = 0;
    while ( Ask::CharReady() == 0 ) {
	    if ( (line % 20) == 0 ) {
		    printf("%s", header);
		}
		// actual - enc, encAux
		if ( (err = c->ActualPosnGet(&enc)) != Controller::ERR_OK ) {
    		fprintf(stderr, 
	"ControllerMenu::viewInputs - not able to get encoder position (%s)\n", 
    	    	    Controller::ErrMsg(err));
		}
		if ( (err = c->ActualPosnAuxGet(&encAux)) != Controller::ERR_OK ) {
    		fprintf(stderr, 
	"ControllerMenu::viewInputs - not able to get encoder Aux position (%s)\n", 
    	    	    Controller::ErrMsg(err));
		}

    	// analog inputs
    	for ( int i = 1 ; i < 3 ; i++ ) {
			if ( (err = c->AnalogInputGet(i, &(ai[i-1]))) != Controller::ERR_OK ) {
    		fprintf(stderr, 
	"ControllerMenu::viewInputs - not able to get analog input %d (%s)\n", 
    	        i, Controller::ErrMsg(err));
			}
    	}
		// digital inputs
		for ( int i = 1 ; i < 8 ; i++ ) {
			if ( (err = c->DigitalInputGet(i, &(di[i-1]))) != Controller::ERR_OK ) {
    			fprintf(stderr, 
	"ControllerMenu::viewInputs - not able to get digital input %d (%s)\n", 
    	        		i, Controller::ErrMsg(err));
			}
		}
		// switch inputs
		if ( (err = c->SwitchForwardGet(&fsw)) != Controller::ERR_OK ) {
    		fprintf(stderr, 
	"ControllerMenu::viewInputs - not able to get forward switch state (%s)\n", 
    		        Controller::ErrMsg(err));
		}

		if ( (err = c->SwitchReverseGet(&rsw)) != Controller::ERR_OK ) {
    		fprintf(stderr, 
	"ControllerMenu::viewInputs - not able to get reverse switch state (%s)\n", 
    		        Controller::ErrMsg(err));
		}
	
		printf(format,
		       enc, encAux, ai[0], ai[1], 
			   di[0], di[1], di[2], di[3], di[4], di[5], di[6], 
			   fsw, rsw);
		line++;
	}
	Ask::CharFlush(); // flush characters entered to stop printing

    Ask::Pause();
}
		    
void ControllerMenu::programLoadList()
{
	//	Build reasonable default filename for each controller's program
	const char defaultfilename[] = "sandbox/gc/src/qnx/control/galil";
	if (programfilename[0] == '\0')						// if no filename
	{	//	build file name
		const char* homedir = getenv("HOME");		// home directory
		const char* hostname = c->Hostname();			// name of controller
		if (homedir && hostname)							// if got home dir
		{	snprintf(programfilename,sizeof(programfilename),"%s/%s/%s.txt",
				homedir, defaultfilename, hostname);
		}
	}
	Controller::Err err;

    if ( Ask::YesNo("Download program", true) ) {	
    	Ask::String("File name", programfilename, programfilename, CONTROLLER_FILENAME_LEN);
		if ( (err = c->ProgramDownload(programfilename)) != 
	    	 Controller::ERR_OK ) {
    		fprintf(stderr, 
	"ControllerMenu::programLoadList - not able to download program (%s)\n", 
    		        Controller::ErrMsg(err));
		}
    }
    if ( Ask::YesNo("List program", true) ) {
    	size_t k_listingsize = 500*82;
    	char listing[k_listingsize];
    	if ( (err = c->ProgramList(listing, k_listingsize)) != Controller::ERR_OK ) {
    		fprintf(stderr, 
	"ControllerMenu::programLoadList - not able to list program (%s)\n", 
    		        Controller::ErrMsg(err));
		} else {
			printf("%s\n", listing);
    	}
    }

    Ask::Pause();
}

void ControllerMenu::programExecuteHalt()
{
	bool running;
	static char label[CONTROLLER_INSTR_LEN] = "#Targets";
	Controller::Err err;
	
	if ( (err = c->ProgramRunning(&running)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
	"ControllerMenu::programExecuteHalt - not able to deterine if program running (%s)\n", 
    	        Controller::ErrMsg(err));
	} else {
		if ( !running ) {
			if ( Ask::YesNo("Run program", true) ) {
    			Ask::String("Label w/ # (or ' ' to start at line 1)", label, label, 
    			            CONTROLLER_INSTR_LEN);
				if ( (err = c->ProgramExecute(label)) != Controller::ERR_OK ) {
					fprintf(stderr,
	"ControllerMenu::programExecuteHalt - not able to execute program (%s)\n", 
    	        			Controller::ErrMsg(err));
				} else {
					printf("Program running\n");
				}
			}
		} else {
			printf("Program running\n");
			if ( Ask::YesNo("Halt program", false) ) {
				if ( (err = c->ProgramHalt()) != Controller::ERR_OK ) {
					fprintf(stderr,
	"ControllerMenu::programExecuteHalt - not able to halt program (%s)\n", 
    	        			Controller::ErrMsg(err));
				}
			}
		}
	}		

    Ask::Pause();
}

void ControllerMenu::programBreakPtSingleStep()
{
	printf("Capability under construction\n");

    Ask::Pause();
}

void ControllerMenu::programVariable()
{
	printf("Capability under construction\n");

    Ask::Pause();
}

void ControllerMenu::statusBits()
{
    bool state;
	Controller::Err err;

    // axis in motion
	if ( (err = c->StatusAxisInMotion(&state)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
	"ControllerMenu::statusBits - not able to get axis in motion status (%s)\n", 
    	        Controller::ErrMsg(err));
	} else if ( state ) {
	    printf(" bit 7 (1): Axis in motion\n");
	} else {
		printf(" bit 7 (0): Axis not in motion\n");
	}

    // error limit exceeded
	if ( (err = c->StatusErrorLimitExceeded(&state)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
	"ControllerMenu::statusBits - not able to get error limit exceeded status (%s)\n", 
    	        Controller::ErrMsg(err));
	} else if ( state ) {
	    printf(" bit 6 (1): Error limit exceeded\n");
	} else {
		printf(" bit 6 (0): Error limit not exceeded\n");
	}

    // motor off
	if ( (err = c->StatusMotorOff(&state)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
	"ControllerMenu::statusBits - not able to get motor off status (%s)\n", 
    	        Controller::ErrMsg(err));
	} else if ( state ) {
	    printf(" bit 5 (1): Motor off\n");
	} else {
		printf(" bit 5 (0): Motor on\n");
	}

	// bit 4 undefined
	printf(" bit 4 ( ): undefinded\n");

    // forward limit
	if ( (err = c->StatusForwardLimitInactive(&state)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
	"ControllerMenu::statusBits - not able to get forward limit status (%s)\n", 
    	        Controller::ErrMsg(err));
	} else if ( state ) {
	    printf(" bit 3 (1): Forward limit inactive\n");
	} else {
		printf(" bit 3 (0): Forward limit active\n");
	}

    // reverse limit
	if ( (err = c->StatusReverseLimitInactive(&state)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
	"ControllerMenu::statusBits - not able to get reverse limit status (%s)\n", 
    	        Controller::ErrMsg(err));
	} else if ( state ) {
	    printf(" bit 2 (1): Reverse limit inactive\n");
	} else {
		printf(" bit 2 (0): Reverse limit active\n");
	}

    // home switch
	if ( (err = c->StatusHomeSwitch(&state)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
	"ControllerMenu::statusBits - not able to get home switch status (%s)\n", 
    	        Controller::ErrMsg(err));
	} else if ( state ) {
	    printf(" bit 1 (1): Home switch high\n");
	} else {
		printf(" bit 1 (0): Home switch low\n");
	}

    // latch armed
	if ( (err = c->StatusLatchNotArmed(&state)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
	"ControllerMenu::statusBits - not able to get latch armed status (%s)\n", 
    	        Controller::ErrMsg(err));
	} else if ( state ) {
	    printf(" bit 0 (1): Latch not armed\n");
	} else {
		printf(" bit 0 (0): Latch armed\n");
	}
	
	// handle info
	size_t k_handsize = 7*CONTROLLER_MSG_LEN;
    char hand[k_handsize];
	if ( (err = c->StatusHandle(hand, k_handsize)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
	"ControllerMenu::statusBits - not able to get handle info (%s)\n", 
    	        Controller::ErrMsg(err));
	} else {
	    printf("Handle info:\n %s\n", hand);
	}
#ifdef OBSOLETE		// don't get firmware version - nonstandard interaction
    // firmware version
	char ver[CONTROLLER_MSG_LEN];
	if ( (err = c->StatusFirmwareVersionGet(ver)) != Controller::ERR_OK ) {
    	fprintf(stderr, 
	"ControllerMenu::statusBits - not able to get firmware version (%s)\n", 
    	        Controller::ErrMsg(err));
	} else {
	    printf("Firmware version: %s\n", ver);
	}
#endif // OBSOLETE

    Ask::Pause();
}

void ControllerMenu::burnParametersProgram()
{
    Controller::Err err;
    
	// save controller parameters to EEPROM (takes approx 1 sec)
	if ( Ask::YesNo("Save controller parameters to EEPROM", false) ) {
		if ( (err = c->BurnParameters()) != Controller::ERR_OK ) {
    		fprintf(stderr, 
		"ControllerMenu::burnParametersProgram - not able to save controller parameters to EEPROM (%s)\n", 
    	    	    Controller::ErrMsg(err));
		} else {
			printf("Saved controller parameters to EEPROM\n");
		}
	}

	// save application program to EEPROM (takes approx 10 sec)
	if ( Ask::YesNo("Save application program to EEPROM", false) ) {
		if ( (err = c->BurnProgram()) != Controller::ERR_OK ) {
    		fprintf(stderr, 
		"ControllerMenu::burnParametersProgram - not able to save application program to EEPROM (%s)\n", 
    	    	    Controller::ErrMsg(err));
		} else {
			printf("Saved application program to EEPROM\n");
		}
	}

    Ask::Pause();
}

void ControllerMenu::resetController()
{
    Controller::Err err;
    
	// reset to power up configuration
	if ( Ask::YesNo("Reset controller to power up configuration", false) ) {
		if ( (err = c->ResetToPowerUp()) != Controller::ERR_OK ) {
    		fprintf(stderr, 
		"ControllerMenu::resetController - not able to reset to power up configuration (%s)\n", 
    	    	    Controller::ErrMsg(err));
		} else {
			printf("Controller reset to power up configuration\n");
		}
	}

    Ask::Pause();
}

void ControllerMenu::verbose()
{
	c->Verbose(Ask::YesNo("Verbose mode", c->Verbose()));
}

//	
//	variableGetSet  -- get and set variable
//
void ControllerMenu::variableGetSet()
{
	Ask::String("Variable", varname, varname, CONTROLLER_VARNAME_LEN);
	if (varname[0])														// if got a string
	{	//	Got variable name, read its value
		float oldval;
		Controller::Err err = c->VariableGet(varname, &oldval);
		if (err  != Controller::ERR_OK ) {
    		fprintf(stderr, 
		"ControllerMenu::variableGetSet - not able to read variable \"%s\" (%s)\n", 
    	    	    varname,Controller::ErrMsg(err));
    	    	    return;
   		}
   		//	Display value, offer the opportunity to change it.
		char prompt[CONTROLLERMENU_PROMPT_LEN];
		snprintf(prompt, CONTROLLERMENU_PROMPT_LEN, "%s = %f", varname, oldval);
		float newval = Ask::Float(prompt, oldval);		// ask for new value.  Default is old value	
		if (newval != oldval)
		{	err = c->VariableSet(varname,newval);
			if (err  != Controller::ERR_OK ) {
    			fprintf(stderr, 
					"ControllerMenu::variableGetSet - not able to set variable \"%s\" (%s)\n", 
    	    	  	 varname,Controller::ErrMsg(err));
    	    	    return;
   			}
   			//	Read back as check
   			float changedval;
   			err = c->VariableGet(varname, &changedval);
			if (err  != Controller::ERR_OK ) {
    			fprintf(stderr, 
				"ControllerMenu::variableGetSet - not able to read variable \"%s\" (%s)\n", 
    	    	    varname,Controller::ErrMsg(err));
    	    	 return;
   			}
			printf("Variable \"%s\" changed from %f to %f\n",
				varname,oldval, changedval);
		}
		Ask::Pause();
	}
}