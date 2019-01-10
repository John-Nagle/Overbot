////////////////////////////////////////////////////////////////////////////
//
//    File: controllermenu.h
//
//    Usage:
//        #include "controllermenu.h"
//
//        Ethernet e("myhostname", 0, 100000);
//        EthernetMenu em(&e, "Ethernet TCP/IP (myhostname)", 'e', 'g');
//
//        Controller c("myhostname");
//        ControllerMenu cm("Galil Controller (myhostname)", 'c', 'S');
//
//        MenuHandler mh;
//
//        mh.Install(&em);
//        mh.Install(&cm);
//        mh.Start();
//
//    Description:
//        Menu capability for Controller objects.  Use of capability is optional.
//        Multiple menus can be installed for multiple Controller units.
//
//    Based on mvpservermenu.h and mvpmenu.h of August and November 2003.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        February, 2004
//
/////////////////////////////////////////////////////////////////////////////

#ifndef CONTROLLERMENU_H
#define CONTROLLERMENU_H

#include "ask.h"
#include "menu.h"
#include "controller.h"

const size_t CONTROLLERMENU_PROMPT_LEN = 82;
const size_t CONTROLLER_VARNAME_LEN = 9;		// max size of variable name in controller + 1

class ControllerMenu: public Menu {
public:
    ControllerMenu(Controller *cont, char *description, char selectChar, char defaultChar) :
        Menu(description, selectChar, defaultChar),
  		c(cont)
    {
    	varname[0]='\0';											// no default variable name yet
    	programfilename[0]='\0';								// no program filename yet
    }
	void Display();
	bool Process(char c);
	bool Reconnect();												// try to reconnect if necessary
private:
	// Controller to which this menu applies
	Controller *c;
	
	// option processing routines
	void motorOff();
	void definePositions();
	void positionLimits();
	void actualValues();
	void feedbackLoopType();
	void pidGains();
	void integratorTorqueInfo();
	void samplePeriod();
	void targetMotion();
	void profileMotion();
	void jogMotion();
	void switchConfigFind();
	void servoHere();
	void stopAbortMotion();
	void convFactors();
	void errorAction();
	void errorCommandMotor();
	void analogInputs();
	void digitalInputs();
	void digitalOutputs();
	void viewInputs();
	void programLoadList();
	void programExecuteHalt();
	void programBreakPtSingleStep();
	void programVariable();
	void statusBits();
	void burnParametersProgram();
	void resetController();
	void verbose();
	void variableGetSet();	
	//	Remember variable name queried between calls.
	char varname[CONTROLLER_VARNAME_LEN];
	//	Remember program download filename
	char programfilename[CONTROLLER_FILENAME_LEN];
};

#endif // CONTROLLERMENU_H
