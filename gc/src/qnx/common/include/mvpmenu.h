/////////////////////////////////////////////////////////////////////////////
//
//    File: mvpmenu.h
//
//    Usage:
//        #include "mvpmenu.h"
//
//        Serial s("/dev/ser1", 38400, "8N1", 100000);
//        SerialMenu sm(&s, "Serial Port (/dev/ser1)", 's', 's');
//
//        MVPServer ms(&s);
//        MVP m(&ms, 4);
//        MVPMenu mm("MVP (node 4)", 'm', 'h');
//
//        MenuHandler mh;
//
//        mh.Install(&sm);
//        mh.Install(&mm);
//        mh.Start();
//
//    Description:
//        Menu capability for MVP objects.  Use of capability is optional.
//        Multiple menus can be installed for multiple MVP units.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        August, 2003
//
/////////////////////////////////////////////////////////////////////////////

#ifndef MVPMENU_H
#define MVPMENU_H

#include "ask.h"
#include "menu.h"
#include "mvp.h"

#define MVPMENU_PROMPT_LEN (82)

class MVPMenu: public Menu {
public:
    MVPMenu(MVP *mvp, char *description, char selectChar, char defaultChar):
        Menu(description, selectChar, defaultChar)
        {
            m = mvp;
		}
	void Display();
	bool Process(char c);
private:
    // MVP object to which this menu applies
    MVP *m;

	// option processing routines
    void nodeHomeEnable();
    void nodeDisable();
	void actualPosnVelError();
    void targetProfile();
	void constantVelocity();
	void errorHandling();
	void limitsPosn();
	void limitsCurrentPWM();
	void motion();
	void motionAbort();
	void motionAbortParameters();
	void homeArm();
	void homeParameters();
	void viewInputs();
	void gains();
	void samplePeriod();
	void analogInputMode();
	void analogInputs();
	void analogOutput();
	void digitalInputs();
	void digitalOutputs();
	void macroStatusExecute();
	void communicationParameters();
	void statusBits();
	void statusAll();
	void eeprom();
	void convFactors();
	void resetRecover();
};

#endif // MVPMENU_H
