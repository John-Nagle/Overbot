/////////////////////////////////////////////////////////////////////////////
//
//    File: actservermenu.h
//
//    Usage:
//        #include "actservermenu.h"
//
//    Description:
//       This is the header file for actservermenu.cc.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        December, 2003
//
/////////////////////////////////////////////////////////////////////////////

#ifndef ACTSERVERMENU_H
#define ACTSERVERMENU_H

#include "actserver_.h"

class ActServerMenu: public Menu {
public:
    ActServerMenu(ActServer_ *actServer, char *description, char selectChar, 
        char defaultChar): Menu(description, selectChar, defaultChar)
    {
    	as = actServer;
    	
    	// define default client port, 0.2 sec timeout
		clientport = new MsgClientPort(getenv("ID"), 0.2);
    }
	void Display();
	virtual void DisplayAdditional() {}
	bool Process(char c);
	virtual bool ProcessAdditional(char c) { return false; }
private:
	// Actuator Server to which this menu applies
	ActServer_ *as;
	
	// server/client info
	MsgClientPort *clientport;
	
	void simulation(bool simulation);
	bool simulation();
	
	bool initialize();
	void initialize(bool initialize);
	
	void verbose(bool verbose);
	bool verbose();
};

#endif // ACTSERVERMENU_H
