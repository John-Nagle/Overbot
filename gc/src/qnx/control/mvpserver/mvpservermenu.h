////////////////////////////////////////////////////////////////////////////
//
//    File: mvpservermenu.h
//
//    Usage:
//        #include "mvpservermenu.h"
//
//    Description:
//        This is the header file for mvpservermenu.cc.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        November, 2003
//
/////////////////////////////////////////////////////////////////////////////

#ifndef MVPSERVERMENU_H
#define MVPSERVERMENU_H

#include "mvpserver_.h"

class MVPServerMenu: public Menu {
public:
    MVPServerMenu(MVPServer_ *mvpServer, char *description, char selectChar, char defaultChar):
        Menu(description, selectChar, defaultChar)
    {
    	ms = mvpServer;

    	// define client port, 0.2 sec timeout
	    clientport = new MsgClientPort(getenv("ID"), 0.2);
               
        // default instruction settings
        node = 3;
        sprintf(cmdStr, "POR");
        paramActive = false;
    }
	void Display();
	bool Process(char c);
private:
	// MVP Server to which this menu applies
	MVPServer_ *ms;
	
	// server/client info
	MsgClientPort *clientport;
	
	// MVP instruction info
	int node;
	char cmdStr[MVPSERVER_CMD_LEN];
	int param;
	int paramActive;
	int value;
	
	// private member functions
	void verbose(bool verbose);
	bool verbose();
};

#endif // MVPSERVERMENU_H