/////////////////////////////////////////////////////////////////////////////
//
//    File: serialmenu.h
//
//    Usage:
//        #include "serial.h"
//        #include "serialmenu.h"
//
//        Serial s("/dev/ser1", 19200, "8N1", 100000);
//
//        SerialMenu sm(&s, "Serial Port (/dev/ser1)", 's', 's');
//        MenuHandler mh;
//        mh.Install(&sm);
//        mh.Start();
//
//    Description:
//        Menu capability for Serial objects.  Use capability is optional.
//        Multiple menus can be installed for multiple serial ports.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        July, 2003
//
/////////////////////////////////////////////////////////////////////////////

#ifndef SERIALMENU_H
#define SERIALMENU_H

#include "ask.h"
#include "menu.h"
#include "serial.h"

class SerialMenu: public Menu {
public:
    SerialMenu(Serial *serial, char *description, char selectChar, 
               char defaultChar):
        Menu(description, selectChar, defaultChar)
        {
            s = serial;
		}
	void Display();
	bool Process(char c);
private:
    Serial *s;
    void configSet();
    void configGet();
    void writeBuf();
    void readBuf();
    void writeMVP();
    void readMVP();
    void interactMVP();
    void verbose();
};

#endif // SERIALMENU_H
