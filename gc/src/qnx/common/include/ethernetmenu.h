/////////////////////////////////////////////////////////////////////////////
//
//    File: ethernetmenu.h
//
//    Usage:
//        #include "ethernet.h"
//        #include "ethernetmenu.h"
//
//        Ethernet e("myhostname", 0, 100000);
//
//        EthernetMenu em(&e, "Ethernet TCP/IP (myhostname)", 'e', 'g');
//        MenuHandler mh;
//        mh.Install(&em);
//        mh.Start();
//
//    Description:
//        Menu capability for Ethernet objects.  Use of the capability is
//        optional. Multiple menus can be installed for multiple Ethernet
//        TCP/IP sockets.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        January, 2003
//
/////////////////////////////////////////////////////////////////////////////

#ifndef ETHERNETMENU_H
#define ETHERNETMENU_H

#include "ask.h"
#include "menu.h"
#include "ethernet.h"

class EthernetMenu: public Menu {
public:
    EthernetMenu(Ethernet *ethernet, char *description, char selectChar, 
                 char defaultChar):
        Menu(description, selectChar, defaultChar)
        {
            e = ethernet;
		}
	void Display();
	bool Process(char c);
private:
    Ethernet *e;
    void configGet();
    void sendBuf();
    void sendController();
    void recvBuf();
    void recvController();
    void interactController();
    void verbose();
};

#endif // ETHERNETMENU_H
