/////////////////////////////////////////////////////////////////////////////
//
//    File: timedloopmenu.h
//
//    Usage:
//        #include "timedloopmenu.h"
//
//        TimedLoop t(&code, 1.0, 13);
//        TimedLoopMenu tm(&t, "My Timed Loop", 't', '?');
//
//        MenuHandler mh;
//
//        mh.Install(&tm);
//        mh.Start();
//
//    Description:
//        Menu capability for TimedLoop objects.  Use of capability is optional.
//        Multiple menus can be installed for multiple TimedLoop objects.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        October, 2003
//
/////////////////////////////////////////////////////////////////////////////

#ifndef TIMEDLOOPMENU_H
#define TIMEDLOOPMENU_H

#include "ask.h"
#include "menu.h"
#include "timedloop.h"

class TimedLoopMenu: public Menu {
public:
    TimedLoopMenu(TimedLoop *timedloop, char *description, char selectChar, char defaultChar):
        Menu(description, selectChar, defaultChar)
        {
            t = timedloop;
		}
	void Display();
	bool Process(char c);
private:
    // TimedLoop object to which this menu applies
    TimedLoop *t;

	// option processing routines
	void start();
	void pause();
	void resume();
	void stop();
	void samplePeriod();
	void priority();
	void execution();
	void overrun();
	void resolution();
};

#endif // TIMEDLOOPMENU_H
