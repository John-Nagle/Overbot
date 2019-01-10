////////////////////////////////////////////////////////////////////////////
//
//    File: check_mvp.cc
//
//    Usage:
//        check_mvp
//
//    Description:
//        This is the check program for the mvp module in the
//        gccontrol library.
//
//        Hook up an MVP unit and check out all the menu options!
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        August, 2003
//
/////////////////////////////////////////////////////////////////////////////

#include "mvpmenu.h"
#include "ideconsole.h"

int main(void) {
	IDEConsole::StdoutNoBuffering();
	
	MVP m1("MVPC", 0.2, 1);
	MVPMenu mm1(&m1, "MVP Node 1: Brake", 'n', 's');
	
	
	MVP m2("MVPC", 0.2, 2);
	MVPMenu mm2(&m2, "MVP Node 2: Transmission", 'n', 's');
	
	
	MVP m3("MVPC", 0.2, 3);
	MVPMenu mm3(&m3, "MVP Node 3: Steering", 'n', 's');
	
	
	MVP m4("MVPC", 0.2, 4);
	MVPMenu mm4(&m4, "MVP Node 4: Throttle", 'n', 's');
	
	
	MVP m5("MVPT", 0.2, 5);
	MVPMenu mm5(&m5, "MVP Node 5: Tilt", 'n', 's');
	
	
	MVP m6("MVPC", 0.2, 6);
	MVPMenu mm6(&m6, "MVP Node 6: Benchtop", 'n', 's');

	MenuHandler mh;

	mh.Install(&mm1);
	mh.Install(&mm2);
	mh.Install(&mm3);
	mh.Install(&mm4);
	mh.Install(&mm5);
	mh.Install(&mm6);
	mh.Start();
}
