////////////////////////////////////////////////////////////////////////////
//
//    File: check_timedloop.cc
//
//    Usage:
//        check_timedloop
//
//    Description:
//        This is the check program for the timedloop module in the
//        gccontrol library.
//
//        Check the module in the following manner:
//			- Run the program in a terminal window
//			- Select 't' to go to the My Timed Loop menu
//			- Select 's' to start the timed loop ('.'s will print for
//				each loop.
//			- Select 'x' to stop the timed loop ('.'s will no longer print)
//			- Select 's' to restart the loop
//			- Select 'p' to change the sample period, confirm visually
//			- Select 'r' to change the priority, confirm with 'pidin'
//			- Select 'e' to view the execution statistics (could change
//				code with usleep() to confirm calaculations)
//			- Select 'o' to view overruns (make sample period short relative
//				to execution time to get some overruns)
//			- Select 'c' to view the clock resolution (should be 1 ms
//				>= 40MHz, and 10 ms for < 40 MHz per help page on 
//				ClockPeriod())
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        October, 2003
//
/////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <unistd.h>
#include "ideconsole.h"
#include "timedloop.h"
#include "timedloopmenu.h"

class MyClass: public TimedLoop {
public:
	MyClass(MenuHandler *menuHand)
	: TimedLoop(2.3,14)
	{
		mh = menuHand;
		
		tm = new TimedLoopMenu(this, "My Timed Loop", 't', 'h');
		mh->Install(tm);
		
		myData = 42;	// number to illustrate accessing data member in code()
	}
	void DoNothing();
private:
	MenuHandler *mh;
	TimedLoopMenu *tm;
	void code();	// this does the work
	  
	int myData;
};

void MyClass::DoNothing()
{
	// does nothing
}

void MyClass::code()
{	
	
	printf(". %i ", myData);
	//usleep(1);	// for timing tests
	
}

int main(int argc, char *argv[]) {
	IDEConsole::StdoutNoBuffering();
	
	MenuHandler mh;

	MyClass c(&mh);
	c.DoNothing();	// so compiler won't complain about not using variable c.
	
	mh.Start();
}