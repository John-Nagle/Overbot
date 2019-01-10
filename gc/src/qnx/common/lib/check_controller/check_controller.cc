/////////////////////////////////////////////////////////////////////////////
//
//    File: check_controller.cc
//
//    Usage:
//        check_controller
//
//    Description:
//        This is the check program for the controller module in the
//        gccontrol library.
//
//        Hook up a Galil controller unit and check out all the menu options!
//
//    Based on check_mvp.cc of July 2003.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        February 2004
//
//	Revised by John Nagle
//		April 2004.
//		-- currently brings up steering and brake controllers.
//		-- when running, toggles watchdog.
//
//		-- Will connect to all controllers on an as-needed basis.
//		-- "-w" option required to command controllers.  Otherwise read-only.
//
////////////////////////////////////////////////////////////////////////////
#include <exception>
#include "ethernetmenu.h"
#include "controller.h"
#include "controllermenu.h"
#include "ask.h"
#include "timedloop.h"
#include "timedloopmenu.h"


const float k_watchdog_interval = 0.100;								// must toggle watchdog this often
const int k_watchdog_priority = 14;										// Value for test program only.
const double k_connectwait = 0.5;										// wait no more than this for a network connection
//

//
//	class WatchdogToggle -- turns watchdog toggling on and off
//
class WatchdogToggle: TimedLoop {
private:
	MenuHandler& m_menuhandler;										// relevant menu handler
	Controller& m_controller;													// relevant controller
	bool m_toggle;																	// current toggle state
	TimedLoopMenu tm;
public:
	WatchdogToggle(MenuHandler& menuHand, Controller& controller)
	: 	TimedLoop(k_watchdog_interval, k_watchdog_priority),
		m_menuhandler(menuHand), m_controller(controller), m_toggle(false),
		tm(this, "WatchdogToggle", 't', 'h')
	{		
		m_menuhandler.Install(&tm);
	}
private:
	
	void code();	  
	int myData;
};

//
//	code -- called every 500ms.
//
//	Toggles watchdog hardware, keeping the E-stop unit happy
//
//	Don't print anything here unless in verbose mode.  This runs at high priority and as a timed loop,
//	so it will keep the program from doing anything else.
//
void WatchdogToggle::code()
{	const int k_watchdogDigitalOut = 1;					// watchdog is on channel 1
	if (!m_controller.Connected()) 
	{	int stat = m_controller.Connect(k_connectwait);	// try to connect if necessary
		if (stat)															// if trouble
		{	if (m_controller.Verbose()) { perror("Watchdog toggle cannot connect");	 }// report it
			return;
		}
	}
	Controller::Err err = m_controller.DigitalOutputSet(k_watchdogDigitalOut, m_toggle ? 1 : 0);	// set toggle
	if (err != Controller::ERR_OK)								// if trouble
	{	if (m_controller.Verbose()) 
		{ printf("Watchdog toggle error: %s\n", Controller::ErrMsg(err));	// report it
			fflush(stdout);
		}
	}
	m_toggle = !m_toggle;										// invert toggle
}

//
//	Main program, inside try block
//
//	Test use only. Not part of running vehicle code
//
int runmain(int argc, const char* argv[], bool readonly, bool datagrammode)
{	
	Ethernet e1("gcbrake", 0, 100000);
	EthernetMenu em1(&e1, "Ethernet TCP/IP (gcbrake)", 'e', 'c');
	Controller c1("gcbrake", readonly);
    ControllerMenu cm1(&c1, "Controller (gcbrake)", 'c', 's');
	Ethernet e2("gcsteer", 0, 100000);
	EthernetMenu em2(&e2, "Ethernet TCP/IP (gcsteer)", 'e', 'c');
	Controller c2("gcsteer",readonly);
    ControllerMenu cm2(&c2, "Controller (gcsteer)", 'c', 's');
	Ethernet e3("gcthrottle", 0, 100000);
	EthernetMenu em3(&e3, "Ethernet TCP/IP (gcthrottle)", 'e', 'c');
	Controller c3("gcthrottle", readonly);
    ControllerMenu cm3(&c3, "Controller (gcthrottle)", 'c', 's');

	Ethernet e4("gctransmission", 0, 100000);
	EthernetMenu em4(&e4, "Ethernet TCP/IP (gctransmission)", 'e', 'c');
	Controller c4("gctransmission",readonly);
    ControllerMenu cm4(&c4, "Controller (gctransmission)", 'c', 's');

	Ethernet e5("gctilt", 0, 100000);
	EthernetMenu em5(&e5, "Ethernet TCP/IP (gctilt)", 'e', 'c');
	Controller c5("gctilt", readonly);
    ControllerMenu cm5(&c5, "Controller (gctilt)", 'c', 's');
    //	Datagram mode support
    e1.DatagramMode(datagrammode);
    e2.DatagramMode(datagrammode);
    e3.DatagramMode(datagrammode);
    e4.DatagramMode(datagrammode);
    e5.DatagramMode(datagrammode);
	c1.DatagramMode(datagrammode);
	c2.DatagramMode(datagrammode);
	c3.DatagramMode(datagrammode);
	c4.DatagramMode(datagrammode);
	c5.DatagramMode(datagrammode);

    MenuHandler mh;
    mh.Install(&em1);
    mh.Install(&cm1);
    mh.Install(&em2);
    mh.Install(&cm2);
    mh.Install(&em3);
    mh.Install(&cm3);
    mh.Install(&em4);
    mh.Install(&cm4);
    mh.Install(&em5);
    mh.Install(&cm5);
    if (!readonly)
    {	WatchdogToggle wt(mh, c1);								// add watchdog toggle and its menu
    }
    mh.Start();
    return(0);																	// success
}
//
//	Usage -- standard usage message
//
static void usage()
{	printf("usage: check_controller [-w]\n");
	printf("  -w	allow writing (operations that affect controller)\n");
	printf("  -u    use UDP (default is TCP)\n");
	exit(1);																		// fails
}
//
//	Main program
//
int main(int argc, const char* argv[])
{	bool readonly = true;												// assume read-only mode
	bool datagrammode = false;									// assume TCP mode	
	for (int i=1; i<argc; i++)
	{	const char* arg = argv[i];									// this arg
		if (arg[0] != '-') continue;										// ignore non-flag args
		switch (arg[1]) {													// fan out on flag arg
		case 'w': readonly = false; break;							// allow writing
		case 'u': datagrammode = true; break;					// UDP mode
		default: usage(); break;
		}
	}
	try
	{	
		return(runmain(argc, argv, readonly, datagrammode));
	}
  	catch (const char* msg)
  	{	fprintf(stderr,"Exception: %s.\n",msg);
  		exit(1);																	// fails
  	}
  	catch (const std::exception& except)
  	{	fprintf(stderr,"Standard exception: %s\n", except.what());
  		exit(1);
  	}
  	catch (...)
  	{	fprintf(stderr,"Exception (unknown type).\n");
  		exit(1);
  	}
  	return(0);
}
