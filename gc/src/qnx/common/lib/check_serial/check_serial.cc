/////////////////////////////////////////////////////////////////////////////
//
//    File: check_serial.cc
//
//    Usage:
//        check_serial
//
//    Description:
//        A test file for serial.cc and serialmenu.cc.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        July 2003
//
////////////////////////////////////////////////////////////////////////////

#include "serial.h"
#include "serialmenu.h"
#include "ask.h"
#include "ideconsole.h"

int main(int argc, char *argv[])
{
	IDEConsole::StdoutNoBuffering();
	
	Serial s("/dev/ser1", 19200, "8N1", 100000);
    SerialMenu sm(&s, "Serial Port (/dev/ser1)", 's', 'o');
    MenuHandler mh;

    mh.Install(&sm);
    mh.Start();
}