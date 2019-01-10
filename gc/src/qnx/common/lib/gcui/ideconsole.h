/////////////////////////////////////////////////////////////////////////////
//
//    File: ideconsole.h
//
//    Usage:
//        #include "ideconsole.h"
//
//    Description:
//        Header file for ideconsole.cc.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        September, 2003
//
/////////////////////////////////////////////////////////////////////////////

#ifndef IDECONSOLE_H
#define IDECONSOLE_H

#include <stdio.h>
#include <unistd.h>

class IDEConsole {
public:
	static void IDEConsole::StdoutNoBuffering();
};

#endif // IDECONSOLE_H