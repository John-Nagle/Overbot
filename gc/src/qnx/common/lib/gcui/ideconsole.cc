////////////////////////////////////////////////////////////////////////////
//
//    File: ideui.cc
//
//    Usage:
//        #include "ideui.h"
//
//    Description:
//        Routines to make the IDE interface usable. Hopefully some of these
//        issues will be addressed in the next release (6.3?).
//
//        This works most of the time.  If after a while the IDE Console
//        view is not accepting input characters, simply close the IDE
//        and restart it.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        September, 2003
//
/////////////////////////////////////////////////////////////////////////////

#include "ideconsole.h"

namespace IDEConsole {

//
//    IDEConsole::StdoutNoBuffering - set stdout to have no buffering
//
//    Momentics Console view currently doesn't flush stdout for printf,
//    so it can't be interactive.  This code sets stdout to have no buffering.
//    The Console view is supposed to be no buffering by default in QNX 6.3?
//
//    Only call this once.
//
//    Returns nothing
//
//	This doesn't work with GCC 3.x, because of a known bug in "setvbuf".
//
void IDEConsole::StdoutNoBuffering() {
	if (isatty(0)) return;									// if stdin is a "terminal" we don't need this.
	char buf[] = "Warning: unable to set IDEConsole::StdoutNoBuffering().\n";
	
	if ( setvbuf(stdout, NULL,_IONBF, 0) != 0 ) {
		write(STDOUT_FILENO, buf, sizeof(buf)-1);
	}
}

}