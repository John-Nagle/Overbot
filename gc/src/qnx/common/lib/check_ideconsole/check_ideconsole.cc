/////////////////////////////////////////////////////////////////////////////
//
//    File: check_ideconsole.cc
//
//    Usage:
//        check_ideconsole
//
//    Description:
//        A test file for ideconsole.cc.
//
//        Run this program 2 times in an IDE Console view: one time with
//        and one time without no buffering. When no buffering is turned
//        on, you should be able to see the output of the printf and
//        interact with the program.  In the other case, no output should
//        be written.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        September, 2003
//
////////////////////////////////////////////////////////////////////////////

#include "ask.h"
#include "ideconsole.h"

int main(int argc, char *argv[])
{
	// comment out the next line to check without no buffering
    IDEConsole::StdoutNoBuffering();
    
    printf("Click in this view to make it active.\n");
    Ask::Pause("Type some characters to this prompt and see them");
    
    return EXIT_SUCCESS;
}
