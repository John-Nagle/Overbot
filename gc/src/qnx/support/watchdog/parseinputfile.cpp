//
//	Watchdog  -- top-level watchdog program for QNX real-time applications
//
//	File parseinputfile --  parses the start file.
//
//	J.	Nagle
//	August, 2002
//
//	The startfile has lines which look like shell command lines, of the
//	form
//
//		ENVVAR=VAL ENVVAR=VAL progrname args...
//
//	Environment variables are passed to the program. Some of the
//	environment variables are also meaningful to the watchdog program
//
#include <stdio.h>
#include "watchdog.h"

//
//	parseinputfile -- parse a watchdog input file
//
int Watchdog::parseinputfile(const char filename[])
{	//	Open file for reading												
	FILE* in = fopen(filename,"r");									// open file name for reading
	if (!in)																		// if fail
	{	printf("Cannot open \"%s\" for reading.\n",filename); return(1);	}
	//	Read file, line by line, and parse
	int status = 0;															// no errors yet
	int lineno = 0;															// no lines yet
	for (;;)
	{	const size_t linemax = 512;									// max input line length
		char line[linemax];												// the input line
		char* s = fgets(line,sizeof(line),in);						// read a line
		if (!s) break;															// EOF or error
		lineno++;																// read another line
		printf("%4d.  %s",lineno,line);						 		// list input file
		CmdArgs args;														// prepare to read in args
		if (!args.parse(line,lineno)) status = 1;					// parse line, note errors
		if (m_verbose) args.dump();									// dump args if verbose
		if (args.valid())														// if valid arg set
		{	WatchedProgram* prg = new WatchedProgram(*this,args);		// add a new program object
			m_programs.push_back(prg);							// add to programs to be launched
			int stat = prg->init();											// initialize object further (look up path, etc.)
			if (stat) status = 1;											// prevent startup
		}
			
	}
	fflush(stdout);															// flush initial messages
	int stat = fclose(in);													// final close
	if (stat) 																	// if an I/O error occured
	{	perror("Error reading input file"); return(1); }		// fails
	return(status);															// return final status
}