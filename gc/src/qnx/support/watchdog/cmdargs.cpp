//
//	cmdargs.cpp  --  parse a line into command-line arguments
//
//	Parses lines of the form 
//
//		ENVVAR1=val1 ENVVAR2="val2"  programname arg arg "arg with spaces" ...
//
//	J.	Nagle
//	Team Overbot
//	August, 2002
//
//	License: LGPL
//
#include <stdio.h>
#include "cmdargs.h"

//
//	parsefield -- get next field
//
//	"pos" is updated, and points to the first character after the field on return
//
//	Returns: 0=success, 1=EOF, -1=error
//
int CmdArgs::parsefield(string& s, const char line[], unsigned int& pos, int lineno, 
	bool allowLeadingWhitespace, bool AllowQuote)
{
	if (allowLeadingWhitespace)											// if need to skip whitespace
	{	for (; line[pos] != '\0' && !isgraph(line[pos]); pos++) {}};	// skip whitespace
	bool inQuote = false;													// not in a quote
	bool inEscape = false;													// if in an escape sequence
	for (; line[pos] != '\0'; pos++)										// advance through field
	{	char ch = line[pos];													// get next char
		if (ch == '\0') break;													// end of string, backup check
		if (ch == '\n') break;													// end of line
		if (!inEscape)															// if not handling an escape
		{	if (ch == '"')															// if quote
			{	inQuote = !inQuote; continue; }						// handle quote
			if (ch == '\\')															// if backslash escape
			{	if (!inQuote)														// if backslash outside quote
				{	printerrmsg("Unexpected \"\\\"",line,lineno);	// error
					return(-1);														// fails
				}
				inEscape = true;												// in an escape
				continue;
			}
		}
		if (!inQuote)																// if outside a quote, more tests
		{	if (ch == '#') break;												// comment delimiter
			if (ch == '=') break;												// argument separator
			if (!isgraph(ch)) break;											// stop on whitespace
			switch (ch) {														// check for limited set of chars allowed outside quotes
				case '-': 
				case '+': 
				case '_': 
				case '$':
				case '/':
				case '\\':
				case '.':
				case '~':
					 break;
				default: 
					if (isalnum(ch)) break;									// allow all alphanumerics
					char msg[100];												// handle illegal character
					snprintf(msg,sizeof(msg),"Unexpected character: \"%c\"",ch);
					printerrmsg(msg,line,lineno);						// diagnose
					return(-1);														// fails
			}	
		}
		s += ch;																	// add to string
		inEscape = false;														// not in an escape now
	}
	if (inQuote)
	{	printerrmsg("Unterminated quote", line, lineno);	return(-1); }
	////printf("FIELD: %s\n",s.c_str());										// ***TEMP DEBUG***
	if (s.size() == 0) return(1);											// EOF if empty symbol
	return(0);																		// normal																	
}
//
//	parseenvvar -- get an environment variable from the input line
//
//	Looking for 
//			VARNAME=VALUE
//			VARNAME="value with spaces"
//
int CmdArgs::parseenvvar(string& s1, string& s2, unsigned int& pos, const char line[], int lineno)
{	unsigned int startpos = pos;
	int stat = parsefield(s1,line,pos,lineno,true,false);				// scan for first part of A=B, not allowing quotes
	if (stat != 0)	{	pos = startpos; return(stat);	}				// if not good, back up and fail
	if (line[pos] != '=') { pos = startpos; return(1); }			// if does not end with equal, fails
	pos++;																			// advance past equal
	stat = parsefield(s2,line,pos,lineno,false,true);					// scan second field, allowing quotes and punctuation
	if (stat != 0)	{	pos = startpos; return(stat);	}				// if not good, back up and fail
	return(0);																		// success
}
//
//	parsearg -- get a command line arg from the input line
//
//	Looking for
//			arg
//			"arg with spaces"
//
int CmdArgs::parsearg(string& s, unsigned int& pos, const char line[], int lineno)
{
	return(parsefield(s,line,pos,lineno,true,true));					// get this field, allow quoting and lead spaces
}
//
//	parse -- parse an input line
//
bool CmdArgs::parse(const char line[], int lineno)
{
	unsigned int pos = 0;
	//	Get environment variables
	for (;;)
	{	string s1,s2;
		int stat = parseenvvar(s1,s2,pos,line,lineno);
		if (stat> 0) break;												// EOF
		if (stat < 0) return(false);									// error
		if (m_envvars.find(s1) != m_envvars.end())		// if duplicate
		{	printerrmsg(line,"The same environment variable appears more than once",lineno);
			return(false);												// fails
		}
		m_envvars[s1] = s2;										// save environment variable
	}
	//	Get program arguments
	for (;;)
	{	string s;	
		int stat = parsearg(s,pos,line,lineno);
		if (stat > 0) break;											// EOF
		if (stat < 0) return(false);									// error
		m_args.push_back(s);										// add arg
	}
	return(true);															// success	
}
//
//	printerrmsg  -- print error message for bad line
//
//	Default version - subclass can override
//
void CmdArgs::printerrmsg(const char line[], const char msg[], int lineno)
{	printf("%4d.  %s\n",lineno,line);									// line in error
	printf("       **** %s ****\n",msg);								// relevant message
}
//
//	getenvvar -- get a specified environment variable for this object
//
const char* CmdArgs::getenvvar(const char* key)
{
	if (!key) return(0);														// no key, fails
	map<string,string>::const_iterator p = getenvvars().find(key);		// look up
	if (p == getenvvars().end()) return(0);						// no find, fails
	return(p->second.c_str());										// find, succeeds
}

//
//	dump -- dump for debug
//
void CmdArgs::dump()
{	//	Dump environment variables
	printf("CmdArgs: ");
	for (map<string,string>::iterator p = m_envvars.begin(); p != m_envvars.end(); p++)
	{	printf("%s=\"%s\" ",	p->first.c_str(), p->second.c_str());	}
	//	Dump args
	for (vector<string>::iterator p = m_args.begin(); p != m_args.end(); p++)
	{	printf("\"%s\" ",	p->c_str()); 	}
	printf("\n");
}
