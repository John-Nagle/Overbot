//
//	cmdargs.h  --  parse a line into command-line arguments
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
#ifndef CMDARGS_H
#define CMDARGS_H
//
#include <vector>
#include <string>
#include <map>
using namespace std;

//
//	class CmdArgs -- a set of command args
//
//	Stores environment variables and program arguments
//
class CmdArgs {
private:
	vector<string> m_args;								// program arguments
	map<string,string> m_envvars;					// program environment variables, as a map
private:
	int parsefield(string& s, const char line[], unsigned int& pos, int lineno, 
		bool allowLeadingWhitespace, bool AllowQuote);
	int parsearg(string& s, unsigned int& pos, const char line[], int lineno);
	int parseenvvar(string& s1, string& s2, unsigned int& pos, const char line[], int lineno);
protected:
	void addenvvar(const string& s1, const string& s2) 
	{	m_envvars[s1] = s2; }
public:
	CmdArgs() {}												// empty constructor
	CmdArgs(const CmdArgs& args) :					// copy constructor
		m_args(args.m_args), m_envvars(args.m_envvars)
		{}
	const vector<string>& getargs() const { return(m_args);}	// get args as constant
	const int getargc() const { return(m_args.size());	}// arg count
	const map<string,string>& getenvvars() const { return(m_envvars); }
	const char* getenvvar(const char* key);		// get value of indicated env. var
	bool parse(const char line[], int lineno);		// parse this line, return true if success
	bool valid() const { return(getargc() > 0); }// must have args
	virtual void dump();										// dump to standard output
	virtual void printerrmsg(const char msg[], const char line[], int lineno);	// report an error
	virtual ~CmdArgs() {}									// destructor
};
#endif // CMDARGS_H