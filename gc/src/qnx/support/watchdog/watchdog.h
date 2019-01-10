//
//	Watchdog  -- top-level watchdog program for QNX real-time applications
//
//	J.	Nagle
//	August, 2002
//
#ifndef WATCHDOG_H
#define WATCHDOG_H
//
#include <vector>
#include <string>
#include <map>
#include <stdint.h>
#include <pthread.h>
#include <atomic.h>
#include "cmdargs.h"
#include "mutexlock.h"
#include "messaging.h"

const int32_t defaultPriority = 10;						// standard QNX background priority
const struct timespec zerotime = {0, 0};			// zero time value
const struct timespec defaulttimeout = {1,0};	// one second
//
//	State of a program
//
enum PgmState { unstartedPgm, runningPgm, killedPgm, exitedPgm };
class Watchdog;
//
//	WatchedProgram -- a single program to be run
//
class WatchedProgram: public CmdArgs {
private:
	Watchdog& m_owner;									// owns this object
	//	Properties of the program, set once at startup
	string m_path;												// full path to program file to launch
	vector<string> m_envparams;						// environment variables in the form NAME=VALUE
	string m_id;													// name of program for messaging, etc.
	string m_node;												// node name on which to launch program
	int32_t	m_priority;											// priority of this process at start
	int32_t m_maxpriority;									// max priority of this process
	size_t m_maxmemory;									// maximum allowed memory
	struct timespec m_initwatch;							// initial watchdog interval
	struct timespec m_watch;								// run watchdog interval
	//	Properties of the running process
	ost::Mutex m_lock;											// process state is being updated 
	PgmState m_state;										// state of the program
	pid_t	m_launchedpid;									// process ID of launched program (parent of tree)
	pid_t m_serverpid;										// process ID of program running server
	int m_chid;													// message channel ID
	int	 m_logpipe;												// logging pipe for this process
	struct timespec m_nextwatch;						// must check in by this time or die
	//	The monitoring thread for this process
	pthread_t m_monitorthread;							// monitoring thread PID
private:
	int initparams();											// initialize program parameters (priority, etc.)
	void statemonitor();										// state monitoring 
	void* monitorstart();										// the monitoring thread
	void monitor();												// the monitoring thread
	static void* startmonitor(void* threaddata) { return(reinterpret_cast<WatchedProgram*>(threaddata)->monitorstart()); }
	void	timestamp(string& s);							// add timestamp
	int launch();													// launch this program
	int getlogpipe() const { return(m_logpipe); }// get the log pipe file descriptor, or -1
	int initserverid();											// initialize server ID, must happen after server starts
public:
	WatchedProgram(Watchdog& owner, const CmdArgs& args)
		: CmdArgs(args), m_owner(owner), m_priority(defaultPriority), m_maxpriority(defaultPriority), 
		m_maxmemory(0), m_initwatch(zerotime), m_watch(zerotime), 
		m_state(unstartedPgm),m_launchedpid(-1),m_serverpid(-1),m_chid(-1),m_logpipe(-1), m_nextwatch(zerotime),
		////m_portid(emptyMsgPortID),
		m_monitorthread(-1) {}
	const string& getpath() const { return(m_path); }	// launch path				
	int init();														// post-construction initialization
	int prep();														// last initialization before start
	int start();														// start this program
	int kill(int sig);												// send signal to program
	int abortpgm();												// abort this program (called from outside thread)
	int exitjoin();													// join with monitoring thread.  If returns OK, safe to delete object
	void watchdogfail(const char* msg);			// trouble - a watchdog check has failed
	MsgPortID getmsgportid() const;
	////void setmsgportid(const MsgPortID& portid) { m_portid = portid; }
	void setserverpidandchid(pid_t pid, int chid) { m_serverpid = pid; m_chid = chid; }		// set server pid and channel ID
	int getnodeid() const;									// return node ID
	pid_t getlaunchedpid() const { return(m_launchedpid); }
	pid_t getserverpid() const { return(m_serverpid); }			// return PID
	int getchid() const { return(m_chid); }			// return channel ID
	const string& getid() const { return(m_id); }// return name
};
//
//	Watchdog  --  main watchdog object
//
//	A singleton
//
class Watchdog {
private:
	ost::Mutex m_lock;											// process state is being updated 
	bool m_verbose;											// verbosity
	bool m_debug;												// true if debug mode (software watchdog, no reboots)
	vector<WatchedProgram*> m_programs;	// the programs
	uint32_t m_minpriority;									// lowest priority thread, all programs
	uint32_t m_maxpriority;									// highest priority thread, all programs
	pthread_t m_hiwatchthread;							// high priority watchdog thread
	pthread_t m_lowatchthread;							// low priority watchdog thread
	pthread_t m_serverthread;							// server thread
	int m_serverchid;											// server channel
	struct timespec m_hiwatchinterval;				// run watchdog interval
	struct timespec m_lowatchinterval;				// run watchdog interval
	volatile bool m_lowwatchreset;						// set to true by low watchdog
	volatile unsigned m_aborting;						// abort flag set, kill all programs and exit
	volatile bool	m_rcvdsignal;							// received signal, must shut down
private:
	int printparseerror(const char line[], int lineno, int charpos, const char* msg);
	WatchedProgram* findbyid(const char* id, size_t idlen);	// look up program by ID
	WatchedProgram* findbymsginfo(const struct _msg_info& msginfo); 	// look up program by sender
	WatchedProgram* findbynodeandpid(pid_t pid, int nd);	// look up by pid and node descriptor	
public:
	Watchdog()													// constructor
	: m_verbose(false), m_debug(false), m_minpriority(defaultPriority), m_maxpriority(defaultPriority), 
	m_hiwatchinterval(defaulttimeout), m_lowatchinterval(defaulttimeout),
	m_lowwatchreset(true), m_aborting(0), m_rcvdsignal(false) {}							// set to true
	void setverbose(bool verbose) { m_verbose = verbose; }
	void setdebug(bool debug) { m_debug = debug; }
	void setrcvdsignal() { m_rcvdsignal = true; }
	int parseinputfile(const char filename[]);		// parse the input file
	int run();														// does all the work
	int getserverchid() const { return(m_serverchid); } // get server channel ID
	////ost::Mutex& getlaunchlock() { return(m_launch_lock); } // get launch lock
	void watchdogfail(const char msg[], WatchedProgram* failedpgm);		// trouble - a watchdog check has failed
	void panic(const char msg[]);						// big trouble - will lead to an e-stop and reboot
	bool verbose() const { return(m_verbose);}// if debug mode
	bool aborting() const { return(m_aborting != 0);	} // true if abort in progress
	//	Called from signals
	bool setabort() { return(atomic_set_value(&m_aborting,1) != 0); }					// set aborting flag
	//	The threads
	void* hiwatchdog();										// high-priority watchdog thread
	void* lowatchdog();										// low-priority watchdog thread
	void* server();												// server for finding programs by name
	void* signals();												// signal handler
	//	Server message functions
	int serverOK(WatchdogMsgOK& msg, const struct _msg_info& msginfo);			// handle msg
	int serverID(WatchdogMsgID& msg, const struct _msg_info& msginfo);				// handle msg
	int serverSV(WatchdogMsgSV& msg, const struct _msg_info& msginfo);			// handle msg
	//	Exception handling for threads
	//	Thread start functions, for use with pthread init function
	static void* starthiwatchdog(void* threaddata) { return(reinterpret_cast<Watchdog*>(threaddata)->hiwatchdog()); }
	static void* startlowatchdog(void* threaddata) { return(reinterpret_cast<Watchdog*>(threaddata)->lowatchdog()); }
	static void* startserver(void* threaddata) { return(reinterpret_cast<Watchdog*>(threaddata)->server()); }
	static void* startsignals(void* threaddata) { return(reinterpret_cast<Watchdog*>(threaddata)->signals()); }
	//	Signal handler
	static void signalhandler(int signo, siginfo_t *info, void *other);
};

#endif // WATCHDOG_H