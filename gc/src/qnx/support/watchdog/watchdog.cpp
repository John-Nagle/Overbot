//
//	Watchdog  -- top-level watchdog program for QNX real-time applications
//
//	J.	Nagle
//	August, 2002
//
//	The watchdog program is started at boot time, and starts
//	all the other programs.  It monitors their performance and
//	can restart them if necessary.
//
//
//	Usage: watchdog [options] startfile
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
#include <time.h>
#include <unistd.h>
#include "watchdog.h"
#include "procinfo.h"
#include "logprint.h"
//
//	The master watchdog object
//
static Watchdog watch;													// Watchdog object - a singleton
//
//	usage -- print usage message and exit
//
static void usage()
{
    logprintf("Usage: watchdog [-v] watchdogfile\n");			// print message
    exit(1);
}
//
//	Main program
//
int main(int argc, const char* argv[])
{	//	Parse input arguments
    const char* filenamearg = 0;									// no file argument yet
    for (int i=1; i<argc; i++)											// for all args
    {	const char* arg= argv[i];										// this arg
        if (arg[0] == '-')													// if flag argument
        {	switch(arg[1])
            {												// interpret flags
            case 'v':
                watch.setverbose(true);
                break;		// set verbose mode
            default:
                usage();												// bad call, fails
            }
            continue;															// next arg
        }
        //	Not flag, must be file arg
        if (filenamearg)
            usage();										// if already have one, fails
        filenamearg = arg;												// save filename arg
    }
    if (!filenamearg)
        usage();											// if no filename arg, fails
    //	Parse input file
    int stat = watch.parseinputfile(filenamearg);			// parse the input file
    if (stat)																		// if fail
    {	logprintf("Error in watchdog file \"%s\" - cannot start real time system.\n",filenamearg);
        exit(1);
    }
    //	Parsing successful - begin real time mode
    stat = watch.run();													// start up run threads
    // flush any outstanding messages from startup
    watch.signals();														// enable signal handling
    pthread_exit(0);														// the initial thread exits
}
//
//	Class watchdog
//
//
//	hiwatchdog  -- the high-priority watchdog thread
//
//	Must be the highest priority thread in the system.
//	Just makes sure the low-priority watchdog thread hasn't been starved out by some CPU-bound thread.
//
void* Watchdog::hiwatchdog()
{
    try
    {
        const uint32_t maxMissedResets = 5;					// max allowed misses
        uint32_t missedResets = 0;									// tally misses
        for (;;)
        {
            nanosleep(&m_hiwatchinterval,0);					// sleep for requested time
            if (m_lowwatchreset)										// if low-priority thread has checked in
            {	m_lowwatchreset = false;								// note checkin
                missedResets = 0;										// reset missed resets
                continue;														// wait for next time
            }
            if (missedResets++ >= maxMissedResets)
                break;	// if missed enough resets, shut down
        }
        //	Trouble - low-priority thread has not checked in.
        watchdogfail("A CPU-bound thread has stalled the real-time system",0);	// shut everything down
    }
    catch (const char* msg)												// during abort, other threads throw this
    {
        logprintf("High priority watchdog thread exception: %s\n", msg);
    }
    return(0);																		// exit thread

    return(0);																// thread exits
}
//
//	lowatchdog  -- the low-priority watchdog thread
//
//	Must be lower priority than all the other real-time threads.  Set to one below the priority
//	of the lowest priority thread with a specified priority.
//
//	Checks all the other tasks for watchdog check-ins.
//
void* Watchdog::lowatchdog()
{
    try
    {
        for (;;)
        {
            nanosleep(&m_lowatchinterval,0);					// sleep for requested time
            m_lowwatchreset = true;									// tell high-priority thread we are OK
            //	***MORE*** check for watchdog checkins from apps
        }
    }
    catch (const char* msg)												// during abort, other threads throw this
    {
        logprintf("Low priority watchdog thread exception: %s\n", msg);
    }
    return(0);																		// exit thread

}
//
//	watchdogfail -- trouble -- global watchdog failure
//
//	We don't know who failed, and must shut down everything.
//
void Watchdog::watchdogfail(const char* msg, WatchedProgram* failedpgm)
{
    if (failedpgm)
        logprintf("****** Watchdog failure: %s in  %s ******\n",msg,failedpgm->getid().c_str());
    panic(msg);																// just panic
}
//
//	findbyid -- find program by named ID
//
//	Used only at startup, or if program restarts
//
//	Search key is string, with or without trailing null.
//
WatchedProgram* Watchdog::findbyid(const char name[], size_t namelen)
{
    ost::MutexLock lok(m_lock);										// lock
    for (vector<WatchedProgram*>::iterator p = m_programs.begin(); p != m_programs.end(); p++)
    {
        WatchedProgram* prg = *p;									// the program
        assert(prg);
        if (namelen < prg->getid().size())
            continue;			// if too short to match, must be unequal
        if (strncmp(name,prg->getid().c_str(),namelen) == 0)
            return(prg);	// if equal, find
    }
    return(0);																	// no find
}
//
//	findbynodeandpid --  find program by message info
//
//	Uses the data about the sender of a message to find who sent it.
//
//	BUG: can't find a program that's on a different machine than the watchdog, is
//	launched via "on", and isn't a server. We don't know its PID.
//
WatchedProgram* Watchdog::findbynodeandpid(pid_t nd, int pid)
{
    ost::MutexLock lok(m_lock);										// lock
    for (vector<WatchedProgram*>::iterator p = m_programs.begin(); p != m_programs.end(); p++)
    {
        WatchedProgram* prg = *p;									// the program
        assert(prg);
        if (ND_NODE_CMP(nd, prg->getnodeid()) != 0)
            continue;		// skip if no node match
        if (pid == prg->getlaunchedpid())
            return(prg);		// node and PID match - find
        if (pid == prg->getserverpid())
            return(prg);			// node and server PID match - find
    }
    return(0);																	// no find
}
//
//	findbymsginfo --  find program by message info
//
//	Uses the data about the sender of a message to find who sent it.
//
WatchedProgram* Watchdog::findbymsginfo(const struct _msg_info& msginfo)
{
    ost::MutexLock lok(m_lock);										// lock
    WatchedProgram*prg = findbynodeandpid(msginfo.nd, msginfo.pid);	// search for sending PID
    if (prg)
        return(prg);													// find
    //	No find for exact PID. Try children, which is slow. We do this so servers can be
    //	run as children of a pterm, allowing user I/O.
    char nodename[512] = "\0";										// name of node
    if (msginfo.nd != 0)													// if not this node
    {	int cnt = netmgr_ndtostr(0,msginfo.nd,nodename,sizeof(nodename));
        if (cnt < 0)															// returns length of string
        {	perror("Findbymsginfo: node lookup unsuccessful");	// they just called us, but they're gone
            return(0);															// fails
        }
    }
    //	Search upwards through parents. Stop at 0, no change, or reached watchdog
    for (pid_t pid = msginfo.pid, parent = -1; ;)
    {
        pid_t child, sibling;													// work upwards
        int stat = getprocinfo(nodename,pid,parent,child,sibling);
        if (stat) 																// if fail
        {	perror("Findbymsginfo: getprocinfo lookup unsuccessful");
            return(0);
        }
        logprintf("Parent of pid %d is %d\n",pid,parent);		// ***TEMP***
        prg = findbynodeandpid(msginfo.nd,parent);		// try parent
        if (prg)
            return(prg);												// find, continue
        if (pid < 0)
            break;													// invalid PID
        if (pid == 0)
            break;												// reached root
        if (pid == parent)
            break;										// fails, we are in a loop
        pid = parent;														// continue searching from parent
    }

    return(0);																	// no find
}
//
//	panic -- total failure
//
void Watchdog::panic(const char msg[])
{
    bool alreadyaborting = setabort();							// begin to abort system
    if (alreadyaborting)													// if abort already in progress
    {	throw(msg);
    }														// unwind and exit
    logprintf("****** PANIC: %s ******\n",msg);
    ost::MutexLock lok(m_lock);										// lock
    //	Examine all programs and start shutting them down
    for (vector<WatchedProgram*>::iterator p = m_programs.begin(); p != m_programs.end(); p++)
    {
        WatchedProgram* prg = *p;									// the program
        assert(prg);
        int stat = prg->abortpgm();									// abort this program
        if (stat)
        {
            logprintf("Unable to abort program \"%s\": %s\n", prg->getid().c_str(), strerror(errno));
        }
    }
    //	Join with each monitoring thread
    for (vector<WatchedProgram*>::iterator p = m_programs.begin(); p != m_programs.end(); p++)
    {
        WatchedProgram* prg = *p;									// the program
        assert(prg);
        prg->exitjoin();														// join with thread, or die
    }
    //	Join with server thread, lo and hi threads.
    {	int coid = ConnectAttach(0, 0, m_serverchid, _NTO_SIDE_CHANNEL, 0);	// connect to own server thread
    	if (coid < 0)
    	{	logprintf("Unable to attach to own server thread.\n");	}
    	else
    	{	MsgSendPulse(coid, defaultPriority, 0, 0);				// send pulse to wake up server
    		ConnectDetach(coid);											// close connection
      		pthread_join(m_serverthread, 0);							// wait for server thread to exit
  		}
    }
    logprintf("Shutdown complete.\n");
    exit(1);																		// finally exit
}
//
//	signals  -- this thread handles all signals
//
void* Watchdog::signals()
{	//	Signal setup - capture control-C
    try
    {
        sigset_t sigmask;													// signals to block during signal handler
        sigemptyset(&sigmask);										// no additional signals to block
        struct sigaction action;											// action to take on signal
        action.sa_handler = 0;
        action.sa_sigaction = signalhandler;						// set our signal handler
        action.sa_mask = sigmask;									// set signals to be blocked during signal handler
        action.sa_flags = 0;
        int stat = sigaction(SIGINT,&action, 0);					// set signal handler for control-C
        if (stat)
        {
            perror("Unable to set signal handler");
            panic("Signal thread initialization failed");
        }
        //	Thread setup
        //	***NEED TO ALLOW WANTED SIGNALS***
        while (!aborting() && !m_rcvdsignal)
        {
            usleep(100000);
        }	// wait for a signal
        panic("Signal received");										// process signal, start shutdown
    }
    catch (const char* msg)										// during abort, other threads throw this
    {
        logprintf("Signal thread exception: %s\n", msg);
    }
    return(0);																// exit during abort
}
//
//	signalhandler -- the actual signal handler (static)
//
void Watchdog::signalhandler(int signo, siginfo_t *info, void *other)
{
    logprintf("*** CAUGHT SIGNAL %d ***\n",signo);
    watch.setrcvdsignal();										// note signal received
}
