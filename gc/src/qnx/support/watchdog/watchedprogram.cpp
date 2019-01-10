//
//	Watchdog  -- top-level watchdog program for QNX real-time applications
//
//	File parseinputfile --  parses the start file.
//
//	J.	Nagle
//	August, 2002
//
//
#include <stdio.h>
#include <errno.h>
#include <libgen.h>
#include <spawn.h>
#include <signal.h>
#include <unistd.h>
#include <assert.h>
#include <sys/netmgr.h>
#include "mutexlock.h"
#include "watchdog.h"
#include "spawnfix.h"
#include "procinfo.h"
#include "timeutil.h"
#include "logprint.h"
//
//	Constants
//
const size_t maxArgs = 64;												// max command line args
const size_t maxPath = 512;												// max path length
//
//	Utility functions - move to somewhere more useful
//
//	atoi  -- ascii to integer, with better checking
//
//	The entire string must be the number. Stuff after the number is an error.
//
bool atoi(const char* s, int& val)
{
    char* endp = 0;															// returned as end of string
    val = strtol(s,&endp,10);												// convert to integer
    return(endp && (endp[0] == '\0'));								// return true if successful conversion and reached end of string
}
//
//	itostring  -- integer to string conversion, decimal
//
string itostring(int n)
{
    char buf[512];
    snprintf(buf,sizeof(buf),"%d",n);
    return(buf);																	// this returns a string by value, not the address of a local buffer
}
//
//	varpart -- extract variable part of VAR=VAL
//
static string varpart(const char envexp[])
{
    string s;
    for (int i=0; envexp[i] != 0 && envexp[i] != '='; i++)	// continue up to '=' or end
    {	s += envexp[i];
    }													// save string up to '='
    return(s);																		// return string
}
//
//	class WatchedProgram implementation
//
//
//	initparams  -- called to initialize program parameters
//
int WatchedProgram::initparams()
{
    //	Set program ID
    const char* id = getenvvar("ID");									// get ID
    if (!id)
        id = getargs().front().c_str();								// if no ID specified, use program name
    m_id = id;																		// set ID
    //	Set node name
    const char* nodename = getenvvar("NODE");				// which node
    if (nodename)																// if a node name is given
    {	m_node = nodename;
    }										// set the node name
    //	Set priorities
    const char* pri = getenvvar("PRI");								// priority
    if (pri)																			// if got a priority
    {	int32_t prival;
        if (!atoi(pri,prival) || prival < 1 || prival > 61)			// bad priority
        {	logprintf("**** PRI=%s is not valid ****\n", pri);		// fails
            return(EINVAL);
        }
    }
    return(EOK);																	// success
}
//
//	prep  -- final setup before start
//
//	Server thread is running at this point, so we know its ID.
//
//	sets
//			WATCHDOG_ND=nodeid
//			WATCHDOG_PID=pid
//			WATCHDOG_COID=servercoid
//
int WatchedProgram::prep()
{
    char nodename[512];
    ost::MutexLock lok(m_lock);											// this program is busy
    int cnt =  netmgr_ndtostr(0,0, nodename,sizeof(nodename));	// get our node name
    if (cnt < 0)
    {
        perror("Unable to get own node name");
        return(errno);
    }			// unable to convert our node name
    addenvvar("WATCHDOG_ND",nodename);						// our node ID
    addenvvar("WATCHDOG_PID",itostring(::getpid()));			// the server's PID
    addenvvar("WATCHDOG_CHID",itostring(m_owner.getserverchid()));	// the server's channel
    //	Build environment variables in VAR=VALUE form needed for spawn.
    //	This duplicates existing data, but we do it once at startup to avoid allocation later during operation.
    for (map<string,string>::const_iterator p = getenvvars().begin(); p != getenvvars().end(); p++)
    {
        string s = p->first.c_str();											// VAR
        s += "=";																	// =
        s += p->second.c_str();											// value
        m_envparams.push_back(s);									// build list
    }
    return(EOK);																	// success
}
//
//	init -- called once after object creation to do misc. initialization
//
int WatchedProgram::init()
{
    ost::MutexLock lok(m_lock);											// this program is busy
    if (!valid())
        return(EINVAL);											// must be valid arg list
    //	Look up program to be executed.
    char buf[maxPath];														// scratch for path search
    const char* paths = getenv("PATH");							// path variable
    if (!paths)
        paths = "";													// if no path variable (might happen in embedded work)
    const char* progname = getargs().front().c_str();			// find program
    if ((progname[0] == '~') && (progname[1] == '/') && (getenv("HOME")))		// if begins with "~/"
    {	m_path = getenv("HOME");										// prepend home directory
        m_path += &progname[1];										// followed by path
    } else
    {																		// hormal case
        const char* progpath = pathfind_r(paths,progname,"r",buf,sizeof(buf));	// look up program in paths
        if (!progpath)																// if no find
        {	logprintf("**** Cannot find program \"%s\" on path \"%s\" ****\n",progname, paths);
            return(EINVAL);															// fails
        }
        m_path = progpath;													// success, set path
    }
    //	Add any environment variables not overridden for this program
    for (unsigned int i=0; environ[i] != 0; i++)					// for all environment variables
    {	string envvar( varpart(environ[i])); 							// get next environment variable
        if (getenvvars().find(envvar) != getenvvars().end())
            continue;		// ignore if overridden
        addenvvar(envvar,string(getenv(envvar.c_str())));	// not overridden, add to environment for this program
    }
    int stat = initparams();													// do next stage of initialization
    if (stat)
        return(stat);														// if fail
    return(EOK);																	// success
}
//
//	start  --  start the monitor thread for this program
//
//	The monitor thread will launch the program.
//
int WatchedProgram::start()
{
    return(pthread_create(&m_monitorthread,0,startmonitor,this));	// start the monitor thread
}
//
//	getnodeid  -- get node ID on which program is to run
//
//	Note that node IDs can change after a network reset. So we always look them up from the name.
//
int WatchedProgram::getnodeid() const
{
    if (m_node.size() == 0)
        return(0);									// is local node if unspecified
    return(netmgr_strtond(m_node.c_str(),0));					// look up node descriptor
}
//
//	Don't allocate memory (use new, or expand collections) after initialization in this class.
//
//
//	launch -- launch the program
//
//	Sets up the environment and spawns the program
//
int WatchedProgram::launch()
{
    ost::MutexLock lok(m_lock);											// this program is busy
    //	Find target node on which process is to run
    int nd = getnodeid();													// get target node ID
    if (nd < 0)																		// if no such node
    {	assert(errno);
        return(errno);
    }									// launch failed. Monitor thread should retry
    //	Build argv. This is a temporary array of pointers to C strings
    size_t argc = 0;																// no args yet
    const char* argv[maxArgs];											// argv built here
    for (vector<string>::const_iterator p = getargs().begin(); p != getargs().end(); p++)
    {
        if (argc >= maxArgs-1)
            return(EINVAL);					// too many args
        argv[argc++] = p->c_str();										// build args
    }
    argv[argc] = 0;																// terminate list
    //	Build environment variables. This is a temporary array of pointers to C strings.
    size_t envc = 0;															// no evironment yet
    const char* envp[maxArgs];										// environment built here
    for (vector<string>::const_iterator p = m_envparams.begin();
            p != m_envparams.end();
            p++)
    {
        if (argc >= maxArgs-1)
            return(EINVAL);					// too many args
        envp[envc++] = p->c_str();
    }
    envp[envc] = 0;															// finish list
    //	Create the logging pipe.  The child process writes to this as standard output or standard error,
    //	and the logging thread combines and tags all the incoming lines, then sends them to the log.
    int fildes[2];																	// pipe ends (read, write)
    int stat = pipe(fildes);
    if (stat)
        return(errno);													// if pipe fails, fail
    assert(m_logpipe < 0);													// must not have a pipe in use
    //	Build inherited file descriptor map
    int fd_count = 0;
    int fd_map[maxArgs];													// the inherited file descriptors
    fd_map[STDIN_FILENO] = -1;											// no stdin, not too sure about this
    fd_map[STDOUT_FILENO] = fildes[1];							// child's standard output and standard error
    fd_map[STDERR_FILENO] = fildes[1];								// both go to the log pipe
    fd_count =STDERR_FILENO+1;										// count of file descriptors
    //	Build inheritance information
    struct inheritance inheirit;
    //	We can't use "wait" to wait for a PID to exit for processes on remote nodes.
    //	They're children of io-net, not us. So we don't let them become zombies if we crash.
    //	This is really for debugging, so users can kill the watchdog.
    inheirit.flags = SPAWN_NOZOMBIE;								// don't wait for parent to read status.
    //khianhao
    //we wont be spawning directly, we would always spawn using on -f
    //if (nd) inheirit.flags |= SPAWN_SETND;							// if node specified, launch on that node
    inheirit.pgroup = 0;														// no special group
    ////inheirit.sigmask = 0;													// no signal mask
    ////inheirit.sigdefault = 0;												// no default signals
    inheirit.nd = nd;															// set remote node

    //khianhao
    //make it be able to run scripts
    inheirit.flags |= SPAWN_CHECK_SCRIPT;
    //	Spawn the new program.
    pid_t pid = spawn(m_path.c_str(), fd_count, fd_map, &inheirit,argv, envp);
    if (pid < 0)																	// if spawn failed
    {	close(fildes[0]);
        close(fildes[1]);
        m_logpipe = -1;
        logprintf("Launch of \"%s\" failed: %s\n", m_path.c_str(), strerror(errno));
        return(errno);															// failed
    }
    m_launchedpid = pid;													// success, save pid
    m_logpipe = fildes[0];													// save log pipe for log thread
    close(fildes[1]);															// close write end of pipe; child keeps it open
    return(EOK);																	// success
}
//
//	kill -- send indicated signal to program, even on another node
//
int WatchedProgram::kill(int sig)
{
    ost::MutexLock lok(m_lock);											// this program is busy
    pid_t pid = getserverpid();											// launch server-registered PID if possible
    if (pid < 0)
    {
        pid = getlaunchedpid();
    }										// launched PID if not registered properly
    if (pid <= 0)
    {
        logprintf("Cannot kill \"%s\" - not running.\n", getid().c_str());
        return(ESRCH);															// no such process
    }
    int nd = getnodeid();													// get node ID for process
    if (nd < 0)
        return(EHOSTUNREACH);								// could not reach host
    int stat = SignalKill(nd,pid,0,sig,0,0);		// try to kill
    if (stat < 0)
    {
        logprintf("Unable to kill process %d on node %d\n", pid, nd);
        return(errno);														// failed
    }
    return(EOK);
}
//
//	abortpgm  -- abort this program
//
int WatchedProgram::abortpgm()
{
    return(kill(SIGKILL));														// kill this program
}
//
//	exitjoin  -- join monitoring thread
//
int WatchedProgram::exitjoin()
{
    pid_t threadid = -1;														// our monitoring thread
    {	ost::MutexLock lok(m_lock);										// this program is busy
        threadid = m_monitorthread;									// don't lock during join wait
    }
    if (threadid < 0)
        return(0);											// no thread to wait for
    struct timespec deadline;
    const uint64_t k_monitor_join_wait_ns = 1000000000*2;	// 2 seconds
    nsec2timespec( &deadline, gettimenowns() + k_monitor_join_wait_ns);
    int stat = pthread_timedjoin(threadid, 0, &deadline);	// join thread, but don't wait forever
    if (stat < 0 && errno != ESRCH)									// trouble
    {	perror("Monitoring thread shutdown failed");			// must abort, already in a panic
        fflush(stderr);

        usleep(100000);														// wait for output
        abort();
    }
    return(0);																		// success
}
//
//	statemonitor --  monitor and change program state
//
void WatchedProgram::statemonitor()								// monitor and update program state
{	ost::MutexLock lok(m_lock);											// lock state
    switch (m_state)
    {														// fan out on state
    case unstartedPgm:														// program has not started
        {	int stat = launch();
            if (stat == 0)
            {
                logprintf("Launched %s successfully.\n",m_id.c_str());	// successful launch
                m_state = runningPgm;											// program now running
            }
            else																			// failure
            {
                logprintf("FAILED to launch %s (%s):  %s\n",m_id.c_str(),getpath().c_str(),strerror(stat));
                watchdogfail("Unable to launch program");			// trouble
                m_state = exitedPgm;											// handle as exited
            }
            // flush any messages
            break;
        }
        //	***STATE MACHINE NEEDS WORK***
    case runningPgm:															// running, normal state, do nothing
        break;

    case killedPgm:																// log pipe broken - killed or dying, but not yet flushed from system
        {	pid_t parent, child, sibling;
            int stat = getprocinfo(m_node.c_str(), m_launchedpid, parent, child, sibling);	// get process state of process
            switch (stat)
            {															// fan out on process request status
            case ENOENT:															// no such file or directory - process has exited
                logprintf("[%s] *** Program has exited. ***\n",m_id.c_str());	// probably because it exited
                m_state = exitedPgm;											// state changes to exited program
                watchdogfail("Program has exited");					// ***TEMP*** always forces reboot
                break;

                //	***MORE*** need to detect network trouble and handle it

            default:
                logprintf("[%s] Unable to get process status: %s\n",m_id.c_str(), strerror(stat));	// probably network trouble
                watchdogfail("Unable to get process status");
            }
            break;
        }

    case exitedPgm:															// exited, final state.
        watchdogfail("Program has exited");						// ***TEMP*** always forces reboot
        //	***MORE***
        break;

    default:
        break;
    }
}
//
//	monitor -- the monitoring thread startup for this program
//
//	This provides a try block, so we can shut down without leaving locks set.
//
void* WatchedProgram::monitorstart()
{

    try
    {
        monitor();																	// run the monitor
    }
    catch (const char* msg)												// during abort, other threads throw this
    {
        logprintf("Monitor thread %d for \"%s\" exception: %s\n", pthread_self(), getid().c_str(),msg);
    }
    return(0);																		// exit thread
}
//
//	monitor -- the monitoring thread for this program
//
void WatchedProgram::monitor()
{
    string s;
    const size_t maxLineLength = 256;								// truncate lines longer than this
    s.reserve(maxLineLength);											// preallocate
    for (;;)																			// forever
    {	if (m_owner.aborting())
        throw("Abort already in progress");	// done if parent is aborting
        statemonitor();															// do update
        int fd = m_logpipe;													// logging pipe
        if (fd < 0)																	// if no logging pipe
        {	////logprintf("[%s] No logging pipe!\n",m_id.c_str());		// print debug msg
            sleep(1);
            continue;
        }
        //	Read from pipe and log.  Multiple threads write to the output, so we do each output
        //	write as an atomic operation, a single "write" call.
        //	Blank lines are not logged. This allows programs to write a blank line once in a while to keep things alive.
        //	If the watchdog exits, and a program writes to the log, the program will get a SIGPIPE signal and abort.
        //	So a program can't outlive the watchdog by much, as long as it writes something occasionally.
        char buf[256];															// max line length
        int cnt = read(fd,buf,sizeof(buf));									// read from pipe
        if (cnt <= 0)																// program has exited
        {	close(m_logpipe);													// close logging pipe
            logprintf("[%s] End of file or error on logging pipe.\n",m_id.c_str());	// probably because it exited
            m_logpipe = -1;													// trouble
            m_state = killedPgm;												// program has exited, state monitor must clean up
            continue;																// will now get "no logging pipe" message
        }
        for (int i=0; i<cnt; i++)												// for all in string
        {	char ch = buf[i];													// next char read
            if (s.size() == 0)													// if beginning a new msg
            {	if (ch == '\n' || ch == '\0')
                continue;					// if empty message, do not log.
                timestamp(s);													// add the time stamp
            }
            if (ch == '\n')														// if end of line
            {	s += '\n'; 															// add final newline
                write(STDOUT_FILENO,s.c_str(),s.size());			// write as an atomic operation, for clean logs
                s.clear();															// clear string
                continue;															// next line
            }
            if (s.size() < maxLineLength-2)
                s += ch;				// append if not full, otherwise truncate
        }
    }
}
//
//	watchdogfail --  a failure has occured
//
void WatchedProgram::watchdogfail(const char* msg)
{
    logprintf("[%s] WATCHDOG FAIL: %s\n",m_id.c_str(),msg);	// ***TEMP***
    m_owner.panic("ABORTING");										// ***TEMP*** reboot everything
}
//
//	timestamp  --  timestamp and mark a log entry
//
//	Output looks like "14:03:44 [progname] "
//
void WatchedProgram::timestamp(string& s)
{
    char timestr[100];															// message assembly
    time_t nowt = time(0);													// time now
    struct tm now;																// time now, structure format
    localtime_r(&nowt,&now);												// format conversion
    strftime(timestr,sizeof(timestr),"%H:%M:%S [",&now);	// format as HH:MM:SS
    s += timestr;
    s += m_id;
    s += "] ";
}
//
//	getmsgportid  -- return a MsgPortID for the connection, if any
//
//	We have to build this; node IDs can change.
//
MsgPortID WatchedProgram::getmsgportid() const
{
    MsgPortID portid;
    portid.m_pid = getserverpid();
    portid.m_nodeid = getnodeid();
    portid.m_chid = getchid();
    return(portid);
}

