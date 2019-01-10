//
//	procinfo -- get information about a process under QNX
//
//	Under QNX, this is done by opening the appropriate file in the /proc subsystem
//	and applying a devctl function to it.
//
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/procfs.h>
#include "procinfo.h"
//
//	getprocinfo -- get information for a given process id on a given node
//
//	This is slow. Don't do it much.
//
//	Returns nonzero errno if fail
//
int getprocinfo(const char* node, pid_t pid, pid_t& parent, pid_t& child, pid_t& sibling)
{
	char path[512];																// node path, traditional max size
	if (node && node[0] != '\0')												// if remote node specified
	{	snprintf(path,sizeof(path),"/net/%s/proc/%d",node,pid);// build "/net/NODE/proc/NNN"
	} else {																			// if local node
		snprintf(path,sizeof(path),"/proc/%d",pid);					// build "/proc/NNN"
	}
	int fd = open(path,O_RDONLY);											// open for reading (???)
	////printf("getprocinfo: Open of %s returned %d\n",path,fd);	// ***TEMP***
	if (fd < 0) return(errno);													// fails
	procfs_info pidinfo;															// proc info area
	int stat = devctl(fd, DCMD_PROC_INFO, &pidinfo, sizeof (pidinfo), 0); // get status info for process
	parent = pidinfo.parent;													// return parent PID
	child = pidinfo.child;															// return child PID
	sibling = pidinfo.sibling;													// return sibling PID 
	close(fd);																			// done with fd
	if (stat) return(errno);
	return(EOK);
}