//
//	mountpfs  --  mount/unmount a psuedo-filesystem
//
//	Intended to run as a set-UID program
//
//	This program is intended to allow non-root users to operate on dummy
//	file systems. It works only if the image of the file system is writeable
//	by the calling user.
//
//	John Nagle
//	Team Overbot
//	July, 2003
//
//	Usage		mountpfs imagefile mountpoint
//
//
#include <stdio.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <libgen.h>
#include <string.h>
//
static const char* mounttype = "qnx4";									// default file system type
bool verbose = false;																// verbose mode
bool unmount = false;																// unmount mode
//
//	usage  --  print usage message and exit
//
static void usage(const char* progname)
{	printf("usage:\n %s [-v] filesystemimagefile.img mountpoint\n",progname);
	printf(" %s -u [-v] mountpoint\n",progname);
	exit(1);
}
//
//	checkowner  --  check that indicated path is owned by this user
//
static int checkowner(const char path[])					
{	struct stat statbuf;																// status info
	int status = stat(path,&statbuf);										// get status
	if (status) 
	{	perror("Cannot get file information"); return(status);	 }	// fails
	int fileowner = statbuf.st_uid;											// get file UID
	int runowner = getuid();													// get me
	//	Validate ownership.
	//	Root cannot run this program, and it cannot be used on files owned by root.
	if (fileowner != runowner || fileowner == 0 || runowner == 0) 
	{	printf("Not allowed - not owner of %s\n",path);
		return(1);																		// fails
	}
	return(0);
}
//
//	Main program
//
int main(int argc, const char* argv[])
{
	const char* file1 = 0;															// first file name	 argument found
	const char* file2 = 0;															// second file name
	if (geteuid() != 0)																	// if not root
	{	printf("This program must be given set-UID to root privileges, so that it can mount file systems.\n");
		exit(1);
	}
	if (getuid() == 0)																	// grumble 
	{	printf("You should not be running%s as root.\n",argv[0]); }	// but continue
	for (int i=1; i<argc; i++)														// scan all command line args
	{	const char* arg = argv[i];												// this arg
		if (arg[0] == '-')																// if flag
		{	switch (arg[1]) {
			case 'v':																		// verbose
				verbose = true;
				break;
			case 'u':																		// unmount
				unmount = true;
				break;
			default:
				usage(argv[0]);														// misused
			}
			continue;
		}
		//	This is a file arg
		if (!file1)																			// if no first file arg
		{	file1 = arg; 																	// keep
			continue;
		}
		if (!file2)																			// if no second file arg
		{	file2 = arg;																	// keep
			continue;
		}
		usage(argv[0]);																// too many file args
	}
	//	Check access permissions on files
	//	Caller must own all file args
	//	Mount or unmount as indicated
	if (unmount)																			// if unmounting
	{ 	if (!file1) usage(argv[0]);													// missing file arg
		char file1copy[512];															// build mount point directory name here
		strncpy(file1copy,file1,sizeof(file1copy));							// copy to working string
		const char* mountpointdir = dirname(file1copy);				// get base directory
		if (verbose) printf("Unmounting \"%s\"\n",file1);						// unmounting 
		if (checkowner(mountpointdir)) exit(1);							// mount point must be in owned dir
		int stat = mount(0,file1,_MOUNT_UNMOUNT,mounttype,0,0);		// unmount
     	if (stat) {	 perror("Unmount failed"); exit(1);	}           
     } else {																				// if mounting
		if (!file1) usage(argv[0]);													// missing file arg
		if (!file2) usage(argv[0]);													// missing file arg
		char file2copy[512];															// build mount point directory name here
		strncpy(file2copy,file2,sizeof(file2copy));							// copy to working string
		const char* mountpointdir = dirname(file2copy);				// get base directory
 		if (verbose) printf("Mounting \"%s\" as \"%s\" in directory \"%s\"\n",file1,file2,mountpointdir);		// mounting 
 		//	Final checks before mount
		if (checkowner(mountpointdir)) exit(1);							// mount point must be in owned dir
		if (checkowner(file1)) exit(1);											// check ownership
    	int stat = mount(file1,file2,_MOUNT_NOSUID,mounttype,0,0);		// mount
     	if (stat) {	 perror("Mount failed"); exit(1);	}
	}		
	return(0);																				// normal exit
}