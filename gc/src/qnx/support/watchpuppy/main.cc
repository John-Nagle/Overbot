 //	
 //		The Watchpuppy, by Khian Hao Lim
 //
 //		Reboots if the watchdog stops.
 //
 //		One of these is run for each CPU, but they all run on the main CPU.
 //
#include <unistd.h> 
#include <stdio.h>
#include <pthread.h>
#include <sys/types.h>
#include <fcntl.h>
#include "mutexlock.h"
#include "messaging.h"
#include "logprint.h"
  
#define HEARTBEAT_SLEEP 1000 //ms
#define REBOOT_SLEEP    3000 //ms
#define INITIAL_REBOOT_SLEEP 10000 //ms
//
//	enable_watchdog  -- reenable the hardware watchdog, by opening the watchdog device.
//
static void
enable_watchdog(unsigned int time, const char* rebootdev)
{
	int fd = open(rebootdev, O_WRONLY);
	if (fd < 0) {
		logprintf("Unable to open reboot device \"%s\": %s\n", rebootdev, strerror(errno));
		return;	
	}
	//	Writing a decimal integer to the reboot device holds off a reboot for that many seconds.
	//	The first write enables the reboot device. 
	char buf[128];
	snprintf(buf, sizeof(buf), "%d", time);	
	if (write(fd, buf, strlen(buf)) != int(strlen(buf))) {
		logprintf("Write error reboot device \"%s\": %s\n", rebootdev, strerror(errno));
	}

	if (close(fd) < 0) {
		logprintf("Unable to close reboot device \"%s\": %s\n", rebootdev, strerror(errno));
	}	
}

static void
heartbeat_thread(const char* rebootdev)
{
 	enable_watchdog(60, rebootdev);														// allow this long for system to start up (?)	
	// define message port, 
	MsgClientPort clientport((const char *)NULL, 0.2);

	while (1) {
		delay (HEARTBEAT_SLEEP); //make it sleep for a while
		
		//TODO, watchdogreset should return meaningful
		if (clientport.watchdogreset() == 0) 
		{  //watchdog happy with watchpuppy
			enable_watchdog(10, rebootdev);	
		} else {
			logprintf("Watchdog did not respond properly. Reboot coming.\n");
		}
	}
}
//
//	usage  -- usual usage 
//
static void usage()
{	printf("Usage: watchpuppy [reboot device]\n");
	printf("Must run under watchdog.\n");
	printf("Must run as user \"vehicle\"\n");
	exit(1);
}
//
//	main program
//
int main(int argc, const char* argv[])
{	const char* rebootname = "/dev/reboot";						// default reboot name
	for (int i=1; i<argc; i++)													// arg processing
	{	const char* arg = argv[i];											// this arg
		if (arg[0] == '-')															// if flag
		{	switch(arg[1]) {														// fan out on arg
			default: usage();
			}
		} else {
			rebootname = arg;													// set reboot device
		}
	}
	//	Reboot if watchdog fails.  
	logprintf("Using reboot timer device \"%s\"\n", rebootname);
	heartbeat_thread(rebootname);
}
