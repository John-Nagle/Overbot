//
//	usercontrol.cpp  -- very simple user interface for Overbot
//
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <libgen.h>
#include <dirent.h>
#include "ask.h"
//
//	Constants
//
const char activefilename[] = "/home/vehicle/goactive";						// if present, will autostart
const char waypointfilename[] = "/home/vehicle/waypoints.txt";			// determines active waypoint set
const char waypointdir[] = "/home/vehicle/waypoints";						// waypoint directory
const char waypointlink[] = "/home/vehicle/waypoints.txt";					// waypoint link
const char waypointsuffix[] = "txt";

/*
Overbot Control
In ACTIVE mode. Current waypoint file "foo"
1 Go to STANDBY mode
2 Go to ACTIVE mode
2 Select waypoint file.
3 Reboot.
*/
//
//	slaywatchdog  -- shut down watchdog as if control-C
//
static int slaywatchdog()
{
	char ss[511];
	snprintf(ss, sizeof(ss), "slay -%d watchdog", SIGINT);										// slay with SIGINT signal
	int stat = system(ss);
	return(stat);
}
//
//	getsuffix -- get suffix for a file name
//
static const char* getsuffix(const char* s)
{
	int n = strlen(s);																				// length of s
	for (int i=n-1; i>=0; i--)
	{	if (s[i] == '.')
		{	return(&s[i+1]);	}																	// return suffix after .
	}
	return(s);																							// no-suffix case
}
//
//	readdirectory -- read a directory and return an array of strings
//
static int readdirectory(const char* path, const char* suffixwanted, std::vector<std::string>& names)
{	names.clear();
	std::string pathstring(path);																// path as string
	DIR* dirinfo =  opendir(path);															// open dir for reading
	if (!dirinfo)
	{	printf("Unable to open directory \"%s\": %s\n", path, strerror(errno));	// explain
		return(-1);
	}
	for (;;)																								// for all directory entries
	{	struct dirent* direntry = readdir(dirinfo);										// read directory entry
		if (!direntry) break;
		const char* fname = direntry->d_name;										// filename
		char s[511];																					// working string
		strncpy(s, fname,sizeof(s));															// copy filename
		const char* base = basename(s);												// base part of filename
		const char* suffix = getsuffix(base);											// suffix
		if (strcmp(suffixwanted, suffix) != 0) continue;								// ignore if wrong suffix
		std::string st(fname);																		// the filename
		st = pathstring + '/' + st;																// build name
		printf("%3d: %s\n", names.size(), base);										// print this filename as menu item
		names.push_back(st);																	// save name
	}
	closedir(dirinfo);
	return(0);
}
//
//	ProcessStandby  -- go active.
//
//	Deletes the "active" file and slays the watchdog
//
static void ProcessStandby()
{
	struct stat statbuf;														// status buffer
	int status = ::stat(activefilename, &statbuf);					// check if file present
	if (stat < 0) return;													// already gone, done
	status = unlink(activefilename);								// remove active file name
	if (status < 0) perror("Unable to unlink active file name");
	slaywatchdog();
	printf("Now in STANDBY.\n");
}
//
//	ProcessActive  -- go active.
//
//	Creates the "active" file and starts the watchdog
//
static void ProcessActive()
{
	struct stat statbuf;														// status buffer
	int status = ::stat(activefilename, &statbuf);				// check if file present
	if (status >= 0) return;												// already there, done
	status = unlink(activefilename);								// remove active file name
	int fd = creat(activefilename,S_IRUSR|S_IRGRP|S_IROTH);	// create as read only
	if (fd < 0) {	perror("Unable to create active file name"); return; }
	close(fd);																	// close
	printf("Now ACTIVE.\n");
}
//
//	ProcessWaypointFile  -- set active waypoint file
//
//	Deletes the "active" file and slays the watchdog
//
static void ProcessWaypointFile()
{
	printf("Select waypoint file:\n");
	std::vector<std::string> names;								// waypoint filenames
	int stat = readdirectory(waypointdir, waypointsuffix, names);
	if (stat < 0) return;
	Ask asker;
	int sel = asker.Int("Select: ", -1);								// ask user to select file
	if (sel < 0 || sel >= int(names.size())) return;
	std::string name = names[sel];									// set new waypoint file
	std::string node = getenv("HOSTNAME");					// get name of this machine
	name = "/net/" + node + name;								// prefix with node name
	stat = unlink(waypointlink);										// remove old file
	stat = symlink(name.c_str(), waypointlink);				// new link
	if (stat < 0)
	{	perror("Unable to create link to new waypoint file.\n");	}
	printf("New waypoint file is \"%s\"\n", name.c_str());	// print new name
}
//
//	ProcessRestart  -- set active waypoint file
//
//	Restarts (?)
//
static void ProcessRestart()
{
	slaywatchdog();														// force shutdown
	sleep(1);
	int stat = system("sh /home/vehicle/bin/startvehicle.sh &");
	if (stat < 0)
	{	perror("Error restarting watchdog");	}
}
//
//	Display Prompt
//
static void DisplayPrompt()
{	
	struct stat statbuf;														// status buffer
	int status = ::stat(activefilename, &statbuf);				// check if file present
	bool isactive = status >= 0;										// if present, active.
	printf("Overbot status: %s\n", isactive ? "ACTIVE" : "STANDBY");
	printf("Select command:\n");
	printf(" a  go ACTIVE\n");
	printf(" s  go to STANDBY\n");
	printf(" w  choose new waypoint file\n");
	printf(" r  restart\n");
	printf(" q  quit\n");
}
//
//	runmenus  -- runs the menus
//
//	Simple select loop
//
static void runmenus()
{	Ask asker;
	for(;;) {
		DisplayPrompt();
		char ch = asker.Char("Command:",'?');
		switch (ch)
		{	case 'a':	ProcessActive(); break;
			case 's':	ProcessStandby(); break;
			case 'w':	ProcessWaypointFile(); break;
			case 'r':		ProcessRestart(); break;
			case 'q':	exit(0);
			default:		printf("?\n"); break;
		}
	}
}
//
//	Main program
//
int main(int argc, const char* argv)
{
	runmenus();
    return(0);				
}