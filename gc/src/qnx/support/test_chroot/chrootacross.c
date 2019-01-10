/**
 * Khian Hao Lim, Team Overbot, 11/29/03
 * 
 * A simple program that chroots to given node's / directory
 * then execs a program whose name is given by as this program's argument
 * 
 * This was meannt to called from the watchdog start file so that programs are run with chroot
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/netmgr.h>

int
main(int argc, char **argv)
{
	if (system ("/bin/ls /") < 0) {
		perror("this one must work");
	}	
	if (chroot("/net/gcrear0/") < 0) {
		perror("chroot /net/gcrear0");
	}
	if (system ("/bin/ls /") < 0) {
		perror("/bin/ls on gcrear0");		
	}	
}
