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
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

void readdir()
{
	int fd;
	char buf[1024];
	int iread;

	fd = open("bin/ls", O_RDONLY);
	if (fd < 0) {
		perror("failed to open /");
		return;
	}

	if (iread = read(fd, buf, sizeof(buf)) < 0) {
		perror("failed to read /");
		return;
	}

	printf("iread: %d\n", iread);	
	printf("buf: %s\n", buf); 	
	
	if(close(fd) < 0) {
		perror("failed to close ");
		return;
	}		
}

int
main(int argc, char **argv)
{
	if (chdir ("/") < 0) {
		perror("chdir");
	}
	if (system ("/bin/ls /") < 0) {
		perror("this one must work");
	}
	readdir();	
	if (chroot("/net/gcrear0.overbot.org/") < 0) {
		perror("chroot /net/gcrear0");
	}
	//if (chdir ("/") < 0) {
	//	perror("chdir on gcrear0");
	//}	
	readdir();	
	if (system ("bin/ls /") < 0) {
		perror("/bin/ls on gcrear0");		
	}	
}
