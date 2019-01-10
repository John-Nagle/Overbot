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


void
help()
{
	printf("chrootexec performs a chroot to the node\n");
	printf(" and performs an exec of the program with the new root\n");
	printf("usage: chrootexec <node> <program name> [program arguments]\n");
}


int
main(int argc, char **argv)
{
		
	if (argc < 3) {
		help();
		exit(EXIT_FAILURE);
	}
	char nodepath[512];

	if (netmgr_path(argv[1], NULL, nodepath, sizeof(nodepath)) < 0) {
		perror("chrootexec: Failed to find path to given node");
		exit (EXIT_FAILURE);
	}

	if (chdir("/") < 0) {
		perror("chdir to / failed");
	}
	
	printf("nodepath: %s\n", nodepath);
	
	//perform chroot
	if (chroot(nodepath) < 0) {
		perror("chrootexec: Failed to chroot to path");
		exit(EXIT_FAILURE);
	}

	if (chdir("/") < 0) {
		perror("chdir to / on new path");
	}

	//perform exec
	if (execv(argv[2], argv+2) < 0) {
		perror("chrootexec: Failed to exec program");
	}
}
