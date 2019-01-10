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

	char path[512];

	netmgr_ndtostr(ND2S_DIR_SHOW|ND2S_DOMAIN_SHOW|ND2S_NAME_SHOW, ND_LOCAL_NODE, path, 512);

	printf("localpath: %s\n", path);
	
}
