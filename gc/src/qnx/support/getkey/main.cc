/**
 * Why doesn't qnx provide more shell utilities
 * This is a basic guess at how the linux /sbin/getkey utility is implemented
 * 
 * example usage:
 * getkey <message> <timeout>
 * 
 * will print out the message and wait for timeout seconds
 * If it gets any input, it would exit with 0 status, otherwise 1 status
 * 
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <fcntl.h>

void
usage()
{
	printf("example usage\n");
	printf("getkey <timeout>\n");
	printf("will wait for <timeout> secs, timeout > 0\n");
	printf("If it gets any input, it would exit with 0 status, otherwise 1 status\n");

	exit(1);
}

int
main(int argc, char ** argv)
{
	if (argc != 2) {
		usage();
	}

	int sec = atoi(argv[1]);
	
	if (! (sec > 0)) {
		usage();
	}
	
	printf("You have %d seconds to press any key\n", sec);
	
	struct timeval t;
	t.tv_usec = 0;
	t.tv_sec = sec;
		
	fd_set fds;
	int max_fd = STDIN_FILENO;
	FD_SET(STDIN_FILENO, &fds);
	select (max_fd + 1, &fds, 0 , 0, &t);
		
	if (FD_ISSET(STDIN_FILENO, &fds)) {
		return 0;
	}
	else {
		return 1;
	}
}