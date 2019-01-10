# A quick makefile for building tests for lidarserver
#
# Team Overbot
# Eric Seidel, December 2003

LDFLAGS =
CC = qcc
CFLAGS = -Wall -O3
CFLAGS_COMPILE_ONLY = -c

all: lidar

lidar: lidar.cc
	${CC} ${CFLAGS_COMPILE_ONLY} ${LDFLAGS} -o lidar.a ${LIBS} lidar.cc

clean:
	rm -f *.o
	rm -f lms_checksum_test lms_client_test
