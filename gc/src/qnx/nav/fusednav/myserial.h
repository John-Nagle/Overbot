//modified from Celia's version

/////////////////////////////////////////////////////////////////////////////
//
//    File: serial.h
//
//    Usage:
//        #include "myserial.h"
//
//        Serial s("/dev/ser1", 38400, "8N1", 100000);
//
//    Description:
//        A uniform interface for using serial ports.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        July, 2003
//
/////////////////////////////////////////////////////////////////////////////

#ifndef SERIAL_H
#define SERIAL_H

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <termios.h>

#define SERIAL_NAME_LEN (82)
#define SERIAL_CONTROL_LEN (4)

class Serial {
public:
    Serial(char *portName, speed_t baud, char *control, int usecTimeout);
	int ConfigSet(char *portName, 
	              speed_t baud, char *control, int usecTimeout);
	int ConfigGet(char *portName, 
	              speed_t *baud, char *control, int *usecTimeout);
	int Open();
	int Close();
	int WriteBuf(void *buf, size_t nbytes);
	int WriteMVP(char *buf, size_t strLen);
	int ReadBuf(void *buf, size_t nbytes);
	int ReadMVP(char *buf, size_t strLen);
	int Flush();
	int fd;

private:
    char fdName[SERIAL_NAME_LEN];
	bool fdOpen;
	struct termios termios_p;
	int usec;                    // timeout in microseconds
	int readPort(fd_set *rfd, struct timeval *tv, void *buf, int nbytes);
};

#endif // SERIAL_H