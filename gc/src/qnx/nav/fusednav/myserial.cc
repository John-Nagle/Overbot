//modfied from Celias

/////////////////////////////////////////////////////////////////////////////
//
//    File: serial.cc
//
//    Usage:
//        see serial.h
//
//    Description:
//        see serial.h
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        July, 2003
//
/////////////////////////////////////////////////////////////////////////////

#include "myserial.h"

//
//    Serial::Serial - default class constructor
//
Serial::Serial(char *portName, speed_t baud, char *control, int usecTimeout)
{
    // initialize configuration
    ConfigSet(portName, baud, control, usecTimeout);

    // initialize other data members
    fdOpen = false;
}

//
//    Serial::ConfigSet - set port configuration parameters
//
//    User can only set port name, baud rate, control, and timeout time.
//    Other settings are made by default.
//
//    Port name should be a character string, like "/dev/ser1".
//    Control should be a three character string like "8N1", where
//    the first character specifies the number of bits (5-8), the second
//    character specifies the parity (E=even, O=odd, and N=none), and the 
//    last character specifies the stop bit (1=one, 2=two).
//    The timeout time is in microseconds.
//
//    ConfigSet will open the port (for reading and writing) to get the 
//    current port attributes and set the port attributes.  It will only
//    close the port if fdOpen is false.
//
//    Returns 0 if successful, -1 if an error occurred (errno is set)
//
int Serial::ConfigSet(char *portName, 
                      speed_t baud, char *control, int usecTimeout)
{
    speed_t speed;
    int bit, stop;
    char par;
    tcflag_t numBits, parity=0x00, stopBits=0x00;

    // set port name
    strncpy(fdName, portName, SERIAL_NAME_LEN);
    fdName[SERIAL_NAME_LEN-1] = '0'; // make sure string is null-terminated

    // open port and get current attributes
    if ((fd = open(portName, O_RDWR)) == -1) {
        return -1;
    }
      
    if (tcgetattr(fd, &termios_p) == -1) {
        return -1;
    }

    // set baud rate
    switch (baud) {
        case   1200:
            speed =   B1200;
            break;
        case   2400:
            speed =   B2400;
            break;
        case   4800:
            speed =   B4800;
            break;
        case   9600:
            speed =   B9600;
            break;
        case  19200:
            speed =  B19200;
            break;
        case  38400:
            speed =  B38400;
            break;
        case  57600:
            speed =  B57600;
            break;
        case  76800:
            speed =  B76800;
            break;
        case 115200:
            speed = B115200;
            break;
        default:
            speed = B9600; // default
            fprintf(stderr, "Warning: baud rate = %i not valid\n", (int) baud);
            break;
    }
    if (cfsetispeed(&termios_p, speed) == -1) {
        return -1;
    }
	if (cfsetospeed(&termios_p, speed) == -1) {
        return -1;
    }

    // set control parameters
	sscanf(control, "%1d%1c%1d", &bit, &par, &stop); // extract params

    switch (bit) { // number of data bits
	    case 5:
		    numBits = CS5;
			break;
	    case 6:
		    numBits = CS6;
			break;
	    case 7:
		    numBits = CS7;
			break;
	    case 8:
		    numBits = CS8;
			break;
		default:
		    numBits = CS8; // default
		    fprintf(stderr, "Warning: number of bits (%i) must be 5-8.\n", bit);
			break;
	}

	if ((par == 'E') || (par == 'e')) {
	    parity        = PARENB;            // even parity
	} else if ((par == 'O') || (par == 'o')) {
	    parity        = (PARENB | PARODD); // odd parity
	}

	if (stop == 2) {
	    stopBits = CSTOPB;                // two stop bits (otherwise 1)
	}

    // configure modes

    // devc-ser8250 supported flags:
	//     c_iflag: BRKINT ICRNL IGNBRK IXON
	//     c_oflag: OPOST
	//     c_cflag: CLOCAL CSIZE CSTOPB PARENB PARODD
	//     c_lflag: ECHO ECHOE ECHOK ECHONL ICANON IEXTEN ISIG NOFLSH

    // configure modes - input modes
	termios_p.c_iflag &= ~( // modes to turn off
	                        BRKINT |      // ignore break
	                        ICRNL  |      // leave CR
							IXON          // no output flow control
						  );

	termios_p.c_iflag |=  ( // modes to turn on
							IGNBRK        // ignore break
						  );
	
	// configure modes - output modes
/*	termios_p.c_oflag &= ~( // modes to turn off
	                        OPOST        // no output processing
						  ); */
	termios_p.c_oflag |= OPOST;

	// configure modes - control modes
	termios_p.c_cflag &= ~( // modes to turn off
							CSIZE   |    // bits-per-byte
							CSTOPB  |    // two stop bits
							PARENB  |    // don't enable parity
							PARODD       // don't enable parity
						  );

	termios_p.c_cflag |=  ( // modes to turn on
	                        CLOCAL   |   // ignore modem status lines
							numBits  |   // bits-per-byte
							stopBits |   // number of stop bits
							parity       // parity setting
						  );

	// configure modes - local modes
	termios_p.c_lflag &= ~( // modes to turn off
	                        ECHO   |     // no echo of any kind
							ECHOE  |
							ECHOK  |
							ECHONL |
							ICANON |     // no canonical input
							IEXTEN |     // no QNX extensions
							ISIG   |     // no signal generation
							NOFLSH       // enable flushing
						  );

	// configure modes - control characters
	termios_p.c_cc[VMIN]  = 0;           // read() as much data as possible
	termios_p.c_cc[VTIME] = 0;           // without waiting

	// set port attributes
	if (tcsetattr(fd, TCSANOW, &termios_p) == -1) {
		return -1;
	}

	// close port, if wasn't open
	if (!fdOpen) {
		if (close(fd) == -1) {
			return -1;
		}
	}

	// set timeout time
	usec = usecTimeout;

	return EXIT_SUCCESS;
}

//
//    Serial::ConfigGet - get port configuration parameters
//
//    Call after setting configuration with ConfigSet().
//
//    User can only get port name, baud rate, control, and timeout time.
//    Other settings are default settings.
//
//    Port name will be a character string, like "/dev/ser1".
//    Control will be a three character string like "8N1", where
//    the first character specifies the number of bits (5-8), the second
//    character specifies the parity (E=even, O=odd, and N=none), and the 
//    last character specifies the stop bit (1=one, 2=two).
//    The timeout time is in microseconds.
//
//    ConfigGet will open the port to get the current port attributes.
//    It will only close the port if fdOpen is false.
//
//    Returns 0 if successful, -1 if an error occurred (errno is set)
//
int Serial::ConfigGet(char *portName, 
                      speed_t *baud, char *control, int *usecTimeout)
{
    speed_t ibaud, obaud;
    int bit, stop;
    char par;

	// get port name
    strncpy(portName, fdName, SERIAL_NAME_LEN);

    // open port and get current attributes
    if ((fd = open(fdName, O_RDWR)) == -1) {
        return -1;
    }
    if (tcgetattr(fd, &termios_p) == -1) {
        return -1;
    }

    // get baud rate
    if ((ibaud = cfgetispeed(&termios_p)) == -1) {
        return -1;
    }
	if ((obaud = cfgetospeed(&termios_p)) == -1) {
        return -1;
    }
	*baud = ibaud;
	if (ibaud != obaud) {
	    fprintf(stderr, 
		        "Warning: input and output baud rates not same on '%s'.\n",
		        fdName);
		return -2;
	}

    // get control parameters
	switch (termios_p.c_cflag & CSIZE) {
	    case CS5:
		    bit = 5;
			break;
	    case CS6:
		    bit = 6;
			break;
	    case CS7:
		    bit = 7;
			break;
	    case CS8:
		    bit = 8;
			break;
		default:
		    bit = 8; // default
		    fprintf(stderr, 
			        "Warning: number of data bits not legal (5-8) on '%s'.\n",
			        fdName);
			break;
	}

    if ((termios_p.c_cflag & PARENB) &&
        (termios_p.c_cflag & PARODD)) {
	    // odd parity
		par = 'O';
	} else if (termios_p.c_cflag & PARENB) {
		// even parity
		par = 'E';
	} else {
	    // no parity
		par = 'N';
	}

    if (termios_p.c_cflag & CSTOPB) {
	    stop = 2;
	} else {
	    stop = 1;
	}

	sprintf(control, "%1d%1c%1d", bit, par, stop); // insert params

	// close port, if wasn't open
	if (!fdOpen) {
		if (close(fd) == -1) {
			return -1;
		}
	}

	// set timeout time
	*usecTimeout = usec;

	return EXIT_SUCCESS;
}

//
//    Serial::Open - open port
//
//    Call after setting configuration with ConfigSet().
//
//    Returns 0 if successful, -1 if an error occurred (errno is set)
//
int Serial::Open()
{
    // open port
    if ((fd = open(fdName, O_RDWR)) == -1) {
        return -1;
    }

    // remember that port is open
    fdOpen = true;

    return EXIT_SUCCESS;
}

//
//    Serial::Close - close port
//
//    Call after setting configuration with ConfigSet().
//
//    Returns 0 if successful, -1 if an error occurred (errno is set)
//
int Serial::Close()
{
    // close port
	if (close(fd) == -1) {
		return -1;
	}

	// remember that port is closed
	fdOpen = false;

    return EXIT_SUCCESS;
}

//
//    Serial::WriteBuf - write buffer data to the port
//
//    Call after openning the port with Open()
//
//    Returns number of bytes written if successful, 
//            -1 if an error occurred (errno is set)
//
ssize_t Serial::WriteBuf(void *buf, size_t nbytes)
{
    ssize_t actBytes;

    // make sure that port is open
    if (fdOpen) {
		if ((actBytes = write(fd, buf, nbytes)) == -1) {
		    return -1;
		}
	} else {
	    fprintf(stderr, "Need to open port '%s' for write.\n", fdName);
	    return -2;
	}

    return actBytes;
}

//
//    Serial::ReadBuf - read buffer data from the port, within timeout time
//
//    Call after opening the port with Open()
//
//    Returns byte count if successful, -1 if an error occurred (errno is set)
//
int Serial::ReadBuf(void *buf, size_t nbytes)
{
    struct timeval tv;
    fd_set rfd;

    // make sure that port is open
    if (fdOpen) {
        // clear the set of file descriptors and add serial port
        FD_ZERO(&rfd);
        FD_SET(fd, &rfd);

        // set the timeout
        tv.tv_sec = 0;
        tv.tv_usec = usec;

		int ret = Serial::readPort(&rfd, &tv, buf, nbytes);
		
		// read character from port when ready
		if (ret == -1) {
			return -1;
		}
		return ret;
	} else {
	    fprintf(stderr, "Need to open port '%s' for read.\n", fdName);
	    return -2;
	}
}

//
//    Serial::Flush - flush the input buffer of the port
//
//    Call Flush() after sufficient time has passed for reading to be
//    complete.
//    
//    Returns 0 if successful, -1 if an error occurred (errno is set)
//
int Serial::Flush()
{
    return tcflush(fd, TCIFLUSH);
}

//
//    Serial::readPort - generic read data from port
//
//    Call from Read routines.
//
//    Returns byte count if successful, -1 if an error occurred (errno is set)
//
int Serial::readPort(fd_set *rfd, struct timeval *tv, void *buf, int nbytes)
{
	// wait until port is ready for reading
	switch (select(fd+1, rfd, 0, 0, tv)) {
		case -1:
			return -1;
			break;
		case 0:
			fprintf(stderr, "readPort select timed out\n");
			return -1;
			break;
		default: // port ready for reading
		    int ret = read(fd,buf, nbytes);
			//printf ("ReadPort: read %d bytes\n", ret);
			if (ret == -1) {
				fprintf(stderr, "Error with 'read' on '%s'.\n", fdName);
				return -2;
			} else {
				return ret;
			}
			break;
	}
}
