/////////////////////////////////////////////////////////////////////////////
//
//    File: serialmenu.cc
//
//    Usage:
//        see serialmenu.h
//
//    Description:
//        see serialmenu.h
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        July, 2003
//
/////////////////////////////////////////////////////////////////////////////

#include <errno.h>
#include "serialmenu.h"

#define SERIALMENU_BUF_LEN (82)

//
//    SerialMenu::Display - display options serial port menu
//
//    Returns nothing
//
void SerialMenu::Display()
{
    printf("  s - set port configuration\n");
	printf("  g - get port configuration\n");
	printf("  o - open port\n");
	printf("  c - close port\n");
	printf("  W - write string to port\n");
	printf("  w - write MVP command to port\n");
	printf("  R - read string from port\n");
	printf("  r - read MVP response from port\n");
	printf("  i - write and read MVP commands to/from port interactively\n");
	printf("  f - flush port\n");
	printf("  v - verbose\n");
}

//
//    SerialMenu::Process - process options for serial port menu
//
//    Returns true if menu option processed, false if not
//
bool SerialMenu::Process(char c)
{
    switch (c) {
	    case 's':
			configSet();
			break;
	    case 'g':
		    configGet();
			break;
	    case 'o':
		    if ( s->Open() == -1 ) {
			    perror("SerialMenu::Process - error opening port");
			}
			break;
	    case 'c':
		    if ( s->Close() == -1 ) {
			    perror("SerialMenu::Process - error closing port");
			}
			break;
	    case 'W':
		    writeBuf();
			break;
	    case 'w':
		    writeMVP();
			break;
	    case 'R':
		    readBuf();
			break;
	    case 'r':
		    readMVP();
			break;
	    case 'i':
		    interactMVP();
			break;
	    case 'f':
		    if ( s->Flush() == -1 ) {
			    perror("SerialMenu;:Process - error flushing port");
			}
			break;
	    case 'v':
	    	verbose();
			break;
		default:
		    return false;
			break;
	}

	return true;
}

void SerialMenu::configSet()
{
	// default variables (save for next time)
	static char defPortName[SERIAL_NAME_LEN];
	static speed_t defBaud;
	static char defControl[SERIAL_CONTROL_LEN];
	static int defUsecTimeout;
	bool baudOK;
	int bits, stopBit;
	char parity;

    // set default variables
    if ( s->ConfigGet(defPortName, &defBaud, defControl, &defUsecTimeout) < 0 ) {
    	if ( errno != EOK ) {
    		perror("SerialMenu::configSet - unable to get port configuration");
    	} else {
    		fprintf(stderr, 
    		        "SerialMenu::configSet - unable to get port configuration\n");
    	}
    }

	// get port descriptor
	Ask::String("Port Descriptor", defPortName, defPortName,
				sizeof(defPortName));

	// get baud rate
	baudOK = false;
	do {
		defBaud = Ask::Int("Baud Rate", defBaud, 1200, 115200); 
		if ( (defBaud ==   1200) ||
			 (defBaud ==   2400) ||
			 (defBaud ==   4800) ||
			 (defBaud ==   9600) ||
			 (defBaud ==  19200) ||
			 (defBaud ==  38400) ||
			 (defBaud ==  57600) ||
			 (defBaud ==  76800) ||
			 (defBaud == 115200) ) {
			baudOK = true;
		} else {
			printf("\007Need to enter one of the following ");
			printf("baud rates:\n");
			printf("  1200, 2400, 4800, 9600, 19200, 38400, 57600, ");
			printf("76800, or 115200\n");
		}
	} while ( !baudOK ); 

	// get control settings
	sscanf(defControl, "%i%c%i", &bits, &parity, &stopBit);
	bits = Ask::Int("Number of data bits (5-8)", bits, 5, 8);
	parity = Ask::Char("Parity ('N'=none, 'O' = odd, 'E' = even)",
					   parity, "NOE");
	stopBit = Ask::Int("Stop bits ('1' or '2')", stopBit, 1, 2);
	sprintf(defControl, "%i%c%i", bits, parity, stopBit);

	// get timeout time
	defUsecTimeout = Ask::Int("Timeout time (in usec)", defUsecTimeout,
					 0, 10000000);
	
	// set parameters
	if ( s->ConfigSet(defPortName, defBaud, defControl, defUsecTimeout) == -1 )
	{
    	perror("SerialMenu::configSet - unable to configure port");
    }
}

void SerialMenu::configGet()
{
	char portName[SERIAL_NAME_LEN];
	speed_t baud;
	char control[SERIAL_CONTROL_LEN];
	int usecTimeout;

	// get parameters or print error
    if ( s->ConfigGet(portName, &baud, control, &usecTimeout) < 0 ) {
    	if ( errno != EOK ) {
    		perror("SerialMenu::configGet - unable to get port configuration");
    	} else {
    		fprintf(stderr, 
    		        "SerialMenu::configGet - unable to get port configuration\n");
    	}
    }

	// print all the parameters
	printf("Baud rate: %i\n", (int) baud);
	printf("Control setting: %s\n", control);
	printf("Timeout (in usec): %i\n", usecTimeout);
	Ask::Pause();
}

void SerialMenu::writeBuf()
{
    char buf[SERIALMENU_BUF_LEN];

	Ask::String("String to write to port", "", buf, sizeof(buf));
	if ( s->WriteBuf((void *) buf, strlen(buf)) < 0 ) {
		if ( errno != EOK ) {
			perror("SerialMenu::writeBuf - error writing to port");
		} else {
			fprintf(stderr, "SerialMenu::writeBuf - error writing to port\n");
		}
	}
}

void SerialMenu::writeMVP()
{
    char buf[SERIALMENU_BUF_LEN];

	Ask::String("MVP Command to write to port", "", buf, sizeof(buf));
	if ( s->WriteMVP(buf, strlen(buf)) < 0 ) {
		if ( errno != EOK ) {
			perror("SerialMenu::writeMVP - error writing to port");
		} else {
			fprintf(stderr, "SerialMenu::writeMVP - error writing to port\n");
		}
	}
}

void SerialMenu::readBuf()
{
    char buf[SERIALMENU_BUF_LEN];
	int bufLen;

	bufLen = Ask::Int("Size of buffer to read from port", sizeof(buf), 
	                  0, sizeof(buf));
	if ( s->ReadBuf((void *) buf, bufLen) < 0 ) {
		if ( errno != EOK ) {
			perror("SerialMenu::readBuf - error reading port buffer");
		} else {
			fprintf(stderr, "SerialMenu::readBuf - error reading port buffer\n");
		}
	}
	printf("Read from port: %s\n", buf);
	Ask::Pause();
}

void SerialMenu::readMVP()
{
    char buf[SERIALMENU_BUF_LEN];

	if ( s->ReadMVP(buf, sizeof(buf)) < 0 ) {
		if ( errno != EOK ) {
			perror("SerialMenu::readMVP - error reading port buffer");
		} else {
			fprintf(stderr, "SerialMenu::readBuf - error reading port buffer\n");
		}
	}
	printf("Read from port: %s\n", buf);
	Ask::Pause();
}

void SerialMenu::interactMVP()
{
    char bufWrite[SERIALMENU_BUF_LEN] = "4 ST";
    char bufRead[SERIALMENU_BUF_LEN];

	printf("Type 'q' at any time to quit\n");
    do {
		Ask::String("Write", bufWrite, bufWrite, sizeof(bufWrite));
		if ( strlen(bufWrite) == 0 ) {
		    printf("Type 'q' at any time to quit\n");
			continue;
		} else if ( bufWrite[0] == 'q' ) {
		    break;
		}
		if ( s->WriteMVP(bufWrite, strlen(bufWrite)) == -1 ) {
			if ( errno != EOK ) {
				perror("SerialMenu::interactMVP - error writing MVP instruction to port");
			} else {
		    	fprintf(stderr,
		    	        "SerialMenu::interactMVP - error writing MVP instruction to port\n");
			}
		}
		if ( s->ReadMVP(bufRead, sizeof(bufRead)) < 0 ) {
			if ( errno != EOK ) {
				perror("SerialMenu::interactMVP - error reading MVP reply from port");
			} else {
		    	fprintf(stderr,
		    	        "SerialMenu::interactMVP - error reading MVP reply from port.\n");
			}
		}
		printf("Read ........ %s\n", bufRead);
	} while ( bufWrite[0] != 'q' );
}

void SerialMenu::verbose()
{
	s->Verbose(Ask::YesNo("Turn on verbose mode", s->Verbose()));
}