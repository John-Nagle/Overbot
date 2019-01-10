/////////////////////////////////////////////////////////////////////////////
//
//    File: ethernetmenu.cc
//
//    Usage:
//        see ethernetmenu.h
//
//    Description:
//        see ethernetmenu.h
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        January, 2003
//
/////////////////////////////////////////////////////////////////////////////

#include <errno.h>
#include "ethernetmenu.h"

#define ETHERNETMENU_BUF_LEN (82)

//
//    EthernetMenu::Display - display options for Ethernet menu
//
//    Returns nothing
//
void EthernetMenu::Display()
{
	printf("  g - get Ethernet configuration\n");
	printf("  c - connection on Ethernet TCP/IP socket\n");
	printf("  d - shutdown Ethernet TCP/IP socket connection\n");
	printf("  S - send string to socket\n");
	printf("  s - send string to controller (via socket)\n");
	printf("  R - receive string from socket\n");
	printf("  r - receive string from controller (via socket)\n");
	printf("  i - send and receive controller commands interactively\n");
	printf("  f - flush receive buffer on socket\n");
	printf("  v - verbose\n");
}

//
//    EthernetMenu::Process - process options for Ethernet menu
//
//    Returns true if menu option processed, false if not
//
bool EthernetMenu::Process(char c)
{
    switch (c) {
	    case 'g':
		    configGet();
			break;
	    case 'c':
		    if ( e->Connect() == -1 ) {
	perror("EthernetMenu::Process - error initiating connection on socket");
			}
			break;
	    case 'd':
		    if ( e->Shutdown() == -1 ) {
	perror("EthernetMenu::Process - error shutting down connection on socket");
			}
			break;
	    case 'S':
		    sendBuf();
			break;
	    case 's':
		    sendController();
			break;
	    case 'R':
		    recvBuf();
			break;
	    case 'r':
		    recvController();
			break;
	    case 'i':
		    interactController();
			break;
	    case 'f':
		    if ( e->Flush() == -1 ) {
			    perror("EthernetMenu;:Process - error flushing receive buffer");
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

void EthernetMenu::configGet()
{
	// print all the parameters
	printf("Hostname    = %s\n", e->Hostname());
	printf("IPAddress   = %s\n", e->IPAddress());

	Ask::Pause();
}

void EthernetMenu::sendBuf()
{
    char buf[ETHERNETMENU_BUF_LEN];

	Ask::String("String to write to socket connection", "", buf, sizeof(buf));
	if ( e->SendBuf(buf, strlen(buf)) < 0 ) {
		if ( errno != EOK ) {
			perror("EthernetMenu::sendBuf - error writing to socket");
		} else {
			fprintf(stderr, "EthernetMenu::sendBuf - error writing to socket\n");
		}
	}
}

void EthernetMenu::sendController()
{
    char buf[ETHERNETMENU_BUF_LEN];

	Ask::String("Controller Command to write to socket", "", 
	            buf, sizeof(buf));
	if ( e->SendController(buf, strlen(buf)) < 0 ) {
		if ( errno != EOK ) {
			perror("EthernetMenu::sendController - error writing to Controller");
		} else {
			fprintf(stderr, "EthernetMenu::sendController - error writing to ");
			fprintf(stderr, "Controller\n");
		}
	}
}

void EthernetMenu::recvBuf()
{
    char buf[ETHERNETMENU_BUF_LEN];
	int bufLen;
	int stat;

	bufLen = Ask::Int("Size of buffer to read from socket", sizeof(buf), 
	                  0, sizeof(buf));
	if ( (stat = e->RecvBuf(buf, bufLen)) < 0 ) {
		if ( stat == -1 ) {
			perror("EthernetMenu::recvBuf - error reading socket");
		} else {	// stat == -2
			fprintf(stderr, "EthernetMenu::recvBuf - error reading socket\n");
		}
	}
	printf("Received from socket: %s\n", buf);
	Ask::Pause();
}

void EthernetMenu::recvController()
{
    char buf[ETHERNETMENU_BUF_LEN];

	if ( e->RecvController(buf, sizeof(buf)) < 0 ) {
		if ( errno != EOK ) {
			perror("EthernetMenu::recvController - error reading socket");
		} else {
			fprintf(stderr, "EthernetMenu::recvController - error reading socket\n");
		}
	}
	printf("Received from controller: %s\n", buf);
	Ask::Pause();
}

void EthernetMenu::interactController()
{
    char bufSend[ETHERNETMENU_BUF_LEN] = "KP?";
    char bufRecv[ETHERNETMENU_BUF_LEN*7];		// enough room for TH command

	printf("Type 'q' at any time to quit\n");
    do {
		Ask::String("Send", bufSend, bufSend, sizeof(bufSend));
		if ( strlen(bufSend) == 0 ) {
		    printf("Type 'q' at any time to quit\n");
			continue;
		} else if ( bufSend[0] == 'q' ) {
		    break;
		}
		if ( e->SendController(bufSend, strlen(bufSend)) < 0 ) {
			if ( errno != EOK ) {
				perror("EthernetMenu::interactController - error writing Controller instruction to socket");
			} else {
		    	fprintf(stderr,
		    	        "EthernetMenu::interactController - error writing Controller instruction to socket\n");
			}
		}
		if ( e->RecvController(bufRecv, sizeof(bufRecv)) < 0 ) {
			if ( errno != EOK ) {
				perror("EthernetMenu::interactController - error reading Controller reply from socket");
			} else {
		    	fprintf(stderr,
		    	        "EthernetMenu::interactController - error reading Controller reply from socket\n");
			}
		}
		printf("Received ........ %s\n", bufRecv);
	} while ( bufSend[0] != 'q' );
}

void EthernetMenu::verbose()
{
	e->Verbose(Ask::YesNo("Turn on verbose mode", e->Verbose()));
}