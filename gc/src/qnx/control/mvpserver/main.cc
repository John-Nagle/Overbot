////////////////////////////////////////////////////////////////////////////
//
//    File: main.cc
//
//    Usage:
//        See mvpserver.h.
//
//    Description:
//        See mvpserver.h and mvpserver_.h.
//
//        This file contains the constructor, destructor, message handling,
//        and main routines for the MVP Server.
//
//        The MVP Server process is blocking.  There is only one thread that
//        both receives messages and send instructions over the serial port
//        (with a timeout).
//
//    See also:
//        mvpserver.h, mvpserver_.h, cmddb.cc, checksum.cc, 
//        mvp.h, mvp.cc, libgccontrol
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        August, 2003
//
/////////////////////////////////////////////////////////////////////////////

#include "mvpserver_.h"
#include "mvpservermenu.h"

//
//    MVPServer::MVPServer - default constructor
//
MVPServer_::MVPServer_()
{
	// initialize server messaging
	serverport = new MsgServerPort(0.0);	// define message port, no timeout
	
	int stat = serverport->ChannelCreate();	// create a channel, tell watchdog
	if  ( stat ) {
		perror("MVPServer_::MVPServer_: ChannelCreate failed.\n");
		serverport->Dump();
	}
	
	// initialize server state
	verbose = true;
    
    // create serial port
    s = new Serial(MVPSERVER_PORTNAME, MVPSERVER_PORTBAUD, 
                   MVPSERVER_PORTCONTROL, MVPSERVER_PORTTIMEOUT);
 
	// server resources initialized with INIT message

	// initialize command database
	cmdDBInit();
    
    // intialize node checksumOn database
    nodeInit = new vector<bool>(MVPSERVER_NODE_MAX+1, false);
};

//
//    MVPServer::~MVPServer - default destructor
//
MVPServer_::~MVPServer_()
{
	// not critical if don't check for errors when closing port
    s->Close();
}

void MVPServer_::MenuThread()
{  
	if ( pthread_create(NULL, NULL, &menuThreadStart, this) != EOK ) {
		fprintf(stderr, 
		        "MVPServer_::MenuThread - error calling pthread_create()\n");
	}
}

void *MVPServer_::menuThread()
{	
	MVPServerMenu mm(this, "MVP Server", 'm', 'i');
	SerialMenu sm(s, "Serial Port", 's', 'o');
	MenuHandler mh;

	mh.Install(&mm);
    mh.Install(&sm);
    mh.Start();
    
    return (void *) NULL;	// so compiler won't complain
}

void MVPServer_::SerialPortInit(char *portName, int portBaud, int portTimeout)
{
    if ( (s->ConfigSet(portName, portBaud, 
                       MVPSERVER_PORTCONTROL, portTimeout) == -1) ||
         (s->Open() == -1) || (s->Flush() == -1) ) {
    	perror("MVPServer_::SerialPortInit - unable to configure serial port\n");
    }
}
    			        	
void MVPServer_::MessageServer()
{
	// start collecting messages forever
	for ( ; ; ) {		
		// wait for message
		rcvid = serverport->MsgReceive(msg);
		
		// handle errors
		if ( rcvid < 0 ) {
			fflush(stdout);
			perror("MVPServer_::MessageServer - MsgReceive failed\n");
			sleep(1);		// avoid tight loop if repeated trouble FIX
			continue;
		}
		if ( rcvid == 0 ) {
			perror("MVPServer_::MessageServer - received a pulse\n");
			// pulses don't require a reply
			continue;
		}
		
		//	handle message
		if ( verbose ) {
			//printf("MVPServer_::MessageServer - received a message.\n");
		}
		MVPServer_::messageHandle();
		
		// tell the watchdog we are still alive FIX
		////serverport.watchdogreset();
	}
}

void MVPServer_::messageHandle()
{	
	// fan out on server requests, based on code at beginning of message
	switch ( msg.m_inst.m_msgtype ) {
		case MVPServerMsgINST::k_msgtype:			// INST msg
			msg.m_inst.err = instructionInit();
			msg.m_inst.err = instructionSend(&msg);
			break;
		case MVPServerMsgVERB::k_msgtype:			// VERB msg
			if ( msg.m_verb.get ) {
				msg.m_verb.verbose = verbose;
			} else {
				verbose = msg.m_verb.verbose;
			}
			msg.m_verb.err = MVPServer::ERR_OK;
			break;
		default:
			break;
	}
	
	// reply to message by sending it back
	int err = MsgReply(rcvid, EOK, msg);
	if ( err ) {
		perror("MVPServer_::messageHandle - MsgReply failed\n");
	}
}

//
//  Usage - print usage message and exit
//
static void Usage()
{
	printf("Usage: mvpserver  [-n <portName>] [-b <portBaud>] ");
	printf("[-t <portTimeout>] [-m]\n");
	printf("Defaults: portName=%s, portBaud=%d, portTimeout=%d usec\n",
	       MVPSERVER_PORTNAME, MVPSERVER_PORTBAUD, MVPSERVER_PORTTIMEOUT);
    exit(1);
}

int main(int argc, char ** argv)
{	
	MVPServer_ ms;
	char portName[MVPSERVER_PORTNAME_LEN] = MVPSERVER_PORTNAME;
	int portBaud = MVPSERVER_PORTBAUD, portTimeout = MVPSERVER_PORTTIMEOUT;
	
	// parse command line arguments
    for ( int i = 1 ; i < argc; i++ ) {
    	const char* arg=argv[i];
        if ( arg[0] == '-' ) {
        	switch( arg[1] ) {
            	case 'n':
            		if ( i < (argc-1) ) {
            			strncpy(portName, argv[++i], MVPSERVER_PORTNAME_LEN-1);
            		} else {
            			Usage();
            		}
            		break;
            	case 'b':
            		if ( i < (argc-1) ) {
            			portBaud = atoi(argv[++i]);
            		} else {
            			Usage();
            		}
            		break;
            	case 't':
            		if ( i < (argc-1) ) {
	            		portTimeout = atoi(argv[++i]);
            		} else {
            			Usage();
            		}
            		break;
            	case 'm':
					// start thread to make menus available
					ms.MenuThread();
            		break;
            	default:
            		Usage();
					break;
            }
            continue;
        }
        Usage();
    }
              	            		 
    // configure serial port
    ms.SerialPortInit(portName, portBaud, portTimeout);
		
	// start collecting messages forever
	ms.MessageServer();
}