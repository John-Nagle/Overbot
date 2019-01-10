/////////////////////////////////////////////////////////////////////////////
//
//    File: actservermenu.cc
//
//    Usage:
//        see actserver.h
//
//    Description:
//        Special menu to access functionality of and test Servers derived
//        from the Actuator Server.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        December, 2003
//
/////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <termios.h>
#include <stdio.h>
#include "actserver_.h"
#include "actservermenu.h"

void ActServer_::MenuThread()
{  
	if ( pthread_create(NULL, NULL, &menuThreadStart, this) != EOK ) {
		fprintf(stderr, 
		        "ActServer_::MenuThread - error calling pthread_create\n");
	}
}

void *ActServer_::menuThread()
{
	// default menus
	ActServerMenu am(this, "Actuator Server", 'g', 's');	
	TimedLoopMenu tcm(tc, "Control Loop", 't', 'h');
	TimedLoopMenu tdm(td, "Data Loop", 't', 'h');

	MenuHandler mh;

	mh.Install(&am);
	mh.Install(&tcm);
	mh.Install(&tdm);
       
    mh.Start();
    
    return (void *) NULL;	// so compiler won't complain
}

//
//    ActServerMenu::Display - display options for Actuator Server menu
//
//    Returns nothing
//
void ActServerMenu::Display()
{
	DisplayAdditional();
	printf("  s - simulation mode (SIMU message)\n");
	printf("  i - initialize %s Server (INIT message)\n", as->ServerName());
	printf("  v - verbose (VERB message)\n");
}

//
//    ActServerMenu::Process - process options for Actuator Server menu
//
//    Returns true if menu option processed, false if not
//
bool ActServerMenu::Process(char c)
{
	char prompt[ACTSERVER_PROMPT_LEN];
	
    switch (c) {
	    case 's':
            simulation(Ask::YesNo("Simulation mode on", simulation()));
			break;
	    case 'i':
	    	if ( bool initialized = initialize() ) {
	    		printf("%s Server already initialized.\n", as->ServerName());
	    	} else {
	    		snprintf(prompt, ACTSERVER_PROMPT_LEN,
	    		         "Initialize the %s Server", as->ServerName());
            	initialize(Ask::YesNo(prompt, initialized));
	    	}
			break;
	    case 'v':
            verbose(Ask::YesNo("Verbose mode on", verbose()));
			break;
		default:
		    return ProcessAdditional(c);
			break;
	}

	return true;
}

bool ActServerMenu::simulation()
{
	ActServerMsg msg;
	int err;
	
	// get simulation state
	msg.m_simu.m_msgtype = char4('S','I','M','U');
	msg.m_simu.get = true;
	err = clientport->MsgSend(msg, msg);
	if ( err ) {
		// MsgSend error
		perror("ActServerMenu::simulation - MsgSend failed");
	} else if ( msg.m_simu.err != ActServer::ERR_OK ) {
		// server processing error - none
		fprintf(stderr, "ActServerMenu::simulation - %s\n", 
		        ActServer::ErrMsg(msg.m_simu.err));
	}
	
	return msg.m_simu.simulation;
}

void ActServerMenu::simulation(bool simulation)
{
	ActServerMsg msg;
	int err;
	
	// set simulation state
	msg.m_simu.m_msgtype = char4('S','I','M','U');
	msg.m_simu.get = false;
	msg.m_simu.simulation = simulation;
	err = clientport->MsgSend(msg, msg);
	if ( err ) {
		perror("ActServerMenu::simulation - MsgSend failed");
	} else if ( msg.m_simu.err != ActServer::ERR_OK ) {
		// server processing error - none
		fprintf(stderr, "ActServerMenu::simulation - %s\n", 
		        ActServer::ErrMsg(msg.m_simu.err));
	}
}

bool ActServerMenu::initialize()
{
	ActServerMsg msg;
	int err;
		
	// get initialized state
	msg.m_init.m_msgtype = char4('I','N','I','T');
	msg.m_init.get = true;
	err = clientport->MsgSend(msg, msg);
	if ( err ) {
		perror("ActServerMenu::initialize - MsgSend failed");
	} else if ( msg.m_init.err != ActServer::ERR_OK ) {
		// server processing error - none
		fprintf(stderr, "ActServerMenu::initialize - %s\n", 
		        ActServer::ErrMsg(msg.m_init.err));
	}
	
	return msg.m_init.initialized;
}

void ActServerMenu::initialize(bool initialize)
{
	ActServerMsg msg;
	int err;
	
	if ( initialize ) {
		// initialize server
		msg.m_init.m_msgtype = char4('I','N','I','T');
		msg.m_init.get = false;
		err = clientport->MsgSend(msg, msg);
		if ( err ) {
			perror("ActServerMenu::initialize - MsgSend failed");
		} else if ( msg.m_init.err != ActServer::ERR_OK ) {
			// server processing error - none
			fprintf(stderr, "ActServerMenu::initialize - %s\n", 
		        	ActServer::ErrMsg(msg.m_init.err));
		}
	}
}

bool ActServerMenu::verbose()
{
	ActServerMsg msg;
	int err;
	
	// get verbose state
	msg.m_verb.m_msgtype = char4('V','E','R','B');
	msg.m_verb.get = true;
	err = clientport->MsgSend(msg, msg);
	if ( err ) {
		// MsgSend error
		perror("ActServerMenu::verbose - MsgSend failed");
	} else if ( msg.m_verb.err != ActServer::ERR_OK ) {
		// server processing error - none
		fprintf(stderr, "ActServerMenu::verbose - %s\n", 
		        ActServer::ErrMsg(msg.m_verb.err));
	}
	
	return msg.m_verb.verbose;
}

void ActServerMenu::verbose(bool verbose)
{
	ActServerMsg msg;
	int err;
	
	// set verbose state
	msg.m_verb.m_msgtype = char4('V','E','R','B');
	msg.m_verb.get = false;
	msg.m_verb.verbose = verbose;
	err = clientport->MsgSend(msg, msg);
	if ( err ) {
		perror("ActServerMenu::verbose - MsgSend failed");
	} else if ( msg.m_verb.err != ActServer::ERR_OK ) {
		// server processing error - none
		fprintf(stderr, "ActServerMenu::verbose - %s\n", 
		        ActServer::ErrMsg(msg.m_verb.err));
	}
}
