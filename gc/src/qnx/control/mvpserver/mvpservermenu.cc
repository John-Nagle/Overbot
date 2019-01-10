/////////////////////////////////////////////////////////////////////////////
//
//    File: mvpservermenu.cc
//
//    Usage:
//        see mvpserver.h
//
//    Description:
//        Special menu to send messages to the MVP Server.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        November, 2003
//
/////////////////////////////////////////////////////////////////////////////

#include "mvpserver_.h"
#include "mvpservermenu.h"

//
//    MVPServerMenu::Display - display options for MVP Server menu
//
//    Returns nothing
//
void MVPServerMenu::Display()
{
	printf("  i - send instruction to MPV unit (INST message)\n");
	printf("  v - verbose (VERB message)\n");
}

//
//    MVPServerMenu::Process - process options for MVP Server menu
//
//    Returns true if menu option processed, false if not
//
bool MVPServerMenu::Process(char c)
{
	bool cmdValid = false;
	MVPServer::Cmd cmd;
	MVPServerMsg msg;
	int err;
	
    switch (c) {
	    case 'i':
			// get instruction parameters
			node = Ask::Int("Node number", node, 0, MVPSERVER_NODE_MAX);
			do {
				Ask::String("Command", cmdStr, cmdStr, MVPSERVER_CMD_LEN);
				if ( ms->CmdDBStr2Enum(cmdStr, &cmd) ) {
					cmdValid = true;
				} else {
					printf("Command not in database, enter another.\n");
				}
			} while ( !cmdValid );
			if ( paramActive = Ask::YesNo("Parameter active", 
			                              paramActive) ) {
				param = Ask::Int("Parameter value", param);
			}
			
			// set up message
			msg.m_inst.m_msgtype = char4('I','N','S','T');
			msg.m_inst.node = node;
			msg.m_inst.cmd = cmd;
			msg.m_inst.param = param;
			msg.m_inst.paramActive = paramActive;
			
			// send message
			err = clientport->MsgSend(msg, msg);
			if ( err ) {
				perror("MVPServerMenu::Process - MsgSend failed");
			} else if ( msg.m_inst.err != MVPServer::ERR_OK ) {
				// server processing error - none
				fprintf(stderr,
				        "MVPServerMenu::Process - %s\n", 
				        MVPServer::ErrMsg(msg.m_inst.err));
			} else {
				printf("Return value (if relevant): %d\n", msg.m_inst.value);
			}	    
			break;
	    case 'v':
           	verbose(Ask::YesNo("Verbose mode on", verbose()));
			break;
		default:
		    return false;
			break;
	}

	return true;
}

bool MVPServerMenu::verbose()
{
	MVPServerMsg msg;
	int err;
	
	// get verbose state
	msg.m_verb.m_msgtype = char4('V','E','R','B');
	msg.m_verb.get = true;
	err = clientport->MsgSend(msg, msg);
	if ( err ) {
		// MsgSend error
		perror("MVPServerMenu::verbose - MsgSend failed");
	} else if ( msg.m_verb.err != MVPServer::ERR_OK ) {
		// server processing error - none
		fprintf(stderr, "MVPServerMenu::verbose - %s\n", 
		        MVPServer::ErrMsg(msg.m_verb.err));
	}
	
	return msg.m_verb.verbose;
}

void MVPServerMenu::verbose(bool verbose)
{
	MVPServerMsg msg;
	int err;
	
	// set verbose state
	msg.m_verb.m_msgtype = char4('V','E','R','B');
	msg.m_verb.get = false;
	msg.m_verb.verbose = verbose;
	err = clientport->MsgSend(msg, msg);
	if ( err ) {
		perror("MVPServerMenu::verbose - MsgSend failed");
	} else if ( msg.m_verb.err != MVPServer::ERR_OK ) {
		// server processing error - none
		fprintf(stderr, "MVPServerMenu::verbose - %s\n", 
		        MVPServer::ErrMsg(msg.m_verb.err));
	}
}
