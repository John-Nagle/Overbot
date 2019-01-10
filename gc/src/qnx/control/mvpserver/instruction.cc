////////////////////////////////////////////////////////////////////////////
//
//    File: instruction.cc
//
//    Usage:
//        See mvpserver.h.
//
//    Description:
//        Routines related to the MVP Server sending to and receiving
//        replies from the MVP units.
//
//    See also:
//        mvpserver.h, main.cc
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        August, 2003
//
/////////////////////////////////////////////////////////////////////////////

#include "mvpserver_.h"

//
//    MVPServer_::instructionInit - set-up MVP unit for MVP Server
//
//    Uses MVPServerMsg contruct to send instructions using instructionSend().
//
//    Returns MVPServer::ERR_OK if command sent successfully, 
//            MVPServer::Err otherwise
//
MVPServer::Err MVPServer_::instructionInit()
{
	MVPServerMsg msgInit;
	MVPServer::Err err;

    if ( !(*nodeInit)[msg.m_inst.node] ) {
		// turn on ok reply
		msgInit.m_inst.node = msg.m_inst.node;
		msgInit.m_inst.cmd = MVPServer::CMD_OK;
		msgInit.m_inst.param = 1;
		msgInit.m_inst.paramActive = true;
		if ( (err=instructionSend(&msgInit)) != MVPServer::ERR_OK ) {
			return err;
		}
		
		// turn on checksums
		msgInit.m_inst.node = msg.m_inst.node;
		msgInit.m_inst.cmd = MVPServer::CMD_CK;
		msgInit.m_inst.param = 1;
		msgInit.m_inst.paramActive = true;
		if ( (err=instructionSend(&msgInit)) != MVPServer::ERR_OK ) {
			return err;
		}
		
		// no serial reply delay
		msgInit.m_inst.node = msg.m_inst.node;
		msgInit.m_inst.cmd = MVPServer::CMD_SD;
		msgInit.m_inst.param = 0;
		msgInit.m_inst.paramActive = true;
		if ( (err=instructionSend(&msgInit)) != MVPServer::ERR_OK ) {
			return err;
		}		
    }

    (*nodeInit)[msg.m_inst.node] = true;
    return MVPServer::ERR_OK;
}

//
//    MVPServer_::instructionSend - issue an instruction to an MVP unit
//
//    Gets the parts of the instruction from the message body.  Fills
//    in the return value from the MVP if necessary.
//
//    NOTE: need to call instructionInit() before instructionSend().
//
//    Returns MVPServer::ERR_OK if command sent successfully, 
//            MVPServer::Err otherwise
//
MVPServer::Err MVPServer_::instructionSend(MVPServerMsg *msgSend)
{
    char instr[MVPSERVER_INSTR_LEN];	// composite instruction to send MVP
    char reply[MVPSERVER_REPLY_LEN];	// reply from MVP
    int nodeReply;						// node that replied
    bool done = false;					// done communicating with MVP
    int tryNum = 0;						// number of times tried

    // return error if parameter is out-of-range
    if ( msgSend->m_inst.paramActive &&
         cmdDB[msgSend->m_inst.cmd].paramRange && 
         ((msgSend->m_inst.param < cmdDB[msgSend->m_inst.cmd].paramMin) || 
		  (msgSend->m_inst.param > cmdDB[msgSend->m_inst.cmd].paramMax)) ) {
       	fprintf(stderr, "MVPServer_::instructionSend - ");
       	fprintf(stderr, "parameter %i out of range (%i, %i).\n",
           	    msgSend->m_inst.param, cmdDB[msgSend->m_inst.cmd].paramMin, 
           	    cmdDB[msgSend->m_inst.cmd].paramMax);
		fprintf(stderr, "MVPServer_::instructionSend - ");
		fprintf(stderr, "command '%s' not sent.\n", 
		        cmdDB[msgSend->m_inst.cmd].cmdStr);
	    return MVPServer::ERR_PARAMETER_OUT_OF_RANGE;
	}

	// construct initial part of instruction
	sprintf(instr, "%i %s", msgSend->m_inst.node, cmdDB[msgSend->m_inst.cmd].cmdStr);

	// add parameter part of instruction if necessary
	if ( msgSend->m_inst.paramActive ) {
	    sprintf(instr, "%s %i", instr, msgSend->m_inst.param);
	}
	
	// append checksum
	checksumAppend(instr);

	do {
		// reset errno
		errno = 0;
		
   	 	// send instruction
	    if ( s->WriteMVP(instr, strlen(instr)) < 0 ) {
			fprintf(stderr, "MVPServer_::instructionSend - bad write;");
			if ( ++tryNum < MVPSERVER_TRIES_NUM ) {
				instructionErrMsgTryAgain(msgSend);
				continue;
			} else {
				instructionErrMsgTryLast(msgSend);
				return MVPServer::ERR_WRITE_BAD;
			}
		}
		
		// get reply
		if ( s->ReadMVP(reply, sizeof(reply)) < 0 ) {
			fprintf(stderr, "MVPServer_::instructionSend - bad read;");
			if ( ++tryNum < MVPSERVER_TRIES_NUM ) {
				instructionErrMsgTryAgain(msgSend);
				continue;
			} else {
				instructionErrMsgTryLast(msgSend);
				return MVPServer::ERR_READ_BAD;
			}
		}

		// deal with reply
		if ( (msgSend->m_inst.paramActive && 
		      !(msgSend->m_inst.cmd == MVPServer::CMD_PI)) ||
		     !cmdDB[msgSend->m_inst.cmd].valueRtn ) {
			// expect "Ok" reply
			if ( strcmp(reply, "Ok") != 0 ) {
				fprintf(stderr, "MVPServer_::instructionSend - no 'Ok' reply;");
				if ( ++tryNum < MVPSERVER_TRIES_NUM ) {
					instructionErrMsgTryAgain(msgSend);
					continue;
				} else {
					instructionErrMsgTryLast(msgSend);
					return MVPServer::ERR_READ_BAD;
				}
			}
			msgSend->m_inst.value = 0;	// make sure value is set to something
		} else {
			// expect full-bodied reply
			// check checksum
			if ( (cmdDB[msgSend->m_inst.cmd].valueCksum && !checksumOK(reply)) ) {
				fprintf(stderr, "MVPServer_::instructionSend - bad checksum;");
				if ( ++tryNum < MVPSERVER_TRIES_NUM ) {
					instructionErrMsgTryAgain(msgSend);
					continue;
				} else {
					instructionErrMsgTryLast(msgSend);
					return MVPServer::ERR_READ_BAD;
				}
			}
			// parse reply
			if ( msgSend->m_inst.cmd == MVPServer::CMD_FV ) {
				// handle FV instruction, special case -- FIX!!!
				// value is an int not int*; need union?
				//sprintf((char *)msgSend->m_inst.value, "%s", reply);
			} else {
				// normal instruction reply
				sscanf(reply, "%d %x#", &nodeReply, &msgSend->m_inst.value);
				
				// check node number
				if ( nodeReply != msgSend->m_inst.node ) {
					fprintf(stderr, "MVPServer_::instructionSend - bad node#;");
					if ( ++tryNum < MVPSERVER_TRIES_NUM ) {
						instructionErrMsgTryAgain(msgSend);
						continue;
					} else {
						instructionErrMsgTryLast(msgSend);
						return MVPServer::ERR_READ_BAD;
					}
				}
				
				// handle 4 digit negative number case
				if ( (msgSend->m_inst.value & 0x8000) &&
				     cmdDB[msgSend->m_inst.cmd].valueNeg &&
				     (cmdDB[msgSend->m_inst.cmd].valueNibbles == 4) ) {
					msgSend->m_inst.value -= 65536;
				}
			}
		}
		done = true;
	} while ( !done  );

	return MVPServer::ERR_OK;
}

//
//    MVPServer_::instructionErrMsgTryAgain - handle try again message
//
//    Returns nothing
//
void MVPServer_::instructionErrMsgTryAgain(MVPServerMsg *msgSend)
{
	// try again message
	fprintf(stderr, "trying again (%s)", cmdDB[msgSend->m_inst.cmd].cmdStr);
	if ( errno != 0 ) {
		perror(NULL);
	} else {
		fprintf(stderr, "\n");
	}
	
	// flush port in case we're out of synch
	if ( s->Flush() != EOK ) {
		fprintf(stderr, "MVPServer_::instructionSend - ");
		fprintf(stderr, "not able to flush serial port.\n");
	}
}

//
//    MVPServer_::instructionErrMsgTryLast - handle last try message
//
//    Returns nothing
//
void MVPServer_::instructionErrMsgTryLast(MVPServerMsg *msgSend)
{
	// bad read on last try (WARNING: possible node reset)
	fprintf(stderr, " (%s) %d tries, possible node reset!",
	        cmdDB[msgSend->m_inst.cmd].cmdStr, MVPSERVER_TRIES_NUM);
	if ( errno != 0 ) {
		perror(NULL);
	} else {
		fprintf(stderr, "\n");
	}
}