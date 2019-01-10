////////////////////////////////////////////////////////////////////////////
//
//    File: controllerinstr.cc
//
//    Usage:
//        See controller.h.
//
//    Description:
//        Routines related to sending instructions and receiving replies
//        from the Galil controller units.
//
//        Based on mvpserver instruction.cc code of August 2003.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        February, 2004
//
/////////////////////////////////////////////////////////////////////////////

#include <limits.h>
#include "controller.h"
#include "logprint.h"
//
//	Constants
//
const int CONTROLLER_TRIES_NUM = 4;										// number of retries before giving up


const char k_endline[] = "\r\n";													// CR LF, for end of line
const int k_resynctimeout = 25*1000;										// wait 25ms for in-transit network data to show up
   
//
//    Controller::instructionSend - issue an instruction to a Galil unit
//
//		int arg, int reply
//
//    Gets the parts of the instruction from the message body.  Fills
//    in the return value from the Galil controller if necessary.
//
//    Returns Controller::ERR_OK if command sent successfully,
//            Controller::Err otherwise
//
Controller::Err Controller::instructionSend(Controller::Cmd cmd, 
                                            int param, int *rtn)
{
	if ( rtn == NULL ) {
		if (readonly) return(ERR_READ_ONLY_MODE);						// command not allowed in read only mode
		return(instructionSend(cmd, (float) param, (float *) NULL));	// just do command
	} else {
		// call char version
		char reply[CONTROLLER_REPLY_LEN];	// reply from controller
		*rtn = 0;																				// return 0 if error
		Controller::Err stat = instructionSend(cmd, (float) param, reply,CONTROLLER_REPLY_LEN);	// make request
		if (stat == Controller::ERR_OK)												// if success
		{	*rtn = atoi(reply);		}														// convert from string to integer
		if ( verbose ) {
			logprintf("Controller::instructionSend int int* - *rtn = %d\n", *rtn);
		}
		return(stat);																			// return status
	}
}
//
//	instructionSend  -- int arg, float reply
//
Controller::Err Controller::instructionSend(Controller::Cmd cmd, 
                                            int param, float *rtn)
{
	return instructionSend(cmd, (float) param, rtn);
}
//
//	instructionSend -- int arg, boolean reply
//
//	Value returned from controller is an integer.
//
Controller::Err Controller::instructionSend(Controller::Cmd cmd, 
                                            int param, bool *rtn)
{
	if ( rtn == NULL ) {
		if (readonly) return(ERR_READ_ONLY_MODE);						// command not allowed in read only mode
		return(instructionSend(cmd, (float) param, (float *) NULL));
	} else {
		int rtnInt;																				// returned integer
		Controller::Err stat = instructionSend(cmd, (float) param, &rtnInt);
		*rtn = (rtnInt != 0); 																// true if nonzero
		return(stat);
	}
}

Controller::Err Controller::instructionSend(Controller::Cmd cmd, 
                                            float param, float *rtn)
{
    char instr[CONTROLLER_INSTR_LEN];	// instruction to send controller
    char reply[CONTROLLER_REPLY_LEN];	// reply from controller
    bool query = false;
    Controller::Err stat;

    // return error if parameter is out-of-range
    if ( (rtn == NULL) && parameterOutOfRange(cmd, param) ) {
	    return Controller::ERR_PARAMETER_OUT_OF_RANGE;
    }
	if ( verbose ) {
		logprintf("Controller::instructionSend float float* - param = %f\n", param);
	}

	// construct instruction, adding parameter if necessary
	if ( rtn == NULL ) {
		if (readonly) return(ERR_READ_ONLY_MODE);						// command not allowed in read only mode
		// need to use command string
		if ( cmdDB[cmd].paramOK ) {
	    	//	Insert parameter
			snprintf(instr, CONTROLLER_INSTR_LEN, "%s %f%s", cmdDB[cmd].cmdStr, param,k_endline);
		}	else {
			//	No parameter
			snprintf(instr, CONTROLLER_INSTR_LEN, "%s%s", cmdDB[cmd].cmdStr,k_endline);
		}
	} else {
		// need to use query string
		snprintf(instr, CONTROLLER_INSTR_LEN, "%s%s", cmdDB[cmd].queryStr,k_endline);
		query = true;	
	}
	if ( verbose ) {
		logprintf("Controller::instructionSend float float* - instr = '%s'\n", instr);
	}
	
	// send instruction and get reply, try multiple times if necessary
	if ( (stat = instrSend(instr, query, reply, CONTROLLER_REPLY_LEN)) != Controller::ERR_OK ) {
		return stat;
	}
	if ( verbose ) {
		logprintf("Controller::instructionSend float float* - reply = '%s'\n", reply);
	}
	
	// deal with reply
	int j;
	if ( rtn != NULL ) {
		if ( (j = sscanf(reply, "%f", rtn)) != 1 ) {
			logprintf( 
			    "Controller::instructionSend - unable to parse rtn value (%s)\n",
			    	reply);
			return(Controller::ERR_RESPONSE_BAD);			// fails
		}
		if ( verbose ) {
			logprintf("Controller::instructionSend float float* - *rtn = %f\n", *rtn);
		}
	}

	return Controller::ERR_OK;
}
//
//	instructionSend, int arg and char return
//
Controller::Err Controller::instructionSend(Controller::Cmd cmd, int param, char *rtn, size_t rtnlen, int timeoutus)
{
    char instr[CONTROLLER_INSTR_LEN];	// instruction to send controller
    bool query = false;
    Controller::Err stat;

	if ( verbose ) {
		logprintf("Controller::instructionSend int char* - param = %d\n", param);
	}
	
	// construct instruction, adding parameter if necessary
	if ( rtn == NULL ) {
		if (readonly) return(ERR_READ_ONLY_MODE);						// command not allowed in read only mode
		// need to use command string
		snprintf(instr, CONTROLLER_INSTR_LEN, "%s%s", cmdDB[cmd].cmdStr,k_endline);
		if ( cmdDB[cmd].paramOK ) {
	    	snprintf(instr, CONTROLLER_INSTR_LEN,"%s%d", instr, param);
		}	
	} else {
		// need to use query string
		snprintf(instr, CONTROLLER_INSTR_LEN, "%s%s", cmdDB[cmd].queryStr,k_endline);
		query = true;
	}
	if ( verbose ) {
		logprintf("Controller::instructionSend int char* - instr = '%s'\n", instr);
	}
	
	// send instruction and get reply, try multiple times if necessary
	if ( (stat = instrSend(instr, query, rtn, rtnlen, timeoutus)) != Controller::ERR_OK ) {
		return stat;
	}
	if ( verbose ) {
		logprintf("Controller::instructionSend int char* - reply = '%s'\n", rtn);
	}

	return Controller::ERR_OK;
}
//
//	InstructionSend  -- sends string, gets string
//
//	Most general and slowest form.  Used for non real time functions like downloading and uploading.
//	Any response is allowed. Caller must check.
//
Controller::Err Controller::instructionSend(Controller::Cmd cmd, bool query, const std::string& param, std::string& outreply, int timeoutus)
{
    char reply[CONTROLLER_REPLY_LEN];	// reply from controller
    std::string instr;										// instruction to send controller

	// construct instruction, adding parameter if necessary
	if (!query ) {
		if (readonly) return(ERR_READ_ONLY_MODE);						// command not allowed in read only mode
		// need to use command string, plus param if supported
		instr = cmdDB[cmd].cmdStr;
		if ( cmdDB[cmd].paramOK ) {
	    	instr += param;
		}	
	} else {
		// need to use query string - param not allowed (NOT SURE ABOUT THIS)
		instr = cmdDB[cmd].queryStr;
	}
	if (cmdDB[cmd].paramType != 3)
	{	instr += k_endline;	}									// except for multiline "download" command, add a CR at the end
	if ( verbose ) {
		//	Print the command
		logprintf("Controller::instructionSend string string& - param = %s\n",param.c_str());
	}	
	// send instruction and get reply, try multiple times if necessary
	Controller::Err stat = instrSend(instr.c_str(), true, reply, CONTROLLER_REPLY_LEN, timeoutus);
	if (stat != Controller::ERR_OK)
	{	return stat;
	}
	if ( verbose ) {
		logprintf("Controller::instructionSend string string& - reply = '%s'\n", reply);
	}
	outreply = reply;													// we do not parse the reply here.
	return Controller::ERR_OK;
}

//
//	InstructionSend  -- sends text command, get float reply
//
//	These are relatively rare, so we can afford to do a string allocation.
//
Controller::Err Controller::instructionSend(Controller::Cmd cmd, const char *param, float *rtn, int timeoutus)
{
    std::string instr;										// instruction to send controller
    char reply[CONTROLLER_REPLY_LEN];	// reply from controller
	bool query = false;								// assume not query
	Controller::Err stat = ERR_OK;				// OK so far
	// construct instruction, adding parameter if necessary
	if ( rtn == NULL ) {
		if (readonly) return(ERR_READ_ONLY_MODE);						// command not allowed in read only mode
		// need to use command string, plus param if supported
		instr = cmdDB[cmd].cmdStr;
		if ( cmdDB[cmd].paramOK ) {
	    	instr += param;
		}	
	} else {
		// need to use query string - param not allowed (NOT SURE ABOUT THIS)
		instr = cmdDB[cmd].queryStr;
		query = true;	
	}
	if (cmdDB[cmd].paramType != 3)
	{	instr += k_endline;	}									// except for multiline "download" command, add a CR at the end
	if ( verbose ) {
		//	Print the command, converting CR to NL if necessary for output.
		logprintf("Controller::instructionSend char float* - param = %s\n", instr.c_str());
	}	
	// send instruction and get reply, try multiple times if necessary
	if ( (stat = instrSend(instr.c_str(), query, reply, CONTROLLER_REPLY_LEN, timeoutus)) != Controller::ERR_OK ) {
		return stat;
	}
	if ( verbose ) {
		logprintf("Controller::instructionSend char float* - reply = '%s'\n", reply);
	}
	
	// deal with reply
	int j;
	if ( rtn != NULL ) {
		if ( (j = sscanf(reply, "%f", rtn)) != 1 ) {
			logprintf( 
			    "Controller::instructionSend - unable to parse rtn value (%s)\n",
			    	reply);
		}
		if ( verbose ) {
			logprintf("Controller::instructionSend char float* - *rtn = %f\n", *rtn);
		}
	}

	return Controller::ERR_OK;
}
//
//	instrSend -- send a message to the controller and wait for a reply
//
//	This does all the work, and is responsible for error recovery.
//
//	Note that we can spend a long time in here if the network connection fails,
//	because TCP will retry. We time this out.
//
//	We do not recover from a loss of the Ethernet connection. The calling program
//	must reopen the connection. We don't want to reconnect without the caller
//	knowing about it; the controller's state may have changed.
//
//	The caller must end the instruction with the usual '\r', or the special syntax
//	of the "DL" instruction.
//
Controller::Err Controller::instrSend(const char *instr, bool query, char *reply, size_t replylen, int timeoutus)
{
	assert(instr);
	assert(reply);
	assert(replylen > 0);											// must have non-zero reply buffer size
	reply[0] = '\0';														// insure reply buffer is null-terminated
	ost::MutexLock lok(m_lock);									// prevent collision between requests
	if (!e.Connected()) return(Controller::ERR_NOT_CONNECTED);			// lost Ethernet connection
	for (int tryNum = 0; tryNum < CONTROLLER_TRIES_NUM; tryNum++)	
	{	// reset errno
		errno = 0;
		if (tryNum > 0) instructionErrMsgTryAgain(instr);							// report if retry

		// Flush queued input in case there's junk from a previous command.
		if ( e.Flush() < 0 ) {
			logprintf( "Controller::instrSend - not able to flush Ethernet socket\n");
		}
   	 	// Send instruction
   	 	Controller::Err err = instrSendOnce(instr, query, reply, replylen, timeoutus);	// try to do the command
   	 	if (err == Controller::ERR_OK)							// success
   	 	{	 if (tryNum > 0) {	logprintf("Controller:: instrSend - Retry successful.\n"); }
   	 		return(Controller::ERR_OK);				// success
   	 	}
   	 	//	Trouble. Handle error
   	 	err = instructionErrMsgResync();													// try to resync controller
   	 	if (err != Controller::ERR_OK)
   	 	{	//	Big trouble - can't resync controller.
   	 		logprintf("Controller::instrSend: Trouble: Unable to resync controller.\n");
   	 		//	Controller is not talking at all.  Disconnect the TCP connection.
   	 		//	We may be able to re-open it later.
   	 		e.Shutdown();
   	 		return(err);																				// fails
   	 	}
		//	Retry,
	}
	instructionErrMsgTryLast(instr);
	return Controller::ERR_SEND_BAD;
}
//
//	instrSendOnce -- send a message to the controller and wait for a reply
//
//	This does all the work, and is responsible for error detection, but not recovery.
//
//	Note that we can spend a long time in here if the network connection fails,
//	because TCP will retry. We time this out.
//	This is used both from UDP and TCP.  In UDP mode, we never fail on the
//	send, but we time out waiting for data on the receive.
//
//	The caller must end the instruction with the usual '\r', or the special syntax
//	of the "DL" instruction.
//
Controller::Err Controller::instrSendOnce(const char *instr, bool query, char *reply, size_t replylen, int timeoutus)
{
	assert(instr);
	assert(reply);
	assert(replylen > 0);											// must have non-zero reply buffer size
	reply[0] = '\0';														// insure reply buffer is null-terminated
	ost::MutexLock lok(m_lock);									// prevent collision between requests

   	// Send instruction
   	if (verbose)
   	{	logprintf("Controller::instrSendOnce: sending %s", instr);	}
	if ( e.SendBuf(instr, strlen(instr)) < 0 )
	{	logprintf( "Controller::instrSendOnce - bad write;");
		return Controller::ERR_SEND_BAD;
	}
	// Get reply
	if ( e.RecvController(reply, replylen, timeoutus) < 0 )
	{	logprintf( "Controller::instrSendOnce - bad read\n");
		return Controller::ERR_RECV_BAD;
	}
   	if (verbose)
   	{	logprintf("Controller::instrSendOnce: received %s\n", reply);	}
	//	We received a reply. 
	//	Replies consist of either ":", "?", or a value. RecvController trims off the CR LF : after values.
	bool singlecolon = strcmp(reply,":") == 0;				// did we get a colon, only?
	bool singlequestion = strcmp(reply,"?") == 0;			// did we get a question mark, only?
	if (singlequestion)
	{	logprintf("Controller::instrSend: controller returned error.\n");
		return Controller::ERR_RESPONSE_BAD;					// bad reply
	} 
	if (query) 																	// if query, must have something more than ":"
	{	if (!singlecolon) return(Controller::ERR_OK);			// expected data, got just ":"
		logprintf("Controller::instrSend - expected data, received \"%s\".\n", reply); 
	}
	else 
	{	if (singlecolon) return(Controller::ERR_OK);			// good case
		logprintf("Controller::instrSend - expected ':', received \"%s\".\n", reply); 
	}
	//	Not good - fails
	return Controller::ERR_RESPONSE_BAD;						// bad reply
}

bool Controller::parameterOutOfRange(Controller::Cmd cmd, float param)
{	
	if ( cmdDB[cmd].paramOK && cmdDB[cmd].paramRange ) {
		if ( (param < cmdDB[cmd].paramMin) || (param > cmdDB[cmd].paramMax) ) {
	       	logprintf( "Controller::parameterOutOfRange - parameter %g out of range (%g, %g).\n",
        	   	    param, cmdDB[cmd].paramMin, cmdDB[cmd].paramMax);
			logprintf( "Controller::parameterOutOfRange - command '%s' not sent.\n", cmdDB[cmd].cmdStr);
		    return true;
		}
	}
	
	return false;
}
//
//	instructionErrMsgResync  -- try to get controller back in synch after error
//
//	We send a newline, in hopes of getting a ":" back.
//
//	***TROUBLE*** it appears to be possible to come out of here after receiving a ":" with
//	another old message still queued, especially in UDP mode.  MUST FIX.
//
Controller::Err Controller::instructionErrMsgResync()
{	const int k_resync_tries = 5;											// try to resync
	const char k_nullcmd[] = "\r";										// do nothing command
	const char k_enddownloadcmd[] = "\r\\\r";					// might be stuck in download mode
	for (int tries = 0; tries < k_resync_tries; tries++)			// retry a few times
	{
		if (!e.Connected()) return(ERR_NOT_CONNECTED);	// lost Ethernet connection
		usleep(k_resynctimeout);											// allow any outstanding datagrams to come in
		e.Flush();																	// flush any queued input
		//	Send a blank line
		if ( e.SendBuf(k_nullcmd, strlen(k_nullcmd)) < 0 ) 
		{		logprintf( "Controller::instructionErrMsgResync - bad write.\n");
				return Controller::ERR_SEND_BAD;
		}
		char reply[CONTROLLER_REPLY_LEN];						// reply area
		// get reply
		int stat = e.RecvController(reply, CONTROLLER_REPLY_LEN);	// read reply
		if (stat == -2)															// if timeout
		{	logprintf("Controller::instructionErrMsgResync - read timeout,\n");		
			// Might be stuck in download mode. Send a backslash to get out of that dead end.
			if (tries % 3 == 1)													// on tries 1, 4, etc. send a backslash.
			{	logprintf("Controller::instructionErrMsgResync: Trying end of download symbol.\n");
				if (e.SendBuf(k_enddownloadcmd, strlen(k_enddownloadcmd)) < 0)
				{	logprintf( "Controller::instructionErrMsgResync - bad write.\n");
					return Controller::ERR_SEND_BAD;						// fails
				}
			}
			continue;																// try again
		}
		if (stat < 0 ) 																// if network error
		{		logprintf( "Controller::instructionErrMsgResync - bad read.\n");
				return Controller::ERR_RECV_BAD;
		}
		//	We should get back a ":" only. Anything else is trouble.
		if (strcmp(reply, ":") == 0)									// success, apparently resynched
		{	
			int flushcnt = e.Flush();										// flush any remaining input, which should not be present
			if (flushcnt == 0)												// normal case
			{	logprintf("Controller::instructionErrMsgResync -- Controller resync successful.\n"); 
				return(Controller::ERR_OK);								// success
			}
			if (flushcnt < 0)													// error from flush
			{	logprintf("Controller::instructionErrMsgResync -- Flush failed: %s .\n",strerror(errno)); 
				continue;														// try again
			}
			logprintf("Controller::instructionErrMsgResync -- %d unexpected bytes of data after good reply.\n", flushcnt); 
			continue;															// try again
		}
		//	Trouble - not in sync.  We may have a backlog of data in transit.
		logprintf( "Controller::instructionErrMsgResync - expected ':', received '%s'\n",reply);
	}
	return(Controller::ERR_RESPONSE_BAD);					// too many retries - fails
}

//
//    Controller::instructionErrMsgTryAgain - try again message
//
//    Returns nothing
//
void Controller::instructionErrMsgTryAgain(const char *instr)
{
	// try again message
	logprintf( "... trying again: %s\n", instr);
	if ( errno != 0 ) {
		logperror("Error status: ");
	}
}

//
//    Controller::instructionErrMsgTryLast - last try message
//
//    Returns nothing
//
void Controller::instructionErrMsgTryLast(const char *instr)
{
	// bad read on last try
	logprintf( "%d tries, giving up on controller command: %s\n", CONTROLLER_TRIES_NUM, instr);
	if ( errno != 0 ) {
		logperror("Final error status: ");
	}
}