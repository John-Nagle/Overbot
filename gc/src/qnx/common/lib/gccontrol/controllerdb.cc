////////////////////////////////////////////////////////////////////////////
//
//    File: controllerdb.cc
//
//    Usage:
//        See controller.h.
//
//    Description:
//        Routines related to the Controller command database.
//
//        Based on the MVP Server cmddb.cc code of August 2003.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        January 2004
//
/////////////////////////////////////////////////////////////////////////////

#include "controller.h"

#define CONTROLLER_CMD_LEN		(82)
#define CONTROLLER_QUERY_LEN	(82)

//
//    Controller::cmdDBInit - initialize the command database
//
//    Returns nothing
//
void Controller::cmdDBInit()
{
	// MUST BE IN SAME ORDER AS controller.h (FIX?)
	//
    // initialize command database
    //    Controller command rules:
    //    	- when enter a parameter => never returns anything
    //		- when don't enter a parameter => may or may not return anything
    //
    // parameter types (pTy; no #defines to save room in table):
    //     0=int
    //     1=float
    //     2=string
    //     3=multiline string, do not append "\r" at end (DL command)
    //
    // in the interest of minimizing the number of commands, if there are two
    // ways to quesry a parameter, only one is included (e.g., DP? is the same
    // as TP).
	//
	//        #                   cmd   param  pTy pRng   Min          Max       queryOK queryStr
	//        --                  ----  -----  --- -----  ----------   ---------  -----  -------
	// motor off
	cmdDBLoad(Controller::CMD_MO, "MO", false, 0, false,           0,          0, false, NULL);
	// define position
	cmdDBLoad(Controller::CMD_DP, "DP", true,  0, true, -2147483648.0, 2147483647.0, false, NULL); // quad counts
	cmdDBLoad(Controller::CMD_DE, "DE", true,  0, true, -2147483648.0, 2147483647.0, false, NULL); // quad counts
    // limits - position range	
	cmdDBLoad(Controller::CMD_BL, "BL", true,  0, true, -2147483648.0, 2147483647.0, true, "BL?"); // quad counts
	cmdDBLoad(Controller::CMD_FL, "FL", true,  0, true, -2147483648.0, 2147483647.0, true, "FL?"); // quad counts
	cmdDBLoad(Controller::CMD_ER, "ER", true,  0, true,            1,      32767, true, "ER?"); // quad counts
    // actuals
	cmdDBLoad(Controller::CMD_TP, "TP", false, 0, false,           0,          0, true, "TP");  // quad counts
	cmdDBLoad(Controller::CMD_RP, "RP", false, 0, false,           0,          0, true, "RPX"); // quad counts
	cmdDBLoad(Controller::CMD_TE, "TE", false, 0, false,           0,          0, true, "TE");  // quad counts (-2147483647 to 2147483647)
	cmdDBLoad(Controller::CMD_TD, "TD", false, 0, false,           0,          0, true, "TD");  // quad counts
	cmdDBLoad(Controller::CMD_TV, "TV", false, 0, false,           0,          0, true, "TV");  // quad counts/sec
	cmdDBLoad(Controller::CMD_TT, "TT", false, 0, false,           0,          0, true, "TT");  // volts (-9.998 to 9.998)
    // control loop parameters	
	cmdDBLoad(Controller::CMD_AF, "AF", true,  0, true,            0,          1, true, "AF?"); // boolean
	cmdDBLoad(Controller::CMD_DV, "DV", true,  0, true,            0,          1, true, "DV?"); // boolean
	cmdDBLoad(Controller::CMD_KP, "KP", true,  1, true,            0,   1023.875, true, "KP?"); // UNITS
	cmdDBLoad(Controller::CMD_KI, "KI", true,  1, true,            0,   2047.875, true, "KI?"); // UNITS
	cmdDBLoad(Controller::CMD_KD, "KD", true,  1, true,            0,   4095.875, true, "KD?"); // UNITS
	cmdDBLoad(Controller::CMD_IL, "IL", true,  1, true,            0,     9.9988, true, "IL?"); // volts
	cmdDBLoad(Controller::CMD_TL, "TL", true,  1, true,            0,      9.998, true, "TL?"); // volts 
	cmdDBLoad(Controller::CMD_OF, "OF", true,  1, true,       -9.998,      9.998, true, "OF?"); // volts
	cmdDBLoad(Controller::CMD_TM, "TM", true,  0, true,          250,      20000, true, "TM?"); // usec
	// absolute targets (application program)	
	cmdDBLoad(Controller::CMD_TARGPOSN, "TargPosn=", true,  1, false,  0,      0, true, "TargPosn="); // position units
	cmdDBLoad(Controller::CMD_TARGVEL,  "TargVel=",  true,  1, false,  0,      0, true, "TargVel=");  // velocity units
	cmdDBLoad(Controller::CMD_TARGVEPS, "TargVeps=", true,  1, false,  0,      0, true, "TargVeps="); // velocity units
	cmdDBLoad(Controller::CMD_TARGACC,  "TargAcc=",  true,  1, false,  0,      0, true, "TargAcc=");  // acceleration units
	cmdDBLoad(Controller::CMD_TARGDEC,  "TargDec=",  true,  1, false,  0,      0, true, "TargDec=");  // deceleration units
	cmdDBLoad(Controller::CMD_TARGNEW,  "TargNew=1", false, 0, false,  0,      0, false, NULL);
    // absolute/relative profile (indep axis posn)
	cmdDBLoad(Controller::CMD_PA, "PA", true,  0, true, -2147483648.0, 2147483647.0, true, "PA?"); // quad counts
	cmdDBLoad(Controller::CMD_PR, "PR", true,  0, true, -2147483648.0, 2147483647.0, true, "PR?"); // quad counts 
	cmdDBLoad(Controller::CMD_SP, "SP", true,  0, true,            0,   12000000, true, "SP?"); // quad counts/sec
	cmdDBLoad(Controller::CMD_AC, "AC", true,  0, true,         1024,   67107840, true, "AC?"); // quad counts/sec^2
	cmdDBLoad(Controller::CMD_DC, "DC", true,  0, true,         1024,   67107840, true, "DC?"); // quad counts/sec^2
    // constant velocity (indep jogging)
	cmdDBLoad(Controller::CMD_JG, "JG", true,  0, true,    -12000000,   12000000, true, "JG?"); // quad counts/sec
    // switches
	cmdDBLoad(Controller::CMD_CN, "CN", true,  2, false,           0,          0, false, NULL); // high/low
	cmdDBLoad(Controller::CMD_FE, "FE", false, 0, false,           0,          0, false, NULL);
	cmdDBLoad(Controller::CMD_LF,   "", false, 0, false,           0,          0, true, "MG _LF"); // boolean
	cmdDBLoad(Controller::CMD_LR,   "", false, 0, false,           0,          0, true, "MG _LR"); // boolean
    // motion	 
	cmdDBLoad(Controller::CMD_BG, "BG", false, 0, false,           0,          0, true, "MG _BG"); // boolean
	cmdDBLoad(Controller::CMD_SH, "SH", false, 0, false,           0,          0, false, NULL);
    // motion abort	
	cmdDBLoad(Controller::CMD_ST, "ST", false, 0, false,           0,          0, false, NULL);
	cmdDBLoad(Controller::CMD_AB, "AB", false, 0, false,           0,          0, false, NULL);
    // errors	 
	cmdDBLoad(Controller::CMD_OE, "OE", true,  0, true,            0,          1, true, "OE?"); // boolean
	cmdDBLoad(Controller::CMD_TC, "TC", false, 0, false,           0,          0, true, "TC1"); // num msg
	cmdDBLoad(Controller::CMD_SC, "SC", false, 0, false,           0,          0, true, "SC");  // code number
    // analog inputs	
	// a bit of a kludge to append 1 & 2, but fits into mold easily that way
	cmdDBLoad(Controller::CMD_AN1,"AN1", false, 0, false,           0,          0, true, "MG @AN[1]"); // volts?
	cmdDBLoad(Controller::CMD_AN2,"AN2", false, 0, false,           0,          0, true, "MG @AN[2]"); // volts?
    // digital inputs/outputs	
	cmdDBLoad(Controller::CMD_IN1,"IN1", false, 0, false,           0,          0, true, "MG @IN[1]"); // boolean
	cmdDBLoad(Controller::CMD_IN2,"IN2", false, 0, false,           0,          0, true, "MG @IN[2]"); // boolean
	cmdDBLoad(Controller::CMD_IN3,"IN3", false, 0, false,           0,          0, true, "MG @IN[3]"); // boolean
	cmdDBLoad(Controller::CMD_IN4,"IN4", false, 0, false,           0,          0, true, "MG @IN[4]"); // boolean
	cmdDBLoad(Controller::CMD_IN5,"IN5", false, 0, false,           0,          0, true, "MG @IN[5]"); // boolean
	cmdDBLoad(Controller::CMD_IN6,"IN6", false, 0, false,           0,          0, true, "MG @IN[6]"); // boolean
	cmdDBLoad(Controller::CMD_IN7,"IN7", false, 0, false,           0,          0, true, "MG @IN[7]"); // boolean
	cmdDBLoad(Controller::CMD_TI, "TI", false, 0, false,           0,          0, true, "TI");  // boolean
	cmdDBLoad(Controller::CMD_SB, "SB", true,  0, true,            1,          3, true, "OP?"); // boolean
	cmdDBLoad(Controller::CMD_CB, "CB", true,  0, true,            1,          3, true, "OP?"); // boolean
	cmdDBLoad(Controller::CMD_OP, "OP", true,  0, true,            0,          7, true, "OP?"); // boolean
    // application programming	
	cmdDBLoad(Controller::CMD_DL, "DL\r", true, 3, false,           0,          0, false, NULL);	// erase, then load program, end with "\\"
	cmdDBLoad(Controller::CMD_DLAPPEND, "DL #\r", true, 3, false,           0,          0, false, NULL);	// append, then load program, end with "\\"
	cmdDBLoad(Controller::CMD_ED,"", false, 0, false,        	   0,      	    0, true, "MG _ED"); // line number of program halt
	cmdDBLoad(Controller::CMD_LS, "LS", true, 0, false,           2,          0, true, "LS"); // line numbers optional
	cmdDBLoad(Controller::CMD_XQ, "XQ",  true, 2, false,           0,          0, true, "MG _XQ"); // line num or -1
	cmdDBLoad(Controller::CMD_HX, "HX", false, 0, false,           0,          0, false, NULL);
    // status	
	cmdDBLoad(Controller::CMD_TS, "TS", false, 0, false,           0,          0, true, "TS");  // boolean
	cmdDBLoad(Controller::CMD_TH, "TH", false, 0, false,           0,          0, true, "TH");  // string
	cmdDBLoad(Controller::CMD_RV, "RV", false, 0, false,           0,          0, true, "^R^V");// string?
    // EEprom	
	cmdDBLoad(Controller::CMD_BN, "BN", false, 0, false,           0,          0, false, NULL); 
	cmdDBLoad(Controller::CMD_BP, "BP", false, 0, false,           0,          0, false, NULL);
	cmdDBLoad(Controller::CMD_RS, "RS", false, 0, false,           0,          0, false, NULL);
}

//
//    Controller::cmdDBLoad - store entry in the command database
//
//    Returns nothing
//
void Controller::cmdDBLoad(Controller::Cmd cmdNum, char *cmdStr, bool paramOK,
                           int paramType, bool paramRange, float paramMin, 
                           float paramMax, bool queryOK, char *queryStr)
{	
	if (strlen(cmdStr) > (CONTROLLER_CMD_LEN-1) ) {
        fprintf(stderr,
               "Controller::cmdDBLoad - command string longer than allowed.\n");
		abort();			// PANIC - program damaged or misbuilt
	}
	//	Compiled-in command table must be in command number order for indexing.  Validate this.
	if (cmdDB.size() != size_t(cmdNum))					// if out of sync
	{	fprintf(stderr, "Controller::cmdDBLoad - command number %d (%s)out of sequence in table.\n",
			cmdNum, (cmdStr ? cmdStr : "???"));
		abort();
	}

	// create a record to store this data
	cmdRec *r = new cmdRec;

	strncpy(r->cmdStr, cmdStr, CONTROLLER_CMD_LEN);
    r->paramOK      = paramOK;
    r->paramType    = paramType;
    r->paramRange   = paramRange;
    r->paramMin     = paramMin;
    r->paramMax     = paramMax;
    r->queryOK      = queryOK;
    if ( queryOK ) {
    	strncpy(r->queryStr, queryStr, CONTROLLER_QUERY_LEN);
    }
    
    // append record to database
    cmdDB.push_back(*r);
}