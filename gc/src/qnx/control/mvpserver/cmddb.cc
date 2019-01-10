////////////////////////////////////////////////////////////////////////////
//
//    File: cmddb.cc
//
//    Usage:
//        See mvpserver.h.
//
//    Description:
//        Routines related to the MVP Server command database.
//
//        FIX - make sure all special cases are handled.
//        FIX - make sure handling all ranges of parameters.
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
//    MVPServer_::cmdDBInit - initialize the command database
//
//    Returns nothing
//
void MVPServer_::cmdDBInit()
{
	// MUST BE IN SAME ORDER AS mvpserver.h!!! FIX so it doesn't have to be
	//
    // initialize command database
    //    MVP command rules:
    //    	- when enter a parameter => never returns anything
    //		- when don't enter a parameter => may or may not return anything
	//
	//                                                                 neg#   vRtn
	//        #                 str   pOK    pRng   Mi Ma vRtn   neg#s digits cksum
	//        --                ----  -----  -----  -- -- -----  ----- -----  -----
    cmdDBLoad(MVPServer::CMD_EN, "EN", false, false, 0, 0, false, false, 0,true);
    cmdDBLoad(MVPServer::CMD_DI, "DI", false, false, 0, 0, false, false, 0,true);
    cmdDBLoad(MVPServer::CMD_RN, "RN", false, false, 0, 0, false, false, 0,true);

    cmdDBLoad(MVPServer::CMD_LA, "LA", true,  false, 0, 0, true,   true, 8,true);
    cmdDBLoad(MVPServer::CMD_LR, "LR", true,  false, 0, 0, true,   true, 8,true);
    cmdDBLoad(MVPServer::CMD_SP, "SP", true,  false, 0, 0, true,   true, 4,true);
    cmdDBLoad(MVPServer::CMD_AC, "AC", true,  false, 0, 0, true,   true, 4,true);
    cmdDBLoad(MVPServer::CMD_DC, "DC", true,  false, 0, 0, true,   true, 4,true);

    cmdDBLoad(MVPServer::CMD_V,  "V",  true,  false, 0, 0, true,   true, 4,true);

    cmdDBLoad(MVPServer::CMD_POS,"POS",false, false, 0, 0, true,   true, 8,true);
    cmdDBLoad(MVPServer::CMD_EP, "EP", false, false, 0, 0, true,   true, 8,true);
    cmdDBLoad(MVPServer::CMD_AS, "AS", false, false, 0, 0, true,   true, 4,true);
    cmdDBLoad(MVPServer::CMD_ERR,"ERR",false, false, 0, 0, true,   true, 4,true);

    cmdDBLoad(MVPServer::CMD_N,  "N",  true,  false, 0, 0, true,   true, 4,true); // only rtns +
    cmdDBLoad(MVPServer::CMD_LL, "LL", true,  false, 0, 0, true,  false, 0,true); // only rtns +
    cmdDBLoad(MVPServer::CMD_LLM,"LL -",false,false, 0, 0, true,  false, 0,true);
    cmdDBLoad(MVPServer::CMD_SA, "SA", true,   true, 0, 2, true,  false, 0,true);

    cmdDBLoad(MVPServer::CMD_SM, "SM", true,  true,  0,1000,true, false, 0,true);
    cmdDBLoad(MVPServer::CMD_ANO,"ANO",true,  true, 2047,4015,true,false,0,true);

	//                                                                 neg#   vRtn
	//        #                 str   pOK    pRng   Mi Ma vRtn   neg#s digits cksum
	//        --                ----  -----  -----  -- -- -----  ----- -----  -----
    cmdDBLoad(MVPServer::CMD_HO, "HO", true,  false, 0, 0, false, false, 0,true);
    cmdDBLoad(MVPServer::CMD_HA, "HA", true,   true, 0, 1, false, false, 0,true);
    cmdDBLoad(MVPServer::CMD_HP, "HP", true,   true, 0, 1, false, false, 0,true);
    cmdDBLoad(MVPServer::CMD_HS, "HS", false, false, 0, 0,  true, false, 0,true);
    cmdDBLoad(MVPServer::CMD_HF, "HF", true,   true, 0, 2,  true, false, 0,true);
    cmdDBLoad(MVPServer::CMD_AE, "AE", true,   true, 0, 1,  true, false, 0,true);

    cmdDBLoad(MVPServer::CMD_M,  "M",  false, false, 0, 0, false, false, 0,true);

    cmdDBLoad(MVPServer::CMD_AB, "AB", false, false, 0, 0, false, false, 0,true);
    cmdDBLoad(MVPServer::CMD_AD, "AD", true,  false, 0, 0,  true,  true, 4,true);
    cmdDBLoad(MVPServer::CMD_AA, "AA", true,   true, 0, 2,  true, false, 0,true);

    cmdDBLoad(MVPServer::CMD_FD, "FD", true,  false, 0, 0,  true,  true, 4,true);
    cmdDBLoad(MVPServer::CMD_FDT,"FDT",true,  false, 0, 0,  true,  true, 4,true);
    cmdDBLoad(MVPServer::CMD_FA, "FA", true,   true, 0, 2,  true, false, 0,true);

    cmdDBLoad(MVPServer::CMD_POR,"POR",true,  false, 0, 0,  true, false, 0,true); // won't let neg
    cmdDBLoad(MVPServer::CMD_I,  "I",  true,  false, 0, 0,  true,  true, 4,true);
    cmdDBLoad(MVPServer::CMD_DER,"DER",true,  false, 0, 0,  true,  true, 4,true);

	//                                                                 neg#   vRtn
	//        #                 str   pOK    pRng   Mi Ma vRtn   neg#s digits cksum
	//        --                ----  -----  -----  -- -- -----  ----- -----  -----
    cmdDBLoad(MVPServer::CMD_SR, "SR", true,  false, 0, 0,  true, false, 0,true); // won't let neg

    cmdDBLoad(MVPServer::CMD_ANI,"ANI",false, false, 0, 0,  true, false, 0,true);
    cmdDBLoad(MVPServer::CMD_EAI,"EAI",false, false, 0, 0,  true, false, 0,true);
    cmdDBLoad(MVPServer::CMD_ANM,"ANM",true,  true,  0, 1,  true, false, 0,true);
	
    cmdDBLoad(MVPServer::CMD_DACA,"DACA",true,true,  0, 1,  true, false, 0,true);
    cmdDBLoad(MVPServer::CMD_PI, "PI", true,  true,  1, 4,  true, false, 0,false); // special case return w/p
    cmdDBLoad(MVPServer::CMD_PO, "PO", true,  false, 0, 0, false, false, 0,true); // special syntax param
    cmdDBLoad(MVPServer::CMD_XST,"XST",false, false, 0, 0,  true, false, 0,false);

    cmdDBLoad(MVPServer::CMD_ME, "ME", true,  false, 0, 0,  true, false, 0,true);
    cmdDBLoad(MVPServer::CMD_MS, "MS", false, false, 0, 0,  true, false, 0,true);


    cmdDBLoad(MVPServer::CMD_SD, "SD", true,  true,  0, 2000,true,false, 0,true);
    cmdDBLoad(MVPServer::CMD_OK, "OK", true,  true,  0, 1,  true, false, 0,true);
    cmdDBLoad(MVPServer::CMD_CK, "CK", true,  true,  0, 1,  true, false, 0,true);
    cmdDBLoad(MVPServer::CMD_CX, "CX", false, false, 0, 0, false, false, 0,true);

    cmdDBLoad(MVPServer::CMD_ST, "ST", false, false, 0, 0,  true, false, 0,true);
    cmdDBLoad(MVPServer::CMD_FV, "FV", false, false, 0, 0,  true, false, 0,false);// special case syntax resp

    cmdDBLoad(MVPServer::CMD_EEPSAV,"EEPSAV",false,false,0,0,false,false,0,true);
    cmdDBLoad(MVPServer::CMD_EEBOOT,"EEBOOT",true, true, 0,1, true,false,0,true);
}

//
//    MVPServer_::cmdDBLoad - store entry in the command database
//
//    Returns nothing
//
void MVPServer_::cmdDBLoad(MVPServer::Cmd cmdNum, char *cmdStr, bool paramOK,
                           bool paramRange, int paramMin, int paramMax,
                           bool valueRtn, bool valueNeg, int valueNibbles,
                           bool valueCksum)
{
	cmdRec *r;
	
	if ( verbose && (strlen(cmdStr) > (MVPSERVER_CMD_LEN-1)) ) {
        fprintf(stderr,
               "MVPServer_::cmdDBLoad - command string longer than allowed.\n");
	}
	
	// create a record to store this data
	r = new cmdRec;

	strncpy(r->cmdStr, cmdStr, MVPSERVER_CMD_LEN);
	
    r->paramOK      = paramOK;
    r->paramRange   = paramRange;
    r->paramMin     = paramMin;
    r->paramMax     = paramMax;
    r->valueRtn     = valueRtn;
    r->valueNeg     = valueNeg;
    r->valueNibbles = valueNibbles;
    r->valueCksum   = valueCksum;
    
    // append record to database
    cmdDB.push_back(*r);
}

//
//    MVPServer_::cmdDBStr2Enum - finds the enum corresponding to the string
//
//    Returns enum corresponding to command string, -1 otherwise
//
bool MVPServer_::CmdDBStr2Enum(char *str, MVPServer::Cmd *cmd)
{
	int i = 0;
	
	for ( cmdDBi = cmdDB.begin() ; cmdDBi != cmdDB.end() ; cmdDBi++ ) {
		if ( strcmp((*cmdDBi).cmdStr, str) == 0 ) {
			*cmd = (MVPServer::Cmd) i;
			return true;
		}
		i++;
	}
	
	return false;
}