////////////////////////////////////////////////////////////////////////////
//
//    File: checksum.cc
//
//    Usage:
//        See mvpserver.h.
//
//    Description:
//        Routines related to the MVP Server check sums.
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
//    MVPServer_::checksumCalc - calculate a checksum for an instruction
//
//    Returns nothing
//
void MVPServer_::checksumCalc(char *instr, char *nibbleL, char *nibbleR)
{
	int charSum = 0;
	int instrLen = strlen(instr);
	
	for ( int i=0; i<instrLen; i++ ) {
		charSum += instr[i];
	}
	*nibbleR = (charSum & 0x0F) + 0x30;
	*nibbleL = ((charSum & 0xF0) >> 4) + 0x30;
}

//
//    MVPServer_::checksumAppend - append a checksum to the instruction
//
//    Assumes there's enough room in "instr" to append a checksum.
//
//    Returns nothing
//
void MVPServer_::checksumAppend(char *instr)
{
	char nibbleL, nibbleR;
	
	checksumCalc(instr, &nibbleL, &nibbleR);
	sprintf(instr, "%s#%c%c", instr, nibbleL, nibbleR);
}

//
//    MVPServer::checksumOK - check if checksum of reply okay
//
//    Returns true if checksum okay, false otherwise
//
bool MVPServer_::checksumOK(char *reply)
{
	char instr[MVPSERVER_INSTR_LEN];
	char *poundSign;
	char nibbleLinstr = ' ', nibbleRinstr = ' ';
	char nibbleL, nibbleR;

    strncpy(instr, reply, MVPSERVER_INSTR_LEN);
    if ( (poundSign = strchr(instr, '#')) != NULL ) {
    	poundSign[0] = '\0';
    	nibbleLinstr = poundSign[1];
    	nibbleRinstr = poundSign[2];
    }
	//sscanf(reply, "%s#%c%c", instr, &nibbleLinstr, &nibbleRinstr);
	checksumCalc(instr, &nibbleL, &nibbleR);
	
	return (nibbleLinstr == nibbleL) && (nibbleRinstr == nibbleR);
}
