//
//	drvbus.cpp  --  low level driver for OHCI adapter, bus reset events
//
//	John Nagle
//	Animats
//	March, 2003
//
//
//	Copyright 2005 by John Nagle
//	999 Woodland Avenue
//	Menlo Park, CA  94025
//
//	This program is free software; you can redistribute it and/or modify
//	it under the terms of the GNU General Public License as published by
//	the Free Software Foundation; either version 2 of the License, or
//	(at your option) any later version.

//	This program is distributed in the hope that it will be useful,
//	but WITHOUT ANY WARRANTY; without even the implied warranty of
//	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//	GNU General Public License for more details.

//	You should have received a copy of the GNU General Public License
//	along with this program; if not, write to the Free Software
//	Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//
#include <errno.h>
#include "mindreadyforcpp.h"
#include "llaError.h"
#include "drvadapter.h"
#include "utilities.h"



//	All of these return POSIX error codes.

//
//	busReset -- cause a bus reset
//
//	Returns POSIX error code.
//
int Drvadapter::busReset(bool shortBusReset)
{
    if (shortBusReset)
    {	return(llacheck(llaShortBusReset (getlla()),"Short bus reset"));	}
    else
    {	return(llacheck(llaBusReset (getlla()),"Bus reset")); }
}
//
//	busResetRecvStart  -- start receiving bus reset packets
//
//	Called at startup.
//
int Drvadapter::busResetRecvStart()
{	return(llacheck(llaSetConfigInCb(getlla(), selfidcallbackstatic, this),"set bus receive callback")); }
//
//	busResetRecvStop  -- start receiving bus reset packets
//
//	Called at shutdown.
//	
int Drvadapter::busResetRecvStop()
{	return(llacheck(llaSetConfigInCb(getlla(), 0, this),"clear bus receive callback")); }
//
//	selfidcallback  --  Callback on bus reset
//
//	Obtains info about all the nodes on the bus.
//
void Drvadapter::selfidcallback(const llaConfigIn_t* pConfigIn)
{
    unsigned portIndex;					      /* Self-Id port index           */
    unsigned nbOfPort;					      /* Number of port               */
    uint8_t currentPhyId;				      /* Current physical ID          */
    uint8_t lastPhyId;					      /* Last physical ID read        */
    unsigned extendedIndex;				      /* Extended sequence index      */

    SelfIdPks_t  selfIdPacket;                /* Drv self id struct           */
    unsigned     bufLen;                      /* Length of the selfid info    */
    uint32_t    *pSelfIdBlock;                /* Quadlet Block info           */

    /* Init decoding variables */
    bufLen        = pConfigIn->listLength;
    pSelfIdBlock  = pConfigIn->list;
	//	Process all self ID quadlets per IEEE spec.
    for (unsigned blockIndex = 0, selfIdIndex = 0;
    			blockIndex < bufLen && selfIdIndex < IEEE1394_NODE_NUMBER_MAX + 1;
    			)
    {	//	Bus reset data validation
        /* Test that next quadlet is the complement of the current one */
        if (pSelfIdBlock[blockIndex] != ~pSelfIdBlock[blockIndex+1])
        {
            selfIdPacket.numOfBlock = selfIdIndex;
            logprintf("ERROR: bad self ID quadlet.\n");
            return;
        }

        /* Test the self-Id packet identifier */
        if (((pSelfIdBlock[blockIndex] >> IEEE1394_SELFID_ID_SHFT) &
                IEEE1394_SELFID_ID_MASK) != IEEE1394_SELFID_PKID)
        {
            selfIdPacket.numOfBlock = selfIdIndex;
            logprintf("ERROR: bad self ID identifier.\n");
            return;
        }

        /* Test the bit 23, must be set to 0 */
        if (pSelfIdBlock[blockIndex] & (IEEE1394_NEXT_MASK<<IEEE1394_NEXT_SHFT))
        {
            selfIdPacket.numOfBlock = selfIdIndex;
            logprintf("ERROR: bad self ID bit.\n");
            return;
        }
		//	Passed validation; will accept.
        /* Get field values */
        currentPhyId = (pSelfIdBlock[blockIndex] >> IEEE1394_PHY_ID_SHFT) & IEEE1394_PHY_ID_MASK;
        lastPhyId = currentPhyId;

        selfIdPacket.block[selfIdIndex].phyId = currentPhyId;

        selfIdPacket.block[selfIdIndex].linkActive = (pSelfIdBlock[blockIndex] >>
                IEEE1394_LINK_ACTIVE_SHFT) &
                IEEE1394_LINK_ACTIVE_MASK;

        selfIdPacket.block[selfIdIndex].gapCount = (pSelfIdBlock[blockIndex] >>
                IEEE1394_GAP_COUNT_SHFT) &
                IEEE1394_GAP_COUNT_MASK;

        selfIdPacket.block[selfIdIndex].phySpeed = (pSelfIdBlock[blockIndex] >>
                IEEE1394_PHY_SPEED_SHFT) &
                IEEE1394_PHY_SPEED_MASK;

        selfIdPacket.block[selfIdIndex].phyDelay = (pSelfIdBlock[blockIndex] >>
                IEEE1394_PHY_DELAY_SHFT) &
                IEEE1394_PHY_DELAY_MASK;

        selfIdPacket.block[selfIdIndex].contender = (pSelfIdBlock[blockIndex] >>
                IEEE1394_CONTENDER_SHFT) &
                IEEE1394_CONTENDER_MASK;

        selfIdPacket.block[selfIdIndex].powerClass = (pSelfIdBlock[blockIndex] >>
                IEEE1394_POWER_CLASS_SHFT) &
                IEEE1394_POWER_CLASS_MASK;
        nbOfPort = 0;
        for (portIndex = 0; portIndex < 3; portIndex++)
        {
            selfIdPacket.block[selfIdIndex].port[portIndex] = (pSelfIdBlock[blockIndex] >>
                    (IEEE1394_PORT0_SHFT - portIndex * 2)) & IEEE1394_PORT0_MASK;

            if (selfIdPacket.block[selfIdIndex].port[portIndex] != IEEE1394_NOT_PRESENT)
            {
                nbOfPort++;
            }
        }

        selfIdPacket.block[selfIdIndex].initiatedReset = (pSelfIdBlock[blockIndex] >>
                IEEE1394_INIT_RESET_SHFT) &
                IEEE1394_INIT_RESET_MASK;

        /* Extended self-Id packets for this node */
        extendedIndex = 0;
        while (pSelfIdBlock[blockIndex] & IEEE1394_MORE_MASK)
        {
            /* Test that next quadlet is the complement of the current one */
            blockIndex += 2;
            if (pSelfIdBlock[blockIndex] != ~pSelfIdBlock[blockIndex+1])
            {
                selfIdPacket.numOfBlock = selfIdIndex;
                return;
            }

            /* Test the self-Id packet identifier */
            if (((pSelfIdBlock[blockIndex] >> IEEE1394_SELFID_ID_SHFT) &
                    IEEE1394_SELFID_ID_MASK) != IEEE1394_SELFID_PKID)
            {
                selfIdPacket.numOfBlock = selfIdIndex;
                return;
            }

            /* Test the bit 23, must be set to 1 */
            if (!(pSelfIdBlock[blockIndex] & (IEEE1394_NEXT_MASK <<
                                              IEEE1394_NEXT_SHFT)))
            {
                selfIdPacket.numOfBlock = selfIdIndex;
                return;
            }

            /* The physical ID must be the same */
            currentPhyId = (pSelfIdBlock[blockIndex] >> IEEE1394_PHY_ID_SHFT) &
                           IEEE1394_PHY_ID_MASK;
            if (lastPhyId != currentPhyId)
            {
                selfIdPacket.numOfBlock = selfIdIndex;
                return;
            }

            /* Extended self-Id sequence must be continuous */
            if (((pSelfIdBlock[blockIndex] >> IEEE1394_EXTENDED_SHFT) &
                    IEEE1394_EXTENDED_MASK) != extendedIndex)
            {
                selfIdPacket.numOfBlock = selfIdIndex;
                return;
            }
            extendedIndex++;

            /* Take status of port 3 to 26 */
            for (portIndex = 0; portIndex < 8; portIndex++)
            {
                selfIdPacket.block[selfIdIndex].port[portIndex] =
                    (pSelfIdBlock[blockIndex] >>
                     (IEEE1394_PORTA_SHFT -
                      portIndex * 2)) &
                    IEEE1394_PORTA_MASK;

                if(	selfIdPacket.block[selfIdIndex].port[portIndex] !=
                        IEEE1394_NOT_PRESENT)
                {
                    nbOfPort++;
                }
            }
        }

        /* Get the number of port that were found */
        selfIdPacket.block[selfIdIndex].numberOfPorts = nbOfPort;

        /* One more self-Id packet was successfully decoded */
        selfIdIndex++;
    		selfIdPacket.numOfBlock = selfIdIndex;

        /* Next self-Id packet */
        blockIndex += 2;
    }
	//	Pass callback upwards
	selfidcallback(selfIdPacket);
}
