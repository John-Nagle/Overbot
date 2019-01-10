/* -*- indent-tabs-mode:T; c-basic-offset:8; tab-width:8; -*- vi: set ts=8:
 * $Id: water_messaging.h,v 1.1 2003/10/22 04:52:30 khianhao Exp $
 *
 * The list of messages that the water sensor server uses
 *
 * Khian Hao Lim
 * Team Overbot
 * October, 2003
 */

#ifndef WATER_MESSAGING
#define WATER_MESSAGING

#include "messaging.h"

//a name wrapper so that these enum are not in global namespace
//use WATER_MSG::INT_ERR for example to refer to the number
class WATER_MSG {
 public:
    //List of possible errors that the GPSINS server can describe in a reply
    enum Err{
        OK,             //when everything is fine and this packet is valid
        INT_ERR,        //internal error in SONAR server

    };
};

// The Msg struct that subclasses the general MsgBase
// used for request
struct WATERMsgReq: public MsgBase {
    static const uint32_t k_msgtype = char4('W','A','R','Q');
};

#define NUM_WATER_SENSORS 2

// The Msg struct that subclasses the general MsgBase
// used for reply
struct WATERMsgRep: public MsgBase {
    static const uint32_t k_msgtype = char4('W','A','R','P');

    WATER_MSG::Err err;      //see above

    //the size below is subject to change after testing
    int dist[NUM_WATER_SONARS];

    //TODO
    //describe the water sensor location here
};

#endif //WATER_MESSAGING
