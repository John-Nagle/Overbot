/* -*- indent-tabs-mode:T; c-basic-offset:8; tab-width:8; -*- vi: set ts=8:
 * $Id: sonar_messaging.h,v 1.1 2003/10/22 04:52:30 khianhao Exp $
 *
 * The list of messages that the sonar server uses
 *
 * Khian Hao Lim
 * Team Overbot
 * October, 2003
 */

#ifndef SONAR_MESSAGING
#define SONAR_MESSAGING

#include "messaging.h"

//a name wrapper so that these enum are not in global namespace
//use SONAR_MSG::INT_ERR for example to refer to the number
class SONAR_MSG {
 public:
    //List of possible errors that the GPSINS server can describe in a reply
    enum Err{
        OK,             //when everything is fine and this packet is valid
        INT_ERR,        //internal error in SONAR server

    };
};

// The Msg struct that subclasses the general MsgBase
// used for request
struct SONARMsgReq: public MsgBase {
    static const uint32_t k_msgtype = char4('S','O','R','Q');
};

#define NUM_SONARS 16

// The Msg struct that subclasses the general MsgBase
// used for reply
struct SONARMsgRep: public MsgBase {
    static const uint32_t k_msgtype = char4('S','O','R','P');

    SONAR_MSG::Err err;      //see above

    float dist[NUM_SONARS];

    //TODO
    //describe the sonar location here
};

#endif //SONAR_MESSAGING
