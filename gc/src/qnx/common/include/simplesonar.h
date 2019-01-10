//
//	simplesonar.h  -- simple messages from a simple sonar
//
//	John Nagle
//	Team Overbot
//	August, 2005
//
#ifndef  SIMPLESONAR_H
#define SIMPLESONAR_H

#include "messaging.h"
#include "faultcodes.h"
//
//	SonarObstacleMsgReq  -- simple obstacle report
//
//	Reports to MAP server.
struct SonarObstacleMsgReq	: public MsgBase {
    static const uint32_t k_msgtype = char4('S','O','O','B');
    Fault::Faultcode m_fault;								// fault code if problem
    bool m_frontobstacle;									// near obstacle in front		
    bool m_rearobstacle;									// near obstacle in back
};
#endif // SIMPLESONAR_H
