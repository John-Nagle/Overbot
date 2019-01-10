/* -*- indent-tabs-mode:T; c-basic-offset:8; tab-width:8; -*- vi: set ts=8:
 * $Id: gpsins_messaging.h,v 1.16 2005/08/19 03:40:39 nagle Exp $
 *
 * The list of messages that the gpsins server supports
 *
 * Khian Hao Lim
 * Team Overbot
 * October, 2003
 */

#ifndef GPSINS_MESSAGING
#define GPSINS_MESSAGING

#include "messaging.h"

//a name wrapper so that these enum are not in global namespace
//use GPSINS_MSG::INT_ERR for example to refer to the number
class GPSINS_MSG
{
public:
    //List of possible errors that the GPSINS server can describe in a reply
    enum Err{
        OK,             //when everything is fine and this packet is valid
        INT_ERR,        //internal error in GPSINS server
        INITIALIZATION, //GPS and fusednav in initialization
    };

    enum SolStatusEnum{
        SOL_COMPUTED =		0,			//solution has been computed, gpsins server position relying heavily on gps
        INSUFFICIENT_OBS=	1,			//gps solution incomplete, gpsins server position relying on other means, should still be relied on
        NO_CONVERGENCE=		2,
        SINGULARITY=		3,
        COV_TRACE=			4,
        TEST_DIST=			5,
        COLD_START=			6,
        V_H_LIMIT=			7,
        VARIANCE=			8,
        RESIDUALS=			9,
        DELTA_POS=			10,
        NEGATIVE_VAR=		11,
        RESERVED=			12,
        INTEGRITY_WARNING=	13,
    };

    enum PosVelTypeEnum{
        NONE=				0,
        FIXEDPOS=			1,
        FIXEDHEIGHT=		2,
        FIXEDVEL=			3,
        DOPPLER_VELOCITY=	8,
        SINGLE=				16,			//purely non differential service
        PSRDIFF=			17,
        WAAS=				18,				//third best service, between omnistar and single
        OMNISTAR=			20,			//second best service, between hp and waas
        L1_FLOAT=			32,
        IONOFREE_FOAT=		33,
        NARROW_FLOAT=		34,
        L1_INT=				48,
        WIDE_INT=			49,
        NARROW_INT=			50,
        RTK_DIRECT_INS=		51,
        INS=				52,
        OMNISTAR_HP	=	64	,			//the best decimeter level service
    } ;

};

// The Msg struct that subclasses the general MsgBase
// used for request
struct GPSINSMsgReq: public MsgBase
{
    static const uint32_t k_msgtype = char4('G','P','R','Q');
};

//uncertainty k refers to the fact that our filtered estimates
//are empirically 95% within +- k m away from the truth

// The Msg struct that subclasses the general MsgBase
// used for reply
//
//	Reply to GPSINSMsgReq.  Also used in log files.
//
struct GPSINSMsgRep
{
	uint64_t timestamp;										// nanoseconds since epoch
    GPSINS_MSG::Err err;     								//	see above
    GPSINS_MSG::SolStatusEnum posStat;           // position status
    GPSINS_MSG::PosVelTypeEnum posType;    // pos type

	//	Postional information. 
	//	North, East, Down coordinate system is relative to current GPS coord system origin.
	//	Distances are in meters. Angles are in degrees (not radians).
    double llh[3];		// Latitude(deg), Longitude(deg) Height(m)
    double pos[3];		// North, East, Down (m)
    float  unc[3]; 		// Uncertainty (North, East Down)  (m)
    float  vel[3]; 			// Velocity (North, East, Down)  (m/s)
    float  acc[3]; 		// Acceleration (North, East Down) (m/s^2)
    float  rpy[3];			// Roll, Pitch, Yaw (deg)
    float  pqr[3];			// Roll Rate, Pitch Rate, Yaw Rate (deg/sec)
};
//
//	GPSINSBasepoint  -- set the basepoint for LLH -> XY conversion
//
//	This becomes the new basepoint, and XY values will be computed from it.
//
//	Usually, the first waypoint is the basepoint.
//
struct GPSINSBasepoint: public MsgBase
{
    static const uint32_t k_msgtype = char4('G','P','B','P');
	double llh[3];											// Latitude(deg), Longitude(deg) Height(m)
};
//
//	GPSINSGetBasepoint  -- get the basepoint for LLH -> XY conversion
//
//	This becomes the new basepoint, and XY values will be computed from it.
//
//	Usually, the first waypoint is the basepoint.
//
struct GPSINSGetBasepoint: public MsgBase
{
    static const uint32_t k_msgtype = char4('G','P','B','G');
};
//
//	GPSINSGetBasepointRep  -- reply from Get Basepoint
//
struct GPSINSGetBasepointRep {
		double llh[3];										// Latitude(deg), Longitude(deg) Height(m)
};
//
//	All GPSINS messages as a union
//
union GPSINSMsg {
	GPSINSMsgReq	m_req;						// request for GPS/INS fix
	GPSINSBasepoint m_basepoint;			// set new basepoint
	GPSINSGetBasepoint m_getbasepoint;	// get current basepoint
};
//////////////////////////////////////////////////////////////////////////////////////////////////
// some useful inline functions
//////////////////////////////////////////////////////////////////////////////////////////////////

inline const char *
decodePosStat(GPSINS_MSG::SolStatusEnum e)
{

    switch (e)
    {
    case	GPSINS_MSG::SOL_COMPUTED:
        return "SOL_COMPUTED";
    case GPSINS_MSG::INSUFFICIENT_OBS:
        return "INSUFFICIENT_OBS";
    case GPSINS_MSG::NO_CONVERGENCE:
        return "NO_CONVERGENCE";
    case GPSINS_MSG::SINGULARITY:
        return "SINGULARITY";
    case GPSINS_MSG::COV_TRACE:
        return "COV_TRACE";
    case GPSINS_MSG::TEST_DIST:
        return "TEST_DIST";
    case GPSINS_MSG::COLD_START:
        return "COLD_START";
    case GPSINS_MSG::V_H_LIMIT:
        return "V_H_LIMIT";
    case GPSINS_MSG::VARIANCE:
        return "VARIANCE";
    case GPSINS_MSG::RESIDUALS:
        return "RESIDUALS";
    case GPSINS_MSG::DELTA_POS:
        return "DELTA_POS";
    case GPSINS_MSG::NEGATIVE_VAR:
        return "NEGATIVE_VAR";
    case GPSINS_MSG::RESERVED:
        return "RESERVED";
    case GPSINS_MSG::INTEGRITY_WARNING:
        return "INTEGRITY_WARNING";
    }

    return "UNKNOWN SOLUTION STATUS";

}

inline const char *
decodePosType(GPSINS_MSG::PosVelTypeEnum e)
{

    switch(e)
    {
    case GPSINS_MSG::NONE:
        return "NONE";
    case	GPSINS_MSG::FIXEDPOS:
        return "FIXEDPOS";
    case GPSINS_MSG::FIXEDHEIGHT:
        return "FIXEDHEIGHT";
    case GPSINS_MSG::FIXEDVEL:
        return "FIXEDVEL";
    case GPSINS_MSG::DOPPLER_VELOCITY:
        return "DOPPLER_VELOCITY";
    case GPSINS_MSG::SINGLE:
        return "SINGLE";
    case GPSINS_MSG::PSRDIFF:
        return "PSRDIFF";
    case GPSINS_MSG::WAAS:
        return "WAAS";
    case GPSINS_MSG::OMNISTAR:
        return "OMNISTAR";
    case GPSINS_MSG::L1_FLOAT:
        return "L1_FLOAT";
    case GPSINS_MSG::IONOFREE_FOAT:
        return "IONOFREE_FOAT";
    case GPSINS_MSG::NARROW_FLOAT:
        return "NARROW_FLOAT";
    case GPSINS_MSG::L1_INT:
        return "L1_INT";
    case GPSINS_MSG::WIDE_INT:
        return "WIDE_INT";
    case GPSINS_MSG::NARROW_INT:
        return "NARROW_INT";
    case GPSINS_MSG::RTK_DIRECT_INS:
        return "RTK_DIRECT_INS";
    case GPSINS_MSG::INS:
        return "INS";
    case GPSINS_MSG::OMNISTAR_HP:
        return "OMNISTAR_HP";
    }

    return "UNKNOWN POS TYPE";
}

inline const char *
decodeErr(GPSINS_MSG::Err e)
{	switch (e) {
	case GPSINS_MSG::OK: return("OK");
	case GPSINS_MSG::INT_ERR: return("INTERNAL ERR");
	case GPSINS_MSG::INITIALIZATION: return("INITIALIZATION");
	default: return("Out of range");
	}
}


#endif //GPSINS_MESSAGING
