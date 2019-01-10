/////////////////////////////////////////////////////////////////////////////
//
//    File: roadserver.h
//
//    Usage:
//        In the watchdog startup file (startfile.txt):
//				ID=ROAD roadserver -v 
///
//    Description:
//       The Road Follower Server provides visual road-following
//			capabilities. It also provides images from the camera
//			for logging and visualization capabilities.
//       
//
//    Written By:
//        John Nagle
//        Team Overbot
//        November, 2003
//
//		The actual road follower was written by John Pierre.
//
/////////////////////////////////////////////////////////////////////////////

#ifndef ROADSERVER_H
#define ROADSERVER_H

#include "messaging.h"

//
//  RoadServer - Road follower messages
//
//
//  RoadServerMsgRDDR - RDDR: Road Direction
//
//  Returns road direction info.
//	This message is both a query and a reply.
//	Blocks until next frame comes in from the camera.
//
struct RoadServerMsgRDDR: public MsgBase {
	static const uint32_t k_msgtype = char4('R','D','D','R');
	enum RoadPavement { RoadPaved, RoadUnpaved, RoadUnknown };
	RoadPavement pavement;					// paved or unpaved; selects which road follower algorithm to use
	float upvector[3];									// which way is up, as a unit vector, from INS.  Used to adjust image
};
//
//	Reply from RoadServerMsgRDDR
//
struct RoadServerMsgReply {
	float m_confidence;								// confidence (< 0, ignore)
	float m_curvature;									// suggested steering curvature
};

//
//  RoadServerMsgRDPC - RDPC: Road Picture
//
//  Returns a picture from the camera, with or without overlaid debug info
//	Blocks until next frame comes in from the camera.
//	RDPC requests do not interfere with RDDR requests.
//
//	Reply is a raw RGB image, 320x240, overlaid with the debug info from the road follower,
//	with no header info, so it can be read directly into image buffers for viewing or storage.
//
struct RoadServerMsgRDPC: public MsgBase {
	static const uint32_t k_msgtype = char4('R','D','P','C');
	int debuglevel;										// overlay with graphic debug info - 0=none, 1=normal, 2=debug
};

//
//  RoadServerMsg - all MVP Server messages as a union
//
//  Used as argument to MsgReceive. Size of union is size of largest 
//  acceptable message
//
union RoadServerMsg {
	RoadServerMsgRDDR m_rddr;
	RoadServerMsgRDPC m_rdpc;
};

#endif // ROADSERVER_H
