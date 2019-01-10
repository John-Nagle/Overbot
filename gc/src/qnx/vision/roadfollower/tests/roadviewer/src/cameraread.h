//
//	cameraread.h  --  support for reading from a FireWire camera using our driver
//
//	John Nagle
//	Team Overbot
//	October, 2003
//
#ifndef CAMERAREAD_H
#define CAMERAREAD_H

//	Need to get phAb to search more directories
#include "../../../../iplimagecameraread.h"
#include "../../../../roadfollower.h"

#include "ablibs.h"
#include <memory>
#include "messaging.h"
#include "remotemessaging.h"

//
//	class CameraRead  --  read from a FireWire camera
//
class CameraRead {
private:
	RemoteMsgClientPort m_roadserver;				// define client port, no timeout
	int m_fd;														// file descriptor of camera
	IplImage m_cvimage;									// CV library image
	PhImage_t*	m_image;									// image area to draw
private:
	////void setroadtrapezoid();								// set up the road trapezoid	
public:
	int reset();
	void video_port_draw( PtWidget_t *widget, PhTile_t *damage );	// redraw
	CameraRead();											// constructor
	~CameraRead();										// destructor
private:
	int setup();												// initial setup
};
#endif // CAMERAREAD_H