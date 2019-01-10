//
//	cameraread.h  --  support for reading from a FireWire camera using our driver
//
//	John Nagle
//	Team Overbot
//	October, 2003
//
#ifndef CAMERAREAD_H
#define CAMERAREAD_H

#include "../../../../../common/include/devfw-camera.h"
#include "ablibs.h"

//
//	class CameraRead  --  read from a FireWire camera
//
class CameraRead {
private:
	int m_fd;													// file descriptor of camera
	PhImage_t*	m_image;								// image area to draw
public:
	int open(const char* path);						// open, given path
	int close();												// close
	int readframe(uint8_t buf[], size_t bufsize);	// read a frame from the camera 
	int setgain(int gain);									// set camera gain
	int setmode(dc1394_video_mode_t& mode);	// set camera mode
	int reset();
	void video_port_draw( PtWidget_t *widget, PhTile_t *damage );	// redraw
	CameraRead();											// constructor
	~CameraRead();										// destructor
private:
	int setup();												// initial setup
};
#endif // CAMERAREAD_H