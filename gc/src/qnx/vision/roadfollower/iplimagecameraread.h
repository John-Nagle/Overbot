//
//	iplimagecameraread.h  --  OpenCV camera interface for QNX
//
//	J. Nagle
//	Team Overbot
//	November, 2003
//
#ifndef IPLIMAGECAMERAREAD_H
#define IPLIMAGECAMERAREAD_H

#include "../../common/include/cv/cv.h"
#include "../../common/include/devfw-camera.h"

//
//	class IplImageCameraRead  --  read from a FireWire camera into an OpenCV image
//
class IplImageCameraRead {
private:
	int m_fd;													// file descriptor of camera
	dc1394_video_mode_t m_mode;				// current camera mode
public:
	int open(const char* path);						// open, given path
	int close();												// close
	int readframe(IplImage& image);				// read a frame from the camera 
	int setgain(int gain);									// set camera gain
	int setmode(int videomode, int framerate);// set camera mode
	int reset();
	void getsize(int& width, int& height, int& bytesperframe)
	{	width = m_mode.width; height = m_mode.height; bytesperframe = m_mode.bytesperframe; }
	IplImageCameraRead();							// constructor
	virtual ~IplImageCameraRead();				// destructor
private:
	int setup();												// initial setup
};

#endif  // IPLIMAGECAMERAREAD_H
