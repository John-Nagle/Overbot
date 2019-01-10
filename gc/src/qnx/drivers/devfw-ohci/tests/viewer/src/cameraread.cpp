//
//	cameraread.h  --  support for reading from a FireWire camera using our driver
//
//	John Nagle
//	Team Overbot
//	October, 2003
//
#include <assert.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include "cameraread.h"
#include "proto.h"

//
//	Constants
//
const int imgwidth = 640;										// width in pixels
const int imgheight = 480;									// height in pixels
const int imgtype = Pg_IMAGE_DIRECT_888;			//	image format, RGB, 3 bytes per pixel
const int imgsize = imgwidth*imgheight*3;			// size of entire image
//
//	Constructor
//
CameraRead::CameraRead()
: m_fd(-1),m_image(0)
{	m_image = PhCreateImage(0,imgwidth,imgheight,imgtype,0,0,0);	// allocate image
	assert(m_image);												// must get object
	assert(m_image->image);									// must get image space
}
//	
//	Destructor
//
CameraRead::~CameraRead()
{	close();
	if (m_image) 													// if image structure is allocated
	{	PhReleaseImage(m_image);							// release image
		free(m_image);												// release image (C-type allocation)
	}
}
//
//	open -- open the camera
//
int CameraRead::open(const char* path)
{	if (m_fd >= 0) close();								// close if necessary
	assert(m_fd < 0);										// must be closed
	m_fd = ::open(path, O_RDWR);					// open for read/write (so we can send functions)
	if (m_fd < 0) 
	{	assert(errno); return(errno);		}			// fails with indicated error
	//	***MORE***
	return(setup());											// success
}
//
//	close  -- close the camera
//
int CameraRead::close()
{	if (m_fd < 0) return(EOK);							// not open
	int fd = m_fd;											// get fd for close
	m_fd = -1;													// no file descriptor now
	if (::close(fd))											// close
	{	assert(errno); return(errno); }				// return error status
	return(EOK);												// success
}
//
//	readframe  --  read a full frame from the camera in its current mode
//
int readframe(uint8_t buf[], size_t bufsize)
{
	return(EOK);												// success
}
int  CameraRead::setgain(int gain)
{
	return(EOK);												// success
}
int  CameraRead::setmode(dc1394_video_mode_t& mode)
{
	return(EOK);												// success
}
int  CameraRead::reset()
{
	return(EOK);												// success
}
//
//	setup -- set up the camera for reading
//
int CameraRead::setup()
{	if (m_fd < 0) return(EINVAL);									// not open
	//	Set camera mode.
	dc1394_video_mode_t mode;
	mode.videomode = MODE_640x480_RGB;			// set some reasonable modes
	mode.framerate = FRAMERATE_15;						
	int stat = devctl(m_fd,DCMD_MEDIA_DEVFW_CAMERA_VIDEO_MODE_WRITE,&mode,sizeof(mode),0);
	if (stat != EOK)
	{	errno = stat; perror("Unable to set camera mode");
		return(errno);
	}
	printf("Set video mode %d, framerate %d:  (%d x %d - %d bytes per frame)\n",
		mode.videomode,mode.framerate,mode.width,mode.height, mode.bytesperframe);
	//	Set camera to auto-exposure
	const uint32_t FEATURE_MODE_AUTO = (1<<31)>>7;	// bit 7 from MSB enables auto
	dc1394_feature_info feature;										// feature info
	feature.feature_id = FEATURE_EXPOSURE;
	feature.value = FEATURE_MODE_AUTO;							// set auto mode
	stat = devctl(m_fd,DCMD_MEDIA_DEVFW_CAMERA_FEATURE_WRITE,&feature,sizeof(feature),0);	// set mode
	if (stat != EOK)
	{	errno = stat; perror("Unable to set feature to auto."); 
		return(errno);
	}
	return(EOK);																	// success
}
//
//	video_port_draw -- draw video into indicated port
//
void CameraRead::video_port_draw(PtWidget_t *widget, PhTile_t *damage)
{
	if (m_fd < 0) return;														// no video available
	//	Read video into image buffer
	int pos = 0;
	int tries = 0;
	char* image = m_image->image;									// image area						
	while (pos < imgsize)
	{	int left = imgsize - pos;												// bytes left to read
		int cnt =  ::read(m_fd,&image[pos], left);					// read as much image as possible
		if (cnt == 0) 															// end of image, done
		{	if (pos == 0 && tries++ == 0)	 continue;				// if missed a frame (normal)
			break;																	// otherwise trouble
		} 
		if (cnt < 0)
		{	perror("Error reading image");	break;	}				// fails
		pos += cnt;
	}
	if (pos != imgsize)
	{	printf("Image not fully read; expected %d bytes, received %d bytes.\n",imgsize,pos);	
		for (int i=0; i<imgsize; i++) { image[i] = 0;	}			// force black frame
	}
	//	Swap bytes for each pixel.  FireWire DCAM appears to be backwards.
	for (int i=0; i<imgsize-2; i += 3)
	{	char tmp = image[i];  image[i] = image[i+2];  image[i+2] = tmp;	}		// BGR => RGB
	//	Draw on screen
	const PhRect_t *rect = PtCalcCanvas(widget, 0);			// location of drawing area for this widget
	int stat = PgDrawPhImage(&(rect->ul), m_image, 0);			// draw the image
	if (stat) printf("Error from PgDrawPhImage\n");				// ***TEMP***
}

