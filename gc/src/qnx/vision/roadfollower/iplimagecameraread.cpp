//
//	iplimageIplImageCameraRead.h  --  support for reading from a FireWire camera using our driver
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
#include "iplimagecameraread.h"
//
//
//
//	Constructor
//
IplImageCameraRead::IplImageCameraRead()
: m_fd(-1)
{	
	m_mode.videomode = MODE_640x480_RGB;	// set some reasonable modes
	m_mode.framerate = FRAMERATE_15;	
	m_mode.bytesperframe = 0;								// driver tells us this when we set the mode		
}
//	
//	Destructor
//
IplImageCameraRead::~IplImageCameraRead()
{	close();
}
//
//	open -- open the camera
//
int IplImageCameraRead::open(const char* path)
{	if (m_fd >= 0) close();								// close if necessary
	assert(m_fd < 0);										// must be closed
	m_fd = ::open(path, O_RDWR);					// open for read/write (so we can send functions)
	if (m_fd < 0) 
	{	assert(errno); return(errno);		}			// fails with indicated error
	return(setup());											// success
}
//
//	close  -- close the camera
//
int IplImageCameraRead::close()
{	if (m_fd < 0) return(EOK);							// not open
	int fd = m_fd;											// get fd for close
	m_fd = -1;													// no file descriptor now
	if (::close(fd))											// close
	{	assert(errno); return(errno); }				// return error status
	return(EOK);												// success
}
//
//	swaprows  -- swap two rows of an image
//
//	NO CHECKING
//
inline void swaprows(char* row1, char* row2, size_t len)
{	for (size_t i=0; i<len; i++)
	{	char temp = row1[i]; row1[i] = row2[i]; row2[i] = temp;	}	// swap
}
//
//	readframe  --  read a full frame from the camera in its current mode
//
int IplImageCameraRead::readframe(IplImage& outimage)
{
	if (m_fd < 0) return(EBADF);											// no device open
	if (outimage.width != m_mode.width) return(EINVAL);			// size check
	if (outimage.height != m_mode.height) return(EINVAL);
	const int imgbytes = outimage.height*outimage.width*outimage.nChannels;	// Image size in pixels
	if (imgbytes != m_mode.bytesperframe) return(EINVAL);		// size does not match
	if (outimage.depth != IPL_DEPTH_8U) return(EINVAL);	// bits per pixel
	//	Read video into image buffer
	int pos = 0;
	int tries = 0;
	char* image = outimage.imageData;							// image area						
	while (pos < imgbytes)
	{	int left = imgbytes - pos;												// bytes left to read
		int cnt =  ::read(m_fd,&image[pos], left);					// read as much image as possible
		if (cnt == 0) 															// end of image, done
		{	if (pos == 0 && tries++ == 0)	 continue;				// if missed a frame (normal)
			break;																	// otherwise trouble
		} 
		if (cnt < 0)
		{	perror("Error reading image");	break;	}				// fails
		pos += cnt;
	}
	if (pos != imgbytes)
	{	if (pos > 0) { printf("Image not fully read; expected %d bytes, received %d bytes.\n",imgbytes,pos); }	//	***TEMP***
		return(EIO);
	}
	//	Format conversions

	switch (m_mode.videomode) {
	case MODE_640x480_RGB:											// RGB mode					
		//	Swap bytes for each pixel.  FireWire DCAM appears to be backwards.
		for (int i=0; i<imgbytes-2; i += 3)
		{	char tmp = image[i];  image[i] = image[i+2];  image[i+2] = tmp;	}		// BGR => RGB
		break;
	default:
		break;
	}
	//	If desired output order is with the origin at the bottom, we must flip top to bottom
	if (outimage.origin == IPL_ORIGIN_BL)							// origin at bottom
	{	const int halfrows = outimage.height / 2;					// half the row count		
		const size_t rowbytes = outimage.width*3;				// bytes per row																	
		for (int i=0, j=outimage.height-1; i<halfrows; i++,j--)					// for half the rows
		{	swaprows(&image[i*rowbytes], &image[j*rowbytes],rowbytes); }	// swap a row
	}
	return(EOK);																	// success
}
int  IplImageCameraRead::setgain(int gain)
{
	return(EOK);												// success
}
//
//	setmode -- set camera mode
//
int IplImageCameraRead::setmode(int videomode, int framerate)
{
	m_mode.videomode = videomode;
	m_mode.framerate = framerate;		
	if (m_fd >= 0)											// if already open
	{	int stat = setup();									// change right now
		return(stat);
	}		
	return(EOK);												// success
}
int  IplImageCameraRead::reset()
{
	return(EOK);												// success
}
//
//	setup -- set up the camera for reading
//
int IplImageCameraRead::setup()
{	if (m_fd < 0) return(EINVAL);									// not open
	//	Set camera mode.
	printf("Setting video mode %d, framerate %d\n",
		m_mode.videomode,m_mode.framerate);				// ***TEMP***
	int stat = devctl(m_fd,DCMD_MEDIA_DEVFW_CAMERA_VIDEO_MODE_WRITE,&m_mode,sizeof(m_mode),0);
	if (stat != EOK)
	{	errno = stat; perror("Unable to set camera mode");
		return(errno);
	}
	printf("Set video mode %d, framerate %d:  (%d x %d - %d bytes per frame)\n",
		m_mode.videomode,m_mode.framerate,m_mode.width,m_mode.height, m_mode.bytesperframe);
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