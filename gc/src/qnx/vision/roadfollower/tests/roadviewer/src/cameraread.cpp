//
//	cameraread.cpp  --  support for reading from a FireWire camera using our driver
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
#include "../../../../iplimagecameraread.h"
#include "cameraread.h"
#include "proto.h"
#include "roadserver.h"

//
//	Constants
//
const char* servername = "RROAD";					// name of server as remotemsgserver
const char* servernode = "gcrear0";					// node on which server runs

const int imgwidth = 320;										// width in pixels
const int imgheight = 240;									// height in pixels
const int imgchannels = 3;									// number of color channels
const int imgdepth = 8;											// bits per pixel
const int imgtype = Pg_IMAGE_DIRECT_888;			// Photon image format, RGB, 3 bytes per pixel
const int cammode = MODE_640x480_RGB;			// FireWire camera mode
const int camframerate = FRAMERATE_15;			// FireWire camera framerate
const int imgsize = imgwidth*imgheight*imgchannels;			// size of entire image
//
//	Constructor
//
CameraRead::CameraRead()
: m_roadserver(servernode, servername,0.0),m_image(0)
{	//	Create Photon image for screen	
	m_image = PhCreateImage(0,imgwidth,imgheight,imgtype,0,0,0);	// allocate image
	assert(m_image);												// must get object
	assert(m_image->image);									// must get image space
	//	Create working openCV image
	CvSize imgsize = {imgwidth, imgheight };		// simple struct
	cvInitImageHeader(&m_cvimage, imgsize, imgdepth , imgchannels, IPL_ORIGIN_BL);
	cvCreateImageData(&m_cvimage);					// ***CHECK STATUS***
	////setmode(cammode,camframerate);					// set camera mode
	//	Intitialize road follower
	////setroadtrapezoid();
}
//	
//	Destructor
//
CameraRead::~CameraRead()
{	
	if (m_image) 													// if image structure is allocated
	{	PhReleaseImage(m_image);							// release image
		free(m_image);												// release image (C-type allocation)
	}
	{	cvReleaseImageData(&m_cvimage);	}				// release it
}
//
//	PhCopyImage  -- copy image from IPL type image to Photon type image
//
//	RGB only; sizes must match
//	General use. Move to library
//
static int PhCopyImage(PhImage_t* dest, const IplImage* src)
{	assert(dest);																	// must be present
	assert(src);																	// must be present
	//	Check that sizes match
	if (src->width != dest->size.w) return(EINVAL);			// width mismatch
	if (src->height != dest->size.h) return(EINVAL);			// height mismatch
	if (dest->type != Pg_IMAGE_DIRECT_888) return(EINVAL);	// not RGB image
	if (src->nChannels != 3) return(EINVAL);						// not RGB dest image
	if (!dest->image) return(EINVAL);									// no destination image
	if (!src->imageData) return(EINVAL);							// no source image
	const size_t cnt = dest->size.w*dest->size.h*3;			// bytes to copy
	if (src->origin == IPL_ORIGIN_TL)									// if origin at top left
	{	memcpy(dest->image,src->imageData,cnt);		}		// matches Photon, just copy
	else																				// origin at bottom left, must flip vertically
	{	const int bytesperrow = dest->size.w*3;					// bytes per row
		char* destrow = dest->image + bytesperrow*(dest->size.h-1);	// beginning of last row
		const char* srcrow = src->imageData;					// beginning of first src row
		for (int i=0; i < dest->size.h; i++)								// for all rows
		{	
			memcpy(destrow, srcrow, bytesperrow);				// copy one row
			destrow -= bytesperrow;										// source row moves up
			srcrow += bytesperrow;										// dest row moves down
		}
	}
	return(EOK);
}
//
//	Draw message in center of widget
//
static void drawmsg(PtWidget_t * widget, const char* s)
{	const int textHeight = 18;												// size of text, points
    PhPoint_t p = { 100,100 };											// ***TEMP*** should be center
    char Helvetica[MAX_FONT_TAG];
    if (PfGenerateFontName("Helvetica", 0, textHeight,
                          Helvetica) == NULL) {
        {	perror("Unable to find font"); return; }
    } else {
        PgSetFont( Helvetica );
    }
    PgSetTextColor( Pg_BLACK );
    PgDrawText( s, strlen( s ), &p, 0 );
}
//
//	video_port_draw -- draw video into indicated port
//
void CameraRead::video_port_draw(PtWidget_t *widget, PhTile_t *damage)
{
	//	Call server for an image
	RoadServerMsgRDPC picmsg;										// Picture request
	picmsg.m_msgtype = RoadServerMsgRDPC::k_msgtype;	// set type
	char* img = m_cvimage.imageData;
	const int imgsize = m_cvimage.height*m_cvimage.width*m_cvimage.nChannels; 
	//	Call the server
	int stat = m_roadserver.MsgSend(&picmsg,sizeof(picmsg),img, imgsize);
	if (stat < 0)																	// if fail
	{	drawmsg(widget,strerror(errno));									// display error
		return;
	}
	//	Copy CV image to Photon image
	stat = PhCopyImage(m_image,&m_cvimage);				// copy to Photon format
	if (stat) printf("Error from PhCopyImage\n"); 					// ***TEMP***
	//	Draw on screen
	const PhRect_t* rect = PtCalcCanvas(widget, 0);			// location of drawing area for this widget
	stat = PgDrawPhImage(&(rect->ul), m_image, 0);			// draw the image
	if (stat) printf("Error from PgDrawPhImage\n");				// ***TEMP***
}

