//
//	avconvertimage  -- convert image from IPL to AV format
//
//	John Nagle
//	Team Overbot
//	January, 2003
//
#include <stdio.h>
#include "../../common/include/cv/cv.h"
#include "avcodec.h"
//
//	memswap -- swap two areas of memory
//
inline void memswap(uint8_t* p1, uint8_t* p2, size_t len)
{	for (unsigned int i=0; i<len; i++)
	{	uint8_t tmp = p1[i]; p1[i] = p2[i]; p2[i] = tmp;	}	// swap
}
//
//	avconvertimage  -- convert from IplImage format (OpenCV) to AVFrame format (ffmpeg)
//
int avconvertimage(AVFrame* out, int out_fmt , int out_width, int out_height, IplImage* in)
{	assert(in);
	assert(out);
	//	Check compatibility
	if ((in->width != out_width) 
		|| (in->height != out_height)
		|| (in->depth != IPL_DEPTH_8U) 
		|| (in->nChannels != 3))
	{	printf("avconvertimage mismatch: width %d vs %d  height %d vs %d  channels %d vs %d  depth %d vs %d\n",
			in->width, out_width, in->height, out_height, in->depth, IPL_DEPTH_8U, in->nChannels, 3);
		fflush(stdout);
		return(-1);
	}
	AVPicture picture;															// Defining structure for picture in AV format
	int pix_fmt = PIX_FMT_BGR24;										// assume input is RGB 24
	int size = avpicture_fill(&picture, (uint8_t*)in->imageData,			// create dummy picture
                  	pix_fmt, in->width, in->height);	
    if (size < 0)
    {	printf("avconvertimage error: avpicture_fill returned %d\n", size	);
    	fflush(stdout);
    	return(-1);
    }
 	//	NOTE: within ffmpeg, AVFrame appears to be a psuedo derived class of AVPicture. 
	//	But since ffmpeg is a C program, this is done badly, with macros.  See "avcodec.h"
	//	The provided "output_example" program does such a cast, in C.
	AVPicture* dst = reinterpret_cast<AVPicture*>(out);
   	int stat = img_convert(dst,out_fmt,
                &picture, pix_fmt, in->width, in->height);	
    if (stat)
    {	printf("avconvertimage error: img_convert returned %d\n", stat);
    	fflush(stdout);
    }
    //	If the origin of the input image is at the bottom, we have to flip the output image vertically
    if (in->origin)																// if inverted image
    {	const size_t rowsize = out->linesize[0];					// number of bytes per line
    	assert(rowsize > 0);
    	////assert(rowsize < out_width*3);							// sanity check on row width
    	uint8_t* firstrowp = out->data[0];							// first output line
    	uint8_t* lastrowp = firstrowp + rowsize*(out_height-1);	// beginning of last row
    	for (int i=0; i < in->height /2; i++)
    	{	memswap(firstrowp, lastrowp,rowsize);				// swap two rows
    		firstrowp += rowsize;											// adjust pointers
    		lastrowp -= rowsize;
    	}
    }
  	return(stat);
}