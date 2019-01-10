//
//	Test program for MPEG encoding
//
//	Uses ffmpeg.
//
//	John Nagle
//	Team Overbot
//	January, 2003
//
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include "avformat.h"
#include "ffmpegwrite.h"

//
//	Dummy image generator from FFMPEG test program.
//
static void fill_yuv_image(AVFrame *pict, int frame_index, int width, int height)
{
    int x, y, i;

    i = frame_index;

    /* Y */
    for(y=0;y<height;y++)
    {
        for(x=0;x<width;x++)
        {
            pict->data[0][y * pict->linesize[0] + x] = x + y + i * 3;
        }
    }

    /* Cb and Cr */
    for(y=0;y<height/2;y++)
    {
        for(x=0;x<width/2;x++)
        {
            pict->data[1][y * pict->linesize[1] + x] = 128 + y + i * 2;
            pict->data[2][y * pict->linesize[2] + x] = 64 + x + i * 5;
        }
    }
}

//
//	runtest -- generate dummy video file
//
int runtest(const char* filename, int width, int height, int fps, int secs)
{
	MPEGwrite out;																	// output image
	int stat = out.open(filename,width , height ,fps);				// open
	if (stat)
	{	perror("Unable to open output");	return(1);	}
	const int fmt = PIX_FMT_YUV420P;										// desired format for file
	MPEGframe frame;
	stat = frame.alloc(fmt, width, height);								// allocate a buffer
	if (stat)
	{	printf("Unable to allocate frame.\n"); return(-1); }
	int framecount = secs * fps;												// number of frames
	double frametime = 1.0/fps;												// time for each frame
	for (int i=0; i<framecount; i++)											// generate some frames
	{	AVFrame* framep = frame.getframe();							// get frame
		fill_yuv_image(framep, i, width, height);						// dimensions
		stat = out.write(*framep,i*frametime);							// write the frame
		if (stat)
		{	printf("Unable to write frame\n");	return(-1); }			// fails	
	}
	out.close();																		// done, close
	return(0);																			// success
}

//
//	usage -- print usage message and exit
//
static void usage()
{	printf("Usage: ffmpegtest [options] outputfilename\n");
	exit(1);
}
//
//	Main program
//
//	Usage: ffmpegtest [options] outputfilename
//
int main(int argc, const char* argv[])
{	bool verbose = false;
	bool debug = false;
	int height = 240;
	int width = 320;
	int fps = 15;
	int seconds = 10;
	const char* outfilename = 0;
	//	Parse input arguments
	for (int i=1; i<argc; i++)											// for all args
	{	const char* arg= argv[i];										// this arg
		if (arg[0] == '-')													// if flag argument
		{	switch(arg[1])	{												// interpret flags
			case 'v': verbose = true;	 break;						// set verbose mode
			case 'd': debug = true; break;							// set verbose debug mode
			case 'h':															// height
				i++;
				if (i >= argc) usage();
				height = atoi(argv[i]);
				continue;
				
			case 'w':															// width
				i++;
				if (i >= argc) usage();
				width = atoi(argv[i]);
				continue;
				
			case 'f':																// width
				i++;
				if (i >= argc) usage();
				fps = atoi(argv[i]);
				continue;

			case 's':															// seconds
				i++;
				if (i >= argc) usage();
				seconds = atoi(argv[i]);
				continue;
				
				default: usage();											// bad call, fails
			}
			continue;															// next arg
		}
		//	Not flag, must be file arg
		if (!outfilename)													// first file arg is camera name
		{	outfilename = arg; continue; }
		usage();
	}
	if (!outfilename) usage();											// must have output file name
	runtest(outfilename, width, height, fps, seconds);		// run ffmpeg test program
	return(0);																	// success
}