//
//	Implementation of MPEGframe
//
//	Wrapper for FFMPEG video output
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

const size_t k_video_outbuf_size = 200000;					// space for video

const int k_bit_rate = 100000;										// output bit rate

//
//	dumpcodecs -- dump list of available codecs
//
static void dumpcodecs()
{	extern AVOutputFormat *first_oformat;
	for (AVOutputFormat* p = first_oformat; p; p = p->next)	// for all registered output codecs
	{	printf("    %s",p->name);									// print name of format
		const char* extensions = p->extensions;		// file extensions
		if (extensions && extensions[0])						// file extensions, if any
		{	printf(" (%s)",extensions);		}
		putchar('\n');
	}
}

//
//	Implementation
//
//	Class MPEGframe
//
//	alloc  -- allocate picture buffer
//
int MPEGframe::alloc(int pix_fmt, int width, int height)
{	free();																		// if already allocated, free
	m_frame = avcodec_alloc_frame();							// allocate frame control	
	if (!m_frame) return(-1);											// fails, no mem
	size_t size = avpicture_get_size(pix_fmt, width, height);	// get size of a raw picture in this format
	uint8_t* picturebuf = (uint8_t*)av_malloc(size);		// get actual data space
	if (!picturebuf)															// if no picture buffer
	{	av_free(m_frame); m_frame = 0; return(-1);	}	// fails
	//	NOTE: within ffmpeg, AVFrame appears to be a psuedo derived class of AVPicture. 
	//	But since ffmpeg is a C program, this is done badly, with macros.  See "avcodec.h"
	//	The provided "output_example" program does such a cast, in C.
	AVPicture* picture = reinterpret_cast<AVPicture*>(m_frame);
	size = avpicture_fill(picture, picturebuf, pix_fmt, width, height);	// fill in picture info
	if (size < 0) return(-1);												// alloc failed
	assert(picture->data[0]);											// must have allocated space
	m_pix_fmt = pix_fmt;													// save dimension and format info
	m_width = width;
	m_height = height;
	return(0);
}
//	
//	free
//
void  MPEGframe::free()
{	if (!m_frame) return;													// if not allocated, ignore
	av_free(m_frame->data[0]);										// release data buffer
	av_free(m_frame);													// release picture object				
	m_frame = 0;
}
//	constructor
//
MPEGwrite::MPEGwrite()
	:	m_opened(false), m_fmt(0),m_video_pts(0.0), m_video_outbuf(0), m_video_outbuf_size(k_video_outbuf_size)
{	memset(&m_oc,sizeof(m_oc),0);	}
//
//	Destructor
//
MPEGwrite::~MPEGwrite()
{	close(); }
//
//	open -- open video stream for writing
//
int MPEGwrite::open(const char* filename, unsigned int width, unsigned int height, unsigned int framerate, const char* format)
{	if (m_opened) close();																// close any previous stream
    /* initialize libavcodec, and register all codecs and formats */
    av_register_all();
   	m_fmt = guess_format(format,NULL,NULL);	
    if (!m_fmt)
    {
        printf( "Could not find video codec \"%s\"\n", format);
        printf("Available codecs:\n");
        dumpcodecs();
        return(-1);																				// fails
    }
	memset(&m_oc,sizeof(m_oc),0);												// clear output context
    m_oc.oformat = m_fmt;
    snprintf(m_oc.filename, sizeof(m_oc.filename), "%s", filename);			// copy filename to context

    /* add the audio and video streams using the default format codecs
       and initialize the codecs */
    m_video_st = NULL;
    if (m_fmt->video_codec != CODEC_ID_NONE)
    {
        int stat = add_video_stream(width, height, framerate, m_fmt->video_codec);
        if (stat)
        {	printf("MPEG output: cannot add video stream.\n"); return(-1); }
    }

    /* set the output parameters (must be done even if no
       parameters). */
    if (av_set_parameters(&m_oc, NULL) < 0)
    {
        printf( "MPEG output: Invalid output video format parameters\n");
       	return(-1);
    }

    dump_format(&m_oc, 0, filename, 1);

    /* now that all the parameters are set, we can open the audio and
       video codecs and allocate the necessary encode buffers */
    if (m_video_st)
    {	int stat =  open_video();
    	if (stat)
    	{	printf("MPEG output: open_video failed.\n");
    		return(-1);
    	}
    }


    /* open the output file, if needed */
    if (!(m_fmt->flags & AVFMT_NOFILE))
    {
        if (url_fopen(&m_oc.pb, filename, URL_WRONLY) < 0)
        {
            printf("Could not open video stream file %s'\n", filename);
            return(-1);
        }
    }

    /* write the stream header, if any */
    av_write_header(&m_oc);
    m_opened = true;													// note as opened
    return(0);
}
//
//	close -- close streams
//
void MPEGwrite::close()
{	if (!m_opened) return;											// if not open, nothing to do
	//	Close codec
	avcodec_close(&m_video_st->codec);
	//	Write trailer
	av_write_trailer(&m_oc);
    /* free the streams */
    for(int i = 0; i < m_oc.nb_streams; i++)
    {
        av_freep(&m_oc.streams[i]);
    }
	assert(m_fmt);														// must have format
    if (!(m_fmt->flags & AVFMT_NOFILE))
    {
        /* close the output file */
        url_fclose(&m_oc.pb);
    }
    m_fmt = 0;															// ***CHECK*** did we allocate, or just look up
    if (m_video_outbuf)												// if have outbuf
    {	free(m_video_outbuf);										// free it
    	m_video_outbuf = 0;
    }
    m_opened = false;												// no longer open
}
//
//	add_video_stream  --  add a video stream
//
int MPEGwrite::add_video_stream(unsigned int width, unsigned int height, unsigned int framerate, CodecID codec_id)
{
    m_video_st = av_new_stream(&m_oc, 0);
    if (!m_video_st)
    {
        printf("Could not alloc video stream for file.\n");
        return(-1);
    }

    AVCodecContext *c = &m_video_st->codec;
    c->codec_id = codec_id;
    c->codec_type = CODEC_TYPE_VIDEO;

    /* put sample parameters */
    c->bit_rate = k_bit_rate;
    /* resolution must be a multiple of two */
    if (width % 2 || height % 2)									// insist on valid frame size
    {	return(-1);	}
    c->width = width;
    c->height = height;
    c->frame_rate = framerate;
    /* frames per second */
    c->frame_rate_base = 1;
    c->gop_size = 12; /* emit one intra frame every twelve frames at most */
    if (c->codec_id == CODEC_ID_MPEG2VIDEO)			// B frames for MPEG 2 video only
    {
        /* just for testing, we also add B frames */
        c->max_b_frames = 2;
    }
    if (c->codec_id == CODEC_ID_MPEG1VIDEO)
    {
		 //	needed to avoid using macroblocks in which some coeffs overflow 
		//	this doesnt happen with normal video, it just happens here as the 
		//	motion of the chroma plane doesnt match the luma plane */
		c->mb_decision=2;
    }	
    // some formats want stream headers to be separate
	if(!strcmp(m_oc.oformat->name, "mp4")
		|| !strcmp(m_oc.oformat->name, "mov")
		|| !strcmp(m_oc.oformat->name, "3gp"))
		        c->flags |= CODEC_FLAG_GLOBAL_HEADER;
    //	Suggested by Andrew Voznytsa on gmane newsgroup to avoid assertion
    //	failure "movenc.c:704 enc->extradata_size -- assertion failed"
    ////c->flags |= CODEC_FLAG_GLOBAL_HEADER;					// ???
    return(0);																		// success
}
//
//	open_video -- open the video stream
//
int MPEGwrite::open_video()
{	
    AVCodecContext* c = &m_video_st->codec;

    /* find the video encoder */
    AVCodec* codec = avcodec_find_encoder(c->codec_id);
    if (!codec)
    {
        printf("MPEG write: codec not found.\n");
       	return(-1);
    }

    /* open the codec */
    if (avcodec_open(c, codec) < 0)
    {
        printf("MPEG write: could not open codec.\n");
       	return(-1);
    }
	return(0);
}
//
//	write  -- write a picture to the file
//
int MPEGwrite::write(AVFrame& picture, const double ptstimestamp)
{	if (!m_video_st) return(-1);															// not open, fails
	int ret = 0;																					// return code
    if (m_oc.oformat->flags & AVFMT_RAWPICTURE)							// if uncompressed
    {
        /* raw video case. The API will change slightly in the near
			future for that */
        ret = av_write_frame(&m_oc, m_video_st->index,
                             (uint8_t *)&picture, sizeof(picture));
    }
    else
    {
	    if (!m_video_outbuf)																// if no video buffer yet
	    {	m_video_outbuf = (uint8_t*)malloc(m_video_outbuf_size);				// allocate video buffer
			if (!m_video_outbuf) return(-1);											// out of space, fail
		}
		//	Timestamp the image
		uint64_t timestamp = uint64_t(ptstimestamp * 1e6);				// timestamp in microseconds
		picture.pts = timestamp;														// timestamp the picture
        /* encode the image */
        int out_size = avcodec_encode_video(&m_video_st->codec, m_video_outbuf, m_video_outbuf_size, &picture);
        /* if zero size, it means the image was buffered */
        if (out_size != 0)
        {
            /* write the compressed frame in the media file */
            /* XXX: in case of B frames, the pts is not yet valid */
            ret = av_write_frame(&m_oc, m_video_st->index, m_video_outbuf, out_size);
        }
        else
        {
            ret = 0;																					// OK, don't need to write yet
        }
    }
    if (ret)																							// if fail
    {
    	printf("MPEG WRITE: Error while writing video frame.\n");
		return(-1);
    }
    return(0);
}
