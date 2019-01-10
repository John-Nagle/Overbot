//
//	ffmpegwrite.h  -- MPEG output support
//
//	John Nagle
//	Team Overbot
//	January, 2004
//
#ifndef FFMPEGWRITE_H
#define FFMPEGWRITE_H

#include "avformat.h"
//
//	class MPEGframe  -- a picture image for use with the ffmpeg package
//
class MPEGframe {
	AVFrame*	m_frame;														// the frame
	int m_pix_fmt;																// frame info
	int m_width;
	int m_height;
public:
	MPEGframe()																	// constructor
	: m_frame(0)
	{}
	~MPEGframe()																// destructor
	{	free();	}
	int alloc(int pix_fmt, int width, int height);
	void free();
	AVFrame* getframe() { 	return(m_frame); }				// access
	int getpixfmt() const { return(m_pix_fmt); }
	int getwidth() const { return(m_width); }
	int getheight() const { return(m_height); }
};

//
//	class MPEGwrite  --- one MPEG output stream
//
class MPEGwrite {
private:
    bool							m_opened;									// stream is opened
    AVOutputFormat*		m_fmt;											// format ID
    AVStream* 				m_video_st;									// output format
    double 						m_video_pts;									// video time
   	uint8_t*						m_video_outbuf;							// output buffer
   	size_t						m_video_outbuf_size;					// size of output buffer
    AVFormatContext	 	m_oc;											// control structure for encoders
private:
	int add_video_stream(unsigned int width, unsigned int height, unsigned int framerate, CodecID codec_id);
	int open_video();
public:
	int open(const char* filename, unsigned int width, unsigned int height, unsigned int framerate, const char* format);
	void close();	
	int write(AVFrame& picture, double ptstimestamp = 0.0);			// write a picture to the file
	MPEGwrite();
	virtual ~MPEGwrite();
	bool isopened() const { return(m_opened); }				// if opened, true
	const AVOutputFormat* getformat() const { return(m_fmt); }
};

#endif // FFMPEGWRITE_H