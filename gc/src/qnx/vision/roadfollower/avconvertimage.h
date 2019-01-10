//
//	avconvertimage.h   -- convert image from IPL to AV format
//
//	John Nagle
//	Team Overbot
//	January, 2003
//
#ifndef AVCONVERTIMAGE_H
#define AVCONVERTIMAGE_H
//
#include "../../common/include/cv/cv.h"
#include "avcodec.h"
//
//	avconvertimage  -- convert from IplImage format (OpenCV) to AVFrame format (ffmpeg)
//
int avconvertimage(AVFrame* out, int out_fmt , int out_width, int out_height, IplImage* in);
////int avconvertimage(IplImage* out, const AVFrame* in);

#endif // AVCONVERTIMAGE_H