#ifndef _IMAGESAMPLER
#define _IMAGESAMPLER

#include <cv.h>

class ImageSampler {
    IplImage *m_image; // pointer to internal image buffer to store sub-sampled image
    int m_x1; // x-coord of lower left
    int m_x2; // x-coord of lower right
    int m_x3; // x-coord of upper right
    int m_x4; // x-coord of upper left
    int m_y1; // y-coord of lower left
    int m_y2; // y-coord of lower right
    int m_y3; // y-coord of upper right
    int m_y4; // y-coord of upper left
    int m_height; // pixel height of sub-sampled image region for road follower to analyze
    int m_width; // pixel width of sub-sampled image region for road follower to analyze
    int m_min_shift; // store the extreme amount we need to shift to the right for hypotheses
    int m_max_shift; // store the extremem amount we need to shift to the left for hypotheses
    int m_display; // flag to toggle display mode (draw trapezoid on the original input image)

    // given a coord in the original input image determine a pixel value by block averaging
    void getAveragePixel(IplImage *img, int x, int y, int xstep, int ystep, unsigned char *r, unsigned char *g, unsigned char *b);

public:
    ImageSampler();
    ~ImageSampler();
    
    //void setSampleSize(int height, int width);
    int getSampleHeight();
    int getSampleWidth();
    void setUpperRight(int x, int y);
    void setUpperLeft(int x, int y);
    void setLowerRight(int x, int y);
    void setLowerLeft(int x, int y);
    void setDisplayMode(int d);
    
    const IplImage *sample(IplImage *img);
    void getSamplePixel(int x, int y, unsigned char *r, unsigned char *g, unsigned char *b);
};

#endif
