#ifndef _OFFROADFOLLOWER
#define _OFFROADFOLLOWER

#include <cv.h>

struct intensityRecord;
struct pixelValue;
class ImageSampler;

class OffRoadFollower {
    IplImage *m_display_image; // internal image buffer for display purposes
    int m_height; // the height of the sub-sampled image to analyze
    int m_width; // the width of the sub-sampled image to analyze
    int m_debug; // flag debug mode which prints out verbose information to stdout
    int m_draw; // flag draw mode which draws informative stuff into m_display_image

    double *m_rowBuffer; // temporary storage to analyze pixel rows
    intensityRecord *m_intensities; // not used here
    pixelValue *m_colBuffer; // temporary storage to analyze pixel columns

    // method to apply a curvature hypothesis to the sub-sampled image and return 
    // intermedidiate results
    double scanlineIntensity(ImageSampler *sampler, int s, double *area=0, int *center=0);

public:
    OffRoadFollower();
    ~OffRoadFollower();
    
    double analyze(ImageSampler *sampler, double *score=0, double *center=0);
    void setDebug(int d);
    void setDrawMode(int d);
    IplImage *getDisplayImage();
};

#endif
