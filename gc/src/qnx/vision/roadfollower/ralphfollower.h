#ifndef _RALPHFOLLOWER
#define _RALPHFOLLOWER

#include <cv.h>

struct intensityRecord;
struct pixelValue;

class RALPHFollower {
    int m_height;
    int m_width;
    int m_debug;
    int m_draw;
    double *m_rowBuffer;
    intensityRecord *m_intensities;
    pixelValue *m_colBuffer;
    double scanlineIntensity(IplImage *img, int s, double *area=0, int *center=0);

public:
    RALPHFollower();
    ~RALPHFollower();
    
    double analyze(IplImage *img, double *score=0, double *center=0);
    void setDebug(int d);
    void setDrawMode(int d);

};

#endif
