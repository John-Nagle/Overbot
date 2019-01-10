#ifndef _ROADFOLLOWER
#define _ROADFOLLOWER

#include <cv.h>

class RALPHFollower;
class OffRoadFollower;
class ImageSampler;

class RoadFollower {
    IplImage *m_display_image; // internal image buffer for display purposes
    double m_angle; // the current road curvature
    double m_score; // a confidence score
    double m_offset; // the offset of the road from center of sub-sampled image
    double m_threshold; // used to establish a threshold on m_score
    int m_debug; // flag for debug mode
    int m_display; // flag for display mode
    RALPHFollower *m_ralph; // an instance of a class to perform the RALPH road follower
    OffRoadFollower *m_offroad; // an instance of a class to perform the OVERBOT road follower
    ImageSampler *m_sampler; // an instance of a class to perform sub-sampling
    unsigned int m_counter; // a counter for the number of frames of video processed 

public:
    RoadFollower();
    ~RoadFollower();
    
    void setDebug(int d);
    void processFrame(IplImage* image);
    double getAngle();
    double getScore();
    double getOffset();
    unsigned int getCounter() {return m_counter;}
    void setUpperRight(int x, int y);
    void setUpperLeft(int x, int y);
    void setLowerRight(int x, int y);
    void setLowerLeft(int x, int y);
    void setDisplayResults(int d);
    void setThreshold(double t);
};

#endif
