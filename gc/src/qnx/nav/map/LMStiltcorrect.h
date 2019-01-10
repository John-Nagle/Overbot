//
//   LMStiltcorrect.h  --  corrector for tilt of LIDAR scanner
//
//   Corrects for LIDAR tilt vibration based on average range reported.
//
//
//   Fred Au
//   John Pierre
//   John Nagle
//
//   Team Overbot
//   August, 2005
//
#ifndef LMSTILTCORRECT_H
#define LMSTILTCORRECT_H
//
#include <stdint.h>
#include <vector>
#include "algebra3.h"
#include "lidarserver.h"
//
class LMSmapUpdater;                                          // forward

//-----------------------------------------------------------------
// LMStiltCorrector - corrects tilt for one LIDAR.
//
//  It uses a symmetric low-pass filter (hamming window) to reduce
//  high frequencies.
//
//  by default the window size is 7. Users can specify window size
//  through interface
//
// Note:
//   1. It is not thread safe
//   2. The first m_size/2 lines and last m_size/2 lines will be lost
//   3. There will be (m_size/2) lines delay. For window size 7, it
//      would be 3
// -----------------------------------------------------------------
class LMStiltCorrector
{

private:

    struct ScanLineData
    {
        bool            goodAvgRange;
        float           avgRange;
        mat4            vehPose;
        uint32_t        cycleStamp;
        LidarScanLine   linePoints;
    };
 
    LMSmapUpdater&             m_owner;         // parent
    std::vector<float>         m_weights;       // Hamming coefficients
    size_t                     m_middle;        // middle item in the queue
    std::deque<ScanLineData*>  m_dataQueue;     // store scan line related data
    std::queue<ScanLineData*>  m_emptyQueue;    // empty scan line data queue


public:
    LMStiltCorrector(LMSmapUpdater& owner);
    ~LMStiltCorrector();
    bool correctTilt(const LidarScanLine& lp,bool& goodAvgRange,float& avgRange,mat4& vehPose,uint32_t& cycleStamp,LidarScanLine& outlp);
    size_t  windowSize() const
    {
        return m_weights.size();            // filter size
    };
    bool windowSize(const size_t size);
    int getVerboseLevel() const;         // get from parent

private:
    bool isValidWindowSize(const size_t size) const;
    void ComputeHammingCoefficient();
};

#if ENABLE_BACKWARD_FILTER


//-----------------------------------------------------------------
// LMSbackwardFilter
//
//    A simple filter algorithm. It checks scan lines whether
//   "go backward" except the vechile is making turns
//
// Note:
//  It assumes the vehicle is moving forward, and not sweeping. It checks two adjacent
//  lines(l_t1, l_t2, t2>t1) whether overlap each other. If they are,
//  set the second line(l_t2) to invalid.
//
//  It first sets a reference point, and update the reference point when
//  scanner head at the point to turn up from down & the scan line is valid.
//  Then, it compares distances of two points on the adjacent lines(l_t1,
//  l_t2) at the same angle to the reference point. If points at t2 is less
//  and the corresponding points at l_t2, the l_t2 is invalid
//
// Note:
//  may not need it,
//-----------------------------------------------------------------
class LMSbackwardFilter
{
private:
    LMStiltCorrector m_owner
    int              m_center;
    int              m_maxGazeAngle;
    mat4             m_referencePose;
    bool             m_resetReferencePose;
    float            m_tilt1;
    float            m_tilt2;
    float*           m_pDistances1
    float*           m_pDistances2
    bool             m_lastScanLineValid;

public:
    LMSbackwardFilter(const LMStiltCorrector& owner,const int center=90,const int maxGazeAngle=20);
    ~LMSbackwardFilter();

    bool isValidScanLine(const mat4& vehPose,const LidarScanLine& lp);
};
#endif

#endif //  LMSTILTCORRECT_H
