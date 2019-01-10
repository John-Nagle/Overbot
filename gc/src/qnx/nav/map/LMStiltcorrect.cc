//
//   LMStiltcorrect.cch  --  corrector for tilt of LIDAR scanner
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
//
#include <stdint.h>
#include <stdio.h>
#include <queue>
#include "algebra3.h"
#include "geocoords.h"   
#include "Constants.h"
#include "logprint.h"
#include "LMSmapupdate.h"
#include "LMStiltcorrect.h"
#include "tuneable.h"

const  size_t k_defaultHammingWindowSize=7;

const Tuneable k_range_outlier_threshold("RANGEOUTLIERTHRESHOLD", 0.00, 2.0, 0.0, "Tilt correction outlier threshold (ratio)");

//-----------------------------------------------------------------
//
//   class LMStiltCorrector   -- corrects tilt for one LIDAR
//
//
//-----------------------------------------------------------------
LMStiltCorrector::LMStiltCorrector(LMSmapUpdater& owner)
        :m_owner(owner)
{
   windowSize(k_defaultHammingWindowSize);               // size weights vector of filter
}
//
//   Destructor
//
LMStiltCorrector::~LMStiltCorrector()
{
    ScanLineData *pData;

    // clean up
    while (!m_dataQueue.empty())
    {
        pData = m_dataQueue.front();
        m_dataQueue.pop_front();
        delete pData;
    }

    while (!m_emptyQueue.empty())
    {
        pData = m_emptyQueue.front();
        m_emptyQueue.pop();
        delete pData;
    }
}
//
//   correctTilt -- correct tilt for one scan line
//
//   Actually does the tilt correction.
//
//   Note that vehPose is the pose for the incoming line on input, and for the outgoing line on output.
//   However, the incoming line comes in as lp, and the outgoing line goes out as outlp.
//
bool LMStiltCorrector::correctTilt(const LidarScanLine& lp,bool& goodAvgRange,float& avgRange,mat4& vehPose,uint32_t& cycleStamp,LidarScanLine& outlp)
{
    ScanLineData *pData=NULL,       // store the current input data to end of queue
                        *pMiddle=NULL;// point to the middle item in queue

    bool  res = true;          // result to be returned

    const size_t msize = m_weights.size();               // filter size

    if (m_dataQueue.size() == msize-1)
    {
        // has enough lines; m_size-1 scan lines in m_queue, one is the current input

        // run low pass filer to get filtered tilt

        // compute the filtered average range

        float filteredAvgRange = 0.0;
        for (size_t i=0; i<msize-1; i++)
        {
            filteredAvgRange += m_weights[i]*m_dataQueue[i]->avgRange;
        }

        filteredAvgRange += m_weights[msize-1]*avgRange;

        // get the pointer to the middle item of queue,
        // for return scan line data to caller
        pMiddle = m_dataQueue[m_middle-1];

        // compute the filtered tilt.
        float newTilt =  pMiddle->linePoints.m_header.m_tilt;				// incoming tilt							
        float avgRatio = pMiddle->avgRange / filteredAvgRange;		// ratio of this range to average
        if (finite(avgRatio) && (fabs(avgRatio - 1.0) < k_range_outlier_threshold))		// if not outlier
		{	float cot = cos(pMiddle->linePoints.m_header.m_tilt);			// OK to change tilt
			newTilt = acos(cot*avgRatio);												// calculate adjusted tilt
		}
        if (getVerboseLevel() >= 2)                                       // if moderately verbose
        {   
           double oldtiltdeg = radians2deg(pMiddle->linePoints.m_header.m_tilt);
           double newtiltdeg = radians2deg(newTilt);
           double tiltcorrdeg = newtiltdeg - oldtiltdeg;
           logprintf("Tilt correction %1.2f deg + %1.2f deg -> %1.2f deg.  Avg range %1.2f m.\n", oldtiltdeg, tiltcorrdeg, newtiltdeg,
               filteredAvgRange); 
        }

        pMiddle->linePoints.m_header.m_tilt = newTilt;
        // remove 1st scan line data from queue
        pData=m_dataQueue.front();
        m_dataQueue.pop_front();

    }
    else
    {
        // no enough data, couldn't run low pass filer

        // get empty scan line data from empty queue for depositing scan line
        if (m_emptyQueue.empty()) throw ("Tilt empty queue allocation error");
        pData = m_emptyQueue.front();
        m_emptyQueue.pop();
        // only need to allocate m_size-1 times
        res = false;
    }

    // deposit scan line related data to the queue for processing later
    pData->goodAvgRange = goodAvgRange ;
    pData->avgRange = avgRange ;
    pData->vehPose = vehPose;
    pData->cycleStamp = cycleStamp;
    pData->linePoints = lp;
    m_dataQueue.push_back(pData);

    if (pMiddle)
    {
        // setup the data to return back to the caller
        goodAvgRange = pMiddle->goodAvgRange;
        avgRange = pMiddle->avgRange;
        vehPose=pMiddle->vehPose;
        cycleStamp = pMiddle->cycleStamp;
        outlp = pMiddle->linePoints;
    }

    return res;

}

int LMStiltCorrector::getVerboseLevel() const
{
    return (m_owner.getVerboseLevel());
}

//
//   windowSize  -- change size of filter
//
bool LMStiltCorrector::windowSize(const size_t size)
{
    if (!isValidWindowSize(size))
        return false;

    m_weights.resize(size);                                    // resize filter table
    m_middle = (m_weights.size())/2 + 1;                        // index of middle value. 
    ComputeHammingCoefficient();

    // empty the data queue & recycle memory blocks
    while (!m_dataQueue.empty())
    {
        ScanLineData* blank = m_dataQueue.front();
        m_dataQueue.pop_front();
        m_emptyQueue.push(blank);
    }
    //	Clear empty queue too
   	while (!m_emptyQueue.empty())
    {
        ScanLineData* blank = m_emptyQueue.front();
        delete(blank);
        m_dataQueue.pop_front();
        m_emptyQueue.push(blank);
    }

    //	Allocate proper number of scan lines for empty queue
    for (size_t i=0;  i < m_weights.size()-1; i++)
    {   
		ScanLineData* blank = new ScanLineData;            // get a blank scan line data
		m_emptyQueue.push(blank);            // fill queue with empty scan line data
    }

    return true;
}



inline bool LMStiltCorrector::isValidWindowSize(const size_t size) const
{
    // size must be odd number for low-pass filter
    return (size - 2*(size/2))==1;
}

inline void LMStiltCorrector::ComputeHammingCoefficient()
{
    float sum = 0.0;

    for (size_t i=0; i< m_weights.size();i++)
    {
        float weight = 0.54 - 0.46 * cos((2*pi*i)/(m_weights.size()-1));
        sum += weight;
        m_weights[i] = weight;
    }

    // normalized the weights
    for (size_t i=0; i< m_weights.size();i++)
        m_weights[i] /= sum;
}
