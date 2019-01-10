#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <math.h>
#include <queue>
#include "algebra3.h"
#include "Constants.h"
#include "mapserver.h"
#include "LMSmapupdate.h"
#include "LMStiltcorrect.h"

MapServer mapServer;
vec3 scanneroffset;
LMSmapUpdater mapUpdater(mapServer,scanneroffset);
void ShowUsage();

class testTiltCorrect {
private:
   FILE              *m_lidar;
   float             m_minTilt;

   LMStiltCorrector  m_tiltCorrector;

public:
   testTiltCorrect(const char* lidarFilename, const float tilt_ignore=20):
								m_tiltCorrector(mapUpdater)
	{
   	m_lidar = fopen(lidarFilename,"r");            // open for reading
   	if (!m_lidar)  {   
     		cout << "Unable to open LIDAR data file " << lidarFilename << endl;
		ShowUsage();
		exit(1);                                             // fails
   	}

   	// convert degree to radian
   	m_minTilt = tilt_ignore*3.1415926/180;
	};
	
   ~testTiltCorrect()
	{
		if (m_lidar) fclose(m_lidar);
	};
	
   float averageRange(const LidarScanLine& lp, const int startRangesIndex=69,const int endRangesIndex=110);

   
   void run(void);
};

float testTiltCorrect::averageRange(const LidarScanLine& lp,int startRangesIndex,int endRangesIndex)
{     
    float totalRangeValue = 0;
    for (int i=startRangesIndex; i < endRangesIndex; i++)
      totalRangeValue += lp.m_range[i];
      
    return (totalRangeValue/(endRangesIndex-startRangesIndex));

}

void testTiltCorrect::run(void)
{
   LidarScanLine line;
   uint32_t cyclestamp = 0;
     
   for (; ;) {   
      int stat = fread(&line, sizeof(line), 1, m_lidar);   // get scan line
      if (stat <= 0) break;                                 // EOF

//      if (line.m_header.m_tilt < m_minTilt)
//         continue;
      
      cyclestamp++;
      float avgRange = averageRange(line);
      mat4 vehPose;
      uint32_t cStamp=cyclestamp;
      LidarScanLine outlp;
      bool goodAvg=true; 
      if (m_tiltCorrector.correctTilt(line,goodAvg,avgRange,vehPose,cStamp,outlp)) {
        cout << " cycleStamp1="<<cyclestamp;
        cout << " cycleStamp2="<< cStamp;
        cout << " avg2="<<avgRange;
		  timespec ts;														// as timespec
	  	  nsec2timespec(&ts,outlp.m_header.m_timestamp);						// convert to seconds
		  tm localtm;
		  localtime_r(&ts.tv_sec,&localtm);							// convert time
		  char s[100];
		  const char* format = "%F %T";							// yyyy-mm-dd hh:mm:ss 
		  strftime(s,sizeof(s),format,&localtm);					// edit time
	  	  cout << " time="<<s<<"."<<ts.tv_nsec;
        cout << " tilt="<<outlp.m_header.m_tilt;
        cout << endl;
      }
    }
}

char *filename="lidar.log";

void ShowUsage()
{
   cout << "Usage:  tiltCorrect [options] " << endl << endl;
   cout <<"\t[options] are any of:" << endl;
   cout << "\t\t/f filename     -- lidar filename, default lidar.log" << endl;

   cout << endl;
   exit(1);
}

bool ParseOptions(int argc, char* argv[])
{
   int nIndex;
   for (nIndex = 1; nIndex < argc; nIndex++)   {
      if (*argv[nIndex] == '-' || *argv[nIndex] == '/') {
         if (argv[nIndex][1] == 'F' || argv[nIndex][1] == 'f') {
            filename = argv[nIndex+1];
            nIndex++;
          } else   {
            cout << "Error: unrecognized option: " << argv[nIndex] << endl;
            return false;
          }
      }
   }

   return true;
}; // ParseOptions

/////////////////////////////////////////////////////////////////////////////
// The main()



int main(int argc, char* argv[]) 
{
    if(argc >= 2)
        if(!ParseOptions(argc, argv))
            ShowUsage();

   testTiltCorrect test(filename);

   test.run();
}
