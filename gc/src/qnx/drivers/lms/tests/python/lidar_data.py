#! python
# -----------------------------------------------------------------
#
# lidar_data.py
#
#  This module contains Lidar data and gps related classes, and
#  an utility program to dump data from lidar data log file
#
# -----------------------------------------------------------------
import struct
import math
from math import *
from datetime import datetime
import time
from array import *

FORCE_GPS_Z_ZERO=1
FORCE_SMOOTH_LIDAR_TIME = 0
bPrintTrace=False
VEHICLE_MOVING_DIR=False


################################################################################
#
# Lidar data related classes
#
################################################################################

# -----------------------------------------------------------------
# LidarData
#   Decoodes packed lidar scan data.
#
#   Use get to retrieve a valid scan line data in format
#   (timestamp,tilt,numOfRanges,[ranges])
#
#   This class supports iterator behavior for convenient.
# -----------------------------------------------------------------
'''
struct LidarScanLineHeader {
	uint64_t	m_timestamp;  // CLOCK_REALTIME, in nanoseconds
	float		m_tilt;	      // tilt angle of unit. 0=straight down, pi/2=straight ahead
	uint8_t		m_sensorid;   // which LMS (future expansion)
	uint8_t		m_statusByte; // LMS status byte
	uint8_t		m_scanIndex;  // scan index (cyclic)
	uint8_t		m_unused1;    // (future expansion)
	uint16_t	m_unused2;    // (future expansiion, and fill to word)
	uint16_t	m_valueCount; // count of m_range
};

struct LidarScanLine {
	LidarScanLineHeader m_header;		      // the header
	uint16_t	m_range[LMS_MAX_DATA_POINTS]; // depth data (m_valuecount entries)
};
'''
current_time = time.time()

class LidarData:
  HeaderFormat = "QfBBBBHHI"                    # format of header part
                                                # with 4 bytes padding (I)
  HeaderFormatL = struct.calcsize(HeaderFormat) # header size (must be 24)
  DataFormat = "H"                              # format of one data item
  DataFormatL = struct.calcsize(DataFormat)     # data item size
  TotalRecordL = 392                            # entire record, including filler
  DataRecordL = TotalRecordL-HeaderFormatL      # data part, including filler
  START_ANGLE=(-3.1415926/2.0)
  ANGLE_STEP=(2.0*3.1415926/360.0)
  
  def __init__(self,lidar_file=0,tilt_ignore=20,startTime=0,endTime=current_time,cyclesOfTiltLagRange=0):
    ''' for debug only
    print "LidarData init with tilt_ignore=",tilt_ignore,\
          "startTime=",datetime(1,1,1).fromtimestamp(startTime).isoformat(' '),\
          "endTime=",datetime(1,1,1).fromtimestamp(endTime).isoformat(' ')
    '''
    self.infile = lidar_file
    self.tilt_ignore = tilt_ignore
    self.radiansOfTiltIgnore= radians(tilt_ignore)
    self.startTime = startTime
    self.endTime = endTime
    
    self.time1 = 0.0
    self.time2 = 0.0
    self.num_scanline = 0
    self.cyclesOfTiltLagRange=cyclesOfTiltLagRange
    self.bufferOfLines=[]

    cycle = abs(cyclesOfTiltLagRange)
    if (cycle > 0):
      print "Cycles Of Tilt Lag Range:" ,cyclesOfTiltLagRange
      while cycle > 0:
        line = self._get()
        # read enough lines
        self.bufferOfLines.append(line)
        cycle -= 1
        continue

  def __iter__(self):
        return self
    
  def next(self):
    data = self.get()
    if (data == []):
      raise StopIteration
    
    return data

  def get(self):
    # return (time,tilt,numOfRanges,Ranges)
    while 1:
      line = self._get()
      if line == []:
        break;
      
      if (self.cyclesOfTiltLagRange != 0 ):
        if (self.cyclesOfTiltLagRange < 0):
          # range lags tilt, get ranges from current line and tilt from
          # cyclesOfTiltLagRange lines ahead
          line1 = self.bufferOfLines.pop(0)
          self.bufferOfLines.append(line)
          line = (line[0],line1[1],line[2],line[3])
        else:
          # tilt lags range, get tilt from current line and ranges from
          # cyclesOfTiltLagRange lines ahead
          self.bufferOfLines.append(line)
          line1 = self.bufferOfLines.pop(0)
          line = (line1[0],line[1],line1[2],line1[3])

      # ignore it?
      if(line[1]<self.radiansOfTiltIgnore):
        continue

      # have line data 
      break
    
    return line

  def _get(self):
    # return (time,tilt,numOfRanges,Ranges)
    self.time1 = self.time2
    bFound = False
    while 1:
      buf=self.infile.read(self.HeaderFormatL)
      if(len(buf)==0):
        break
      
      rangeData=self.infile.read(self.DataRecordL);
      if(len(rangeData) < self.DataRecordL):
        # incomplete record,skip
        break
      
      header=struct.unpack(self.HeaderFormat,buf)
      #	Compute timestamp in seconds from 64-bit nanosecond counter
      self.time2 = header[0]*0.000000001
      if(FORCE_SMOOTH_LIDAR_TIME and self.num_scanline >= 1):
        self.time2=self.time1+1.0/75.0
        
      if(self.time2<self.startTime):
        continue
      
      if(self.time2>self.endTime):
        break
      
      tilt = header[1]
      #if(tilt<self.radiansOfTiltIgnore):
      #    continue
      
      numOfRanges = header[7]
      if(numOfRanges!=181 and numOfRanges!=180):
        continue

      # found a line
      bFound = True
      break

    vert=[]
    if (bFound):
      # found a line
      RangeVert=[]
      self.num_scanline +=1
              
      for i in range(0,numOfRanges):
        data=rangeData[i*self.DataFormatL:(i+1)*self.DataFormatL]
        RangeVert.append(struct.unpack(self.DataFormat,data)[0])
        
      vert = (self.time2,tilt,numOfRanges,RangeVert)
      
    return vert

  def getTimeRange(self, filename):
    '''
      get first record timestamp & last record timestamp from lidar log file
    '''
    # open for read
    try:
      infile=open(filename,"rb")
    except IOError,(errno,strerror):
      print "Open file <%s>, IOError #%s: %s" % (filename,errno,strerror)
      return(0,0)

    '''
      Read first record to retrieve the first recorded time
    '''
    buf=infile.read(self.HeaderFormatL)
    if (len(buf) < self.HeaderFormatL): return (0,0)  # don't have any data

    header=struct.unpack(self.HeaderFormat,buf)
    
    startTime = header[0]*0.000000001 # convert nanoseconds to  microseconds

    '''
      seek to last record, and retrieve the last recorded time
    '''
    fileSize = os.path.getsize(filename)
    recordNum = int(fileSize/self.TotalRecordL)
    if (recordNum > 1):
      offset = (recordNum-1)*self.TotalRecordL
      infile.seek(offset)
      buf=infile.read(self.HeaderFormatL)
      header=struct.unpack(self.HeaderFormat,buf)
      endTime = header[0]*0.000000001
    else:
      endTime = startTime

    # close
    infile.close()

    return (startTime,endTime)
  
  def _avg(self,ranges,startRangesIndex,endRangesIndex):

    totalRangeValue = 0
    for i in range(startRangesIndex,endRangesIndex):
      totalRangeValue += ranges[i]
      
    return (1.0*totalRangeValue/(endRangesIndex-startRangesIndex))
  
# -----------------------------------------------------------------
# LidarData_SLPfiltered
#
#  Derived class of LidarData that implements the symmetric low-pass filter.
#
#  Use getData to retrieve a valid scan line data in format
#   (timestamp,tilt,numOfRanges,[ranges])
#
#  This class supports iterator behavior for convenient.
#
# The delta-tilt is generated in the following steps:
# 1. compute average  range from a line starting point startAngle
#    to endAngle
# 2. generate windowSize(7) adjacent-line averages
#    ...avg[-3],avg[-2],avg[1],avg[0],avg[1],avg[2],avg[3],...
# 3, use low pass filter to get new avg
#     new-avg[0] = ...+w[-3]*avg[-3]+w[-2]*avg[-2]+w[-1]*avg[-1]+w[0]*avg[0]
#                  +w[1]*avg[1]+w[2]*avg[2]+w[3]*avg[3]+...
#    for example, on window size 7, 
#     new-avg[0] = 0.0241* avg[-3]+0.0934*avg[-2]+0.2319*avg[1]+0.3012*avg[0]
#                        +0.2319*avg[1]+0.0934*avg[2]+0.0241*avg[3]
# 4. delta-tilt = tilt[0]*(new-avg[0]/avg[0] - 1)
#    Note. we need to find a better formula
#
# -----------------------------------------------------------------
class LidarData_SLPfiltered(LidarData):
  
  def __init__(self,lidar_file,tilt_ignore,startTime,endTime,startAngle=70,endAngle=110,\
               cyclesOfTiltLagRange=0,windowSize=7):
    #print "LidarData_SLPfiltered init with startAngle=%d endAngle=%d windowSize=%d" % (startAngle,endAngle,windowSize)
    # initial the base class
    LidarData.__init__(self,lidar_file,tilt_ignore,startTime,endTime,cyclesOfTiltLagRange)
    
    # since angle step is 1 degree, we can simply do the following assignment
    if (startAngle > 1):
      self.startRangesIndex = startAngle-1
    else:
      self.startRangesIndex = 0
      
    if (endAngle < 180):
      self.endRangesIndex = endAngle
    else:
      self.endRangesIndex = 179

    self.numOfRanges = self.endRangesIndex-self.startRangesIndex
    self.lineVerts=[]
    self.avgRangeVerts=[]

    self.filter = HammingWindow(windowSize)
    self.windowSize = windowSize
    self.middleLineIndex = int(windowSize/2)
    if (self.middleLineIndex *2 == windowSize):
      self.middleLineIndex -= 1

    self.time1 = 0.0
    self.time2 = 0.0
    self.num_scanline = 0
    self.bMoreScanLine = True

    # for dumping average related data only
    self.filteredAvgRange = 0.0
    self.avgRange = 0.0

    # Advance to first valid line based on start time, and
    # get enough lines for low pass filter
    
    while 1:
      line = self.get()
      if (line == []):
        self.bMoreScanLine = False
        break;
      
      self.lineVerts.append(line)
      avgRange = self._avg(line[3],self.startRangesIndex,self.endRangesIndex)
      self.avgRangeVerts.append(avgRange)

      if (len(self.lineVerts) < self.windowSize-1):
        continue
      
      # enough data
      break;

  def __iter__(self):
        return self
    
  def next(self):
    data = self.getData()
    if (data == []):
      raise StopIteration
    
    return data
      
  def getData(self):
    # get a scan line from LidarData
    line = self.get()    
    if (line == []):
      self.bMoreScanLine = False
    else:
      self.lineVerts.append(line)
      avgRange = self._avg(line[3],self.startRangesIndex,self.endRangesIndex)
      self.avgRangeVerts.append(avgRange)
      
    # check has enough lines for filter
    if (len(self.lineVerts) < self.windowSize):
      if (self.bMoreScanLine == False and (len(self.lineVerts) == 0)):
        # no more data
        return []

      # still has a few line of data in lineVerts
      vert = self.lineVerts.pop(0)
      return vert

    # has enough lines
    # run low pass filer to get delta tilt
    deltaTilt = self._deltaTilt()
    
    # adjust tilt on the middle line

    vert = self.lineVerts[self.middleLineIndex]
    # pop up first data to return
    self.lineVerts.pop(0)
    self.avgRangeVerts.pop(0)
    return (vert[0],vert[1]+deltaTilt,vert[2],vert[3])

  def _deltaTilt(self):
    self.filteredAvgRange = self.filter.compute(self.avgRangeVerts)
    self.avgRange = self.avgRangeVerts[self.middleLineIndex]
    oldTilt = self.lineVerts[self.middleLineIndex][1]
    '''
     So far, There are three ways to compute delta tilt.
     1. deltaTilt = acos((cos(oldTilt)*avgRange)/filteredRange) - oldTilt
        Since the filteredRange can be too small, so that calculation of newTilt
        will fail on acos(x) where x > 1
        
     2. deltaTilt = oldTilt * (filteredRange/avgRange - 1.0)
         
     3. deltaTilt = ( 1.0 - self.avgRange / self.filteredAvgRange) / tan(oldTilt)
    '''
    avgRatio = self.avgRange/self.filteredAvgRange
    cot = cos(oldTilt)
    if (avgRatio * cot) > 1.0:
        deltaTilt = oldTilt * (1.0 /avgRatio - 1.0)
    else:
        deltaTilt = acos(cot*avgRatio) - oldTilt
    
    return (deltaTilt)

  # for dumping average range related data
  def getAvgRanges(self):
    return (self.avgRange,self.filteredAvgRange)
    
# -----------------------------------------------------------------
# HammingWindow
#
#   hamming-window filter, by default,
# the window size is 7. Users can specify window size through interface
# -----------------------------------------------------------------
class HammingWindow:
  def __init__(self,size=7):
    if (self.setWindowSize(size) == False):
      raise ValueError

  def setWindowSize(self,size):
    if self.isValidWindowSize(size) == False:
      return False
    
    self.windowSize = size
    self.weights = self._hamming(size)
    return True

  def isValidWindowSize(self,size):
    # size must be odd number for low-pass filter
    return (size - 2*int(size/2))==1
  
  def getWindowSize(self):
    return self.windowSize
  
  def compute(self,values):
    if (len(values) != self.windowSize):
      raise AssertionError

    res = 0.0
    for i in range (0,self.windowSize):
      res += self.weights[i]*values[i]

    return res

  def _hamming(self,windowSize):
    weights=[]
    sum = 0.0

    for i in range (0,windowSize):
      weight = 0.54 - 0.46 * cos((2*pi*i)/(windowSize-1))
      sum += weight
      weights.append(weight)

    normalizedWeights=[]
    for i in range (0,windowSize):
      normalizedWeights.append(weights[i]/sum)

    return normalizedWeights

# -----------------------------------------------------------------
# Butterworth_Filter
#
#   butterworth filter, a recursive low-pass filter.
#
# For order 4, new avg is calculated as follow
# new-avg = a0*avg + a1*avg3+a2*avg2+a3*avg1+a4*avg0
#                    + b1*newAvg3+b2*newAvg2+b3*newAvg1+b4*newAvg0
# -----------------------------------------------------------------
class Butterworth_Filter:
  maxDeltaTilt = radians(5)

#  coefficient_dict={4:[[1.0,0.0,0.486,0.0,0.0177],[0.0946,0.3759,0.5639,0.3759,0.0940]],\
  coefficient_dict={4:[[0.0946,0.3759,0.5639,0.3759,0.0940],[1.0,0.0,0.486,0.0,0.0177]],\
                    3:[[0.0317,0.0951,0.0951,0.0317],[1.0,-1.4590,0.9104,-0.1978]],\
                    2:[[8.663387e-04,1.732678e-03,8.663387e-04],[0.0,1.919129,-9.225943e-01]]\
                   }
  maxCorruptedTilts_dict={4:30,2:25,6:40}
  MaxCorruptedTiltsForAllOrders = 100
  def __init__(self,cutoffFrequency=0.02,order=4):
    print "Butterworth_Filter init with cutoff frequency=",cutoffFrequency," order=",order
       
    self.order = order
    if order == 2*int(order/2)  and order != 2:
#    if order == 2*int(order/2):
      # even number, we can calculate the coefficient
      # we want low-pass filter
      (self.a,self.b)=self._computeCoefficient(cutoffFrequency,order,0,0)
    else:
      # odd number,check dictionary for support
        self.a = self.coefficient_dict[order][0]
        self.b = self.coefficient_dict[order][1]
        
    print "a=",self.a
    print "b=",self.b
    if (self.a == [] or self.b == []):
      raise NotImplementedError
    
    # initialized the ranges verts
    self.oldAvgRangeVerts = []
    self.newAvgRangeVerts = []
    
    for i in range (0,self.order+1):
      self.oldAvgRangeVerts.append(0.0)
      self.newAvgRangeVerts.append(0.0)

    self.numOfTilts = 0
    self.maxCorruptedTilts = self.maxCorruptedTilts_dict[self.order]
    if self.maxCorruptedTilts == 0:
      self.maxCorruptedTilts = self.MaxCorruptedTiltsForAllOrders

  def correctTilt(self,tilt,avg):
  # return (newTilt,newAvg)

    # compute the new avg
    newAvg = self.a[0]*avg
    i = 1
    while i <= self.order:
      newAvg += self.a[i]* self.oldAvgRangeVerts[i-1]+\
                self.b[i]* self.newAvgRangeVerts[i-1]
      i += 1
        
    # remove earlist data from end of lists
    self.oldAvgRangeVerts.pop(-1)
    self.newAvgRangeVerts.pop(-1)
    # add new data to the begining of lists
    self.oldAvgRangeVerts.insert(0,avg)
    self.newAvgRangeVerts.insert(0,newAvg)

    
    # At the beginning of the time serices, the data will be corrupted
    # by recursive filter, we need to ignore the corrected tilt & use original one
    self.numOfTilts += 1
    if self.numOfTilts < self.maxCorruptedTilts:
      newTilt = tilt
    else:
      # compute the new tilt
      newTilt = tilt + ( 1.0 - avg / newAvg) / tan(tilt)
    
      
    return (newTilt,newAvg)

  def _computeCoefficient(self,cutoffFreq=0.02,numOfPoles=4,percent_ripple=0,LH=0):   
    '''
      Recursion coefficient calculation for both chebyshev & butterworth
      filters. When ripple is 0, the calculation is for butterworth,
      LH=0 for low-pass,
      LH=1 for high-pass

      return (a,b)

      note: numOfPoles must be event
    '''
    
    #LH = 0   # low-pass only
    maxArraySize = 22
    zeroArray = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,\
                 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,\
                 0.0,0.0]
    A = array('d',zeroArray) # holds the "a" coefficient upon program completion
    B = array('d',zeroArray) # holds the "a" coefficient upon program completion
    TA= array('d',zeroArray)
    TB= array('d',zeroArray)
    A[2] = 1.0
    B[2] = 1.0

    p = 1
    while p <= numOfPoles/2:
      (A0,A1,A2,B1,B2) = self._recursiveCoefficientPerStage(cutoffFreq,numOfPoles,LH,percent_ripple,p)

      # Add coefficients to the cascade 
      for i in range (0, maxArraySize): 
        TA[i] = A[i]
        TB[i] = B[i]

      for i in range (2, maxArraySize):
        A[i] = A0*TA[i] + A1*TA[i-1] + A2*TA[i-2]
        B[i] = TB[i] - B1*TB[i-1] - B2*TB[i-2]

      p +=1

    # Finish combining coefficients
    B[2] = 0.0
    for i in range (0,maxArraySize-2):
      A[i] = A[i+2]
      B[i] = -B[i+2]

    # NORMALIZE THE GAIN
    SA = 0.0
    SB = 0.0
    for i in range (0,maxArraySize-2):
      if LH == 0:
        SA += A[i]
        SB += B[i]
      else:
        SA += A[i] * (-1)**i
        SB += B[i] * (-1)**i

    GAIN = SA / (1.0 - SB)
    
    for i in range (0,maxArraySize-2):
      A[i] = A[i] / GAIN

    # The final recursion coefficients are in A[ ] and B[ ]
    
    a = []
    b = []
    for i in range (0,numOfPoles+1):
      a.append(A[i])
      b.append(B[i])

    return (a,b)
    

  def _recursiveCoefficientPerStage(self,cutoffFreq,numOfPoles,LH,percent_ripple,p):
    # calculate the pole location on the unit circle
    RP = -cos(pi/(numOfPoles*2.0) + (p-1)*pi/numOfPoles)
    IP = sin(pi/(numOfPoles*2.0) + (p-1)*pi/numOfPoles)
    if (percent_ripple > 0):
      ES = sqrt((100.0/(100-percent_ripple))**2 - 1.0)
      VX = (1.0/numOfPoles)*log((1.0/ES)+sqrt((1.0/ES**2)+1.0))
      KX = (1.0/numOfPoles)*log((1.0/ES)+sqrt((1.0/ES**2)-1.0))
      KX = (exp(KX) + exp(-KX))/2.0
      RP = RP * ((exp(VX) - exp(-VX))/2)/KX
      IP = IP * ((exp(VX) + exp(-VX))/2)/KX

    # s-domain to z-domain conversion
    T = 2.0*tan(0.5)
    W = 2.0*pi*cutoffFreq
    M = RP**2 + IP**2
    D = 4.0-4.0*RP*T+M*T**2
    X0= T**2/D
    X1=2.0*X0
    X2=X0
    Y1=(8.0-2.0*M*T**2)/D
    Y2=(-4.0-4.0*RP*T-M*T**2)/D

    # low-pass to low-pass or low-pass to high-pass transform
    if LH == 1:
      K = -cos(W/2.0+0.5)/cos(W/2.0-0.5)
    else:
      K = sin(0.5-W/2.0)/sin(0.5+W/2.0)
    D  = 1.0+Y1*K - Y2*K**2
    A0 = (X0 - X1*K + X2*K**2)/D
    A1 = (-2.0*X0*K + X1 + X1*K**2 - 2.0*X2*K)/D
    A2 = (X0*K**2 - X1*K + X2)/D
    B1 = (2.0*K + Y1 + Y1*K**2 - 2.0*Y2*K)/D
    B2 = (-(K**2) - Y1*K + Y2)/D
    if LH == 1:
      A1 = -A1
      B1 = -B1

    return (A0,A1,A2,B1,B2)
  
# -----------------------------------------------------------------
# LidarData_RLPfiltered
#
#  Derived class of LidarData that implements the recusive low-pass filter.
#
#  Use getData to retrieve a valid scan line data in format
#   (timestamp,tilt,numOfRanges,[ranges])
#
#  This class supports iterator behavior for convenient.
#
#
# -----------------------------------------------------------------
class LidarData_RLPfiltered(LidarData):
  
  def __init__(self,lidar_file,tilt_ignore,startTime,endTime,startAngle=70,endAngle=110,\
               cyclesOfTiltLagRange=0,freqCutoff=0.02,order=4):
    print "LidarData_RLPfiltered init with startAngle=%d endAngle=%d freqCutoff=%f order=%d" % (startAngle,endAngle,freqCutoff,order)
    # initial the base class
    LidarData.__init__(self,lidar_file,tilt_ignore,startTime,endTime,cyclesOfTiltLagRange),

    self.filter = Butterworth_Filter(freqCutoff,order)
    # since angle step is 1 degree, we can simply do the following assignment
    if (startAngle > 1):
      self.startRangesIndex = startAngle-1
    else:
      self.startRangesIndex = 0
      
    if (endAngle < 180):
      self.endRangesIndex = endAngle
    else:
      self.endRangesIndex = 179

    # for dumping average related data only
    self.filteredAvgRange = 0.0
    self.avgRange = 0.0

  def __iter__(self):
        return self
    
  def next(self):
    data = self.getData()
    if (data == []):
      raise StopIteration
    
    return data
      
  def getData(self):
    # get a scan line from LidarData
    line = self.get()    
    if (line == []):
      return []
    
    # line has (time,tilt,numOfRanges,Ranges)  
    self.avgRange = self._avg(line[3],self.startRangesIndex,self.endRangesIndex)
    (tilt,self.filteredAvgRange) = self.filter.correctTilt(line[1],self.avgRange)
    
    return (line[0],tilt,line[2],line[3])
  
  # for dumping average range related data
  def getAvgRanges(self):
    return (self.avgRange,self.filteredAvgRange)
  
# -----------------------------------------------------------------
# BackwardFilter
#
#    A simple filter algorithm. It removes scan lines that
#   "go backward".
#
#   getVert() retrieves valid line points in (x,y,z) format
#   (pose,[(x,y,z),...])
#
#   getData() retrieves valid line data in format
#   (timestamp,tilt,numOfRanges,[ranges])
#
# Note:
#  It assumes the vehicle is moving forward. It checks two adjacent
#  lines(l_t1, l_t2, t2>t1) whether overlap each other. If they are,
#  set the second line(l_t2) to invalid.
#
#  It first sets a reference point, and update the reference point when
#  scanner head at the point to turn up from down & the scan line is valid.
#  Then, it compares distances of two points on the adjacent lines(l_t1,
#  l_t2) at the same angle to the reference point. If points at t2 is less
#  and the corresponding points at l_t2, the l_t2 is invalid
#
# -----------------------------------------------------------------
class BackwardFilter:
  START_ANGLE=(-3.1415926/2.0)
  ANGLE_STEP=(2.0*3.1415926/360.0)
 
  def __init__(self,gps,lidar,startAngle=70,endAngle=110):
    #print "BackwardFiler init with startAngle=%d endAngle=%d" % (startAngle,endAngle)
    self.reference_pos=[]
    self.tilt1 = 0.0
    self.tilt2 = 0.0
    self.verts1=[]
    self.distances1 = []
    self.pose=[]
    self.verts2=[]
    self.distances2 = []
    self.ray = lidar_ray()
    self.lidar = lidar
    self.bLastVertValid=False
    self.reset_referencePos = False
    # since angle step is 1 degree, we can simply do the following assignment
    if (startAngle > 1):
      self.startScanDataPoint = startAngle-1
    else:
      self.startScanDataPoint = 0
      
    if (endAngle < 180):
      self.endScanDataPoint = endAngle
    else:
      self.endScanDataPoint = 179
    self.gps = gps
    self.num_scanline = 0
    self.num_invalidLine = 0

  def __iter__(self):
        return self
    
  def next(self):
    data = self.getData()
    if (data == []):
      raise StopIteration
    
    return data
      
  def getVerts(self):
    global bPrintTrace
    while 1:
      # get a scan line from LidarData
      try:
        self.line = self.lidar.next()
      except StopIteration:
        # not more data
        return []
      
      self.num_scanline += 1
      # line has (time,tilt,numOfRanges,Ranges)
      self.pose = self.gps.pose(self.line[0])
      if (self._isValidScanLine(self.pose,self.line[0],self.line[1],\
                                self.line[2],self.line[3]) == False):
        if (bPrintTrace):
          print "scan line ",self.num_scanline," is invalid"
            
        self.num_invalidLine += 1
        # invalid, skip
        continue

      break

    # found a valid line
    return (self.pose,self.verts2)

  def getData(self):
    global bPrintTrace
    while 1:
      # get a scan line from LidarData
      try:
        self.line = self.lidar.next()
      except StopIteration:
        # not more data
        return []
      
      self.num_scanline += 1
      # line has (time,tilt,numOfRanges,Ranges)
      self.pose = self.gps.pose(self.line[0])
      if (self._isValidScanLine(self.pose,self.line[0],self.line[1],\
                                self.line[2],self.line[3]) == False):
        if bPrintTrace:
          print "scan line ",self.num_scanline," is invalid"
        self.num_invalidLine += 1
        # invalid, skip
        continue

      break

    # found a valid line
    return self.line

  def getNumOfScanLines(self):
    return self.num_scanline

  def getNumOfInvalidScanLines(self):
    return self.num_invalidLine
  
  def _isValidScanLine(self,pos,timestamp,tilt,valueCount,scanData):
    global bPrintTrace
    
    # In current version, we use two lines to do validation.
    if (self.bLastVertValid):
      self.verts1 = self.verts2
      self.distances1 = self.distances2


    self.verts2=[]
    # calculate the verts from line data
    self.ray.set_tilt(3.1415926-tilt)     # scanner tilt is down
    self.ray.set_pose(pos)

    if (valueCount == 181):
      theta = self.START_ANGLE
    else:
      theta=self.START_ANGLE+0.5*self.ANGLE_STEP
      
    for i in range(0,valueCount):
      self.ray.set_theta(theta)
      self.ray.set_value(scanData[i])
      self.verts2.append(self.ray.projection())
      theta+=self.ANGLE_STEP

    # assume the scan line is valid
    self.bLastVertValid=True
    
    if self.reference_pos == []:
      # first scan line, valid
      self.reference_pos = pos
      self.distances2=self._distancesFromVehiclePosition(pos,self.verts2)
      self.tilt2 = tilt
      self.tilt1 = tilt
      return self.bLastVertValid
    
    # calculate the distances from vehicle position
    self.distances2=self._distancesFromVehiclePosition(self.reference_pos,self.verts2)

    s1=0
    s2=0
    while 1:
      '''
      if (self.tilt1 <= self.tilt2) and (self.tilt2 <= tilt):
        # scan head keep turning up. don't need check overlap
        break;
      '''
      
      if (self.tilt1 >= self.tilt2) and (self.tilt2 < tilt):
        if (bPrintTrace):
          print "tilt turn up point"
          
        # need to check overlap too
        self.reset_referencePos = True
      
      elif (bPrintTrace) and (self.tilt1 <= self.tilt2) and (self.tilt2 > tilt):
          print "tilt turn down point"
        
      '''
        check overlap
      '''

      for i in range(0,len(self.distances1)):
        if (self.distances1[i] >= self.distances2[i]):
          if (s1 <= 0):
            if (s2 == 0):
              s1 -= 1
            elif (s2 > 0):
              # overlap
              self.bLastVertValid = False
              break
            else:
              print "impossible"
          else: # s1 > 0
            if (s2 <= 0):
              s2 -= 1
            else:
              print "impossible"
        else:
          if (s1 >= 0):
            if (s2 == 0):
              s1 += 1
            elif (s2 < 0):
              # overlap
              self.bLastVertValid = False
              break
            else:
              print "impossible"
          else: # s1 < 0
            if (s2 >= 0):
              s2 += 1
            else:
              print "impossible"
              
      if (self.bLastVertValid == False):
        break
      
      if (self.bLastVertValid and s1 < 0 and s2 == 0):
        # totally backward overlap
        self.bLastVertValid = False
        break

      break
    
    if (bPrintTrace):
      print "Number of ranges to be compared ",len(self.distances1),"s1=",s1,"s2=",s2
      if (self.bLastVertValid and s1 < 0 and s2 > 0):
        print "Right turn"
      elif (self.bLastVertValid and s1 > 0 and s2 < 0):
        print "Left trun"
      
    self.tilt1 = self.tilt2
    self.tilt2 = tilt
    
    # setup reference point
    if (self.bLastVertValid) and (self.reset_referencePos):
      if (bPrintTrace):
        print "reset reference position on ",datetime(1,1,1).fromtimestamp(timestamp).isoformat(' ')
        
      self.reference_pos = pos
      # re-calculate distances2
      self.distances2=self._distancesFromVehiclePosition(self.reference_pos,self.verts2)
      self.reset_referencePos = False

    return self.bLastVertValid

  def _getLastValidLine(self):
    return self.verts2

  def _distancesFromVehiclePosition(self,pos,verts):
    distances = []

    for i in range (self.startScanDataPoint,self.endScanDataPoint):
      dx=verts[i][1]-pos[0]
      dy=verts[i][2]-pos[1]
      dz=verts[i][3]-pos[2]
      d2=dx*dx+dy*dy+dz*dz
      #d2=dx*dx+dy*dy
      distances.append(sqrt(d2))
      
    return distances

################################################################################
#
# GPS related classes
#
################################################################################
# -----------------------------------------------------------------
# 3d coordinate system (right handed)
# x is forward
# y is left
# z is up
# theta is rotations in the xy plane, x axis is zero (forward)
# tilt is rotations in the zx plane, z axis is zero (up)
# -----------------------------------------------------------------
GPS_POSETYPE_FIXEDPOS=1
GPS_POSETYPE_FIXEDHEIGHT=2
GPS_POSETYPE_FIXEDVEL=3
GPS_POSETYPE_DOPPLER_VELOCITY=8
GPS_POSETYPE_SINGLE=16     # purely non differential service
GPS_POSETYPE_PSRDIFF=17
GPS_POSETYPE_WAAS=18       # third best service, between omnistar and single
GPS_POSETYPE_OMNISTAR=20  #second best service, between hp and waas
GPS_POSETYPE_L1_FLOAT=32
GPS_POSETYPE_IONOFREE_FOAT=33
GPS_POSETYPE_NARROW_FLOAT=34
GPS_POSETYPE_L1_INT=48
GPS_POSETYPE_WIDE_INT=49
GPS_POSETYPE_NARROW_INT=50
GPS_POSETYPE_RTK_DIRECT_INS=51
GPS_POSETYPE_INS=52
GPS_POSETYPE_OMNISTAR_HP =64    #the best decimeter level service

#------------------------------------------------------------------------------
'''
struct GPSINSMsgRep
{
    uint64_t timestamp;			  // nanoseconds since epoch
    GPSINS_MSG::Err err;      		  //	see above
    GPSINS_MSG::SolStatusEnum posStat;    // position status
    GPSINS_MSG::PosVelTypeEnum posType;   // pos type

	//	Postional information. 
	//	North, East, Down coordinate system is relative to current GPS coord system origin.
	//	Distances are in meters. Angles are in degrees (not radians).
    double llh[3];		// Latitude(deg), Longitude(deg) Height(m)
    double pos[3];		// North, East, Down (m)
    float  unc[3]; 		// Uncertainty (North, East Down)  (m)
    float  vel[3]; 		// Velocity (North, East, Down)  (m/s)
    float  acc[3]; 		// Acceleration (North, East Down) (m/s^2)
    float  rpy[3];		// Roll, Pitch, Yaw (deg)
    float  pqr[3];		// Roll Rate, Pitch Rate, Yaw Rate (deg/sec)
};

This struct is defined in src\qnx\common\include\gpsins_messaging.h
'''
#------------------------------------------------------------------------------

# -----------------------------------------------------------------
class gps_visualization:
# -----------------------------------------------------------------
  msg_fmt="=QIII3d3d3f3f3f3f3f"               # format of gps message
  msg_len=struct.calcsize(msg_fmt)            # length of gps message in bytes
  startGPSposX=0.0
  startGPSposY=0.0
  startGPSposZ=0.0

  time1=0.0
  pose1=( 0.0, 0.0, 0.0,           # position
          1.0, 0.0, 0.0,           # x axis
          0.0, 1.0, 0.0,           # y axis
          0.0, 0.0, 1.0            # z axis
        )
  time2=0.0
  pose2=( 0.0, 0.0, 0.0,           # position
          1.0, 0.0, 0.0,           # x axis
          0.0, 1.0, 0.0,           # y axis
          0.0, 0.0, 1.0            # z axis
        )

  vehMovingDir=0.0
  bDumpGPS = False
  rawGPSdata1=(0,0,0.0,0.0,0.0,0.0,0.0,0.0,0.0) # (solStatus,poseType,X,Y,Z,roll,pitch,yaw)
  rawGPSdata2=(0,0,0.0,0.0,0.0,0.0,0.0,0.0,0.0)
  firstLine=True

  checkGPSdataStatus=False
  
  def set_file(self,file):
    self.infile=file

  def consume(self):
    global GPS_POSETYPE_OMNISTAR_HP,GPS_POSETYPE_OMNISTAR,GPS_POSETYPE_WAAS
      
    while 1: # just for now
      buf=self.infile.read(self.msg_len)
      if(len(buf)==0): return False
      msg=struct.unpack(self.msg_fmt,buf)
      t=float(msg[0])*0.000000001
      if(self.time2>=t-0.0001): return True
      # check the SolStatus msg[2] to SOL_COMPUTED (0)
      # pose type msg[3] to be OMNISTAR_HP (64)
      if (self.checkGPSdataStatus and (msg[2] != 0 or msg[3] != GPS_POSETYPE_OMNISTAR_HP)):
        continue

      break

    '''
      To convert from local coords to blender coords
        roll then pitch then yaw
        
      local coords is right handed system, x forward, y right, z down
      world coords is right handed system, y backward, x right, z up
      blender coords is right handed system, y forward, x right, z up
     
      Convert:
      1. roll msg[19]
      1. pitch msg[20]+ 180 degree
      2. yaw  msg[21]+ 90 degree
      3. a vehicle pose in blender coords is (Yl,Xl,-Zl) where (Xl,Yl,Zl)
         is a pose in local coords
    '''
#    roll=(-msg[19]/360.0*2.0*3.1415926)  # original
#    pitch=(-msg[20]/360.0*2.0*3.1415926) # original
#    yaw=((-msg[21])/360.0*2.0*3.1415926) # original

    vehPoseX = msg[8]-self.startGPSposX
    vehPoseY = msg[7]-self.startGPSposY
    vehPoseZ = -msg[9]-self.startGPSposZ

    #use DRMS to show 2-D GPS error
    self.GPSerror = sqrt(msg[10]*msg[10] + msg[11]*msg[11])
    
    roll=radians(msg[19])
    pitch=radians(msg[20]+180)
    
    if VEHICLE_MOVING_DIR != True:
      yaw=radians(msg[21]+90)
    else:
      vehMovingDir = self.vehAngle((vehPoseY, vehPoseX),(self.pose1[1],self.pose1[0]),self.vehMovingDir)
      yaw = radians(vehMovingDir+90)

    cy=cos(yaw)
    sy=sin(yaw)
    cp=cos(pitch)
    sp=sin(pitch)
    cr=cos(roll)
    sr=sin(roll)
    '''
    # original
    xax=(  cp*cy,
          -cp*sy,
           sp
        )
    yax=( -sr*sp*cy+cr*sy,
           sr*sp*sy+cr*cy,
           sr*cp
        )
    zax=( -sr*sy-cr*sp*cy,
          -sr*cy+cr*sp*sy,
           cr*cp
        )
    '''
    '''
      Rotation Matrix is computed as
      R = Rx X Ry X Rz, where
      
         | 1    0          0          |     | cos(pitch)  0 sin(pitch)|     | cos(yaw) -sin(yaw) 0|
      Rx=| 0    cos(roll)  -sin(roll) |  Ry=| 0           1 0         |  Rz=| sin(yaw)  cos(yaw) 0|
         | 0    sin(roll)  cos(roll)  |     | -sin(pitch) 0 cos(pitch)|     | 0         0        1|

    '''
    xax=(  cp*cy,
          -cp*sy,
           sp
        )
    yax=( sr*sp*cy+cr*sy,
          -sr*sp*sy+cr*cy,
           -sr*cp
        )
    zax=( -cr*sp*cy+sr*sy,
          cr*sp*sy+sr*cy,
          cr*cp
        )
    
    self.time1=self.time2
    self.pose1=self.pose2
    self.time2=t
    
    
    if(FORCE_GPS_Z_ZERO):
      self.pose2=( vehPoseX, vehPoseY, 0.0,
                   xax[0], xax[1], xax[2],
                   yax[0], yax[1], yax[2],
                   zax[0], zax[1], zax[2]
                 )
    else:
      self.pose2=( vehPoseX, vehPoseY, vehPoseZ,
                   xax[0], xax[1], xax[2],
                   yax[0], yax[1], yax[2],
                   zax[0], zax[1], zax[2]
                 )
    '''
    dx=self.pose2[0]-self.pose1[0]
    dy=self.pose2[1]-self.pose1[1]
    dz=self.pose2[2]-self.pose1[2]

    print "gps consume %f, dist=%f" % (self.time2,sqrt(dx*dx+dy*dy+dz*dz))
    '''
    
    # for dumping gps data
    self.rawGPSdata1 = self.rawGPSdata2
    self.rawGPSdata2 =(msg[2],msg[3],msg[7],msg[8],msg[9],msg[19],msg[20],msg[21]) 

    if (self.firstLine):
      self.pose1 = self.pose2
      self.rawGPSdata1 = self.rawGPSdata2
      self.firstLine = False
    
    return True

  def skip(self,t):
    while 1:
      if(self.time2>=t and self.time2 != 0.0): break
      if (self.consume() == False): break

  def currentError(self):
    return self.GPSerror

  def currentPose(self):
    return self.pose1
  
  def dump(self):
    print "gps position is %f,%f,%f" % (self.pose1[0],self.pose1[1],self.pose1[2])
    print "gps xax is %f,%f,%f" % (self.pose1[3],self.pose1[4],self.pose1[5])
    print "gps yax is %f,%f,%f" % (self.pose1[6],self.pose1[7],self.pose1[8])
    print "gps zax is %f,%f,%f" % (self.pose1[9],self.pose1[10],self.pose1[11])

  def pose(self,t):
    if(t>self.time2): self.consume()
    if(t<self.time1): 
      print "underrun"
      return(self.pose1)
    if(t>self.time2):
      print "overrun"
      return(self.pose2)
    
    
    pose=[ self.pose1[0], self.pose1[1], self.pose1[2],    # position
           self.pose1[3], self.pose1[4], self.pose1[5],    # x axis
           self.pose1[6], self.pose1[7], self.pose1[8],    # y axis
           self.pose1[9], self.pose1[10], self.pose1[11]   # z axis
         ]
    
    f=1.0-(t-self.time1)/(self.time2-self.time1)
    
    pose[0]=self.pose1[0]*f+self.pose2[0]*(1.0-f)
    pose[1]=self.pose1[1]*f+self.pose2[1]*(1.0-f)
    pose[2]=self.pose1[2]*f+self.pose2[2]*(1.0-f)
    pose[3]=self.pose1[3]*f+self.pose2[3]*(1.0-f)
    pose[4]=self.pose1[4]*f+self.pose2[4]*(1.0-f)
    pose[5]=self.pose1[5]*f+self.pose2[5]*(1.0-f)
    # normalized
    m=sqrt(pose[3]*pose[3]+pose[4]*pose[4]+pose[5]*pose[5]);
    pose[3]=pose[3]/m
    pose[4]=pose[4]/m
    pose[5]=pose[5]/m
    pose[6]=self.pose1[6]*f+self.pose2[6]*(1.0-f)
    pose[7]=self.pose1[7]*f+self.pose2[7]*(1.0-f)
    pose[8]=self.pose1[8]*f+self.pose2[8]*(1.0-f)
    m=sqrt(pose[6]*pose[6]+pose[7]*pose[7]+pose[8]*pose[8]);
    pose[6]=pose[6]/m
    pose[7]=pose[7]/m
    pose[8]=pose[8]/m
    pose[9]=self.pose1[9]*f+self.pose2[9]*(1.0-f)
    pose[10]=self.pose1[10]*f+self.pose2[10]*(1.0-f)
    pose[11]=self.pose1[11]*f+self.pose2[11]*(1.0-f)
    m=sqrt(pose[9]*pose[9]+pose[10]*pose[10]+pose[11]*pose[11]);
    pose[9]=pose[9]/m
    pose[10]=pose[10]/m
    pose[11]=pose[11]/m
    
    
    return(pose)

  def getPoses(self):
    return (self.pose1,self.pose2)

  def getTimes(self):
    return (self.time1,self.time2)

  def getModes(self):
    return ((self.rawGPSdata1[0],self.rawGPSdata1[1]),(self.rawGPSdata2[0],self.rawGPSdata2[1]))
  
  def setGPSstartPose(self):
    self.startGPSposX = self.pose1[0]
    self.startGPSposY = self.pose1[1]
    self.startGPSposZ = self.pose1[2]
    self.pose1=( 0.0, 0.0, 0.0,
                 self.pose1[3], self.pose1[4], self.pose1[5],
                 self.pose1[6], self.pose1[7], self.pose1[8],
                 self.pose1[9], self.pose1[10],self.pose1[11]
                 )
    self.pose2=( self.pose2[0]-self.startGPSposX, self.pose2[1]-self.startGPSposY,self.pose2[2]-self.startGPSposZ,
                 self.pose2[3], self.pose2[4], self.pose2[5],
                 self.pose2[6], self.pose2[7], self.pose2[8],
                 self.pose2[9], self.pose2[10],self.pose2[11]
                 )

  def setCheckGPSdataStatus(self,status=True):
    self.checkGPSdataStatus = status

  def getCheckGPSdataStatus(self):
    return self.checkGPSdataStatus
    
  def dump_poses(self,startTime,endTime):
    self.bDumpGPS = True
    self.skip(startTime)
    angle = 0.0
    startPose = (self.rawGPSdata1[2],self.rawGPSdata1[3],self.rawGPSdata1[4])
    while 1:
      rx = self.rawGPSdata1[2] - startPose[0]
      ry = self.rawGPSdata1[3] - startPose[1]
      rz = self.rawGPSdata1[4] - startPose[2]
      print datetime(1,1,1).fromtimestamp(self.time1).isoformat(' '),
      # rawGPSdata1= (solStatus,poseType,X,Y,Z,roll,pitch,yaw)
      print "SS",self.rawGPSdata1[0],"PT",self.rawGPSdata1[1],
      print "xyz (%f,%f,%f)" % (self.rawGPSdata1[2],self.rawGPSdata1[3],self.rawGPSdata1[4]),
      angle = self.vehAngle((self.rawGPSdata2[2],self.rawGPSdata2[3]),(self.rawGPSdata1[2],self.rawGPSdata1[3]),angle)
      deltaAngle=angle-self.rawGPSdata1[7]
      if (deltaAngle > 181.0):
        deltaAngle -= 360
      elif deltaAngle < -181:
        deltaAngle += 360

      print "rpy (%f,%f,%f)" % (self.rawGPSdata1[5],self.rawGPSdata1[6],self.rawGPSdata1[7]),
      #print "                          ",
      print "r-xyz=(%f,%f,%f)" % (rx,ry,rz),
      print "MD=%f" % angle,"delta=%f" % deltaAngle
      
      if (self.consume() == False):
        break
      if self.time2 > endTime:
        break;

  def vehAngle(self,XY2,XY1,oldAngle):
    dx = XY2[0]-XY1[0]
    dy = XY2[1]-XY1[1]
    if dx == 0.0 and dy == 0.0:
      # vehicle didn't move
      return oldAngle

    angle = atan(dy/dx)
    if dx < 0.0:
      angle = pi + angle

    return (angle*180/pi)
          
  def getTimeRange(self, filename):
    '''
      get first record timestamp & last record timestamp from GPS log file
    '''
    # open for read
    try:
      infile=open(filename,"rb")
    except IOError,(errno,strerror):
      print "Open file <%s>, IOError #%s: %s" % (filename,errno,strerror)
      return(0,0)

    '''
      Read first record to retrieve the first recorded time
    '''
    buf=infile.read(self.msg_len)
    if (len(buf) < self.msg_len): return (0,0)  # don't have any data
    
    msg=struct.unpack(self.msg_fmt,buf)   
    startTime = msg[0]*0.000000001 # convert nanoseconds to  microseconds

    '''
      seek to last record, and retrieve the last recorded time
    '''
    fileSize = os.path.getsize(filename)
    recordNum = int(fileSize/self.msg_len)
    if (recordNum > 1):
      offset = (recordNum-1)*self.msg_len
      infile.seek(offset)
      buf=infile.read(self.msg_len)
      msg=struct.unpack(self.msg_fmt,buf)
      endTime = msg[0]*0.000000001
    else:
      endTime = startTime

    # close
    infile.close()

    return (startTime,endTime)

# -----------------------------------------------------------------
class lidar_ray:
# -----------------------------------------------------------------
  '''this class stores the orientation of a lidar ray and projects it into a 3D 4-tuple'''
  theta=0.0
  tilt=0.0
  value=0
  pose=( 0.0, 0.0, 0.0,           # position
         1.0, 0.0, 0.0,           # x axis
         0.0, 1.0, 0.0,           # y axis
         0.0, 0.0, 1.0            # z axis
       )
  scaling=0.01   # convert scanner centimeters to meters
  scannerOffset=(0.0,0,0.0)
#  scannerOffset=(1.65,0,2.08)

  def set_theta(self,theta):
    self.theta=theta

  def set_tilt(self,tilt):
    self.tilt=tilt

  def set_value(self,value):
    self.value=value

  def set_pose(self,pose):
    self.pose=pose

  # Rx(s)=Px+Ox*K*s : s in [0,1]
  def projection(self):
    Px=self.pose[0]
    Py=self.pose[1]
    Pz=self.pose[2]

    sa=sin(self.tilt)
    ca=cos(self.tilt)
    sb=sin(self.theta)
    cb=cos(self.theta)
    
    Ox=cb*sa*self.pose[3]+sb*self.pose[6]+cb*ca*self.pose[9]
    Oy=cb*sa*self.pose[4]+sb*self.pose[7]+cb*ca*self.pose[10]
    Oz=cb*sa*self.pose[5]+sb*self.pose[8]+cb*ca*self.pose[11]
    
    # fmylb, handle error conditions
    K=self.value*self.scaling
    v=( self.value<8183,      # valid
        Px+self.scannerOffset[0]+Ox*K,              # x
        Py+self.scannerOffset[1]+Oy*K,              # y
        Pz+self.scannerOffset[2]+Oz*K               # z
      )
    return(v)


################################################################################
#
# lidar_data stand alone program
#
#   print lidar scan data.
#
################################################################################
usage_msg = """\

USAGE: lidar_data [-h] [-g filename] [-l filename] [-s startTime][-m number]
                  [-e endTime] [-i tilt_ignore] [-f low-pass-filter] [-w size]
                  [-f backward-filter] [-f butterworth-filter] [-d order] [-q cutoffFreq]
                  [-a left angle] [-r right angle] [-p] [-n] [-c cyclesOfTiltLagRange]
                  [-o [lineNumber|timestamp|tilt|angle|delta-timestamp
                       |delta-tilt|delta-angle|rangeCount|ranges|avg-range|filtered-avg-range]*] |
                  [-t cutoff-freq number-poles percent-ripple]
                  [-x]
                  
  -h      print this help message and exit
  -g      gps log filename (default c:\gps.log)
  -l      lidar log filename (default c:\lidar.log)
  -s      start time in format YYYY-MM-DD hh:mm:ss
  -e      end time in format YYYY-MM-DD hh:mm:ss
  -i      tilt in degree to be ignored (default 0)
  -f      enable filtering, low-pass-filter, butterworth-filter or backward-filter
  -w      window size for low pass filter (default 7)
  -o      output fields, by default, output all fields
  -m      max. number of lines to be printed
  -a      left angle from straight ahead , for example, -20
  -r      right angle from straight ahead , for example, 20
  -p      print trace messages
  -n      print hamming weights
  -c      number of cycles which tilt data lags range data (default 0)
  -d      order size for butterworth low-pass filter
  -q      cut off frequency for butterworth low-pass filter
  -t      print recursion coefficient for Chebyshev or Butterworth filter
          cutoff-freq in (0,0.5) percent-ripple in (0-29) number-poles in (2,4,...20)
  -x      dump gps 
  
"""

TIME_STRING_FORMAT='%Y-%m-%d %H:%M:%S' # time format example, 2005-07-15 16:21:01

import os
import sys
import getopt
import re
def main():
  global bPrintTrace
  gps_filename = 'c:\gps.log'
  lidar_filename='c:\lidar.log'
  startTime = 0
  endTime = time.time()
  tilt_ignore = 0
  windowSize = 7
  bw_order=4
  bw_cutoffFreq=0.02
  bLowPassFilterEnable = False
  bButterworthFilterEnable = False
  bBackwardFilterEnable = False
  bOut_lineNumber = False
  bOut_timeStamp = False
  bOut_tilt = False
  bOut_deltaTimeStamp = False
  bOut_deltaTilt = False
  bOut_deltaAngle = False
  bOut_angle = False
  bOut_rangeCount = False
  bOut_ranges = False
  bOut_all = True
  bOut_allRangeData = True
  leftAngle  = -90
  rightAngle = 90
  out_maxLineNumber=0
  bPrintHammingWeights=False
  bPrintRecursionCoefficient=False
  cyclesOfTiltLagRange = 0
  LH = 0
  bDumpGps=False
  bPrintAvgRange=False
  bPrintNewAvgRange=False
  
  try:
    opts, args = getopt.getopt(sys.argv[1:], "hg:l:s:e:i:f:w:om:a:r:pnc:d:tq:x")
  except getopt.error, msg:
    sys.stderr.write("Error: %s\n" % str(msg))
    sys.stderr.write(usage_msg)
    sys.exit(2)

  # parse options  
  for o, a in opts:
    if o == '-h':
      sys.stdout.write(usage_msg)
      sys.exit()

    if o == '-p':
      bPrintTrace = True
      continue
    
    if o == '-n':
      bPrintHammingWeights = True
      continue

    if o == '-g':
      gps_filename = a
      continue
    
    if o == '-l':
      lidar_filename = a
      continue
    if o == '-s':
      try:
        startTime = time.mktime(time.strptime(a,TIME_STRING_FORMAT))
        continue
      except (ValueError,OverflowError),msg:
        sys.stderr.write("Error: %s\n" % str(msg))
        sys.stderr.write(usage_msg)
        sys.exit(2)
    if o == '-e':
      try:
        endTime = time.mktime(time.strptime(a,TIME_STRING_FORMAT))
        continue
      except (ValueError,OverflowError),msg:
        sys.stderr.write("Error: %s\n" % str(msg))
        sys.stderr.write(usage_msg)
        sys.exit(2)

    if o == '-i':
      tilt_ignore = int(a)
      continue

    if o == '-f':
      re_lpf = re.compile(r'low-pass-filter',re.I)
      re_bf = re.compile(r'backward-filter',re.I)
      re_bwf = re.compile(r'butterworth-filter',re.I)
      if re_lpf.match(a):
        bLowPassFilterEnable = True
      if re_bf.match(a):
        bBackwardFilterEnable = True
      if re_bwf.match(a):
        bButterworthFilterEnable = True
      continue
        
    if o == '-w':
      windowSize = int(a)
      continue
    
    if o == '-d':
      bw_order = int(a)
      continue

    if o == '-q':
      bw_cutoffFreq = float(a)
      continue

    if o == '-o':
      bOut_all = False
      re_lineNumber = re.compile(r'lineNumber',re.I)
      re_timeStamp = re.compile(r'timeStamp',re.I)
      re_deltaTimeStamp = re.compile(r'delta-timestamp',re.I)
      re_tilt = re.compile(r'tilt',re.I)
      re_deltaTilt = re.compile(r'delta-tilt',re.I)
      re_angle = re.compile(r'angle',re.I)
      re_deltaAngle = re.compile(r'delta-angle',re.I)
      re_rangeCount = re.compile(r'rangeCount',re.I)
      re_ranges = re.compile(r'ranges',re.I)
      re_avg_range=re.compile(r'avg-range',re.I)
      re_filtered_avg_range=re.compile(r'filtered-avg-range',re.I)
      for arg in args:
        if re_lineNumber.match(arg):
          bOut_lineNumber = True
        elif re_timeStamp.match(arg):
          bOut_timeStamp = True
        elif re_deltaTimeStamp.match(arg):
          bOut_deltaTimeStamp = True
        elif re_tilt.match(arg):
          bOut_tilt = True
        elif re_deltaTilt.match(arg):
          bOut_deltaTilt = True
        elif re_angle.match(arg):
          bOut_angle = True
        elif re_deltaAngle.match(arg):
          bOut_deltaAngle = True
        elif re_rangeCount.match(arg):
          bOut_rangeCount = True
        elif re_ranges.match(arg):
          bOut_ranges = True

        elif re_avg_range.match(arg):
          bPrintAvgRange=True
        elif re_filtered_avg_range.match(arg):
          bPrintNewAvgRange=True
          
      continue
    
    if o == '-m':
      out_maxLineNumber = int(a)
      continue

    if o == '-c':
      cyclesOfTiltLagRange = int(a)
      continue
    
    if o == '-a':
      bOut_allRangeData = False
      leftAngle = int(a)
      continue
    
    if o == '-r':
      bOut_allRangeData = False
      rightAngle = int(a)
      continue

    if o == '-t':
      bPrintRecursionCoefficient=True
      cutoff_freq = float(args[0])
      number_poles=int(args[1])
      percent_ripple=float(args[2])
      continue

    if o == '-x':
      bDumpGps=True
      
  # finished parse command options
  '''
  # for debug only
  #bBackwardFilterEnable = True
  #bLowPassFilterEnable = True
  bButterworthFilterEnable = True
  bw_order=4
  bw_cutoffFreq=0.02
  bOut_lineNumber = True
  bOut_timeStamp = True
  bOut_tilt = True
  #bOut_deltaTimeStamp = False
  bOut_deltaTilt = True
  #bOut_deltaAngle = False
  #bOut_angle = False
  bOut_rangeCount = False
  #bOut_ranges = False
  bOut_all = False
  bOut_allRangeData = False
  leftAngle  = -20
  rightAngle = 20
  out_maxLineNumber=50
  #startTime = time.mktime(time.strptime('2005-07-05 21:04:45',TIME_STRING_FORMAT))
  #endTime = time.mktime(time.strptime('2005-07-05 21:04:46',TIME_STRING_FORMAT))
  cyclesOfTiltLagRange = 0
    
  bPrintRecursionCoefficient=True
  cutoff_freq = 0.1
  number_poles=4
  percent_ripple=0
  
  gps_filename = 'c:\\dev\\fred\\lms.data\\15AUG2005\\lgps20050815_122910.gpslog'
  startTime = time.mktime(time.strptime('2005-08-15 12:29:26',TIME_STRING_FORMAT))
  endTime = time.mktime(time.strptime('2005-08-15 12:29:51',TIME_STRING_FORMAT))
  bDumpGps =True
#  bPrintAvgRange=True
  bPrintNewAvgRange=True
  bButterworthFilterEnable = True
  bw_order=2
  bw_cutoffFreq=0.01
  '''
  
  if bDumpGps:
    global FORCE_GPS_Z_ZERO
    FORCE_GPS_Z_ZERO=0
    try:
      gps_file=open(gps_filename,"rb")
    except IOError,(errno,strerror):
      errmsg="IOError #%s: %s" % (errno,strerror)
      print errmsg
      sys.exit(2)
      
    gps=gps_visualization()
    gps.setCheckGPSdataStatus(False)
    gps.set_file(gps_file)
    gps.dump_poses(startTime,endTime)
    sys.exit(2)

  if bPrintHammingWeights:
    print "Hamming weight array on window size ",windowSize
    lpf = LowPassFilter()
    if lpf.isValidWindowSize(windowSize) == False:
      print "Invalid window size, must be odd number"
    else:
      print lpf._hamming(windowSize)
      
    sys.exit(2)
    
  if bPrintRecursionCoefficient:
    print "Recursion coefficient with cutoff_freq=",cutoff_freq,\
          " number_poles=",number_poles," percent_ripple=",percent_ripple
    bwf = Butterworth_Filter()
    print bwf._computeCoefficient(cutoff_freq,number_poles,percent_ripple,LH)
      
    sys.exit(2) 

  if (bLowPassFilterEnable and bButterworthFilterEnable):
    print "couldn't enable low-pass filter and butterworth filter at the same time"
    sys.exit(2)
    
  lidar_infile = open(lidar_filename,"rb")
  if bLowPassFilterEnable:
    startAngle = 90 + leftAngle
    endAngle = 90 + rightAngle
    lidar_data = LidarData_SLPfiltered(lidar_infile,tilt_ignore,startTime,\
                                       endTime,startAngle,endAngle,cyclesOfTiltLagRange,windowSize)
  elif bButterworthFilterEnable:
    startAngle = 90 + leftAngle
    endAngle = 90 + rightAngle
    lidar_data = LidarData_RLPfiltered(lidar_infile,tilt_ignore,startTime,\
                                             endTime,startAngle,endAngle,cyclesOfTiltLagRange,\
                                               bw_cutoffFreq,bw_order)
  else:
    lidar_data = LidarData(lidar_infile,tilt_ignore,startTime,endTime,cyclesOfTiltLagRange)
    
  if bBackwardFilterEnable:
    gps_infile = open(gps_filename,"rb")
    gps=gps_visualization()
    gps.setCheckGPSdataStatus(False)
    gps.set_file(gps_infile)
    gps.skip(startTime)
    startAngle = 90 + leftAngle
    endAngle = 90 + rightAngle
    lidar_data = BackwardFilter(gps,lidar_data,startAngle,endAngle)
  
  if (bPrintAvgRange or bPrintNewAvgRange) and (bButterworthFilterEnable != True and bLowPassFilterEnable != True):
    print " Average Range or Filtered Average Range data is not supported!"
    sys.exit(2)
      
  scanLine = 0
  time1 = 0.0
  for item in lidar_data:
    scanLine += 1
    
    if (bPrintAvgRange or bPrintNewAvgRange) and (bButterworthFilterEnable or bLowPassFilterEnable):
      #if time1 == 0.0:
        #time1 = item[0]

      deltaTime = item[0] - time1 # delta time in seconds
      print "%4.3f" % deltaTime,

      (avgRange,newAvgRange) = lidar_data.getAvgRanges()
      if (bPrintAvgRange):
        print ", %4.15f" % (avgRange),
        
      if bPrintNewAvgRange:
        print ", %4.15f" % (newAvgRange),
      
      print 
      continue
    
    timeStr = datetime(1,1,1).fromtimestamp(item[0]).isoformat(' ')
    if bOut_all:
      print "line %5d time %s tilt %2.15f %3d points" % (scanLine,timeStr,item[1], item[2]),item[3]
    else:
      if bOut_lineNumber:
        print "line %5d" % scanLine,
      if bOut_timeStamp:
        print " time %s" % timeStr,
      if bOut_deltaTimeStamp:
        print " delta-timestamp ",
        if scanLine == 1:
          # first line delta-timeStamp is 0
          print "0.000000",
        else:
          deltaTimeStamp = item[0] - time1
          print "%0.6f" % deltaTimeStamp,
        time1 = item[0]
      if bOut_tilt:
        print " tilt %2.15f" %item[1],
      if bOut_deltaTilt:
        print " delta-tilt ",
        if scanLine == 1:
          # first line delta-tilt is 0
          print "0.000000000000000",
        else:
          deltaTilt = item[1] - tilt1
          print "%2.15f" % deltaTilt,
        tilt1 = item[1]
      if bOut_angle:
        angle = 180*item[1]/(pi)
        print " angle %3.15f" %angle,
      if bOut_deltaAngle:
        angle = 180*item[1]/(pi)
        print " delta-angle ",
        if scanLine == 1:
          # first line delta-angle is 0
          print "0.000000000000000",
        else:
          deltaAngle = angle - angle1
          print "%3.15f" % deltaAngle,
        angle1 = angle
      if bOut_rangeCount:
        print " %3d points" % item[2],
      if bOut_ranges:
        if bOut_allRangeData:
          print item[3]
        else:
          # angle step is 1 degree 
          startRangeIndex = 90 + leftAngle
          if (startRangeIndex > 0):
            startRangeIndex -= 1
          endRangeIndex = 90 + rightAngle
          print "[",
          for i in range (startRangeIndex,endRangeIndex):
            print item[3][i],
          print "]"
      else:
        print ""

    if (out_maxLineNumber == 0):
    # print lines based on start time and end time
      continue
    else:
      if (scanLine >= out_maxLineNumber):
        # enough lines
        break

  if bBackwardFilterEnable:
    print "Total number of scan lines is ",lidar_data.getNumOfScanLines()
    print "Total number of invalid scan lines is ",lidar_data.getNumOfInvalidScanLines()

  lidar_infile.close()
      
if __name__ == '__main__':
    main()
      

