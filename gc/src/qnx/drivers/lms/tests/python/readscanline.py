#
#	readscanline.py  --  read one LIDAR scan line from a log file
#
#struct LidarScanLineHeader {
#	uint64_t	m_timestamp;				// CLOCK_REALTIME, in nanoseconds
#	float		m_tilt;						// tilt angle of unit. 0=straight down, pi/2=straight ahead
#	uint8_t		m_sensorid;					// which LMS (future expansion)
#	uint8_t		m_statusByte;				// LMS status byte
#	uint8_t		m_scanIndex;				// scan index (cyclic)
#	uint8_t		m_unused1;					// (future expansion)
#	uint16_t	m_unused2;					// (future expansiion, and fill to word)
#	uint16_t	m_valueCount;				// count of m_range
#};
#
#struct LidarScanLine {
#	LidarScanLineHeader m_header;			// the header
#	uint16_t	m_range[LMS_MAX_DATA_POINTS];// depth data (m_valuecount entries)
#};
import os
import struct

class LidarScanLine:
	'''One LIDAR scan line'''
	HeaderFormat = "LLfBBBBHHI"				# format of header part
	HeaderFormatL = struct.calcsize(HeaderFormat)	# header size (must be 24)
	DataFormat = "H"						# format of one data item
	DataFormatL = struct.calcsize(DataFormat)	# data item size
	TotalRecordL = 392						# entire record, including filler
	DataRecordL = TotalRecordL-HeaderFormatL # data part, including filler
	print "Header Length = ",HeaderFormatL	# ***TEMP***
	print "Data Length = ",DataFormatL		# ***TEMP***
	data = []								# part of object
	header = []								# part of object
	def readfile(self,infile):
		'''Read a scan line into the current object'''
		s = infile.read(self.HeaderFormatL)	# read one line
		if len(s) == 0:						# EOF check
			return(-1);
		self.header = struct.unpack(self.HeaderFormat,s)		# unpack the header
		self.datacnt = self.header[8]			# data item count
		self.scanindex = self.header[5]			# scan index (cyclic)
		self.tilt = self.header[2]				# tilt (radians)
		# print "Scan line.",header[0],header[1],datacnt," points.\n"			# ***TEMP***
		#	Compute timestamp in seconds from 64-bit nanosecond counter
		self.timestamp = (self.header[0]*0.000000001) + (self.header[1]*(4.294967296))
		self.data = []							# no data yet
		s = infile.read(self.DataRecordL);		# read all data, including filler
		for i in range(self.datacnt):			# for N data items
			rangedatum = s[i*self.DataFormatL:(i+1)*self.DataFormatL]
			# print "Length = ",len(rangedatum)
			datum = struct.unpack(self.DataFormat,rangedatum)	# unpack data item
			self.data.append(datum[0])			# build array of data items	
		return(0)

	def dump(self):
		'''Dump scan line'''
		# print "Scan data points: ",len(self.data)
		# print "Scan line.",self.header[0],self.header[1],self.datacnt," points."			# ***TEMP***
		angle = 180*self.tilt/(3.14159)
		#	print "Scan line",self.scanindex, "angle",angle,"at",self.timestamp, self.datacnt,"points"
		print "Scan line %3d  angle %6.2f  time %10.4f: %d points." % (self.scanindex,angle,self.timestamp, self.datacnt)


#	Test program
fname = 'lidarscans20041211_194820.log'
obj = LidarScanLine()
infile = open(fname,"rb")					# binary read
print "Dumping log file: ",fname
while True:
	stat = obj.readfile(infile)
	if stat < 0:
		break
	obj.dump()
#	obj.readfile(infile)
infile.close()
# obj.dump()
		
