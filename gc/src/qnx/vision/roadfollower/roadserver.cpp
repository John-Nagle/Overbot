//
//	roadserver.cpp  --  The road follower, as a QNX server
//
//	John Nagle
//	Team Overbot
//	November, 2003
//
#include <assert.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include "iplimagecameraread.h"
#include "roadserver.h"
#include "roadfollower.h"
#include "avformat.h"
#include "ffmpegwrite.h"
#include "avconvertimage.h"
#include "logprint.h"
#include "tuneable.h"

//
//	TODO:
//		1. Generate trapezoid of interest based on params. pitch, and roll.
//
//
//	Constants
//
const char* k_logprefix = "video";						// prefixed to log file

//
const int camimgwidth = 640;								// camera image width in pixels
const int camimgheight = 480;								// camera image height in pixels
const int imgwidth = 320;										// width in pixels
const int imgheight = 240;									// height in pixels
const int imgchannels = 3;									// number of color channels
const int imgdepth = 8;											// bits per pixel
const int imgfps = 7;												// framerate ***TEMP***
const int cammode = MODE_640x480_RGB;			// FireWire camera mode
const int camframerate = FRAMERATE_7_5;			// FireWire camera framerate
const int imgsize = imgwidth*imgheight*imgchannels;			// size of entire image
//
// tuneable parameters for roadfollower
const Tuneable k_threshold("ROADSCORETHRESHOLD", 0.0, 100.0, 3.0, "Above threshold means confident of seeing road");
const Tuneable k_llx("LLX", 0.0, 320.0, 1.0, "x-coord of lower left corner of trapezoid");
const Tuneable k_lly("LLY", 0.0, 240.0, 80.0, "y-coord of lower left corner of trapezoid");
const Tuneable k_lrx("LRX", 0.0, 320.0, 315.0, "x-coord of lower  right corner of trapezoid");
const Tuneable k_lry("LRY", 0.0, 240.0, 80.0, "y-coord of lower  right corner of trapezoid");
const Tuneable k_ulx("ULX", 0.0, 320.0, 110.0, "x-coord of upper left corner of trapezoid");
const Tuneable k_uly("ULY", 0.0, 240.0, 160.0, "y-coord of upper  left corner of trapezoid");
const Tuneable k_urx("URX", 0.0, 320.0, 230.0, "x-coord of upper  right corner of trapezoid");
const Tuneable k_ury("URY", 0.0, 240.0, 160.0, "y-coord of upper  right corner of trapezoid");

//
static bool verbose = false;									// true if verbose mode
static bool debug = false;										// true if verbose debug mode
//
//	Utility functions
//
//	gettimens  -- get current time in nanoseconds
//
static uint64_t gettimens()
{	timespec tp;
	clock_gettime(CLOCK_REALTIME, &tp);				// get time
	return(timespec2nsec(&tp));							// in nanoseconds
}
//
//	buildlogfilename  -- build name of log file
//
//	Builds a log file name of the form
//
//	logdir/PREFIXyymmdd_hhmmss.SUFFIX
//
static void buildlogfilename(char* timestr, size_t timestrl, const char* logdir,  const char* prefix, const char* suffix)
{	assert(timestrl > 0);
	timestr[0] = '\0';
	if (!logdir) return;											// no string if no dir
	assert(prefix);
	assert(suffix);
	assert(timestr);	
	time_t now;													// current time
	time(&now);													// get current time
	struct tm* nowtime = localtime(&now);		// get time components
	snprintf(timestr, timestrl,"%s/%s%04d%02d%02d_%02d%02d%02d.%s",
		logdir,prefix,
		nowtime->tm_year+1900, nowtime-> tm_mon+1, nowtime->tm_mday,
		nowtime->tm_hour, nowtime->tm_min, nowtime->tm_sec,
		suffix);
}
//	
//	avgpixels -- average four pixels
//
inline char avgpixels(char p1, char p2, char p3, char p4)
{	return((uint8_t(p1) + uint8_t(p2) + uint8_t(p3) + uint8_t(p4)) >> 2); }
//
//	halveimage -- cut size of image in half
//
//	Output must be half the size of the input.
//
//	Averages pixels, which gets rid of some noise.
//
static int halveimage(IplImage& outimage, const IplImage& inimage)
{
	if (outimage.width*2 != inimage.width) return(EINVAL);			// size check
	if (outimage.height*2 != inimage.height) return(EINVAL);			// size check
	if (inimage.nChannels != 3) return(EINVAL);								// bytes per pixel
	if (outimage.nChannels != 3) return(EINVAL);							// bytes per pixel
	if (inimage.depth != IPL_DEPTH_8U) return(EINVAL);					// bytes per pixel
	if (outimage.depth != IPL_DEPTH_8U) return(EINVAL);				// bytes per pixel
	//	Output image size is exactly half the input image size. Conversion can proceed.
	const int inrowsize = inimage.width*inimage.nChannels;
	const int outrowsize = outimage.width*3;
	//	If this program is slow, look here. Unless the optimizer does a good job on this,
	//	which it should, this is inefficient.
	for (int iout=0, iin=0; iout<outimage.height; iout++, iin += 2)	// for each output row
	{	
		for (int jout=0, jin=0; jout<outimage.width*3;  jout += 3, jin += 6)	// for each output column
		{	for (int chan = 0; chan<3; chan++)									// for each color
			{	outimage.imageData[iout*outrowsize+jout+chan] = 	// result pixel is sum of four input pixels / 4
				avgpixels(inimage.imageData[iin*inrowsize+jin+chan], 
					inimage.imageData[iin*inrowsize+jin+3+chan],
					inimage.imageData[(iin+1)*inrowsize+jin+chan],
					inimage.imageData[(iin+1)*inrowsize+jin+3+chan]);
			}
		}
	}
	return(EOK);																				// success
}
//
//
//	class CameraRoadFollower  --  read from a FireWire camera, do road following
//
class CameraRoadFollower: public IplImageCameraRead {
private:
	int m_fd;														// file descriptor of camera
	IplImage m_cvcamimage;								// CV image from camera
	IplImage m_cvimage;									// CV image to work on (smaller)
	RoadFollower m_roadfollower;						// the road follower
	MPEGwrite m_imagelog;								// output image log
	MPEGframe m_imageframe;							// output image frame
	int m_logfps;													// logging frame rate
	uint64_t m_nextframetime;							// don't log until this time, for log throttling
private:
	void setroadtrapezoid();								// set up the road trapezoid
	int processframe();										// process a frame
	void logframe();											// log frame if needed	
public:
	int reset();
	void serverPC(int rcvid, RoadServerMsgRDPC& msg);
	void serverDR(int rcvid, RoadServerMsgRDDR& msg);
	void serverTimeout();
	void setDebug(int n) { m_roadfollower.setDebug(n); }
	CameraRoadFollower();								// constructor
	~CameraRoadFollower();								// destructor
	int openlog(const char* logdir, const char* logformat, int logfps);		// open log file in log dir
private:
	int setup();													// initial setup
};
//
//	Constructor
//
CameraRoadFollower::CameraRoadFollower()
: m_fd(-1), m_nextframetime(0)
{	//	Create working openCV images
	CvSize camimgsize = {camimgwidth, camimgheight };		// simple struct
	cvInitImageHeader(&m_cvcamimage, camimgsize, imgdepth , imgchannels, IPL_ORIGIN_BL);
	cvCreateImageData(&m_cvcamimage);					// ***CHECK STATUS***
	CvSize imgsize = {imgwidth, imgheight };				// simple struct
	cvInitImageHeader(&m_cvimage, imgsize, imgdepth , imgchannels, IPL_ORIGIN_BL);
	cvCreateImageData(&m_cvimage);							// ***CHECK STATUS***
	setmode(cammode,camframerate);							// set initial camera mode. 
	//	Intitialize road follower
	setroadtrapezoid();
	m_roadfollower.setThreshold(k_threshold);					// set minimum acceptable road quality
}
//	
//	Destructor
//
CameraRoadFollower::~CameraRoadFollower()
{	close();
	cvReleaseImageData(&m_cvimage);				// release it
	cvReleaseImageData(&m_cvcamimage);			// release it
}
//
//	logframe -- log a frame of video
//
void CameraRoadFollower::logframe()
{	if (!m_imagelog.isopened()) return;									// if closed, ignore
	uint64_t now = gettimens();												// get time now in nanoseconds
	if (now < m_nextframetime) return;									// not yet time to log
#ifdef OBSOLETE
	{	uint64_t err = now - m_nextframetime;							// timing error
		double derr = err;
		derr *= (1e-9);
		printf("%f\n",derr);fflush(stdout);
	}
#endif // OBSOLETE
	m_nextframetime = now + (1000000000/m_logfps);		// set next log time
	int stat = avconvertimage(m_imageframe.getframe(),
		m_imageframe.getpixfmt(),
			m_imageframe.getwidth(),  m_imageframe.getheight(), &m_cvimage);	// convert image format
	if (stat)
	{	logprintf("LOG TERMINATED - error in image conversion.\n");
		m_imagelog.close();
		fflush(stdout);
		return;
	}
	stat = m_imagelog.write(*m_imageframe.getframe());// write the image
	if (stat)
	{	logprintf("LOG TERMINATED - error in image output.\n");
		m_imagelog.close();
		fflush(stdout);
	}
}
//
//	openlog  -- open video log file
//
int CameraRoadFollower::openlog(const char* logdir, const char* logformat, int logfps)
{	m_logfps = logfps;														// save frame rate
	char logfilename[512];											
	buildlogfilename(logfilename,sizeof(logfilename),logdir,k_logprefix, logformat);	// build log file name
	int stat = m_imagelog.open(logfilename, imgwidth, imgheight, logfps, logformat);
	if (stat)
	{	logprintf("NO LOGGING: Unable to start video log  \"%s\"\n", logfilename);	// report log file problem
		m_imagelog.close();													// close it
		return(stat);
	}
	//	Allocate frame buffer
	stat = m_imageframe.alloc(PIX_FMT_YUV420P, imgwidth, imgheight);
	if (stat)
	{	logprintf("NO LOGGING: Unable to allocate space for video log frame.\n");	// report log file problem
		m_imagelog.close();
		fflush(stdout);
		return(stat);
	}	
	logprintf("Logging video to \"%s\"\n", logfilename);			// log to this file name
	return(0);
}
//
//	processframe  -- process a frame
//
int CameraRoadFollower::processframe()
{
	int stat = readframe(m_cvcamimage);							// read a frame
	if (stat) return(stat);														// status
	stat = halveimage(m_cvimage,m_cvcamimage);			// halve the image size
	if (stat) return(stat);														// fails
	//	***MORE*** need to set road trapezoid based on roll and pitch info
	//	Process through road follower
	m_roadfollower.processFrame(&m_cvimage);				// process the frame
	//	Log the image
	logframe();
	return(0);																		// success
}
//	
//	serverTimeout  --  get picture from camera and log. Only if nobody is querying the road follower.
//
void CameraRoadFollower::serverTimeout()
{
	m_roadfollower.setDisplayResults(1);							// display visible debug info
	processframe();															// read a frame, log if necessary
}
//	
//	serverPC  --  get picture from camera
//
void CameraRoadFollower::serverPC(int rcvid, RoadServerMsgRDPC& msg)
{
	m_roadfollower.setDisplayResults(1);							// display visible debug info
	int stat = processframe();												// read a frame
	if (stat) 																		// if camera failed
	{	MsgError(rcvid,stat);													// reply with result code only
		return;																		// fails
	}	
	//	Ship annotated frame back as a message. The image is raw, with no headers.
	int cnt = m_cvimage.height * m_cvimage.width * m_cvimage.nChannels;	// size of image
	MsgReply(rcvid, cnt,m_cvimage.imageData,cnt);			// normal reply
}
//	
//	serverDR  --  get road direction
//
void CameraRoadFollower::serverDR(int rcvid, RoadServerMsgRDDR& msg)
{
	m_roadfollower.setDisplayResults(1);							// visible debug info
	int stat = processframe();												// read a frame
	if (stat) 																		// if camera failed
	{	MsgError(rcvid,stat);													// reply with result code only
		return;		
	}																					// fails
	//	Fill in results in message
	RoadServerMsgReply replymsg;
	replymsg.m_curvature = m_roadfollower.getAngle();
	replymsg.m_confidence = m_roadfollower.getScore();
	MsgReply(rcvid, replymsg);													// normal reply
}

//
//	setroadtrapezoid -- set up the trapezoid used for sampling
//
void CameraRoadFollower::setroadtrapezoid()
{	//	***TEMP*** canned values
	m_roadfollower.setUpperLeft(k_ulx,k_uly);
	m_roadfollower.setUpperRight(k_urx,k_ury);
	m_roadfollower.setLowerLeft(k_llx,k_lly);
	m_roadfollower.setLowerRight(k_lrx,k_lry);
	m_roadfollower.setDisplayResults(1);							// display debug info
}
//
//	runserver -- run as a server
//
//	This server accepts request messages, does some computation, and returns the result.
//
void runserver(const char* cameraname, const char* logdir, const char* logformat, int logfps)															
{	CameraRoadFollower follower;									// the road follower
	if (debug) follower.setDebug(1);								// set debug mode if needed
	//	Initialize camera
	logprintf("Opening camera\"%s\"\n",cameraname);		// Log camera opened
	fflush(stdout);
	int stat = follower.open(cameraname);						// try to open camera
	if (stat != EOK)															// if unsuccessful open
	{																				// pop up a standard dialog, centered over the open dialog
		perror("Unable to open camera");
		exit(1);																	// fails
	}
	//	Set up video logfile if requested
	if (logdir)
	{
		follower.openlog(logdir, logformat, logfps);			// open log dir, ignore status
	}
	//	The server normally has the name given the program by the watchdog file. This is in the env. variable "ID"
	MsgServerPort serverport(0.2);								// define message port, timeout every 200ms.
	if (verbose) serverport.setverbose();						// more talkative
	stat = serverport.ChannelCreate();							// create a channel, tell watchdog about it
	if (stat) 
	{	perror("ChannelCreate failed in server"); 
		serverport.Dump();
		exit(1);																	// fails
	}
	fflush(stdout);															// force out any startup messages
	for (;;)																		// forever
	{	logprintf("\n");														// periodically log a blank line to keep the watchdog happy
		RoadServerMsg msgin;											// area for incoming msg
		_msg_info msginfo;												// aux info
		int rcvid = serverport.MsgReceive(msgin,&msginfo);	// get msg
		if (rcvid < 0)															// if error
		{	if (errno == ETIMEDOUT)									// if timeout
			{	follower.serverTimeout();								// handle timeout
				continue;
			}
			fflush(stdout); perror("MsgReceive failed in server");				// fails
			sleep(1);															// avoid tight loop if repeated trouble
			continue;															// fails
		}
		if (rcvid == 0)														// pulse
		{	logprintf("Server received a pulse.\n");					// pulses don't require a reply
			continue;
		}
		//	We have received a message
		switch (msgin.m_rdpc.m_msgtype) {					// fan out on type
		case RoadServerMsgRDPC::k_msgtype:				// request for picture
		{	if (msginfo.msglen != sizeof(msgin.m_rdpc)) { MsgError(rcvid,EBADRPC); break; }	// msg size check
			follower.serverPC(rcvid,msgin.m_rdpc);			// handle msg
			break;
		}

		case RoadServerMsgRDDR::k_msgtype:				// request for road direction
		{	if (msginfo.msglen != sizeof(msgin.m_rddr)) { MsgError(rcvid,EBADRPC); break; }	// msg size check
			follower.serverDR(rcvid,msgin.m_rddr);			// handle msg
			break;
		}
			
		default:																	// unknown, fails
			MsgError(rcvid,EBADRPC);									// reply with result code only
			break;
		}
		////serverport.watchdogreset();								// tell the watchdog we are still alive
	}
}
//
//	runclient -- run as a client
//
//	usage  -- print usage and exit
//
static void usage()
{	printf("Usage: roadserver [options] cameraname [log directory]\n");
	printf("  Options:  -v              verbose\n");
	printf("  			   -d    	        debug\n");
	printf("                 -f format    log format  (default MPEG-1)\n");
	printf("					-r n			logging framerate (default 1 FPS)\n");
	exit(1);																		// fails
}
//
//	Main program
//
//	Usage: roadserver [options] cameraname [logging dir]
//
int main(int argc, const char* argv[])
{	const char* cameraname = 0;									// no camera name yet
	const char* logdir = 0;												// logfiles into this directory
	const char* logformat = "mpeg";								// assume MPEG output
	int logfps = imgfps;													// log every frame
	//	Parse input arguments
	for (int i=1; i<argc; i++)											// for all args
	{	const char* arg= argv[i];										// this arg
		if (arg[0] == '-')													// if flag argument
		{	switch(arg[1])	{												// interpret flags
			case 'v': verbose = true;	 break;						// set verbose mode
			case 'd': debug = true; break;							// set verbose debug mode
			case 'f':																// -f format
			{	i++;
				if (i >= argc) usage();
				logformat = argv[i];
				break;
			}
			case 'r':																// -r framerate for video
			{	i++;
				if (i >= argc) usage();
				logfps = atoi(argv[i]);
				if (logfps < 1 || logfps > imgfps) usage();		// sanity check
				break;
			}
			default: usage();												// bad call, fails
			}
			continue;															// next arg
		}
		//	Not flag, must be file arg
		if (!cameraname)													// first file arg is camera name
		{	cameraname = arg; continue; }
		if (!logdir)
		{	logdir = arg; continue; }									// second file arg is log dir
		usage();
	}
	if (!cameraname) usage();										// must have camera name
	runserver(cameraname, logdir, logformat, logfps);				// run road follower server
	return(0);																	// success
}
