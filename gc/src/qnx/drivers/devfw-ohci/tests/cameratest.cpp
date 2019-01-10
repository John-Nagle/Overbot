//
//	Cameratest.cpp  --  tester for QNX camera support
//
//	John Nagle
//	Animats
//	January, 2003
//
#include <stdio.h>
#include <stdlib.h>
#include <devctl.h>
#include <inttypes.h>
#include <fcntl.h>
#include <string>
#include <vector>
#include <errno.h>
#include <string>
#include "devfw-camera.h"
#include "../dc1394constants.h"							// debug only, for raw test interface
//
//	Local data
static bool verbose = false;									// usual verbose flag
static bool capture = false;									// capturing frames
static bool printfeat = false;									// printing features
static bool singleshot = false;								// using single shot mode?
static int framestotake = 1;									// take one frame
static char* filename = 0;										// no filename yet
static vector<string> cameranames;					// camera names
//	Forward declarations
static void testcamera(const string& s);				// try this camera
static void printfeatures(int fd);								// print feature set for camera
int GetCameraControlRegister(int fd, uint32_t offset, uint32_t&  value);
int SetCameraControlRegister(int fd, uint32_t offset, uint32_t value);
int dc1394_camera_on(int fd);
int dc1394_camera_off(int fd);
void dc1394_camera_power_print(int fd);
int dc1394_get_camera_ident(int fd, uint64_t& euid, string& vendor, string& model);
int testcapture(int fd, const char* filename);
void dc1394_print_feature(const dc1394_feature_info& f);


//
//	Main program
//
int main(int argc, char* argv[])
{
	for (int i=1; i<argc; i++)									// for all args
	{	const char* arg = argv[i];							// this arg
		if (arg[0] == '-')											// if flag
		{	switch (arg[1])	{										// fan out on arg type
			case 'v':													// verbose
				verbose = true; break;							// set flag
			case 'c':													// start capturing
				capture = true;
				if (i < argc-1 && argv[i+1][0] == '-')	// if no filename after -c
				{	break;	}											// just sink output 
				i++;
				if (i >= argc) break;
				filename = argv[i];								// set output file
				break;
			case 'f':														// print features
				printfeat = true; break;
			case 's':
				singleshot = true; break;
			case 'n':													// number of frames to take
				i++;
				if (i >= argc) { printf("Usage: requires number after -n\n"); exit(1); }
				framestotake = atoi(argv[i]);
				break;
			default:
				printf("Usage: %s [-v] [-c filename] [-n framecount] [-s] [-f] cameranames\n",argv[0]);
				exit(1);
			}
			continue;													// not flag, is file
		}
		string s(arg);													// get this file arg
		cameranames.push_back(s);						// add to list
	}
	//	Input complete; try cameras
	for (vector<string>::iterator p = cameranames.begin(); p != cameranames.end(); p++)
	{	testcamera(*p);	}										// try this camera
}
//
//	testcamera  -- try a camera
//
static void testcamera(const string& s)				// try this camera
{	printf("Testing camera \"%s\":\n",s.data());		// print camera name
	int fd = open(s.data(),O_RDWR);						// open camera
	if (fd < 0)															// if open failed
	{	perror("Unable to open camera");				// explain
		return;															// fails
	}
	if (printfeat) printfeatures(fd);							// print features of this camera
	string vendor, model;
	uint64_t euid;
	int stat = dc1394_get_camera_ident(fd,euid,vendor,model);		// get info
	if (stat)
	{	errno = stat; perror("Unable to get camera identity");	}
	else
	{	printf("Camera UID %0llx  Vendor \"%s\", model \"%s\"\n",euid, vendor.c_str(), model.c_str());
	}
	//	Set camera mode.
	dc1394_video_mode_t mode;
	mode.videomode = MODE_640x480_RGB;			// set some reasonable modes
	mode.framerate = FRAMERATE_15;						
	stat = devctl(fd,DCMD_MEDIA_DEVFW_CAMERA_VIDEO_MODE_WRITE,&mode,sizeof(mode),0);
	if (stat != EOK)
	{	errno = stat; perror("Unable to set camera mode");
		return;
	}
	printf("Set video mode %d, framerate %d:  (%d x %d - %d bytes per frame)\n",
		mode.videomode,mode.framerate,mode.width,mode.height, mode.bytesperframe);
	//	Set camera to auto-exposure
	const uint32_t FEATURE_MODE_AUTO = (1<<31)>>7;	// bit 7 from MSB enables auto
	dc1394_feature_info feature;										// feature info
	feature.feature_id = FEATURE_EXPOSURE;
	feature.value = FEATURE_MODE_AUTO;							// set auto mode
	stat = devctl(fd,DCMD_MEDIA_DEVFW_CAMERA_FEATURE_WRITE,&feature,sizeof(feature),0);	// set mode
	if (stat != EOK)
	{	errno = stat; perror("Unable to set feature to auto."); 
		return;
	}
	if (capture)															// if capturing requested
	{	////sleep(3);														// allow camera time to auto-iris
		for (int i=0; i<framestotake; i++)						// for indicated number of frames
		{	if (filename)
			{	char sname[255];													// working filename
				snprintf(sname,sizeof(sname),"%s%04d.raw",filename,i);		// build filename
				printf("Capturing into %s\n",sname);						// where output is going
				stat = testcapture(fd,sname);								// capture a frame
			} else {
				stat = testcapture(fd,0);								// just sink a frame
			}
			if (stat)
			{	errno = stat; perror("Start test capture failed");  }
		}
	}
	close(fd);															// done with this camera
}
//
//	printbasicfeatures  -- get basic features register and print
//
static void printbasicfeatures(int fd)
{	unsigned int basicfeatreg;										// basic features register
	//	Values in basic features register
	const unsigned int hibit = 0x80000000;					// high bit
	const unsigned int Cam_Power_Cntl = hibit >>16;	// Camera process power ON/OFF capability
	const unsigned int One_Shot_Inq = hibit >> 19;		// One shot transmission capability
	const unsigned int Multi_Shot_Inq = hibit >> 20;		//  Multi shot transmission capability



	int stat = GetCameraControlRegister(fd,
                                         REG_CAMERA_BASIC_FUNC_INQ, basicfeatreg);
    if (stat != EOK)							// if fail
    {	perror("ERROR: can't get basic features register.\n");
    		return;
    	}
    	printf("Basic features register: %08x\n",basicfeatreg);
    	printf("  Can turn power on/off:  %s\n", basicfeatreg & Cam_Power_Cntl ? "YES" : "NO");
    	printf("  Can do one-shot mode:   %s\n", basicfeatreg & One_Shot_Inq ? "YES" : "NO");
    	printf("  Can do multi-shot mode: %s\n", basicfeatreg & Multi_Shot_Inq ? "YES" : "NO");
}
//
//	printfeatures  -- print camera features
//
static void printfeatures(int fd)
{	printbasicfeatures(fd);										// print the basic features register
	const unsigned int readfn = DCMD_MEDIA_DEVFW_CAMERA_FEATURE_READ;	// read a camera feature
	printf("Testing devctl function %02x.\n",readfn);	// 
    for (int i= FEATURE_MIN; i <= FEATURE_MAX; i++) 
    {	dc1394_feature_info feature;						// current feature
    		feature.feature_id = i;									// set desired feature ID
    		int stat = devctl(fd,readfn,&feature,sizeof(feature),0);	// get feature
    		if (stat)
    		{	errno = stat;												// status in return, not errno
    			perror("Unable to read camera feature");	// trouble
    			return;
    		}
        dc1394_print_feature(feature);					// print feature
    }
}
const char *dc1394_feature_desc[NUM_FEATURES] =
{
    "Brightness",
    "Exposure",
    "Sharpness",
    "White Balance",
    "Hue",
    "Saturation",
    "Gamma",
    "Shutter",
    "Gain",
    "Iris",
    "Focus",
    "Temperature",
    "Trigger",
    "Zoom",
    "Pan",
    "Tilt",
    "Optical Filter",
    "Capture Size",
    "Capture Quality"
};
/*****************************************************
 dc1394_print_feature

 Displays the bounds and options of the given feature
*****************************************************/
void
dc1394_print_feature(const dc1394_feature_info& f) 
{
    int fid= f.feature_id;

    if ( (fid < FEATURE_MIN) || (fid > FEATURE_MAX) )
    {
        return;
    }

    printf("%s:\n\t", dc1394_feature_desc[fid - FEATURE_MIN]);

    if (!f.available) 
    {
        printf("NOT AVAILABLE\n");
        return;
    }

    if (f.one_push) 
        printf("OP  ");
    if (f.readout_capable)
        printf("RC  ");
    if (f.on_off_capable)
        printf("O/OC  ");
    if (f.auto_capable)
        printf("AC  ");
    if (f.manual_capable)
        printf("MC  ");
    if (f.absolute_capable)
        printf("ABS  ");
    printf("\n");

    if (f.on_off_capable) 
    {
        if (f.is_on) 
            printf("\tFeature: ON  ");
        else
            printf("\tFeature: OFF  ");
    }
    else 
    {
        printf("\t");
    }

    if (f.one_push) 
    {
        if (f.one_push_active)
            printf("One push: ACTIVE  ");
        else
            printf("One push: INACTIVE  ");
    }

    if (f.auto_active) 
        printf("AUTO  ");
    else
        printf("MANUAL ");

    if (fid != FEATURE_TRIGGER) 
    {
        printf("min: %d max %d\n", f.min, f.max);
    }

    if (fid == FEATURE_TRIGGER)
    {
        printf("\n\tAvailableTriggerModes: ");

        if (f.trigger_mode_capable_mask & 0x08)
            printf("0 ");
        if (f.trigger_mode_capable_mask & 0x04)
            printf("1 ");
        if (f.trigger_mode_capable_mask & 0x02)
            printf("2 ");
        if (f.trigger_mode_capable_mask & 0x02)
            printf("3 ");
        if (!(f.trigger_mode_capable_mask & 0x0f))
            printf("No modes available");

        printf("\n\tPolarity Change Capable: ");

        if (f.polarity_capable) 
            printf("True");
        else 
            printf("False");

        printf("\n\tCurrent Polarity: ");

        if (f.trigger_polarity) 
            printf("POS");
        else 
            printf("NEG");

        printf("\n\tcurrent mode: %d\n", f.trigger_mode);
    }
    else if (fid == FEATURE_WHITE_BALANCE) 
    {
        printf("\tB/U value: %d R/V value: %d\n", f.BU_value, f.RV_value);
    }
    else if (fid == FEATURE_TEMPERATURE) 
    {
        printf("\tTarget temp: %d Current Temp: %d\n", f.target_value,
               f.value);
    }
    else 
    {
        printf("\tcurrent value is: %d\n",f.value);
    }
    if (f.absolute_capable)
      printf("\tabsolute settings:\n\t value: %f\n\t min: %f\n\t max: %f\n",
	     f.abs_value,f.abs_min,f.abs_max);
}
//
//	GetCameraControlRegister -- get register, as in dc1394
//
int GetCameraControlRegister(int fd, uint32_t offset, uint32_t&  value)
{	dc1394_devctl_quad_t	quad;
	quad.offset = offset;
	int stat = devctl(fd,DCMD_MEDIA_DEVFW_CCR_READ,&quad,sizeof(quad),0);
	value = quad.value;
	return(stat);
}
//
//	SetCameraControlRegister -- set register, as in dc1394
//	
int SetCameraControlRegister(int fd, uint32_t offset, uint32_t value)
{	dc1394_devctl_quad_t	quad;
	quad.offset = offset;
	quad.value = value;
	return(devctl(fd,DCMD_MEDIA_DEVFW_CCR_WRITE,&quad,sizeof(quad),0));
}
//
//	GetCameraROMValue -- get register, as in dc1394
//
int GetCameraROMValue(int fd, uint32_t offset, uint32_t&  value)
{	dc1394_devctl_quad_t	quad;
	quad.offset = offset;
	int stat = devctl(fd,DCMD_MEDIA_DEVFW_ROM_READ,&quad,sizeof(quad),0);
	value = quad.value;
	return(stat);
}

//
//	dc1394_camera_on  --  turn camera on
//
int dc1394_camera_on(int fd)
{
  	return(SetCameraControlRegister(fd, REG_CAMERA_POWER,ON_VALUE));
}

int dc1394_camera_off(int fd)
{
    return(SetCameraControlRegister(fd, REG_CAMERA_POWER, OFF_VALUE));
}

void  dc1394_camera_power_print(int fd)
{	uint32_t value;
	int stat = GetCameraControlRegister(fd, REG_CAMERA_POWER, value);
	if (stat)
	{	errno = stat; perror("Unable to get camera power register");
		return;
	}
	printf("Camera power register: %08x\n",value);
}
//
//	appendstringfromrom  --  get a text string from device ROM
//
//	Assumes string is ASCII, not UNICODE, although the IEEE-1394 spec allows UNICODE device names.
//
//	Appends to string.
//
int appendstringfromrom(int fd, uint32_t offset, int len, string& s)
{
    while (len > 0)															// until all quads read
    {	uint32_t quad;														// quadlet, in host order
    		int stat = GetCameraROMValue(fd, offset, quad);	// get 4 chars from ROM
        if (stat != EOK) return(stat);									// if fail
		for (int i=0; i<4; i++)											// for bytes of quad
		{	char ch = quad >> 24;										// get next char
			if (ch == '\0') return(EOK);								// stop at null
			s += ch;															// append to string
			quad <<= 8;													// get next byte
		}
		offset+= 4;															// advance to next quad
        len-= 4;																// use up that quad
    }
    return(EOK);
}
//
//	dc1394_get_camera_ident  -- get camera indentity info
//
int dc1394_get_camera_ident(int fd, uint64_t& euid, string& vendor, string& model)
{
    int len;
    uint32_t offset, offset2;
    uint32_t value[2];
    unsigned int count;
	//	Hard-coded constants below are from dc1394,
    //	Get the 64-bit EUID 
    int stat;
    if ((stat = GetCameraROMValue(fd, ROM_BUS_INFO_BLOCK+0x0C, value[0]))) return(stat);
    if ((stat = GetCameraROMValue(fd, ROM_BUS_INFO_BLOCK+0x10, value[1]))) return(stat);
	euid = ((uint64_t)value[0] << 32) | (uint64_t)value[1];	// combine into one octlet

    //	Get the unit directory offset
    if ((stat = GetCameraROMValue(fd, (ROM_ROOT_DIRECTORY + 16), offset))) return(stat);
    offset<<= 2;																	// offset is in units of quads
    offset&= 0x00ffffff;
    offset+= (ROM_ROOT_DIRECTORY + 16);

    //	Get the unit-dependent directory offset.
    if ((stat = GetCameraROMValue(fd, offset+12, offset2)))  return(stat);
    offset2<<= 2;																// offset is in units of quads
    offset2+= (offset + 12);
    offset2&= 0x00ffffff;

    //	Get the base address of the command registers
    if ((stat = GetCameraROMValue(fd, offset2+4, offset)))  return(stat);
    offset<<= 2;																	// offset is in units of quads
    offset&= 0x00ffffff;

    //	Get the vendor name leaf offset
    if ((stat = GetCameraROMValue(fd, offset2+8, offset))) return(stat);
    offset<<= 2;																	// offset is in units of quads
    offset&= 0x00ffffff;
    offset+= (offset2 + 8);

    //	Get the length of the vendor name
    if ((stat = GetCameraROMValue(fd, offset, value[0]))) return(stat);
    len= (int)((value[0] >> 16) & 0xFFFFUL)*4-8;				// was in quads, not including length
    offset+= 12;																	// advance to name
    if ((stat = appendstringfromrom(fd,offset,len,vendor))) return(stat);	// get vendor name
    //	Get the model name leaf offset
    if ((stat = GetCameraROMValue(fd, offset2+12, offset))) return(stat);
    offset<<= 2;																// units are quads
    offset &=0x00ffffff;
    offset+= (offset2 + 12);

    //	Get length of model name
    if ((stat = GetCameraROMValue(fd, offset, value[0]))) return(stat);
    len= (int)((value[0] >> 16) & 0xFFFFUL)*4-8;
    offset+= 12;
    count= 0;
    if ((stat = appendstringfromrom(fd,offset,len,model))) return(stat);	// get model name
    return(EOK);
}
int
dc1394_set_iso_channel_and_speed(int fd, 
                                 unsigned int channel, unsigned int speed)
{
    return(SetCameraControlRegister(fd, REG_CAMERA_ISO_DATA,
                                         (uint32_t)
                                         ( ((channel & 0xFUL) << 28) |
                                           ((speed & 0x3UL) << 24) )));
}

int
dc1394_start_iso_transmission(int fd)
{
   	return(SetCameraControlRegister(fd, REG_CAMERA_ISO_EN,
                                         ON_VALUE));
 }
//
//	testcapture  -- capture some data from the camera
//
//	Doesn't do anything with it yet.
//
int testcapture(int fd, const char* filename) 
{
	const size_t bufsize = 8192;
	////for (int i=0; i<3; i++)												// ***TEMP TEST*** take many pictures 
	if (singleshot)															// if single shot mode
	{	const unsigned int takefn = DCMD_MEDIA_DEVFW_CAMERA_TAKEPICTURE;	// take a picture
		printf("Takepicture.\n");												// ***TEMP***
		int ret = devctl(fd,takefn,0,0,0);									// take a picture now, default mode
		if (ret != EOK)
		{	printf("Take picture returned %d\n",ret);				// failed to take picture
			errno = ret;
			perror("Error taking picture");
			return(errno);
		}
	}
	////usleep(1000000/15*2);											// wait two frame times ***TEMP***
	int bytesread = 0;
	int outfd = -1;															// no filename
	if (filename)																// if has filename
	{	outfd = creat(filename,0644);								// open for writing
		if (outfd < 0)
		{	perror("Cannot open output file for writing.\n");	// output to file
			return(errno);
		}
	}
	for (;;)																		// read a frame
	{	char buf[bufsize];													// sink	
		int ret = read(fd,buf,sizeof(buf));							// read bytes
		if (ret == 0) break;												// EOF
		if (ret > 0)
		{	bytesread += ret;	 
			if (outfd >= 0)
			{	int wret = write(outfd,buf,ret);									// write to latest picture file
				if (wret < 0)
				{	perror("Error writing output file\n");
					close(outfd);
					return(errno);
				}
			}
			continue;
		}
		perror("Error reading from camera");
		if (outfd >= 0) close(outfd);
		return(errno);														// fails
	}
	if (outfd >= 0) close(outfd);										// close file
	printf("Picture: %d bytes.\n",bytesread);					// report success
	return(EOK);
}