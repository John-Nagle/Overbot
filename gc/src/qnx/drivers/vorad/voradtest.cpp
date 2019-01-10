//
//	Eaton VORAD test program
//
//	John Nagle
//	Team Overbot
//	April, 2003
//
#include "voradradar.h"
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <termios.h>
#include <errno.h>
#include <string.h>
//
//	setbaudrate  --  set serial port baud rate, and set to raw mode
//
//	This is paranoid, because we've had cases where the speed didn't change.
//
//	Returns POSIX error code.
static int setbaudrate(int fd, uint32_t baud)
{
	//	Set serial port modes
	struct termios ttymodes;
	int stat = tcgetattr(fd,&ttymodes);					// get modes before change
	if (stat) {	perror("Cannot get input modes"); return(errno); }
	//	Set raw mode and speed.
    ttymodes.c_cc[VMIN]  =  1;							// MIN value???
    ttymodes.c_cc[VTIME] =  0;							// Timeout value???  Check this
    ttymodes.c_lflag &= ~( ECHO|ICANON|ISIG|ECHOE|ECHOK|ECHONL );
    ttymodes.c_oflag &= ~( OPOST );
    stat = cfsetispeed(&ttymodes,baud);			// set speed in structure
    if (stat) {	perror("Invalid serial port input speed value"); return(errno); }
    stat = cfsetospeed(&ttymodes,baud);			// set speed in structure
    if (stat) {	perror("Invalid serial port output speed value"); return(errno); }
	stat = tcsetattr(fd,TCSANOW,&ttymodes);
	if (stat) {	perror("Cannot set serial port input modes"); return(errno); }
	stat = tcgetattr(fd,&ttymodes);						// get after set, to see what really happened.
	if (stat) 
	{	perror("Cannot get input modes"); return(errno); }
   	uint32_t actualispeed = cfgetispeed( &ttymodes);
    if (actualispeed != baud)
    {	printf("Serial port input speed change did not take place. Expected %u baud, got %u baud.\n",
    			baud,actualispeed); return(EINVAL);
    	}
    uint32_t actualospeed = cfgetispeed( &ttymodes);
    if (actualospeed != baud)
    {	printf("Serial port output speed change did not take place. Expected %u baud, got %u baud.\n",
    			baud,actualospeed); return(EINVAL);
    	}
    	return(EOK);													// success
}
//
//	class VoradRadarTest  -- test class, one per radar device
//
class VoradRadarTest: public VoradRadar {
protected:																			// override these to get target data
	void delivertargetreport(uint8_t sequence, uint8_t fieldcount,VoradField fields[]);
	void delivererror(int stat, const char* msg);
	void deliverstartup(); 													// unit has reset - clear target info
private:
	void delivertargetupdate(uint8_t sequence, uint8_t targetid, const VoradTarget& targetdata);
	void delivertargetdrop(uint8_t sequence, uint8_t targetid);
};
//
//	delivertargetreport --  deliver a target report with up to 7 fields.
//
//	Note that it is rare, but possible, to get more than one of these per sequence number.
//	The "high priority targets" are supposed to be in the first one.
//
void VoradRadarTest::delivertargetreport(uint8_t sequence, uint8_t fieldcount,VoradField fields[])
{
	for (int i=0; i<fieldcount; i++)												// for all fields
	{	const VoradField& field = fields[i];									// this field
		if (field.m_targetid)															// if target update
		{	delivertargetupdate(sequence,field.m_targetid,field.m_targetinfo);	}
		else																					// if list of dropped targets
		{	for (int i=0; i<7; i++)													// for possible dropped targets.
			{	uint8_t targetid = field.m_targetsdropped.m_targetdropped[i];			// get target dropped, or 0
				if (targetid)
				{	delivertargetdrop(sequence,targetid);	}				// if nonzero, deliver
			}
		}
	}
}

//
//	delivertargetupdate  -- deliver target update data to user
//
void VoradRadarTest::delivertargetupdate(uint8_t sequence, uint8_t targetid, const VoradTarget& targetdata)
{
	printf("[%3d] #%3d: ",sequence, targetid); targetdata.dump();								// just dump
}
//	
//	delivertargetdrop  --  deliver target drop data to user
//
void VoradRadarTest::delivertargetdrop(uint8_t sequence, uint8_t targetid)
{
	printf("[%3d] #%3d: Dropped.\n",sequence,targetid);	// just dumps
}
//
//	delivererror -- deliver error message
//
void VoradRadarTest::delivererror(int stat, const char* msg)
{	printf(" VORAD error: %s: %s\n",msg,strerror(stat));	// edit msg
}
//
//	deliverstartup  -- unit has restarted
//
void VoradRadarTest::deliverstartup()
{	printf(" VORAD restart.\n");
}
static VoradRadarTest vorad1;                                        // input device
//
//    readloop  --  input loop
//
static int readloop(int fd)                                     	// read from file descriptor
{    
       for (;;)
       {   	int stat = vorad1.waitforinput(fd);					// read
			if (stat)
			{	printf("VORAD serial port read error: %s\n",strerror(stat)); 
				return(1);														// fails
			}
       }
}

//
//	Main program
//
int main(int argc, char* argv[])
{	const char* filename = 0;
	for (int i=1; i<argc; i++)
	{	const char* arg = argv[i];									// this arg
		if (arg[0] == '-')													// if flag arg
		{	switch (arg[1]) {												// fan out on flag arg
			default:
				printf("Unknown argument: %s\n",arg);			// arg
				return(1);														// fails
			}
		}
		filename = arg;														// get filename
	}
	//	Flags read, check state.
	if (filename == 0)
	{	printf("No filename specified for input.\n");
		return(1);
	}
	//	We have a filename
	printf("Reading Vorad data from %s using SLIP.\n",filename);
	int fd = open(filename,O_RDWR);							// open for reading
	if (fd < 0)
	{	perror("Cannot open input device"); return(1); }
	//	Set serial port modes
	const long int ispeed = 19200;						// desired baud rate
	int stat = setbaudrate(fd,ispeed);					// set the baud rate
	if (stat) {	printf("Cannot set serial port modes"); return(1); }
	//	Reset the VORAD unit
	stat = vorad1.reset(fd);											// reset the thing
	if (stat) { printf("Cannot send reset to VORAD.\n"); return(1);	}											// if send fails
	readloop(fd);
	close(fd);
	return(0);
}