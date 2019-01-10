//
//	Eaton VORAD interface program.
//
//	The Eaton VORAD radar unit operates on a serial port. This class
//	handles the details of processing messages from the radar.
//	It was developed on QNX, but contains no QNX specific code
//	and may work on other Posix-compatible UNIX variants.
//
//	John Nagle
//	Team Overbot
//	www.overbot.com
//	April, 2003
//
//	License: LGPL. No warranty.
//
#include "voradradar.h"
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <termios.h>
#include <errno.h>
#include <sys/time.h>
#include <assert.h>
//
//	The EATON VORAD radar unit, in normal operation, generates target messages
//	every 65536 microseconds. We expect to get such messages regularly. If
//	no message is received in 2 update times, a timeout is reported and the
//	unit is considered down. After "errorsBeforeReset" missed updates,
//	the reset sequence is sent to the VORAD unit. This usually brings the
//	unit back up.  The reset sequence will be sent every "errorsBeforeReset" 
//	errors (including timeouts), unless a valid target message intervenes.
//
//	At startup, the unit is considered down until a target message comes in.
//
//	Constants
//
const uint32_t errorsBeforeReset = 5;								// 5 errors and a reset is needed

//
//	Class VoradRadar
//
//
//	dump -- dump one SLIP frame.
//
void VoradRadar::dump(const uint8_t buf[], uint32_t len)
{
	printf("Message:%3d bytes: ",len);								// Dump raw message
	for (unsigned int i=0; i<len; i++)
	{	printf("%02x ",buf[i]);	}											// dump bytes 
	printf("\n");
}
//	
//	parsetargetreport  -- parse one "target report" message.
//
//	This contains a header followed by zero to seven VoradField items.
//
int VoradRadar::parsetargetreport(const uint8_t buf[], uint32_t len)
{	const uint32_t  headerlength= 3;										// 1 at start,  1 at end (???)
	if (len < headerlength)														// if undersized
	{	m_errors++;																	// error, reset counter
		return(EINVAL);																// too small
	}
	VoradTargetReport* rpt = (VoradTargetReport*)(buf);		// COERCION - force into TargetReport structure
	uint32_t calclen = headerlength + rpt->m_targetcount * sizeof(VoradField);	// expected size
	if (len != calclen)																// if size is wrong
	{	printf("Bad target report size.  Expected %d, got %d\n",calclen,len);		// report problem
		printf("  %d targets, %d byte report size\n",rpt->m_targetcount, sizeof(VoradField));	// ***TEMP***
		m_errors++;																	// bad, incr. error counter
		dump(buf,len);																// dump as bad
		return(EIO);
	}
	m_errors = 0;																	// good data, reset error counter
	delivertargetreport(rpt->m_sequence,rpt->m_targetcount, rpt->m_targets);
	return(EOK);
}
//
//	deliver -- one SLIP frame from serial line being delivered.
//
void VoradRadar::deliver(const uint8_t buf[], uint32_t len)
{
	////dump(buf,len);															// debug output
	if (len < 1) return;															// useless if no data
	switch (buf[0]) {															// fan out on VORAD message type
	case VoradTargetReportMsg_e:									// TargetReport message
		parsetargetreport(&buf[1],len-1);								// parse the target report
		break;
		
	case VoradStartupMsg_e:												// unit has started up successfully.  OK.
		deliverstartup();														// VORAD has restarted.
		break;																		// ignore
	
	case VoradCalibrationMsg_e:										// occasional calibration data
		break;																		// ignore
		
	default:																			// unknown message type
		m_errors++;																// this is an error
		delivererror(EIO,"Unexpected VORAD message");																		
		break;
	}
}

//
//	reset  -- send message to reset the VORAD unit
//
//	Per Robert Anderson, Eaton VORAD:
//	"The sequence is in hexadecimal:
//	\xfe\xfc
//	This is then followed by the message:
//	\x00\x20\xe0"
//
//	Returns POSIX error code.  Resets the error counter to 1, so that we
//	don't send resets continually. 
//
int VoradRadar::reset(int fd)
{	const uint8_t resetmsg1[] = { 0xfe, 0xfc };
	const uint8_t resetmsg2[] = { 0x00, 0x20, 0xe0 }; 
	int stat = write(fd,resetmsg1,sizeof(resetmsg1));		// send message 1
	if (stat < 0) { return(errno); }									// return error if any
	usleep(100000);														// wait 100 ms (???) for VORAD to reset.										
	stat = write(fd,resetmsg2,sizeof(resetmsg2));			// send message 2
	if (stat < 0) { return(errno); }									// return error if any
	m_errors = 1;															// reduce error counter to prevent reset loop
	return(EOK);
}
//
//	setupserial -- set up the serial port for the VORAD
//
int VoradRadar::setupserial(int fd)
{
	assert(fd >= 0);											// must be open
    struct termios options ;
    tcgetattr(fd,&options) ;									// get current options

    /* 19200 baud, raw mode */
	cfmakeraw(&options);
    cfsetispeed(&options,B19200) ;
    cfsetospeed(&options,B19200) ;
	int stat = tcsetattr(fd, TCSANOW ,&options);	// set modes
	if (stat) return(errno);									// return errno if fail
    return(EOK);
}
//
//	waitforinput  -- wait for input and handle it
//
//	Returns EOK normally.
//	Returns an error only if there's no hope of recovery.
//
int VoradRadar:: waitforinput(int fd)
{						
	if (m_errors > errorsBeforeReset)					// do reset if needed
	{	reset(fd);	}												// do reset if needed
	//	Wait for input, but only for two update times.
	const uint32_t maxvoradwait = 2*65536;	// max time to wait for VORAD unit, microseconds
	fd_set rfd;														// file descriptor set for select
	FD_ZERO( &rfd );
	FD_SET(fd, &rfd );
	struct timeval timeout;									// timeout 
	timeout.tv_sec = 0; timeout.tv_usec = maxvoradwait;
	int stat = select(fd+1,&rfd,0,0,&timeout);		// wait, but no longer than timeout
	if (stat ==0)													// if timeout
	{	m_errors++;												// tally error
		delivererror(ETIMEDOUT,"No response from VORAD");
		return(EOK); 												// report, but do not fail
	}
	//	Input is available - read it.
	const uint32_t bufsize = 100;                   	// larger than needed
	uint8_t buf[bufsize];                              		// data to read
	int cnt = read(fd,buf,sizeof(buf));					// read from serial port
	if (cnt < 0)                                       				// if fail
	{	return(errno);}											// returns failure
	for (int i=0; i<cnt; i++)
	{	accept(buf[i]);   }          					   	 	// put in chars
	return(EOK);													// success
}

//
//	Debug support
//
//
//	dump  --  dump field contents
//
void VoradField::dump() const
{	if (m_targetid)												// if list of targets
	{	printf("  Target %3d: ",m_targetid); m_targetinfo.dump();	}
	else																// if list of dropped targets
	{	m_targetsdropped.dump();	}
	////printf("\n");
}
//
//	dump --  dump for debug
//
void VoradTarget::dump() const
{	const bool rawdump = false;											// ***TEMP TEST***
	if (rawdump)
	{	printf("  Target: ");														// temporary dumb version
		for (int i=0; i<8; i++)
		{	printf(" %02x",((uint8_t*)(this))[i]);	}
		printf("\n");
	}
	printf("  %6.1f ft.  %6.1f fps   %6.3f radians  %6.2f dB  %02x lock\n",
		rangeft(), velocityftsec(), azimuthrad(),magnitudedb(),m_lock); 
}
//
//	dump  --  dump for debug
//
void VoradTargetDrop::dump() const
{	printf("  Dropped targets: ");
	for (unsigned int i=0; i<sizeof(m_targetdropped); i++)
	{	if (m_targetdropped[i])
		{	printf("#%d ",m_targetdropped[i]);	}
	}
	printf("\n");
}
//	
//	dump  --  dump for debug
//
void VoradTargetReport::dump() const
{
	if (m_targetcount == 0) return;										// skip empty entries
	printf("[%03d]  %2d targets: \n", m_sequence, m_targetcount);
	for (int i=0; i<m_targetcount; i++)
	{	m_targets[i].dump();	}												// dump the targets
}