//
//	Eaton VORAD radar interface
//
//	John Nagle
//	Team Overbot
//	www.overbot.com
//	April, 2003
//
//	License: LGPL. No warranty.
//
#ifndef VORADRADAR_H
#define VORADRADAR_H
#include "sliprcv.h"
//
//	User quick start instructions:
//
//	Subclass VoradRadar, and define the "deliver" functions. 
//	Open the serial port, set it to raw mode and 19200 baud.
//	Call "waitforinput" repeatedly, at least once per 65ms.
//	See "voradtest.cpp" for an example program.
//	
//	VORAD constants
//
//
//	Conversions
//
inline float fttometers(float x) { return(x*0.3048); }		// metric conversion
////inline float degtorad(float x) { return(x*(3.1416926/180)); } // radian conversion
//
//	
//	VORAD message types - first byte of each message from the VORAD radar.
//
enum VoradMessage_e {VoradStartupMsg_e = 81, VoradTargetReportMsg_e = 82, VoradCalibrationMsg_e = 85 };
//
//	VoradTarget  -- one per target found
//
struct VoradTarget {
	uint8_t		m_range[2];									// 	Range			3-4	int	LSB = 0.1 ft
	uint8_t		m_velocity[2];								//	Relative Velocity	5-6	int	LSB = 0.1 ft/sec
	int8_t		m_azimuth;									// Azimuth			7	char	LSB = 0.002 radians 
	uint8_t		m_magnitude;								// Magnitude		8	uchar	LSB = -0.543 dB
	uint8_t		m_lock;											// Lock			9	bmb	bit mapped M-of-N reporting
public:
	uint16_t range() const { return((m_range[1] << 8) | m_range[0]);} // Vorad units, as unsigned
	float rangeft() const { return(range()*0.1);	}			// range in feet
	float rangem() const { return(fttometers(rangeft())); } // range in meters
	int16_t velocity() const { return((m_velocity[1] << 8) | m_velocity[0]);} // Vorad units, as signed (>0 if receding)
	float velocityftsec() const { return(velocity()*0.1);	} // velocity in feet per second.
	float velocitymps() const { return(fttometers(velocityftsec()));	} // velocity in meters per second
	float azimuthrad() const { return(m_azimuth*0.002);}	// azimuth in radians
	float magnitudedb() const { return(m_magnitude*(-0.543)); } // magnitude in dB
	void dump() const;										// dumps this data for debug
};
//
//	VoradTargetDrop  --  indicates targets dropped on this cycle.
//
struct VoradTargetDrop {
	uint8_t		m_targetdropped[7];						// targets dropped
public:
	void dump() const;										// dumps this data for debug
};

//
//	VoradField  -- packed into messages
//
struct VoradField {
	uint8_t		m_targetid;									// Target ID Number	2	uchar	Target ID number 1-255
																		//	(Target ID of 0 is not allowed, and indicates a target drop)	
	union {															// one of two formats			
		VoradTarget	m_targetinfo;							// this target's info
		VoradTargetDrop m_targetsdropped;		// list of dropped targets
	};
	void dump() const;										// dumps this data for debug
};
//
//	VoradTargetReport  --  data structure returned for a VORAD target report
//
//	Header, plus some number of target messages
//
struct VoradTargetReport {
	uint8_t		m_sequence;												// message sequence, cycles
	uint8_t		m_targetcount;												// number of targets
	VoradField m_targets[7];												// up to 7 targets
public:
	void dump() const;
};
//
//	class VoradRadar  -- one per radar device
//
class VoradRadar: public SlipRecv {
private:
	uint32_t m_errors;															// too many, and a reset is needed
protected:
	void deliver(const uint8_t buf[], uint32_t len);				// SLIP delivers input messages by calling this
public:
	VoradRadar() 																// needs a reset at startup
	: m_errors(1) {}															// no errors yet, but not up
	virtual ~VoradRadar() {}												// for subclass
	int reset(int fd);																// reset device
	int waitforinput(int fd);													// wait for event and handle it.
	bool isup() const { return(m_errors == 0); }				// working if error count is 0.
	int setupserial(int fd);													// set serial port modes
private:
	int parsetargetreport(const uint8_t buf[], uint32_t len);
	int parsetargetfield(uint8_t sequence, const VoradField& field);
	void dump(const uint8_t buf[], uint32_t len);					// print for debug
protected:																			// override these to get target data
	virtual void delivererror(int stat, const char* msg) {};	// deliver error message
	virtual void deliverstartup() {} 									// unit has reset - clear target info
	virtual void delivertargetreport(uint8_t sequence, uint8_t fieldcount,VoradField fields[]) = 0;	// must override
};

#endif // VORADRADAR_H