//
//	Fault codes for the Overbot
//
//	J.	Nagle
//	October, 2003
//
//	Feel free to add codes.
//	Add a message if you do.
//
#ifndef FAULTCODES
#define FAULTCODES

//
//	Fault -- dummy class for fault codes.
//
//	Just to encapsulate the enum and provide a message file
//
class Fault
{	
public:
	enum Faultcode { none = 0, starter, shift, engine, 
		steer, brake, throttle, trans, tilthead,	// controller faults
		gps, radar, lidar, gen, 
		overspeed, pitch, pitchrate, roll, rollrate, vibration, yawrate, sideaccel,
		stuck, speed, movedist, offcourse,
		internalerror, movetimeout, speedtimeout,
		manualpause, manualkill, userstop, networkerror, eventsequenceerror, ins, noposition,
		drivingfault, done, missionstopped, nearcollision, obstacle, boundary, slipback, mapcleared, nomission, manualmode,
		sonar, sonarobstacle, tiltedpath, cpuoverrun };
		
	static const char* ErrMsg (Faultcode err);				// message for fault code
};
//
//	Messages for fault codes.  These appear in the LED sign, so they must be very short.
//
inline const char* Fault::ErrMsg(Faultcode err)
{
	switch (err) {
	case none: return("");
	case starter: return("STARTER FAULT");
	case shift: return("SHIFTER FAULT");
	case engine: return("ENGINE FAULT");
	case steer: return("STEERING FAULT");
	case brake: return("BRAKE FAULT");
	case throttle: return("THROTTLE FAULT");		
	case trans: return("TRANS FAULT");
	case tilthead: return("LIDAR HEAD FAULT");
	case gps: return("GPS FAULT");
	case radar: return("RADAR FAULT");
	case lidar: return("LIDAR FAULT");
	case gen: return("GEN FAULT");
	case overspeed: return("OVERSPEED FAULT");
	case pitch: return("PITCH FAULT");
	case pitchrate: return("PITCH RATE FAULT");
	case roll: return("ROLL FAULT");
	case vibration: return("VIBRATION FAULT");
	case yawrate: return("YAW RATE FAULT");
	case sideaccel: return("SIDE ACCEL FAULT");
	case rollrate: return("ROLL RATE FAULT");
	case speed: return("SPEED CONTROL FAULT");
	case stuck: return("VEHICLE STUCK");
	case movedist: return("OVERRAN MOVE");
	case offcourse: return("OFF COURSE");
	case internalerror: return("INTERNAL ERROR");
	case movetimeout: return("MOVE SERVER TIMEOUT");
	case speedtimeout: return("SPEED SERVER TIMEOUT");
	case manualpause: return("PAUSE REQUEST");
	case manualkill: return("E-STOP");
	case userstop: return("USER STOP");
	case networkerror: return("NETWORK FAULT");
	case eventsequenceerror: return("EVENT SEQUENCE FAULT");
	case ins: return("INERTIAL NAV FAULT");
	case noposition: return("LOST GPS");
	case drivingfault: return("DRIVING FAULT");
	case done: return("MISSION COMPLETED");
	case missionstopped: return("MISSION STOPPED");
	case nearcollision: return("NEAR COLLISION");
	case obstacle: return("OBSTACLE");	
	case boundary: return("AT BOUNDARY");
	case slipback: return("SLIPPED BACK");
	case mapcleared: return("MAP CLEARED");
	case nomission: return("NO WAYPOINTS LOADED");
	case manualmode: return("MANUAL DRIVING");
	case sonar: return("SONAR FAULT");
	case sonarobstacle: return("SONAR DETECTED OBSTACLE");
	case tiltedpath: return("PATH TOO TILTED");
	case cpuoverrun: return("CPU OVERRUN");
	default: return("UNKNOWN FAULT");
	}
};	
#endif // FAULTCODES