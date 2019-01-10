/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

/* Standard headers */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"
#include "vehicle.h"
#include "../../../common/include/dirserver.h"
#include "../../../common/include/brakeserver.h"
#include "../../../common/include/throttleserver.h"
#include "../../../common/include/gearserver.h"

//TODO
//all this vehicle logic should be in a separate file

char * dirservername = "DIR";
MsgClientPort dirclientport(dirservername, 0.2);

char * throttleservername = "THROTTLE";
MsgClientPort throttleclientport(throttleservername, 0.2);

char * brakeservername = "BRAKE";
MsgClientPort brakeclientport(brakeservername, 0.2);

char * gearservername = "GEAR";
MsgClientPort gearclientport(gearservername, 0.2);

/**
 * @returns whether we are initialized with the gear server
 */
 bool
gear_initialized()
{
	int * val;
	PtGetResource(ABW_onOffButton_gear_initialized, Pt_ARG_ONOFF_STATE, &val, 0);
	return *val != 0;
}

/**
 * @returns whether we are initialized with the steering server
 */
 bool
steering_initialized()
{
	int * val;
	PtGetResource(ABW_onOffButton_steering_initialized, Pt_ARG_ONOFF_STATE, &val, 0);
	return *val != 0;
}

/**
 * @returns whether we are initialized with the throttle server
 */
 bool
throttle_initialized()
{
	int * val;
	PtGetResource(ABW_onOffButton_throttle_initialized, Pt_ARG_ONOFF_STATE, &val, 0);
	return *val != 0;
}

/**
 * @returns whether we are initialized with the brake server
 */
 bool
brake_initialized()
{
	int * val;
	PtGetResource(ABW_onOffButton_brake_initialized, Pt_ARG_ONOFF_STATE, &val, 0);
	return *val != 0;
}

/**
 * change the simulation mode of the steering server to sim
 */
void
send_steering_sim(bool sim)
{
	DirServerMsg req;
	req.m_simu.m_msgtype = char4('S', 'I', 'M', 'U');
	req.m_simu.get = false;
	req.m_simu.simulation = sim;
	
	DirServerMsg rep;
	int err = dirclientport.MsgSend(req, rep);
	
	if (err) {																	           // if fail
		perror("send_steering_sim: MsgSend failed");		   // trouble
	}
	else if (rep.m_simu.m_msgtype != DirServerMsgSIMU::k_msgtype) {
		printf("send_steering_sim: rep from Dir Server is not SIMU::k_msgtype\n");
	}
	 else if (rep.m_simu.err != DirServer::ERR_OK) {
		printf("send_steering_sim: err from Dir Server is not ok: %s\n", DirServer::ErrMsg(rep.m_simu.err));		
	}
	else {
		//success
		printf("send_steering_sim: success\n");
	  	PtSetResource(ABW_onOffButton_steering_sim, Pt_ARG_ONOFF_STATE, sim ? 1 : 0, 0);
	}
	
	fflush(stdout); 
	fflush(stderr);
		
}

/**
 * actually send out the INIT msg to the dir server
 * @returns true if alls good, false otherwise
 */
bool send_steering_init()
{
	DirServerMsg req;
	req.m_init.m_msgtype = char4('I', 'N', 'I', 'T');
	req.m_init.get = false;
	
	DirServerMsg rep;
	int err = dirclientport.MsgSend(req, rep);
	
	if (err) {																	           // if fail
		perror("send_steering_init: MsgSend failed");		   // trouble
		return false;
	}
	else if (rep.m_init.m_msgtype != DirServerMsgINIT::k_msgtype) {
		printf("send_steering_init: rep from Dir Server is not INIT::k_msgtype for send_steering_init");
		return false;
	}
	 else if (rep.m_init.err != DirServer::ERR_OK) {
		printf("send_steering_init: err from Dir Server is not ok: %s\n", DirServer::ErrMsg(rep.m_init.err));		
		return false;
	}
	else {
		//success
		printf("send_steering_init: success\n");
	  	PtSetResource(ABW_onOffButton_steering_initialized, Pt_ARG_ONOFF_STATE, 1, 0);
	}
	
	fflush(stdout); 
	fflush(stderr);
	
	return true;
}

/**
 * gets the desired steering angle from the steering target numeric
 */
float getTargSteeringAngle()
{
 	int * val;
	
	PtGetResource (ABW_numeric_steering, Pt_ARG_NUMERIC_VALUE, &val, 0);
	
	return (float)*val;
	
}

float getTargSteeringAngleRate()
{
	int * val;

	PtGetResource (ABW_numeric_steering_rate, Pt_ARG_NUMERIC_VALUE, &val, 0);
	
	return (float)*val;
	
}

/**
 * sends out the DATA command to the DIR server
 */
void 
send_steering_cmd()
{
	if (!steering_initialized()) {
		printf("send_steering_cmd: steering not initialized, not sending msg\n");
		fflush(stdout);
		return;
	}

 	DirServerMsg req;
 	req.m_data.m_msgtype = char4('D', 'A', 'T', 'A');
 	req.m_data.angle_target = 	getTargSteeringAngle();
 	req.m_data.angle_rate_max_target = getTargSteeringAngleRate(); //degrees per sec
 	//we are setting now
	req.m_data.get = false;

	printf("target steering angle: %f\n", req.m_data.angle_target);
	fflush(stdout);
				
	DirServerMsg rep;
	int err = dirclientport.MsgSend(req, rep); 	
	
	if (err) {																				// if fail
		perror("send_steering_cmd: Client MsgSend");						// trouble
	}
	else if (rep.m_data.m_msgtype != DirServerMsgDATA::k_msgtype ||
	          rep.m_data.err != DirServer::ERR_OK) {
		printf("send_steering_cmd: err from Dir Server is not ok: %s\n", DirServer::ErrMsg(rep.m_data.err));
		fflush(stdout);
	} 	
}

/**
 * update the 4 meters for steering
 */
void update_steering_gui(float angle_target, 
										float angle_rate_max_target, 
										float angle, 
										float angle_rate)
{
	
	PtSetResource (ABW_meter_target_steering, Pt_ARG_METER_NEEDLE_POSITION, (long)angle_target, 0);
	PtSetResource (ABW_meter_current_steering, Pt_ARG_METER_NEEDLE_POSITION, (long)angle, 0);
	PtSetResource (ABW_meter_target_steering_rate, Pt_ARG_METER_NEEDLE_POSITION, (long)angle_rate_max_target, 0);
	PtSetResource (ABW_meter_current_steering_rate, Pt_ARG_METER_NEEDLE_POSITION, (long)angle_rate, 0);
	
	
}

/**
 * this gets called periodically to get the steering data from the DIR server
 * it then calls update_steering_gui() to update the 2 meter displays
 */
void get_steering_data()
{
	
	if (!steering_initialized()) {
		printf("get_steering_data: steering needs to be initialized\n");
		fflush(stdout);
		return;
	}	
	
 	DirServerMsg req;
	req.m_data.m_msgtype = char4('D','A','T','A');
	//we are getting now
	req.m_data.get = true;
 	
	DirServerMsg rep;
	
	int err = dirclientport.MsgSend(req, rep); 	
	
	if (err) {																				// if fail
		perror("get_steering_data: Client MsgSend");						// trouble
	}
	else if (rep.m_data.m_msgtype != DirServerMsgDATA::k_msgtype ||
	          rep.m_data.err != DirServer::ERR_OK) {
		printf("get_steering_data: err from Dir Server is not ok err: %s\n", DirServer::ErrMsg(rep.m_data.err));
		fflush(stdout);
	} 
	else {
	  	//got the data we want

		float angle_target 				= rep.m_data.angle_target;
		float angle_rate_max_target = rep.m_data.angle_rate_max_target;

		float angle 		    				= rep.m_data.angle;
		float angle_rate					= rep.m_data.angle_rate;
	  	
	  	//printf("get_steering_data: got the data we want angle_target: %f, angle: %f, angle_rate_max_target: %f, angle_rate: %f\n",
	  	//		angle_target, angle, angle_rate_max_target, angle_rate);         
		//fflush(stdout);
			    
	    //update gui's
	    update_steering_gui(angle_target, angle_rate_max_target, angle, angle_rate);
	
	}
}

/**
 * change the simulation mode of the throttle server to sim
 */
void
send_throttle_sim(bool sim)
{
	ThrottleServerMsg req;
	req.m_simu.m_msgtype = char4('S', 'I', 'M', 'U');
	req.m_simu.get = false;
	req.m_simu.simulation = sim;
	
	ThrottleServerMsg rep;
	int err = throttleclientport.MsgSend(req, rep);
	
	if (err) {																	           // if fail
		perror("send_throttle_sim: MsgSend failed");		   // trouble
	}
	else if (rep.m_simu.m_msgtype != ActServerMsgSIMU::k_msgtype) {
		printf("send_throttle_sim: rep from Throttle Server is not SIMU::k_msgtype\n");
	}
	 else if (rep.m_simu.err != ActServer::ERR_OK) {
		printf("send_throttle_sim: err from Throttle Server is not ok: %s\n", ActServer::ErrMsg(rep.m_simu.err));		
	}
	else {
		//success
		printf("send_throttle_sim: success\n");
	  	PtSetResource(ABW_onOffButton_throttle_sim, Pt_ARG_ONOFF_STATE, sim ? 1 : 0, 0);
	}
	
	fflush(stdout); 
	fflush(stderr);
		
}


/**
 * actually send the INIT msg to the THROTTLE server
 */
bool
send_throttle_init()
{
	printf("send_throttle_init\n");
	
	ThrottleServerMsg req;
	req.m_init.m_msgtype = char4('I', 'N', 'I', 'T');
	req.m_init.get = false;
	
	ThrottleServerMsg rep;
	int err = throttleclientport.MsgSend(req, rep);
	
	if (err) {																	           // if fail
		perror("send_throttle_init: MsgSend failed");		   // trouble
		fflush(stdout);
		fflush(stderr);
		return false;
	}
	else if (rep.m_init.m_msgtype != ActServerMsgINIT::k_msgtype) {
		printf("send_throttle_init: rep from Throttle Server is not INIT::k_msgtype");
		fflush(stdout);
		return false;
	}
	 else if (rep.m_init.err != ActServer::ERR_OK) {
		printf("send_throttle_init: err from Throttle Server is not ok: %s\n", ActServer::ErrMsg(rep.m_init.err));		
		fflush(stdout);
		return false;
	}
	else {
		//success
		printf("send_throttle_init: success\n");
	  	PtSetResource(ABW_onOffButton_throttle_initialized, Pt_ARG_ONOFF_STATE, 1, 0);
	}
	
	fflush(stdout); 
	fflush(stderr);
	
	return true;
}

/**
 * gets the target throttle from the target throttle numeric
 */
float
getTargThrottle()
{
	int * val;
	
	PtGetResource(ABW_numeric_throttle_and_brake, Pt_ARG_NUMERIC_VALUE, &val, 0);
	
	float target = (float)*val;

	//here we do bounds check, also taking note that -100 to 0 refers to brake and
	//gets bounded to 0 here
	if (target < MIN_THROTTLE) {
		target = MIN_THROTTLE;
	}
	else if (target > MAX_THROTTLE) {
		target = MAX_THROTTLE;
	}
	
	return target;	
}



/**
 * send the THROTTLE server a DATA msg with get = false
 */
void
send_throttle_cmd()
{
	if (!throttle_initialized() || !brake_initialized()) {
		printf("send_throttle_cmd: Throttle and Brake need to be both initialized to send msg\n");
		fflush(stdout);
		return;
	} 
	
 	ThrottleServerMsg req;
 	req.m_data.m_msgtype = char4('D', 'A', 'T', 'A');
 	req.m_data.pressure_target = 	getTargThrottle();
 	//we are setting now
	req.m_data.get = false;

	printf("target throttle pressure: %f\n", req.m_data.pressure_target);
	fflush(stdout);
				
	ThrottleServerMsg rep;
	int err = throttleclientport.MsgSend(req, rep); 	
	
	if (err) {																				// if fail
		perror("send_throttle_cmd: Client MsgSend");						// trouble
		fflush(stdout);
		fflush(stderr);
	}
	else if (rep.m_data.m_msgtype != ThrottleServerMsgDATA::k_msgtype ||
	          rep.m_data.err != ThrottleServer::ERR_OK) {
		printf("send_throttle_cmd: err from Throttle Server is not ok: %s\n", ThrottleServer::ErrMsg(rep.m_data.err));
		fflush(stdout);
	} 	
}

void
update_throttle_gui(float pressure_target, float pressure)
{
	PtSetResource(ABW_meter_target_throttle, Pt_ARG_METER_NEEDLE_POSITION, (long)pressure_target, 0);
	PtSetResource(ABW_meter_current_throttle, Pt_ARG_METER_NEEDLE_POSITION, (long)pressure, 0);
}

/**
 * send the THROTTLE server a DATA msg with get = true
 */
void
get_throttle_data()
{
	if (!throttle_initialized()) {
		printf("get_throttle_data: throttle needs to be initialized\n");
		return;
	}	
	
	ThrottleServerMsg req;
	req.m_data.m_msgtype = char4('D','A','T','A');
	//we are getting now
	req.m_data.get = true;
 	
	ThrottleServerMsg rep;
	
	int err = throttleclientport.MsgSend(req, rep); 	
	
	if (err) {																				// if fail
		perror("get_throttle_data: Client MsgSend");						// trouble
		fflush(stdout);
		fflush(stderr);
	}
	else if (rep.m_data.m_msgtype != ThrottleServerMsgDATA::k_msgtype ||
	          rep.m_data.err != ThrottleServer::ERR_OK) {
		printf("get_throttle_data: err from Throttle Server is not ok err: %s\n", ThrottleServer::ErrMsg(rep.m_data.err));
		fflush(stdout);
	} 
	else {
	  	//got the data we want

			
		float pressure_target 			= rep.m_data.pressure_target;
		float pressure						= rep.m_data.pressure;
			  	
	  	//printf("get_throttle_data:  pressure_target: %f, pressure: %f\n",
	  	//		pressure_target, pressure);
	  	//fflush(stdout);
			    
	    //update gui's
	    update_throttle_gui(pressure_target, pressure);
		
	}

}

/**
 * change the simulation mode of the brake server to sim
 */
void
send_brake_sim(bool sim)
{
	BrakeServerMsg req;
	req.m_simu.m_msgtype = char4('S', 'I', 'M', 'U');
	req.m_simu.get = false;
	req.m_simu.simulation = sim;
	
	BrakeServerMsg rep;
	int err = brakeclientport.MsgSend(req, rep);
	
	if (err) {																	           // if fail
		perror("send_brake_sim: MsgSend failed");		   // trouble
	}
	else if (rep.m_simu.m_msgtype != ActServerMsgSIMU::k_msgtype) {
		printf("send_brake_sim: rep from Brake Server is not SIMU::k_msgtype\n");
	}
	 else if (rep.m_simu.err != ActServer::ERR_OK) {
		printf("send_brake_sim: err from Brake Server is not ok: %s\n", ActServer::ErrMsg(rep.m_simu.err));		
	}
	else {
		//success
		printf("send_brake_sim: success\n");
	  	PtSetResource(ABW_onOffButton_brake_sim, Pt_ARG_ONOFF_STATE, sim ? 1 : 0, 0);
	}
	
	fflush(stdout); 
	fflush(stderr);

}

void
update_brake_gui(float pressure_target, float pressure)
{
	PtSetResource(ABW_meter_target_brake, Pt_ARG_METER_NEEDLE_POSITION, (long)pressure_target, 0);
	PtSetResource(ABW_meter_current_brake, Pt_ARG_METER_NEEDLE_POSITION, (long)pressure, 0);
}


/**
 * gets the target brake from the target brake numeric
 */
float
getTargBrake()
{
	int * val;
	
	PtGetResource(ABW_numeric_throttle_and_brake, Pt_ARG_NUMERIC_VALUE, &val, 0);
	
	//note that here we reverse the sign because 0% to 100% maps to throttle
	//0% to -100% maps to 0% brakes to 100% brakes
	float target = -(float)*val;

	//here we do bounds check, also taking note that 0 to 100 refers to throttle and
	//gets bounded to 0 here
	if (target < MIN_BRAKE) {
		target = MIN_BRAKE;
	}
	else if (target > MAX_BRAKE) {
		target = MAX_BRAKE;
	}
	
	return target;	
}

/** 
 * send the INIT msg to the BRAKE server
 */
bool
send_brake_init()
{
	printf("send_brake_init\n");
	fflush(stdout);
	
	BrakeServerMsg req;
	req.m_init.m_msgtype = char4('I', 'N', 'I', 'T');
	req.m_init.get = false;
	
	BrakeServerMsg rep;
	int err = brakeclientport.MsgSend(req, rep);
	
	if (err) {																	           // if fail
		perror("send_brake_init: MsgSend failed");		   // trouble
		fflush(stdout);
		fflush(stderr);
		return false;
	}
	else if (rep.m_init.m_msgtype != ActServerMsgINIT::k_msgtype) {
		printf("send_brake_init: rep from Brake Server is not INIT::k_msgtype");
		fflush(stdout);
		return false;
	}
	 else if (rep.m_init.err != ActServer::ERR_OK) {
		printf("send_brake_init: err from Brake Server is not ok: %s\n", ActServer::ErrMsg(rep.m_init.err));		
		fflush(stdout);
		return false;
	}
	else {
		//success
		printf("send_brake_init: success\n");
	  	PtSetResource(ABW_onOffButton_brake_initialized, Pt_ARG_ONOFF_STATE, 1, 0);
	}
	
	fflush(stdout); 
	fflush(stderr);
	
	return true;

}

/**
 * send the DATA msg to the BRAKE Server with get = false
 */
void
send_brake_cmd()
{
	if (!brake_initialized()) {
		printf("send_brake_cmd: brake need to be initialized\n");
		fflush(stdout);
		return;
	}
	
 	BrakeServerMsg req;
 	req.m_data.m_msgtype = char4('D', 'A', 'T', 'A');
 	req.m_data.pressure_target = 	getTargBrake();
 	//we are setting now
	req.m_data.get = false;

	printf("target brake pressure: %f\n", req.m_data.pressure_target);
	fflush(stdout);
				
	BrakeServerMsg rep;
	int err = brakeclientport.MsgSend(req, rep); 	
	
	if (err) {																				// if fail
		perror("send_brake_cmd: Client MsgSend");						// trouble
		fflush(stdout);
		fflush(stderr);
	}
	else if (rep.m_data.m_msgtype != BrakeServerMsgDATA::k_msgtype ||
	          rep.m_data.err != BrakeServer::ERR_OK) {
		printf("send_brake_cmd: err from Brake Server is not ok: %s\n", BrakeServer::ErrMsg(rep.m_data.err));
		fflush(stdout);
	} 	

}



/**
 * send the DATA msg to the BRAKE Server with get = true
 */
void
get_brake_data()
{
	if (!brake_initialized()) {
		printf("get_brake_data: brake needs to be initialized\n");
		fflush(stdout);
		return;
	}	
	
	BrakeServerMsg req;
	req.m_data.m_msgtype = char4('D','A','T','A');
	//we are getting now
	req.m_data.get = true;
 	
	BrakeServerMsg rep;
	
	int err = brakeclientport.MsgSend(req, rep); 	
	
	if (err) {																				// if fail
		perror("get_brake_data: Client MsgSend");						// trouble
		fflush(stdout);
		fflush(stderr);
	}
	else if (rep.m_data.m_msgtype != BrakeServerMsgDATA::k_msgtype ||
	          rep.m_data.err != BrakeServer::ERR_OK) {
		printf("get_brake_data: err from Brake Server is not ok err: %s\n", BrakeServer::ErrMsg(rep.m_data.err));
		fflush(stdout);
	} 
	else {
	  	//got the data we want
			
		float pressure_target 			= rep.m_data.pressure_target;
		float pressure						= rep.m_data.pressure;
			  	
	  	//printf("get_brake_data:  pressure_target: %f, pressure: %f\n",
	  	//		pressure_target, pressure);
	  	//fflush(stdout);
			    
	    //update gui's
	    update_brake_gui(pressure_target, pressure);
		
	}
}

/**
 * change the simulation mode of the gear server to sim
 */
void
send_gear_sim(bool sim)
{
	GearServerMsg req;
	req.m_simu.m_msgtype = char4('S', 'I', 'M', 'U');
	req.m_simu.get = false;
	req.m_simu.simulation = sim;
	
	GearServerMsg rep;
	int err = gearclientport.MsgSend(req, rep);
	
	if (err) {																	           // if fail
		perror("send_gear_sim: MsgSend failed");		   // trouble
	}
	else if (rep.m_simu.m_msgtype != ActServerMsgSIMU::k_msgtype) {
		printf("send_gear_sim: rep from Gear Server is not SIMU::k_msgtype\n");
	}
	 else if (rep.m_simu.err != ActServer::ERR_OK) {
		printf("send_gear_sim: err from Gear Server is not ok: %s\n", ActServer::ErrMsg(rep.m_simu.err));		
	}
	else {
		//success
		printf("send_gear_sim: success\n");
	  	PtSetResource(ABW_onOffButton_gear_sim, Pt_ARG_ONOFF_STATE, sim ? 1 : 0, 0);
	}
	
	fflush(stdout); 
	fflush(stderr);

}


/**
 * send the INIT msg to the GEAR server
 */
bool
send_gear_init()
{
	//printf("send_gear_init\n");
	
	GearServerMsg req;
	req.m_init.m_msgtype = char4('I', 'N', 'I', 'T');
	req.m_init.get = false;
	
	GearServerMsg rep;
	int err = gearclientport.MsgSend(req, rep);
	
	if (err) {																	           // if fail
		perror("send_gear_init: MsgSend failed");		   // trouble
		fflush(stdout);
		fflush(stderr);
		return false;
	}
	else if (rep.m_init.m_msgtype != ActServerMsgINIT::k_msgtype) {
		printf("send_gear_init: rep from Gear Server is not INIT::k_msgtype\n");
		fflush(stdout);
		return false;
	}
	 else if (rep.m_init.err != ActServer::ERR_OK) {
		printf("send_gear_init: err from Gear Server is not ok: %s\n", ActServer::ErrMsg(rep.m_init.err));		
		fflush(stdout);
		return false;
	}
	else {
		//success
		printf("send_gear_init: success\n");
	  	PtSetResource(ABW_onOffButton_gear_initialized, Pt_ARG_ONOFF_STATE, 1, 0);
	}
	
	fflush(stdout); 
	fflush(stderr);
	
	return true;

}

/**
 * gets the target gear from the gui
 * this really should return the type GearServer::Gear but proto.h cannot easily know about this type
 * so we cast to int and back to GearServer::Gear in the caller of this function
 */
int
getTargGear()
{
	int * gear_high;
	int * gear_low;
	int * gear_neutral;
	int * gear_reverse;
	
	PtGetResource(ABW_onOffButton_gear_high, Pt_ARG_ONOFF_STATE, &gear_high, 0);
	PtGetResource(ABW_onOffButton_gear_low, Pt_ARG_ONOFF_STATE, &gear_low, 0);
	PtGetResource(ABW_onOffButton_gear_neutral, Pt_ARG_ONOFF_STATE, &gear_neutral, 0);
	PtGetResource(ABW_onOffButton_gear_reverse, Pt_ARG_ONOFF_STATE, &gear_reverse, 0);
	
	//check that only one and only one of them is true
	assert(*gear_high + *gear_low + *gear_neutral + *gear_reverse == 1);
	
	if (*gear_high) {
		return (int)GearServer::GEAR_H;
	}
	else if (*gear_low) {
		return (int)GearServer::GEAR_L;
	}
	else if (*gear_neutral) {
		return (int)GearServer::GEAR_N;
	}
	else if (*gear_reverse) {
		return (int)GearServer::GEAR_R;
	}
	else {
		printf("getTargGear: dead else\n");
		assert(0);
		return -1;
	}
}

void
send_gear_cmd()
{
	if (!gear_initialized()) {
		printf("send_gear_cmd: gear not initialized, not sending msg\n");
		fflush(stdout);
		return;
	}
					   
 	GearServerMsg req;
 	req.m_data.m_msgtype = char4('D', 'A', 'T', 'A');
 	req.m_data.gear_target = (GearServer::Gear)getTargGear();
 	//we are setting now
	req.m_data.get = false;

	printf("target gear: %s\n", GearServer::GearMsg(req.m_data.gear_target));
	fflush(stdout);
				
	GearServerMsg rep;
	int err = gearclientport.MsgSend(req, rep); 	
	
	if (err) {																				// if fail
		perror("send_gear_cmd: Client MsgSend");						// trouble
		fflush(stdout);
		fflush(stderr);
	}
	else if (rep.m_data.m_msgtype != GearServerMsgDATA::k_msgtype ||
	          rep.m_data.err != GearServer::ERR_OK) {
		printf("send_gear_cmd: err from Gear Server is not ok: %s\n", GearServer::ErrMsg(rep.m_data.err));
		fflush(stdout);
	} 	
	
}

void
get_gear_data()
{
	if (!gear_initialized()) {
		printf("get_gear_data: gear need to be initialized\n");
		fflush(stdout);
		return;
	}
	//printf("get_gear_data\n");
	
	GearServerMsg req;
	req.m_data.m_msgtype = char4('D','A','T','A');
	//we are getting now
	req.m_data.get = true;
 	
	GearServerMsg rep;
	
	int err = gearclientport.MsgSend(req, rep); 	
	
	if (err) {																				// if fail
		perror("get_gear_data: Client MsgSend");						// trouble
		fflush(stdout);
		fflush(stderr);
	}
	else if (rep.m_data.m_msgtype != GearServerMsgDATA::k_msgtype ||
	          rep.m_data.err != GearServer::ERR_OK) {
		printf("get_gear_data: err from Gear Server is not ok err: %s\n", GearServer::ErrMsg(rep.m_data.err));
		fflush(stdout);
	} 
	else {
	  	//got the data we want
		
		//printf("get_brake_data: gear_target: %s, gear: %s\n",
		//GearServer::GearMsg(	rep.m_data.gear_target),
		//GearServer::GearMsg(rep.m_data.gear));
		//fflush(stdout);
			    
	    //update gui's
	    update_gear_gui(GearServer::GearMsg(rep.m_data.gear_target), 
	    					       GearServer::GearMsg(rep.m_data.gear));
	}
}

void
update_gear_gui(char * gear_target, char *gear)
{
		PtSetResource(ABW_text_gear_target, Pt_ARG_TEXT_STRING, gear_target, 0);
		PtSetResource(ABW_text_gear, Pt_ARG_TEXT_STRING, gear, 0);
}