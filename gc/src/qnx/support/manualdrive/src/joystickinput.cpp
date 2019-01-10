//
//	joystickinput.cpp  --  Joystick input support
//
//	John Nagle
//	Team Overbot
//	October, 2003
//
#include <assert.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/hidut.h>

#include "manualdrive.h"
#include "logprint.h"
#include "proto.h"
//
//	Joystick support
//
//
//	onInsertion -- a new USB device has appeared.
//
//	If the device is something we want to talk to, we request reports of what it
//	is doing and attach a device object to it. The device object will then
//	get all further callbacks.
//
//	We get these messages at startup, too, so this handles startup as well.
//
void JoystickConnection::onInsertion(hidd_device_instance_t* instance)
{	//	Try to attach to the device. Get all reports.
	JoystickDevice* dev = new JoystickDevice(*this, instance);	// create a device
	int stat = dev->report_attach_if_interested(HIDD_PAGE_DESKTOP, HIDD_USAGE_JOYSTICK);
	if (stat != EOK)
	{	printf("Human interface device not of interest: %s\n",strerror(stat));
		delete(dev);															// we don't want this device
		return;
	}
	//	We have a device.  Create and register it. 
	Register(dev);															// register to the connection, so we get callbacks.
	return;
}
//
//	onHidReport -- an incoming input report has been received.
//
//	Note that this is called from a thread with a tiny stack, so there are
//	limits to what can be done here.
//
void JoystickDevice::onHidReport(struct hidd_report* handle, void* report_data,
                            _uint32 report_len, _uint32 flags)
{	uint32_t xval = 0;
	uint32_t yval = 0;
    int stat = hidd_get_usage_value( GetReportInstance(), NULL, HIDD_PAGE_DESKTOP, HIDD_USAGE_X, report_data, &xval);
	if (stat != EOK) { logprintf("Joystick error getting X value"); return; }
    stat = hidd_get_usage_value( GetReportInstance(), NULL, HIDD_PAGE_DESKTOP, HIDD_USAGE_Y, report_data, &yval);
	if (stat != EOK) { trouble("Joystick error getting Y value"); return; }
	float x = ScaleValue(m_xprop,xval);								// scale into range
	float y = ScaleValue(m_yprop,yval);								// scale into range
	GetManualDrive().joystick_move(x,y);							// tell GUI about joystick action.
	const size_t k_maxbuttons = 20;									// allow way too many buttons
	uint16_t buttons[k_maxbuttons];									// buttons currently pressed
	uint16_t buttoncnt = k_maxbuttons;								// count of buttons pressed
	memset(buttons,sizeof(buttons),0);								// ***TEMP***
	stat = hidd_get_buttons(GetReportInstance(),NULL, HIDD_PAGE_BUTTONS, report_data, buttons, &buttoncnt);
	if (stat) { trouble("Error getting joystick button values"); return; }
	if (buttoncnt > 0)
	{	////printf("  Buttons:");
		////for (int i=0; i<buttoncnt; i++)
		////{	printf(" %d",buttons[i]);	}
		GetManualDrive().joystick_button(true,false);			// ***TEMP*** all buttons are e-stops
	}	
}
//
//	onInsertion -- at the device level
//
void JoystickDevice::onInsertion()
{
	//	Get property info for devices that have values
	std::valarray<hidd_report_props_t> allprops;
	int stat = get_report_props(allprops);
	if (stat == EOK) 	{stat = get_report_prop(allprops, HIDD_PAGE_DESKTOP, HIDD_USAGE_X, m_xprop); }
	if (stat == EOK)   	{stat = get_report_prop(allprops, HIDD_PAGE_DESKTOP, HIDD_USAGE_Y, m_yprop); }
	if (stat != EOK)
	{	trouble("Joystick connection problem. Disabling joystick."); return; }
	GetManualDrive().joystick_valid(true,"Joystick present.");			// report joystick present
}
//
//	onRemoval -- the device has been removed.
//
//	The library will shortly delete the device object.
//
void JoystickDevice::onRemoval()
{	trouble("Joystick unplugged.");
}
//
//	GetManualDrive -- get owner object
//
Manualdrive& JoystickDevice::GetManualDrive()
{	HIDDconnection& conn = GetOwner();										// owner object	
	JoystickConnection& joystickconn = dynamic_cast<JoystickConnection&>(conn); 
	return(joystickconn.GetOwner());												// finally the desired object
}
//
//	trouble -- report joystick trouble
//
void JoystickDevice::trouble(const char* msg)
{	GetManualDrive().joystick_valid(false,msg);	}							// report problem
