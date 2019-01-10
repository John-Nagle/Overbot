//
//	***TEMP*** test program
//
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <sys/hidut.h>

#include "hiddconnection.hpp"

class OurDevice: public HIDDdevice
{
private:
	hidd_report_props_t m_xprop, m_yprop, m_zprop, momo_prop;		// X and Y property sets
public:
	OurDevice(HIDDconnection& conn, hidd_device_instance_t *instance)
		: HIDDdevice(conn, instance)
{}
	void onInsertion();
	void onHidReport(struct hidd_report* handle, void* report_data,
                            _uint32 report_len, _uint32 flags);
	void onRemoval();
	void printval(void* report_data, uint16_t usage_page, uint16_t usage, hidd_report_props_t& prop, const char* name);
};

class HIDtest: public HIDDconnection 
{
public:
	HIDtest() 
	{}
protected:
	void onInsertion(hidd_device_instance_t* instance);

};

void HIDtest::onInsertion(hidd_device_instance_t* instance)
{	printf("Device plugged in: ");
	Dump(instance);
	
	struct hidd_collection **hidd_collections = 0;
    _uint16	 num_col;
    // Get root level HID collections
    hidd_get_collections( instance, NULL, &hidd_collections, &num_col);
	//	Loop over outer collections
    for(int i = 0; i < num_col; i++)
    {	_uint16 usage_page, usage, num_mcol;
        hidd_collection_usage(hidd_collections[i], &usage_page, &usage);
		printf("  Usage page %d, usage %d\n", usage_page, usage);
    	struct hidd_collection  **hidd_mcollections;
		hidd_get_collections( NULL, hidd_collections[i], &hidd_mcollections, &num_mcol);
		for (int j=0; j<num_mcol; j++)
		{	printf("    M-collection %d\n",j);
  		}
    }

	//	Try to attach to the device. Get all reports.
	OurDevice* dev = new OurDevice(*this, instance);	// create a device
	int stat = dev->report_attach_if_interested(HIDD_PAGE_DESKTOP, HIDD_USAGE_JOYSTICK);
	if (stat != EOK)
	{	printf("Human interface device not of interest: %s\n",strerror(stat));
		delete(dev);															// we don't want this device
		return;
	}
	//	We have a device.  Create and register it. (move this to base class?)
	Register(dev);															// register to the connection, so we get callbacks.
	return;
}
//
//	printival -- print a value if it is present
//
void OurDevice::printval(void* report_data, uint16_t usage_page, uint16_t usage, hidd_report_props_t& prop, const char* name)
{
	uint32_t val = 0;
	int  stat = hidd_get_usage_value( GetReportInstance(), NULL, usage_page, usage, report_data, &val);
	if (stat != EOK)  return;												// no value available
	float fval = ScaleValue(prop,val);								// scale into range
	printf("%s = %5.3f  ",name,fval);								// print value
}	

//
//	OurDevice -- a joystick tester
//
void OurDevice::onHidReport(struct hidd_report* handle, void* report_data,
                            _uint32 report_len, _uint32 flags)
{
	printval(report_data, HIDD_PAGE_DESKTOP, HIDD_USAGE_X, m_xprop, "X");
	printval(report_data, HIDD_PAGE_DESKTOP, HIDD_USAGE_Y, m_yprop, "Y");
	printval(report_data, HIDD_PAGE_DESKTOP, HIDD_USAGE_Z, m_zprop, "Z");
	printval(report_data, 65280,1,momo_prop,"MOMO");	// accelerator pedal only for MOMO wheel

#ifdef NOTPRESENT	// our joystick doesn't have any of these.
	uint32_t dval = 0;
	uint32_t sval = 0;
	uint32_t wval = 0;
    stat = hidd_get_usage_value( GetReportInstance(), NULL, HIDD_PAGE_DESKTOP, HIDD_USAGE_SLIDER, report_data, &sval);
	if (stat != EOK) { printf("Error getting slider value: %s\n", strerror(stat)); }
    stat = hidd_get_usage_value( GetReportInstance(), NULL, HIDD_PAGE_DESKTOP, HIDD_USAGE_DIAL, report_data, &dval);
	if (stat != EOK) { printf("Error getting dial value: %s\n", strerror(stat)); }
    stat = hidd_get_usage_value( GetReportInstance(), NULL, HIDD_PAGE_DESKTOP, HIDD_USAGE_WHEEL, report_data, &wval);
	if (stat != EOK) { printf("Error getting wheel value: %s\n", strerror(stat)); }	
	uint32_t rxval, ryval, rzval;
	stat = hidd_get_usage_value( GetReportInstance(), NULL, HIDD_PAGE_DESKTOP, HIDD_USAGE_RX, report_data, &rxval);
	if (stat != EOK) { printf("Error getting RX value: %s\n", strerror(stat));  }
    stat = hidd_get_usage_value( GetReportInstance(), NULL, HIDD_PAGE_DESKTOP, HIDD_USAGE_RY, report_data, &ryval);
	if (stat != EOK) { printf("Error getting RY value: %s\n", strerror(stat));  }
    stat = hidd_get_usage_value( GetReportInstance(), NULL, HIDD_PAGE_DESKTOP, HIDD_USAGE_RZ, report_data, &rzval);
	if (stat != EOK) { printf("Error getting RZ value: %s\n", strerror(stat)); }
#endif // NOTPRESENT
	const size_t k_maxbuttons = 20;									// allow way too many buttons
	uint16_t buttons[k_maxbuttons];									// buttons currently pressed
	uint16_t buttoncnt = k_maxbuttons;								// count of buttons pressed
	memset(buttons,sizeof(buttons),0);								// ***TEMP***
	int stat = hidd_get_buttons(GetReportInstance(),NULL, HIDD_PAGE_BUTTONS, report_data, buttons, &buttoncnt);
	if (stat) { printf("Error getting button values: %s\n",strerror(stat)); return; }
	if (buttoncnt > 0)
	{	printf("  Buttons:");
		for (int i=0; i<buttoncnt; i++)
		{	printf(" %d",buttons[i]);	}
	}	
	printf("\n");
}
//
//	onInsertion -- at the device level
//
void OurDevice::onInsertion()
{
	DumpReportProps();																	// ***TEMP***
	//	Get property info for devices that have values
	std::valarray<hidd_report_props_t> allprops;
	int stat = get_report_props(allprops);
	if (stat != EOK) { printf("Error getting report properties: %s\n", strerror(stat)); return; }
	stat = get_report_prop(allprops, HIDD_PAGE_DESKTOP, HIDD_USAGE_X, m_xprop);
	stat = get_report_prop(allprops, HIDD_PAGE_DESKTOP, HIDD_USAGE_Y, m_yprop);
	stat = get_report_prop(allprops, HIDD_PAGE_DESKTOP, HIDD_USAGE_Z, m_zprop);
	stat = get_report_prop(allprops, 65280, 1, momo_prop);	// nonstandard property of Momo wheel

#ifdef NOTYET	// ***GARBAGE VALUES ARE RETURNED FOR POSITIONS***
	//	Set a reasonable idle message rate.
	stat = hidd_set_idle(m_ourdevice.GetReport(), HIDD_TIME_DEFAULT);	// not sure what the values mean
	if (stat != EOK)
	{	printf("Human interface device idle rate not set: %s\n",strerror(stat));
		return;
	}
#endif // NOTYET
}
//
//	onRemoval -- at the device level
//
void OurDevice::onRemoval()
{	printf("Device  removed.\n");
}


int main()
{	HIDtest conn;
	int stat = conn.Connect();
	if (stat != EOK)
	{	perror("Unable to connect to human interface device server"); return(1); }
	while (1) sleep(100);	// wait forever, reporting joystick events
	return(0);
}