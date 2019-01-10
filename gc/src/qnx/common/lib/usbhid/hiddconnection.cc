//
//	hiddconnection.cpp  -- encapsulation for QNX Human Interface Device Driver access
//
//	John Nagle
//	Team Overbot
//	June, 2004.
//
//	Before this will work, the HDD driver must be started:
//
//		/sbin/devu-uhci &
// 		/sbin/io-hid &
// 		mount -Tio-hid devh-usb.so &
// 		chmod 666 /dev/io-hid/io-hid
//
#include <errno.h>
#include <assert.h>
#include <stdio.h>
#include <sys/hiddi.h>
#include <sys/hidut.h>
#include "hiddconnection.hpp"
//
//
//	class HIDDconnection -- one connection to an HIDD server
//
//
//	Statics
//
std::list<HIDDconnection*> HIDDconnection::m_registry;				// list of all HIDDconnection objects
ost::Mutex HIDDconnection::m_registrylock;									// lock on above collection
//
//	Constructor
//
HIDDconnection::HIDDconnection()
: m_connection(0)
{	Register(this);	}																		// make self known for later callbacks
//
//	Destructor
//
HIDDconnection::~HIDDconnection()
{	Disconnect();	
	Unregister(this);
}

int HIDDconnection::Connect()
{	if (m_connection) return(EBUSY);									// already in use
    hidd_funcs_t funcs = {_HIDDI_NFUNCS,
                          onInsertion,
                          onRemoval,
                          onHidReport,
                          0};
    //	Devices we are interested in
    hidd_device_ident_t* interest = 0;								// accept everything
	//	Connection parameters
    hidd_connect_parm_t parm = {0,
                                HID_VERSION,
                                HIDD_VERSION,
                                0,
                                0,
                                interest,
                                &funcs,
                                HIDD_CONNECT_WAIT};
 	// Connect to the server, if possible                               
	int stat = hidd_connect(&parm, &m_connection);
	if (stat != EOK)																// insure m_connection null if no connection
	{	m_connection = 0; }	
	return(stat);
}
//
//	Disconnect -- disconnect from server
//
//	Stops callback thread
//
int HIDDconnection::Disconnect()
{	if (!m_connection) return(EOK);										// nothing to do
	int stat = hidd_disconnect(m_connection);					// disconnect
	m_connection = 0;														// forget connection
	return(stat);
}
//
//	Callbacks
//                    
//
//	onInsertion (static) -- a new device has been inserted
//
void HIDDconnection::onInsertion(struct hidd_connection* connection,
                            hidd_device_instance_t* instance)
{	HIDDconnection* conn = FindByConnection(connection);		// look up correct target object
	if (conn) conn->onInsertion(instance);										// dispatch to it
}
//
//	onRemoval (static)  -- an existing device has been removed
//                            
void HIDDconnection::onRemoval(struct hidd_connection* connection,
                          hidd_device_instance_t* instance)
{	HIDDconnection* conn = FindByConnection(connection);		// look up correct target object
	printf("Removal. %s\n", conn ? "OK" : "No match");					// ***TEMP***
	if (conn) conn->onRemoval(instance);										// dispatch to it
}

//
//	onHidReport (static) -- a device we are interested in has something to tell us
//
//	We only get these for specific devices for which we have requested callbacks.
//
void HIDDconnection::onHidReport(struct hidd_connection* connection,
                            struct hidd_report* handle, void* report_data,
                            _uint32 report_len, _uint32 flags, void* user)
{
	HIDDconnection* conn = FindByConnection(connection);		// look up correct target object
	if (conn) conn->onHidReport(handle, report_data, report_len, flags);		// dispatch to it
}
//
//	onRemoval  -- non-static
//	
//	Passed to device, then device is deleted.
//
void HIDDconnection::onRemoval(hidd_device_instance_t* instance)
{	HIDDdevice* dev = FindByDevice(instance);							// find relevant device
	if (!dev) return;																			// no such object
	dev->onRemoval();																	// tell device it has been removed
	Unregister(dev);																		// unregister the device
	delete(dev);																				// done with device
}
//
//	onHidReport -- non-static
//
//	Passed to device
//
void HIDDconnection::onHidReport(struct hidd_report *handle, void *report_data,
                            _uint32 report_len, _uint32 flags)
 {	HIDDdevice* dev =	FindByReport(handle);								// find relevant device
	if (!dev) return;																			// no such object
	dev->onHidReport(handle, report_data, report_len, flags);		// pass report along
}        

//
//	Static registry - registry of all HIDDconnection objects.
//
//	Used for callback dispatch
//
//	We maintain a list of all HIDDconnection objects. We have to do this because the
//	"insert" and "remove" callbacks don't include a user-definable field. So we have to
//	do a lookup to find out which function needs to be called.
//
HIDDconnection* HIDDconnection::FindByConnection(struct hidd_connection* connection)
{	ost::MutexLock lok(m_registrylock);							// lock registry
	for (std::list<HIDDconnection*>::iterator p = m_registry.begin(); p != m_registry.end(); p++)
	{	HIDDconnection* conn = (*p);
		if (conn->GetConnection() == connection) return(conn);
	}
	return(0);
}
//
//	Registry of connections, per program 
//
void HIDDconnection::Register(HIDDconnection* conn)
{	ost::MutexLock lok(m_registrylock);							// lock registry
	m_registry.push_back(conn);									// add to list
}
void HIDDconnection::Unregister(HIDDconnection* conn)
{	ost::MutexLock lok(m_registrylock);							// lock registry
	for (std::list<HIDDconnection*>::iterator p = m_registry.begin(); p != m_registry.end(); p++)
	{	 if ((*p) == conn)													// if find
		{	m_registry.erase(p);											// remove from list
			return;
		}
	}
}
//
//	Registry of devices, per connection
//
//	Register -- tell connection about a newly registered device
//
//	The device must first be registered for reports, using "report_attach_if_interested".
//	Then it must be attached here.
//	
void HIDDconnection::Register(HIDDdevice* dev)
{	ost::MutexLock lok(m_lock);										// lock connection
	assert(dev);																// dev must exist
	assert(dev->GetReport());										// dev must be registered to get reports
	m_devices.push_back(dev);										// add to list
	dev->onInsertion();													// tell device about insertion
}
void HIDDconnection::Unregister(HIDDdevice* dev)
{	ost::MutexLock lok(m_lock);										// lock connection
	for (std::list<HIDDdevice*>::iterator p = m_devices.begin(); p != m_devices.end(); p++)
	{	 if ((*p) == dev)													// if find
		{	m_devices.erase(p);											// remove from list
			return;
		}
	}
}
//
//	FindByReport -- who gets this report?
//
//	Linear search, but usually N=1. Maybe 2.
//
HIDDdevice* HIDDconnection::FindByReport(struct hidd_report* report)
{	ost::MutexLock lok(m_lock);										// lock connection
	for (std::list<HIDDdevice*>::iterator p = m_devices.begin(); p != m_devices.end(); p++)
	{	HIDDdevice* dev = (*p);
		if (dev->GetReport() == report) return(dev);		// find
	}
	return(0);
}
HIDDdevice* HIDDconnection::FindByDevice(struct hidd_device_instance* device)
{	ost::MutexLock lok(m_lock);									// lock registry
	for (std::list<HIDDdevice*>::iterator p = m_devices.begin(); p != m_devices.end(); p++)
	{	HIDDdevice* dev = (*p);
		if (dev->GetDevice() == device) return(dev);	// find
	}
	return(0);
}

//
//	Dump  -- dump device instance information
//
void HIDDconnection::Dump(hidd_device_instance* deviceinstance)
{
	assert(deviceinstance);
    char mfgr[100], product[100];
    hidd_get_manufacturer_string(GetConnection(), deviceinstance, mfgr, sizeof(mfgr));
    hidd_get_product_string(GetConnection(), deviceinstance, product, sizeof(product));
	printf("Manufacturer: %s  Product: %s\n",mfgr,product);  
}
//
//	class HIDDdevice -- one device attached to an HIDD connection
//
//
//	Constructor
//
HIDDdevice::HIDDdevice(HIDDconnection& owner, hidd_device_instance_t* deviceinstance)
:	m_conn(owner), m_report(0), m_reportinstance(0), m_device(deviceinstance),
	m_collection(0)
{
}
//
//	Destructor
//
HIDDdevice::~HIDDdevice()
{	report_detach();																// detach from any existing reports
	m_conn.Unregister(this);													// unregister self
}

//
//	report_attach -- attach to a device
//
int HIDDdevice::report_attach(hidd_device_instance_t* deviceinstance, hidd_report_instance* reportinstance, uint32_t flags)
{	report_detach();																// detach if attached
	// attach new instance
	if (!m_conn.GetConnection()) return(ENOTCONN);				// fails if not connected
	return(hidd_report_attach(m_conn.GetConnection(), deviceinstance, reportinstance, flags, 0, &m_report));
}			

//
//	report_detach -- detach if attached
//
int HIDDdevice::report_detach()
{	if (!m_device) return(EOK);												// no device	
	if (!m_report) return(EOK);												// not attached
	int stat = hidd_report_detach(m_report);							// detatch
	m_report = 0;																	// no report
	return(stat);
}
//
//	report_attach_if_interested  -- attach to this device if it is one we want
//
//	For example, to attach to a joystick, ask for HIDD_PAGE_DESKTOP, HIDD_USAGE_JOYSTICK
//
//	Recursive descent, returns first match. Call with startcoll=0 (default) to start.
//
int HIDDdevice::report_attach_if_interested(uint16_t pagewanted, uint16_t usagewanted, struct hidd_collection* startcoll)
{
    //	Detach from any previous reports
    report_detach();
    m_collection = 0;													// no current collection
    // Get root level HID collections
    uint16_t num_col;
    struct hidd_connection* connection = GetOwner().GetConnection();
    if (!connection) return(ENOTCONN);						// no connection to server
    struct hidd_collection** hidd_collections = 0;
    //	Start at device root if startcoll == NULL. Otherwise start at startcoll
    hidd_device_instance* device = (startcoll) ? 0 : m_device;
    int stat = hidd_get_collections(device, startcoll, &hidd_collections, &num_col);
    if (stat != EOK) return(stat);									// fails
    if (num_col == 0) return(ENODEV);						// no collections
    assert(hidd_collections);										// should now have a table
	//	Examine all collections for this device instance.
    for(unsigned int i = 0; i < num_col; i++)
    {	uint16_t usage_page, usage;
        hidd_collection_usage(hidd_collections[i], &usage_page, &usage);
        printf("At collection #%d, Found usage page %d, usage %d\n",i, usage_page, usage);	// ***TEMP***
        if (startcoll || ((usage_page == pagewanted) && (usage == usagewanted)))
		{	//	Device has capabilities of interest.
			if (startcoll == 0) m_collection = hidd_collections[i];	// this is the main collection
	       	int stat =  hidd_get_report_instance(hidd_collections[i], 0, HID_INPUT_REPORT,
                                             &m_reportinstance);
 	      	if (stat == EOK) 											// if valid
			{	////printf("Got report instance.\n");					// ***TEMP***
				stat =  hidd_report_attach( connection, m_device, m_reportinstance,
                                            0, 0 , &m_report );
   				if (stat == EOK) 										// success
   			    {	////m_collection = hidd_collections[i];		// this is the collection of interest
   			    	////printf("Attached.\n");								// ***TEMP***
   			    	return(EOK); 		    	
   			    }
   			}
   		}
   		//	Recurse downward and keep looking
   		int stat = report_attach_if_interested(pagewanted, usagewanted, hidd_collections[i]);
   		if (stat == EOK) return(EOK);								// find
    }
 	return(ENODEV);														// didn't find anything to attach to       
}
#ifdef OBSOLETE
//
//	report_attach_if_interested  -- attach to this device if it is one we want
//
//	For example, to attach to a joystick, ask for HIDD_PAGE_DESKTOP, HIDD_USAGE_JOYSTICK
//
int HIDDdevice::report_attach_if_interested(uint16_t pagewanted, uint16_t usagewanted)
{
    //	Detach from any previous reports
    report_detach();
    m_collection = 0;													// no current collection
    // Get root level HID collections
    uint16_t num_col;
    struct hidd_connection* connection = GetOwner().GetConnection();
    if (!connection) return(ENOTCONN);						// no connection to server
    struct hidd_collection **hidd_collections = 0;
    int stat = hidd_get_collections(m_device, NULL, &hidd_collections, &num_col);
    if (stat != EOK) return(stat);									// fails
    if (num_col == 0) return(ENOTCONN);					// no collections
    assert(hidd_collections);										// should now have a table
	//	Examine all collections for this device instance.
    for(unsigned int i = 0; i < num_col; i++)
    {	uint16_t usage_page, usage;
        hidd_collection_usage(hidd_collections[i], &usage_page, &usage);
        if (usage_page != pagewanted) continue;
        if (usage != usagewanted) continue;
        printf("At collection #%d, Found usage page %d, usage %d\n",i, usage_page, usage);	// ***TEMP***
		//	Device has capabilities of interest.
		// ***UNSURE ABOUT [0] BELOW
       	int stat =  hidd_get_report_instance(hidd_collections[0], 0, HID_INPUT_REPORT,
                                             &m_reportinstance);
       	if (stat == EOK) 											// if valid
		{	stat =  hidd_report_attach( connection, m_device, m_reportinstance,
                                            0, 0 , &m_report );
   		    if (stat == EOK) 										// success
   		    {	m_collection = hidd_collections[0];		// this is the collection of interest
   		    	assert(i == 0);										// ***TEMP TEST***
   		    	return(EOK);		    	
   		    }	
   		  
   		} else printf("ERROR getting report instance: %s\n", strerror(stat)); // ***TEMP***
		//	Try next level down
		uint16_t num_mcol;
		struct hidd_collection** hidd_mcollections = 0;

        stat = hidd_get_collections( NULL, hidd_collections[i], &hidd_mcollections, &num_mcol);
        if (stat != EOK) continue;									// 
       	if (num_mcol == 0) continue;							// no such device
        assert(hidd_mcollections);								// should have a table
        stat =  hidd_get_report_instance( hidd_mcollections[0], 0 ,
                        HID_INPUT_REPORT, &m_reportinstance );
        if (stat == EOK)												// if valid
        {	stat = hidd_report_attach( connection, m_device, m_reportinstance,
                                            0, 0 , &m_report );
   		     if (stat == EOK) 												// success
   		     {	m_collection = hidd_mcollections[0];			// only look at first collection?
   		     	return(EOK);
   		     }
   		}
 	}
 	return(ENODEV);														// didn't find anything to attach to
}
#endif // OBSOLETE       
//
//	Dump functions
//
//	Dump -- for report properties
//
void HIDDdevice::Dump(const hidd_report_props_t& props)
{
	printf("  Report properties for page %d, report ID %d, usages %d .. %d:\n",
		props.usage_page, props.report_id,props.usage_min, props.usage_max);
	printf("    Data properties: 0x%08x\n", props.data_properties);
	printf("    Unit exponent: %d  Unit: %d\n", props.unit_exponent, props.unit);
	printf("    Logical min/max: %d .. %d\n", props.logical_min, props.logical_max);
	printf("    Physical min/max: %d .. %d\n",props.physical_min, props.physical_max);
	printf("    String min/max: %d .. %d\n", props.string_min, props.string_max);
	printf("    Designator min/max: %d .. %d\n", props.designator_min, props.designator_max);
}
//
//	get_report_props  -- get all report properties for a report
//
int HIDDdevice::get_report_props(std::valarray<hidd_report_props_t>& allprops)
{	uint16_t cnt = 0;													// number of properties
	int stat = hidd_get_num_props(m_reportinstance, &cnt);	// get count of properties
	if (stat != EOK) return(stat);									// fails
	allprops.resize(cnt);												// resize properties array
	stat = hidd_get_report_props(m_reportinstance, &allprops[0], &cnt);	// get the properties
	return(stat);															// return result
}
//
//	find_report_prop  -- find property set for a specific page/usage pair (i.e. a control or button)
//
int HIDDdevice::get_report_prop(const std::valarray<hidd_report_props_t>& allprops,
	uint16_t usage_page, uint32_t usage, hidd_report_props_t& prop)
{	prop.physical_min = prop.physical_max = 0;		// clear content
	for (unsigned int i=0; i<allprops.size(); i++)			// search all props
	{	if (allprops[i].usage_page != usage_page) continue;	// wrong page
		if (usage < allprops[i].usage_min) continue;		// wrong usage
		if (usage > allprops[i].usage_max) continue;	// wrong usage
		prop = allprops[i];											// return prop set
		return(EOK);														// match - have props
	}
	return(ENODEV);
} 
//
//	DumpReportProps  -- dump for our device
//
void HIDDdevice::DumpReportProps()
{	std:: valarray<hidd_report_props_t> allprops;
	int stat = get_report_props(allprops);
	if (stat != EOK) { printf("Error getting report properties: %s\n", strerror(stat)); return; }
	printf("Device properties:\n");
	for (unsigned int i=0; i<allprops.size(); i++)
	{	Dump(allprops[i]);	}
}
//
//	makemask -- make a mask with enough 1 bits to hold the indicated value
//
//	This is a workaround for a bug
//
inline uint16_t makemask(uint16_t lim)
{
	uint16_t mask = 0;												// start with empty mask
	while ((mask & lim) != lim)									// until mask covers lim
	{	mask = (mask << 1) | 1;	}								// add 1 bits to mask
	return(mask);
}
//
//	ScaleValue -- scale a value based on property info.
//	
//	Scales into the range 0..1, based on physical limits
//	Most devices center at 0.5.
//
float HIDDdevice::ScaleValue(const hidd_report_props_t& prop, uint16_t value)
{	
	if (prop.physical_max == 0) return(0);								// no props, return zero
	//	We must mask off meaningless high bits. Some devices return junk in high bits of values.
	//	But different devices have different data lengths, so we must check.
	value &= makemask(prop.physical_max);						// mask off
	return(float(value)/float(prop.physical_max));					// scale into 0..1.
}
          


