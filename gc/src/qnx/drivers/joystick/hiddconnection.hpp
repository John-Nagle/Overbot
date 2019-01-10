//
//	hiddconnection.hpp  -- encapsulation for QNX Human Interface Device Driver access
//
//	John Nagle
//	Team Overbot
//	June, 2004.
//
#ifndef HIDDCONNECTION_HPP
#define HIDDCONNECTION_HPP
//
#include <sys/hiddi.h>
#include <list>
#include <valarray>
#include <map>
#include "mutexlock.h"
//
class HIDDdevice;														// forward
//
//	class HIDDconnection -- one connection to an HIDD server
//
//	Abstract class - subclass for an application.
//
class HIDDconnection {
private:
    struct hidd_connection* m_connection;				// connection handle
    std::list<HIDDdevice*> m_devices;						// all associated devices
	ost::Mutex m_lock;													// lock for above
private:
	static std::list<HIDDconnection*> m_registry;		// list of all HIDDconnection objects
	static ost::Mutex m_registrylock;							// lock for above
public:
	HIDDconnection();
	virtual ~HIDDconnection();
	int Connect();														// connect to server
	int Disconnect();
	int get_device_instance(uint16_t devno, hidd_device_instance_t** instance);
	hidd_connection* GetConnection()  { return(m_connection); }
	void Dump(hidd_device_instance* deviceinstance);
	void Dump();															// dump all devices
protected:
	//	Callbacks at the object level - concurrent, from another thread
    virtual void onInsertion(hidd_device_instance_t* instance) = 0;
private:
    void onRemoval(hidd_device_instance_t* instance);
    void onHidReport(struct hidd_report *handle, void *report_data,
                            _uint32 report_len, _uint32 flags);
   
	//	Statics. The system callbacks are static, and we have to dispatch
	//	them to the correct object.
private:
	//	Registry of connection objects, per program
	static HIDDconnection* FindByConnection(struct hidd_connection* connection);
	static void Register(HIDDconnection* conn);
	static void Unregister(HIDDconnection* conn);
	//	Registry of devices, per connection
	HIDDdevice* FindByReport(struct hidd_report* report);
	HIDDdevice* FindByDevice(struct hidd_device_instance* device);
public:
	void Register(HIDDdevice* dev);
	void Unregister(HIDDdevice* dev);
private:
	//	Callbacks - concurrent, from another thread
    static void onInsertion(struct hidd_connection *connection,
                            hidd_device_instance_t *instance);
    static void onRemoval(struct hidd_connection *connection,
                          hidd_device_instance_t *instance);
    static void onHidReport(struct hidd_connection *connection,
                            struct hidd_report *handle, void *report_data,
                            _uint32 report_len, _uint32 flags, void *user);

};
//
//	class HIDDdevice -- one device attached to an HIDD connection
//
class HIDDdevice {
private:
	HIDDconnection& m_conn;								// the attached connection
	struct hidd_report* m_report;							// the report we are receiving
	struct hidd_report_instance* m_reportinstance;	// the report instance
	struct hidd_device_instance* m_device;			// the device of interest
	struct hidd_collection* m_collection;					// the usage collection of interest
public:
	HIDDdevice::HIDDdevice(HIDDconnection& owner, hidd_device_instance_t* deviceinstance);
	virtual ~HIDDdevice();
	HIDDconnection& GetOwner() { return(m_conn); }
	struct hidd_report* GetReport() { return(m_report); }
	struct hidd_report_instance* GetReportInstance() { return(m_reportinstance); }
	void SetDevice(struct hidd_device_instance* device) { report_detach(); m_device = device; }
	struct hidd_device_instance* GetDevice() { return(m_device); }
	struct hidd_collection* GetCollection() { return(m_collection); }
	int report_attach(hidd_device_instance_t* deviceinstance, hidd_report_instance* reportinstance, uint32_t flags = 0);				
	int report_detach();											// detach from any existing reports
	int report_attach_if_interested(uint16_t pagewanted, uint16_t usagewanted, struct hidd_collection* startcoll = 0);
	//	Callbacks - concurrent, from another thread
	virtual void onInsertion() {}										// device has apppeared
	virtual void onRemoval() {}										// device has disappeared		
	virtual void onHidReport(struct hidd_report* handle, void* report_data,
                            _uint32 report_len, _uint32 flags) = 0;
    //	Utility functions
    void Dump(const hidd_report_props_t& props);
	void DumpReportProps();
    int get_report_props(std::valarray<hidd_report_props_t>& allprops);
	int get_report_prop(const std::valarray<hidd_report_props_t>& allprops,
		uint16_t usage_page, uint32_t usage, hidd_report_props_t& prop);
	static float ScaleValue(const hidd_report_props_t& prop, uint16_t value); // utility

};

#endif // HIDDCONNECTION_HPP
