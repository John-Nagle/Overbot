//
//	lidarsocket.h  -- LIDAR socket I/O
//
//	Usable from multiple threads
//
//	Designed not to generate SIGPIPE signals
//
//	J.	Nagle
//	Team Overbot
//	August, 2005
//
#ifndef LIDARSOCKET_H
#define LIDARSOCKET_H
#include "mutexlock.h"
//
//	class LidarSocket  -- one socket for talking to a serial to Ethernet converter
//
class LidarSocket {
private:
	int m_sock;														// file descriptor
	ost::Mutex m_reading;										// read in progress
	ost::Mutex m_writing;										// write in progress
	bool m_closing;													// close in progress
	bool m_verbose;												// true if verbose
public:
	LidarSocket()													// constructor
	:m_sock(-1), m_closing(false), m_verbose(false)					
	{}
	~LidarSocket();													// destructor
	//	usual open/close/read/write
	int open(const char *hostname, unsigned short portnum);
	void close();
	int read_data(char *buf, int n);
	int write_data(const char *buf, int n);
	//	Access
	bool isopen() const;											// true if open
	//	Debug
	void set_verbose(bool verbose) { m_verbose = verbose; }
};
#endif // LIDARSOCKET_H
