//
// SocketRx.cpp: implementation of the CSocketRx class.
//
//////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "protocolrx.h"
#include "telnetcomport.h"
#include "socketrx.h"
//
//	Constants
//
const size_t k_bufsize = 2048;												// at least one Ethernet packet worth of serial data
//
//	Functions called from outside the thread
//
CSocketRx::~CSocketRx()
{
    m_forceexit = true;															// force exit. Parent destructor will wait for join
}
int CSocketRx::start()
{	ost::MutexLock lok(m_lock);												// lock the thread
	if (isrunning()) return(1);													// fails if running
	m_forceexit = false;															// not requesting an exit
	return(create());																// create the thread
}
//
//	stop -- stop thread, and wait for it to stop
//
//	We set the force_exit flag, which should cause the thread to stop shortly.
//
int CSocketRx::stop()																
{
	ost::MutexLock lok(m_lock);												// lock the thread against outside access
	m_forceexit = true;															// force an exit
	if (!isrunning()) return(0);													// OK if not running
	return(join());																	// join with the thread
}
//	
//	Functions called from inside the thread
//
//	
//	run  -- read thread
//
//	Reads from socket, sends data to destination.
//
//	Efficiency matters here.
//
void CSocketRx::run()
{
    while(1)
    {
    	uint8_t inbuf[k_bufsize];												// raw input from socket
   	 	uint8_t outbuf[k_bufsize];												// output after escape handling
    	uint32_t outbufpos = 0;												// no output yet
    	if (m_forceexit) return;													// if closing, done
        int nRet = recv(getowner().getsocket(),inbuf,sizeof(inbuf),0);	// wait for input
        if ( nRet < 0)
        {
        	perror("Error reading from socket");							// fails
        	return;																		// failed
        }
		if (nRet == 0)																// should not happen. recv should block.
		{ 	sleep(1);																	// avoid looping at high priority
			printf("Read zero bytes from socket");						// sometimes happens at startup
			continue;
		}
		//	Have data.  Process it.
		//	Speed matters here.
		////printf("Read %d bytes.\n", nRet);									//***TEMP***
		for (int i=0; i<nRet; i++)
		{	uint8_t ch = inbuf[i];													// incoming char
			bool isdata = m_protocol.TelnetProtocol(getowner().getsocket(),ch);			// send to protocol handler (an inline fn)
			if (isdata)																	// if ordinary data, no special handling
			{	if (outbufpos >= sizeof(outbuf))							// if buffer full
				{	deliver(outbuf,outbufpos);								// deliver it
					outbufpos = 0;													// buffer now empty
				}
				outbuf[outbufpos++] = ch;									// append to buffer
			}
		}
		//	Deliver accumulated data, if any
		if (outbufpos >= 0)														// if buffer not empty
		{	deliver(outbuf,outbufpos);										// deliver it
			outbufpos = 0;															// buffer now empty
		}
    }
}
//
//	deliver -- deliver to owner
//
void CSocketRx::deliver(const uint8_t msg[], uint32_t len)
{	////printf("CSocketRx::deliver\n");	// ***TEMP***
	if (m_forceexit) return;														// quitting, no more deliveries
	getowner().deliver(msg,len); }											// deliver to owner
//
//	setverbose -- set verbose mode for debug
//
void CSocketRx::setverbose(bool mode)
{	m_protocol.setverbose(mode); }
//
//	getbaudrate -- get baud rate from protocol
//
uint32_t CSocketRx::getbaudrate()
{	ost::MutexLock lok(m_lock);												// lock for transaction
	return(m_protocol.getbaudrate());									// return the baud rate
}
//
//	clearbaudrate -- get baud rate from protocol
//
void CSocketRx::clearbaudrate()
{	ost::MutexLock lok(m_lock);												// lock for transaction
	m_protocol.clearbaudrate();												// return the baud rate
}
