//
//	voradread.h  --  support for reading from a FireWire vorad using our driver
//
//	John Nagle
//	Team Overbot
//	October, 2003
//
#ifndef VORADREAD_H
#define VORADREAD_H

#include "ablibs.h"
#include <memory>
#include "messaging.h"
#include "remotemessaging.h"
class VoradTargetItem;

//
//	class VoradRead  --  read from the VORAD server
//
class VoradRead {
private:
	RemoteMsgClientPort	m_voradserver;					// connection to the server
private:
public:
	int reset();
	void video_port_draw( PtWidget_t *widget, PhTile_t *damage );	// redraw
	VoradRead();											// constructor
	~VoradRead();										// destructor
private:
	void drawtargetitem(PtWidget_t* widget, const VoradTargetItem& target);
	int setup();												// initial setup
};
#endif // VORADREAD_H