//
//	betabrite.cpp  -- BetaBrite interface class
//
//	J. Nagle
//	Team Overbot
//	November, 2003
//	
//	Based on the project "betabrite" from SourceForge.
//	Original copyright notice appears below.	
//
/*
   betabrite.c -- command line utility to display a message on a BetaBrite LED sign
 
   Copyright (C) 2002 boB Rudis, bob@rudis.net
 
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2, or (at your option)
   any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307,
   USA
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <assert.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "betabrite.h"

//
//	Implementation
//
//
//	constructor
//
BetaBrite::BetaBrite(): 
		m_fd(-1), m_betafile('A'), m_holdspeed(BETABRITE_NO_HOLD_SPEED),m_color(BETABRITE_COLOR_AMBER),
		m_displaymode(BETABRITE_HOLD)
{}		

//	open -- open and set up the serial port
//
int BetaBrite::open(const char* serial_port)	
{	close();														// close if already open
    m_fd = ::open(serial_port, O_RDWR|O_NOCTTY|O_NDELAY) ;
	if (m_fd < 0)
	{	perror("BetaBrite: Unable to open port") ;
		return(errno);
	}
    fcntl(m_fd,F_SETFL,0);
	return(setupserial());
}
//
//	close -- open and set up the serial port
//
void BetaBrite::close()	
{	if (m_fd >= 0) ::close(m_fd);						// close if open
	m_fd = -1;													// closed
}
//
//	setcolor -- set BetaBrite color (BETABRITE_COLOR_AMBER, etc.)
//
void BetaBrite::setcolor(const char* color)
{	if (!color) return;
	m_color = color;
}
//
//	setmode -- set BetaBrite mode (BETABRITE_HOLD, etc.)
//
void BetaBrite::setmode(const char* mode)
{	if (!mode) return;
	m_displaymode = mode;
}
//
//	setholdspeed -- set BetaBrite speed (BETABRITE_NO_HOLD_SPEED, etc.)
//
void BetaBrite::setholdspeed(const char* speed)
{	if (!speed) return;
	m_holdspeed = speed;
}
//
//	setupserial -- set up the serial port for the BetaBrite
//
int BetaBrite::setupserial()
{
	assert(m_fd >= 0);									// must be open
    struct termios options ;
    tcgetattr(m_fd,&options) ;

    /* 9600 baud */

    cfsetispeed(&options,B9600) ;
    cfsetospeed(&options,B9600) ;

    options.c_cflag |= (CLOCAL|CREAD) ;

    tcsetattr(m_fd,TCSANOW,&options) ;

    /* 7 bits */

    options.c_cflag &= ~CSIZE ;
    options.c_cflag |= CS7 ;

    /* even parity */

    options.c_cflag |= PARENB ;
    options.c_cflag &= ~PARODD ;
    options.c_cflag &= ~CSTOPB ;
    options.c_cflag &= ~CSIZE ;
    options.c_cflag |= CS7 ;

    /* h/w flow */

    //// 	options.c_cflag |= CRTSCTS ;	// not valid under QNX

    options.c_oflag &= ~OPOST;
    options.c_oflag &= ~ONLCR;
    options.c_oflag &= ~OCRNL;

    tcsetattr(m_fd,TCSANOW,&options);
    return(EOK);
}
//
//	display -- display a message
//
//	Uses modes currently in effect
//
int BetaBrite::display(const char* msg)
{
	//	Decide on appropriate mode, given message length
	const char* displaymode = m_displaymode;					// assume requested display mode
	if ((strcmp(displaymode,BETABRITE_HOLD) == 0)				// if HOLD mode
		&& (strlen(msg) > BETABRITE_MAX_CHARS))				// and message is too long for one line 
		{	displaymode = BETABRITE_ROTATE;	}						// scroll horizontally instead
	
    /* Get the BetaBrite's attention */
	const char* null20 = "\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0";
    int n = write(m_fd,null20, sizeof(null20)) ;
    if (n <= 0) { return(errno); }


    /* Start of Header */

    n = write(m_fd,"\001",1) ;
    if (n <= 0) { return(errno); }

    /* Type Code - 'Z' = 'All Signs' */

    n = write(m_fd, "Z",1) ;
    if (n <= 0) { return(errno); }

    /* Sign Address - '00' */

    n = write(m_fd,"00",2) ;
    if (n <= 0) { return(errno); }

    /* Start of Text */

    n = write(m_fd,"\002",1) ;
    if (n <= 0) { return(errno); }

    /* Command Code - 'A' = 'Write TEXT' */

    n = write(m_fd,"A",1) ;
    if (n <= 0) { return(errno); }

    /* File Label - e.g. 'A' */

    n = write(m_fd,&m_betafile,1) ;
    if (n <= 0) { return(errno); }

    /* Command code info: ESCape SPACE 'a' */

    ////n = write(m_fd,"\x1B \x6F",3) ;
    n = write(m_fd,"\x1B ",2) ;
    if (n <= 0) { return(errno); }
    n = write(m_fd,displaymode,strlen(displaymode)) ;
    n = write(m_fd,"\x07",1) ;
    if (n <= 0) { return(errno); }

    /* Set hold speed (I don't trust the betabrite) */

	for (int j=0; j<5; j++)
	{
	    n = write(m_fd,m_holdspeed,strlen(m_holdspeed)) ;
	    if (n <= 0) { return(errno); }
	}

    /* just in case - no wide characters */

    n = write(m_fd,"\x11",1) ;
    if (n <= 0) { return(errno); }

    /* set color of the whole line */

    n = write(m_fd,m_color,strlen(m_color));
    if (n <= 0) { return(errno); }

    /* The data */

    n = write(m_fd,&msg[0],strlen(msg)) ;
    if (n <= 0) { return(errno); }

    /* End of transmission */

    n = write(m_fd,"\004",1) ;
    if (n <= 0) { return(errno); }
    return(EOK);
}


