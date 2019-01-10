//
//	betabrite.h -- class to interface with BetaBrite display
//
//	J. Nagle
//	Team Overbot
//	November, 2003
//
//	Based on the SourceForge project "betabrite"
/*
   betabrite.h - common declarations for betabrite main program
 
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
#ifndef BETABRITE_H
#define BETABRITE_H
//
//	class BetaBrite  -- one BetaBrite display
//
class BetaBrite {
private:
	int m_fd;													// file descriptor
	char m_betafile;										// which "file" in the BetaBrite (A, B or C)
	const char* m_holdspeed;						// which "hold speed" in the BetaBrite
	const char* m_color;									// which "color" in the BetaBrite
	const char* m_displaymode;					// which "display mode" in the BetaBrite
private:
	int setupserial();										// set up the serial port
public:
	BetaBrite();												// constructor
	virtual ~BetaBrite() { close(); }				// destructor
	int open(const char* serial_port);
	void close();												// close port
	int display(const char* msg);					// display a message
	void setcolor(const char* color);				// set BetaBrite color (BETABRITE_COLOR_AMBER, etc.)
	void setmode(const char* mode);			// set BetaBrite mode (BETABRITE_HOLD, etc.)
	void setholdspeed(const char* speed);	// set BetaBrite hold speed (BETABRITE_SPEED_HIGH, etc.)
};
/* these #defines make it easer to deal with the codes used to 'program' the
   display. The PDF document mentioned in the README provides much more detail
   on the protocol itself. */

/* COMMAND CODES (for documentation only) */
/*
#define SOH "\x01"
#define STX "\x02"
#define ETX "\x03"
#define EOT "\x04"
#define ESCAPE "\x1B"
*/
/* ATTRIBUTES */

#define BETABRITE_DOUBLE_HEIGHT_OFF "\x05\x30"
#define BETABRITE_DOUBLE_HEIGHT_ON "\x05\x31"
#define BETABRITE_TRUE_DESCENDERS_OFF "\x06\x30"
#define BETABRITE_TRUE_DESCENDERS_ON "\x06\x31"
#define BETABRITE_WIDE_CHARS_OFF "\x11"
#define BETABRITE_WIDE_CHARS_ON "\x12"

#define BETABRITE_CALL_DATE_0 "\x0B\x30"
#define BETABRITE_CALL_DATE_1 "\x0B\x31"
#define BETABRITE_CALL_DATE_2 "\x0B\x32"
#define BETABRITE_CALL_DATE_3 "\x0B\x33"
#define BETABRITE_CALL_DATE_4 "\x0B\x34"
#define BETABRITE_CALL_DATE_5 "\x0B\x35"
#define BETABRITE_CALL_DATE_6 "\x0B\x36"
#define BETABRITE_CALL_DATE_7 "\x0B\x37"
#define BETABRITE_CALL_DATE_8 "\x0B\x38"
#define BETABRITE_CALL_DATE_9 "\x0B\x39"
#define BETABRITE_CALL_TIME "\x13"

#define BETABRITE_NEW_PAGE "\x0C"
#define BETABRITE_NEW_LINE "\x0D"

#define BETABRITE_CALL_STRING_X "\x10"
#define BETABRITE_CALL_DOTS_PICTURE_X "\x14"

#define BETABRITE_NO_HOLD_SPEED "\x09"
#define BETABRITE_SPEED_SLOWEST "\x15"
#define BETABRITE_SPEED_LOW "\x16"
#define BETABRITE_SPEED_MEDIUM "\x17"
#define BETABRITE_SPEED_HIGH "\x18"
#define BETABRITE_SPEED_FASTEST "\x19"

#define BETABRITE_COLOR_RED "\x1C\x31"
#define BETABRITE_COLOR_GREEN "\x1C\x32"
#define BETABRITE_COLOR_AMBER "\x1C\x33"
#define BETABRITE_COLOR_DIMRED "\x1C\x34"
#define BETABRITE_COLOR_DIMGREEN "\x1C\x35"
#define BETABRITE_COLOR_BROWN "\x1C\x36"
#define BETABRITE_COLOR_ORANGE "\x1C\x37"
#define BETABRITE_COLOR_YELLOW "\x1C\x38"
#define BETABRITE_COLOR_RAINBOW1 "\x1C\x39"
#define BETABRITE_COLOR_RAINBOW2 "\x1C\x41"
#define BETABRITE_COLOR_MIX "\x1C\x42"
#define BETABRITE_COLOR_AUTO "\x1C\x43"

#define BETABRITE_WIDTH_PROPORTIONAL "\x1E\x30"
#define BETABRITE_WIDTH_FIXEX "\x1E\x31"

#define BETABRITE_TEMPERATURE_CELSIUS "\x08\x1C"
#define BETABRITE_TEMPERATURE_FARENHEIT "\x08\x1D"

#define BETABRITE_CALL_COUNTER_1 "\x08\x7A"
#define BETABRITE_CALL_COUNTER_2 "\x08\x7B"
#define BETABRITE_CALL_COUNTER_3 "\x08\x7C"
#define BETABRITE_CALL_COUNTER_4 "\x08\x7D"
#define BETABRITE_CALL_COUNTER_5 "\x08\x7E"

/* POSITIONS */

#define BETABRITE_MIDDLE "\x20"
#define BETABRITE_TOP "\x22"
#define BETABRITE_BOTTOM "\x26"
#define BETABRITE_FILL "\x30"

/* MODES */

#define BETABRITE_ROTATE "\x61"
#define BETABRITE_HOLD "\x62"
#define BETABRITE_FLASH "\x63"
#define BETABRITE_ROLL_UP "\x65"
#define BETABRITE_ROLL_DOWN "\x66"
#define BETABRITE_ROLL_LEFT "\x67"
#define BETABRITE_ROLL_RIGHT "\x68"
#define BETABRITE_WIPE_UP "\x69"
#define BETABRITE_WIPE_DOWN "\x6a"
#define BETABRITE_WIPE_LEFT "\x6b"
#define BETABRITE_WIPE_RIGHT "\x6c"
#define BETABRITE_SCROLL "\x6d"
#define BETABRITE_AUTOMODE "\x6f"
#define BETABRITE_ROLL_IN "\x70"
#define BETABRITE_ROLL_OUT "\x71"
#define BETABRITE_WIPE_IN "\x72"
#define BETABRITE_WIPE_OUT "\x73"
#define BETABRITE_COMPRESSED_ROTATE "\x74"
#define BETABRITE_TWINKLE "\x6e\x30"
#define BETABRITE_SPARKLE "\x6e\x31"
#define BETABRITE_SNOW "\x6e\x32"
#define BETABRITE_INTERLOCK "\x6e\x33"
#define BETABRITE_SWITCH "\x6e\0x34"
#define BETABRITE_SLIDE "\x63\0x35"
#define BETABRITE_SPRAY "\x63\0x36"
#define BETABRITE_STARBURST "\x63\0x37"
#define BETABRITE_WELCOME "\x63\0x38"
#define BETABRITE_SLOT_MACHINE "\x63\0x39"
//
//	Additional info
//
#define BETABRITE_MAX_CHARS (16)			// max chars that will fit in the default font
//
#endif // BETABRITE_H
