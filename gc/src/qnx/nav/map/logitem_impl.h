//
//	LogItem_impl.h  -- logging information for NewSteer / Overbot maps
//
//	John Nagle
//	Team Overbot
//	January, 2004
//
//
//
//	Copyright 2005 by John Nagle
//	999 Woodland Avenue
//	Menlo Park, CA  94025
//
//	This program is free software; you can redistribute it and/or modify
//	it under the terms of the GNU General Public License as published by
//	the Free Software Foundation; either version 2 of the License, or
//	(at your option) any later version.

//	This program is distributed in the hope that it will be useful,
//	but WITHOUT ANY WARRANTY; without even the implied warranty of
//	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//	GNU General Public License for more details.

//	You should have received a copy of the GNU General Public License
//	along with this program; if not, write to the Free Software
//	Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//
#ifndef LOGITEM_IMPL_H
#define LOGITEM_IMPL_H
#include <string>
#include <inttypes.h>
#include <stdio.h>
//
//	class LogMarshall implementation
//
//
//	additem  --  add an item to a stream
//
template<class T> void LogMarshall::additem(const T& item, LogItem::Type_e type)
{	if (!iswriteable()) return;						// ignore if not open
	assert(sizeof(item) <= 255);				// length byte limitation. 	
	size_t size = sizeof(item);						// size of item
	put(uint8_t(size));									// add length byte
	put(uint8_t(type));									// add type byte
	write((uint8_t*)&item, size);					// add data
}
//
//	getitem -- get next item
//
inline bool LogMarshall::getitem(LogItem& item, LogItem::Type_e& type)
{	
	if (!valid())
	{	if (isverbose()) 
		{	printf("LogMarshall::getitem - file not in valid state.\n"); fflush(stdout); }
		return(false);									// EOF or error
	}
	uint8_t size;											// size byte
	uint8_t typebyte;									// type byte
	get(size);												// get next byte - size
	get(typebyte);										// get next byte - type
	type = LogItem::Type_e(typebyte);		// convert type
	if (size > sizeof(item)) return(false);		// oversize item or out of sync
	if (size == 0) return(false);					// undersize item
	int stat = read((uint8_t*)&item, size);	// read item
	if (isverbose())
	{	printf("LogMarshall::getitem: stat = %d, size = %d, type = %d\n",stat,size,type); fflush(stdout); }
	return(stat == size);								// size must match
}
//
//	class LogFile implementation
//
//	
//	open -- open for read or write
//
inline int LogFile::open(const char* filename, bool writing)
{	close();														// close if open
	m_fd = fopen(filename, writing ? "w" : "r" );	// create/open file
	if (m_fd == NULL) return(-1);					// fail
	m_writeable = writing;								// set writeable
	return(0);													// success
}
//
//	close -- close if open
//
inline void LogFile::close()
{	if (m_fd)
	{	fclose(m_fd);											// close
		m_fd = NULL;										// close if open
		m_writeable = false;								// not writeable
	}
}
//
//	valid -- true if stream in valid state
//
inline bool LogFile::valid()												
{	if (!m_fd) return(false);								// not open, fails	
	if (ferror(m_fd)) printf("Log stream error.\n");	// ***TEMP***
	if (feof(m_fd)) printf("Log stream EOF.\n");
	return(ferror(m_fd) == 0 && feof(m_fd) == 0); 
}
//
//	write -- write requested bytes
//
inline void LogFile::write(const uint8_t buf[], size_t size)
{	fwrite(buf,1,size,m_fd); }
//
//	read -- read requested bytes
//
inline int LogFile::read(uint8_t buf[], size_t size)				// read requested bytes
{	int stat = fread(buf,1,size,m_fd); 
	if (isverbose())																// debug
	{	long pos = ftell(m_fd);												// get file position
		printf("Read %d bytes, now at byte %ld\n", stat, pos); 
		fflush(stdout);
	}
	return(stat);																	// return byte count
}
//
//	flush  -- flush log to current point
//
inline void LogFile::flush()
{	if (!m_fd) return;		
	fflush(m_fd);
}	 


#endif // LOGITEM_IMPL_H