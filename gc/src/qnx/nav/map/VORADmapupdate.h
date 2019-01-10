//
//	VORADmapupdate.h  --  map updating based on sensor data
//
//	Functions specific to EATON VORAD radar
//	
//	VORAD data is usually not sent in for map updating, because the azimuth angles from the
//	VORAD aren't that accurate. But the code to support it is here and works.
//
//	John Nagle
//	Team Overbot
//	February 2005
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
#ifndef VORADMAPUPDATE_H
#define VORADMAPUPDATE_H
//
class MapServer;																	// forward
class VoradServerMsgVDTG;													// forward
//
//	class VORADmapUpdater  --  updater for one VORAD unit
//
class VORADmapUpdater {
private:
	MapServer& m_owner;														// owning map server
	vec3 m_offset;																	// position of VORAD relative to GPS antenna
public:
	VORADmapUpdater(MapServer& owner, const vec3& voradoffset);
	//	Queue of lines waiting for a useful GPS update
	std::queue< VoradServerMsgVDTG*> m_linequeue;						// lines waiting to be processed
	std::queue< VoradServerMsgVDTG*> m_emptyqueue;					// empty line buffers
	void handleRadarData(const VoradServerMsgVDTG& msg);
	bool getVerbose() const;
private:
	void handlePosedData(const VoradServerMsgVDTG& msg, const mat4& vehpose);
	void handleRadarTarget(const VoradTargetItem& target, const mat4& vehpose);
};
#endif //  VORADMAPUPDATE_H
