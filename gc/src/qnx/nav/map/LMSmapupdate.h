//
//	LMSmapupdate.h  --  support for updating the map via a SICK LMS unit
//
//	John Nagle
//	Team Overbot
//	December, 2004
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
#ifndef LMSMAPUPDATE_H
#define LMSMAPUPDATE_H
//
#include <stdint.h>
#include <stdio.h>
#include <queue>
#include "algebra3.h"
#include "lidarserver.h"
#include "gazecontrol.h"
#include "LMStiltcorrect.h"
//
class MapServer;																	// forward
//
//	class LMSmapUpdater  --  updater for one LMS unit
//
class LMSmapUpdater {
private:
	MapServer& m_owner;														// owning map server
	GazeController m_gazecontrol;											// gaze controller
	vec3		m_scanneroffset;													// scanner position relative to GPS
	mat4		m_scanneroffsetransform;									// scanner offset as 4x4 transform matrix.
	//	Queue of lines waiting for a useful GPS update
	std::queue<LidarScanLine*> m_linequeue;						// lines waiting to be processed
	std::queue<LidarScanLine*> m_emptyqueue;					// empty line buffers
	//	Previous item, for line pair
	bool m_prevvalid;																// previous info valid
	LidarScanLine m_prevscanline;										// previous scan line
	mat4	m_prevscannerpose;												// previous scanner pose
	//	Tilt correction history
	LMStiltCorrector m_tiltcorrector;										// the tilt corrector
public:
	LMSmapUpdater(MapServer& owner, const vec3& scanneroffset);	// position relative to GPS
	void LMShandleLidarData(const LidarScanLine& lp);
	int getVerboseLevel() const;												// 3 or more for this 
	void LMSupdateScanlinePair(const LidarScanLine& lp1, const mat4& scannerpose1,
					const LidarScanLine& lp2, const mat4& scannerpose2, uint32_t cyclestamp, bool playback = false);
	mat4 LMStiltPose(float tilt);
	bool LMSvalid() const { return(m_prevvalid);	}				// getting valid scan lines? ***TEMP***
	void resetAncientStamp();												// reset ancient stamp so new data overrides old
	void requestNod();															// request a "nod" - look closer
	void requestSweep();														// request a full sweep. Usually when stopped
	bool gazeistracking();														// true if up and in tracking mode
	bool gazeissweeping();													// true if up and in sweeping  mode 
	LMStiltCorrector& getLMStiltCorrector() {return(m_tiltcorrector);}
	bool LMScalcAverageRange(const LidarScanLine& lp, float& avgrange);

private:
	void LMShandlePosedLidarData(const LidarScanLine& lp, const mat4& vehpose);
	float LMScalcThreatRange(const mat4& scannerpose, float minheight, float radius);
	bool LMSgazeCheck(float tilt, float avgrange, bool rangevalid);

	void LMScalcScanLimits(const mat4& scannerpose, vec2& center, vec2& dir, bool& cuttoleft, bool& needcut);
};
#endif //  LMSMAPUPDATE_H
	
