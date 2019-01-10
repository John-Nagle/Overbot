//
//    mapcell.h -- data structure for single cells of the terrain map.
//
//	Preliminary version by Achut Reddy.
//
//	John Nagle
//	Team Overbot
//	January, 2004
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
#ifndef MAPCELL_H
#define MAPCELL_H

#include <inttypes.h>
#include <vector>
//
//	Constants
//
const float k_better_range_ratio = 0.5;										// must be much closer to override previous "bad" 
const int k_roughness_move_threshold = 5;								// units of cm, must move a few cm to add to roughness
const int k_approxcellsizecm = 20;											// must move at least this much for roughness update
//
inline bool norangevalue(uint16_t range)		{ return(range > 8182);	}	// special values from SICK unit
//	
//	MapPoint -- defines a Cartesian point
//
struct MapPoint {
    double	x;	// defined as extension in the East direction
    double	y;	// defined as extension in the North direction
};

//
// CellData - data associated with each cell of the map
//
//	There are a million of these. Size matters.
//
class CellData
{
public:
    // CellType - type of map cell
    enum CellType {
        UNKNOWN = 0,			// unexplored terrain (grey)
        CLEAR,						// perfectly passable (green)
        POSSIBLE,              	// obstacles, may or not be passable (yellow)
        NOGO						// totally impassable or out of bounds (red)
    };

    CellType	m_type : 3;	// cell type
    bool m_valid : 1;			// there is valid data in this cell
    ////bool m_offcourse : 1;	// true if all of cell is off the course
    ////bool	m_moving : 1;		// cell contains a moving obstacle (from radar)

    uint8_t	m_roughness;	// an aggregate measure of roughness
   										// based on triangles with a point on this cell
	uint8_t	m_datapoints;	// triangles contributing to this cell

	uint16_t	m_typerange;		// range at which cell changed type 
										// info from closer ranges overrides longer ones
	uint16_t 	m_roughnessrange;	// last range at which seen
	uint32_t	m_cyclestamp;	// update cycle timestamp
    float		m_minelev;      	 	// minimum elevation seen
    float		m_maxelev;			// maximum elevation seen
public:
	void clear()
	{	m_valid = false;	}
	bool passable() const { return(m_valid && (m_type == CLEAR)); }
	bool possible() const { return(m_valid && (m_type == POSSIBLE)); }
	bool unknown() const { return((!m_valid) || (m_type == UNKNOWN)); }
	uint8_t roughness() const { return(m_roughness); }
	float avgelev() const { return((m_minelev + m_maxelev) * 0.5); }	// average elevation
	bool update(CellType newtype, bool sweeping, uint16_t newrange, uint8_t roughness, float elev, uint32_t cyclestamp, 
		uint8_t clearthreshold, uint8_t nogothreshold, uint32_t ancientstamp);
	bool updateroughness(bool sweeping, uint16_t newrange, uint8_t roughness, float elev, uint32_t cyclestamp, 
		uint8_t clearthreshold, uint8_t nogothreshold);
	bool updatetype(CellType newtype, uint16_t newrange, float elev, uint32_t cyclestamp, uint32_t ancientstamp);

};
//
//	updatetype  -- update a single cell's type.
//
//	This does all the real work for a cell
//
inline bool CellData::updatetype(CellType newtype, uint16_t newrange, float elev, uint32_t cyclestamp, uint32_t ancientstamp)
{	if (!m_valid || m_cyclestamp < ancientstamp)						// if cell is empty or really old
	{	m_type = newtype;															// just stamp with new data
		m_typerange = newrange;
		m_roughnessrange = newrange;												// first time through
		m_minelev = elev;															// set elevation
		m_maxelev = elev;
		m_cyclestamp = cyclestamp;
		m_roughness = 0;
		m_datapoints = 1;
		m_valid = true;
		return(true);																		// an update has occured
	}
	//	Cell is nonempty.  Must combine information
	//	Update basic cell evaluation
	bool changed = false;
	if (newtype > int(m_type))													// if new type is worse
	{	m_type = newtype;															// accept worse type
		m_typerange = newrange;												// use new range
		changed = true;
	}
	else if (newtype < int(m_type))											// new type is better
	{	if (newrange < k_better_range_ratio*m_typerange)			// if significantly closer range
		{	m_type = newtype;														// accept more favorable evaluation
			m_typerange = newrange;											// use new, closer range
			changed = true;
		}
	}
	return(changed);																	// an update has occured
}
//
//	updateroughness  -- update a single cell's roughness and elevation values
//
//	This updates the non-local information for a cell
//
inline bool CellData::updateroughness(bool sweeping, uint16_t newrange, uint8_t roughness, float elev, uint32_t cyclestamp, 
	uint8_t clearthreshold, uint8_t nogothreshold)
{	bool changed = false;
	if (!m_valid)																			// if not yet valid
	{	changed = updatetype(UNKNOWN, newrange, elev, cyclestamp, 0);	// make valid
	}
	m_minelev = std::min(m_minelev, elev);									// update max and min elevations
	m_maxelev = std::max(m_maxelev, elev);								// update max and min elevations
	m_datapoints++;																		// count datapoint contributing to this cell
	//	"roughness" is largest elevation change in any triangle touching this point.
	//	Also, although this is something of a hack, if we are getting closer to something,
	//	we ADD to the roughness. This detects vertical surfaces as we approach them.
	if (sweeping)																			// sweep mode update, vehicle stationary, good elevation data
	{	float elevdiff = m_maxelev - m_minelev;								// roughness is just elevation difference
		m_roughness = uint16_t(std::min(elevdiff*100, 255.0f));			// convert to cm, max at 255				
		m_roughness = std::max(m_roughness, roughness);			// also consider single-triangle roughness
	} else {																					// non sweep update, elevation data unreliable
		if ((int(newrange) < int(m_roughnessrange)-k_approxcellsizecm))			// if moved to a significantly closer position
		{	uint16_t r = m_roughness + roughness;							// sum roughness
			roughness = std::min(uint16_t(255),r);								// clamp at 255
			m_roughnessrange = newrange;										// last range accumulated into roughness
		}
		m_roughness = std::max(m_roughness, roughness);			// use maximum roughness
	}
	m_cyclestamp = cyclestamp;												// update cycle stamp
	//	Cumulative roughness update
	if (m_roughness > nogothreshold)										// if too rough, force NOGO
	{	changed |= updatetype(NOGO, newrange, elev, cyclestamp,0);
	}
	else if (m_roughness > clearthreshold)
	{	changed |= updatetype(POSSIBLE, newrange, elev, cyclestamp,0); }
	return(changed);																	// no change										
}
//
//	update  -- both local and global info
//
inline bool CellData::update(CellType newtype, bool sweeping, uint16_t newrange, uint8_t roughness, float elev, uint32_t cyclestamp, 
	uint8_t clearthreshold, uint8_t nogothreshold, uint32_t ancientstamp)
{	bool changed = false;
	if (newtype != UNKNOWN)													// if requesting a type change
	{	changed |= updatetype(newtype, newrange, elev, cyclestamp, ancientstamp);	}	// update based on local info
	changed |= updateroughness(sweeping, newrange, roughness, elev, cyclestamp, clearthreshold, nogothreshold);	// update based on cumulative info
	return(changed);
}

#endif // MAPCELL_H
