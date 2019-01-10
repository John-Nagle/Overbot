//
//	ScrollableMap.h  --  a generic scrollable map
//
//	John Nagle
//	Team Overbot
//	December, 2004
//
//	This is used both in building the map from laser rangefinder data and in the program for
//	viewing that data, which is why it was made generic.
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

#ifndef SCROLLABLEMAP_H
#define SCROLLABLEMAP_H
#include <stddef.h>
#include <vector>
#include <assert.h>

//
//	ScrollableMap  -  a generic scrollable map
//
//	A scrollable map is a 2D array of cells, which can be updated and scrolled.
//
template <class CELL, class PARENT> class ScrollableMap
	: public PARENT
{
private:
    std::vector<CELL> m_map;				// the working array of cells
    double m_cellspermeter;					// width represented by a cell
    int		m_dimincells;							// dimension of map in cells
   	int		m_xcenter;								// center of map (cells)
   	int		m_ycenter;								// center of map (cells)
public:
	ScrollableMap(int dimincells, double cellspermeter);						// constructor
	virtual ~ScrollableMap();																// destructor
	void resize(int dimincells, double cellspermeter);							// resize and clear
	//	Access to the map
	double celltocoord(int n)	const													// cell to coordinate conversion
	{	return(n/m_cellspermeter); }													// must calc in floating point
	int coordtocell(double pos)	const												// coordinate to cell conversion
	{
		// This transform is wrong. The zero cell is squeezed.
		// Check 0.5 and -0.5 to see the squeeze effect.
		if(pos<0.0) return(int(pos * m_cellspermeter-0.5));
		return(int(pos * m_cellspermeter+0.5));
	}								// round, to avoid roundoff problems going back and forth	
	bool cellonmap(int x, int y) const;													// true if this cell is on the map
    CELL& at(int ix, int iy);									  								// at - return cell at (x,y), fatal if off map
    const CELL& at(int ix, int iy) const;
    double getcellspermeter() const { return(m_cellspermeter); }		// cells per meter
    int getdimincells() const { return(m_dimincells); }						// map dimension in cells
    int getix() const { return(m_xcenter); }										// map center
    int getiy() const { return(m_ycenter); }
    int getminix() const { return(m_xcenter - m_dimincells/2); }			// access to bounds
	int getminiy() const { return(m_ycenter - m_dimincells/2); }
    int getmaxix() const { return(m_xcenter + (m_dimincells-m_dimincells/2-1)); }
    int getmaxiy() const { return(m_ycenter + (m_dimincells-m_dimincells/2-1)); }
	void setmapcenter(double x, double y);										// set the center of the map, by coords
	void setmapcentercell(int ix, int iy);												// set the center of the map, by cell index
	void clearmap();																			// clear entire map
	//	Compatibility with NewSteer
	void setCellDimensions(double sizeinm)										// cell dimensions
	{	resize(m_dimincells, 1.0/sizeinm);	 }											// resize
	double getCellDimensions() const { return(1.0/m_cellspermeter); } // dimensions of cell in meters

private:
	int cellindex(int x, int y) const;														// get index of cell in map, UNCHECKED
	void clearmapx(int y);																	// clear row at y
	void clearmapy(int x);																	// clear column at x
	void clearmapxy(int x, int y);														// clear a specific cell

public:
	//	Must be defined for NewSteer
	bool passableCell(int ix, int iy) const;											// is cell definitely passable?
	bool possibleCell(int ix, int iy) const;												// is cell marginally passable?
	bool unknownCell(int ix, int iy) const;											// is cell unknown?
	virtual bool passableMove(const double startx,							// start point (world coordinates)
					const double starty,
					const float startdirx,														// going in this direction (unit vector)
					const float startdiry,
					const float movedist,													// trying to move this distance
					const float invturnradius,												// with this turn radius
					float& allowedmovedist) const;									// if can't move as far as requested, a safe move distance
protected:
	//	May be defined for map update
	virtual void fillmap(int ixmin, int ixmax, int iymin, int iymax) {}		// fill entire map
	virtual void fillmapx(int ix, int iymin, int iymax) {}						// fill column that just scrolled on
	virtual void fillmapy(int ixmin, int ixmax, int iy) {}						// fill row that just scrolled on
public:																								// statics
	static int mod(int i, int j)																	// mod, with always positive results
	{	if (i >= 0) return(i % j);																// positive case
		return((j-1) - ((-(i+1)) % j));														// negative case, continuous through zero
	}								
};
//
//	cellonmap -- is this cell on the map?
//
//	Works for both even and odd map sizes.
//
template <class CELL, class PARENT> inline bool ScrollableMap<CELL,PARENT>::cellonmap(int ix, int iy) const
{	const int halfdim = m_dimincells/2;												// half the dimension, rounded down
	return ((ix >= m_xcenter - halfdim)
	&& (ix < m_xcenter + (m_dimincells-halfdim))
	&& (iy >= m_ycenter - halfdim)
	&& (iy < m_ycenter + (m_dimincells-halfdim))); 
}
//
//		cellindex  --  get index of cell in map, unchecked
//
template<class CELL, class PARENT> inline int ScrollableMap<CELL,PARENT>::cellindex(int ix, int iy) const
{	int mx = mod(ix,m_dimincells);															// always get a positive result with mod
	int my = mod(iy,m_dimincells);
	unsigned int index = my*m_dimincells+mx;										// calculate actual coord
	assert(index < m_map.	size());															// final subscript check
	return(index);
}
//
//		at  -- return cell at (x,y)
//
template<class CELL, class PARENT> inline CELL& ScrollableMap<CELL,PARENT>::at(int ix, int iy)
{
	assert(cellonmap(ix,iy));
	return(m_map[cellindex(ix,iy)]);
}
//
//		at  -- return cell at (x,y)
//
template<class CELL, class PARENT> inline const CELL& ScrollableMap<CELL,PARENT>::at(int ix, int iy) const
{
	assert(cellonmap(ix,iy));
	return(m_map[cellindex(ix,iy)]);
}
//
//		clearmapxy  -- clear map cell at indicated location
//
template<class CELL, class PARENT> inline void ScrollableMap<CELL,PARENT>::clearmapxy(int ix, int iy)
{	if (!cellonmap(ix,iy)) return;																// off the map
	at(ix,iy).clear();																					// clear the cell
}
//
//	passableCell  -- is cell definitely passable?
//
template<class CELL, class PARENT> inline bool ScrollableMap<CELL,PARENT>::passableCell(int ix, int iy) const
{	return(at(ix,iy).passable());	}
//
//	possibleCell  --  is cell possibly passable?
//
template<class CELL, class PARENT> inline bool ScrollableMap<CELL,PARENT>::possibleCell(int ix, int iy) const
{	return(at(ix,iy).possible());	}
//
//	unknownCell  --  is nothing known about this cell?
//
template<class CELL, class PARENT> inline bool ScrollableMap<CELL,PARENT>::unknownCell(int ix, int iy) const
{	return(at(ix,iy).unknown());	}
//
//	Constructor
//
template<class CELL, class PARENT> inline ScrollableMap<CELL,PARENT>::ScrollableMap(int dimincells, double cellspermeter)
:	m_cellspermeter(cellspermeter),
 	m_dimincells(dimincells),
 	m_xcenter(0), m_ycenter(0)
{	resize(dimincells, cellspermeter);												// resize as indicated
}
//
//	Destructor
//
template<class CELL, class PARENT> inline ScrollableMap<CELL,PARENT>::~ScrollableMap()
{
}
//
//	resize -- resize for a new size
//
template<class CELL, class PARENT> inline void ScrollableMap<CELL,PARENT>::resize(int dimincells, double cellspermeter)
{	m_cellspermeter = cellspermeter;
 	m_dimincells = dimincells;
	m_xcenter = m_ycenter = 0;													// reset center
	m_map.resize(m_dimincells * m_dimincells);								// unit of measure is cells here
	clearmap();																				// clear the map at startup
}
//
//
//	setmapcenter -- set the new center of the map, using coords
//
//	Areas that scroll off are cleared.
//
template<class CELL, class PARENT> inline void ScrollableMap<CELL,PARENT>::setmapcenter(double x, double y)
{	int xcell = coordtocell(x);									// calc new center cell
	int ycell = coordtocell(y);	
	setmapcentercell(xcell, ycell);							// set appropriate center cell
}
//
//
//	setmapcentercell -- set the new center of the map
//
//	Areas that scroll off are cleared.
//
template<class CELL, class PARENT> inline void ScrollableMap<CELL,PARENT>::setmapcentercell(int xcell, int ycell)
{	if (xcell == m_xcenter && ycell == m_ycenter) return;	// nothing to do
	if (!cellonmap(xcell,ycell))								// if completely off map
	{	m_xcenter = xcell;										// force new center
		m_ycenter = ycell;
		clearmap();													// clear entire map, reloading any data from derived classes
		return;
	}
	//	Normal move - we have to scroll the map.
	//	This involves erasing the edge row or column and adjusting the center cell position
	//	Example: map is 1000 cells across, center is 0,0, map is -500..499
	while (xcell > m_xcenter) 								// if need to scroll horizontally
	{	m_xcenter++; clearmapy(m_xcenter+m_dimincells-m_dimincells/2-1); 	 }	// center=1, clear column 500 of -499..500
	while (xcell < m_xcenter)
	{	m_xcenter--; clearmapy(m_xcenter-m_dimincells/2); }	// center=-1, clear column -501 of -501..498
	while (ycell > m_ycenter) 								// if need to scroll vertically
	{	m_ycenter++; clearmapx(m_ycenter+m_dimincells-m_dimincells/2-1); } // center=1, clear row 500 of -499..500
	while (ycell < m_ycenter)
	{	m_ycenter--; clearmapx(m_ycenter-m_dimincells/2); }	// center =-1, clear row -501 of -501..498
}
//
//	clearmapx -- clear a map row
//
//	Input is in absolute cell coords
//
template<class CELL, class PARENT> inline void ScrollableMap<CELL,PARENT>::clearmapx(int iy)
{	assert(cellonmap(m_xcenter,iy));								// must be on map
	const int ixmin = getminix();
	const int ixmax = getmaxix();
	for (int ix=ixmin; ix<=ixmax; ix++) clearmapxy(ix,iy);	// clear out old data
	fillmapy(ixmin, ixmax, iy);											// fill new column with new data (in subclass)
}
//
//	clearmapy -- clear a map column
//
template<class CELL, class PARENT> inline void ScrollableMap<CELL,PARENT>::clearmapy(int ix)
{	assert(cellonmap(ix, m_ycenter));							// must be on map
	const int iymin = getminiy();
	const int iymax = getmaxiy();
	for (int iy=iymin; iy<=iymax; iy++) clearmapxy(ix,iy);	// clear out old data
	fillmapx(ix, iymin, iymax);											// fill new row with new data										
}
//
//	clearmap -- clear entire map
//
template<class CELL, class PARENT> inline void ScrollableMap<CELL,PARENT>::clearmap(void)
{
	for (size_t i=0; i<m_map.size(); i++)
	{	m_map[i].clear();	}
	fillmap(getminix(), getmaxix(), getminiy(), getmaxiy());		// refilll after scrolling
}
//
//	passableMove  -- complex terrain evaluation
//
//	Subclass must implement if anything useful is to happen
//
template<class CELL, class PARENT> inline bool ScrollableMap<CELL,PARENT>::passableMove(
					const double startx,														// start point (world coordinates)
					const double starty,
					const float startdirx,														// going in this direction (unit vector)
					const float startdiry,
					const float movedist,													// trying to move this distance
					const float invturnradius,												// with this turn radius
					float& allowedmovedist)	const									// if can't move as far as requested, a safe move distance	
{	allowedmovedist = 0; 																	// can't handle complex terrain yet
	return(false);
}									
#endif // SCROLLABLEMAP_H
