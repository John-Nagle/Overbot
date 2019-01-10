//
//	mapupdate.cc  --  map updating based on sensor data
//
//	John Nagle
//	Team Overbot
//	December, 2004
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
#include "mapserver.h"
#include "logprint.h"
#include "tuneable.h"
#include "algebra3.h"
#include "maplog.h"
#include "mapraster.h"
//
//	Configurable constants
//
const Tuneable k_maxangle_param("SCANWIDTH",60,180,120,"Scan width, degrees");
const double k_scan_interval = 1.0;										// scans are 1 degree apart
const int k_maxangle = int(k_maxangle_param/(2*k_scan_interval));				// half angle, 
//
//	Triangle size limits  -- scanned triangles outsize these ranges are ignored.
//
//	Min size should be small enough that, looking down at the bumper, we get a valid triangle.
//
const Tuneable k_min_triangle_size("TRIANGLEMIN",0.01,1.0,0.01,"Minimum triangle size, square meters");
const Tuneable k_max_triangle_size("TRIANGLEMAX",0.01,1.0,1.0,"Maximum triangle size, square meters");
const Tuneable k_max_triangle_side("TRIANGLEMAXSIDE",0.01,1.0,0.5,"Maximum triangle side length, meters");
//
//	Roughness limits for good/bad/medium cells
//
const Tuneable k_clear_cell_roughness_limit("CLEARCELLROUGHNESS", 1, 30, 15,"Maximum roughness of clear area, cm");
const Tuneable k_nogo_cell_roughness_limit("NOGOCELLROUGHNESS", 1, 30, 30,"Minimum roughness of no-go area, cm");
const float k_clear_cell_roughness_limit_m = float(k_clear_cell_roughness_limit)*0.01; // in meters
const float k_nogo_cell_roughness_limit_m = float(k_nogo_cell_roughness_limit)*0.01; // in meters
//
inline double sqr(double x) { return(x*x); }								// not in math.h
//	External static access to constants above
float MapServer::getClearCellRoughnessLimit() { return(float(k_clear_cell_roughness_limit_m)); }	
float MapServer::getNogoCellRoughnessLimit() { return(float(k_nogo_cell_roughness_limit_m)); }
//
//	updateMapRectangle -- add a rectangle into the map
//
//	This is used for marking the area under the vehicle as valid.
//
//	Points are clockwise.
//
void MapServer::updateMapRectangle(const vec3& p1, const vec3& p2, const vec3& p3, const vec3& p4,
	float minrange, uint32_t cyclestamp, bool fillallasgreen)
{
  ivect2 verts[4];
  RASTER_CTX ctx;

  verts[0].x=m_map.coordtocell(p1[0]);
  verts[0].y=m_map.coordtocell(p1[1]);
  verts[1].x=m_map.coordtocell(p2[0]);
  verts[1].y=m_map.coordtocell(p2[1]);
  verts[2].x=m_map.coordtocell(p3[0]);
  verts[2].y=m_map.coordtocell(p3[1]);
  verts[3].x=m_map.coordtocell(p4[0]);
  verts[3].y=m_map.coordtocell(p4[1]);
  ctx.map=this;
  ctx.newtype=CellData::CLEAR;             // clear
  ctx.minrange=int(minrange*100);          // in cm
  ctx.roughness=0;                         // flat
  ctx.elev=(p1[2]+p2[2]+p3[2]+p4[2])*0.25; // avg elevation
  ctx.cyclestamp=cyclestamp;
  ctx.forcetogood = fillallasgreen;					// force to green if this set, even if red. Used only under vehicle
  write_ordered_poly_cell(verts,4,ctx);
}
//
//	updateMapTriangle  --  add a triangle into the map
//
//	First, check that triangle is not too large or too small.
//	If it's too large, it may cover a hole in the data.
//	If it's too small, the roughness won't be meaningful.
//
//	Note that this won't fill a triangle so big it covers more than two cells in either axis.
//	We might miss a pothole if we did that.
//
//	Second, check the worst-case height variation.
//		If below green threshold, mark cells as green.
//		If above red threshold, mark cells as red.
//		Otherwise, mark cells as yellow.
//
void MapServer::updateMapTriangle(const vec3& p1, const vec3& p2, const vec3& p3, bool sweeping, float minrange, uint32_t cyclestamp)
{	//	Compute triangle bounds
	if (minrange <= 0) return;												// invalid value, no update
	//	Compute area of triangle
	vec3 p1p2(p1-p2);															// triangle side vectors
	vec3 p2p3(p2-p3);
	vec3 p3p1(p3-p1);
	double a = (p1p2).length();												// length of side P1-P2 
	double b = (p2p3).length();												// length of side P2-P3 
	double c = (p3p1).length();												// length of side P3-P1
	double perim = a+b+c;													// perimeter
	double areasq = perim*(perim-a)*(perim-b)*(perim-c);	// area squared, by formula of Heron
	double longestside = std::max(std::max(a,b),c);				// longest side of triangle
	if (getVerboseLevel() >= 3)												// really, really verbose
	{	logprintf(" Updating triangle [%6.3f %6.3f %6.3f] - [%6.3f %6.3f %6.3f] - [%6.3f %6.3f %6.3f] area %1.3f \n",
			p1[0],p1[1],p1[2],
			p2[0],p2[1],p2[2],
			p3[0],p3[1],p3[2],
			sqrt(areasq));
	}
	if (areasq < sqr(k_min_triangle_size)) return;					// too small, ignore
	if (areasq > sqr(k_max_triangle_size)) return;					// too big, ignore
	if (longestside > k_max_triangle_side) return;					// too big , ignore
	//	Check for oblique triangle -- too much shift between successive scan lines.
	//	Assumes that p1 and p2 are on the same scan line, and p3 is the third point.
	////if (p1p2 * p2p3 > 0) return;												// too oblique (***DIDN'T HELP***)
	////if (p1p2 * p3p1 < 0) return;												// too oblique
	//	Compute x, y, z bounds of triangle
	double minx = std::min(std::min(p1[0],p2[0]),p3[0]);
	double maxx = std::max(std::max(p1[0],p2[0]),p3[0]);
	double miny = std::min(std::min(p1[1],p2[1]),p3[1]);
	double maxy = std::max(std::max(p1[1],p2[1]),p3[1]);
	//	Triangle of reasonable size, evaluate.
	double minz = std::min(std::min(p1[2],p2[2]),p3[2]);
	double maxz = std::max(std::max(p1[2],p2[2]),p3[2]);
	double zrange = maxz-minz;											// elevation range
	float elev = (maxz+minz)*0.5;											// elevation average
	uint8_t roughness = int(std::min(fabs(zrange)*100.0,255.0));	// roughness in cm, for the map
	CellData::CellType newtype = CellData::NOGO;					// assume bad area
	if (roughness < k_clear_cell_roughness_limit)					// if very flat
	{	newtype = CellData::CLEAR;	}										// good (green) area
	else if (roughness < k_nogo_cell_roughness_limit)			// if not totally hopeless
	{	newtype = CellData::POSSIBLE; }									// questionable (yellow) area to be evaluated later
	uint16_t iminrange = uint16_t(minrange*100);					// minrange as unsigned, in cm
	//	Update entire rectangle containing triangle. Inefficient, but gets all the cells.
	int minix = m_map.coordtocell(minx);							// get cell index from X, Y location
	int miniy = m_map.coordtocell(miny);							// get cell index from X, Y location
	int maxix = m_map.coordtocell(maxx);							// get cell index from X, Y location
	int maxiy = m_map.coordtocell(maxy);							// get cell index from X, Y location
	if (newtype == CellData::POSSIBLE)								// if POSSIBLE mode (yellow)
	{	minix--;																		// increase coverage area so elevation eval will occur
		maxix++;
		miniy--;
		maxiy++;
	}
	for (int ix=minix; ix<= maxix; ix++)								// cover rectangle of interest
	{	for (int iy=miniy; iy <= maxiy; iy++)
		{	updateCell(ix, iy, newtype, sweeping, iminrange, roughness, elev, cyclestamp);	}
	}
}
//
//	updateMapEdge  -- update a single map edge
//
//	This recognizes big changes in elevation, to detect vertical objects.
//	It will never mark a cell good, only bad.
//
void MapServer::updateMapEdge(const vec3& p1, const vec3& p2, float r1, float r2, bool sweeping, uint32_t cyclestamp)
{	if ((r1 <= 0) || (r2 <= 0)) return;										// invalid value, ignore
	double minx = std::min(p1[0],p2[0]);
	double maxx = std::max(p1[0],p2[0]);
	double miny = std::min(p1[1],p2[1]);
	double maxy = std::max(p1[1],p2[1]);
	double minz = std::min(p1[2],p2[2]);
	double maxz = std::max(p1[2],p2[2]);
	double xrange = maxx-minx;											// east-west range of triangle
	double yrange = maxy-miny;											// north south range of triangle
	double zrange = maxz-minz;											// elevation range
	////double minxyrange = std::min(xrange,yrange);				// minimum XY range
	double maxxyrange = std::max(xrange,yrange);				// maxmum XY range
	double minrange = std::min(r1,r2);									// minimum range sensed
	if (maxxyrange > k_max_triangle_size) return;					// too big, ignore (but small ones are OK)
	float elev = (maxz+minz)*0.5;											// elevation average
	uint8_t roughness = int(std::min(fabs(zrange)*100.0,255.0));	// roughness in cm, for the map
	CellData::CellType newtype = CellData::NOGO;					// assume bad area
	if (roughness < k_clear_cell_roughness_limit)					// if very flat
	{	newtype = CellData::UNKNOWN;	}								// an edge is not enough to make a cell good
	else if (roughness < k_nogo_cell_roughness_limit)			// if not totally hopeless
	{	newtype = CellData::POSSIBLE; }									// questionable (yellow) area to be evaluated later
#ifdef OBSOLETE
	if (roughness >= k_nogo_cell_roughness_limit)				// ***TEMP*** show bad areas
		printf("Edge [%6.2f %6.2f %6.2f] [%6.2f %6.2f %6.2f] roughness %d maxxy %6.2f.\n",
			p1[0],p1[1],p1[2], p2[0],p2[1],p2[2],roughness,maxxyrange);			// ***TEMP***
#endif // OBSOLETE
	uint16_t iminrange = uint16_t(minrange*100);					// minrange as unsigned
#ifdef OBSOLETE
	updateCell(p1, newtype, sweeping, iminrange, roughness, elev, cyclestamp);					// update one cell
	updateCell(p2, newtype, sweeping, iminrange, roughness, elev, cyclestamp);					// update one cell
#endif // OBSOLETE
	//	Update entire rectangle containing triangle. Inefficient, but gets all the cells.
	int minix = m_map.coordtocell(minx);							// get cell index from X, Y location
	int miniy = m_map.coordtocell(miny);							// get cell index from X, Y location
	int maxix = m_map.coordtocell(maxx);							// get cell index from X, Y location
	int maxiy = m_map.coordtocell(maxy);							// get cell index from X, Y location
	for (int ix=minix; ix<= maxix; ix++)								// cover rectangle of interest
	{	for (int iy=miniy; iy <= maxiy; iy++)
		{	updateCell(ix, iy, newtype, sweeping, iminrange, roughness, elev, cyclestamp);	}
	}
}
//
//	updateCell -- update a single cell of the map
//
void MapServer::updateCell(const vec3& p, CellData::CellType newtype, bool sweeping, uint16_t minrange, uint8_t roughness, float elev, uint32_t cyclestamp)
{	int ix = m_map.coordtocell(p[0]);							// get cell index from X, Y location
	int iy = m_map.coordtocell(p[1]);
	updateCell(ix, iy, newtype, sweeping, minrange, roughness, elev, cyclestamp);	// use cell index form
}
//
//	updateCell -- update a single cell of the map
//
void MapServer::updateCell(int ix, int iy, CellData::CellType newtype, bool sweeping, uint16_t minrange, uint8_t roughness, float elev, uint32_t cyclestamp)   
{
	if (!m_map.cellonmap(ix, iy)) 								// off the grid, ignore
	{	if (getVerboseLevel() >= 3)
		logprintf("Tried to update cell [%d,%d], off the map.\n",ix,iy);	// unlikely
		return;																// must ignore
	}
	const uint32_t ancientstamp = getMap().getancientstamp();		// override if newer than this.
	CellData& cell = m_map.at(ix,iy);							// the relevant cell
	bool updated = cell.update(newtype, sweeping, minrange, roughness, elev, cyclestamp,
		 k_clear_cell_roughness_limit, k_nogo_cell_roughness_limit, ancientstamp);

	if (updated)
	{	m_log.logMapChange(cell,ix,iy);						// log change to cell
	}
	if (getVerboseLevel() >= 3)
	{	////printf("Updated [%d,%d] to %d %s\n",ix,iy,newtype, updated ? "(CHANGED)":"");	//	very verbose ***TEMP***
	}
}
