/**
  This module scans the interior of a curved wedge and tests for
  collisions with the terrain map.
 
  Paul Upchurch
  Team Overbot
  2005-03
  
  Heavily modified by J. Nagle
 
  TODO: tune the distance_metric
  TODO: speed up by terminating wedge scanning earlier
*/
//
//	Converted to arbitrary curved paths by John Nagle
//
#include "terrainmap.h"
#include "curvedwedge.h"														// use common definitions
#include "curvedpath.h"
#include "logprint.h"
#include "algebra3aux.h"
#include "tuneable.h"
const double k_right_outside = -1.0;											// what used to be "outside" is now "left side"
//	Base halfwidth used for computing base tilt of a quad. Should be same halfwidth as vehicle.
const Tuneable k_plane_quad_basewidth("PLANEQUADBASEWIDTH", 0.5, 2.0, 1.0, "Base  of quad. Roughly, half width of vehicle (m)");
//
//	ivect2  -- used for positions as cell indices
//
struct ivect2
{
    int x,y;
};
//
//	IMPINGEMENT_RESULT -- primary internal data structure for an impingement
//
struct IMPINGEMENT_RESULT
{
    double center_hw;         // halfwidth of current quad, used to separate
    // center and shoulders
    vec2 origin;            // the origin of the wedge, used to calc metric
    vec2 c_position;        // position of nearest central impingement
    double c_metric;          // distance metric of central impingement
    // values approaching zero imply closer
    // values less than zero indicate no impingement
    double c_outside;         // signed distance to centerline of quad
    // positive is outside

    vec2 out_pt,out;        // point-normal line for the current quad
    vec2 o_position;        // outside (left) shoulder impingement
    double o_metric;
    vec2 i_position;        // inside (right) shoulder impingement
    double i_metric;
};
//
//	update_worst_tilt  -- update worst (most tilted) quad scanned
//
//	Used to detect overly tilted terrain
//
static void update_worst_tilt(vec3& worsttiltvector, const vec4& quadplane, const vec3& spnt, const vec3& epnt)
{
	vec3 tilt(quadplane, VW);								// normal vector of this quad - drop offset from origin
	if (tilt[2] < 0) tilt = -tilt;									// make tilt be upward
	if ((tilt * vec3(0,0,1))*1.01 < worsttiltvector * vec3(0,0,1))	// if more tilted
	{	worsttiltvector = tilt;									// save new one
		logprintf("Worst tilt (%1.2f %1.2f %1.2f) at (%1.2f, %1.2f) to (%1.2f, %1.2f)\n", 
			worsttiltvector[0], worsttiltvector[1], worsttiltvector[2], spnt[0], spnt[1], epnt[0], epnt[1]);	// ***TEMP***
	}
}
//
//	class CurvedPathObstacleScanner -- scans a curved path, trapezoid by trapezoid
//
//	Uses common path/trapezoid generator
//
class CurvedPathObstacleScanner: public CurvedPathScanner
{
private:
    const TerrainMap& m_map;							// map input
    ImpingementGroup& m_impingements;			// the impingements, as output
    IMPINGEMENT_RESULT& m_result;					// result output, local work
    vec4	m_quadplane;										// plane of the current quad
    float	m_terrainthreshold;								// allowable deviation from plane
public:
    CurvedPathObstacleScanner(const TerrainMap& map, ImpingementGroup& impingements, 
    	IMPINGEMENT_RESULT& result, float terrainthreshold)
            : m_map(map), m_impingements(impingements), m_result(result), m_quadplane(vec4(0,0,1,0)), 
            	m_terrainthreshold(terrainthreshold)
    {}
protected:
    bool raster_ordered_trapezoid(const vec2 quad[4], const vec2& last_sforward, const vec2& next_sforward,
                                  double distalongpath, float basewidth, float shoulderwidth);
private:
   	void collision_point(int x, int y, bool is_unknown);
   	void scan_point(int x, int y);
	void scan_hline0(int x, int y, int len);
	void scan_hline(int x1, int x2, int y);
	void scan_line(int x1, int y1, int x2, int y2);
	void raster_ordered_poly(ivect2 verts[], int numverts);
	bool evaluate_possible_terrain(int x, int y);
	void update_marginal_terrain_impingement(int x, int y, bool terrainimpassable, ImpingementInfo& impingement);
	bool construct_plane(const vec2& spnt, const vec2& epnt, const vec2& baseforward, float halfwidth);
	float get_elev(const vec2& pos);
};
const int IGNORE_UNKNOWN_SHOULDER=1;
const int USE_DISTANCE_APPROXIMATION=1;
const int DEBUG_PRINTF=0;
//
//	planeFromThreePoints  -- compute a plane from three points
//
//	A plane is a normal plus the distance from the origin.
//
vec4 planeFromThreePoints(const vec3& p0, const vec3& p1, const vec3& p2)
{
	vec3 e1(p1-p0);													// edge 1
	vec3 e2(p2-p0);													// edge 2
	vec3 perpen(e1 ^ e2);											// cross product, normal to plane
	if (perpen.length2() < 0.00001)							// if tiny vector
	{	return(vec4(0,0,1,0));	}									// assume flat, normal is straight up
	perpen.normalize();												// normalize
	vec4 planevec(perpen, 0);									// equation of plane, not including distance
	planevec[3] = -PointToPlane(p0, planevec);			// get dist from plane, use for plane vector
#ifdef OBSOLETE	// debug only
	//	***TEMP***
	logprintf("Plane  (%1.2f,%1.2f,%1.2f,%1.2f) from (%1.2f, %1.2f, %1.2f) (%1.2f, %1.2f, %1.2f) (%1.2f, %1.2f, %1.2f) \n",
		planevec[0], planevec[1], planevec[2], planevec[3],
		p0[0], p0[1], p0[2], 
		p1[0], p1[1], p1[2], 
		p2[0], p2[1], p2[2]);
#endif // OBSOLETE
	return(planevec);													// return resulting plane
}

/* ----------------------------------------------------------- */
/* code to compute the nearest collision */
/* ----------------------------------------------------------- */

/**
  computes a metric to measure distance to collision
*/
inline double distance_metric(double x, double y)
{
    if(USE_DISTANCE_APPROXIMATION)
    {
        // faster but less accurate (on the order of +=6%)
        if(x<0.0)
            x=(-x);
        if(y<0.0)
            y=(-y);
        if(x>y)
            return(x*0.941246+y*0.41);
        return(y*0.941246+x*0.41);
    }
    else
    {
        return(sqrt(x*x+y*y));
    }
}
//
//	evaluate_possible_terrain  -- evaluate possible (marginal) terrain
//
//	Returns true if marginal cell is in fact passable.
//
bool CurvedPathObstacleScanner::evaluate_possible_terrain(int x, int y)
{
	////return(false);																					// always reject for now ***TEMP***
	const CellData& cell = m_map.at(x,y);												// get indicated cell
	float elev = cell.avgelev();																// get its elevation
	vec3 pt(m_map.celltocoord(x), m_map.celltocoord(y), elev);			// point in 3-space
	float dist = fabs(PointToPlane(pt, m_quadplane));								// distance from plane
	bool good = dist <= m_terrainthreshold;											// good spot?
#ifdef OBSOLETE	// ***TEMP*** test only
	logprintf("Elev from test plane (%1.2f,%1.2f,%1.2f,%1.2f) at (%1.2f, %1.2f, %1.2f) is %1.2f  %s.\n",
		m_quadplane[0], m_quadplane[1], m_quadplane[2], m_quadplane[3], pt[0], pt[1], pt[2], dist, good ? "GOOD": "BAD");	// ***TEMP***
#endif // OBSOLETE
	return(good);																					// if success
}
//
//	construct_plane -- construct normal plane for quad
//
//	Plane construction is done from the two points at the base of the trapezoid, and the center of the trapezoid.
//
bool CurvedPathObstacleScanner::construct_plane(const vec2& spnt, const vec2& epnt, const vec2& baseforward, float halfwidth)
{	const vec2 baseright(baseforward[1], -baseforward[0]);				// base vector of triangle
	const vec2 p0(spnt + baseright*halfwidth);										// first base point
	const vec2 p1(spnt - baseright*halfwidth);										// other base point
	////const vec2 p2((spnt + epnt)*0.5);													// center of triangle
	const vec2 p2(epnt);																		// end of quad, center
	float q0elev = get_elev(p0);																// get elevation of triangle vertices
	float q1elev = get_elev(p1);
	float q2elev = get_elev(p2);
	if (!finite(q0elev) || !finite(q1elev))													// if base points are bad
	{	return(false);	}																			// reject this quad. Must be in unknown territory
	if (!finite(q2elev))																				// if distant point not defined
	{	q2elev = q0elev; }																		// assume flat
	const vec3 q0(p0[0], p0[1], q0elev);												// build 3D points of triangle
	const vec3 q1(p1[0], p1[1], q1elev);												// build 3D points of triangle
	const vec3 q2(p2[0], p2[1], q2elev);												// build 3D points of triangle
	m_quadplane = planeFromThreePoints(q0, q1, q2);							// compute plane from triangle	
	return(true);																						// success
}
//
//	get_elev  -- get elevation for a point
//
//	Assumes pos is within map.  All quads must be in map
//
float CurvedPathObstacleScanner::get_elev(const vec2& pos)
{	int x = m_map.coordtocell(pos[0]);													// compute cell number for pos
	int y = m_map.coordtocell(pos[1]);
	const CellData& cell = m_map.at(x,y);												// get indicated cell
	if (!cell.m_valid)																				// if cell value is unknown
	{	return(k_NaN);	}																			// return a NaN
	return(cell.avgelev());																		// otherwise return its elevation
}
//
//	update_marginal_terrain_impingement  -- update debug info for marginal terrain impingements
//
//	Called often.
//	Call only for known cells, not unknown.
//
inline void CurvedPathObstacleScanner::update_marginal_terrain_impingement(int x, int y, bool terrainimpassable, ImpingementInfo& impingement)
{
	impingement.m_terrainimpassable = terrainimpassable;					// failed because of marginal terrain eval?
	if (!terrainimpassable) return;															// no, done
	const CellData& cell = m_map.at(x,y);												// get indicated cell
	impingement.m_terrainimpassable = true;										// failed because of this cell
	impingement.m_elev = cell.avgelev();												// cell elevation
	impingement.m_plane = m_quadplane;												// plane of current quad
}
//
//	collision_point  -- called when a potential collision point has been found
//
//	Stores the nearest collision point in "result"
//
void CurvedPathObstacleScanner::collision_point(int x, int y, bool is_unknown)
{	const TerrainMap& map = m_map;													// convenient abbreviation
	IMPINGEMENT_RESULT& result = m_result;										// convenient abbreviation
    const vec2 position(map.celltocoord(x), map.celltocoord(y));			// the collision point
    //	Evaluation of "possible" or "yellow" terrain.
    bool possiblerejected = false;															// not marginal terrain case
    if (!is_unknown && m_map.possibleCell(x, y))									// if "yellow" (marginal) terrain
    {	bool good = evaluate_possible_terrain(x,y);								// evaluate it
    	if (good) return;																			// marginal terrain OK, continue
    	possiblerejected = true;																// note possible terrain rejected
    }
    // distance from the centerline
    // dot product of <unit normal, ray from the line to the point>
    // where unit normal is the out basis vector
    double dist = (position - result.out_pt) * result.out;					// dot product
    if(dist>=(-result.center_hw) && dist<=result.center_hw)
    {
        // central impingement
        double metric=distance_metric(position[0]-result.origin[0],position[1]-result.origin[1]);
        if(result.c_metric>=0.0 && metric>=result.c_metric)
            return;
      	//	Updating center impingement
        result.c_position=position;
        result.c_metric=metric;
        result.c_outside=dist;
      	update_marginal_terrain_impingement(x, y, possiblerejected, m_impingements.m_center);	// update center problem info for debug
        return;
    }
    if(dist>0.0)
    {
        // outside (left) impingement
        if(IGNORE_UNKNOWN_SHOULDER && is_unknown)
            return;
        if(result.o_metric>=0.0 && dist>=result.o_metric)
            return;
       	// Updating left impingement
        result.o_position=position;
        result.o_metric=dist;
		update_marginal_terrain_impingement(x, y,  possiblerejected, m_impingements.m_left);	// update left problem info for debug
        return;
    }
    // inside (right) impingement 
    // negate dist to make metric positive
    if(IGNORE_UNKNOWN_SHOULDER && is_unknown)
        return;
    dist=(-dist);
    if(result.i_metric>=0.0 && dist>=result.i_metric)
        return;
    // Updating right impingement
    result.i_position=position;
    result.i_metric=dist;
	update_marginal_terrain_impingement(x, y, possiblerejected, m_impingements.m_right);		// update right problem info for debug
}

/* ----------------------------------------------------------- */
/* polygon rasterizing code, internal use only */
/* ----------------------------------------------------------- */

/*
  Most of these functions are generic graphics primitive scanline
  rasterizers.
*/

void CurvedPathObstacleScanner::scan_point(int x, int y)
{
    if(m_map.passableCell(x,y))
        return;
    collision_point(x, y, m_map.unknownCell(x, y));
}
//
//	scan_hline0  -- scan one line in the rasterizer
//
//	This is where all the time goes.
//
void CurvedPathObstacleScanner::scan_hline0(int x, int y, int len)
{
	const TerrainMap& map = m_map;													// convenient abbreviation
    for(int i=0;i<len;i++,x++)
    {
        if(map.passableCell(x,y))																// if no problem
            continue;																					// continue
        collision_point(x, y, map.unknownCell(x,y));									// otherwise handle error or problem
    }
}

void CurvedPathObstacleScanner::scan_hline(int x1, int x2, int y)
{
    if(x1<x2)
        scan_hline0(x1, y, x2-x1+1 );
    else
        scan_hline0(x2, y, x1-x2+1 );
}

void CurvedPathObstacleScanner::scan_line(int x1, int y1, int x2, int y2)
{
    int dx,dy;
    int xunit,yunit;
    int term;

    dx=x2-x1;
    dy=y2-y1;
    if(dx<0)
    {
        dx=(-dx);
        xunit=(-1);
    }
    else
        xunit=1;
    if(dy<0)
    {
        dy=(-dy);
        yunit=(-1);
    }
    else
        yunit=1;
    term=0;
    if(dx>=dy)
    {
        while(x1!=x2)
        {
            scan_point(x1,y1);
            term+=dy;
            if(term>=dx)
            {
                term-=dx;
                y1+=yunit;
            }
            x1+=xunit;
        }
        scan_point(x1,y1);
    }
    else
    {
        while(y1!=y2)
        {
            scan_point(x1,y1);
            term+=dx;
            if(term>=dy)
            {
                term-=dy;
                x1+=xunit;
            }
            y1+=yunit;
        }
        scan_point(x1,y1);
    }
}

/**
  Most of the "dead" code in here is to handle degenerate polygons.
  Be very careful when changing this code since improperly handled
  degenerates tend to create wildly wrong results or infinite loops.
*/
void CurvedPathObstacleScanner::raster_ordered_poly(ivect2 verts[], int numverts)
{
    int i,j,py,bottom;
    int vert1=0,dx1=0,dy1=0,xunit1=0,term1=0,togo1=0;
    int vert2=0,dx2=0,dy2=0,xunit2=0,term2=0,togo2=0;
    int topv;
    int px1,px2;

    term1=1;
    bottom=verts[0].y;
    topv=0;
    py=bottom;
    for(i=1;i<numverts;i++)
    {
        if(verts[i].y>bottom)
            bottom=verts[i].y;
        if(verts[i].y<py)
        {
            py=verts[i].y;
            topv=i;
        }
        for(j=0;j<i && !(verts[j].x==verts[i].x && verts[j].y==verts[i].y);j++)
            ;
        if(j==i)
            term1++;
    }
    if(term1==1)
    {
        scan_point(verts[0].x,verts[0].y);
        return;
    }
    if(term1==2)
    {
        for(i=1;i<numverts;i++)
        {
            for(j=0;j<i && !(verts[j].x==verts[i].x && verts[j].y==verts[i].y);j++)
                ;
            if(j==i)
            {
                scan_line(verts[0].x,verts[0].y,verts[i].x,verts[i].y);
                return;
            }
        }
        return;
    }
    if(py==bottom)
    {
        px1=verts[0].x;
        for(i=1;i<numverts;i++)
            px1=(px1<verts[i].x?px1:verts[i].x);
        px2=verts[0].x;
        for(i=1;i<numverts;i++)
            px2=(px2>verts[i].x?px2:verts[i].x);
        scan_hline(px1,px2,bottom);
        return;
    }

    px1=verts[topv].x;
    px2=px1;

    vert1=topv;
    togo1=0;
    vert2=topv;
    togo2=0;

    for(;py<=bottom;)
    {
        while(togo1<1 && py<bottom)
        {
            if(px1!=verts[vert1].x)
                return;
            vert1=vert1-1;
            if(vert1<0)
                vert1=numverts-1;
            dy1=verts[vert1].y-py;
            togo1=dy1;
            if(togo1<1)
            {
                px1=verts[vert1].x;
                scan_hline(px1,px2,py);
                continue;
            }
            dx1=verts[vert1].x-px1;
            if(dx1<0)
            {
                dx1=-dx1;
                xunit1=(-1);
            }
            else
                xunit1=1;
            term1=0;
        }
        while(togo2<1 && py<bottom)
        {
            if(px2!=verts[vert2].x)
                return;
            vert2=vert2+1;
            if(vert2>numverts-1)
                vert2=0;
            dy2=verts[vert2].y-py;
            togo2=dy2;
            if(togo2<1)
            {
                px2=verts[vert2].x;
                scan_hline(px1,px2,py);
                continue;
            }
            dx2=verts[vert2].x-px2;
            if(dx2<0)
            {
                dx2=-dx2;
                xunit2=(-1);
            }
            else
                xunit2=1;
            term2=0;
        }
        scan_hline(px1,px2,py);
        if(togo1>0 && py<bottom)
        {
            term1+=dx1;
            while(term1>=dy1)
            {
                term1-=dy1;
                px1+=xunit1;
            }
            togo1--;
        }
        if(togo2>0 && py<bottom)
        {
            term2+=dx2;
            while(term2>=dy2)
            {
                term2-=dy2;
                px2+=xunit2;
            }
            togo2--;
        }
        py++;
    }
}
//
//	raster_ordered_trapezoid  -- process one trapezoid
//
//	Input are the four corners of a trapezoid to be tested against the map.
//	Input is in coordinates, not cell indices.
//
bool CurvedPathObstacleScanner::raster_ordered_trapezoid(const vec2 quad[4], const vec2& last_sforward, const vec2& next_sforward,
        double distalongpath, float basewidth, float shoulderwidth)
{
    // store quad-specific numbers in the result ctx for
    // correct collision point classification
    m_result.center_hw = basewidth*0.5;										// halfwidth info
    vec2 spnt((quad[0] + quad[1])*0.5);										// center of base of quad
    m_result.out_pt = spnt;																// center of base of quad
    //	Compute direction for perpendicular to path.
    //	Really should call pointalongpath again, but for speed we just
    //	use the average of the forward vectors for the quad
    vec2 midforward(last_sforward + next_sforward);					// vector forward at middle of quad, unnormalized
    midforward.normalize();
    vec2 midright(midforward[1], -midforward[0]);							// vector to right at middle of quad
    m_result.out = -midright;															// vector to left at middle of quad
    //	Construct the test quad (basewidth only) for computing the normal plane.
    vec2 epnt((quad[2]+quad[3])*0.5);											// end of quad
    bool knownquad = construct_plane(spnt, epnt, last_sforward, k_plane_quad_basewidth);		// beginning center, ending center, halfwidth
    if (!knownquad)																		// if quad rejected (center start point unknown)
    {	int ix = m_map.coordtocell(spnt[0]);										// mark center of baseline of quad as unknown
        int iy = m_map.coordtocell(spnt[1]);
    	collision_point(ix, iy, true);													// mark as unknown
		return(false);																		// stop scanning
    }
    //	Construct an integer quad.
    // Note that the verticies of the quad are ordered.
    ivect2 verts[4];																			// corners of the quad, as cell indices
    for(int j=0;j<4;j++)
    {
        verts[j].x = m_map.coordtocell(quad[j][0]);
        verts[j].y = m_map.coordtocell(quad[j][1]);
    }
    // rasterize
    raster_ordered_poly(verts,4);													// actually scan the trapezoid
    //	If no obstacle found in center yet, update worst tilt seen.
  	if (m_result.c_metric < 0.0)														// if no center obstacles yet
	{	//	Update the worst tilted plane seen before scan stopped.
		update_worst_tilt(m_impingements.m_worsttiltvector, m_quadplane, spnt, epnt);	// update most tilted quad seen
	}
    return(true);																				// don't stop
}
/**
  glue logic between this module and the rest of newsteer
*/
void scanPathAgainstObstacles(const TerrainMap& map,
                              const CurvedPath& path,
                              const float basewidth,
                              const float shoulderwidth,
                              const float pathlength, 									// only test this much of path
                              const float terrainthreshold,							// allowed deviation from plane
                              ImpingementGroup& impingements)				// returned impingements
{
    impingements.clear();																// no outputs yet
    IMPINGEMENT_RESULT collision;
    ivect2 cell;

    // The correct value for num_segments depends heavily on the
    // shape of the curved wedge. As an approximation we use a single
    // constant. Optimally, we would want to calculate num_segments based
    // on some product of arclength and curvature.
    const int num_segments=10;

    collision.origin= path.getstartpos();
    collision.c_metric=(-1.0);
    collision.o_metric=(-1.0);
    collision.i_metric=(-1.0);
    CurvedPathObstacleScanner scanner(map, impingements, collision, terrainthreshold);						// create scanner object
    bool good = scanner.scan_wedge(path, basewidth, shoulderwidth, pathlength, num_segments);		// do the scan, update collision
    if (!good)																											// trouble
    {	impingements.m_center.m_valid = true;													// create phony collision at start
        impingements.m_center.m_pos = path.getstartpos();
        impingements.m_center.m_offset = 0;
        impingements.m_center.m_passable = true;
        impingements.m_center.m_unknown = true;
        impingements.m_center.m_possible= true;
        return;																										// fails
    }
    // fill in center collision
    if(collision.c_metric>=0.0)
    {
        impingements.m_center.m_valid=true;
        impingements.m_center.m_pos=collision.c_position;
        impingements.m_center.m_offset=collision.c_outside*k_right_outside;
        cell.x=map.coordtocell(collision.c_position[0]);
        cell.y=map.coordtocell(collision.c_position[1]);
        impingements.m_center.m_passable=map.passableCell(cell.x,cell.y);
        impingements.m_center.m_unknown=map.unknownCell(cell.x,cell.y);
        impingements.m_center.m_possible=map.possibleCell(cell.x,cell.y);
        if(DEBUG_PRINTF)
        {
            const double arclength = path.getlength();
            const vec2 d_pt(path.getstartpos());
            const vec2 d_forward(path.getstartforward());
            const double curvature = 0;	// ***TEMP***
            logprintf("pt=%1.2f,%1.2f forward=%1.2f,%1.2f basewidth=%1.2f shoulderwidth=%1.2f curvature=%f arclength=%1.2f center collision: %f,%f\n",
                      d_pt[0],d_pt[1],d_forward[0],d_forward[1],basewidth,shoulderwidth,curvature,arclength,collision.c_position[0],collision.c_position[1]);
        }
    }
    else
    {
        impingements.m_center.m_valid=false;
    }

    ImpingementInfo& op = impingements.m_left;
    ImpingementInfo& ip = impingements.m_right;

    // fill in outside collision
    // note: offset will always be positive
    if(collision.o_metric>=0.0)
    {
        op.m_valid=true;
        op.m_pos[0]=collision.o_position[0];
        op.m_pos[1]=collision.o_position[1];
        op.m_offset=collision.o_metric;
        cell.x=map.coordtocell(collision.o_position[0]);
        cell.y=map.coordtocell(collision.o_position[1]);
        op.m_passable=map.passableCell(cell.x,cell.y);
        op.m_unknown=map.unknownCell(cell.x,cell.y);
        op.m_possible=map.possibleCell(cell.x,cell.y);
        if(DEBUG_PRINTF)
        {
            const double arclength = path.getlength();
            const vec2 d_pt(path.getstartpos());
            const vec2 d_forward(path.getstartforward());
            const double curvature = 0;	// ***TEMP***
            logprintf("pt=%1.2f,%1.2f forward=%1.2f,%1.2f basewidth=%1.2f shoulderwidth=%1.2f curvature=%f arclength=%1.2f outside collision: %f,%f\n",
                      d_pt[0],d_pt[1],d_forward[0],d_forward[1],basewidth,shoulderwidth,curvature,arclength,collision.o_position[0],collision.o_position[1]);
        }
    }
    else
    {
        op.m_valid=false;
    }

    // fill in inside collision
    // note: offset will always be positive
    if(collision.i_metric>=0.0)
    {
        ip.m_valid=true;
        ip.m_pos[0]=collision.i_position[0];
        ip.m_pos[1]=collision.i_position[1];
        ip.m_offset=collision.i_metric;
        cell.x=map.coordtocell(collision.i_position[0]);
        cell.y=map.coordtocell(collision.i_position[1]);
        ip.m_passable=map.passableCell(cell.x,cell.y);
        ip.m_unknown=map.unknownCell(cell.x,cell.y);
        ip.m_possible=map.possibleCell(cell.x,cell.y);
        if(DEBUG_PRINTF)
        {
            const double arclength = path.getlength();
            const vec2 d_pt(path.getstartpos());
            const vec2 d_forward(path.getstartforward());
            const double curvature = 0;	// ***TEMP***
            logprintf("pt=%1.2f,%1.2f forward=%1.2f,%1.2f basewidth=%1.2f shoulderwidth=%1.2f curvature=%f arclength=%1.2f inside collision: %f,%f\n",
                      d_pt[0],d_pt[1],d_forward[0],d_forward[1],basewidth,shoulderwidth,curvature,arclength,collision.i_position[0],collision.i_position[1]);
        }
    }
    else
    {
        ip.m_valid=false;
    }

    // leftshoulder always has negative offset
    impingements.m_left.m_offset=(-impingements.m_left.m_offset);
}
