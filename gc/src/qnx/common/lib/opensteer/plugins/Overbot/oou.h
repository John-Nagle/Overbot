// ----------------------------------------------------------------------------
//
// Overbot/OpenSteer utilities
// by Craig Reynolds
//
// Contains support classes used for the Overbot/OpenSteer interface:
//
//     OoTerrainMap:   O/O map class (used only for OpenSteerDemo)
//
//     OoGCRoute:      O/O Grand Challenge Route class
//
//     Waypoint:       a route waypoint, specifies width of next leg
//         Waypoints:  STL vector of Waypoint objects
//         WPI:        iterator over Waypoints
//         utilities:  makePathFromWaypoints
//                     copyPathToWaypoints
//
// 01-21-05 cwr: split off from OverbotPlugIn.cpp
//
// ----------------------------------------------------------------------------


#ifndef OOU_H
#define OOU_H


#include "OpenSteer/SimpleVehicle.h"

using namespace OpenSteer;


// ----------------------------------------------------------------------------
// abstract base class for terrain traversability map


class AbstractTerrainMap
{
//protected:
public:

    // width/height of a cell
    virtual void setCellDimensions (double cellsize) = 0;
    virtual double getCellDimensions (void) const = 0;

    // is cell definitely passable?
    virtual bool passableCell (int x, int y) = 0;

    // is cell marginally or definitely passable?
    virtual bool possibleCell (int x, int y) = 0;

    virtual bool passableMove (
                               // start point (world coordinates)
                               const double startx,
                               const double starty,
                               // going in this direction (unit vector)
                               const float startdirx,
                               const float startdiry,
                               // trying to move this distance
                               const float movedist,
                               // with this turn radius
                               const float invturnradius,
                               // if can't move as far as requested,
                               // a safe move distance
                               float& allowedmovedist
                               ) = 0;
};

typedef AbstractTerrainMap OpenSteerMap;


// ----------------------------------------------------------------------------
// OoTerrainMap: terrain map class for the Overbot plug-in to OpenSteerDemo.
//
// XXX Moved here from OverbotPlugIn.cpp
// XXX Currently defined inline in the header file -- reconsider.


#ifndef NOT_OPENSTEERDEMO  // only when building OpenSteerDemo plug-in


class OoTerrainMap
{
public:

    // constructor
    OoTerrainMap (const Vec3& c, float x, float z, int r)
        : center(c),
          xSize(x),
          zSize(z),
          resolution(r),
          outsideValue (false)
    {
        map.reserve (resolution * resolution);
    }

    // destructor
    ~OoTerrainMap ()
    {
    }

    // clear the map (to false)
    void clear (void)
    {
        for (int i = 0; i < resolution; i++)
            for (int j = 0; j < resolution; j++)
                setMapBit (i, j, 0);
    }


    // get and set a bit based on 2d integer map index
    bool getMapBit (int i, int j) const
    {
        return map[mapAddress(i, j)];
    }

    bool setMapBit (int i, int j, bool value)
    {
        return map[mapAddress(i, j)] = value;
    }


    // get a value based on a position in 3d world space
    bool getMapValue (const Vec3& point) const
    {
        const Vec3 local = point - center;
        const Vec3 localXZ = local.setYtoZero();

        const float hxs = xSize/2;
        const float hzs = zSize/2;

        const float x = localXZ.x;
        const float z = localXZ.z;

        const bool out = (x > +hxs) || (x < -hxs) || (z > +hzs) || (z < -hzs);

        if (out) 
        {
            return outsideValue;
        }
        else
        {
            const float r = (float) resolution; // prevent VC7.1 warning
            const int i = (int) remapInterval (x, -hxs, hxs, 0.0f, r);
            const int j = (int) remapInterval (z, -hzs, hzs, 0.0f, r);
            return getMapBit (i, j);
        }
    }


    float minSpacing (void) const
    {
        return minXXX (xSize, zSize) / (float)resolution;
    }


    int cellwidth (void) const {return resolution;}  // xxx cwr
    int cellheight (void) const {return resolution;}  // xxx cwr

    Vec3 center;
    float xSize;
    float zSize;
    int resolution;

    bool outsideValue;

private:

    int mapAddress (int i, int j) const {return i + (j * resolution);}

    std::vector<bool> map;
};


#endif // NOT_OPENSTEERDEMO


// ----------------------------------------------------------------------------
// A variation on PolylinePathway (whose path tube radius is constant)
// OoGCRoute (Grand Challenge Route) has an array of radii-per-segment
//
// XXX The OpenSteer path classes are long overdue for a rewrite.  When
// XXX that happens, support should be provided for constant-radius,
// XXX radius-per-segment (as in OoGCRoute), and radius-per-vertex.


class OoGCRoute : public PolylinePathway
{
public:

    // construct a OoGCRoute given the number of points (vertices), an
    // array of points, an array of per-segment path radii, and a flag
    // indiating if the path is connected at the end.
    OoGCRoute (const int _pointCount,
               const Vec3 _points[],
               const float _radii[],
               const bool _cyclic)
    {
        initialize (_pointCount, _points, _radii[0], _cyclic);

        radii = new float [pointCount];

        // loop over all points
        for (int i = 0; i < pointCount; i++)
        {
            // copy in point locations, closing cycle when appropriate
            const bool closeCycle = cyclic && (i == pointCount-1);
            const int j = closeCycle ? 0 : i;
            points[i] = _points[j];
            radii[i] = _radii[i];
        }
    }

    // override the PolylinePathway method to allow for OoGCRoute-style
    // per-leg radii

    // Given an arbitrary point ("A"), returns the nearest point ("P") on
    // this path.  Also returns, via output arguments, the path tangent at
    // P and a measure of how far A is outside the Pathway's "tube".  Note
    // that a negative distance indicates A is inside the Pathway.

    Vec3 mapPointToPath (const Vec3& point, Vec3& tangent, float& outside)
    {
        Vec3 onPath;
        outside = FLT_MAX;

        // loop over all segments, find the one nearest to the given point
        for (int i = 1; i < pointCount; i++)
        {
            // QQQ note bizarre calling sequence of pointToSegmentDistance
            segmentLength = lengths[i];
            segmentNormal = normals[i];
            const float d =pointToSegmentDistance(point,points[i-1],points[i]);

            // measure how far original point is outside the Pathway's "tube"
            // (negative values (from 0 to -radius) measure "insideness")
            const float o = d - radii[i];

            // when this is the smallest "outsideness" seen so far, take
            // note and save the corresponding point-on-path and tangent
            if (o < outside)
            {
                outside = o;
                onPath = chosen;
                tangent = segmentNormal;
            }
        }

        // return point on path
        return onPath;
    }

    // ignore that "tangent" output argument which is never used
    // XXX eventually move this to Pathway class
    Vec3 mapPointToPath (const Vec3& point, float& outside)
    {
        Vec3 tangent;
        return mapPointToPath (point, tangent, outside);
    }

    // get the index number of the path segment nearest the given point
    // XXX consider moving this to path class
    int indexOfNearestSegment (const Vec3& point)
    {
        int index = 0;
        float minDistance = FLT_MAX;

        // loop over all segments, find the one nearest the given point
        for (int i = 1; i < pointCount; i++)
        {
            segmentLength = lengths[i];
            segmentNormal = normals[i];
            float d = pointToSegmentDistance (point, points[i-1], points[i]);
            if (d < minDistance)
            {
                minDistance = d;
                index = i;
            }
        }
        return index;
    }

    // returns the dot product of the tangents of two path segments, 
    // used to measure the "angle" at a path vertex: how sharp is the turn?
    float dotSegmentUnitTangents (int segmentIndex0, int segmentIndex1)
    {
        return normals[segmentIndex0].dot (normals[segmentIndex1]);
    }

    // return path tangent at given point (its projection on path)
    Vec3 tangentAt (const Vec3& point)
    {
        return normals [indexOfNearestSegment (point)];
    }

    // return path tangent at given point (its projection on path),
    // multiplied by the given pathfollowing direction (+1/-1 =
    // upstream/downstream).  Near path vertices (waypoints) use the
    // tangent of the "next segment" in the given direction
    Vec3 tangentAt (const Vec3& point, const int pathFollowDirection)
    {
        const int segmentIndex = indexOfNearestSegment (point);
        const int nextIndex = segmentIndex + pathFollowDirection;
        const bool insideNextSegment = isInsidePathSegment (point, nextIndex);
        const int i = (segmentIndex +
                       (insideNextSegment ? pathFollowDirection : 0));
        return normals [i] * (float)pathFollowDirection;
    }

    // is the given point "near" a waypoint of this path?  ("near" == closer
    // to the waypoint than the max of radii of two adjacent segments)
    bool nearWaypoint (const Vec3& point)
    {
        // loop over all waypoints
        for (int i = 1; i < pointCount; i++)
        {
            // return true if near enough to this waypoint
            const float r = maxXXX (radii[i], radii[i+1]);
            const float d = (point - points[i]).length ();
            if (d < r) return true;
        }
        return false;
    }

    // is the given point inside the path tube of the given segment
    // number?  (currently not used. this seemed like a useful utility,
    // but wasn't right for the problem I was trying to solve)
    bool isInsidePathSegment (const Vec3& point, const int segmentIndex)
    {
        const int i = segmentIndex;

        // QQQ note bizarre calling sequence of pointToSegmentDistance
        segmentLength = lengths[i];
        segmentNormal = normals[i];
        const float d = pointToSegmentDistance(point, points[i-1], points[i]);

        // measure how far original point is outside the Pathway's "tube"
        // (negative values (from 0 to -radius) measure "insideness")
        const float o = d - radii[i];

        // return true if point is inside the tube
        return o < 0;
    }

    // per-segment radius (width) array
    float* radii;
};


// ----------------------------------------------------------------------------
//
//	struct Waypoint	-- one waypoint
//
//	An array of waypoints defines "pathways", upon which the vehicle must
//	stay.  The vehicle's objective is to move along the pathways in the
//	direction of later waypoints.


struct Waypoint
{
    int m_serial;       // serial number (begins at 1)
    float m_speedlimit; // maximum speed in meters per second
    double m_x, m_y;    // waypoint coordinates
    float m_width;      // allowed "road width"
};


// a list (STL vector) of Waypoint objects
typedef std::vector<Waypoint> Waypoints;


// a const iterator for Waypoints
typedef Waypoints::const_iterator WPI;


// Take a list (STL vector) of Waypoints and allocate a new
// OoGCRoute object with corresponding geometry
OoGCRoute* makePathFromWaypoints (const Waypoints& waypoints);


// given a path and a vector of waypoints, copy the path's contents
// into the waypoints.
void copyPathToWaypoints (const OoGCRoute& path, Waypoints& waypoints);


// converting between Overbot XY coordinates and OpenSteer Vec3s
inline Vec3 convertOverbotXYtoVec3 (double x, double y) {return Vec3 (-x, 0.0f, y);}
inline double convertVec3toOverbotX (const Vec3& v) {return -v.x;}
inline double convertVec3toOverbotY (const Vec3& v) {return v.z;}


// ----------------------------------------------------------------------------
// for redefine some global constants previously in Draw.h


#ifdef NOT_OPENSTEERDEMO  // when NOT building OpenSteerDemo plug-in

const Vec3 gBlack   (0, 0, 0);
const Vec3 gWhite   (1, 1, 1);

const Vec3 gRed     (1, 0, 0);
const Vec3 gYellow  (1, 1, 0);
const Vec3 gGreen   (0, 1, 0);
const Vec3 gCyan    (0, 1, 1);
const Vec3 gBlue    (0, 0, 1);
const Vec3 gMagenta (1, 0, 1);

const Vec3 gOrange (1, 0.5f, 0);

inline Vec3 grayColor (const float g) {return Vec3 (g, g, g);}

const Vec3 gGray10 = grayColor (0.1f);
const Vec3 gGray20 = grayColor (0.2f);
const Vec3 gGray30 = grayColor (0.3f);
const Vec3 gGray40 = grayColor (0.4f);
const Vec3 gGray50 = grayColor (0.5f);
const Vec3 gGray60 = grayColor (0.6f);
const Vec3 gGray70 = grayColor (0.7f);
const Vec3 gGray80 = grayColor (0.8f);
const Vec3 gGray90 = grayColor (0.9f);

#endif // NOT_OPENSTEERDEMO

// ----------------------------------------------------------------------------
#endif // OOU_H
