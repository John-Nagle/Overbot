////////////////////////////////////////////////////////////////////////////////
//
//    File:
//      wayptserver.cc
//
//    Usage:
//
//	In the watchdog startup file (startfile.txt):
//		ID=WAYPT waypt
//
//    Description:
//
//
//	The Waypt server consists of the following threads:
//          	- a main thread, which accepts messages
//
//    Messages received:
//
//	WayptServer::GetFirstWaypoint
//	WayptServer::GetCurrentWaypoint
//
//    Messages sent:
//
//
//    Written By:
//
//      Achut Reddy
//      Team Overbot
//      January 2004
//
////////////////////////////////////////////////////////////////////////////////


#include <unistd.h>
#include <stdio.h>
#include <string>
#include <strings.h>
#include <ctype.h>
#include <errno.h>
#include <math.h>

#include "Nav.h"
#include "Vector.h"
#include "inpoly.h"
#include "wayptserver.h"

using namespace std;

// convert feet to meters
#define FEET2METERS(s)	((s) * 12.0 * 2.54 / 100.)

// convert mph to meters/s
#define MPH2METRIC(s)	((s) * 5280.0 * 12.0 / 3600.0 * 2.54 / 100.)

// Local function declarations

static double dist(tPointd q, const Waypoint& w1);
static double distToLine(tPointd q, const Waypoint& w1p, const Waypoint& w2p);


//
// Class member functions
//

// constructor
WayptServer::WayptServer()
        :	m_serverPort(0.0),
        m_verbose(false)
{
    // initialize server messaging
    int stat = m_serverPort.ChannelCreate();	// create a channel, tell watchdog
    if  (stat )
    {
        perror("WayptServer::WayptServer - ChannelCreate failed.\n");
        m_serverPort.Dump();
    }

    // initialize server state

    // initialize server data

};

// destructor
WayptServer::~WayptServer()
{}

void
WayptServer::messageThread()
{

    // loop collecting messages forever
    while (true)
    {
        WayptServerMsg::MsgUnion	msg;	// area for incoming and outgoing msgs

        // wait for message
        int rcvid = m_serverPort.MsgReceive(msg);

        // handle errors
        if ( rcvid < 0 )
        {
            fflush(stdout);
            perror("WayptServer::messageThread - MsgReceive failed\n");
            sleep(1);		// avoid tight loop if repeated trouble FIX
            continue;
        }
        else if ( rcvid == 0 )
        {
            perror("WayptServer::messageThread - received a pulse\n");

            // pulses don't require a reply
            continue;
        }

        if (m_verbose )
        {
            //perror("WayptServer::messageThread - received a message\n");
        }

        // handle message
        WayptServer::handleMessage(rcvid, msg);

    }
}

// Handle messages
void
WayptServer::handleMessage(int rcvid, const WayptServerMsg::MsgUnion& msg)
{
    // Dispatch on message type
    switch ( msg.m_header.m_msgtype )
    {

    case WayptServerMsg::GetNumberOfWaypoints::k_msgtype:// GetNumberOfWaypoints msg
        handleGetNumberOfWaypoints(rcvid, msg.m_getnumberofwaypoints);
        break;

    case WayptServerMsg::GetWaypoint::k_msgtype:	// GetWaypoint msg
        handleGetWaypoint(rcvid, msg.m_getwaypoint);
        break;

    case WayptServerMsg::GetWaypointByXY::k_msgtype:	// GetWaypointByXY msg
        handleGetWaypointByXY(rcvid, msg.m_getwaypointbyxy);
        break;

    default:
        fprintf(stderr, "WayptServer::handleMessage - unknown message type: %x\n",
                msg.m_header.m_msgtype);
        MsgError(rcvid, EBADMSG);
        break;
    }

}

// handle GetNumberOfWaypoints msg
void
WayptServer::handleGetNumberOfWaypoints(int rcvid, const WayptServerMsg::GetNumberOfWaypoints& msg)
{
    WayptServerMsg::GetNumberOfWaypoints reply = msg;
    if (m_waypointList.empty())
    {
        reply.number = 0;
        // reply to sender
        int err = MsgReply(rcvid, WayptServerMsg::EMPTY, msg);
        if ( err )
        {
            perror("WayptServer::messageThread - MsgReply failed\n");
            exit(1);
        }
        return;
    }

    reply.number = m_waypointList.size();

    int err = MsgReply(rcvid, WayptServerMsg::OK, reply);
    if ( err )
    {
        perror("WayptServer: MsgReply failed\n");
        exit(1);
    }

}

// handle GetWaypoint msg
void
WayptServer::handleGetWaypoint(int rcvid, const WayptServerMsg::GetWaypoint& msg)
{
    WayptServerMsg::GetWaypoint reply = msg;
    if (m_waypointList.empty())
    {
        // reply to sender
        int err = MsgReply(rcvid, WayptServerMsg::EMPTY, reply);
        if ( err )
        {
            perror("WayptServer::messageThread - MsgReply failed\n");
            exit(1);
        }
        return;
    }

    // return the specified waypoint
    if (msg.index >= int(m_waypointList.size()))
    {
        MsgError(rcvid,EINVAL);							// out of range
        return;
    }
    reply.waypt = m_waypointList[msg.index];

    // reply to sender
    int err = MsgReply(rcvid, WayptServerMsg::OK, reply);
    if ( err )
    {
        perror("WayptServer: MsgReply failed\n");
    }

}


// handle GetWaypointByXY msg
void
WayptServer::handleGetWaypointByXY(int rcvid, const WayptServerMsg::GetWaypointByXY& msg)
{
    if (m_waypointList.empty() || (m_waypointList.size() <= 2))
    {
        // reply to sender
        int err = MsgReply(rcvid, WayptServerMsg::EMPTY, msg);
        if ( err )
        {
            perror("WayptServer::messageThread - MsgReply failed\n");
            exit(1);
        }
        return;
    }
    WayptServerMsg::GetWaypointByXY reply = msg;							// reply area
    Waypoint w1, w2;
    tPolygond wRect;
    double x1, x2, y1, y2;
    double mag;
    double margin = msg.margin;
    tPointd q;

    q[0] = msg.x;
    q[1] = msg.y;

    int start = msg.startNum;
    int n = m_waypointList.size() - 1;

    for (int i=start; i < n; i++)
    {
        w1 = m_waypointList[i];
        w2 = m_waypointList[i+1];

        // given 2 adjacent waypoints, compute the waypoint corridor.
        // the waypoint corridor is shaped like a rectangle with
        // semicircular caps on both ends.

        // first check if the point is in the rectangle part;
        // if that fails, check the two circles at the endpoints

        mag = sqrt(w1.x*w1.x + w1.y*w1.y);
        x1 = (w1.y / mag) * (w1.boundary / 2);
        y1 = (w1.x / mag) * (w1.boundary / 2);
        x2 = -x1;
        y2 = -y1;

        wRect[0][0] = x1;
        wRect[1][0] = y1;
        wRect[0][1] = x2;
        wRect[1][1] = y2;

        mag = sqrt(w2.x*w2.x + w2.y*w2.y);
        x1 = (w2.y / mag) * (w2.boundary / 2);
        y1 = (w2.x / mag) * (w2.boundary / 2);
        x2 = -x1;
        y2 = -y1;

        wRect[0][2] = x2;
        wRect[1][2] = y2;
        wRect[0][3] = x1;
        wRect[1][3] = y1;

        bool inside = false;

        char result = InPoly(q, wRect, 4);

        if (result != 'o')
        {
            inside = true;
        }
        else if (dist(q, w1) <= ((w1.boundary / 2) + margin))
        {
            inside = true;
        }
        else if (dist(q, w2) <= ((w1.boundary / 2) + margin))
        {
            inside = true;
        }
        else if (distToLine(q, w1, w2) <= (w1.boundary/2  + margin))
        {
            inside = true;
        }


        if (inside)
        {
            reply.waypt1 = m_waypointList[i];
            reply.waypt2 = m_waypointList[i+1];

            // reply to sender
            int err = MsgReply(rcvid, WayptServerMsg::OK, msg);
            if ( err )
            {
                perror("WayptServer: MsgReply failed\n");
                exit(1);
            }

            return;
        }
    }


    // point is not in any waypoint corridor

    reply.waypt1.number = -1;
    reply.waypt1.x = -1;
    reply.waypt1.y = -1;
    reply.waypt2.number = -1;
    reply.waypt2.x = -1;
    reply.waypt2.y = -1;

    // reply to sender
    int err = MsgReply(rcvid, WayptServerMsg::OOB, reply);
    if ( err )
    {
        perror("WayptServer: MsgReply failed\n");
        exit(1);
    }


}






//
//	dist  -- distance between two waypoints
//
static double dist(tPointd q, const Waypoint& w1p)
{
    double dx = w1p.x - q[0];
    double dy = w1p.y - q[1];
    return sqrt(dx*dx + dy*dy);
}

// compute distance from a point to a line segment
//
// Algorithm from Dan Cornford (d.cornford@aston.ac.uk)

static double distToLine(tPointd q, const Waypoint& w1p, const Waypoint& w2p)
{
    double xp = q[0];
    double yp = q[1];
    double x1 = w1p.x;
    double y1 = w1p.y;
    double x2 = w2p.x;
    double y2 = w2p.y;

    double dx1p = x1 - xp;
    double dx21 = x2 - x1;
    double dy1p = y1 - yp;
    double dy21 = y2 - y1;

    // compute distance along the line that the normal intersects
    double lambda = -(dx1p*dx21 + dy1p*dy21) / (dx21*dx21 + dy21*dy21);

    // accept if along the line segment, else choose the correct end point
    if (lambda < 0.0)
        lambda = 0.0;
    else if (lambda > 1.0)
        lambda = 1.0;

    // compute the x and y separations between the point on the line that is
    // closest to (xp,yp) and (xp,yp)
    double xsep = dx1p + lambda*dx21;
    double ysep = dy1p + lambda*dy21;

    return sqrt(xsep*xsep + ysep*ysep);
}
