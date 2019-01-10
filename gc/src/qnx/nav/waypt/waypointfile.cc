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
//
// Local function declarations
//
static bool validate(const char *str, const char *validStr);

// convert feet to meters
inline float
feet2meters(float s)
{
    return ((s) * 12.0 * 2.54 / 100.);
}

// convert mph to meters/s
inline float
mph2metric(float s)
{
    return ((s) * 5280.0 * 12.0 / 3600.0 * 2.54 / 100.);
}
// read in waypoint list
int
WayptServer::readWaypts(const char *fileName)
{

    char buf[BUFSIZ + 1];
    char *s;
    char *sep = ", \n\r";		// field separator characters
    int line = 0;
    RDDF rddf;
    Waypoint waypt;
    FILE *fp = stdin;
    bool first = true;
    time_t ltime;
    struct tm lctime;
    struct tm *lctimep;
    Vector<3> llhOrigin;
    Vector<3> llh;
    Vector<3> ned;
    int errors = 0;


    ltime = time(NULL);
    lctimep = localtime(&ltime);
    lctime = *lctimep;


    // if file name was given, open it
    if (fileName != NULL)
    {
        if ((fp = fopen(fileName, "r")) == NULL)
        {
            perror(fileName);
            exit(1);
        }
    }

    //
    // read waypoints
    //

    while (!feof(fp))
    {

        // read one line
        buf[0] = '\0';
        if (fgets(buf, BUFSIZ, fp) == NULL)
        {
            perror("readWaypts");
            return -1;
        }

        ++line;

        //
        // extract waypt number
        //

        s = strtok(buf, sep);

        // blank line -- skip
        if (s == NULL)
        {
            return -1;
        }

        std::string str(s);

        if (!validate(s, "0123456789"))
        {
            fprintf(stderr, "waypt: bad number at line %d\n", line);
            errors++;
            continue;	// skip line
        }

        rddf.number = atoi(s);
        waypt.number = rddf.number;

        if (m_verbose)
        {
            printf("read waypt number %d\n", rddf.number);
        }

        //
        // extract latitude
        //

        s = strtok(NULL, sep);
        if (s == NULL)
        {
            fprintf(stderr, "waypt: no latitude in line %d\n", line);
            errors++;
            continue;	// skip line
        }

        if (!validate(s, "-.0123456789"))
        {
            fprintf(stderr, "waypt: bad latitude at line %d\n", line);
            errors++;
            continue;	// skip line
        }

        rddf.latitude = strtod(s, NULL);
        waypt.latitude = rddf.latitude;
        if (!first)
        {
            llh[0] = waypt.latitude;
            llh[1] = waypt.longitude;
            llh[2] = 0.0;
            ned = LLH_dist(llhOrigin, llh, waypt.latitude, waypt.longitude);
            waypt.y = ned[0];
            waypt.x = ned[1];
        }

        //
        // extract longitude
        //

        s = strtok(NULL, sep);
        if (s == NULL)
        {
            fprintf(stderr, "waypt: no longitude at line %d\n", line);
            errors++;
            continue;	// skip line
        }

        if (!validate(s, "-.0123456789"))
        {
            fprintf(stderr, "waypt: bad longitude at line %d\n", line);
            errors++;
            continue;	// skip line
        }

        rddf.longitude = strtod(s, NULL);
        waypt.longitude = rddf.longitude;

        //
        // extract lateral boundary
        //

        s = strtok(NULL, sep);
        if (s == NULL)
        {
            fprintf(stderr, "waypt: no boundary at line %d\n", line);
            errors++;
            continue;	// skip line
        }

        if (!validate(s, ".0123456789"))
        {
            fprintf(stderr, "waypt: bad boundary at line %d\n", line);
            errors++;
            continue;	// skip line
        }

        rddf.boundary = atof(s);
        waypt.boundary = feet2meters(rddf.boundary);

        //
        // extract speed limit
        //

        s = strtok(NULL, sep);
        if (s == NULL)
        {
            fprintf(stderr, "waypt: no limit at line %d\n", line);
            errors++;
            continue;	// skip line
        }

        if (!validate(s, ".0123456789"))
        {
            fprintf(stderr, "waypt: bad limit at line %d\n", line);
            errors++;
            continue;	// skip line
        }

        rddf.limit = atof(s);
        waypt.speedLimit = mph2metric(rddf.limit);

        //
        // extract phase line hour
        //

        s = strtok(NULL, sep);
        if (s == NULL)
        {
            fprintf(stderr, "waypt: no hour at line %d\n", line);
            errors++;
            continue;	// skip line
        }

        if (!validate(s, "#0123456789"))
        {
            fprintf(stderr, "waypt: bad hour at line %d\n", line);
            errors++;
            continue;	// skip line
        }

        bool noTime = false;

        if (*s == '#')
        {
            rddf.phaseLineHour = -1;
            noTime = true;
        }
        else
        {
            rddf.phaseLineHour = atoi(s);
        }

        //
        // extract phase line minute
        //

        s = strtok(NULL, sep);
        if (s == NULL)
        {
            fprintf(stderr, "waypt: no minute at line %d\n", line);
            errors++;
            continue;	// skip line
        }

        if (!validate(s, "#0123456789"))
        {
            fprintf(stderr, "waypt: bad minute at line %d\n", line);
            errors++;
            continue;	// skip line
        }

        if (*s == '#')
        {
            rddf.phaseLineMinute = -1;
            noTime = true;
        }
        else
        {
            rddf.phaseLineMinute = atoi(s);
        }

        //
        // extract phase line second
        //

        s = strtok(NULL, sep);
        if (s == NULL)
        {
            fprintf(stderr, "waypt: no second at line %d\n", line);
            errors++;
            continue;	// skip line
        }

        if (!validate(s, "#0123456789"))
        {
            fprintf(stderr, "waypt: bad second at line %d: %s\n", line, s);
            errors++;
            continue;	// skip line
        }

        if (*s == '#')
        {
            rddf.phaseLineSecond = -1;
            noTime = true;
        }
        else
        {
            rddf.phaseLineSecond = atoi(s);
        }

        if (noTime)
        {
            waypt.deadline = 0;
        }
        else
        {
            lctime.tm_hour = rddf.phaseLineHour;
            lctime.tm_min = rddf.phaseLineMinute;
            lctime.tm_sec = rddf.phaseLineSecond;
            waypt.deadline = mktime(&lctime);
        }


        if (m_verbose)
        {
            printf("\tlat=%10.7f,lon=%10.7f,lb=%f,sl=%f,pl=%d:%d:%d\n",
                   rddf.latitude,
                   rddf.longitude,
                   rddf.boundary,
                   rddf.limit,
                   rddf.phaseLineHour,
                   rddf.phaseLineMinute,
                   rddf.phaseLineSecond);
        }

        //
        // Save waypoint in list
        //

        if (first)
        {
            waypt.x = 0.0;
            waypt.y = 0.0;
            m_firstWaypoint = waypt;
            first = false;
            llhOrigin[0] = m_firstWaypoint.latitude;
            llhOrigin[1] = m_firstWaypoint.longitude;
            llhOrigin[2] = 0.0;
        }

        m_waypointList.push_back(waypt);	// add to list

    }

    return errors;

}

// validate a string
bool validate(const char *str, const char *validStr)
{
    while (*str != '\0')
    {
        if (index(validStr, *str++) == NULL)
        {
            fprintf(stderr, "validate: valid='%s', c='%d'\n", validStr, *--str);
            return false;
        }
    }

    return true;
}
