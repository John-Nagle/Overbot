#include "wayptserver.h"
#include "inpoly.h"
#include <math.h>

// compute distance from a point to a line segment
//
// Algorithm from Dan Cornford (d.cornford@aston.ac.uk)

double
distToLine(tPointd q, Waypoint *w1p, Waypoint *w2p)
{
    double xp = q[0];
    double yp = q[1];
    double x1 = w1p->x;
    double y1 = w1p->y;
    double x2 = w2p->x;
    double y2 = w2p->y;

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

int
main(int argc, char *argv[])
{

	Waypoint w1, w2;
	tPointd q;

	printf("Enter 1st end point of line segment: ");
	scanf("%lf %lf", &w1.x, &w1.y);
	printf("Enter 2nd end point of line segment: ");
	scanf("%lf %lf", &w2.x, &w2.y);
	printf("Enter boundary: ");
	scanf("%f", &w2.boundary);

	while (true) {
	    printf("Enter point: ");
	    scanf("%lf %lf", &q[0], &q[1]);
	    printf("dist = %f\n", distToLine(q, &w1, &w2));
	}
}
