/*
This code is described in "Computational Geometry in C" (Second Edition),
Chapter 7.  It is not written to be comprehensible without the 
explanation in that book.

This program reads a polygon P followed by query points from stdin.
The input format is:
	n
	x0 y0
	x1 y1 
	...
	xn-1 yn-1
	qx qy
	qx qy
	qx qy
	...
For each query point q, InPoly returns one of four char's:
	i : q is strictly interior to P
	o : q is strictly exterior to P
	v : q is a vertex of P
	e : q lies on the relative interior of an edge of P
These represent mutually exclusive categories.
For an explanation of the code, see Chapter 7 of 
"Computational Geometry in C (Second Edition)."

Written by Joseph O'Rourke, contributions by Min Xu, June 1997.
Questions to orourke@cs.smith.edu.
--------------------------------------------------------------------
This code is Copyright 1998 by Joseph O'Rourke.  It may be freely 
redistributed in its entirety provided that this copyright notice is 
not removed.
--------------------------------------------------------------------
*/

#define DIM     2               /* Dimension of points */
typedef int     tPointi[DIM];   /* type integer point */
typedef double  tPointd[DIM];   /* type double point */
#define PMAX    4               /* Max # of pts in polygon */

typedef tPointi tPolygoni[PMAX];/* type integer polygon */
typedef tPointd tPolygond[PMAX];/* type integer polygon */

char	InPoly( tPointd q, tPolygond P, int n );
