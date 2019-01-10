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
#include	<stdio.h>
#include	<math.h>
#include 	"inpoly.h"
#define	X	0
#define	Y	1
//typedef	enum { FALSE, TRUE }	bool;

typedef tPointi tPolygoni[PMAX];/* type integer polygon */
typedef tPointd tPolygond[PMAX];/* type integer polygon */

void	PrintPoly( int n, tPolygond P );
void	PrintPoint( tPointd p );
void    Copy ( tPolygond a, tPolygond b , int n );
int 	ReadPoly( tPolygond P );

bool verbose = false;

#ifdef STANDALONE
int
main(int argc, char *argv[])
{
    int		n;
    tPolygond	P, Porig;
    tPointd	q;

    verbose = true;
    n = ReadPoly( P );
    Copy( P, Porig, n );

    while( scanf( "%lf %lf", &q[X], &q[Y]) != EOF )
    {
        printf( "InPoly (%3f, %3f) = %c\n", q[X], q[Y], InPoly( q, P, n ) );
        /* Refill the destroyed polygon with original. */
        Copy( Porig, P, n );
    }
}
#endif

/*
InPoly returns a char in {i,o,v,e}.  See above for definitions.
*/
char InPoly( tPointd q, tPolygond P, int n )
{
    int	 i, i1;      /* point index; i1 = i-1 mod n */
    int	 d;          /* dimension index */
    double x;          /* x intersection of e with ray */
    int	 Rcross = 0; /* number of right edge/ray crossings */
    int    Lcross = 0; /* number of left edge/ray crossings */

    if (verbose)
    {
        printf("\n==>InPoly: q = ");
        PrintPoint(q);
        putchar('\n');
    }

    /* Shift so that q is the origin. Note this destroys the polygon.
       This is done for pedogical clarity. */
    for( i = 0; i < n; i++ )
    {
        for( d = 0; d < DIM; d++ )
            P[i][d] = P[i][d] - q[d];
    }

    /* For each edge e=(i-1,i), see if crosses ray. */
    for( i = 0; i < n; i++ )
    {
        /* First see if q=(0,0) is a vertex. */
        if ( P[i][X]==0 && P[i][Y]==0 )
            return 'v';
        i1 = ( i + n - 1 ) % n;
        if (verbose)
            printf("e=(%d,%d)\t", i1, i);

        /* if e "straddles" the x-axis... */
        /* The commented-out statement is logically equivalent to the one
           following. */
        /* if( ( ( P[i][Y] > 0 ) && ( P[i1][Y] <= 0 ) ) ||
           ( ( P[i1][Y] > 0 ) && ( P[i] [Y] <= 0 ) ) ) { */

        if( ( P[i][Y] > 0 ) != ( P[i1][Y] > 0 ) )
        {

            /* e straddles ray, so compute intersection with ray. */
            x = (P[i][X] * (double)P[i1][Y] - P[i1][X] * (double)P[i][Y])
                / (double)(P[i1][Y] - P[i][Y]);
            if (verbose)
                printf("straddles: x = %g\t", x);

            /* crosses ray if strictly positive intersection. */
            if (x > 0)
                Rcross++;
        }
        if (verbose)
            printf("Right cross=%d\t", Rcross);

        /* if e straddles the x-axis when reversed... */
        /* if( ( ( P[i] [Y] < 0 ) && ( P[i1][Y] >= 0 ) ) ||
           ( ( P[i1][Y] < 0 ) && ( P[i] [Y] >= 0 ) ) )  { */

        if ( ( P[i][Y] < 0 ) != ( P[i1][Y] < 0 ) )
        {

            /* e straddles ray, so compute intersection with ray. */
            x = (P[i][X] * (double)P[i1][Y] - P[i1][X] * (double)P[i][Y])
                / (double)(P[i1][Y] - P[i][Y]);
            if (verbose)
                printf("straddles: x = %g\t", x);

            /* crosses ray if strictly negative intersection. */
            if (x < 0)
                Lcross++;
        }
        if (verbose)
            printf("Left cross=%d\n", Lcross);
    }

    /* q on the edge if left and right cross are not the same parity. */
    if( ( Rcross % 2 ) != (Lcross % 2 ) )
        return 'e';

    /* q inside iff an odd number of crossings. */
    if( (Rcross % 2) == 1 )
        return 'i';
    else
        return 'o';
}

void PrintPoint( tPointd p )
{
    int	i;

    putchar('(');
    for ( i = 0; i < DIM; i++ )
    {
        printf("%f", p[i]);
        if ( i != DIM-1 )
            putchar(',');
    }
    putchar(')');
}

/*
   Reads in the coordinates of the vertices of a polygon from stdin,
   puts them into P, and returns n, the number of vertices.
   Formatting conventions: etc.
   */
int ReadPoly( tPolygond P )
{
    int	i, n;

    do
    {
        printf( "Input the number of vertices:\n");
        scanf( "%d", &n );
        if ( n <= PMAX )
            break;
        printf("Error in read_poly:  too many points; max is %d\n", PMAX);
    }
    while ( 1 );

    printf( "Polygon:\n" );
    printf( "   i   x   y\n");
    for ( i = 0; i < n; i++ )
    {
        scanf( "%lf %lf", &P[i][0], &P[i][1] );
        printf("%3d %4f %4f\n", i, P[i][0], P[i][1]);
    }
    printf("n = %d vertices read\n",n);
    putchar('\n');

    return n;
}

void PrintPoly( int n, tPolygond P )
{
    int	i;

    printf("Polygon:\n");
    printf("  i   x   y\n");
    for( i = 0; i < n; i++ )
        printf("%3d %f %f\n", i, P[i][0], P[i][1]);
}

/* Copy polygon a to b (overwriting b). */
void Copy( tPolygond a, tPolygond b, int n )
{
    int i, j;

    for ( i=0; i < n; i++)
        for ( j = 0; j < DIM; j++ )
            b[i][j] = a[i][j];
}

bool EqPoint( tPointd a, tPointd b )
{
    int     i;
    for ( i = 0; i < DIM; i++ )
        if ( a[i] != b[i])
            return  false;
    return  true;
}

