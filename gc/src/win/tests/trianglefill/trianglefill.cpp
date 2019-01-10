/* trifle.c - fill a triangle */

/*
 * Copyright Roger Willcocks, 2005
 * Permission is granted to copy and distribute this
 * work provided that this notice remains intact.
 * rkww - 11 Mar 2005
 */

typedef long longlong;	// ***TEMP*** should be int64_t for GCC compilers 

typedef struct _vertex {
    int x, y;
} vertex;


typedef struct _triangle {
    vertex v[3];
} triangle;


/* forward definition: paint a line span - caution x values unordered */
extern void span(int line, int x0, int x1, char c);

/* swap vertices p & q if necessary so y[p] <= y[q] */

inline int swapv(triangle& t, int p, int q)
{
    if (t.v[p].y > t.v[q].y) {
        vertex v = t.v[p];
        t.v[p] = t.v[q];
        t.v[q] = v;
        return 1;
    }
    return 0;
}


/*
 * Suppose a rectangular grid. Vertices lay at the centre
 * of grid rectangles. We 'cut' the plane along grid lines.
 * Life is made simpler by doubling the coordinates.
 */

typedef struct _edge {
    int xint, xfrac;
    int dxint, dxfrac;
    int dy, life;
} edge;


/* initialize DDA for edge p->q */

static edge initedge(triangle& t, int p, int q)
{
    longlong x; /* need long long intermediate for muldivmod */
 
    edge e;

    int dx = t.v[q].x - t.v[p].x;
    int dy = t.v[q].y - t.v[p].y;

    e.life = dy;
    e.dy = dy += dy;

    if (dy == 0)
        return e;

    /* initial intersection is x + (dx/dy) / 2 */

    x = t.v[p].x * (longlong)e.dy + dx;

    e.xint = (int)(x / dy);
    e.xfrac = (int)(x % dy);

    /* -3.5 = -4 + .5 */

    if (e.xfrac < 0) {
        e.xint--;
        e.xfrac += dy;
    }

    dx += dx;
    e.dxint = dx / dy;
    e.dxfrac = dx % dy;

    if (e.dxfrac < 0) {
        e.dxint--;
        e.dxfrac += dy;
    }

    return e;
}


/* advance x position by dx/dy avoiding division in this inner loop */

inline void advance(edge& e)
{
    e.xint += e.dxint;
    if ((e.xfrac += e.dxfrac) >= e.dy) {
        e.xfrac -= e.dy;
        e.xint++;
    }
    e.life--;
}


void filltriangle(triangle& t, char c)
{

    /* switch vertices around so they're in y order; overwrites
       source triangle. Bubble sort :-) */

    while (swapv(t, 0, 1) + swapv(t, 1, 2))
        ;

    /* set up DDA for first two edges */

    edge longedge = initedge(t, 0, 2);
    edge shortedge = initedge(t, 0, 1);

    /* draw upper triangle */

    int line = t.v[0].y;

    while (shortedge.life) {
        span(line++, longedge.xint, shortedge.xint, c);
        advance(longedge);
        advance(shortedge);
    }

    /* now set up DDA for third edge */

    shortedge = initedge(t, 1, 2);

    /* and draw lower triangle */

    while (longedge.life) {
        span(line++, longedge.xint, shortedge.xint, c);
        advance(longedge);
        advance(shortedge);
    }
}


/* test code from here on */

char plane[40 * 40];

void span(int line, int x0, int x1, char c)
{
    int i;

    if (x0 > x1) {
        int t = x0;
        x0 = x1;
        x1 = t;
    }

    for (i = x0; i < x1; i++) {
        char *p = &plane[40 * line + i];

        if (*p == ' ')
            *p = c;
        else
            *p = '$';
    }
}


#include <stdio.h>
#include <string.h>

int trianglefilltest()
{
    triangle t = { 20, 20, 30, 23, 23, 10 };
    triangle u = { 20, 20, 10, 23, 23, 10 };
    triangle v = { 20, 20, 23, 30, 30, 23 };
    triangle w = { 20, 20, 23, 30, 10, 23 };

    int i;

    memset(plane, ' ', sizeof(plane));

    filltriangle(t, '.');
    filltriangle(u, 'x');
    filltriangle(v, 'o');
    filltriangle(w, '~');

    for (i = 0; i < sizeof(plane); i++) {
        printf("%c", plane[i]);
        if (i % 40 == 39)
            printf("\n");
    }

    return 0;
}

/* end */
