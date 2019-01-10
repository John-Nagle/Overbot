/**
  This module performs generic raster operations on a MapServer by
  calling updateCell().

  Paul Upchurch
  Team Overbot
  2005-04

*/
#include"mapserver.h"
#include"mapraster.h"

/* ----------------------------------------------------------- */
/* polygon rasterizing code, internal use only */
/* ----------------------------------------------------------- */

/*
  Most of these functions are generic graphics primitive scanline
  rasterizers.
*/

void write_cell(int x, int y, const RASTER_CTX &ctx) {
  bool sweeping = false;														// don't trust as if sweeping, for now.
  TerrainMap& themap = ctx.map->getMap();
  if(ctx.forcetogood || themap.unknownCell(x,y) || themap.possibleCell(x,y)) {
    ctx.map->updateCell(x,y,ctx.newtype, sweeping, ctx.minrange,ctx.roughness,ctx.elev,ctx.cyclestamp);
  }
}

static inline void write_hline0_cell(int x, int y, int len, const RASTER_CTX &ctx) {
  int i;

  for(i=0;i<len;i++,x++) {
    write_cell(x,y,ctx);	// use common update routine in all cases	
  }
}

void write_hline_cell(int x1, int x2, int y, const RASTER_CTX &ctx) {
  if(x1<x2) write_hline0_cell(x1,y,x2-x1+1,ctx);
  else write_hline0_cell(x2,y,x1-x2+1,ctx);
}

void write_line_cell(int x1, int y1, int x2, int y2, const RASTER_CTX &ctx) {
  int dx,dy;
  int xunit,yunit;
  int term;

  dx=x2-x1;
  dy=y2-y1;
  if(dx<0) { dx=(-dx); xunit=(-1); }
  else xunit=1;
  if(dy<0) { dy=(-dy); yunit=(-1); }
  else yunit=1;
  term=0;
  if(dx>=dy) {
    while(x1!=x2) {
      write_cell(x1,y1,ctx);
      term+=dy;
      if(term>=dx) {
        term-=dx;
        y1+=yunit;
      }
      x1+=xunit;
    }
    write_cell(x1,y1,ctx);
  }
  else {
    while(y1!=y2) {
      write_cell(x1,y1,ctx);
      term+=dx;
      if(term>=dy) {
        term-=dy;
        x1+=xunit;
      }
      y1+=yunit;
    }
    write_cell(x1,y1,ctx);
  }
}

/**
  Most of the "dead" code in here is to handle degenerate polygons.
  Be very careful when changing this code since improperly handled
  degenerates tend to create wildly wrong results or infinite loops.
*/
void write_ordered_poly_cell(ivect2 *verts, int numverts, const RASTER_CTX &ctx) {
  int i,j,py,bottom;
  int vert1=0,dx1=0,dy1=0,xunit1=0,term1=0,togo1=0;
  int vert2=0,dx2=0,dy2=0,xunit2=0,term2=0,togo2=0;
  int topv;
  int px1,px2;

  term1=1;
  bottom=verts[0].y;
  topv=0;
  py=bottom;
  for(i=1;i<numverts;i++) {
    if(verts[i].y>bottom) bottom=verts[i].y;
    if(verts[i].y<py) {
      py=verts[i].y;
      topv=i;
    }
    for(j=0;j<i && !(verts[j].x==verts[i].x && verts[j].y==verts[i].y);j++);
    if(j==i) term1++;
  }
  if(term1==1) {
    write_cell(verts[0].x,verts[0].y,ctx);
    return;
  }
  if(term1==2) {
    for(i=1;i<numverts;i++) {
      for(j=0;j<i && !(verts[j].x==verts[i].x && verts[j].y==verts[i].y);j++);
      if(j==i) {
        write_line_cell(verts[0].x,verts[0].y,verts[i].x,verts[i].y,ctx);
        return;
      }
    }
    return;
  }
  if(py==bottom) {
    px1=verts[0].x;
    for(i=1;i<numverts;i++) px1=(px1<verts[i].x?px1:verts[i].x);
    px2=verts[0].x;
    for(i=1;i<numverts;i++) px2=(px2>verts[i].x?px2:verts[i].x);
    write_hline_cell(px1,px2,bottom,ctx);
    return;
  }

  px1=verts[topv].x;
  px2=px1;

  vert1=topv;
  togo1=0;
  vert2=topv;
  togo2=0;

  for(;py<=bottom;) {
    while(togo1<1 && py<bottom) {
      if(px1!=verts[vert1].x) return;
      vert1=vert1-1;
      if(vert1<0) vert1=numverts-1;
      dy1=verts[vert1].y-py;
      togo1=dy1;
      if(togo1<1) {
        px1=verts[vert1].x;
        write_hline_cell(px1,px2,py,ctx);
        continue;
      }
      dx1=verts[vert1].x-px1;
      if(dx1<0) { dx1=-dx1; xunit1=(-1); }
      else xunit1=1;
      term1=0;
    }
    while(togo2<1 && py<bottom) {
      if(px2!=verts[vert2].x) return;
      vert2=vert2+1;
      if(vert2>numverts-1) vert2=0;
      dy2=verts[vert2].y-py;
      togo2=dy2;
      if(togo2<1) {
        px2=verts[vert2].x;
        write_hline_cell(px1,px2,py,ctx);
        continue;
      }
      dx2=verts[vert2].x-px2;
      if(dx2<0) { dx2=-dx2; xunit2=(-1); }
      else xunit2=1;
      term2=0;
    }
    write_hline_cell(px1,px2,py,ctx);
    if(togo1>0 && py<bottom) {
      term1+=dx1;
      while(term1>=dy1) {
        term1-=dy1;
        px1+=xunit1;
      }
      togo1--;
    }
    if(togo2>0 && py<bottom) {
      term2+=dx2;
      while(term2>=dy2) {
        term2-=dy2;
        px2+=xunit2;
      }
      togo2--;
    }
    py++;
  }
}

