#ifndef _x_MAPRASTER_INCLUDED__
#define _x_MAPRASTER_INCLUDED__

typedef struct { int x,y; } ivect2;
typedef struct {
  MapServer *map;
  CellData::CellType newtype;
  uint16_t minrange;
  uint8_t roughness;
  float elev;
  uint32_t cyclestamp;
  bool forcetogood;			// if set, force good, even if known. Only used under vehicle
} RASTER_CTX;

extern void write_cell(int x, int y, const RASTER_CTX &ctx);
extern void write_hline_cell(int x1, int x2, int y, const RASTER_CTX &ctx);
extern void write_line_cell(int x1, int y1, int x2, int y2, const RASTER_CTX &ctx);
extern void write_ordered_poly_cell(ivect2 *verts, int numverts, const RASTER_CTX &ctx);
extern void write_circle_cell(int cx, int cy, int rad, const RASTER_CTX &ctx);

#endif
