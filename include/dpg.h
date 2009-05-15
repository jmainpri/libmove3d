#ifndef _DPG_H
#define _DPG_H

#ifdef DPG

typedef struct dpgGrid{
  double cellSize;
  double originPos[3];
  int nbCells;
  int nbCellsX;
  int nbCellsY;
  int nbCellsZ;
  struct dpgCell ** cells;
} p3d_dpgGrid;

typedef struct dpgCell{
  int id;
  int nbEdges;
  p3d_list_edge *edges;
  int valid; //there is no static obstacles crossing this cell
} p3d_dpgCell;
#endif//DPG

#endif
