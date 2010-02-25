#include "../planner/dpg/proto/DpgCell.h"
#include "Graphic-pkg.h"

using namespace std;

DpgCell::DpgCell(int i, Vector3d corner, API::ThreeDGrid* grid): API::ThreeDCell(i, corner, grid){
  _valid = 1;
  _edges.clear();
  _nodes.clear();
  _cellSize = grid->getCellSize();
  _visited = 0;
}

void DpgCell::draw(int color, int width){
  Vector3d corner = this->getCorner();
  double xmin = corner[0], xmax = corner[0] + _cellSize[0], ymin = corner[1], ymax = corner[1] + _cellSize[1], zmin = corner[2], zmax = corner[2] + _cellSize[2];
  g3d_draw_simple_box(xmin, xmax, ymin, ymax, zmin, zmax, color, 0, width);
}
void DpgCell::draw(){
  if(this->_valid){
    draw(Blue, 1);
  }else{
    draw(Red, 1);
  }
}