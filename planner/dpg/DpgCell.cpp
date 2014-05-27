/*
 * Copyright (c) 2001-2014 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 */
#include "DpgCell.h"
#include "Graphic-pkg.h"

using namespace std;
// import most common Eigen types 
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

DpgCell::DpgCell(int i, Eigen::Vector3d corner, Move3D::ThreeDGrid* grid): Move3D::ThreeDCell(i, corner, grid){
  _valid = 1;
  _edges.clear();
  _nodes.clear();
  _cellSize = grid->getCellSize();
  _visited = 0;
}

void DpgCell::draw(int color, int width){
  Eigen::Vector3d corner = this->getCorner();
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