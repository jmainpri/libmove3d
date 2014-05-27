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
#ifndef __DPGCELL_H__
#define __DPGCELL_H__
#include <vector>
#include "Planner-pkg.h"
#include "ThreeDCell.h"
#include "ThreeDGrid.h"

class DpgCell : public Move3D::ThreeDCell{
  public:
    DpgCell(int i, Eigen::Vector3d corner, Move3D::ThreeDGrid* grid);
    //setters and getters
    inline int isValid(void){return _valid;}
    inline void setValid(int value){_valid = value;}
    inline int isVisited(void){return _visited;}
    inline void setVisited(int value){_visited = value;}
    void draw(int color, int width);
    void draw(void);
    
  private:
    std::vector<p3d_edge*> _edges;
    std::vector<p3d_node*> _nodes;
    int _valid; //There is no static obstacles crossing this cell
    int _visited;
    Eigen::Vector3d _cellSize;
};

#endif
