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
#ifndef __DPGGRID_H__
#define __DPGGRID_H__
#include "P3d-pkg.h"
#include "DpgCell.h"
#include <vector>
#include "ThreeDGrid.h"

class DpgGrid : public Move3D::ThreeDGrid{
  public:  
    //constructors and destructors
    DpgGrid(p3d_env* env);
    virtual ~DpgGrid();

    //setters and getters
    inline int getNbCells(void){return _nbCells;}
    inline int getNbCellsOverX(void){return _nbCellsX;}
    inline int getNbCellsOverY(void){return _nbCellsY;}
    inline int getNbCellsOverZ(void){return _nbCellsZ;}

    //functions
    void init(void);
    void updateRobotOccupationCells(p3d_rob* robot);
    std::vector<DpgCell*> getCellListForObject(p3d_obj* obj, p3d_matrix4 pointTransform);
    void unvalidObjectCells(p3d_obj* obj);
    void draw();
  protected:
  virtual Move3D::ThreeDCell* createNewCell(unsigned int index, unsigned int x, unsigned int y, unsigned int z );
  private:
    p3d_env* _env;
    //The position of the origin of the grid regarding th eorigin of the world
    int _nbMaxCells; //the number of cell along the longest axis of the environment
    int _nbCells;
};

#endif
