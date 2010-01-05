#ifndef __DPGCELL_H__
#define __DPGCELL_H__
#include <vector>
#include "Planner-pkg.h"
#include "../planner_cxx/API/3DGrid/cell.h"
#include "../planner_cxx/API/3DGrid/grid.h"

class DpgCell : public API::Cell{
  public:
    DpgCell(int i, Vector3d corner, API::Grid* grid);
    //setters and getters
    inline int isValid(void){return _valid;}
    inline void setValid(int value){_valid = value;}
    inline int isVisited(void){return _visited;}
    inline void setVisited(int value){_visited = value;}
    void draw(void);
    
  private:
    std::vector<p3d_edge*> _edges;
    std::vector<p3d_node*> _nodes;
    int _valid; //There is no static obstacles crossing this cell
    int _visited;
    Vector3d _cellSize;
};

#endif
