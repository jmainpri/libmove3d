#ifndef __DPGCELL_H__
#define __DPGCELL_H__
#include <vector>
#include "Planner-pkg.h"
#include "ThreeDCell.h"
#include "ThreeDGrid.h"

class DpgCell : public API::ThreeDCell{
  public:
    DpgCell(int i, Eigen::Vector3d corner, API::ThreeDGrid* grid);
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
