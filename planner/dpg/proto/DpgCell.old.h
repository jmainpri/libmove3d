#ifndef __DPGCELL_H__
#define __DPGCELL_H__
#include <vector>
#include "Planner-pkg.h"

class DpgCell {
  public:
    //constructors and destructors
    DpgCell();
    DpgCell(int id);
    virtual ~DpgCell();
    //setters and getters
    inline void unvalid(void){_valid = 0;}
    inline void setValid(void){_valid = 1;}
  private:
    int _id;
    std::vector<p3d_edge*> _edges;
    std::vector<p3d_node*> _nodes;
    int _valid; //There is no static obstacles crossing this cell
};

#endif
