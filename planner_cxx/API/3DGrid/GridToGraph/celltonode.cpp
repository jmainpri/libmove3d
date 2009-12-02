#include "celltonode.h"

CellToNode::CellToNode()
{

}

CellToNode::CellToNode(int i, std::vector<double> corner, GridToGraph* grid) :
        Cell(i,corner,grid),
        _CellHasNode(false)
{

}

CellToNode::~CellToNode()
{

}
