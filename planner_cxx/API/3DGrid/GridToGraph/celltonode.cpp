#include "celltonode.h"

CellToNode::CellToNode()
{

}

CellToNode::CellToNode(int i, Vector3d corner, GridToGraph* grid) :
        Cell(i,corner,grid),
        _CellHasNode(false)
{

}

CellToNode::~CellToNode()
{

}
