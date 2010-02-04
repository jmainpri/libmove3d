#include "celltonode.h"

CellToNode::CellToNode()
{

}

CellToNode::CellToNode(int i, Vector3d corner, GridToGraph* grid) :
        ThreeDCell(i,corner,grid),
        _CellHasNode(false)
{

}

CellToNode::~CellToNode()
{

}
