#ifndef CELLTONODE_H
#define CELLTONODE_H

#include "../cell.h"
#include "gridtograph.h"


class CellToNode : public Cell
{
public:
    CellToNode();
    CellToNode(int i, std::vector<double> corner, GridToGraph* grid);

    ~CellToNode();

    bool cellHasNode() { return _CellHasNode; }

    void setNode(Node* N) {_Node = N; _CellHasNode = true; }

    Node* getNode() { return _Node; }


private:
    bool _CellHasNode;
    Node* _Node;
};

#endif // CELLTONODE_H
