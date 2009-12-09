#ifndef CELL_HPP
#define CELL_HPP

#include "../../other_libraries/Eigen/Core"

USING_PART_OF_NAMESPACE_EIGEN

class Grid;

class Cell
{
public:
    Cell();
    Cell(int i, Vector3d corner, Grid* grid);
    virtual ~Cell();

    bool isInsideCell(Vector3d point);
    Vector3d getCenter();
    Vector3d getCorner() { return _corner; }
    int getIndex() { return _index; }

protected:
    int _index;
    Vector3d _corner;
    Grid* _grid;
};

#endif // CELL_HPP
