#ifndef CELL_HPP
#define CELL_HPP

#include "../../../other_libraries/Eigen/Core"

USING_PART_OF_NAMESPACE_EIGEN

namespace API
{
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
        Vector3d getRandomPoint();

        int getIndex() { return _index; }

        bool operator==( Cell otherCell) { return ((otherCell._index) == (this->_index)); }

    protected:
        int _index;
        Vector3d _corner;
        Grid* _grid;
    };

}
#endif // CELL_HPP
