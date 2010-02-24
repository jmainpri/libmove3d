#ifndef CELL_HPP
#define CELL_HPP

#include "../../../other_libraries/Eigen/Core"

#include "BaseCell.hpp"

USING_PART_OF_NAMESPACE_EIGEN

/**
  * @ingroup CPP_API
  * @defgroup GRID Grid over the WS
  */

/**
  @ingroup GRID
  */
namespace API
{
    class ThreeDGrid;

    class ThreeDCell : public BaseCell
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        ThreeDCell();
        ThreeDCell(int i, Vector3d corner, ThreeDGrid* grid);
        virtual ~ThreeDCell();

        bool isInsideCell(Vector3d point);

        Vector3d getCenter();
        Vector3d getCorner() { return _corner; }
        Vector3d getRandomPoint();
        Vector3d getCellSize();

        int getIndex() { return _index; }

        virtual void draw();

        bool operator==( ThreeDCell otherCell) { return ((otherCell._index) == (this->_index)); }

    protected:
        int _index;
        Vector3d _corner;
        ThreeDGrid* _grid;
    };

}
#endif // CELL_HPP