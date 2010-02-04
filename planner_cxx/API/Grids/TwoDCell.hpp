#ifndef TWODCELL_HPP
#define TWODCELL_HPP

#include "../../../other_libraries/Eigen/Core"

#include "BaseCell.hpp"

USING_PART_OF_NAMESPACE_EIGEN

namespace API
{
    class TwoDGrid;
    /**
      * @ingroup CPP_API
      * @defgroup GRID Grid over the WS
      */

    /**
      @ingroup GRID
      */
    class TwoDCell : public BaseCell
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        TwoDCell();
        TwoDCell(int i, Vector2d corner, TwoDGrid* grid);
        virtual ~TwoDCell();

        bool isInsideCell(Vector2d point);

        Vector2d getCenter();
        Vector2d getCorner() { return _corner; }
        Vector2d getRandomPoint();
        Vector2d getCellSize();

        int getIndex() { return _index; }

        virtual void draw();

        bool operator==( TwoDCell otherCell) { return ((otherCell._index) == (this->_index)); }

    protected:
        int _index;
        Vector2d _corner;
        TwoDGrid* _grid;
    };

}

#endif // TWODCELL_HPP
