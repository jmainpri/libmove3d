#ifndef BASEGRID_HPP
#define BASEGRID_HPP

#include "BaseCell.hpp"

#include <vector>

#include "../../../other_libraries/Eigen/Core"

USING_PART_OF_NAMESPACE_EIGEN

/**
  * Base class for a Grid
  */
namespace API
{
    class BaseGrid
    {
    public:
        BaseGrid();
        virtual ~BaseGrid();

        BaseCell* getCell(int i);

        virtual void draw() =0;

    protected:
        std::vector<BaseCell*> _cells;
    };
}

extern API::BaseGrid* API_activeGrid;

#endif // BASEGRID_HPP
