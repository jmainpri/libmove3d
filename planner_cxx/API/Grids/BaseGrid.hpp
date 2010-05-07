#ifndef BASEGRID_HPP
#define BASEGRID_HPP

#include "BaseCell.hpp"

#include <vector>

#include <Eigen/Core>

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
		unsigned int getNumberOfCells();
        virtual void draw() =0;
		virtual bool writeToXmlFile(std::string file);
		virtual bool loadFromXmlFile(std::string file);

    protected:
        std::vector<BaseCell*> _cells;
    };
}

extern API::BaseGrid* API_activeGrid;

#endif // BASEGRID_HPP
