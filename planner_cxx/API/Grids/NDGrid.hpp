#ifndef NDGRID_HPP
#define NDGRID_HPP

#include "NDCell.hpp"
#include <vector>
#include "../../../other_libraries/Eigen/Core"
USING_PART_OF_NAMESPACE_EIGEN

/**
  * Base class for a Grid
  */
namespace API
{
    class NDGrid
    {
    public:
        NDGrid();
        virtual ~NDGrid();

        NDCell* getCell(int i);

        ~TwoDGrid();

        void createAllCells();

        Vector< getCellSize() { return _cellSize; }

        TwoDCell* getCell(const Vector2i& cell);
        TwoDCell* getCell(int x, int y );
        TwoDCell* getCell(Vector2d pos);
        TwoDCell* getCell(double* pos);

        bool isCellCoordInGrid(const Vector2i& coord);

        Vector2i getCellCoord(TwoDCell* ptrCell);
        int getNumberOfCells();
        TwoDCell* getNeighbour(const Vector2i& pos, int i);
        Vector2d getCoordinates(TwoDCell* cell);

        virtual void draw() =0;

    protected:
        std::vector<NDCell*> _cells;
    };
};

extern API::NDGrid* API_activeGrid;
#endif // NDGRID_HPP
