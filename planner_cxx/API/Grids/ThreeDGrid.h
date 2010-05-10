#ifndef GRID_HPP
#define GRID_HPP

#include <vector>

#include <Eigen/Core>

USING_PART_OF_NAMESPACE_EIGEN

#include "ThreeDCell.h"
#include "BaseGrid.hpp"

 /*!
  @ingroup GRID

 * \brief Base class for 3D grid based algorithms
 *
 * Deriving the Grid class and the Cell class permits to generates
 * easier grid algorithm. The function createNewCell is virtual just reimplement
 * this function in the new class as well as the constructors which allready
 * call the base one.
 */
namespace API
{
    class ThreeDGrid : public BaseGrid
    {

    public:
        ThreeDGrid();
        ThreeDGrid( Vector3i size,    std::vector<double> envSize );
        ThreeDGrid( double samplingRate,      std::vector<double> envSize );

        virtual ~ThreeDGrid();

        void createAllCells();

        Vector3d getCellSize() { return _cellSize; }

        ThreeDCell* getCell(int x, int y, int z);
        ThreeDCell* getCell(Vector3i cell);
        ThreeDCell* getCell(Vector3d pos);
        ThreeDCell* getCell(double* pos);

        Vector3i getCellCoord(ThreeDCell* ptrCell);
        int getNumberOfCells();
        ThreeDCell* getNeighbour(const Vector3i& pos, int i);
        Vector3d getCoordinates(ThreeDCell* cell);

        virtual void draw();

    protected:
        virtual ThreeDCell* createNewCell(int index, int x, int y, int z );
        Vector3d computeCellCorner(int x, int y, int z);

        Vector3d _originCorner;
        Vector3d _cellSize;

        int _nbCellsX;
        int _nbCellsY;
        int _nbCellsZ;
    };
}

#endif // GRID_HPP
