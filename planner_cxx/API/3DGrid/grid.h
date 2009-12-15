#ifndef GRID_HPP
#define GRID_HPP

#include <vector>

#include "../../../other_libraries/Eigen/Core"

USING_PART_OF_NAMESPACE_EIGEN

#include "cell.h"

 /*!
 * \brief Base class for 3D grid based algorithms
 *
 * Deriving the Grid class and the Cell class permits to generates
 * easier grid algorithm. The function createNewCell is virtual just reimplement
 * this function in the new class as well as the constructors which allready
 * call the base one.
 */
namespace API
{
    class Grid
    {

    public:
        Grid();
        Grid( Vector3i size,    std::vector<double> envSize );
        Grid( double samplingRate,      std::vector<double> envSize );

        ~Grid();

        void createAllCells();

        Vector3d getCellSize() { return _cellSize; }

        Cell* getCell(int i);
        Cell* getCell(int x, int y, int z);
        Cell* getCell(Vector3i cell);
        Cell* getCell(Vector3d pos);

        Vector3i getCellCoord(Cell* ptrCell);
        int getNumberOfCells();
        Cell* getNeighbour(const Vector3i& pos, int i);

    protected:
        virtual Cell* createNewCell(int index, int x, int y, int z );
        Vector3d computeCellCorner(int x, int y, int z);

        Vector3d _originCorner;
        Vector3d _cellSize;

        int _nbCellsX;
        int _nbCellsY;
        int _nbCellsZ;

    private:
        std::vector<Cell*> _cells;
    };
}

#endif // GRID_HPP
