#ifndef GRID_HPP
#define GRID_HPP

#include <vector>

#include "cell.h"

/*!
 * \brief Base class for 3D grid based algorithms
 *
 * Deriving the Grid class and the Cell class permits to generates
 * easier grid algorithm. The function createNewCell is virtual just reimplement
 * this function in the new class as well as the constructors which allready
 * call the base one.
 */
class Grid
{

public:
    Grid();
    Grid( std::vector<int> size, std::vector<double> envSize );
    Grid( double pace, std::vector<double> envSize );

    ~Grid();

    void createAllCells();

    std::vector<double> getCellSize() { return _cellSize; }

    Cell* getCell(int i);
    Cell* getCell(int x, int y, int z);
    Cell* getCell(std::vector<int> cell);
    Cell* getCell(std::vector<double> pos);

    std::vector<int> getCellCoord(Cell* ptrCell);
    int getNumberOfCells();
    Cell* getNeighbour(const std::vector<int>& pos, int i);

protected:
    virtual Cell* createNewCell(int index, int x, int y, int z );
    std::vector<double> computeCellCorner(int x, int y, int z);

    std::vector<double> _originCorner;
    std::vector<double> _cellSize;

    int _nbCellsX;
    int _nbCellsY;
    int _nbCellsZ;

private:
    std::vector<Cell*> _cells;
};

#endif // GRID_HPP
