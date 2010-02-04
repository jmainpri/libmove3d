#ifndef TWODGRID_HPP
#define TWODGRID_HPP

#include <vector>

#include "../../../other_libraries/Eigen/Core"

USING_PART_OF_NAMESPACE_EIGEN

#include "BaseGrid.hpp"

#include "TwoDCell.hpp"


/*!
 @ingroup GRID

* \brief Base class for 2D grid based algorithms
*
* Deriving the Grid class and the Cell class permits to generates
* easier grid algorithm. The function createNewCell is virtual just reimplement
* this function in the new class as well as the constructors which allready
* call the base one.
*/
namespace API
{
    class TwoDGrid : public BaseGrid
   {

   public:
       TwoDGrid();
       TwoDGrid( Vector2i size, std::vector<double> envSize );
       TwoDGrid( double samplingRate, std::vector<double> envSize );

       ~TwoDGrid();

       void createAllCells();

       Vector2d getCellSize() { return _cellSize; }

       TwoDCell* getCell(const Vector2i& cell);
       TwoDCell* getCell(int x, int y );
       TwoDCell* getCell(Vector2d pos);
       TwoDCell* getCell(double* pos);

       bool isCellCoordInGrid(const Vector2i& coord);

       Vector2i getCellCoord(TwoDCell* ptrCell);
       int getNumberOfCells();
       TwoDCell* getNeighbour(const Vector2i& pos, int i);
       Vector2d getCoordinates(TwoDCell* cell);

       void draw();

   protected:
       virtual TwoDCell* createNewCell(int index, int x, int y );
       Vector2d computeCellCorner(int x, int y );

       Vector2d _originCorner;
       Vector2d _cellSize;

       int _nbCellsX;
       int _nbCellsY;
       int _nbCellsZ;

   };
}

#endif // TWODGRID_HPP
