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

        ThreeDCell* getCell(unsigned int x, unsigned int y, unsigned int z);
        ThreeDCell* getCell(Vector3i cell);
        ThreeDCell* getCell(Vector3d pos);
        ThreeDCell* getCell(double* pos);

        Vector3i getCellCoord(ThreeDCell* ptrCell);
        ThreeDCell* getNeighbour(const Vector3i& pos, unsigned int i);
        Vector3d getCoordinates(ThreeDCell* cell);

        virtual void draw();
		
		bool writeToXmlFile(std::string file);
		bool loadFromXmlFile(std::string file);

    protected:
        virtual ThreeDCell* createNewCell(unsigned int index, unsigned int x, unsigned int y, unsigned int z );
        Vector3d computeCellCorner(unsigned int x, unsigned int y, unsigned int z);

        Vector3d _originCorner;
        Vector3d _cellSize;

        unsigned int _nbCellsX;
        unsigned int _nbCellsY;
        unsigned int _nbCellsZ;
    };
}

#endif // GRID_HPP
