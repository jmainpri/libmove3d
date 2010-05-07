#ifndef NDGRID_HPP
#define NDGRID_HPP

#include "NDCell.hpp"
#include <vector>
#include <Eigen/Core>
USING_PART_OF_NAMESPACE_EIGEN

/**
  * Base class for a Grid
  */
namespace API
{
    template <int _nDimGrid_Dimension_ > 
	class nDimGrid
    {
		
    public:
		/**
		 * Constructors
		 */
        nDimGrid();
        nDimGrid( Matrix< int, _nDimGrid_Dimension_ , 1 > size, std::vector<double> envSize );
        nDimGrid( double samplingRate, std::vector<double> envSize );
		
		/**
		 * Destructor
		 */
        virtual ~nDimGrid();
		
		/**
		 * Creates all cells in
		 * the grid calling the function create new cell
		 */
        void createAllCells();
		
		/**
		 * Returns the dimension of on cell
		 */
        Matrix< double, _nDimGrid_Dimension_ , 1 > getCellSize() { return m_cellSize; }
		
		/**
		 * Accessors to cells
		 */
		nDimCell* getCell(unsigned int i);
        nDimCell* getCell(const Matrix<    int, _nDimGrid_Dimension_ , 1 > & coordinate);
        nDimCell* getCell(const Matrix< double, _nDimGrid_Dimension_ , 1 > & pos);
        nDimCell* getCell(double* pos);

		
		/**
		 * Returns the coordinate of a cell in the grid
		 */
		Matrix< int, _nDimGrid_Dimension_ , 1 > getCellCoord(unsigned int index);
        Matrix< int, _nDimGrid_Dimension_ , 1 > getCellCoord(nDimCell* ptrCell);
		
		/**
		 * Returns the number of cell in the grid 
		 */
        unsigned int getNumberOfCells();
		
		/**
		 * Returns the neighbor cells
		 * of the cell at coordinate 'pos'
		 */
        nDimCell* getNeighbour( const Matrix< int, _nDimGrid_Dimension_ , 1 > & coordinate, int i);
				
		/**
		 * Function to display
		 * the grid
		 */
        virtual void draw();
		
    protected:
		
		/**
		 * Allocates one cell
		 */
        virtual nDimCell* createNewCell(int index, const Matrix< int, _nDimGrid_Dimension_ , 1 > & coordinate );
		
		/**
		 * Compute the cell's corner as the 
		 * center is stored inside the cell and the size 
		 * is stored in the Grid
		 */
        Matrix< double, _nDimGrid_Dimension_ , 1 > computeCellCorner( const Matrix< int, _nDimGrid_Dimension_ , 1 > & coordinate );
		
		
	private:
		/**
		 * All cells are stored
		 * in this array
		 */
        std::vector<nDimCell*> m_cells;
		
		/**
		 * Origin
		 */
        Matrix< double, _nDimGrid_Dimension_ , 1 > m_originCorner;
		
		/**
		 * One cell dimension (Size)
		 */
        Matrix< double, _nDimGrid_Dimension_ , 1 > m_cellSize;
		
		/**
		 * Number of Cell per dimension
		 */
		Matrix<    int, _nDimGrid_Dimension_ , 1 > m_nbOfCell;
    };

};

#endif // NDGRID_HPP
