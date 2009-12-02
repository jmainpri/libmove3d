#include "cell.h"
#include "grid.h"
#include <iostream>

using namespace std;

/*!
 * \brief Constructor of cell
 *
 * \param integer index
 */
Cell::Cell()
{
}

/*!
 * \brief Constructor of cell
 *
 * \param integer index
 */
Cell::Cell(int i, std::vector<double> corner, Grid* grid) :
        _index(i),
        _corner(corner),
        _grid(grid)
{
//    cout << " Cell " << i << ", Cornner = "<<  _corner.at(0) <<  _corner.at(1) <<  _corner.at(2) << ", Grid = " << _grid << endl;
}

/*!
 * \brief Constructor of cell
 *
 * \param integer index
 */
Cell::~Cell()
{

}

/*!
 * \brief Function is inside cell
 *
 * \param 3D point vector
 */
bool Cell::isInsideCell(std::vector<double> point)
{

}

/*!
 * \brief Function to get the center of the cell
 *
 * \param 3D point vector
 */
vector<double> Cell::getCenter()
{
//    cout << "getCenter()" << endl;

    vector<double> dimentions = _grid->getCellSize();

    for(int i=0;i< dimentions.size(); i++)
    {
        dimentions[i] = dimentions[i]/2 + _corner[i];
    }

    return dimentions;
}


