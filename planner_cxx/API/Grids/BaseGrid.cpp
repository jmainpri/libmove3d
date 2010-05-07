#include "BaseGrid.hpp"

using namespace API;

BaseGrid* API_activeGrid = NULL;

BaseGrid::BaseGrid()
{
}

BaseGrid::~BaseGrid()
{
    for(unsigned int i=0;i<_cells.size();i++)
    {
        delete _cells.at(i);
    }
}

/*!
 * \brief Get Cell
 *
 * \param index
 */
BaseCell* BaseGrid::getCell(int i)
{
    return _cells[i];
}

/*!
 * \brief Get Number Of Cells
 */
unsigned int BaseGrid::getNumberOfCells()
{
    return _cells.size();
}

/*!
 * \brief Virtual function for
 * creating an xml document
 */
bool BaseGrid::writeToXmlFile(std::string file)
{

}

/*!
 * \brief Virtual function for
 * reading from an xml document
 */
bool BaseGrid::loadFromXmlFile(std::string file)
{
	
}
