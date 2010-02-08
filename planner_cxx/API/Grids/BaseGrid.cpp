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
