#include "BaseGrid.hpp"

using namespace API;

BaseGrid::BaseGrid()
{
}

BaseGrid::~BaseGrid()
{

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
