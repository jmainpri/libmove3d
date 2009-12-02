#ifndef HRIGRID_HPP
#define HRIGRID_HPP

#include "../grid.h"

class HriGrid : public Grid
{
public:
    HriGrid();
    HriGrid( std::vector<int> size );
    HriGrid(double pace, std::vector<double> envSize);

    Cell* createNewCell(int index, int x, int y, int z );
    void computeAllCellCost();

    void drawGrid();
    void resetCellCost();


};

#endif // HRIGRID_HPP
