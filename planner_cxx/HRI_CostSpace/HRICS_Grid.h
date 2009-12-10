#ifndef HRIGRID_HPP
#define HRIGRID_HPP

#include "../API/planningAPI.hpp"

class HriCell;

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

    void setRobot(Robot* rob) { _Robot = rob; }
    Robot* getRobot() { return _Robot; }

    bool isVirtualObjectPathValid(HriCell* fromCell,HriCell* toCell);

private:

    Robot* _Robot;
};

#endif // HRIGRID_HPP
