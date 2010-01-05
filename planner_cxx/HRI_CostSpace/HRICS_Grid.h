#ifndef HRIGRID_HPP
#define HRIGRID_HPP

#include "../API/planningAPI.hpp"

namespace HRICS
{

    class Cell;

    class Grid : public API::Grid
    {
    public:
        Grid();
        Grid( std::vector<int> size );
        Grid(double pace, std::vector<double> envSize);

        API::Cell* createNewCell(int index, int x, int y, int z );
        void computeAllCellCost();

        void drawGrid();
        void resetCellCost();

        void setRobot(Robot* rob) { _Robot = rob; }
        Robot* getRobot() { return _Robot; }

        bool isVirtualObjectPathValid(Cell* fromCell,Cell* toCell);

    private:

        Robot* _Robot;
    };
}

#endif // HRIGRID_HPP
