#ifndef HRIGRID_HPP
#define HRIGRID_HPP

#include "../../API/planningAPI.hpp"

/**
  @ingroup HRICS
  @brief Cell for the HRICS AStar
  */
namespace HRICS
{

    class Cell;

    class Grid : public API::ThreeDGrid
    {
    public:
        Grid();
        Grid( std::vector<int> size );
        Grid(double pace, std::vector<double> envSize);

        API::ThreeDCell* createNewCell(unsigned int index, unsigned int x, unsigned int y, unsigned int z );
        void computeAllCellCost();

		void drawSpheres();
        void draw();
        void resetCellCost();

        void setRobot(Robot* rob) { _Robot = rob; }
        Robot* getRobot() { return _Robot; }

        bool isVirtualObjectPathValid(Cell* fromCell,Cell* toCell);

    private:

        Robot* _Robot;
    };
}

#endif // HRIGRID_HPP
