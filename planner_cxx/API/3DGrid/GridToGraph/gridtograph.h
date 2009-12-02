#ifndef GRIDTOGRAPH_H
#define GRIDTOGRAPH_H

#include "../grid.h"
#include "../planner_cxx/API/planningAPI.hpp"

class GridToGraph : public Grid
{
public:
    GridToGraph();
    GridToGraph( std::vector<int> size );
    GridToGraph( double pace, std::vector<double> envSize );

    Cell* createNewCell(int index, int x, int y, int z );

    void putGridInGraph();

private:
    Robot* _Robot;
    Graph* _Graph;
};

#endif // GRIDTOGRAPH_H
