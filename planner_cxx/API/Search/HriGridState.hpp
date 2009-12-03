#ifndef HRIGRIDSTATE_HPP
#define HRIGRIDSTATE_HPP

#include "Astar/State.h"
#include "../3DGrid/Hri/HriGrid.hpp"
#include "../3DGrid/Hri/HriCell.h"

class HriGridState : public State
{
public:
    HriGridState() {}
    HriGridState(std::vector<int> cell, HriGrid* grid);
    HriGridState( HriCell* cell , HriGrid* grid);

    std::vector<State*> getSuccessors();

    bool isLeaf();		/* leaf control for an admissible heuristic function; the test of h==0*/
    bool equal(State* other);

    void setClosed(std::vector<State*>& closedStates,std::vector<State*>& openStates);
    bool isColsed(std::vector<State*>& closedStates);

    void setOpen(std::vector<State*>& openStates);
    bool isOpen(std::vector<State*>& openStates);

    void reset();

    void print();

    HriCell* getCell() { return _Cell; }

protected:
    double computeLength(State *parent);       /* g */
    double computeHeuristic(State *parent = NULL ,State* goal = NULL);    /* h */

private:
    HriGrid* _Grid;
    HriCell* _Cell;
};

#endif // HRIGRIDSTATE_HPP
