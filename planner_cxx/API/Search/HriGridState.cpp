#include "HriGridState.hpp"
#include "Hri_planner-pkg.h"
#include "../3DGrid/Hri/HriGrid.hpp"
#include <iostream>

using namespace std;

HriGridState::HriGridState( vector<int>& pos , HriGrid* grid) :
        _Grid(grid)
{
    _Cell = dynamic_cast<HriCell*>(grid->getCell(pos));
}

HriGridState::HriGridState( HriCell* cell , HriGrid* grid) :
        _Cell(cell),
        _Grid(grid)
{
}


vector<State*> HriGridState::getSuccessors()
{
    vector<State*> newStates;

    for(int i=0;i<26;i++)
    {
        HriCell* neigh = dynamic_cast<HriCell*>(_Grid->getNeighbour( _Cell->getCoord(), i));
        if(neigh!= NULL)
        {
            newStates.push_back( new HriGridState(neigh,_Grid));
        }
    }

    return newStates;
}

bool HriGridState::isLeaf()
{
    return false;
}

bool HriGridState::equal(State* other)
{
    bool equal(false);
    HriGridState* state = dynamic_cast<HriGridState*>(other);
    vector<int> pos = _Cell->getCoord();
    for(int i=0;i<3;i++)
    {
        if( pos.at(i) != state->_Cell->getCoord()[i])
        {
            //            cout << "HriGridState::equal false" << endl;
            return false;
        }
    }

    //    cout << "HriGridState::equal true" << endl;
    return true;
}

void HriGridState::setClosed(std::vector<State*>& closedStates,std::vector<State*>& openStates)
{
    //    cout << "HriGridState :: set Closed" <<endl;
    _Cell->setClosed();
}

bool HriGridState::isColsed(std::vector<State*>& closedStates)
{
    //    cout << "HriGridState :: get Closed" <<endl;
    return _Cell->getClosed();
}

void HriGridState::setOpen(std::vector<State*>& openStates)
{
    //     cout << "HriGridState :: set open" <<endl;
    _Cell->setOpen();
}


bool HriGridState::isOpen(std::vector<State*>& openStates)
{
    //    cout << "HriGridState :: get open" <<endl;
    return _Cell->getOpen();
}

void HriGridState::reset()
{
    _Cell->resetExplorationStatus();
}

void HriGridState::print()
{

}

double HriGridState::computeLength(State *parent)
{
    HriGridState* preced = dynamic_cast<HriGridState*>(parent);

    vector<double> pos1 = _Cell->getCenter();
    vector<double> pos2 = preced->_Cell->getCenter();

    double dist=0;

    for(int i=0;i<3;i++)
    {
        dist += pow((pos1[i]-pos2[i]),2);
    }

    dist = sqrt(dist);

    double cost1 = preced->_Cell->getCost();
    double cost2 = _Cell->getCost();

    double g = preced->g() +     ((cost1 + cost2) * dist / 2);
    return g;
}

double HriGridState::computeHeuristic(State *parent)
{
    HriGridState* preced = dynamic_cast<HriGridState*>(parent);

    vector<int> pos1 = _Cell->getCoord();
    vector<int> pos2 = preced->_Cell->getCoord();

    double dist=0;

    for(int i=0;i<3;i++)
    {
        dist += pow((double)(pos1[i]-pos2[i]),2);
    }
    return 0.001*dist;
}
