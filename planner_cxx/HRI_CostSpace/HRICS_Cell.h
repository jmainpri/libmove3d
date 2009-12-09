#ifndef HRICELL_H
#define HRICELL_H

#include "../API/planningAPI.hpp"
#include "HRICS_Grid.h"

class HriCell : public Cell
{

public:
    HriCell();
    HriCell(int i, std::vector<int> pos , Vector3d corner, HriGrid* grid);

    ~HriCell() { }

    double getCost();
    double getHRICostSpace();
    void setBlankCost() { _CostIsComputed = false; }

    std::vector<int> getCoord() { return _Coord; }

    bool getOpen() { return _Open; }
    void setOpen() { _Open = true; }

    bool getClosed() { return _Closed; }
    void setClosed() { _Closed = true; }

    void resetExplorationStatus();

    void drawCell();

private:

    std::vector<int> _Coord;

    double* _v0; double* _v1; double* _v2; double* _v3;
    double* _v4; double* _v5; double* _v6; double* _v7;

    bool _Open;
    bool _Closed;

    bool _CostIsComputed;
    double _Cost;

};

#endif // HRICELL_H
