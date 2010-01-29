#ifndef HRICELL_H
#define HRICELL_H

#include "../../API/planningAPI.hpp"
#include "HRICS_Grid.h"

/**
  @ingroup HRICS
  @brief Cell for the HRICS AStar
  */
namespace HRICS
{
    class Cell : public API::Cell
    {

    public:
        Cell();
        Cell(int i, Vector3i pos , Vector3d corner, HRICS::Grid* grid);

        ~Cell() { }

        double getCost();
        double getHRICostSpace();
        void setBlankCost() { _CostIsComputed = false; }

        Vector3i getCoord() { return _Coord; }

        bool getOpen() { return _Open; }
        void setOpen() { _Open = true; }

        bool getClosed() { return _Closed; }
        void setClosed() { _Closed = true; }

        void resetExplorationStatus();

        void draw();

    private:

        Vector3i _Coord;

        double* _v0; double* _v1; double* _v2; double* _v3;
        double* _v4; double* _v5; double* _v6; double* _v7;

        bool _Open;
        bool _Closed;

        bool _CostIsComputed;
        double _Cost;

    };
    //Contenu du namespace
}


#endif // HRICELL_H
