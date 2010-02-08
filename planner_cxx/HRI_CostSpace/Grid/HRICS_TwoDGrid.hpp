#ifndef HRICS_TWODGRID_HPP
#define HRICS_TWODGRID_HPP

#include "../../API/planningAPI.hpp"

namespace HRICS
{

    /**
      @ingroup HRICS
      @brief Plannar HRI Grid
      */
    class PlanGrid : public API::TwoDGrid
    {
    public:
        PlanGrid();
        PlanGrid(double pace, std::vector<double> envSize);

        void setRobot(Robot* R) { mRobot = R; }
        Robot* getRobot() { return mRobot; }


        API::TwoDCell* createNewCell(int index, int x, int y );

        void draw();

    private:
        Robot* mRobot;
    };

    /**
      @ingroup HRICS
      @brief Plannar HRI Cell
      */
    class PlanCell : public API::TwoDCell
    {

    public:
        PlanCell();
        PlanCell(int i, Vector2d corner, PlanGrid* grid);

        ~PlanCell() { }

        double getCost(); /* { std::cout << " Warning not implemented"  << std::endl; }*/

        void setBlankCost() { _CostIsComputed = false; }

//        Vector3i getCoord() { return _Coord; }

        bool getOpen() { return _Open; }
        void setOpen() { _Open = true; }

        bool getClosed() { return _Closed; }
        void setClosed() { _Closed = true; }

        void resetExplorationStatus() { _Open = false; _Closed = false; }

//        void draw();

    private:

//        Vector2i _Coord;
//
//        double* _v0; double* _v1; double* _v2; double* _v3;
//        double* _v4; double* _v5; double* _v6; double* _v7;

        bool _Open;
        bool _Closed;

        bool _CostIsComputed;
        double _Cost;

    };
}

#endif // HRICS_TWODGRID_HPP
