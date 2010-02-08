#ifndef HRICS_CSPACE_HPP
#define HRICS_CSPACE_HPP

#include "../API/planningAPI.hpp"
#include "../planner.hpp"

#include "HRICS_Grid.h"
#include "HRICS_TwoDGrid.hpp"
#include "HRICS_Distance.h"

/**
    @defgroup HRICS Hri Cost space
 */

/**
  @ingroup HRICS
  */
namespace HRICS
{
    /**
      * Configuration space
      */
    class CSpace : public Planner
    {
    public:
        CSpace();
        CSpace(Robot* R, Robot* H);

        ~CSpace();

        /**
          *
          */
        double getConfigCost();
        double getDistanceCost();
        double getVisibilityCost(Vector3d WSPoint);

        void computeVisibilityGrid();
        void computeDistanceGrid();

        Distance* getDistance() { return mDistance; }
        Grid* getGrid() { return m3DGrid; }
        PlanGrid* getPlanGrid() { return m2DGrid; }

        double getLastDistanceCost() {return mDistCost; }
        double getLastVisibiliCost() {return mVisiCost; }

    private:
        void initCostSpace();

        //        Robot* mRobot;
        Robot* mHuman;

        Grid* m3DGrid;
        Distance* mDistance;

        PlanGrid* m2DGrid;

        int mIndexObjectDof;

        Vector3d mVisibilityPoint;

        double mDistCost;
        double mVisiCost;

        std::vector<double> mEnvSize;
    };
}

//extern int VIRTUAL_OBJECT_DOF;
extern HRICS::CSpace* HRICS_CSpaceMPL;

#endif // HRICS_CSPACE_HPP
