#ifndef HRICS_CSPACE_HPP
#define HRICS_CSPACE_HPP

#include "HRICS_Distance.h"
#include "../API/planningAPI.hpp"
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
    class CSpace
    {
    public:
        CSpace();
        CSpace(Robot* R, Robot* H);

        double getConfigCost();
        double getDistanceCost();
        double getVisibilityCost(Vector3d WSPoint);

        void computeVisibilityGrid();
        void computeDistanceGrid();

    private:
        void init();

         Distance* mDistance;
         Robot* mHuman;
         Robot* mRobot;

         Vector3d mVisibilityPoint;

         std::vector<double> mEnvSize;
    };
}

extern int VIRTUAL_OBJECT_DOF;
extern HRICS::CSpace* HRICS_CSpaceMPL;

#endif // HRICS_CSPACE_HPP
