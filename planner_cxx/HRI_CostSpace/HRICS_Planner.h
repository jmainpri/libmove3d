#ifndef HRICS_PLANNER_H
#define HRICS_PLANNER_H

/*
 *  HRICS_CostSpace.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 04/12/09.
 *  Copyright 2009 mainprice@laas.fr All rights reserved.
 *
 */

#include "../API/planningAPI.hpp"
#include "../planner.hpp"
#include "../Diffusion/RRT.hpp"
#include "HRICS_Distance.h"
#include "Grid/HRICS_Grid.h"
#include "Grid/HRICS_GridState.h"

/**
    @defgroup HRICS Hri Cost space
 */

/**
  @ingroup HRICS
  */
namespace HRICS
{
    class MainPlanner : public Planner
    {

    public :

        /**
          * Constructors & Destructors
          */
        MainPlanner();
        MainPlanner(Robot* rob, Graph* graph);
        ~MainPlanner();

        /**
          * Init Associated Objects
          */
        void initGrid();
        void deleteGrid();
        void initDistance();

        /**
          * Computes A* in Grid
          */
        bool computeAStarIn3DGrid();
        void draw3dPath();

        /**
          *
          */
        double distanceToEntirePath();
        double distanceToCellPath();

        /**
          * Getters
          */
        Grid* getGrid() { return m3DGrid; }
        Distance* getDistance() { return mDistance; }
        std::vector<Vector3d> get3DPath() { return m3DPath; }
        int getIndexObjectDof() { return mIndexObjectDof; }

        /**
          * Run RRT
          */
        bool runHriRRT();

    private:

        void solveAStar(State* start,State* goal);

        /** Members
          */
        std::vector<Robot*>     mHumans;
        Grid*                   m3DGrid;
        Distance*               mDistance;
        bool mPathExist;
        std::vector<Vector3d>   m3DPath;
        std::vector<API::ThreeDCell*> m3DCellPath;
        int mIndexObjectDof;

    };
}

extern HRICS::MainPlanner* HRICS_MOPL;
//extern int VIRTUAL_OBJECT_DOF; // dof

#endif
