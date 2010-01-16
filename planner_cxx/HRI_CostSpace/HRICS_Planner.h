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
        void solveAStar(State* start,State* goal);
        void draw3dPath();

        /**
          *
          */
        double distanceToEntirePath();
        double distanceToCellPath();

        /**
          * Getters
          */
        Grid* getGrid() { return _3DGrid; }
        Distance* getDistance() { return _Distance; }
        std::vector<Vector3d> get3DPath() { return _3DPath; }

        /**
          * Run RRT
          */
        bool runHriRRT();

    private:
        std::vector<Robot*>     _Humans;
        Grid*                   _3DGrid;
        Distance*               _Distance;
        std::vector<Vector3d>   _3DPath;
        std::vector<API::Cell*> _3DCellPath;

    };
}

extern HRICS::MainPlanner* HRICS_MOPL;
extern int VIRTUAL_OBJECT_DOF; // dof

#endif
