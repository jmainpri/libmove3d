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
#include "HRICS_Visibility.h"
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
    class Workspace : public Planner
    {

    public :

        /**
          * Constructors & Destructors
          */
        Workspace();
        Workspace(Robot* rob, Graph* graph);
        ~Workspace();

        /**
          * Init Associated Objects
          */
        void initGrid();
        void deleteGrid();
        void initDistance();
		void initVisibility();

        /**
          * Computes A* in Grid
          */
        bool computeAStarIn3DGrid();
        double pathCost();
        void draw3dPath();

        /**
          *
          */
        double distanceToEntirePath();
        double distanceToCellPath();

        /**
          * Getters
          */
        Grid*			getGrid() { return m3DGrid; }
		
        Distance*	getDistance() { return mDistance; }
		Visibility* getVisibility() { return m_Visibility; }
		
        std::vector<Vector3d> get3DPath() { return m3DPath; }
        std::vector<API::ThreeDCell*> getCellPath() { return m3DCellPath; }
		
        int getIndexObjectDof() { return mIndexObjectDof; }

        /**
          * Run RRT
          */
        bool initHriRRT();

    private:

        void solveAStar(State* start,State* goal);

        /** 
		 * Members
		*/
		
		/**
		 * Humans in the scene
		 */
        std::vector<Robot*>     mHumans;
		
		int mIndexObjectDof;
        
		/**
		 * 3d grid to compute a Workspace path
		 */
		Grid*                   m3DGrid;
		
		/**
		 * Distance and Visibility 
		 * cost spaces
		 */
        Distance*               mDistance;
		Visibility*             m_Visibility;
        
		
		/**
		 * 3d path internals
		 */
		bool mPathExist;
		std::vector<Vector3d>   m3DPath;
        std::vector<API::ThreeDCell*> m3DCellPath;
    };
}

#endif
