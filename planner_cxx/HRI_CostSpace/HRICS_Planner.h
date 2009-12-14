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

#include "../planner.hpp"
#include "../API/planningAPI.hpp"
#include "HRICS_Grid.h"
#include "HRICS_Distance.h"
#include "HRICS_GridState.h"

class HRICS_Planner : public Planner {
	
public :
	
	HRICS_Planner();
	HRICS_Planner(Robot* rob, Graph* graph);

        ~HRICS_Planner();

        void initGrid();
        void deleteGrid();

        void initDistance();

        bool computeAStarIn3DGrid();

        void solveAStar(HriGridState* start,HriGridState* goal);

        void draw3dPath();
        double distanceToEntirePath();
        double distanceToCellPath();

        HriGrid* getGrid() { return _3DGrid; }
        HRICS_Distance* getDistance() { return _Distance; }

        Cell* getCellFromNode(Node* node);
        Node* nearestNeighbourInCell(Node* node, std::vector<Node*> neigbour);

        std::tr1::shared_ptr<Configuration> getConfigurationInNextCell(Node* node,bool foward);

private:
        std::vector<Robot*>     _Humans;
        HriGrid*                _3DGrid;
        HRICS_Distance*         _Distance;
        std::vector<Vector3d>   _3DPath;
        Graph*                  _Graph;
	
};


extern HRICS_Planner* HRICS_MOPL;
#define VIRTUAL_OBJECT 21 // Jido Horizontal 21

#endif
