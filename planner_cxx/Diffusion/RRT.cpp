//
// C++ Implementation: rrt
//
// Description: 
//
//
// Author: Florian Pilardeau,B90,6349 <fpilarde@jolimont>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "RRT.hpp"
#include "../API/Grids/ThreeDPoints.h"

using namespace std;
using namespace tr1;

RRT::RRT(Robot* R, Graph* G) :
        TreePlanner(R,G)
{
    _nbConscutiveFailures = 0;
    cout << "RRT Constructor " << endl;
}

RRT::~RRT()
{
}

/**
 * Checks out the Stop condition
 */
bool RRT::checkStopConditions()
{
    if(TreePlanner::checkStopConditions())
    {
        return true;
    }

    return false;

}

/**
 * Checks out the preconditions
 */
bool RRT::preConditions()
{
    if(TreePlanner::preConditions())
    {
        if (ENV.getBool(Env::expandToGoal))
        {
            if(trajFound())
            {
                cout << "Start And Goal in same component" << endl;
                return true;
            }

            if(!ENV.getBool(Env::isCostSpace))
            {
                LocalPath direct(_Start->getConfiguration(), _Goal->getConfiguration());
                if (direct.isValid())
                {
                    connectNodeToCompco(_Start,_Goal);
                    cout << "Direct connection" << endl;
                    return true;
                }
            }
        }
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * Initializes an RRT Planner
 */
int  RRT::init()
{
    int added = TreePlanner::init();
    _expan = new RRTExpansion(_Graph);
    setInit(true);
    return added;
}

/**
 * Three phases One Step Expansion
 *  - Direction
 *  - Node
 *  - Process
 *
 *  @param fromComp the component which is expanded
 *  @param toComp the goal component
 */
int RRT::expandOneStep(Node* fromComp, Node* toComp)
{
    // ML-RRT expansion case
    /*	if (ENV.getBool(Env::isManhattan) && !(this->manhattanSamplePassive()))
        {
                return passiveExpandOneStep(fromComp, toComp);
        }
        // Standard expansion case
        else
        {*/
//	cout << "---------------------------------------------------" << endl;
//	cout <<"Robot Name = " << _Robot->getName() << endl;
//	cout << _Robot->getRobotStruct() << endl;
	
    Node* directionNode(NULL);
    Node* expansionNode(NULL);
    shared_ptr<Configuration> directionConfig;

    mNbExpansion++;

    // get direction
    directionConfig = _expan->getExpansionDirection(fromComp, toComp, false,
                                                    directionNode);
//    directionConfig->setConstraints();
//
    if(ENV.getBool(Env::drawPoints))
    {
        int VirtualObjDof = _Robot->getObjectDof();

        Vector3d randomPoint;
        randomPoint[0] = directionConfig->at(VirtualObjDof+0);
        randomPoint[1] = directionConfig->at(VirtualObjDof+1);
        randomPoint[2] = directionConfig->at(VirtualObjDof+2);

        PointsToDraw->push_back(randomPoint);
    }

//    cout << "***********************************************************"  << endl;
//    cout << "directionConfig->print()"  << endl;
//   directionConfig->print();

    // get node for expansion
    expansionNode = _expan->getExpansionNode(fromComp, directionConfig,
                                             ENV.getInt(Env::DistConfigChoice));

//    cout << "***********************************************************"  << endl;
//    cout << "expansionNode->print()"  << endl;
//    expansionNode->getConfiguration()->print();

    // expansion
    return _expan->expandProcess(expansionNode, directionConfig, directionNode,
                                 ENV.getExpansionMethod());
    //	}
}

