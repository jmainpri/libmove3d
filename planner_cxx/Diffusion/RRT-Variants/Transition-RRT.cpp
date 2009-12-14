/*
 * RRT-Transition.cpp
 *
 *  Created on: Jul 27, 2009
 *      Author: jmainpri
 */

#include "Transition-RRT.hpp"
#include "../Expansion/TransitionExpansion.h"

#include "../../HRI_CostSpace/HRICS_Planner.h"

using namespace std;
using namespace tr1;

TransitionRRT::TransitionRRT(Robot* R, Graph* G) :
        RRT(R,G)
{

}

TransitionRRT::~TransitionRRT()
{

}

int TransitionRRT::init()
{
    int added = TreePlanner::init();

    _expan = new TransitionExpansion(this->getActivGraph());

    p3d_InitSpaceCostParam(this->getActivGraph()->getGraphStruct(),
                           this->getStart()->getNodeStruct(),
                           this->getGoal()->getNodeStruct());

    return added;
}

/**
 * costConnectNodeToComp
 * Try to connect a node to a given component
 * taking into account the fact that the space
 * is a cost space
 * @return: TRUE if the node and the componant have
 * been connected.
 */
bool TransitionRRT::connectNodeToCompco(Node* node, Node* compNode)
{
    int SavedIsMaxDis = FALSE;
    Node* node2(NULL);

    SavedIsMaxDis =  p3d_GetIsMaxDistNeighbor();
    p3d_SetIsMaxDistNeighbor(FALSE);

    if( ENV.getBool(Env::hriCsMoPlanner) )
    {
        vector<Node*> nodes = _Graph->getNodesInTheCompCo(compNode);
        //        cout << "nodes in compco =" << nodes.size() << endl;

        //        node->print();

        for(int i=0;i<nodes.size();i++)
        {
            if( *nodes[i] == *node )
            {
                cout << "TransitionRRT::Error" << endl;
            }

            //            nodes[i]->print();
        }
#ifdef HRI_COSTSPACE
        Node* neighbour = HRICS_MOPL->nearestNeighbourInCell(node,nodes);

        if( neighbour )
        {
            cout << "Neihbour in Cell" << endl;

            //            cout << "Cell(node) = " << HRICS_MOPL->getCellFromNode(node)->getIndex() << endl;
            //            cout << "Cell(neigh) = " << HRICS_MOPL->getCellFromNode(neighbour)->getIndex() << endl;

            LocalPath path(node->getConfiguration(),neighbour->getConfiguration());

            if(path.getValid())
            {
                return p3d_ConnectNodeToComp(
                        node->getGraph()->getGraphStruct(),
                        node->getNodeStruct(),
                        neighbour->getCompcoStruct());

//                return dynamic_cast<TreeExpansionMethod*>(_expan)->expandProcess(node,
//                                                                                 neighbour->getConfiguration(),
//
                                   neighbour,
//                                                                                 Env::nExtend);
            }
            else
            {
                cout << "Path not Valid" << endl;
            }
        }
#endif

        return false;
    }

    node2 = _Graph->nearestWeightNeighbour(compNode,
                                           node->getConfiguration(),
                                           false,
                                           p3d_GetDistConfigChoice());

    p3d_SetIsMaxDistNeighbor(SavedIsMaxDis);

    LocalPath path(node->getConfiguration(),node2->getConfiguration());

    if(!ENV.getBool(Env::CostBeforeColl))
    {
        if( path.getValid() )
        {
            if( path.length() <= _expan->step() )
            {
                int nbCreatedNodes=0;

                _expan->addNode(node,path,1.0,node2,nbCreatedNodes);
                cout << "Path Valid Connected" << endl;
                return true;
            }

            if( _expan->expandToGoal(
                    node,
                    node2->getConfiguration()))
            {
                int nbCreatedNodes=0;

                _expan->addNode(node,path,1.0,node2,nbCreatedNodes);
                cout << "attempting connect " << node->getConfiguration()->cost() << " to " << node2->getConfiguration()->cost() << endl;
                return true;
            }
        }
        return false;
    }
    else
    {
        if( path.length() <= _expan->step() )
        {
            int nbCreatedNodes=0;

            if( path.getValid() )
            {
                _expan->addNode(node,path,1.0,node2,nbCreatedNodes);
                cout << "Path Valid Connected" << endl;
                return true;
            }
            else
            {
                return false;
            }
        }
        if( _expan->expandToGoal(
                node,
                node2->getConfiguration()))
        {
            if( path.getValid() )
            {
                int nbCreatedNodes=0;
                _expan->addNode(node,path,1.0,node2,nbCreatedNodes);
                cout << "attempting connect " << node->getConfiguration()->cost() << " to " << node2->getConfiguration()->cost() << endl;
                return true;
            }
            else
            {
                return false;
            }
        }

    }
}
