#include "HRICS_rrt.h"
#include "HRICS_rrtExpansion.h"
#include "../HRICS_Planner.h"

using namespace std;
using namespace tr1;
using namespace HRICS;

/**
  * Basic constructor
  */
HRICS_RRT::HRICS_RRT(Robot* R, Graph* G) : RRT(R,G)
{

}

/**
 * Initializes an RRT Planner
 */
int  HRICS_RRT::init()
{

    int added = TreePlanner::init();

    _expan = new HRICS_rrtExpansion(_Graph);

    setInit(true);

    return added;
}

/**
 * Tries to connect a node to a given component
 * taking into account the fact that the space
 * is a cost space
 * @return: TRUE if the node and the componant have
 * been connected.
 */
bool HRICS_RRT::connectNodeToCompco(Node* node, Node* compNode)
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

    Node* neighbour = nearestNeighbourInCell(node,nodes);

    if( neighbour )
    {
        cout << "Neihbour in Cell" << endl;

        //            cout << "Cell(node) = " << HRICS_MOPL->getCellFromNode(node)->getIndex() << endl;
        //            cout << "Cell(neigh) = " << HRICS_MOPL->getCellFromNode(neighbour)->getIndex() << endl;

//        LocalPath path(node->getConfiguration(),neighbour->getConfiguration());

//        if(path.getValid())
//        {
            return p3d_ConnectNodeToComp(
                    node->getGraph()->getGraphStruct(),
                    node->getNodeStruct(),
                    neighbour->getCompcoStruct());

            //                return dynamic_cast<TreeExpansionMethod*>(_expan)->expandProcess(node,
            //                                                                                 neighbour->getConfiguration(),
            //                                                                                   neighbour,
            //                                                                                 Env::nExtend);
//        }
//        else
//        {
//            cout << "Path not Valid" << endl;
//        }
    }

    return false;
}


Node* HRICS_RRT::nearestNeighbourInCell(Node* node, std::vector<Node*> neigbour)
{

    API::Cell* cell = getCellFromNode(node);
    vector<Node*> nodesInCell;

    for(int i=0;i<neigbour.size();i++)
    {
        if((*cell) == (*getCellFromNode(neigbour[i])))
        {
            nodesInCell.push_back(neigbour[i]);
        }
    }

    double minDist = numeric_limits<double>::max();
    Node* nearest = 0x00;

    for(int i=0;i<nodesInCell.size();i++)
    {
        shared_ptr<Configuration> config = nodesInCell[i]->getConfiguration();
        double dist = node->getConfiguration()->dist(*config);
        if(minDist>dist)
        {
            minDist = dist;
            nearest = nodesInCell[i];
        }
    }

    return nearest;
}

API::Cell* HRICS_RRT::getCellFromNode(Node* node)
{
    shared_ptr<Configuration> config = node->getConfiguration();

    Vector3d pos;

    pos[0] = config->getConfigStruct()[VIRTUAL_OBJECT+0];
    pos[1] = config->getConfigStruct()[VIRTUAL_OBJECT+1];
    pos[2] = config->getConfigStruct()[VIRTUAL_OBJECT+2];
    //        cout << "pos = " << endl << pos << endl;

    return _Grid->getCell(pos);
}
