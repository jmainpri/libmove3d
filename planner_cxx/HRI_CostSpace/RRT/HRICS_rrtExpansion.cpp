#include "HRICS_rrtExpansion.h"
#include "../HRICS_Planner.h"

using namespace std;
using namespace tr1;
using namespace HRICS;

HRICS_rrtExpansion::HRICS_rrtExpansion() :
        TransitionExpansion()
{
}

HRICS_rrtExpansion::HRICS_rrtExpansion(Graph* ptrGraph) :
        TransitionExpansion(ptrGraph)
{
}


shared_ptr<Configuration> HRICS_rrtExpansion::getExpansionDirection(
        Node* expandComp, Node* goalComp, bool samplePassive, Node*& directionNode)
{
    bool foward = true;

    if(goalComp->getCompcoStruct()->num == mGraph->getStart()->getCompcoStruct()->num )
    {
        foward = false;
    }

    shared_ptr<Configuration> q = getConfigurationInNextCell(expandComp,foward);

    return q;

}

API::Cell* HRICS_rrtExpansion::getLastCellOnPath(vector<Vector3d> nodes)
{
    set<API::Cell*> Cells;
    set<API::Cell*>::iterator it;

    for(int i=0;i<nodes.size();i++)
    {
        Cells.insert( _3DGrid->getCell(nodes[i]) );
    }

//    double minDist = numeric_limits<double>::max();
//    Vector3d point;
//
//    for(it=Cells.begin(); it!=Cells.end(); it++)
//    {
//        double dist = ( point - _3DPath[i] ).norm();
//        if(minDist > dist )
//        {
//            minDist = dist;
//            pathCellID = i;
//        }
//    }

}


shared_ptr<Configuration> HRICS_rrtExpansion::getConfigurationInNextCell(Node* CompcoNode,bool foward)
{
    vector<Vector3d> nodes;

    p3d_list_node* ListNode = CompcoNode->getCompcoStruct()->dist_nodes;

    while (ListNode!=NULL)
    {
        Vector3d pos;

        pos[0] = ListNode->N->q[VIRTUAL_OBJECT+0];
        pos[1] = ListNode->N->q[VIRTUAL_OBJECT+1];
        pos[2] = ListNode->N->q[VIRTUAL_OBJECT+2];

        nodes.push_back( pos );

        ListNode = ListNode->next;
    }



    unsigned int pathCellID;

    //    cout << "Path Size = " << _3DPath.size() << endl;
    //    cout << "Foward = " << foward << endl;
    //    cout << "pathCellID = " << pathCellID << endl;

    // Next Cell after the one beeing the closest
    API::Cell* NextCell;

    if( foward )
    {
        if( pathCellID < _3DPath.size()-1)
        {
            NextCell = _3DGrid->getCell(_3DPath[pathCellID+1]);
        }
        else
        {
            NextCell = _3DGrid->getCell(_3DPath[pathCellID]);
        }
    }
    else
    {
        if( pathCellID > 0 )
        {
            NextCell = _3DGrid->getCell(_3DPath[pathCellID-1]);
        }
        else
        {
            NextCell = _3DGrid->getCell(_3DPath[pathCellID]);
        }
    }

    Vector3d randomPoint = NextCell->getRandomPoint();
    cout << "Random Point = " << randomPoint << endl;

    shared_ptr<Configuration> q = mGraph->getRobot()->shoot(false);

}

bool HRICS_rrtExpansion::afterAndOnPath(API::Cell* cell)
{
    if(_foward)
    {
        for(int i=0;i<_3DPath.size();i++)
        {
//            afterAndOnPath
        }
    }
}

Node* HRICS_rrtExpansion::addNode(Node* currentNode, LocalPath& path, double pathDelta,
                Node* directionNode, int& nbCreatedNodes)
{
    Node* newNode = BaseExpansion::addNode(
            currentNode,path,pathDelta,directionNode,nbCreatedNodes);

    Vector3d pos;

    pos[0] = currentNode->getNodeStruct()->q[VIRTUAL_OBJECT+0];
    pos[1] = currentNode->getNodeStruct()->q[VIRTUAL_OBJECT+1];
    pos[2] = currentNode->getNodeStruct()->q[VIRTUAL_OBJECT+2];

    if( _foward )
    {
        API::Cell* cell = _3DGrid->getCell(pos);
        if(_LastForward != cell)
        {
            if( afterAndOnPath( cell ) )
            {

            }
        }

    }
}
