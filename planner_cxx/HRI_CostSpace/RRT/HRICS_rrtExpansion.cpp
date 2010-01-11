#include "HRICS_rrtExpansion.h"
#include "../HRICS_Planner.h"
#include "../../API/3DGrid/points.h"

using namespace std;
using namespace tr1;
using namespace HRICS;

HRICS_rrtExpansion::HRICS_rrtExpansion() :
        TransitionExpansion(),
        _biasing(false)
{
    this->init();
}

HRICS_rrtExpansion::HRICS_rrtExpansion(Graph* ptrGraph) :
        TransitionExpansion(ptrGraph),
        _biasing(false)
{
    this->init();
}

void HRICS_rrtExpansion::init()
{
    cout << "Init Box Jido" << endl;
    double box[] = {-1.3,1.3,-1.3,1.3,0,1.5};
    shared_ptr<Configuration> qInit = mGraph->getRobot()->getInitialPosition();

    _Box = new double[6];

    _Box[0] = box[0] + qInit->getConfigStruct()[6];
    _Box[1] = box[1] + qInit->getConfigStruct()[6];
    _Box[2] = box[2] + qInit->getConfigStruct()[7];
    _Box[3] = box[3] + qInit->getConfigStruct()[7];
    _Box[4] = box[4];
    _Box[5] = box[5];
}

void HRICS_rrtExpansion::setCellPath(vector<API::Cell*> cellPath)
{
    _3DCellPath = cellPath;
    _LastForward = cellPath.at(0);
    _LastBackward = cellPath.back();
}

int Direction=0;
shared_ptr<Configuration> HRICS_rrtExpansion::getExpansionDirection(
        Node* expandComp, Node* goalComp, bool samplePassive, Node*& directionNode)
{
    shared_ptr<Configuration> q;

    _biasing = ENV.getBool(Env::isGoalBiased) && p3d_random(0., 1.) <= ENV.getDouble(Env::Bias);

    if( _biasing )
    {
        q = getConfigurationInNextCell(expandComp);
    }
    else
    {
        if(ENV.getBool(Env::isInverseKinematics))
        {
            q = mGraph->getRobot()->shootFreeFlyer(_Box);

            if(ENV.getBool(Env::drawPoints))
            {
                if(PointsToDraw==NULL)
                {
                    PointsToDraw = new Points();
                }
                Vector3d randomPoint;
                randomPoint[0] = q->getConfigStruct()[VIRTUAL_OBJECT+0];
                randomPoint[1] = q->getConfigStruct()[VIRTUAL_OBJECT+1];
                randomPoint[2] = q->getConfigStruct()[VIRTUAL_OBJECT+2];
                PointsToDraw->push_back(randomPoint);
            }
        }
        else
        {
            q = mGraph->getRobot()->shoot(samplePassive);
        }
    }


    //    cout << ++Direction << " New Direction (Biased = " << biasing << ") " << endl;
    return q;
}

API::Cell* BiasedCell=NULL;
Points* PointsToDraw=NULL;

shared_ptr<Configuration> HRICS_rrtExpansion::getConfigurationInNextCell(Node* CompcoNode)
{
    API::Cell* farthestCell;

    // Get the farthest cell explored depending on
    // the way the tree explores
    if( CompcoNode->getCompcoStruct()->num == mGraph->getStart()->getCompcoStruct()->num )
    {
        _forward = true;
        farthestCell = _LastForward;
    }
    else
    {
        _forward = false;
        farthestCell = _LastBackward;
    }

    Vector3d randomPoint;
    int cellId;

    // Get Id of Next cell on the 3D Path
    for(int i=0; i<_3DCellPath.size(); i++)
    {
        if(_3DCellPath[i] == farthestCell )
        {
            if( _forward  )
            {
                i++;
                if( i == _3DCellPath.size() )
                {
                    i = _3DCellPath.size()-1;
                }
                cellId = i;
                break;
            }
            else
            {
                i--;
                if( i == -1 )
                {
                    i = 0;
                }
                cellId = i;
                break;
            }
        }
    }

//    shared_ptr<Configuration> q = mGraph->getRobot()->shoot(false);

    // Get a random config in the cell
//    randomPoint = _3DCellPath[cellId]->getRandomPoint();
    BiasedCell = _3DCellPath[cellId];

    //     if(PointsToDraw==NULL)
    //     {
    //         PointsToDraw = new Points();
    //     }
    //
    //    PointsToDraw->push_back(randomPoint);
    //    g3d_draw_allwin_active();

//    Matrix3d mat = Matrix3d::Identity();
//    randomPoint =2*mat*randomPoint;

    shared_ptr<Configuration> q(new Configuration(mGraph->getRobot()));

    Vector3d corner = BiasedCell->getCorner();
    Vector3d cellSize = BiasedCell->getCellSize();

    double biasedBox[6];

    biasedBox[0] = corner[0];
    biasedBox[1] = corner[0] + cellSize[0];
    biasedBox[2] = corner[1];
    biasedBox[3] = corner[1] + cellSize[1];
    biasedBox[4] = corner[2];
    biasedBox[5] = corner[2] + cellSize[2];

    p3d_FreeFlyerShoot( mGraph->getRobot()->getRobotStruct() , q->getConfigStruct() , biasedBox );

//    q->getConfigStruct()[VIRTUAL_OBJECT+0] = randomPoint[0];
//    q->getConfigStruct()[VIRTUAL_OBJECT+1] = randomPoint[1];
//    q->getConfigStruct()[VIRTUAL_OBJECT+2] = randomPoint[2];

    return q;
}

/**
  * Return true if the cell is on the path
  * and after the first cell depending on the order (forward or backwards)
  */
bool HRICS_rrtExpansion::on3DPathAndAfter(API::Cell* cell)
{
    // Is cell on path
    bool cellOnPath;

    for(unsigned int i=0;i<_3DCellPath.size();i++)
    {
        if(cell == _3DCellPath[i])
        {
            cellOnPath = true;
            break;
        }
    }

    if( cellOnPath == false )
    {
        cout << "Not on path" << endl;
        return false;
    }

    //    cout << "Cell on Path" << endl;
    //    for(unsigned int i=0;i<_3DCellPath.size();i++)
    //    {
    //        cout << i << " => " << _3DCellPath[i] << endl;
    //    }

    //    cout << "Cell = " << cell << endl;
    //    cout << "Last = " << _LastForward << endl;

    // Looks if it is the one further away
    // forward and backwards
    if(_forward)
    {
        for(unsigned int i=0;i<_3DCellPath.size();i++)
        {
            if( _3DCellPath[i] == _LastForward )
            {
                //                cout << "After" << endl;
                return true;
            }
            if( _3DCellPath[i] == cell )
            {
                //                cout << "Before" << endl;
                return false;
            }
        }

    }
    else
    {
        for(unsigned int i=_3DCellPath.size()-1;i>=0;i--)
        {
            if( _3DCellPath[i] == _LastBackward )
            {
                return true;
            }
            if( _3DCellPath[i] == cell )
            {
                return false;
            }
        }
    }
}

/**
  * Adds a node and checks
  * if it explores the path
  */
Node* HRICS_rrtExpansion::addNode(Node* currentNode, LocalPath& path, double pathDelta,
                                  Node* directionNode, int& nbCreatedNodes)
{
    Node* newNode = BaseExpansion::addNode(
            currentNode,path,pathDelta,directionNode,nbCreatedNodes);

    //    cout << "New Node " << endl;

    Vector3d pos;

    pos[0] = currentNode->getNodeStruct()->q[VIRTUAL_OBJECT+0];
    pos[1] = currentNode->getNodeStruct()->q[VIRTUAL_OBJECT+1];
    pos[2] = currentNode->getNodeStruct()->q[VIRTUAL_OBJECT+2];

    API::Cell* cell = _3DGrid->getCell(pos);

    _forward = false;

    if( currentNode->getCompcoStruct()->num == mGraph->getStart()->getCompcoStruct()->num )
    {
        _forward = true;
    }

    if( _forward )
    {
        if(_LastForward != cell)
        {
            if( on3DPathAndAfter( cell ) )
            {
                _LastForward = cell;
            }
        }
    }
    else
    {
        if(_LastForward != cell)
        {
            if( on3DPathAndAfter( cell ) )
            {
                _LastBackward = cell;
            }
        }
    }

    return newNode;
}
