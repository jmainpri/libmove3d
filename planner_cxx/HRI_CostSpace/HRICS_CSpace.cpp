#include "HRICS_CSpace.h"
#include "Grid/HRICS_Grid.h"

using namespace std;
using namespace tr1;
using namespace HRICS;

const int HUMANj_NECK_PAN=  4;
const int HUMANj_NECK_TILT= 7; // 5

const double HRI_EYE_TOLERANCE_TILT=0.3;
const double HRI_EYE_TOLERANCE_PAN=0.3;

CSpace* HRICS_CSpaceMPL=NULL;

/**
  * Reads the ENV structure and gets the Humans and the Robots named respectivly
  * HUMAN and ROBOT and performs init
  */
CSpace::CSpace() : Planner() , mPathExist(false)
{
    cout << "New ConfigSpace HRI planner" << endl;

    for (int i=0; i<XYZ_ENV->nr; i++)
    {
        string name(XYZ_ENV->robot[i]->name);

        if(name.find("ROBOT") != string::npos )
        {
            _Robot = new Robot(XYZ_ENV->robot[i]);
            cout << "Robot is " << name << endl;
        }

        if(name.find("HUMAN") != string::npos )
        {
            mHuman = new Robot(XYZ_ENV->robot[i]);
            cout << "Humans is " << name << endl;
        }
    }

    initCostSpace();
}

/**
  * Take as input a robot and a human and performs init
  */
CSpace::CSpace(Robot* R, Robot* H) : Planner() , mHuman(H) , mPathExist(false)
{
    this->setRobot(R);
    initCostSpace();
}

CSpace::~CSpace()
{
    delete mDistance;
    delete m3DGrid;
}

/**
  * Gets the
  */
void CSpace::initCostSpace()
{
    mEnvSize.resize(6);
    mEnvSize[0] = XYZ_ENV->box.x1; mEnvSize[1] = XYZ_ENV->box.x2;
    mEnvSize[2] = XYZ_ENV->box.y1; mEnvSize[3] = XYZ_ENV->box.y2;
    mEnvSize[4] = XYZ_ENV->box.z1; mEnvSize[5] = XYZ_ENV->box.z2;

    p3d_jnt* FF_Joint = _Robot->getRobotStruct()->ccCntrts[0]->actjnts[0];
    ENV.setInt(Env::akinJntId,FF_Joint->num);
    mIndexObjectDof = _Robot->getObjectDof();
    cout << "mIndexObjectDof Joint is " << mIndexObjectDof << endl;
    cout << " Type of test is "  << ENV.getInt(Env::hriCostType) << endl;

    vector<Robot*> Humans;
    Humans.push_back(mHuman);

    mDistance = new Distance(/*_Robot,Humans*/);
    mDistance->parseHumans();

    m3DGrid = new Grid(ENV.getDouble(Env::CellSize),mEnvSize);
    m3DGrid->setRobot(_Robot);

    mEnvSize.resize(4);
    m2DGrid = new PlanGrid(ENV.getDouble(Env::CellSize),mEnvSize);
    m2DGrid->setRobot(_Robot);
}

double CSpace::getConfigCost()
{
//    ENV.setBool(Env::useBoxDist,true);
    mDistCost = ENV.getDouble(Env::Kdistance)*getDistanceCost();
//    mVisibilityPoint = Vector3d::Random();
    mVisiCost = ENV.getDouble(Env::Kvisibility)*getVisibilityCost(mVisibilityPoint);

//    cout << "DistCost = "  << DistCost << endl;
//    cout << "VisibCost = "  << VisiCost << endl;

    return  mDistCost + mVisiCost;
}

void CSpace::computeDistanceGrid()
{
    mDistance->setSafeRadius(ENV.getDouble(Env::zone_size));

    ENV.setBool(Env::drawGrid,true);
    ENV.setInt(Env::hriCostType,0);
    ENV.setBool(Env::useBoxDist,false);
    ENV.setBool(Env::useBallDist,true);
    m3DGrid->computeAllCellCost();
    API_activeGrid = m3DGrid;
}

void CSpace::computeVisibilityGrid()
{
    ENV.setBool(Env::drawGrid,true);
    ENV.setInt(Env::hriCostType,1);
//    ENV.setBool(Env::useBoxDist,false);
//    ENV.setBool(Env::useBallDist,true);
    m3DGrid->computeAllCellCost();
    API_activeGrid = m3DGrid;
}

double CSpace::getDistanceCost()
{
    double Cost = mDistance->getDistToZones()[0];
    mVisibilityPoint = mDistance->getColsestPointToHuman();
//    getVisibilityCost(mVisibilityPoint);
    return Cost;
}

double CSpace::getVisibilityCost(Vector3d WSPoint)
{
    double phi,theta;
    double Dphi, Dtheta;
//    double Ccoord[6];
    p3d_vector4 realcoord,newcoord;
    p3d_matrix4 inv;

    realcoord[0] = WSPoint[0];
    realcoord[1] = WSPoint[1];
    realcoord[2] = WSPoint[2];
    realcoord[3] = 1;

    // get the right frame
    p3d_matrix4 rotation = {
        {-1,0,0,0},
        {0,-1,0,0},
        {0,0,1,0},
        {0,0,0,1}};
    p3d_matrix4 newABS;
    p3d_mat4Mult(mHuman->getRobotStruct()->joints[HUMANj_NECK_TILT]->abs_pos,rotation,newABS);

    // Invert frame and get the point in this frame
    p3d_matInvertXform(
            newABS, inv);
    p3d_matvec4Mult(inv, realcoord, newcoord);


    // Compute the angle the point make with the
    Vector3d newCoordVect;
    newCoordVect[0] = newcoord[0];
    newCoordVect[1] = newcoord[1];
    newCoordVect[2] = newcoord[2];

    phi = ABS(atan2( newCoordVect[0],newCoordVect[1]));
    theta = ABS(acos( newCoordVect[2]/newCoordVect.norm() )- M_PI_2);

    if(phi < HRI_EYE_TOLERANCE_PAN/2.)
        Dphi = 0;
    else
        Dphi = phi - HRI_EYE_TOLERANCE_PAN/2.;

    if(theta < HRI_EYE_TOLERANCE_TILT/2.)
        Dtheta = 0;
    else
        Dtheta = theta - HRI_EYE_TOLERANCE_TILT/2.;

    double cost = (1/0.65)*((Dtheta+Dphi)/(M_2PI-(HRI_EYE_TOLERANCE_TILT/2.)-(HRI_EYE_TOLERANCE_PAN/2.)));

//    cout << "Visib =  "  << cost << endl;
    return cost;
}

/**
  * Takes the robot initial config and calls the solve A*
  * to compute the 2D path
  */
bool CSpace::computeAStarIn2DGrid()
{
    //	if(!ENV.getBool(Env::isHriTS))
    //	{
    //		return this->computeAStar();
    //	}
    //
    ENV.setBool(Env::drawTraj,false);

    shared_ptr<Configuration> config = _Robot->getInitialPosition();

    config->print();

    Vector2d pos;

    pos[0] = config->at(6);
    pos[1] = config->at(7);

    PlanCell* startCell = dynamic_cast<PlanCell*>(m2DGrid->getCell(pos));
    Vector2i startCoord = startCell->getCoord();

    cout << "Start Pos = (" <<
            pos[0] << " , " <<
            pos[1] << ")" << endl;

    cout << "Start Coord = (" <<
            startCoord[0] << " , " <<
            startCoord[1] << ")" << endl;

    PlanState* start = new PlanState(
            startCell,
            m2DGrid);

    config = _Robot->getGoTo();

    pos[0] = config->at(6);
    pos[1] = config->at(7);

    PlanCell* goalCell = dynamic_cast<PlanCell*>(m2DGrid->getCell(pos));
    Vector2i goalCoord = goalCell->getCoord();

    cout << "Goal Pos = (" <<
            pos[0] << " , " <<
            pos[1] << ")" << endl;

    cout << "Goal Coord = (" <<
            goalCoord[0] << " , " <<
            goalCoord[1] << ")" << endl;

    if( startCoord == goalCoord )
    {
        cout << " no planning as cells are identical" << endl;
        return false;
    }

    PlanState* goal = new PlanState(
            goalCell,
            m2DGrid);

    solveAStar(start,goal);

    for( int i=0; i< m2DPath.size() ; i++ )
    {
        cout << "Cell "<< i <<" = " << endl << m2DPath[i] << endl;
        cout << "Cost =" << dynamic_cast<PlanCell*>(m2DGrid->getCell(m2DPath[i]))->getCost() << endl;
    }

    //    Trajectory* traj = new Trajectory(new Robot(XYZ_ROBOT));
    //
    //    traj->replaceP3dTraj();
    //    string str = "g3d_draw_allwin_active";
    //    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
    //    ENV.setBool(Env::drawTraj,true);
    //    cout << "solution : End Search" << endl;
}

/**
  * Solve A Star in a 2D grid using the API A Star on
  * takes as input A* States
  */
void CSpace::solveAStar(PlanState* start,PlanState* goal)
{
    m2DPath.clear();
//    m2DCellPath.clear();

//    shared_ptr<Configuration> config = _Robot->getCurrentPos();

    /*
    * Change the way AStar
    * is computed to go down
    */
    if( start->getCell()->getCost() < goal->getCell()->getCost() )
    {
        API::AStar* search = new API::AStar(start);
        vector<API::State*> path = search->solve(goal);

        if(path.size() == 0 )
        {
            m2DPath.clear();
//            m3DCellPath.clear();
            mPathExist = false;
            return;
        }

        for (int i=0;i<path.size();i++)
        {
            API::TwoDCell* cell = dynamic_cast<PlanState*>(path[i])->getCell();
            m2DPath.push_back( cell->getCenter() );
//            m3DCellPath.push_back( cell );
        }
    }
    else
    {
        API::AStar* search = new API::AStar(goal);
        vector<API::State*> path = search->solve(start);

        if(path.size() == 0 )
        {
            m2DPath.clear();
//            m3DCellPath.clear();
            mPathExist = false;
            return;
        }

        for (int i=path.size()-1;i>=0;i--)
        {
            API::TwoDCell* cell = dynamic_cast<PlanState*>(path[i])->getCell();
            m2DPath.push_back( cell->getCenter() );
//            m3DCellPath.push_back( cell );
        }
    }

    mPathExist = true;
    return;
}

/**
  * Draws the 3D path as a yellow line
  */
void CSpace::draw2dPath()
{
    if( mPathExist)
    {
//        cout << "Drawing 2D path" << endl;
        for(int i=0;i<m2DPath.size()-1;i++)
        {
            glLineWidth(3.);
            g3d_drawOneLine(m2DPath[i][0],      m2DPath[i][1],      0.4,
                            m2DPath[i+1][0],    m2DPath[i+1][1],    0.4,
                            Yellow, NULL);
            glLineWidth(1.);
        }
    }
}
