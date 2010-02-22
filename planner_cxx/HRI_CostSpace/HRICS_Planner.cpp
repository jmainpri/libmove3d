/*
 *  HRICS_CostSpace.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 04/12/09.
 *  Copyright 2009 mainprice@gmail.com All rights reserved.
 *
 */

#include "HRICS_Planner.h"
#include "Grid/HRICS_Grid.h"
#include "Grid/HRICS_GridState.h"
#include "RRT/HRICS_rrt.h"
#include "RRT/HRICS_rrtExpansion.h"
#include "../Diffusion/RRT-Variants/Transition-RRT.hpp"
//#include "../../qtWindow/cppToQt.hpp"
#include "API/Trajectory/BaseOptimization.hpp"

#include "../../other_libraries/Eigen/Array"

using namespace std;
using namespace tr1;
using namespace HRICS;

HRICS::MainPlanner* HRICS_MOPL;

MainPlanner::MainPlanner() : Planner() , mPathExist(false)
{
    cout << "New MainPlanner" << endl;
    
    for (int i=0; i<XYZ_ENV->nr; i++)
    {
        string name(XYZ_ENV->robot[i]->name);
        
        if(name.find("ROBOT") != string::npos )
        {
            this->setRobot(new Robot(XYZ_ENV->robot[i]));
            cout << "Robot is " << name << endl;
            cout << "Robot " << _Robot->getName() << endl;
            cout << "Robot get struct " << _Robot->getRobotStruct() << endl;
        }
        
        if(name.find("HUMAN") != string::npos )
        {
            mHumans.push_back(new Robot(XYZ_ENV->robot[i]));
            cout << "Humans is " << name << endl;
        }
    }

    p3d_jnt* FF_Joint = _Robot->getRobotStruct()->ccCntrts[0]->actjnts[0];
    ENV.setInt(Env::akinJntId,FF_Joint->num);
    
    mIndexObjectDof = _Robot->getObjectDof() ;
    cout << "VIRTUAL_OBJECT_DOF Joint is " << mIndexObjectDof << endl;
    cout << "HRI Cost type is "  << ENV.getInt(Env::hriCostType) << endl;
    cout << "Ball Dist is " << ENV.getBool(Env::useBallDist) << endl;

    ENV.setBool(Env::useBallDist,true);

    p3d_del_graph(XYZ_GRAPH);
    XYZ_GRAPH = NULL;
    _Graph = new Graph(this->getActivRobot(),XYZ_GRAPH);
    m3DPath.clear();
}

MainPlanner::MainPlanner(Robot* rob, Graph* graph) :
        Planner(rob, graph) , mPathExist(false)
{
    cout << "Robot is " << rob->getName() << endl;
    
    if(rob->getName().find("ROBOT") == string::npos )
    {
        cout << "MainPlanner::Error robot des not contain ROBOT" << endl;
    }
    
    for (int i=0; i<XYZ_ENV->nr; i++)
    {
        string name(XYZ_ENV->robot[i]->name);
        
        if(name.find("HUMAN") != string::npos )
        {
            mHumans.push_back(new Robot(XYZ_ENV->robot[i]));
            cout << "Humans is " << name << endl;
        }
    }
    m3DPath.clear();
}

MainPlanner::~MainPlanner()
{
    delete (this->getGrid());
    delete (this->getDistance());
}


void MainPlanner::initGrid()
{
    //    vector<int> size;
    vector<double>  envSize(6);
    envSize[0] = XYZ_ENV->box.x1; envSize[1] = XYZ_ENV->box.x2;
    envSize[2] = XYZ_ENV->box.y1; envSize[3] = XYZ_ENV->box.y2;
    envSize[4] = XYZ_ENV->box.z1; envSize[5] = XYZ_ENV->box.z2;
    
    double pace = ENV.getDouble(Env::CellSize);
    
    //    GridToGraph theGrid(pace,envSize);
    //    theGrid.putGridInGraph();
    
    m3DGrid = new Grid(pace,envSize);
    
    m3DGrid->setRobot(_Robot);

    BiasedCell = m3DGrid->getCell(0,0,0);
    cout << "Biased Cell is " << BiasedCell << endl;
    //#ifdef QT_LIBRARY
    //    std::string str = "g3d_draw_allwin_active";
    //    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
    //#endif
}

void MainPlanner::initDistance()
{
    mDistance = new Distance(_Robot,mHumans);
    cout << "Robot " << _Robot->getName() << endl;
    cout << "Robot get struct " << _Robot->getRobotStruct() << endl;
    mDistance->parseHumans();
}


/**
  * Takes the robot initial config and calls the solve A*
  * to compute the 3D path
  */
bool MainPlanner::computeAStarIn3DGrid()
{
    //	if(!ENV.getBool(Env::isHriTS))
    //	{
    //		return this->computeAStar();
    //	}
    //
    ENV.setBool(Env::drawTraj,false);
    
    shared_ptr<Configuration> config = _Robot->getInitialPosition();
    
    config->print();
    
    Vector3d pos;
    
    pos[0] = config->at(mIndexObjectDof+0);
    pos[1] = config->at(mIndexObjectDof+1);
    pos[2] = config->at(mIndexObjectDof+2);
    
    Cell* startCell = dynamic_cast<Cell*>(m3DGrid->getCell(pos));
    Vector3i startCoord = startCell->getCoord();
    
    cout << "Start Pos = (" <<
            pos[0] << " , " <<
            pos[1] << " , " <<
            pos[2] << ")" << endl;
    
    cout << "Start Coord = (" <<
            startCoord[0] << " , " <<
            startCoord[1] << " , " <<
            startCoord[2] << ")" << endl;
    
    State* start = new State(
            startCell,
            m3DGrid);
    
    config = _Robot->getGoTo();
    
    pos[0] = config->at(mIndexObjectDof+0);
    pos[1] = config->at(mIndexObjectDof+1);
    pos[2] = config->at(mIndexObjectDof+2);
    
    Cell* goalCell = dynamic_cast<Cell*>(m3DGrid->getCell(pos));
    Vector3i goalCoord = goalCell->getCoord();
    
    cout << "Goal Pos = (" <<
            pos[0] << " , " <<
            pos[1] << " , " <<
            pos[2] << ")" << endl;
    
    cout << "Goal Coord = (" <<
            goalCoord[0] << " , " <<
            goalCoord[1] << " , " <<
            goalCoord[2] << ")" << endl;
    
    if( startCoord == goalCoord )
    {
        cout << " no planning as cells are identical" << endl;
        return false;
    }
    
    State* goal = new State(
            goalCell,
            m3DGrid);
    
    solveAStar(start,goal);

    if(mPathExist)
    {
        cout << " Path Cost = "  << pathCost() <<  endl;
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
  * Solve A Star in a 3D grid using the API A Star on
  * takes as input A* States
  */
void MainPlanner::solveAStar(State* start,State* goal)
{
    m3DPath.clear();
    m3DCellPath.clear();

    shared_ptr<Configuration> config = _Robot->getCurrentPos();
    
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
            m3DPath.clear();
            m3DCellPath.clear();
            mPathExist = false;
            return;
        }
        
        for (int i=0;i<path.size();i++)
        {
            API::ThreeDCell* cell = dynamic_cast<State*>(path[i])->getCell();
            m3DPath.push_back( cell->getCenter() );
            m3DCellPath.push_back( cell );
        }
    }
    else
    {
        API::AStar* search = new API::AStar(goal);
        vector<API::State*> path = search->solve(start);
        
        if(path.size() == 0 )
        {
            m3DPath.clear();
            m3DCellPath.clear();
            mPathExist = false;
            return;
        }
        
        for (int i=path.size()-1;i>=0;i--)
        {
            API::ThreeDCell* cell = dynamic_cast<State*>(path[i])->getCell();
            m3DPath.push_back( cell->getCenter() );
            m3DCellPath.push_back( cell );
        }
    }
    
    mPathExist = true;
    return;
}

double MainPlanner::pathCost()
{
    if( m3DPath.size() != m3DCellPath.size() )
    {
        cout << "Error:pathCost() => m3DPath.size() != m3DCellPath.size()" << endl;
    }
    Vector3d currentPos, prevPos;
    double currentCost, prevCost;

    prevCost = dynamic_cast<Cell*>(m3DCellPath[0])->getCost();
    prevPos = m3DPath[0];

    double SumOfCost=0.0;
    double distStep;

    for (unsigned int i = 1; i < m3DPath.size(); i++)
    {
        currentCost = dynamic_cast<Cell*>(m3DCellPath[i])->getCost();
        currentPos = m3DPath[i];

        // Case of task space
        distStep = ( currentPos - prevPos ).norm();

//        cout << "Current Cost = " << currentCost << endl;
//        cout << "Delta Cost = " << p3d_ComputeDeltaStepCost(prevCost, currentCost, distStep) << endl;

        SumOfCost += p3d_ComputeDeltaStepCost(prevCost, currentCost, distStep);

        prevCost = currentCost;
        prevPos = currentPos;
    }

    return SumOfCost;
}

/**
  * Draws the 3D path as a yellow line
  */
void MainPlanner::draw3dPath()
{
    if( mPathExist)
    {
        for(int i=0;i<m3DPath.size()-1;i++)
        {
            glLineWidth(3.);
            g3d_drawOneLine(m3DPath[i][0],      m3DPath[i][1],      m3DPath[i][2],
                            m3DPath[i+1][0],    m3DPath[i+1][1],    m3DPath[i+1][2],
                            Yellow, NULL);
            glLineWidth(1.);
        }
    }
}

/**
  * Computes a distance from the robot
  * Current config to the 3D path
  */
double MainPlanner::distanceToEntirePath()
{
    double minDist = numeric_limits<double>::max();
    
    Vector3d point;
    Vector3d interPolSaved;
    
    shared_ptr<Configuration> config = _Robot->getCurrentPos();
    
    point[0] = config->at(mIndexObjectDof+0);
    point[1] = config->at(mIndexObjectDof+1);
    point[2] = config->at(mIndexObjectDof+2);
    
    double nbSamples = 20;
    
    for(int i=0;i<m3DPath.size()-1;i++)
    {   
        for(int j=0;j<nbSamples;j++)
        {
            double alpha = (double)(j/nbSamples);
            Vector3d interpol = m3DPath[i] + alpha*(m3DPath[i+1] - m3DPath[i]);
            double dist = ( point - interpol ).norm();
            if(minDist > dist )
            {
                minDist = dist;
                interPolSaved = interpol;
            }
        }
    }

    if(ENV.getBool(Env::drawDistance))
    {
        vector<double> vect_jim;

        vect_jim.push_back(point[0]);
        vect_jim.push_back(point[1]);
        vect_jim.push_back(point[2]);
        vect_jim.push_back(interPolSaved[0]);
        vect_jim.push_back(interPolSaved[1]);
        vect_jim.push_back(interPolSaved[2]);

        mDistance->setVector(vect_jim);
    }
    
    //    cout << "minDist = " <<minDist << endl;
    return 100*minDist;
}


/**
  * Computes a distance to the Cells in the 3D Path
  * Coarse grain compared to the above distance
  */
double MainPlanner::distanceToCellPath()
{
    double minDist = numeric_limits<double>::max();
    
    Vector3d point;
    
    shared_ptr<Configuration> config = _Robot->getCurrentPos();
    
    point[0] = config->at(mIndexObjectDof+0);
    point[1] = config->at(mIndexObjectDof+1);
    point[2] = config->at(mIndexObjectDof+2);
    
    for(int i=0;i<m3DPath.size();i++)
    {
        double dist = ( point - m3DPath[i] ).norm();
        if( minDist > dist )
        {
            minDist = dist;
        }
    }
    
    cout << "minDist = " <<minDist << endl;
    return 100*minDist;
}

/**
  * Runs a HRI RRT
  */
bool MainPlanner::initHriRRT()
{
    p3d_del_graph(XYZ_GRAPH);
    XYZ_GRAPH = NULL;
    _Graph = new Graph(this->getActivRobot(),XYZ_GRAPH);

    if(ENV.getBool(Env::drawPoints)&&PointsToDraw)
    {
        delete PointsToDraw;
        PointsToDraw = NULL;
    }

    //    ENV.setBool(Env::costBeforeColl,true);

    if(ENV.getBool(Env::isInverseKinematics))
    {
        activateCcCntrts(_Robot->getRobotStruct(),-1,true);
    }
    else
    {
        deactivateCcCntrts(_Robot->getRobotStruct(),-1);//true);
    }

    cout << " -----------------------------------------------" << endl;
    cout << " HRISCS Workspace RRT Initialized : "  << endl;
    cout << " Inverse Kinemactics : " << ENV.getBool(Env::isInverseKinematics) << endl;
    cout << " Number of Cell : "  << m3DCellPath.size() << endl;
    cout << " Path Cost : "  << pathCost() << endl;

    return true;
}

const int HUMANj_NECK_PAN=  4;
const int HUMANj_NECK_TILT= 7; // 5

const double HRI_EYE_TOLERANCE_TILT=0.3;
const double HRI_EYE_TOLERANCE_PAN=0.3;

double MainPlanner::getVisibilityCost(Vector3d WSPoint)
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
    p3d_mat4Mult(mHumans[0]->getRobotStruct()->joints[HUMANj_NECK_TILT]->abs_pos,rotation,newABS);

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
