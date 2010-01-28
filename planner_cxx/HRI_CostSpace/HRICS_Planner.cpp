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
#include "../API/3DGrid/points.h"
#include "../../qtWindow/cppToQt.hpp"
#include "../../lightPlanner/proto/lightPlannerApi.h"

#include "../../other_libraries/Eigen/Array"

using namespace std;
using namespace tr1;
using namespace HRICS;

HRICS::MainPlanner* HRICS_MOPL;
int VIRTUAL_OBJECT_DOF;

MainPlanner::MainPlanner() : Planner()
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
            _Humans.push_back(new Robot(XYZ_ENV->robot[i]));
            cout << "Humans is " << name << endl;
        }
    }

    p3d_jnt* FF_Joint = _Robot->getRobotStruct()->ccCntrts[0]->actjnts[0];
    ENV.setInt(Env::akinJntId,FF_Joint->num);
    VIRTUAL_OBJECT_DOF = FF_Joint->index_dof;
    cout << "VIRTUAL_OBJECT_DOF Joint is " << VIRTUAL_OBJECT_DOF << endl;

    p3d_del_graph(XYZ_GRAPH);
    XYZ_GRAPH = NULL;
    _Graph = new Graph(this->getActivRobot(),XYZ_GRAPH);
    _3DPath.resize(0);
}

MainPlanner::MainPlanner(Robot* rob, Graph* graph) : Planner(rob, graph)
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
            _Humans.push_back(new Robot(XYZ_ENV->robot[i]));
            cout << "Humans is " << name << endl;
        }
    }
    _3DPath.resize(0);
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
    
    _3DGrid = new Grid(pace,envSize);
    
    _3DGrid->setRobot(_Robot);

    BiasedCell = _3DGrid->getCell(0,0,0);
    cout << "Biased Cell is " << BiasedCell << endl;
#ifdef QT_LIBRARY
    std::string str = "g3d_draw_allwin_active";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
#endif
}

void MainPlanner::initDistance()
{
    _Distance = new Distance(_Robot,_Humans);
    cout << "Robot " << _Robot->getName() << endl;
    cout << "Robot get struct " << _Robot->getRobotStruct() << endl;
    _Distance->parseHumans();
}


/*
 * Free Flyer Astar
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
    
    pos[0] = config->getConfigStruct()[VIRTUAL_OBJECT_DOF+0];
    pos[1] = config->getConfigStruct()[VIRTUAL_OBJECT_DOF+1];
    pos[2] = config->getConfigStruct()[VIRTUAL_OBJECT_DOF+2];
    
    Cell* startCell = dynamic_cast<Cell*>(_3DGrid->getCell(pos));
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
            _3DGrid);
    
    config = _Robot->getGoTo();
    
    pos[0] = config->getConfigStruct()[VIRTUAL_OBJECT_DOF+0];
    pos[1] = config->getConfigStruct()[VIRTUAL_OBJECT_DOF+1];
    pos[2] = config->getConfigStruct()[VIRTUAL_OBJECT_DOF+2];
    
    Cell* goalCell = dynamic_cast<Cell*>(_3DGrid->getCell(pos));
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
            _3DGrid);
    
    solveAStar(start,goal);
    
    for( int i=0; i< _3DPath.size() ; i++ )
    {
        cout << "Cell "<< i <<" = " << endl << _3DPath[i] << endl;
    }
    
    
    //    Trajectory* traj = new Trajectory(new Robot(XYZ_ROBOT));
    //
    //    traj->replaceP3dTraj();
    //    string str = "g3d_draw_allwin_active";
    //    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
    //    ENV.setBool(Env::drawTraj,true);
    //    cout << "solution : End Search" << endl;
}

void MainPlanner::solveAStar(State* start,State* goal)
{
    _3DPath.clear();
    _3DCellPath.clear();

    shared_ptr<Configuration> config = _Robot->getCurrentPos();
    
    /*
    * Change the way AStar
    * is computed to go down
    */
    if( start->getCell()->getCost() < goal->getCell()->getCost() )
    {
        AStar* search = new AStar(start);
        vector<API::State*> path = search->solve(goal);
        
        if(path.size() == 0 )
        {
            return;
        }
        
        for (int i=0;i<path.size();i++)
        {
            API::Cell* cell = dynamic_cast<State*>(path[i])->getCell();
            _3DPath.push_back( cell->getCenter() );
            _3DCellPath.push_back( cell );
            
        }
    }
    else
    {
        AStar* search = new AStar(goal);
        vector<API::State*> path = search->solve(start);
        
        if(path.size() == 0 )
        {
            return;
        }
        
        for (int i=path.size()-1;i>=0;i--)
        {
            API::Cell* cell = dynamic_cast<State*>(path[i])->getCell();
            _3DPath.push_back( cell->getCenter() );
            _3DCellPath.push_back( cell );
        }
    }
    
    return;
}

void MainPlanner::draw3dPath()
{
    for(int i=0;i<_3DPath.size()-1;i++)
    {
        glLineWidth(3.);
        g3d_drawOneLine(_3DPath[i][0],      _3DPath[i][1],      _3DPath[i][2],
                        _3DPath[i+1][0],    _3DPath[i+1][1],    _3DPath[i+1][2],
                        Yellow, NULL);
        glLineWidth(1.);
    }
}

double MainPlanner::distanceToEntirePath()
{
    double minDist = numeric_limits<double>::max();
    
    Vector3d point;
    Vector3d interPolSaved;
    
    shared_ptr<Configuration> config = _Robot->getCurrentPos();
    
    point[0] = config->getConfigStruct()[VIRTUAL_OBJECT_DOF+0];
    point[1] = config->getConfigStruct()[VIRTUAL_OBJECT_DOF+1];
    point[2] = config->getConfigStruct()[VIRTUAL_OBJECT_DOF+2];
    
    double nbSamples = 20;
    
    for(int i=0;i<_3DPath.size()-1;i++)
    {   
        for(int j=0;j<nbSamples;j++)
        {
            double alpha = (double)(j/nbSamples);
            Vector3d interpol = _3DPath[i] + alpha*(_3DPath[i+1] - _3DPath[i]);
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

        _Distance->setVector(vect_jim);
    }
    
    //    cout << "minDist = " <<minDist << endl;
    return 100*minDist;
}


double MainPlanner::distanceToCellPath()
{
    double minDist = numeric_limits<double>::max();
    
    Vector3d point;
    
    shared_ptr<Configuration> config = _Robot->getCurrentPos();
    
    point[0] = config->getConfigStruct()[VIRTUAL_OBJECT_DOF+0];
    point[1] = config->getConfigStruct()[VIRTUAL_OBJECT_DOF+1];
    point[2] = config->getConfigStruct()[VIRTUAL_OBJECT_DOF+2];
    
    for(int i=0;i<_3DPath.size();i++)
    {
        double dist = ( point - _3DPath[i] ).norm();
        if( minDist > dist )
        {
            minDist = dist;
        }
    }
    
    cout << "minDist = " <<minDist << endl;
    return 100*minDist;
}

bool MainPlanner::runHriRRT()
{
    p3d_del_graph(XYZ_GRAPH);
    XYZ_GRAPH = NULL;
    _Graph = new Graph(this->getActivRobot(),XYZ_GRAPH);

    if(ENV.getBool(Env::drawPoints)&&PointsToDraw)
    {
        delete PointsToDraw;
        PointsToDraw = NULL;
    }

    ENV.setBool(Env::CostBeforeColl,true);

    if(ENV.getBool(Env::isInverseKinematics))
    {
        activateCcCntrts(_Robot->getRobotStruct(),-1,true);
    }
    else
    {
        deactivateCcCntrts(_Robot->getRobotStruct(),-1);//true);
    }

    RRT* rrt = new HRICS_RRT(_Robot,_Graph);

    int nb_added_nodes = rrt->init();
    cout << "nb nodes "<< _Graph->getNodes().size() << endl;

    dynamic_cast<HRICS_RRT*>(rrt)->setGrid(_3DGrid);
    dynamic_cast<HRICS_RRT*>(rrt)->setCellPath(_3DCellPath);

    //    ENV.setBool(Env::biDir,true);
    //    ENV.setBool(Env::isGoalBiased,true);
    //    ENV.setDouble(Env::Bias,0.5);

    ENV.setBool(Env::isRunning,true);
    nb_added_nodes += rrt->run();

    cout << "nb added nodes " << nb_added_nodes << endl;
    cout << "nb nodes " << _Graph->getNodes().size() << endl;

    bool trajFound = rrt->trajFound();

    if(trajFound)
    {
        p3d_ExtractBestTraj(_Graph->getGraphStruct());
        BaseOptimization traj(_Robot,_Robot->getTrajStruct());
        traj.runShortCut(ENV.getInt(Env::nbCostOptimize));
        traj.replaceP3dTraj();
    }

    return trajFound;
}
