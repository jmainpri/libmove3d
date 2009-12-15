/*
 *  HRICS_CostSpace.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 04/12/09.
 *  Copyright 2009 mainprice@gmail.com All rights reserved.
 *
 */

#include "HRICS_Planner.h"
#include "HRICS_Grid.h"
#include "HRICS_GridState.h"
#include "RRT/HRICS_rrt.h"

#include "../../qtWindow/cppToQt.hpp"

#include "../../other_libraries/Eigen/Array"

using namespace std;
using namespace tr1;
using namespace HRICS;

HRICS::MainPlanner* HRICS_MOPL;

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

    p3d_del_graph(XYZ_GRAPH);
    XYZ_GRAPH = NULL;
    _Graph = new Graph(this->getActivRobot(),XYZ_GRAPH);
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
    
    std::string str = "g3d_draw_allwin_active";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
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
    
    pos[0] = config->getConfigStruct()[VIRTUAL_OBJECT+0];
    pos[1] = config->getConfigStruct()[VIRTUAL_OBJECT+1];
    pos[2] = config->getConfigStruct()[VIRTUAL_OBJECT+2];
    
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
    
    pos[0] = config->getConfigStruct()[VIRTUAL_OBJECT+0];
    pos[1] = config->getConfigStruct()[VIRTUAL_OBJECT+1];
    pos[2] = config->getConfigStruct()[VIRTUAL_OBJECT+2];
    
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
            Vector3d cellCenter = dynamic_cast<State*>(path[i])->getCell()->getCenter();
            _3DPath.push_back( cellCenter );
            
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
            Vector3d cellCenter = dynamic_cast<State*>(path[i])->getCell()->getCenter();
            _3DPath.push_back( cellCenter );
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
                        Green, NULL);
        glLineWidth(1.);
    }
}

double MainPlanner::distanceToEntirePath()
{
    double minDist = numeric_limits<double>::max();
    
    Vector3d point;
    
    shared_ptr<Configuration> config = _Robot->getCurrentPos();
    
    point[0] = config->getConfigStruct()[VIRTUAL_OBJECT+0];
    point[1] = config->getConfigStruct()[VIRTUAL_OBJECT+1];
    point[2] = config->getConfigStruct()[VIRTUAL_OBJECT+2];
    
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
            }
        }
    }
    
    //    cout << "minDist = " <<minDist << endl;
    return 10*minDist;
}

double MainPlanner::distanceToCellPath()
{
    double minDist = numeric_limits<double>::max();
    
    Vector3d point;
    
    shared_ptr<Configuration> config = _Robot->getCurrentPos();
    
    point[0] = config->getConfigStruct()[VIRTUAL_OBJECT+0];
    point[1] = config->getConfigStruct()[VIRTUAL_OBJECT+1];
    point[2] = config->getConfigStruct()[VIRTUAL_OBJECT+2];
    
    for(int i=0;i<_3DPath.size();i++)
    {
        double dist = ( point - _3DPath[i] ).norm();
        if(minDist > dist )
        {
            minDist = dist;
        }
    }
    
    cout << "minDist = " <<minDist << endl;
    return 100*minDist;
}

bool MainPlanner::runHriRRT()
{
    _RRT = new HRICS_RRT(_Robot,_Graph);

    dynamic_cast<HRICS_RRT*>(_RRT)->setGrid(_3DGrid);
    int nb_added_nodes = _RRT->init();
    cout << "nb nodes "<< _Graph->getNodes().size() << endl;

    ENV.setBool(Env::isRunning,true);
    nb_added_nodes += _RRT->run();
    ENV.setBool(Env::isRunning,false);

    cout << "nb added nodes " << nb_added_nodes << endl;
    cout << "nb nodes " << _Graph->getNodes().size() << endl;
    return _RRT->trajFound();
}
