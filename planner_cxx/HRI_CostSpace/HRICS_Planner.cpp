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
#include "../../qtWindow/cppToQt.hpp"

using namespace std;
using namespace tr1;

HRICS_Planner* HRICS_MOPL;

HRICS_Planner::HRICS_Planner() : Planner()
{
    cout << "New HRICS_Planner" << endl;

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

    this->setGraph(new Graph(this->getActivRobot(),XYZ_GRAPH));
}



HRICS_Planner::HRICS_Planner(Robot* rob, Graph* graph) : Planner(rob, graph) 
{
    cout << "Robot is " << rob->getName() << endl;

    if(rob->getName().find("ROBOT") == string::npos )
    {
        cout << "HRICS_Planner::Error robot des not contain ROBOT" << endl;
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

HRICS_Planner::~HRICS_Planner()
{
    delete (this->getGrid());
    delete (this->getDistance());
}

void HRICS_Planner::initGrid()
{
    //    vector<int> size;
    vector<double>  envSize(6);
    envSize[0] = XYZ_ENV->box.x1; envSize[1] = XYZ_ENV->box.x2;
    envSize[2] = XYZ_ENV->box.y1; envSize[3] = XYZ_ENV->box.y2;
    envSize[4] = XYZ_ENV->box.z1; envSize[5] = XYZ_ENV->box.z2;

    double pace = ENV.getDouble(Env::CellSize);

    //    GridToGraph theGrid(pace,envSize);
    //    theGrid.putGridInGraph();

    _3DGrid = new HriGrid(pace,envSize);

    _3DGrid->setRobot(_Robot);

    std::string str = "g3d_draw_allwin_active";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}


/*
 * Free Flyer Astar
 */
bool HRICS_Planner::computeAStarIn3DGrid()
{
    //	if(!ENV.getBool(Env::isHriTS))
    //	{
    //		return this->computeAStar();
    //	}
    //
    ENV.setBool(Env::drawTraj,false);

    shared_ptr<Configuration> config = _Robot->getCurrentPos();

    config->print();

    Vector3d pos;

    pos[0] = config->getConfigStruct()[VIRTUAL_OBJECT+0];
    pos[1] = config->getConfigStruct()[VIRTUAL_OBJECT+1];
    pos[2] = config->getConfigStruct()[VIRTUAL_OBJECT+2];

    HriCell* startCell = dynamic_cast<HriCell*>(_3DGrid->getCell(pos));
    Vector3i startCoord = startCell->getCoord();

    cout << "Start Pos = (" << pos[0] << " , " << pos[1] << " , " << pos[2] << ")" << endl;
    cout << "Start Coord = (" << startCoord[0] << " , " << startCoord[1] << " , " << startCoord[2] << ")" << endl;

    HriGridState* start = new HriGridState(
            startCell,
            _3DGrid);

    config = _Robot->getGoTo();

    pos[0] = config->getConfigStruct()[VIRTUAL_OBJECT+0];
    pos[1] = config->getConfigStruct()[VIRTUAL_OBJECT+1];
    pos[2] = config->getConfigStruct()[VIRTUAL_OBJECT+2];

    HriCell* goalCell = dynamic_cast<HriCell*>(_3DGrid->getCell(pos));
    Vector3i goalCoord = goalCell->getCoord();

    cout << "Goal Pos = (" << pos[0] << " , " << pos[1] << " , " << pos[2] << ")" << endl;
    cout << "Goal Coord = (" << goalCoord[0] << " , " << goalCoord[1] << " , " << goalCoord[2] << ")" << endl;


    if( (startCoord[0] == goalCoord[0]) &&
        (startCoord[1] == goalCoord[1]) &&
        (startCoord[2] == goalCoord[2]) )
    {
        cout << " no planning as cells are identical" << endl;
        return false;
    }

    HriGridState* goal = new HriGridState(
            goalCell,
            _3DGrid);


    Trajectory* traj = new Trajectory(new Robot(XYZ_ROBOT));

    traj->replaceP3dTraj();
    string str = "g3d_draw_allwin_active";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
    ENV.setBool(Env::drawTraj,true);
    cout << "solution : End Search" << endl;
}

vector< Vector3d > HRICS_Planner::solveAStar(HriGridState* start,HriGridState* goal)
{
    vector< Vector3d > solution;

    Trajectory* traj = new Trajectory(new Robot(XYZ_ROBOT));

    shared_ptr<Configuration> config = _Robot->getCurrentPos();

    /*
    * Change the way AStar
    * is computed to go down
    */
    if( start->getCell()->getCost() < goal->getCell()->getCost() )
    {
        AStar* search = new AStar(start);
        vector<State*> path = search->solve(goal);

        if(path.size() == 0 )
        {
            return solution;
        }

        for (int i=0;i<path.size();i++)
        {
            Vector3d cellCenter = dynamic_cast<HriGridState*>(path[i])->getCell()->getCenter();
            config  = shared_ptr<Configuration>(new Configuration(new Robot(XYZ_ROBOT)));

            config->getConfigStruct()[VIRTUAL_OBJECT+0] = cellCenter[0];
            config->getConfigStruct()[VIRTUAL_OBJECT+1] = cellCenter[1];
            config->getConfigStruct()[VIRTUAL_OBJECT+2] = cellCenter[2];
            //            config->getConfigStruct()[VIRTUAL_OBJECT+3] =   0;
            //            config->getConfigStruct()[VIRTUAL_OBJECT+4] =   0;
            //            config->getConfigStruct()[VIRTUAL_OBJECT+5] =   0;

            traj->push_back(config);
        }
    }
    else
    {
        AStar* search = new AStar(goal);
        vector<State*> path = search->solve(start);

        if(path.size() == 0 )
        {
            return solution;
        }

        for (int i=path.size()-1;i>=0;i--)
        {
            Vector3d cellCenter = dynamic_cast<HriGridState*>(path[i])->getCell()->getCenter();
            config  = shared_ptr<Configuration>(new Configuration(new Robot(XYZ_ROBOT)));

            config->getConfigStruct()[VIRTUAL_OBJECT+0] = cellCenter[0];
            config->getConfigStruct()[VIRTUAL_OBJECT+1] = cellCenter[1];
            config->getConfigStruct()[VIRTUAL_OBJECT+2] = cellCenter[2];
            //            config->getConfigStruct()[VIRTUAL_OBJECT+3] =   0;
            //            config->getConfigStruct()[VIRTUAL_OBJECT+4] =   0;
            //            config->getConfigStruct()[VIRTUAL_OBJECT+5] =   0;

            traj->push_back(config);
        }
    }

    return solution;
}

void HRICS_Planner::initDistance()
{
    _Distance = new HRICS_Distance(_Robot,_Humans);
    cout << "Robot " << _Robot->getName() << endl;
    cout << "Robot get struct " << _Robot->getRobotStruct() << endl;
    _Distance->parseHumans();
}
