//
// C++ Implementation: robot
//
// Description:
//
//
// Author: Florian Pilardeau,B90,6349 <fpilarde@jolimont>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "../planningAPI.hpp"
#include "P3d-pkg.h"
#include "Planner-pkg.h"

using namespace std;
using namespace tr1;

//constructor and destructor
Robot::Robot(p3d_rob* R)
{
  _Robot = R;
  _nbCreatedGraph = 0;
  Graph* G = this->newGraph();
  _Robot->GRAPH = G->getGraphStruct();
  _Name = _Robot->name;
}

Robot::~Robot()
{
}

//Accessors
p3d_rob* Robot::getRobotStruct()
{
  return _Robot;
}

string Robot::getName()
{
  return _Name;
}

void Robot::setActivGraph(string S)
{
  for (uint i = 0; i < _Graph.size()-1; i = i + 1)
  {
    if (_Graph[i]->getName() == S)
    {
      activ_graph = i;
      _Robot->GRAPH = _Graph[i]->getGraphStruct();
      //XYZ_GRAPH = _Graph[i]->get_graph();
    }
  }
}

void Robot::setActivGraph(int i)
{
  activ_graph = i;
  _Robot->GRAPH = _Graph[i]->getGraphStruct();
  //XYZ_GRAPH = _Graph[i]->get_graph();
}

Graph* Robot::getActivGraph()
{
  if(activ_graph < 0 || activ_graph > nbGraph())
  {
    return NULL;
  }
  return _Graph[activ_graph];
}

int Robot::getNbCreatedGraph()
{
  return _nbCreatedGraph;
}

int Robot::getActivGraphNumber()
{
  return activ_graph;
}

Graph* Robot::getGraph(int i)
{
  if(i < 0 || i > nbGraph())
  {
    return NULL;
  }
  return _Graph[i];
}

Graph* Robot::getGraph(string S)
{
  for (uint i = 0; i < _Graph.size()-1; i = i + 1)
  {
    if (_Graph[i]->getName() == S)
    {
      return _Graph[i];
    }
  }
  return this->newGraph();
}

Graph* Robot::newGraph()
{
  _Robot->GRAPH = NULL;

  Graph* g = new Graph(this);

  this->insertGraph(g);
  _nbCreatedGraph++;
  setActivGraph(nbGraph()-1);
  return g;
}

int Robot::nbGraph()
{
  return _Graph.size();
}

void Robot::insertGraph(Graph* G)
{
  _Graph.push_back(G);
}

void Robot::removeGraph(int num)
{
  _Graph.erase(_Graph.begin()+num);
}

shared_ptr<Configuration> Robot::shoot(bool samplePassive)
{
  shared_ptr<Configuration> q = shared_ptr<Configuration>(new Configuration(this));
  p3d_shoot(_Robot,q->getConfigurationStruct(),samplePassive);
  return q;
}


shared_ptr<Configuration> Robot::shootDir(bool samplePassive)
{
  shared_ptr<Configuration> q = shared_ptr<Configuration>(new Configuration(this));
  p3d_RandDirShoot(_Robot,q->getConfigurationStruct(),samplePassive);
  return q;
}

int Robot::setAndUpdate(Configuration& q)
{
  p3d_set_robot_config(_Robot, q.getConfigurationStruct());
  return(p3d_update_this_robot_pos(_Robot));
}

shared_ptr<Configuration> Robot::getInitialPosition()
{
  return(shared_ptr<Configuration>(new Configuration(this, _Robot->ROBOT_POS)));
}

shared_ptr<Configuration> Robot::getGoTo()
{
  return(shared_ptr<Configuration>(new Configuration(this, _Robot->ROBOT_GOTO)));
}
