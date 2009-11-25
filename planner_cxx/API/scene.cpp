//
// C++ Implementation: environnement
//
// Description: 
//
//
// Author: Florian Pilardeau,B90,6349 <fpilarde@jolimont>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "scene.h"

using namespace std;

Scene::Scene(string name)
{
  _Name = name;
  _activRobot = "";
}

Scene::Scene(string name, Robot* Robot)
{
  _Name = name;
  setActivRobot(Robot->getName());
}


Scene::~Scene()
{
}

string Scene::getName()
{
  return _Name;
}

void Scene::setActivRobot(string name)
{
  _activRobot = name;
}

Robot* Scene::getActivRobot()
{
  for(uint i = 0; i < _Robots.size(); i = i + 1)
  {
    if(_Robots[i]->getName() == _activRobot)
    {
      return _Robots[i];
    }
  }
  if (XYZ_ROBOT->name == _activRobot)
  {
    Robot* R = new Robot(XYZ_ROBOT);
    this->insertRobot(R);
    return R;
  }
  return NULL;
}

void Scene::insertRobot(Robot* R)
{
  _Robots.push_back(R);
}


