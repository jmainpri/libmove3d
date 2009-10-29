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
#include "environnement.hpp"

using namespace std;

Environnement::Environnement(string name)
{
  _Name = name;
  _activRobot = "";
}

Environnement::Environnement(string name, Robot* Robot)
{
  _Name = name;
  setActivRobot(Robot->getName());
}


Environnement::~Environnement()
{
}

string Environnement::getName()
{
  return _Name;
}

void Environnement::setActivRobot(string name)
{
  _activRobot = name;
}

Robot* Environnement::getActivRobot()
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
    Robot* R = new Robot(XYZ_ROBOT,new Graph(XYZ_GRAPH));
    this->insertRobot(R);
    return R;
  }
  return NULL;
}

void Environnement::insertRobot(Robot* R)
{
  _Robots.push_back(R);
}


