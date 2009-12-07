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

using namespace std;
using namespace tr1;

//constructor and destructor
//Robot::Robot(p3d_rob* R,Graph* G)
//{
//	_Robot = R;
//	_nbCreatedGraph = 0;
//
//	if(!G)
//	{
//		G = this->newGraph();
//	}
//	else
//	{
//		this->insertGraph(G);
//		setActivGraph(nbGraph() - 1);
//	}
//
//	_Robot->GRAPH = G->getGraphStruct();
//	_Name = _Robot->name;
//}

Robot::Robot(p3d_rob* R)
{
        _Robot = R;
        string name(R->name);
        _Name = name;
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

shared_ptr<Configuration> Robot::shoot(bool samplePassive)
{
	shared_ptr<Configuration> q = shared_ptr<Configuration> (new Configuration(
			this));
	p3d_shoot(_Robot, q->getConfigStruct(), samplePassive);
	return q;
}

shared_ptr<Configuration> Robot::shootDir(bool samplePassive)
{
	shared_ptr<Configuration> q = shared_ptr<Configuration> (new Configuration(
			this));

//	p3d_RandDirShoot(_Robot, q->getConfigStruct(), samplePassive);
	p3d_RandNShpereDirShoot(_Robot, q->getConfigStruct(), samplePassive);

	return q;
}

int Robot::setAndUpdate(Configuration& q)
{
	p3d_set_robot_config(_Robot, q.getConfigStruct());
	return (p3d_update_this_robot_pos(_Robot));
}

shared_ptr<Configuration> Robot::getInitialPosition()
{
	return (shared_ptr<Configuration> (new Configuration(this,
			_Robot->ROBOT_POS)));
}

shared_ptr<Configuration> Robot::getGoTo()
{
	return (shared_ptr<Configuration> (new Configuration(this,
			_Robot->ROBOT_GOTO)));
}

shared_ptr<Configuration> Robot::getCurrentPos()
{
	return (shared_ptr<Configuration> (new Configuration(this,
			p3d_get_robot_config(_Robot))));
}


vector<double> Robot::getJointPos(int id)
{
	vector<double> vect(3);

	p3d_jnt* jntPt= _Robot->joints[id];

	vect.at(0) = jntPt->abs_pos[0][3];
	vect.at(1) = jntPt->abs_pos[1][3];
	vect.at(2) = jntPt->abs_pos[2][3];

	return vect;
}

