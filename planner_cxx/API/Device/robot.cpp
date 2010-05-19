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
#include "../../API/Grids/gridsAPI.hpp"

#include "P3d-pkg.h"
#include "Planner-pkg.h"
#ifdef LIGHT_PLANNER
#include "../../lightPlanner/proto/lightPlannerApi.h"
#endif

using namespace std;
using namespace tr1;

// import most common Eigen types 
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

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

//Robot::Robot(p3d_rob* R)
//{
//    _Robot = R;
//    string name(R->name);
//    _Name = name;
//}

Robot::Robot(p3d_rob* robotPt, bool copy )
{
	_copy = copy;
	
	if (_copy) {
		_Robot = copyRobStructure(robotPt);
	}
	else {
		_Robot = robotPt;
	}


    string name(robotPt->name);
    _Name = name;
	
	m_Joints.clear();
	
	for (int i=0; i<_Robot->njoints; i++) 
	{
		m_Joints.push_back( new Joint( _Robot->joints[i] , _copy ) );
	}

}

Robot::~Robot()
{
	if(_copy)
	{
		deleteRobStructure(_Robot);
	}
}

string Robot::getName()
{
    return _Name;
}

//Accessors
p3d_rob* Robot::getRobotStruct()
{
    return _Robot;
}

/**
 * Gets traj
 * @return pointer to structure p3d_traj
 */
p3d_traj* Robot::getTrajStruct()
{
	return _Robot->tcur;
}


unsigned int Robot::getNumberOfJoints()
{
    return m_Joints.size();
}

Joint* Robot::getJoint(unsigned int i)
{
	return m_Joints[i];
}

shared_ptr<Configuration> Robot::shoot(bool samplePassive)
{
    shared_ptr<Configuration> q(new Configuration(this));
#ifdef LIGHT_PLANNER
    if(ENV.getBool(Env::FKShoot))
    {
        deactivateCcCntrts(_Robot,-1);
        p3d_shoot(_Robot, q->getConfigStruct(), false);
        this->setAndUpdate(*q);
        q = this->getCurrentPos();
		
        activateCcCntrts(_Robot,-1,true);

		if (ENV.getBool(Env::drawPoints)) {
		
			int VirtualObjDof = this->getObjectDof();
			Vector3d randomPoint;
			randomPoint[0] = (*q)[VirtualObjDof+0];
			randomPoint[1] = (*q)[VirtualObjDof+1];
			randomPoint[2] = (*q)[VirtualObjDof+2];
			
			PointsToDraw->push_back(randomPoint);
			
			g3d_draw_allwin_active();
		}

        return q;
    }
    else
    {
        p3d_shoot(_Robot, q->getConfigStruct(), samplePassive);
        return q;
    }
#else
    p3d_shoot(_Robot, q->getConfigStruct(), samplePassive);
    return q;
#endif
}

shared_ptr<Configuration> Robot::shootDir(bool samplePassive)
{
    shared_ptr<Configuration> q(new Configuration(this));
    //	p3d_RandDirShoot(_Robot, q->getConfigStruct(), samplePassive);
    p3d_RandNShpereDirShoot(_Robot, q->getConfigStruct(), samplePassive);
    return q;
}

shared_ptr<Configuration> Robot::shootFreeFlyer(double* box)
{
//    cout << "box  = ( " << box[0] << " , " ;
//    cout << box[1] << " , " ;
//    cout << box[2] << " , " ;
//    cout << box[3] << " , " ;
//    cout << box[4] << " , " ;
//    cout << box[5] << " )" << endl;
    shared_ptr<Configuration> q(new Configuration(this));
    p3d_FreeFlyerShoot(_Robot, q->getConfigStruct(), box);
    return q;
}

/**
 * Returns false if does not respect the 
 * constraints
 */
int Robot::setAndUpdate(Configuration& q)
{
    p3d_set_robot_config(_Robot, q.getConfigStruct());
    return p3d_update_this_robot_pos(_Robot);
}

bool Robot::setAndUpdateMultiSol(Configuration& q)
{
	int *ikSol = NULL;
	p3d_set_robot_config(_Robot, q.getConfigStruct());
	return p3d_update_this_robot_pos_multisol(_Robot, NULL, 0, ikSol);	
}

void Robot::setAndUpdateWithoutConstraints(Configuration& q)
{
    p3d_set_robot_config(_Robot, q.getConfigStruct());
    p3d_update_this_robot_pos_without_cntrt(_Robot);
}

shared_ptr<Configuration> Robot::getInitialPosition()
{
    return (shared_ptr<Configuration> (new Configuration(this,
                                                         _Robot->ROBOT_POS)));
}

void Robot::setInitialPosition(Configuration& conf)
{
	configPt q = _Robot->ROBOT_POS;
	
	p3d_copy_config_into(_Robot,
						 conf.getConfigStruct(),
						 &q);
}

shared_ptr<Configuration> Robot::getGoTo()
{
    return (shared_ptr<Configuration> (new Configuration(this,
                                                         _Robot->ROBOT_GOTO)));
}

void Robot::setGoTo(Configuration& conf)
{
	configPt q = _Robot->ROBOT_GOTO;
	
	p3d_copy_config_into(_Robot,
						 conf.getConfigStruct(),
						 &q);
}

shared_ptr<Configuration> Robot::getCurrentPos()
{
    return (shared_ptr<Configuration> (new Configuration(this,
                                                         p3d_get_robot_config(_Robot))));
}

shared_ptr<Configuration> Robot::getNewConfig()
{
    return (shared_ptr<Configuration> (new Configuration(this,
                                                         p3d_alloc_config(_Robot),true)));
}

Matrix4d Robot::getJointAbsPos(int id)
{
	Matrix4d absPos;
	
	//p3d_jnt* jntPt= _Robot->joints[id];
	
//	absPos[0][0] = jntPt->abs_pos[0][0];
//    absPos[1][1] = jntPt->abs_pos[1][0];
//    absPos[2][2] = jntPt->abs_pos[2][0];
//	absPos[3][2] = jntPt->abs_pos[3][0];
//	
//	absPos[0][0] = jntPt->abs_pos[0][1];
//    absPos[1][1] = jntPt->abs_pos[1][1];
//    absPos[2][2] = jntPt->abs_pos[2][1];
//	absPos[3][2] = jntPt->abs_pos[3][1];
//	
//	absPos[0][0] = jntPt->abs_pos[0][2];
//    absPos[1][1] = jntPt->abs_pos[1][2];
//    absPos[2][2] = jntPt->abs_pos[2][2];
//	absPos[3][2] = jntPt->abs_pos[3][2];
//	
//	absPos[0][0] = jntPt->abs_pos[0][4];
//    absPos[1][1] = jntPt->abs_pos[1][4];
//    absPos[2][2] = jntPt->abs_pos[2][4];
//	absPos[3][2] = jntPt->abs_pos[3][4];
	
	return absPos;
}

Vector3d Robot::getJointPos(int id)
{
    Vector3d vect;

    p3d_jnt* jntPt= _Robot->joints[id];

    vect[0] = jntPt->abs_pos[0][3];
    vect[1] = jntPt->abs_pos[1][3];
    vect[2] = jntPt->abs_pos[2][3];

//    cout << "vect = " << endl << vect << endl;

    return vect;
}


