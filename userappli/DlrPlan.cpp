#include "../userappli/proto/DlrPlan.h"
#include "Collision-pkg.h"

DlrPlan::DlrPlan(planType type){
	_type = type;
	_object = NULL;
	for(int i = 0; i < 4; i++){
		for(int j = 0; j < 4; j++){
			_startPos[i][j] = i == j ? 1 : 0;
			_targetPos[i][j] = i == j ? 1 : 0;
		}
	}
	_execute = false;
}
DlrPlan::~DlrPlan(){}
//functions
void DlrPlan::setObstaclesAtRightPos(){
	for(std::map<DlrObject*, int>::iterator iter = _obstacles.begin(); iter != _obstacles.end(); iter++){
		DlrObject* object = iter->first;
		double * pos = object->getPosition(iter->second);
		p3d_matrix4 position;
		DlrObject::convertArrayToP3d_matrix4(pos, position);
		p3d_obj* m3dObj = object->getObject();
		if(m3dObj->type == P3D_BODY){
			setBodyJntAtRightPos(object->getRobot() ,m3dObj->jnt, position);
		}else{//is a static object
			for(int i = 0; i < 4; i++){
				for(int j = 0; j < 4; j++){
					m3dObj->opos[i][j] = position[i][j];
				}
			}
			p3d_col_stop();
			p3d_col_start(p3d_col_mode_kcd);
		}
	}
}
void DlrPlan::setBodyJntAtRightPos(p3d_rob* robot, p3d_jnt* jnt, p3d_matrix4 position){
	double euler[6] = {0,0,0,0,0,0};
	p3d_mat4ExtractPosReverseOrder(position, &euler[0], &euler[1], &euler[2], &euler[3], &euler[4], &euler[5]);
	for(int i = 0; i < jnt->dof_equiv_nbr; i++ ){
		p3d_jnt_set_dof(jnt, i, euler[i]);
	}
	//update the robot Pos
	p3d_update_this_robot_pos(robot);
}
//setters and getters
void DlrPlan::setType(planType type){
	_type = type;
}
void DlrPlan::setObject(DlrObject* object){
	_object = object;
}
void DlrPlan::setStartPos(DlrObject* object, int posId){
	double * pos = object->getPosition(posId);
	DlrObject::convertArrayToP3d_matrix4(pos, _startPos);
}
void DlrPlan::setTargetPos(DlrObject* object, int posId){
	double * pos = object->getPosition(posId);
  DlrObject::convertArrayToP3d_matrix4(pos, _targetPos);
}
void DlrPlan::addObstacle(DlrObject* object, int posId){
	_obstacles.insert(std::make_pair(object, posId));
}
void DlrPlan::setExecute(bool execute){
	_execute = execute;
}
DlrPlan::planType DlrPlan::getType(){
	return _type;
}
DlrObject* DlrPlan::getObject(){
	return _object;
}
void DlrPlan::getStartPos(p3d_matrix4 startPos){
	for(int i = 0; i < 4; i++){
		for(int j = 0; j < 4; j++){
			startPos[i][j] = _startPos[i][j];
		}
	}
}
void DlrPlan::getTargetPos(p3d_matrix4 targetPos){
	for(int i = 0; i < 4; i++){
		for(int j = 0; j < 4; j++){
			targetPos[i][j] = _targetPos[i][j];
		}
	}
}
bool DlrPlan::getExecute(){
	return _execute;
}