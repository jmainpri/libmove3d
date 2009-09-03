#include "../userappli/proto/DlrPlanner.h"
#include <stdlib.h>

DlrPlanner::DlrPlanner(){
  _startConfig = NULL;
  _approachConfig = NULL;
  _graspConfig = NULL;
  _finalConfig = NULL;
  _robot = XYZ_ROBOT;
}

DlrPlanner::~DlrPlanner(){
  for(std::iterator i = _staticObjects.begin(); i != _staticObjects.end(); i++){
    
  }
}

void DlrPlanner::setStartConfig(std::vector<double> config){
  if(_startConfig != NULL){
    p3d_destroy_config(_robot, _startConfig);
  }
  _startConfig = vectorToConfigPt(config);
}
void DlrPlanner::setApproachConfig(std::vector<double> config){
  if(_approachConfig != NULL){
    p3d_destroy_config(_robot, _approachConfig);
  }
  _approachConfig = vectorToConfigPt(config);
}
void DlrPlanner::setGraspConfig(std::vector<double> config){
  if(_graspConfig != NULL){
    p3d_destroy_config(_robot, _graspConfig);
  }
  _graspConfig = vectorToConfigPt(config);
}
void DlrPlanner::setFinalConfig(std::vector<double> config){
  if(_finalConfig != NULL){
    p3d_destroy_config(_robot, _finalConfig);
  }
  _finalConfig = vectorToConfigPt(config);
}
void DlrPlanner::addStaticObject(std::string name){
  DlrObject* object = new DlrObject(name);
  _staticObjects.insert(std::pair<std::string, DlrObject*> (name, object));
}
void DlrPlanner::addManipObject(std::string name, std::vector<double> rightFrame, std::vector<double> leftFrame){
  DlrObject* object = new DlrObject(name, rightFrame, leftFrame);
  _staticObjects.insert(std::pair<std::string, DlrObject*> (name, object));
}
void DlrPlanner::addPositionToObject(std::string name, int id, std::vector<double> position){
  DlrObject* object = _staticObjects[name];
  if(object != NULL){
    object = _manipObjects[name];
  }
  object->addPosition(position, id);
}
configPt DlrPlanner::vectorToConfigPt(std::vector<double> config){
  configPt q = p3d_alloc_config(_robot);
  for(int i = 0; i < 6; i++){
    q[i] = 0;
  }
  for(int i = _robot->nb_dof - 1; i > _robot->nb_dof - 7; i--){
    q[i] = 0;
  }
  for(unsigned int i = 0; i < config.size(); i++){
    q[i + 6] = config[i];
  }
  return q;
}
