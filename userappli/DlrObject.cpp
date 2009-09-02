#include "../userappli/proto/DlrObject.h"

DlrObject::DlrObject(std::string name){
  _name.append(name);
  _m3dObject = NULL;
  _m3dRobot = NULL;
  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 4; j++){
      _leftAttFrame[i*4+j] = i == j? 1 : 0;
      _rightAttFrame[i*4+j] = i == j? 1 : 0;
    }
  }
  setObjectOrRobot(name);
}

DlrObject::~DlrObject(){
  free(_rightAttFrame);
  free(_leftAttFrame);
  for(unsigned int i = 0; i < _positions.size(); i++){
    double* mat = _positions.at(i);
    free(mat);
  }
}

void DlrObject::setRightAttachFrame(std::vector<double> attachFrame){
  _rightAttFrame = convertFrame(attachFrame);
}
double* DlrObject::getRightAttachFrame(){
  return _rightAttFrame;
}
void DlrObject::setLeftAttachFrame(std::vector<double> attachFrame){
  _rightAttFrame = convertFrame(attachFrame);
}
double* DlrObject::getLeftAttachFrame(){
  return _leftAttFrame;
}
void DlrObject::addPosition(std::vector<double> pos, int id){
  _positions.push_back(convertFrame(pos));
}
double* DlrObject::getPosition(int id){
  return _positions[id];
}
p3d_rob* DlrObject::p3d_getRobot(){
  return _m3dRobot;
}
p3d_obj* DlrObject::p3d_getObject(){
  return _m3dObject;
}
int DlrObject::isAStaticOrMobileObject(){
  if(_m3dObject->type != P3D_BODY){
    return 1; //mobile object
  }
  return 0; //static object
}
void DlrObject::setObjectOrRobot(std::string name){
  for(int i = 0; i < XYZ_ENV->no; i++){
    std::string objectName(XYZ_ENV->o[i]->name);
    if(!objectName.compare(name)){
      _m3dObject = XYZ_ENV->o[i];
      if(_m3dObject->type == P3D_BODY){
        _m3dRobot = _m3dObject->jnt->rob;
      }
    }
  }
}

double* DlrObject::convertFrame(std::vector<double> vec){
  double* frame = (double*)malloc(16*sizeof(double));
  for(unsigned int i = 0; i < vec.size(); i++){
    frame[i] = vec[i];
  }
  if(vec.size() == 12){
    frame[12] = frame[13] = frame[14] = 0;
    frame[15] = 1;
  }
  return frame;
}
