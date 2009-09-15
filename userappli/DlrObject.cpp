#include "../userappli/proto/DlrObject.h"

DlrObject::DlrObject(std::string name){
  _name.append(name);
  _m3dObject = NULL;
  _m3dRobot = NULL;
	_leftAttFrame = (double*) malloc(16*sizeof(double));
	_rightAttFrame = (double*) malloc(16*sizeof(double));
  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 4; j++){
      _leftAttFrame[i*4+j] = i == j? 1 : 0;
      _rightAttFrame[i*4+j] = i == j? 1 : 0;
    }
  }
  setObjectOrRobot(name);
}

DlrObject::DlrObject(std::string name, std::vector<double> rightAttachFrame, std::vector<double> leftAttachFrame){
  _name.append(name);
  _m3dObject = NULL;
  _m3dRobot = NULL;
	_leftAttFrame = (double*) malloc(16*sizeof(double));
	_rightAttFrame = (double*) malloc(16*sizeof(double));
  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 4; j++){
      _leftAttFrame[i*4+j] = i == j? 1 : 0;
      _rightAttFrame[i*4+j] = i == j? 1 : 0;
    }
  }
  setObjectOrRobot(name);
  setRightAttachFrame(rightAttachFrame);
  setLeftAttachFrame(leftAttachFrame);
}

DlrObject::~DlrObject(){
  free(_rightAttFrame);
  free(_leftAttFrame);
  for(unsigned int i = 0; i < _positions.size(); i++){
    double* mat = _positions.at(i);
    free(mat);
  }
	_positions.clear();
}

void DlrObject::setRightAttachFrame(std::vector<double> attachFrame){
  _rightAttFrame = convertFrame(attachFrame);
	convertDlrGraspFrameToMove3d(_rightAttFrame);
}
double* DlrObject::getRightAttachFrame(){
  return _rightAttFrame;
}
void DlrObject::setLeftAttachFrame(std::vector<double> attachFrame){
  _leftAttFrame = convertFrame(attachFrame);
	convertDlrGraspFrameToMove3d(_leftAttFrame);
}
double* DlrObject::getLeftAttachFrame(){
  return _leftAttFrame;
}
void DlrObject::addPosition(std::vector<double> pos, int id){
	double* array = convertFrame(pos);
  _positions.push_back(array);
}
double* DlrObject::getPosition(int id){
  return _positions[id];
}
p3d_rob* DlrObject::getRobot(){
  return _m3dRobot;
}
p3d_obj* DlrObject::getObject(){
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
    }
  }
	if(_m3dObject == NULL){//search in the bodys
		for(int i = 0; i < XYZ_ENV->nr; i++){
			p3d_rob* robot = XYZ_ENV->robot[i];
			for(int j = 0; j < robot->no; j++){
				std::string objectName(robot->o[j]->name);
				if(!objectName.compare(name)){
					_m3dObject = robot->o[j];
					_m3dRobot = robot;
				}
			}
		}
	}
}

double* DlrObject::convertFrame(std::vector<double> vec){
	if(vec.size() > 1){
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
	return NULL;
}

void DlrObject::convertDlrGraspFrameToMove3d(double* array){
	if(array){
		p3d_matrix4 dlrMat, move3dMat, tmp;
		p3d_matrix4 convert = {{-1, 0, 0, 0},{0, 0, 1, 0.118},{0, 1, 0, 0},{0, 0, 0, 1}};
		convertArrayToP3d_matrix4(array, dlrMat);
		p3d_matInvertXform(convert, tmp);
		p3d_matMultXform(dlrMat, tmp, move3dMat);
		for(int i = 0; i < 4; i++){
			for(int j = 0; j < 4; j++){
				array[i*4+j] = move3dMat[i][j];
			}
		}
	}
}
