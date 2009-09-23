#ifndef __DLROBJECT_H__
#define __DLROBJECT_H__
#include <iostream>
#include <vector>
#include "P3d-pkg.h"
#include "device.h"

#include "p3d_matrix.h"

class DlrObject {
public:
  //Constructors and destructors
  DlrObject(std::string name);
  DlrObject(std::string name, std::vector<double> rightAttachFrame, std::vector<double> leftAttachFrame);
  virtual ~DlrObject();
  //setters and getters
  void setRightAttachFrame(std::vector<double> attachFrame);
  double* getRightAttachFrame();
  void setLeftAttachFrame(std::vector<double> attachFrame);
  double* getLeftAttachFrame();
  void addPosition(std::vector<double>pos, int id);
  double* getPosition(int id);
  p3d_rob* getRobot();
  p3d_obj* getObject();
  int isAStaticOrMobileObject();
protected:
  void setObjectOrRobot(std::string name);
  double* convertFrame(std::vector<double> vec);
	void convertDlrGraspFrameToMove3d(double* array);
private:
  std::string _name;
  p3d_obj* _m3dObject;
  p3d_rob* _m3dRobot;
  std::vector<double*> _positions;
  double* _rightAttFrame;
  double* _leftAttFrame;
//static members
public:
  static void convertArrayToP3d_matrix4(double* array, p3d_matrix4 mat){
		for(int i = 0; i < 4; i++){
			for(int j = 0; j < 4; j++){
				if(array){
					mat[i][j] = array[i*4+j];
				}else{
					mat[i][j] = 0;
				}
			}
		}
  }
	static void convertDlrToMove3dFrame(double* array){
		p3d_matrix4 dlrMat, move3dMat, tmp;
		p3d_matrix4 transform = {{0, -1, 0, 0},{0, 0, 1, 0},{-1, 0, 0, 0},{0, 0, 0, 1}};
		convertArrayToP3d_matrix4(array, dlrMat);
		p3d_matMultXform(dlrMat, transform, tmp);
		p3d_matInvertXform(tmp, move3dMat);
		for(int i = 0; i < 4; i++){
			for(int j = 0; j < 4; j++){
				array[i*4+j] = move3dMat[i][j];
			}
		}
  }
};

#endif
