/*
 * Copyright (c) 2001-2014 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 */
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
