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
#ifndef __DLRPLANNER_H__
#define __DLRPLANNER_H__
#include <iostream>
#include <vector>
#include <fstream>
#include "../lightPlanner/proto/DlrObject.h"
#include "../lightPlanner/proto/DlrPlan.h"

class DlrPlanner {
public:
  //Constructors and destructors
  DlrPlanner(char* fileName);
  virtual ~DlrPlanner();
  //functions

//////////////  Move3d Function ///////////////

//////////////  Move3d Function ///////////////
  //setters and getters
  void setStartConfig(std::vector<double> config);
  void setApproachConfig(std::vector<double> config);
  void setGraspConfig(std::vector<double> config);
  void setFinalConfig(std::vector<double> config);
	void setParseFile(std::string parseFile);
  void addObject(std::string name);
  void addObject(std::string name, std::vector<double> rightFrame, std::vector<double> leftFrame);
  void addPositionToObject(std::string name, int id, std::vector<double> position);
	void addObjectPositionToConfig(p3d_matrix4 objectPos, p3d_jnt* jnt, configPt config);
	void addPlan(DlrPlan::planType type);
	DlrPlan* getCurrrentPlan();
	DlrObject* getObject(std::string name);
	std::string getTrajFileName();
	int process();
protected:
	void saveTraj(p3d_traj* traj, DlrPlan* plan);
	bool isABaseLocalPath(p3d_localpath* lp);
  configPt vectorToConfigPt(std::vector<double> config);
private:
	std::string _trajFile;
	std::string _parseFile;
  configPt _startConfig;
  configPt _approachConfig;
  configPt _graspConfig;
  configPt _finalConfig;
  std::map<std::string, DlrObject*> _objects;
  p3d_rob* _robot;
	std::vector<DlrPlan*> _execStack;
//static members
public:
};

#endif
