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
