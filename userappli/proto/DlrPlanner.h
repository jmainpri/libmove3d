#ifndef __DLRPLANNER_H__
#define __DLRPLANNER_H__
#include <iostream>
#include <vector>
#include <fstream>
#include "../userappli/proto/DlrObject.h"

class DlrPlanner {
public:
  //Constructors and destructors
  DlrPlanner();
  virtual ~DlrPlanner();
  //functions

//////////////  Move3d Function ///////////////

//////////////  Move3d Function ///////////////
  //setters and getters
  void setStartConfig(std::vector<double> config);
  void setApproachConfig(std::vector<double> config);
  void setGraspConfig(std::vector<double> config);
  void setFinalConfig(std::vector<double> config);
  void addStaticObject(std::string name);
  void addManipObject(std::string name, std::vector<double> rightFrame, std::vector<double> leftFrame);
  void addPositionToObject(std::string name, int id, std::vector<double> position);
protected:
  configPt vectorToConfigPt(std::vector<double> config);
private:
  configPt _startConfig;
  configPt _approachConfig;
  configPt _graspConfig;
  configPt _finalConfig;
  std::map<std::string, DlrObject*, std::less<std::string> > _staticObjects;
  std::map<std::string, DlrObject*, std::less<std::string> > _manipObjects;
  p3d_rob* _robot;
//static members
public:
};

#endif
