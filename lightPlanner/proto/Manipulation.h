#ifndef __MANIPULATION_H__
#define __MANIPULATION_H__
#include <vector>
#include <map>
#include "GraspPlanning-pkg.h"

class ManipulationData{
  public:
    ManipulationData(p3d_rob* robot){
      _robot = robot;
      _graspConfig = p3d_alloc_config(robot);
      _openConfig = p3d_alloc_config(robot);;
      _approachConfig = p3d_alloc_config(robot);;
      p3d_mat4Copy(p3d_mat4IDENTITY ,_graspAttachFrame);
    };
    ManipulationData(p3d_rob* robot, gpGrasp grasp, configPt graspConfig, configPt openConfig, configPt approachConfig, p3d_matrix4 graspAttachFrame){
      _robot = robot;
      _grasp = grasp;
      _graspConfig = graspConfig;
      _openConfig = openConfig;
      _approachConfig = approachConfig;
      p3d_mat4Copy(graspAttachFrame ,_graspAttachFrame);
    };
    virtual ~ManipulationData(){
      if(_graspConfig){
        p3d_destroy_config(_robot, _graspConfig);
      }
      if(_openConfig){
        p3d_destroy_config(_robot, _openConfig);
      }
      if(_approachConfig){
        p3d_destroy_config(_robot, _approachConfig);
      }
    }
    //Getters
    gpGrasp getGrasp(){
      return _grasp;
    }
    configPt getGraspConfig(){
      return _graspConfig;
    }
    configPt getOpenConfig(){
      return _openConfig;
    }
    configPt getApproachConfig(){
      return _approachConfig;
    }
    void getAttachFrame(p3d_matrix4 graspAttachFrame){
      p3d_mat4Copy(_graspAttachFrame, graspAttachFrame);
    }
    //Setters
    void setGrasp(gpGrasp grasp){
      _grasp = grasp;
    }
    void setGraspConfig(configPt graspConfig){
      _graspConfig = graspConfig;
    }
    void setOpenConfig(configPt openConfig){
      _openConfig = openConfig;
    }
    void setApproachConfig(configPt approachConfig){
      _approachConfig = approachConfig;
    }
    void setAttachFrame(p3d_matrix4 graspAttachFrame){
      p3d_mat4Copy(graspAttachFrame, _graspAttachFrame);
    }
  private:
    p3d_rob* _robot;
    gpGrasp _grasp;
    configPt _graspConfig;
    configPt _openConfig;
    configPt _approachConfig;
    p3d_matrix4 _graspAttachFrame;
};

class Manipulation{
  public :
    Manipulation(p3d_rob *robot);
    virtual ~Manipulation();
    int findAllArmsGraspsConfigs(p3d_matrix4 objectStartPos, p3d_matrix4 objectEndPos);
    int findAllSpecificArmGraspsConfigs(int armId, p3d_matrix4 objectPos);
    int getCollisionFreeGraspAndApproach(p3d_matrix4 objectPos, gpHand_properties handProp, gpGrasp grasp, int whichArm, p3d_matrix4 tAtt, configPt* graspConfig, configPt* approachConfig);
  protected:
    void getHandGraspsMinMaxCosts(int armId, double* minCost, double* maxCost);
  private :
    std::map<double, ManipulationData*, std::less<double> > _handsGraspsConfig;
    p3d_rob * _robot;
    double _armMinMaxCost[2][2];
    static const int _maxColGrasps = 10;
};

#endif