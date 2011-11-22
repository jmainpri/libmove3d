//
//  ManipulationData.hpp
//  libmove3d
//
//  Created by Jim Mainprice on 22/11/11.
//  Copyright 2011 LAAS/CNRS. All rights reserved.

#ifndef MANIPULATION_DATA_HPP
#define MANIPULATION_DATA_HPP

#ifdef GRASP_PLANNING
#include "GraspPlanning-pkg.h"
#endif
#include "P3d-pkg.h"
#include <vector>


//! @ingroup manipulation
//! this class holds the manipulation data hence every intermediary 
//! configuration that is used to plan a pick and place problem
class ManipulationData 
{
public:
  
  ManipulationData(p3d_rob* robot) 
  {
    _robot = robot;
#ifdef GRASP_PLANNING
    _grasp = NULL;
#endif
    _graspConfig = NULL;
    _graspConfigCost = P3D_HUGE;
    _openConfig = NULL;
    _approachFreeConfig = NULL;
    _approachGraspConfig = NULL;
    _graspAttachFrame[0][0] = _graspAttachFrame[0][1] = _graspAttachFrame[0][2] = _graspAttachFrame[0][3] =  0;
  };
#ifdef GRASP_PLANNING   
  ManipulationData(p3d_rob* robot, gpGrasp* grasp, configPt graspConfig, configPt openConfig, configPt approachFreeConfig, configPt approachGraspConfig, p3d_matrix4 graspAttachFrame){
    _robot = robot;
    _grasp = grasp;
    _graspConfig = p3d_copy_config(robot, graspConfig);
    _graspConfigCost = P3D_HUGE;
    _openConfig = p3d_copy_config(robot, openConfig);
    _approachFreeConfig = p3d_copy_config(robot, approachFreeConfig);
    _approachGraspConfig = p3d_copy_config(robot, approachGraspConfig);
    p3d_mat4Copy(graspAttachFrame ,_graspAttachFrame);
  };
#endif
  virtual ~ManipulationData(){
    if(_graspConfig){
      p3d_destroy_config(_robot, _graspConfig);
      _graspConfig = NULL;
    }
    if(_openConfig){
      p3d_destroy_config(_robot, _openConfig);
      _openConfig = NULL;
    }
    if(_approachFreeConfig){
      p3d_destroy_config(_robot, _approachFreeConfig);
      _approachFreeConfig = NULL;
    }
    if(_approachGraspConfig){
      p3d_destroy_config(_robot, _approachGraspConfig);
      _approachGraspConfig = NULL;
    }
#ifdef GRASP_PLANNING
    if(_grasp){
      delete(_grasp);
      _grasp = NULL;
    }
#endif
    
  }
  //Reset
  void clear(){
    if(_graspConfig){
      p3d_destroy_config(_robot, _graspConfig);
      _graspConfig = NULL;
    }
    if(_openConfig){
      p3d_destroy_config(_robot, _openConfig);
      _openConfig = NULL;
    }
    if(_approachFreeConfig){
      p3d_destroy_config(_robot, _approachFreeConfig);
      _approachFreeConfig = NULL;
    }
    if(_approachGraspConfig){
      p3d_destroy_config(_robot, _approachGraspConfig);
      _approachGraspConfig = NULL;
    }
#ifdef GRASP_PLANNING
    if(_grasp){
      delete(_grasp);
      _grasp = NULL;
    }
#endif
  }
  //Getters
  inline p3d_rob* getRobot() const{
    return _robot;
  }
#ifdef GRASP_PLANNING
  inline gpGrasp* getGrasp() const{
    return _grasp;
  }
#endif
  inline configPt getGraspConfig() const{
    return _graspConfig;
  }
  inline configPt getOpenConfig() const{
    return _openConfig;
  }
  inline configPt getApproachFreeConfig() const{
    return _approachFreeConfig;
  }
  inline configPt getApproachGraspConfig() const{
    return _approachGraspConfig;
  }
  std::vector<configPt> getAllConfigs() const{
    std::vector<configPt> vect(4);
    vect[0] =  _graspConfig;
    vect[1] =  _openConfig;
    vect[2] =  _approachFreeConfig;
    vect[3] =  _approachGraspConfig;
    return vect;
  }
  inline void getAttachFrame(p3d_matrix4 graspAttachFrame) const{
    p3d_mat4Copy((p3d_matrix_type(*)[4])_graspAttachFrame, graspAttachFrame);
  }
  inline double getGraspConfigCost() const{
    return _graspConfigCost;
  }
  //Setters
#ifdef GRASP_PLANNING
  inline void setGrasp(gpGrasp* grasp){
    if(_grasp == grasp){
      return;
    }
    if (!grasp) {
      if(_grasp){
        delete(_grasp);
      }
      _grasp = NULL;
      return;
    }
    if(_grasp){
      delete(_grasp);
      _grasp = NULL;
    }
    _grasp = new gpGrasp(*grasp);
  }
#endif
  inline void setGraspConfig(configPt graspConfig){
    if(graspConfig){
      if(_graspConfig){
        p3d_copy_config_into(_robot, graspConfig, &_graspConfig);
      }else{
        _graspConfig = p3d_copy_config(_robot, graspConfig);
      }
    }
  }
  inline void setOpenConfig(configPt openConfig){
    if(openConfig){
      if(_openConfig){
        p3d_copy_config_into(_robot, openConfig, &_openConfig);
      }else{
        _openConfig = p3d_copy_config(_robot, openConfig);
      }
    }
  }
  inline void setApproachFreeConfig(configPt approachFreeConfig){
    if(approachFreeConfig){
      if(_approachFreeConfig){
        p3d_copy_config_into(_robot, approachFreeConfig, &_approachFreeConfig);
      }else{
        _approachFreeConfig = p3d_copy_config(_robot, approachFreeConfig);
      }
    }
  }
  inline void setApproachGraspConfig(configPt approachGraspConfig){
    if(approachGraspConfig){
      if(_approachGraspConfig){
        p3d_copy_config_into(_robot, approachGraspConfig, &_approachGraspConfig);
      }else{
        _approachGraspConfig = p3d_copy_config(_robot, approachGraspConfig);
      }
    }
  }
  inline void setAttachFrame(p3d_matrix4 graspAttachFrame){
    p3d_mat4Copy(graspAttachFrame, _graspAttachFrame);
  }
  inline void setGraspConfigCost(double graspConfigCost){
    _graspConfigCost = graspConfigCost;
  }
  ManipulationData& operator = (const ManipulationData& data){
    _robot = data.getRobot();
#ifdef GRASP_PLANNING
    setGrasp(data.getGrasp());
#endif
    setGraspConfig(data.getGraspConfig());
    setOpenConfig(data.getOpenConfig());
    setApproachFreeConfig(data.getApproachFreeConfig());
    setApproachGraspConfig(data.getApproachGraspConfig());
    data.getAttachFrame(_graspAttachFrame);
    setGraspConfigCost(data.getGraspConfigCost());
    return *this;
  }
private:
  p3d_rob* _robot;
#ifdef GRASP_PLANNING
  gpGrasp* _grasp;
#endif
  configPt _graspConfig;
  configPt _openConfig;
  configPt _approachFreeConfig;
  configPt _approachGraspConfig;
  double _graspConfigCost;
  p3d_matrix4 _graspAttachFrame;
};

#endif