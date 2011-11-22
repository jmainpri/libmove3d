//
//  ArmManipulationData.hpp
//  libmove3d
//
//  Created by Jim Mainprice on 22/11/11.
//  Copyright 2011 LAAS/CNRS. All rights reserved.
//
#ifndef __MANIPULATION_ARM_DATA_HPP__
#define __MANIPULATION_ARM_DATA_HPP__

#include "ManipulationStruct.hpp"
#ifdef GRASP_PLANNING
#include "GraspPlanning-pkg.h"
#endif
#include "P3d-pkg.h"
#include <vector>

//! @ingroup manipulation
/** This Class contains all necessessary data to specify a arm to manipulate with it*/
class ArmManipulationData 
{		
public:
  ArmManipulationData(int id = 0, p3d_cntrt* ccCntrt = NULL, p3d_cntrt* fkCntrt = NULL, p3d_jnt *manipulationJnt = NULL, int cartesianGroup = -1, int cartesianSmGroup = -1, int handGroup = -1, int handSmGroup = -1){
    _id = id;
    _ccCntrt = ccCntrt;
    _fkCntrt = fkCntrt;
    _manipulationJnt = manipulationJnt;
    if(_ccCntrt == NULL){
      _tAtt[0][0] = _tAtt[0][1] = _tAtt[0][2] = _tAtt[0][3] =  0;
    }else{
      p3d_mat4Copy(_ccCntrt->Tatt, _tAtt);
    }
    _carriedObject = NULL;
    _placement = NULL;
    _human = NULL;
    _cartesian = false;
  };
  
  virtual ~ArmManipulationData(){};
  
  /*************/
  /* Functions */
  /*************/
  /** Fix hand dof, Disable autocollisions, and set the arm to rest configuration*/
  void fixHand(p3d_rob* robot, bool rest);
  /** Unfix hand dof, Enable autocollisions*/
  void unFixHand(p3d_rob* robot);
  /** Deactivate the closed and direct kinematic constraints of an arm*/
  void deactivateManipulationCntrts(p3d_rob* robot);
  
  /***********/
  /* Setters */
  /***********/
  inline void setId(int id) {
    _id = id;
  }
  inline void setCcCntrt(p3d_cntrt* ccCntrt){
    _ccCntrt = ccCntrt;
  };
  inline void setCcCntrt(p3d_rob* robot, int id){
    _ccCntrt = robot->cntrt_manager->cntrts[id];
  };
  inline void setFkCntrt(p3d_cntrt* fkCntrt){
    _fkCntrt = fkCntrt;
  };
  inline void setFkCntrt(p3d_rob* robot, int id){
    _fkCntrt = robot->cntrt_manager->cntrts[id];
  };
  inline void setManipulationJnt(p3d_jnt* manipulationJnt){
    _manipulationJnt = manipulationJnt;
  };
  inline void setManipulationJnt(p3d_rob* robot, int manipulationJnt){
    _manipulationJnt = robot->joints[manipulationJnt];
  };
  inline void setAttachFrame(p3d_matrix4 tAtt){
    p3d_mat4Copy(tAtt, _tAtt);
  }
  inline void setCarriedObject(p3d_rob* carriedObject){
    _carriedObject = carriedObject;
  };
  inline void setCarriedObject(const char* robotName){
    if(robotName){
      _carriedObject = p3d_get_robot_by_name(robotName);
    }else{
      _carriedObject = NULL;
    }
  };
  inline void setPlacement(p3d_rob* placement){
    _placement = placement;
  };
  inline void setHuman(p3d_rob* human){
    _human = human;
  };
#ifdef GRASP_PLANNING
  inline void setHandProperties(int handId){
    _handProp.initialize((gpHand_type)handId);
  };
#endif
  inline void setCartesian(bool cartesian){
    _cartesian = cartesian;
  };
  
  /***********/
  /* Getters */
  /***********/
  inline int getId() {
    return _id;
  }
  inline p3d_cntrt* getCcCntrt(void) const{
    return _ccCntrt;
  };
  inline p3d_cntrt* getFkCntrt(void) const{
    return _fkCntrt;
  };
  inline p3d_jnt* getManipulationJnt(void) const{
    return _manipulationJnt;
  };
  inline void getAttachFrame(p3d_matrix4 tAtt) const{
    p3d_mat4Copy((p3d_matrix_type(*)[4])_tAtt, tAtt);
  }
  inline p3d_rob* getCarriedObject(void) const{
    return _carriedObject;
  };
  inline p3d_rob* getPlacement(void) const{
    return _placement;
  };
  inline p3d_rob* getHuman(void) const{
    return _human;
  };
#ifdef GRASP_PLANNING
  inline gpHand_properties getHandProperties() const{
    return _handProp;
  };
#endif
  inline bool getCartesian(void) const{
    return _cartesian;
  };
  
	private :
  //!arm ID
  int _id;
  
	/***************/
	/* Constraints */
	/***************/
	/** Arm associated Closed Chain Constraint*/
	p3d_cntrt * _ccCntrt;
	/** Arm corresopnding Forward kinematic Constraint*/
	p3d_cntrt * _fkCntrt;
  /** Default Attach Matrix*/
  p3d_matrix4 _tAtt;
	/** Freeflyer */
	p3d_jnt * _manipulationJnt;
	/** < choose to plan the arm motion in cartesian space (for the end effector) or joint space  */
	bool _cartesian;
	
	/************************/
	/* Manipulation Objects */
	/************************/
	/** The object carried by this arm*/
	p3d_rob* _carriedObject;
	/** The object where to place the object carried by this arm*/
	p3d_rob* _placement;
	/** The human to interract with with this arm*/
	p3d_rob* _human;
	
#ifdef GRASP_PLANNING
	/************/
	/* Grasping */
	/************/
	/** Arm end effector property */
	gpHand_properties _handProp;
#endif
	
};

#endif