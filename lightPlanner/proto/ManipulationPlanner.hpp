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
#ifndef __MANIPULATIONPLANNER_HPP__
#define __MANIPULATIONPLANNER_HPP__

#if defined (LIGHT_PLANNER) && defined (GRASP_PLANNING)

#include "P3d-pkg.h"
#include "Localpath-pkg.h"

#include "ManipulationStruct.hpp"
#include "ManipulationUtils.hpp"
#include "ManipulationData.hpp"
#include "ManipulationConfigs.hpp"

#include <map>

//! @ingroup manipulation
class  ManipulationPlanner {
public:
  /* ******************************* */
  /* ******* (Con)Destructor ******* */
  /* ******************************* */
  ManipulationPlanner(p3d_rob * robot);
  virtual ~ManipulationPlanner();
  
  /* ******************************* */
  /* *********** Cleaning ********** */
  /* ******************************* */
  /** Clean the class */
  void clear();
  /** set variable value for default use */
  void setDefaultPlanner();
  /** set variable value for navigation use */
  void setNavigationPlanner();
  /** Delete the current roadmaps */
  int cleanRoadmap();
  /** Delete class robot trajectory list */
  int cleanTraj();
  
  /* ******************************* */
  /* ******* (Ge)Setters *********** */
  /* ******************************* */
  void setDebugMode(bool value);
#ifdef MULTILOCALPATH
  void setDebugSoftMotionMode(bool value);
  void setSmoothingSoftMotionMode(bool value);
  bool getSmoothingSoftMotionMode();

  /** Multilocalpath Id for the base */
  int getBaseMLP() { return _BaseMLP; } 
  /** Multilocalpath Id for the head */
  int getHeadMLP() { return _HeadMLP; }
  /** Multilocalpath Id for the upper body (arms + torso) with linear localplanner */
  int getUpBodyMLP() { return _UpBodyMLP; }
  /** Multilocalpath Id for the upper body (arms + torso) with softMotion localplanner */
  int getUpBodysmMLP() { return _UpBodySmMLP; }
#endif
  
  // Set and reset the planning and smoothing
  // Functions used by the Manipulation planner
  void setPlanningMethod(p3d_traj* (*funct)(p3d_rob* robot, configPt qs, configPt qg));
  void resetPlanningMethod();
  
  void setSmoothingMethod(void (*funct)(p3d_rob* robot, p3d_traj* traj, int nbSteps, double maxTime));
  void resetSmoothingMethod();
  
  void setReplanningMethod(p3d_traj* (*funct)(p3d_rob* robotPt, p3d_traj* traj, p3d_vector3 target, int deformationViaPoint));
  //void resetReplanningMethod();
  
  void setCleanningRoadmaps(bool clean);
  
  void setPlanningTime(double time);
  double getPlanningTime(void) const;
  
  void setOptimizeSteps(int nbSteps);
  int getOptimizeSteps(void) const;
  
  void setOptimizeTime(double time);
  double getOptimizeTime(void) const;
  
  void setSafetyDistanceValue(double value);
  double getSafetyDistanceValue(void) const;
  
  void setPlacementTry(int nbTry);
  int getPlacementTry(void);
  
  void setUseBaseMotion(bool useBase);
  bool getUseBaseMotion(void);
  
  void resetTimers();
  void printTimers();
  
  void stopPlanning();
  
  void setRobotPath(p3d_traj* path) { _robotPath = _robot->tcur; } 
  
  inline p3d_rob* robot()  const{return _robot;}
  
  inline configPt robotStart() const{if (_robot != NULL) {return _robot->ROBOT_POS;} else {return NULL;}}
  inline configPt robotGoto() const{if (_robot != NULL) {return _robot->ROBOT_GOTO;} else {return NULL;}}
	
  
  inline ManipulationData& getManipulationData() { return _manipData; }
  inline const ManipulationConfigs& getManipulationConfigs() const { return _manipConf; }
  
  /** Generate needed configurations from the given grasp and object position */
  MANIPULATION_TASK_MESSAGE computeManipulationData(int armId,p3d_rob* object, gpGrasp& grasp);
  
	/* ******************************* */
  /* ******* Planning Modes ******** */
  /* ******************************* */
  /** Update the config given as parameter to deal with cartesian mode */
  void checkConfigForCartesianMode(configPt q, p3d_rob* object);
  /** Set the given Arm to be planified as cartesian */
  void setArmCartesian(int armId, bool cartesian);
  /** Get if the Arm will be planned in cartesian or not */
  bool getArmCartesian(int armId) const;
  
  /* ******************************* */
  /* ******* Motion Planning ******* */
  /* ******************************* */
  /** Creates a trajectory cut in localpaths every step **/
  MANIPULATION_TASK_MESSAGE cutTrajInSmall ( p3d_traj* inputTraj, p3d_traj* outputTraj );
  /** concatenate a vector of p3d_traj */
  MANIPULATION_TASK_MESSAGE concatTrajectories (std::vector<p3d_traj*>& trajs, p3d_traj** concatTraj);
  /** Correct the given configuration to be inside of the joints bounds. and print Warning */
  void fitConfigurationToRobotBounds(configPt q);
  /** Set the parameters to compute an RRT */
  MANIPULATION_TASK_MESSAGE computeRRT(int smoothingSteps, double smootingTime, bool biDir);
  /** Set the parameters to compute a PRM the input is the maximal computation time*/
  /* TODO Add CXX PLanner computation*/
  MANIPULATION_TASK_MESSAGE armComputePRM(double ComputeTime);
  /** Compute a trajectory between the two given configurations. The constraints, the planning modes and the dof to plan have to be setted before */
  p3d_traj* computeTrajBetweenTwoConfigs(configPt qi, configPt qf, MANIPULATION_TASK_MESSAGE* status);
#ifdef MULTILOCALPATH
  /** Given a trajectory, compute the corrsponding softMotion path */
  int computeSoftMotion(p3d_traj* traj, MANPIPULATION_TRAJECTORY_CONF_STR &confs, SM_TRAJ &smTraj);
#endif
  /** Move the arm from a free configuration to a grasping configuration of the object placed on a support */
  MANIPULATION_TASK_MESSAGE armToFreePoint(int armId, configPt qStart, std::vector<double> &objGoto, p3d_rob* object, std::vector <p3d_traj*> &trajs);
  MANIPULATION_TASK_MESSAGE armExtract(int armId, configPt qStart, p3d_rob* object, std::vector <p3d_traj*> &trajs);
  MANIPULATION_TASK_MESSAGE armToFree(int armId, configPt qStart, configPt qGoal, bool useSafetyDistance, p3d_rob* object, std::vector <p3d_traj*> &trajs);
  
  /** Move the arm from a free configuration to a grasping configuration of the object placed on a support */
  MANIPULATION_TASK_MESSAGE armPickGoto(int armId, configPt qStart, p3d_rob* object, gpGrasp& grasp, std::vector <p3d_traj*> &trajs);
  MANIPULATION_TASK_MESSAGE armPickGoto(int armId, configPt qStart, p3d_rob* object, configPt graspConfig, configPt openConfig, configPt approachFreeConfig, std::vector <p3d_traj*> &trajs);
  
  /** Move the arm from a grasping configuration (of the object placed on a support) to a free configuration */
  MANIPULATION_TASK_MESSAGE armPickTakeToFreePointCheckEscape(int armId, configPt qStart, std::vector<double> &objGoto , p3d_rob* object, p3d_rob* support, std::vector <p3d_traj*> &trajs);
  MANIPULATION_TASK_MESSAGE armPickTakeToFreePoint(int armId, configPt qStart, std::vector<double> &objGoto, p3d_rob* object, p3d_rob* support, std::vector <p3d_traj*> &trajs);
  MANIPULATION_TASK_MESSAGE armPickTakeToFree(int armId, configPt qStart, configPt qGoal, p3d_rob* object, p3d_rob* support, std::vector <p3d_traj*> &trajs);
  MANIPULATION_TASK_MESSAGE armPickTakeToFree(int armId, configPt qStart, configPt qGoal, p3d_rob* object, p3d_rob* support, configPt approachGraspConfig, gpGrasp &grasp, std::vector <p3d_traj*> &trajs);
  
  /** Move the arm from a grasping configuration to a configuration with the same grasp but a different object placement */
  MANIPULATION_TASK_MESSAGE armTakeToPlace(int armId, configPt qStart, p3d_rob* object, p3d_rob* support, p3d_rob* placement, std::vector <p3d_traj*> &trajs);
  MANIPULATION_TASK_MESSAGE armTakeToPlace(int armId, configPt qStart, p3d_rob* object, p3d_rob* support, std::vector<double> &objGoto, p3d_rob* placement, std::vector <p3d_traj*> &trajs);
  MANIPULATION_TASK_MESSAGE armTakeToPlace(int armId, configPt qStart, configPt approachGraspConfig, configPt approachGraspConfigPlacement, configPt qGoal, p3d_rob* object,  p3d_rob* support, p3d_rob* placement, std::vector <p3d_traj*> &trajs);
  
  /**  Move the arm from a free configuration to a placement configuration */
  MANIPULATION_TASK_MESSAGE armPlaceFromFree(int armId, configPt qStart, p3d_rob* object, p3d_rob* placement, std::vector <p3d_traj*> &trajs);
  MANIPULATION_TASK_MESSAGE armPlaceFromFree(int armId, configPt qStart, p3d_rob* object, std::vector<double> &objGoto, p3d_rob* placement, std::vector <p3d_traj*> &trajs);
  MANIPULATION_TASK_MESSAGE armPlaceFromFree(int armId, configPt qStart, p3d_rob* object, p3d_rob* placement, configPt approachGraspConfig, configPt depositConfig, std::vector <p3d_traj*> &trajs);
  
  /** Move the arm to escape from a placed object */
  MANIPULATION_TASK_MESSAGE armEscapeObject(int armId, configPt qStart, p3d_rob* object, std::vector <p3d_traj*> &trajs);
  MANIPULATION_TASK_MESSAGE armEscapeObject(int armId, configPt qStart, configPt openGraspConfig, configPt approachFreeConfig, p3d_rob* object, std::vector <p3d_traj*> &trajs);
  /* ******************************* */
  /* ******** Task Planning ******** */
  /* ******************************* */
  /** Replan a trajectory to a given target **/
  MANIPULATION_TASK_MESSAGE armReplan(p3d_vector3 target, int qSwitchId, SM_TRAJ &smTrajs);
  
  /** Computes a path for a given manipulation elementary task. Generate a set of Trajectories */
  MANIPULATION_TASK_MESSAGE armPlanTask(MANIPULATION_TASK_TYPE_STR task, int armId, configPt qStart, configPt qGoal, std::vector<double> &objStart, std::vector<double> &objGoto, const char* objectName,  const char* supportName, const char* placementName, gpGrasp& grasp, std::vector <p3d_traj*> &trajs);
  
  MANIPULATION_TASK_MESSAGE armPlanTask(MANIPULATION_TASK_TYPE_STR task, int armId, configPt qStart, configPt qGoal, std::vector<double> &objStart, std::vector<double> &objGoto, const char* objectName,  const char* supportName, const char* placementName, std::vector <p3d_traj*> &trajs);
  MANIPULATION_TASK_MESSAGE planNavigation(configPt qStart, configPt qGoal, bool fixAllArm, std::vector <p3d_traj*> &trajs);
  
#ifdef MULTILOCALPATH
  /** Computes a path for a given manipulation elementary task. Generate a set of SoftMotion Paths */
  MANIPULATION_TASK_MESSAGE armPlanTask(MANIPULATION_TASK_TYPE_STR task, int armId, configPt qStart, configPt qGoal, std::vector<double> &objStart, std::vector<double> &objGoto, const char* objectName,  const char* supportName, const char* placementName, gpGrasp& grasp, std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> &confs, std::vector <SM_TRAJ> &smTrajs);
  MANIPULATION_TASK_MESSAGE armPlanTask(MANIPULATION_TASK_TYPE_STR task, int armId, configPt qStart, configPt qGoal, std::vector<double> &objStart, std::vector<double> &objGoto, const char* objectName, const char* supportName, const char* placementName, std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> &confs, std::vector <SM_TRAJ> &smTrajs);
  MANIPULATION_TASK_MESSAGE armPlanTask(MANIPULATION_TASK_TYPE_STR task, int armId, configPt qStart, configPt qGoal, std::vector<double> &objStart, std::vector<double> &objGoto, const char* objectName, const char* supportName, const char* placementName, std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> &confs, SM_TRAJ &smTraj_q, SM_TRAJ &smTraj_x, Gb_th th_Rrob_Robj, Gb_th th_Robj_R7);
  MANIPULATION_TASK_MESSAGE planNavigation(configPt qStart, configPt qGoal,  bool fixAllArm, std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> &confs, std::vector <SM_TRAJ> &smTrajs);
#endif
	
private:
	
  /*!< pointer to the robot */
  p3d_rob * _robot;
  
  /* ******************************* */
  /* ******* Motion Planning ******* */
  /* ******************************* */
#ifdef MULTILOCALPATH
  /** Multilocalpath Id for the base */
  int _BaseMLP;
  /** Multilocalpath Id for the head */
  int _HeadMLP;
  /** Multilocalpath Id for the upper body (arms + torso) with linear localplanner */
  int _UpBodyMLP;
  /** Multilocalpath Id for the upper body (arms + torso) with softMotion localplanner */
  int _UpBodySmMLP;
#endif
  /** Time limit for the planning*/
  double _planningTimeLimit;
  /** Number of steps for the optimisation*/
  int _optimizeSteps;
  /** Time limit for the optimisation*/
  double _optimizeTimeLimit;
  /** Offset to generate the approach configuration of a grasp (carrying an object)*/
  double _safetyDistanceValue;
  
  /** nbTry to find placement configuration and graspApproche*/
  int _placementTry;
  
  /** use base motion to compute pick and place */
  bool _useBaseMotion;
  
  /* ******************************* */
  /* ******* Timing Data *********** */
  /* ******************************* */
  double _configurationTime;
  double _plannerTime;
  double _smootherTime;
  double _motionLawTime;
  double _totalTime;
	
  /* ******************************* */
  /* *******  Manipulation Data **** */
  /* ******************************* */
	ManipulationData _manipData;
  ManipulationConfigs _manipConf;
	
  /* ******************************* */
  /* ** Motion Planning funtions *** */
  /* ******************************* */
  
  bool _cleanRoadmap;
  
  //! Last computed geometric path
  p3d_traj* _robotPath;
  
  //! Basic Motion planning functions
  p3d_traj* (*_plannerMethod)(p3d_rob* robot, configPt qs, configPt qg);
  void (*_smoothingMethod)(p3d_rob* robot, p3d_traj* traj, int nbSteps, double maxTime);
  
  //! Replanning Function
  p3d_traj* (*_replanningMethod)(p3d_rob* robotPt, p3d_traj* traj, p3d_vector3 target, int deformationViaPoint);
  
};

#endif

#endif 
