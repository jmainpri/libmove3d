#ifndef __MANIPULATIONPLANNER_HPP__
#define __MANIPULATIONPLANNER_HPP__

#include "P3d-pkg.h"
#include "Localpath-pkg.h"

#include "ManipulationStruct.h"
#include "ManipulationUtils.hpp"

#include <map>

//! @ingroup manipulation
class  ManipulationPlanner {
  public:
  /* ******************************* */
  /* ******* (Con)Destructor ******* */
  /* ******************************* */
    ManipulationPlanner(p3d_rob * robotPt);
    virtual ~ManipulationPlanner();

  /* ******************************* */
  /* *********** Cleaning ********** */
  /* ******************************* */
    /** Clean the class */
    void clear();
    /** Delete the current roadmaps */
    int cleanRoadmap();
    /** Delete class robot trajectory list */
    int cleanTraj();

  /* ******************************* */
  /* ******* (Ge)Setters *********** */
  /* ******************************* */
    void setDebugMode(bool value);
  
#ifdef MULTILOCALPATH
  /** Multilocalpath Id for the base */
  int getBaseMLP() { return _BaseMLP; } 
  /** Multilocalpath Id for the head */
  int getHeadMLP() { return _HeadMLP; }
  /** Multilocalpath Id for the upper body (arms + torso) with linear localplanner */
  int getUpBodyMLP() { return _UpBodyMLP; }
  /** Multilocalpath Id for the upper body (arms + torso) with softMotion localplanner */
  int getUpBodysmMLP() { return _UpBodySmMLP; }
#endif
  
    void setOptimizeSteps(int nbSteps);
    int getOptimizeSteps(void) const;

    void setOptimizeTime(double time);
    double getOptimizeTime(void) const;

    void setOptimizeRedundentSteps(int nbSteps);
    int getOptimizeRedundentSteps(void) const;
    
    void setApproachFreeOffset(double offset);
    double getApproachFreeOffset(void) const;

    void setApproachGraspOffset(double offset);
    double getApproachGraspOffset(void) const;

    void setSafetyDistanceValue(double value);
    double getSafetyDistanceValue(void) const;
  
    inline p3d_rob* robot()  const{return _robot;}

    inline configPt robotStart() const{if (_robot != NULL) {return _robot->ROBOT_POS;} else {return NULL;}}
    inline configPt robotGoto() const{if (_robot != NULL) {return _robot->ROBOT_GOTO;} else {return NULL;}}
	

    inline ManipulationData getManipulationData()  const {return _configs;}
	/* ******************************* */
  /* ******* Hands / Grasping ****** */
  /* ******************************* */
	/** Fix the sampling of all the robots hands, desactivate hands self collisions and set the to rest configuration */
	void fixAllHands(configPt q, bool rest) const;
	/** UnFix the sampling of all the robots hands, activate hands self collision */
	void unFixAllHands();
	/** Fix the free flyer on the object pos. TODO: This function have to be changed to deal with cartesian mode (fix on the arm not on the object)*/
	void fixManipulationJoints(int armId, configPt q, p3d_rob* object);
	/** UnFix the free flyers*/
	void unfixManipulationJoints(int armId);
	/** Generate needed configurations from the given grasp and object position */
	MANIPULATION_TASK_MESSAGE findArmGraspsConfigs(int armId, p3d_rob* object, ManipulationData& configs);
	MANIPULATION_TASK_MESSAGE findArmGraspsConfigs(int armId, p3d_rob* object, gpGrasp grasp, ManipulationData& configs);
  
  //! Compute the distance between 2 configurations for 
  //! one local path group
  double distConfig( configPt q1, configPt q2, int group ) const ;
  
  //! Generates a free configuration from a worspace point and a grasp 
  configPt getFreeHoldingConf( p3d_rob* obj, int armId, gpGrasp& grasp, p3d_matrix4 tAtt, std::vector<double> &objGoto ) const;
  /** Generate the grasp configuration given the grasp the arm and the object.
  @return the attach matrix computed given the grasp and Tatt2 from the p3d file
  @return the configuration cost
  @return the grasp configuration */
  configPt getGraspConf(p3d_rob* object, int armId, gpGrasp& grasp, p3d_matrix4 tAtt, double& confCost) const;
  /** Generate the open configuration given the grasp configuration, the grasp, the arm and the object.*/
  configPt getOpenGraspConf(p3d_rob* object, int armId, gpGrasp& grasp, configPt graspConf) const;
  /** Generate the open approach configuration given the grasp configuration, the grasp, the arm, the attach matrix and the object.*/
  configPt getApproachFreeConf(p3d_rob* object, int armId, gpGrasp& grasp, configPt graspConf, p3d_matrix4 tAtt) const;
  /** Generate the grasp approach configuration given the grasp configuration, the grasp, the arm, the attach matrix and the object.*/
  configPt getApproachGraspConf(p3d_rob* object, int armId, gpGrasp& grasp, configPt graspConf, p3d_matrix4 tAtt) const;
  
	MANIPULATION_TASK_MESSAGE getGraspOpenApproachExtractConfs(p3d_rob* object, int armId, gpGrasp& grasp, p3d_matrix4 tAtt, ManipulationData& configs) const;
  
  MANIPULATION_TASK_MESSAGE computeManipulationData(int armId,p3d_rob* object);
  MANIPULATION_TASK_MESSAGE computeManipulationData(int armId,p3d_rob* object,gpGrasp grasp);
  
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
    int computeRRT(int smoothingSteps, double smootingTime, bool biDir);
    /** Set the parameters to compute a PRM the input is the maximal computation time*/
    /* TODO Add CXX PLanner computation*/
    MANIPULATION_TASK_MESSAGE armComputePRM(double ComputeTime);
    /** Compute a trajectory between the two given configurations. The constraints, the planning modes and the dof to plan have to be setted before */
    p3d_traj* computeTrajBetweenTwoConfigs(configPt qi, configPt qf);
#ifdef MULTILOCALPATH
    /** Given a trajectory, compute the corrsponding softMotion path */
    int computeSoftMotion(p3d_traj* traj, MANPIPULATION_TRAJECTORY_CONF_STR &confs, SM_TRAJ &smTraj);
#endif
    /** Move the arm from a free configuration to a grasping configuration of the object placed on a support */
    MANIPULATION_TASK_MESSAGE armPickGoto(int armId, configPt qStart, p3d_rob* object, std::vector <p3d_traj*> &trajs);
    MANIPULATION_TASK_MESSAGE armPickGoto(int armId, configPt qStart, p3d_rob* object, configPt graspConfig, configPt openConfig, configPt approachFreeConfig, std::vector <p3d_traj*> &trajs);
    MANIPULATION_TASK_MESSAGE armPickGoto(int armId, configPt qStart, p3d_rob* object, gpGrasp grasp, std::vector <p3d_traj*> &trajs);

    /** Move the arm from a grasping configuration (of the object placed on a support) to a free configuration */
    MANIPULATION_TASK_MESSAGE armPickTakeToFreePoint(int armId, configPt qStart, std::vector<double> &objGoto, p3d_rob* object, p3d_rob* support, std::vector <p3d_traj*> &trajs);
    MANIPULATION_TASK_MESSAGE armPickTakeToFree(int armId, configPt qStart, configPt qGoal, p3d_rob* object, p3d_rob* support, std::vector <p3d_traj*> &trajs);
    MANIPULATION_TASK_MESSAGE armPickTakeToFree(int armId, configPt qStart, configPt qGoal, p3d_rob* object, p3d_rob* support, configPt approachGraspConfig, gpGrasp &grasp, std::vector <p3d_traj*> &trajs);

    /** Move the arm from a grasping configuration to a configuration with the same grasp but a different object placement */
    MANIPULATION_TASK_MESSAGE armPickTakeToPlace(int armId, p3d_rob* object, p3d_rob* placement, std::vector <p3d_traj*> &trajs);

    /**  Move the arm from a free configuration to a placement configuration */
    MANIPULATION_TASK_MESSAGE armPlaceFromFree(int armId, configPt qStart, p3d_rob* object, p3d_rob* placement, std::vector <p3d_traj*> &trajs);

    /** Move the arm from a free configuration to a grasping configuration of the object placed on a support then to a placement configuration */
    MANIPULATION_TASK_MESSAGE armPickAndPlace(int armId, configPt qStart, configPt qGoal, p3d_rob* object, p3d_rob* placement, std::vector <p3d_traj*> &trajs);
	
		/** Move the arm to a stable object position then move the object to a free configuration with the object freeflying **/
		MANIPULATION_TASK_MESSAGE armPickGotoAndTakeToFree(int armId, configPt qStart, configPt qGoal, p3d_rob* object, p3d_rob* placement, std::vector <p3d_traj*> &trajs);
		 

#ifdef DPG
    /** \brief Check if the current path is in collision or not. Start from the begining of the trajectory
    \return 1 in case of collision, 0 otherwise*/
    int checkCollisionOnTraj();
    /** \brief Check if the current path is in collision or not. Start from the given local path id
    \return 1 in case of collision, 0 otherwise*/
    int checkCollisionOnTraj(int currentLpId);
    /** Plans a path to go from the currently defined ROBOT_POS config to the currently defined ROBOT_GOTO config for the arm only.
    \return MANIPULATION_TASK_OK for success */
    MANIPULATION_TASK_MESSAGE replanCollidingTraj(int currentLpId, std::vector <p3d_traj*> &trajs);
    /** Plans a path to go from the currently defined ROBOT_POS config to the currently defined ROBOT_GOTO config for the arm only.
    \return MANIPULATION_TASK_OK for success */
    MANIPULATION_TASK_MESSAGE replanCollidingTraj(int currentLpId, std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> &confs, std::vector <SM_TRAJ> &smTrajs);
#endif
  /* ******************************* */
  /* ******** Task Planning ******** */
  /* ******************************* */
    /** Computes a path for a given manipulation elementary task. Generate a set of Trajectories */
    MANIPULATION_TASK_MESSAGE armPlanTask(MANIPULATION_TASK_TYPE_STR task, int armId, configPt qStart, configPt qGoal, std::vector<double> &objStart, std::vector<double> &objGoto, const char* objectName,  const char* supportName, std::vector <p3d_traj*> &trajs);
    MANIPULATION_TASK_MESSAGE armPlanTask(MANIPULATION_TASK_TYPE_STR task, int armId, configPt qStart, configPt qGoal, std::vector<double> &objStart, std::vector<double> &objGoto, const char* objectName,  const char* supportName, gpGrasp grasp, std::vector <p3d_traj*> &trajs);

#ifdef MULTILOCALPATH
    /** Computes a path for a given manipulation elementary task. Generate a set of SoftMotion Paths */
    MANIPULATION_TASK_MESSAGE armPlanTask(MANIPULATION_TASK_TYPE_STR task, int armId, configPt qStart, configPt qGoal, std::vector<double> &objStart, std::vector<double> &objGoto, const char* objectName,  const char* supportName, std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> &confs, std::vector <SM_TRAJ> &smTrajs);
    MANIPULATION_TASK_MESSAGE armPlanTask(MANIPULATION_TASK_TYPE_STR task, int armId, configPt qStart, configPt qGoal, std::vector<double> &objStart, std::vector<double> &objGoto, const char* objectName,  const char* supportName, gpGrasp grasp, std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> &confs, std::vector <SM_TRAJ> &smTrajs);
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
    /** Number of steps for the optimisation*/
    int _optimizeSteps;
    /** Time limit for the optimisation*/
    double _optimizeTime;
    /** Number of steps when optimizing the redundent joint*/
    int _optimizeRedundentSteps;
    /** Offset to generate the approach configuration of a grasp (not carrying an object)*/
    double _approachFreeOffset;
    /** Offset to generate the approach configuration of a grasp (carrying an object)*/
    double _approachGraspOffset;
    /** Offset to generate the approach configuration of a grasp (carrying an object)*/
    double _safetyDistanceValue;
	
  /* ******************************* */
  /* *******  Manipulation Data **** */
  /* ******************************* */
	ManipulationData _configs;
	
	std::map<MANIPULATION_TASK_MESSAGE,std::string> _ErrorMap;
};

#endif
