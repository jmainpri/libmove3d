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
#ifndef __MANIPULATION_H__
#define __MANIPULATION_H__

#if defined (GRASP_PLANNING) && defined (LIGHT_PLANNER)

#include <vector>
#include <map>

#include "GraspPlanning-pkg.h"

#include "P3d-pkg.h"
#include <list>

#include "ManipulationStruct.hpp"
#include "ManipulationUtils.hpp"
#include "ManipulationData.hpp"

//! @ingroup manipulation 
class DoubleGraspData{
  public:
  DoubleGraspData(p3d_rob* robot){
    _robot = robot;
    _config = p3d_alloc_config(robot);
  }
  DoubleGraspData(p3d_rob* robot, gpDoubleGrasp dGrasp, configPt cCConfig){
    _robot = robot;
    _doubleGrasp = dGrasp;
    _config = cCConfig;
  }
  virtual ~DoubleGraspData(){
    p3d_destroy_config(_robot, _config);
  }
  inline gpDoubleGrasp getDoubleGrasp(){
    return _doubleGrasp;
  }
  inline void setDoubleGrasp(gpDoubleGrasp doubleGrasp){
    _doubleGrasp = doubleGrasp;
  }
  inline configPt getConfig(){
    return _config;
  }
  inline void setConfig(configPt cCConfig){
    _config = cCConfig;
  }
  inline void setObjectExchangeMat(p3d_matrix4 objectExchangeMat){
    p3d_mat4Copy(objectExchangeMat, _objectExchangeMat);
  }
  inline void getObjectExchangeMat(p3d_matrix4 objectExchangeMat){
    p3d_mat4Copy(_objectExchangeMat, objectExchangeMat);
  }
  private:
  p3d_rob* _robot;
  gpDoubleGrasp _doubleGrasp;
  p3d_matrix4 _objectExchangeMat;
  configPt _config;
};

//! @ingroup manipulation 
class Manipulation{
  public :
    Manipulation(p3d_rob *robot);
    virtual ~Manipulation();
    void clear();
    void computeOfflineRoadmap(); 
    p3d_traj* computeRegraspTask(configPt startConfig, configPt gotoConfig, std::string offlineFile);
    p3d_traj* computeRegraspTask(configPt startConfig, configPt gotoConfig, std::string offlineFile, int whichTest);
  
    int findAllArmsGraspsConfigs(p3d_matrix4 objectStartPos, p3d_matrix4 objectEndPos);
    int findAllSpecificArmGraspsConfigs(int armId, p3d_matrix4 objectPos);
    double getCollisionFreeGraspAndApproach(p3d_matrix4 objectPos, gpHand_properties handProp, gpGrasp grasp, int whichArm, p3d_matrix4 tAtt, configPt* graspConfig, configPt* approachConfig);
    void computeExchangeMat(configPt startConfig, configPt gotoConfig);
    void computeDoubleGraspConfigList();
    inline void setExchangeMat(p3d_matrix4 exchangeMat){
      p3d_mat4Copy(exchangeMat, _exchangeMat);
    }
    inline void getExchangeMat(p3d_matrix4 exchangeMat){
      p3d_mat4Copy(_exchangeMat, exchangeMat);
    }
    void drawSimpleGraspConfigs();
    void drawDoubleGraspConfigs();
    void printStatDatas();
  
  protected:
    double getRobotGraspArmCost(gpGrasp grasp, configPt q);
    int getCollisionFreeDoubleGraspAndApproach(p3d_matrix4 objectPos, std::vector<gpHand_properties> armsProp, gpDoubleGrasp doubleGrasp, configPt* doubleGraspConfig);
    std::vector<gpHand_properties> InitHandProp(int armId);
    std::list<gpGrasp>* getGraspListFromMap(int armId);
    int checkTraj(p3d_traj * traj, p3d_graph* graph);
  
  private :
    std::map < int, std::map<int, ManipulationData*, std::less<int> >, std::less<int> > _handsGraspsConfig;
    std::list<DoubleGraspData*> _handsDoubleGraspsConfigs;
    p3d_rob * _robot;
    p3d_graph * _offlineGraph;
    static const int _maxColGrasps = 10;
    p3d_matrix4 _exchangeMat;
    std::vector<std::vector<double> > _statDatas;
};



//! @ingroup manipulation 
class  Manipulation_JIDO {
  private :
     p3d_rob * _robotPt; /*!< pointer to the robot (Jido) */
     p3d_rob * _hand_robotPt;  /*!< pointer to the hand robot (a freeflyer robot with the same structure as the robot's hand) */
     //gpHand_type _handType;
     gpHand_properties _handProp;  /*!< information about the used hand */
     std::list<gpGrasp> _graspList; 
     gpGrasp _grasp;   /*!< the current grasp */
     unsigned int _graspID; /*!< the current grasp ID */
     gpPlacement _placement;   /*!< the current or target object placement */     
     p3d_jnt *_cameraJnt; /*!< the robot's joint that gives the pose of the pan/tilt camera */ 
     double _cameraFOV; /*!< robot's camera field of view angle (IN DEGREES) used for grasp visibility score computation */
     int _cameraImageWidth, _cameraImageHeight; /*!< dimensions of the synthetic images of what the robot views that will be used to compute the visibility score. Small values gives faster computations. */

     configPt _configStart;
     configPt _configGoto;

     std::vector<configPt> _configTraj; /*!< this array stores the key configurations that will be used to compute a sequence of trajectory*/

     double _liftUpDistance;  /*!< the distance the object is lifted up after it is grasped, before any other movement */

     int _nbGraspsToTestForPickGoto; /*!< the  _nbGraspsToTestForPickGoto first grasps of the grasp list will be tested for the ARM_PICK_GOTO task planning */

     //! for stable placement computation, the space of possible poses on the support is sampled with the following steps:
     double _placementTranslationStep; /*!< the translation step of the discretization of the horizontal faces of the support */
     double _placementNbOrientations; /*!<  the number of orientations (around vertical axis) that will be tested to place the object on the support*/
     double _placementOffset; /*!< the arm configuration used for placement object will be slightly vertically shifted in order to be sure that the object will contact the support in real life (set the value with caution)*/

     double _QCUR[6];
     double _QGOAL[6];
     double _XCUR[6];
     double _XGOAL[6];
     double _qrest[6];  /*!< rest configuration of the arm */

     p3d_rob *_object; /*!< the object to work with (a freeflyer robot) */
     p3d_rob *_support; /*!< the support to possibly place the object on (a freeflyer robot) */
     p3d_rob *_human; /*!< the human that may interact with the robot */

     std::list<gpPlacement> _placementList; /*!< stable placements of the object (context independent) */
     std::list<gpPlacement> _placementOnSupportList; /*!< stable placements of the object on the current support (context dependent) */
     bool _capture;
     bool _cartesian; /*!< choose to plan the arm motion in cartesian space (for the end effector) or joint space  */
     bool _objectGrabed; // not used for now (redundent with robot->isCarryingObject)
 public:
     bool displayGrasps; /*!< boolean to enable/disable the display of the grasps of the current grasp list */
     bool displayPlacements; /*!<  boolean to enable/disable the display of the placements of the current object pose list */
     std::vector < std::vector <double> > positions;
     std::vector <int> lp;
 public :
     Manipulation_JIDO(p3d_rob * robotPt, gpHand_type handType);
     virtual ~Manipulation_JIDO();
     void clear();

     configPt robotStart();
     configPt robotGoto();
     configPt robotRest();

     void setCapture(bool v);
     bool getCapture();
     int centerCamera();
     int forbidWindowEvents();
     int allowWindowEvents();
     /*Functions relative to JIDO */
     int setArmQ(double q1, double q2, double q3, double q4, double q5, double q6);
     int getArmQ(double *q1, double *q2, double *q3, double *q4, double *q5, double *q6);
     int setArmX(double x, double y, double z, double rx, double ry, double rz);
     int setArmX(double x, double y, double z, unsigned int nbTries= 30);
     int getArmX(double* x, double* y, double* z, double* rx, double* ry, double* rz);
     void setArmCartesian(bool v);
     bool getArmCartesian();
     int setArmTask(MANIPULATION_TASK_TYPE_STR t);
     int setObjectToManipulate(char *objectName);
     int setSupport(char *supportName);
     int setHuman(char *humanName);
     int setCameraJnt(char *cameraJntName);
     int setCameraFOV(double fov);
     int setCameraImageSize(int width, int height);
     int setNbGraspsToTestForPickGoto(int n);
     int reduceGraspList(int maxSize);
     int printConstraintInfo();
     int setPoseWrtEndEffector(double x, double y, double z, double rx, double ry, double rz, configPt q);
     int dynamicGrasping(char *robot_name, char *hand_robot_name, char *object_name);
     int robotBaseGraspConfig(char *objectName, double *x, double *y, double *theta);
     MANIPULATION_TASK_MESSAGE armPlanTask(MANIPULATION_TASK_TYPE_STR task, configPt qStart, configPt qGoal, char* objectName, std::vector <int> &lp, std::vector < std::vector <double> > &positions);

  
     int armComputePRM();
     
     int cleanRoadmap();
     int cleanTraj();

     int grabObject(char* objectName);
     int releaseObject();

     void draw();
 
     int setLiftUpDistance(double dist) {
       _liftUpDistance= fabs(dist); 
       return 0;
     }
     double liftUpDistance() {
       return _liftUpDistance;
     }
     
     /* Functions relative to other robots */
     int setFreeflyerPose(p3d_rob *robotPt, double x, double y, double z, double rx, double ry, double rz);
     int getObjectPose(p3d_matrix4 pose);
     int getObjectPose(double *x, double *y, double *z, double *rx, double *ry, double *rz);
     int setFreeflyerPoseByName(char *name, double x, double y, double z, double rx, double ry, double rz);
     p3d_obj * getObjectByName(char *object_name);
     int setObjectPoseWrtEndEffector(double x, double y, double z, double rx, double ry, double rz);

     /* Functions relative to object grasping */
    int findPregraspAndGraspConfiguration(double distance, double *pre_q1, double *pre_q2, double *pre_q3, double *pre_q4, double *pre_q5, double *pre_q6, double *q1, double *q2, double *q3, double *q4, double *q5, double *q6);
    bool isObjectGraspable(char *objectName);

     /* Functions relative to object placement */
    int findPlacementConfigurations(double distance, double *pre_q1, double *pre_q2, double *pre_q3, double *pre_q4, double *pre_q5, double *pre_q6, double *q1, double *q2, double *q3, double *q4, double *q5, double *q6);

    int addConfigTraj(configPt config);
    int clearConfigTraj();
    int copyConfigTrajToFORM();
    int destroyTrajectories();
    int checkCollisionOnTraj();
    int checkCollisionOnTraj(int currentLpId);
    int replanCollidingTraj(int currentLpId, std::vector <int> &lp, std::vector < std::vector <double> > &positions);
  protected:
     /*Functions relative to JIDO */
     int computeTrajBetweenTwoConfigs(bool cartesian, configPt qi, configPt qf);
     int computeGraspList();
     int findSimpleGraspConfiguration(double *q1, double *q2, double *q3, double *q4, double *q5, double *q6);
     int computePlacementList();
     int markGraspAsTested(int id);

     int computeRRT();
     int computeOptimTraj();
};

#endif

#endif