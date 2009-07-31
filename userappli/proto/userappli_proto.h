/* 
 *    File generated automatically. Do not edit by hand.
 */
#ifndef __CEXTRACT__
#include "Planner-pkg.h"

extern void openChainPlannerOptions(void);
extern void closedChainPlannerOptions(void);
extern void pathOpenChainOptions(void);
extern void pathGraspOptions(void);
extern void switchBBActivationForGrasp(void);

extern void viewTraj(void);
extern void saveTrajInFile(p3d_traj* traj);
extern void optimiseTrajectory(void);

extern void checkForCollidingLpAlongPath(void);

extern void deactivateHandsVsObjectCol(p3d_rob* robot);
extern void activateHandsVsObjectCol(p3d_rob* robot);

extern void disableAutoCol(p3d_rob* robot);
extern void enableAutoCol(p3d_rob* robot);
/** ////////// Setters /////////////*/
extern void setLinearLp(int useLinear);
extern void setSafetyDistance(double safetyDistance);
/** ////////// Setters /////////////*/
/** ////////// Fonctions Principales /////////////*/
extern void computeOfflineOpenChain(p3d_rob* robot, p3d_matrix4 objectInitPos);
extern void computeOfflineClosedChain(p3d_rob* robot, p3d_matrix4 objectInitPos);

extern void pickAndMoveObjectByMat(p3d_rob * robot, p3d_matrix4 objectInitPos, p3d_matrix4 objectGotoPos, p3d_matrix4 att1, p3d_matrix4 att2);
extern void pickAndMoveObjectByConf(p3d_rob * robot, p3d_matrix4 objectInitPos, configPt approachConf, configPt graspConf, configPt finalConf);

extern void pickObjectByMat(p3d_rob * robot, p3d_matrix4 objectInitPos, p3d_matrix4 att1, p3d_matrix4 att2);
extern p3d_traj* pickObjectByConf(p3d_rob * robot, p3d_matrix4 objectInitPos, configPt approachConf);

extern void moveObjectByMat(p3d_rob * robot, p3d_matrix4 objectGotoPos, p3d_matrix4 att1, p3d_matrix4 att2);
extern p3d_traj* moveObjectByConf(p3d_rob * robot, configPt initConf, configPt finalConf);

extern void graspObjectByMat(p3d_rob * robot, p3d_matrix4 objectInitPos, p3d_matrix4 att1, p3d_matrix4 att2);
extern p3d_traj* graspObjectByConf(p3d_rob * robot, p3d_matrix4 objectInitPos, configPt approachConf, configPt graspConf);
/** ////////// Fonctions Principales /////////////*/
/** //////////// Compute Robot Pos /////////////*/
extern configPt setTwoArmsRobotGraspPosWithoutBase(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2);
extern configPt setTwoArmsRobotGraspApproachPosWithHold(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2);
extern configPt setTwoArmsRobotGraspPosWithHold(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2);
extern void setTwoArmsRobotGraspAndApproachPosWithHold(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2, configPt* graspConf, configPt* approachConf);
/** //////////// Compute Robot Pos /////////////*/
/** ////////// MISC /////////////*/
extern void globalPlanner(void);
extern void findPath(void);
extern void p3d_specificSuperGraphLearn(void);
extern void p3d_computeTests(void);
/** ////////// MISC /////////////*/


#endif /* __CEXTRACT__ */
