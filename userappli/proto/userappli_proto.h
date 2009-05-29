/* 
 *    File generated automatically. Do not edit by hand.
 */
#ifndef __CEXTRACT__
#include "Planner-pkg.h"

extern void openChainPlannerOptions(void);
extern void closedChainPlannerOptions(void);
extern void pathOpenChainOptions(void);
extern void pathGraspOptions(int start);

extern void globalPlanner(void);
extern void findPath(void);
extern void p3d_specificSuperGraphLearn(void);

extern void viewTraj(void);
extern void optimiseTrajectory(void);

extern void checkForCollidingLpAlongPath(void);

extern void deactivateHandsVsObjectCol(p3d_rob* robot);
extern void activateHandsVsObjectCol(p3d_rob* robot);

extern void pickAndMoveObjectByMat(p3d_rob * robot, p3d_matrix4 objectInitPos, p3d_matrix4 objectGotoPos, p3d_matrix4 att1, p3d_matrix4 att2);
extern void pickAndMoveObjectByConf(p3d_rob * robot, p3d_matrix4 objectInitPos, configPt approachConf, configPt graspConf, configPt finalConf);

extern void pickObjectByMat(p3d_rob * robot, p3d_matrix4 objectInitPos, p3d_matrix4 att1, p3d_matrix4 att2);
extern p3d_traj* pickObjectByConf(p3d_rob * robot, p3d_matrix4 objectInitPos, configPt approachConf);

extern void moveObjectByMat(p3d_rob * robot, p3d_matrix4 objectGotoPos, p3d_matrix4 att1, p3d_matrix4 att2);
extern p3d_traj* moveObjectByConf(p3d_rob * robot, configPt finalConf);

extern void graspObjectByMat(p3d_rob * robot, p3d_matrix4 objectInitPos, p3d_matrix4 att1, p3d_matrix4 att2);
extern p3d_traj* graspObjectByConf(p3d_rob * robot, p3d_matrix4 objectInitPos, configPt approachConf, configPt graspConf);

extern void setRobotStartPosByObjectPos(p3d_rob* robot, double x, double y, double z, double rx, double ry, double rz);
extern void setRobotGotoPosByObjectPos(p3d_rob* robot, double x, double y, double z, double rx, double ry, double rz);
extern configPt setTwoArmsRobotGraspPos(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2);
extern configPt setTwoArmsRobotGraspApproachPos(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2);
extern void setTwoArmsRobotGraspAndApproachPos(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2, configPt* graspConf, configPt* approachConf);

#endif /* __CEXTRACT__ */
