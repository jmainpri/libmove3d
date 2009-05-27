/* 
 *    File generated automatically. Do not edit by hand.
 */
#ifndef __CEXTRACT__

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


#include "Planner-pkg.h"
extern void pickObject(p3d_rob * robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2);
extern void graspObject(p3d_rob * robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2);

extern void setRobotStartPosByObjectPos(p3d_rob* robot, double x, double y, double z, double rx, double ry, double rz);
extern void setRobotGotoPosByObjectPos(p3d_rob* robot, double x, double y, double z, double rx, double ry, double rz);
extern configPt setTwoArmsRobotGraspPos(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2);
extern configPt setTwoArmsRobotGraspApproachPos(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2);
extern void setTwoArmsRobotGraspAndApproachPos(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2, configPt* graspConf, configPt* approachConf);

#endif /* __CEXTRACT__ */
