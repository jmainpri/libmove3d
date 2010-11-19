#ifndef LIGHTPLANNER_H
#define LIGHTPLANNER_H
#include "Planner-pkg.h"
#ifdef MULTILOCALPATH
extern void initLightPlannerForMLP(p3d_rob* robot);
#endif
extern void saveTrajInFile(const char* fileName, p3d_traj* traj, int smallIntervals,double dmax);
extern void optimiseTrajectory(p3d_rob* robot, p3d_traj* traj, int nbSteps, double maxTime);
extern p3d_traj* platformGotoObjectByMat(p3d_rob * robot, p3d_matrix4 objectStartPos, p3d_matrix4 att1, p3d_matrix4 att2);
extern p3d_traj* platformGotoObjectByConf(p3d_rob * robot,  p3d_matrix4 objectStartPos, configPt conf);
extern traj* pickObject(p3d_rob* robot, p3d_matrix4 objectStartPos, p3d_matrix4 att1, p3d_matrix4 att2);
extern p3d_traj* gotoObjectByMat(p3d_rob * robot, p3d_matrix4 objectStartPos, p3d_matrix4 att1, p3d_matrix4 att2);
extern p3d_traj* gotoObjectByConf(p3d_rob * robot,  p3d_matrix4 objectStartPos, configPt conf, bool biDir);
extern p3d_traj* touchObjectByMat(p3d_rob * robot, p3d_matrix4 objectStartPos, p3d_matrix4 att1, p3d_matrix4 att2);
extern p3d_traj* touchObjectByConf(p3d_rob * robot,  p3d_matrix4 objectStartPos, configPt conf);
extern traj* carryObject(p3d_rob* robot, p3d_matrix4 objectGotoPos, p3d_matrix4 att1, p3d_matrix4 att2);
extern p3d_traj* carryObjectByMat(p3d_rob * robot, p3d_matrix4 objectGotoPos, p3d_matrix4 att1, p3d_matrix4 att2);
extern p3d_traj* carryObjectByConf(p3d_rob * robot, p3d_matrix4 objectGotoPos, configPt conf, int cntrtToActivate, int cartesian, bool biDir);
extern p3d_traj* platformCarryObjectByMat(p3d_rob * robot, p3d_matrix4 objectGotoPos, p3d_matrix4 att1, p3d_matrix4 att2);
extern p3d_traj* platformCarryObjectByConf(p3d_rob * robot,  p3d_matrix4 objectGotoPos, configPt conf, int cntrtToActivate);
extern void deleteAllGraphs(void);
extern void preComputePlatformGotoObject(p3d_rob * robot, p3d_matrix4 objectStartPos);
extern void preComputeGotoObject(p3d_rob * robot, p3d_matrix4 objectStartPos);
extern void preComputePlatformCarryObject(p3d_rob * robot);
extern void preComputeCarryObject(p3d_rob * robot, p3d_matrix4 att1, p3d_matrix4 att2);


#ifdef GRASP_PLANNING
#include "GraspPlanning-pkg.h"
extern void debugLightPlanner();
extern p3d_traj* graspTheObject(p3d_rob * robot, p3d_matrix4 objectStartPos, int* whichArm, gpGrasp* curGrasp, bool cartesian);
extern p3d_traj* carryTheObject(p3d_rob * robot, p3d_matrix4 objectGotoPos, gpGrasp grasp, int whichArm, bool cartesian);
#endif

#endif

extern int findBestExchangePosition(p3d_rob *object, p3d_vector3 Oi, p3d_vector3 Of, p3d_vector3 Ai, p3d_vector3 Af, p3d_vector3 Bi, p3d_vector3 Bf, p3d_vector3 result);
extern int findBestExchangePosition2(p3d_rob *object, p3d_matrix4 Oi, p3d_matrix4 Of, p3d_matrix4 Ai, p3d_matrix4 Af, p3d_matrix4 Bi, p3d_matrix4 Bf, p3d_matrix4 result);
#ifdef GRASP_PLANNING
extern int findBestExchangePositionGraphic(p3d_rob *object, p3d_vector3 Oi, p3d_vector3 Of, p3d_vector3 Ai, p3d_vector3 Af, p3d_vector3 Bi, p3d_vector3 Bf, p3d_vector3 result);
#endif
