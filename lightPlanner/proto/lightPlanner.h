#ifndef LIGHTPLANNER_H
#define LIGHTPLANNER_H

#include "Planner-pkg.h"
#ifdef MULTILOCALPATH
void initLightPlannerForMLP(p3d_rob* robot);
#endif
void saveTrajInFile(const char* fileName, p3d_traj* traj, int smallIntervals,double dmax);
p3d_traj* rrtQuerry(p3d_rob* robot, configPt qs, configPt qg);
void optimiseTrajectory(p3d_rob* robot, p3d_traj* traj, int nbSteps, double maxTime);
p3d_traj* platformGotoObjectByMat(p3d_rob * robot, p3d_matrix4 objectStartPos, p3d_matrix4 att1, p3d_matrix4 att2);
p3d_traj* platformGotoObjectByConf(p3d_rob * robot,  p3d_matrix4 objectStartPos, configPt conf);
traj* pickObject(p3d_rob* robot, p3d_matrix4 objectStartPos, p3d_matrix4 att1, p3d_matrix4 att2);
p3d_traj* gotoObjectByMat(p3d_rob * robot, p3d_matrix4 objectStartPos, p3d_matrix4 att1, p3d_matrix4 att2);
p3d_traj* gotoObjectByConf(p3d_rob * robot,  p3d_matrix4 objectStartPos, configPt conf, bool biDir);
p3d_traj* touchObjectByMat(p3d_rob * robot, p3d_matrix4 objectStartPos, p3d_matrix4 att1, p3d_matrix4 att2);
p3d_traj* touchObjectByConf(p3d_rob * robot,  p3d_matrix4 objectStartPos, configPt conf);
traj* carryObject(p3d_rob* robot, p3d_matrix4 objectGotoPos, p3d_matrix4 att1, p3d_matrix4 att2);
p3d_traj* carryObjectByMat(p3d_rob * robot, p3d_matrix4 objectGotoPos, p3d_matrix4 att1, p3d_matrix4 att2);
p3d_traj* carryObjectByConf(p3d_rob * robot, p3d_matrix4 objectGotoPos, configPt conf, int cntrtToActivate, int cartesian, bool biDir);
p3d_traj* platformCarryObjectByMat(p3d_rob * robot, p3d_matrix4 objectGotoPos, p3d_matrix4 att1, p3d_matrix4 att2);
p3d_traj* platformCarryObjectByConf(p3d_rob * robot,  p3d_matrix4 objectGotoPos, configPt conf, int cntrtToActivate);
void deleteAllGraphs(void);
void preComputePlatformGotoObject(p3d_rob * robot, p3d_matrix4 objectStartPos);
void preComputeGotoObject(p3d_rob * robot, p3d_matrix4 objectStartPos);
void preComputePlatformCarryObject(p3d_rob * robot);
void preComputeCarryObject(p3d_rob * robot, p3d_matrix4 att1, p3d_matrix4 att2);

#ifdef GRASP_PLANNING
#include "GraspPlanning-pkg.h"
void debugLightPlanner();
p3d_traj* graspTheObject(p3d_rob * robot, p3d_matrix4 objectStartPos, int* whichArm, gpGrasp* curGrasp, bool cartesian);
p3d_traj* carryTheObject(p3d_rob * robot, p3d_matrix4 objectGotoPos, gpGrasp grasp, int whichArm, bool cartesian);
#endif

#endif

int findBestExchangePosition(p3d_rob *object, p3d_vector3 Oi, p3d_vector3 Of, p3d_vector3 Ai, p3d_vector3 Af, p3d_vector3 Bi, p3d_vector3 Bf, p3d_vector3 result);
int findBestExchangePosition2(p3d_rob *object, p3d_matrix4 Oi, p3d_matrix4 Of, p3d_matrix4 Ai, p3d_matrix4 Af, p3d_matrix4 Bi, p3d_matrix4 Bf, p3d_matrix4 result);
#ifdef GRASP_PLANNING
int findBestExchangePositionGraphic(p3d_rob *object, p3d_vector3 Oi, p3d_vector3 Of, p3d_vector3 Ai, p3d_vector3 Af, p3d_vector3 Bi, p3d_vector3 Bf, p3d_vector3 result);
#endif

std::map<double, configPt, std::less<double> > * searchForLowCostNode(p3d_rob* robot, configPt startConfig, int whichArm);
void correctGraphForNewFixedJoints(p3d_graph* graph, configPt refConf, int nbJoints, p3d_jnt** joints);
void removeAloneNodesInGraph(p3d_rob* robot, p3d_graph* graph);
void correctGraphForHandsAndObject(p3d_rob* robot, p3d_graph* graph, int rightHandStatus, gpGrasp rightGrasp, int leftHandStatus, gpGrasp leftGrasp, bool carryobject, int whichArm);
