#ifndef ROBOTPOS_H
#define ROBOTPOS_H
#include "Planner-pkg.h"

extern configPt p3d_getRobotBaseConfigAroundTheObject(p3d_rob* robot, p3d_jnt* baseJnt, p3d_jnt* objectJnt, double x, double y, double z, double rx, double ry, double rz, double minRadius, double maxRadius, int shootBase, int cntrtToActivate, bool nonUsedCntrtDesactivation);
extern configPt setBodyConfigForBaseMovement(p3d_rob * robot, configPt baseConfig, configPt bodyConfig);
extern void adaptClosedChainConfigToBasePos(p3d_rob *robot, p3d_matrix4 base, configPt refConf);
extern configPt setTwoArmsRobotGraspPosWithoutBase(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2, int cntrtToActivate, bool nonUsedCntrtDesactivation);
extern configPt setTwoArmsRobotGraspApproachPosWithoutBase(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2, int cntrtToActivate, bool nonUsedCntrtDesactivation);
extern configPt setTwoArmsRobotGraspApproachPosWithHold(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2, int cntrtToActivate);
extern configPt setTwoArmsRobotGraspPosWithHold(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2, int cntrtToActivate);
extern void setTwoArmsRobotGraspAndApproachPosWithHold(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2, configPt* graspConf, configPt* approachConf);
extern double computeRobotConfCostSpecificArm(p3d_rob* robot, configPt q, int whichArm);
extern double computeRobotConfCost(p3d_rob* robot, configPt q);
extern void correctGraphForNewFixedJoints(p3d_graph* graph, configPt refConf, int nbJoints, p3d_jnt** joints);

#ifdef GRASP_PLANNING
#include "GraspPlanning-pkg.h"
extern void correctGraphForHandsAndObject(p3d_rob* robot, p3d_graph* graph, int rightHandStatus, gpGrasp rightGrasp, int leftHandStatus, gpGrasp leftGrasp, bool carryobject, int whichArm, p3d_matrix4 tAtt);
#endif

#endif
