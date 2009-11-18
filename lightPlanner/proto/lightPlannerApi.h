#ifdef LIGHT_PLANNER

#ifndef LIGHTPLANNERAPI_H
#define LIGHTPLANNERAPI_H
#include "Planner-pkg.h"

extern void deactivateCcCntrts(p3d_rob * robot, int cntrtNum);
extern void activateCcCntrts(p3d_rob * robot, int cntrtNum);
extern void switchBBActivationForGrasp(void);
extern void setSafetyDistance(p3d_rob* robot, double dist);
extern void getObjectBaseAttachMatrix(p3d_matrix4 base, p3d_matrix4 object, p3d_matrix4 result);
extern void deactivateHandsVsObjectCol(p3d_rob* robot);
extern void activateHandsVsObjectCol(p3d_rob* robot);
extern void deactivateObjectCol(p3d_rob* robot);
extern void activateObjectCol(p3d_rob* robot);
extern void fixAllJointsExceptBaseAndObject(p3d_rob * robot, configPt conf);
extern void unFixAllJointsExceptBaseAndObject(p3d_rob * robot);
extern void fixJoint(p3d_rob * robot, p3d_jnt * joint, p3d_matrix4 initPos);
extern void unFixJoint(p3d_rob * robot, p3d_jnt * joint);
extern double* getJntDofValue(p3d_rob * robot, p3d_jnt * joint, p3d_matrix4 initPos);
extern void setAndActivateTwoJointsFixCntrt(p3d_rob * robot, p3d_jnt* passiveJnt, p3d_jnt* activeJnt);
extern void desactivateTwoJointsFixCntrt(p3d_rob * robot, p3d_jnt* passiveJnt, p3d_jnt* activeJnt);
extern void shootTheObjectInTheWorld(p3d_rob* robot, p3d_jnt* objectJnt);
extern void shootTheObjectArroundTheBase(p3d_rob* robot, p3d_jnt* baseJnt, p3d_jnt* objectJnt, double radius);
extern void setAndActivateTwoJointsFixCntrt(p3d_rob * robot, p3d_jnt* passiveJnt, p3d_jnt* activeJnt);
#endif

#endif
