#ifndef LIGHTPLANNERAPI_H
#define LIGHTPLANNERAPI_H
#include "Planner-pkg.h"
#include "P3d-pkg.h"

extern void deactivateCcCntrts(p3d_rob * robot, int cntrtNum);
extern void activateCcCntrts(p3d_rob * robot, int cntrtNum, bool nonUsedCntrtDesactivation);
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
extern p3d_cntrt* setAndActivateTwoJointsFixCntrt(p3d_rob * robot, p3d_jnt* passiveJnt, p3d_jnt* activeJnt);
extern void desactivateTwoJointsFixCntrt(p3d_rob * robot, p3d_jnt* passiveJnt, p3d_jnt* activeJnt);
extern void shootTheObjectInTheWorld(p3d_rob* robot, p3d_jnt* objectJnt);
extern void shootTheObjectArroundTheBase(p3d_rob* robot, p3d_jnt* baseJnt, p3d_jnt* objectJnt, double radius);
extern p3d_cntrt * findTwoJointsFixCntrt(p3d_rob* robot, p3d_jnt* passiveJnt, p3d_jnt* activeJnt);
extern int getGraspingArm(p3d_rob* robot, bool cartesian);
extern int getClosestWristToTheObject(p3d_rob* robot);
extern int getClosestWristToTheObject(p3d_rob* robot, p3d_rob* object);
extern int getClosestWristToTheObject(p3d_rob* robot, p3d_matrix4 objectPos);
#ifdef GRASP_PLANNING
#include "GraspPlanning-pkg.h"
extern int getBetterCollisionFreeGraspAndApproach(p3d_rob* robot, p3d_matrix4 objectPos, gpHand_type handType, p3d_matrix4 tAtt, configPt* graspConfig, configPt* approachConfig, gpGrasp * grasp);
extern int selectHandAndGetGraspApproachConfigs(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 tAtt, configPt* graspConfig, configPt* approachConfig, gpGrasp* grasp, int* whichArm, bool cartesian);
#endif

#endif
