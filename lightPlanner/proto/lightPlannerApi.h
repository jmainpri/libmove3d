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
#ifndef LIGHTPLANNERAPI_H
#define LIGHTPLANNERAPI_H
#include "Planner-pkg.h"
#include "P3d-pkg.h"

void p3d_change_ff_translation_bounds( p3d_rob* robotPt, double limits[6] );
void deactivateCcCntrts(p3d_rob * robot, int cntrtNum);
void activateCcCntrts(p3d_rob * robot, int cntrtNum, bool nonUsedCntrtDesactivation);
void switchBBActivationForGrasp(void);
void setSafetyDistance(p3d_rob* robot, double dist);
void getObjectBaseAttachMatrix(p3d_matrix4 base, p3d_matrix4 object, p3d_matrix4 result);
void deactivateHandsVsObjectCol(p3d_rob* robot);
void activateHandsVsObjectCol(p3d_rob* robot);
void deactivateObjectCol(p3d_rob* robot);
void activateObjectCol(p3d_rob* robot);
double** saveJointSamplingState(p3d_rob* robot);
void restoreJointSamplingState(p3d_rob* robot, double** jointSamplingState);
void destroyJointSamplingState(p3d_rob* robot, double** jointSamplingState);
void fixAllJointsWithoutArm(p3d_rob* robot, int armId);
void fixAllJointsExceptBase(p3d_rob * robot);
void fixAllJointsExceptBaseAndObject(p3d_rob * robot, configPt conf);
void unFixAllJointsExceptBaseAndObject(p3d_rob * robot);
void fixJoint(p3d_rob * robot, p3d_jnt * joint, p3d_matrix4 initPos);
void unFixJoint(p3d_rob * robot, p3d_jnt * joint);
double* getJntDofValue(p3d_rob * robot, p3d_jnt * joint, p3d_matrix4 initPos);
p3d_cntrt* setAndActivateTwoJointsFixCntrt(p3d_rob * robot, p3d_jnt* passiveJnt, p3d_jnt* activeJnt);
void desactivateTwoJointsFixCntrt(p3d_rob * robot, p3d_jnt* passiveJnt, p3d_jnt* activeJnt);
void shootTheObjectInTheWorld(p3d_rob* robot, p3d_jnt* objectJnt);
void shootTheObjectArroundTheBase(p3d_rob* robot, p3d_jnt* baseJnt, p3d_jnt* objectJnt, double radius);
p3d_cntrt * findTwoJointsFixCntrt(p3d_rob* robot, p3d_jnt* passiveJnt, p3d_jnt* activeJnt);
int getGraspingArm(p3d_rob* robot, bool cartesian);
int getClosestWristToTheObject(p3d_rob* robot);
int getClosestWristToTheObject(p3d_rob* robot, p3d_rob* object);
int getClosestWristToTheObject(p3d_rob* robot, p3d_matrix4 objectPos);
#ifdef GRASP_PLANNING
#include "GraspPlanning-pkg.h"
int getBetterCollisionFreeGraspAndApproach(p3d_rob* robot, p3d_matrix4 objectPos, gpHand_type handType, p3d_matrix4 tAtt, configPt* graspConfig, configPt* approachConfig, gpGrasp * grasp);
int selectHandAndGetGraspApproachConfigs(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 tAtt, configPt* graspConfig, configPt* approachConfig, gpGrasp* grasp, int* whichArm, bool cartesian);
#endif

#endif
