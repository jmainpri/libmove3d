#ifndef LIGHTPLANNERAPI_H
#define LIGHTPLANNERAPI_H
#include "Planner-pkg.h"
#include "P3d-pkg.h"

/**
 * @breif small class that takes a matrix and extracts the rotation for each axes
 */
class p3d_objectPos {
  
public:
  p3d_objectPos() { };
  p3d_objectPos(p3d_matrix4 m) 
  {
    p3d_mat4ExtractPosReverseOrder(m, &_x, &_y, &_z, &_rx, &_ry, &_rz);
  };
  
  void setFromMatrix(p3d_matrix4 m)
  {
    p3d_mat4ExtractPosReverseOrder(m, &_x, &_y, &_z, &_rx, &_ry, &_rz);
  };
  
  void getMatrix(p3d_matrix4 m)
  {
    p3d_mat4PosReverseOrder( m, _x, _y, _z,
                            _rx, _ry, _rz);
  };  
  
  double _x;//! x the object x coordinate
  double _y; //! y the object y coordinate
  double _z; //! z the object z coordinate
  double _rx; //! rx the object rotation around x axis
  double _ry; //! ry the object rotation around y axis
  double _rz; //! rz the object rotation around z axis
};


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
