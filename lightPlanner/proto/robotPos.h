
#ifndef ROBOT_POS_H
#define ROBOT_POS_H
#include "P3d-pkg.h"

#include <map>
#include "lightPlannerApi.h"

//! @breif small class to go from eulers angls to matrix representation,
//! the p3d_matrix is extracted the rotation for each axes with convention (X,Y,Z)
class p3d_objectPos 
{  
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
    p3d_mat4PosReverseOrder( m, _x, _y, _z, _rx, _ry, _rz);
  };  
  
  double _x;//! x the object x coordinate
  double _y; //! y the object y coordinate
  double _z; //! z the object z coordinate
  double _rx; //! rx the object rotation around x axis
  double _ry; //! ry the object rotation around y axis
  double _rz; //! rz the object rotation around z axis
};  

void setMaxNumberOfTryForIK(int value);
int getMaxNumberOfTryForIK();
void setDebugConfAroundTheObject(bool value);
int getDebugConfAroundTheObject();

configPt setBodyConfigForBaseMovement(p3d_rob * robot, configPt baseConfig, configPt bodyConfig);
void adaptClosedChainConfigToBasePos(p3d_rob *robot, p3d_matrix4 base, configPt refConf);

#ifdef GRASP_PLANNING
#include "GraspPlanning-pkg.h"
double computeRobotConfCostSpecificArm(p3d_rob* robot, configPt refConfig, configPt q, int whichArm);
double computeFreeArmsConfigCost(p3d_rob* robot, int armToActivate, configPt restConf, configPt conf);
double computeRobotConfCost(p3d_rob* robot, configPt q);
double computeForwardCostSpecificArm(p3d_rob* robot, int whichArm);
double computeRobotGraspArmCost(p3d_rob* robot, int whichArm, gpGrasp grasp, configPt q, configPt refConfig, p3d_matrix4 objectPos);
double optimizeRedundentJointConfigCost(p3d_rob* robot, int redJntId, configPt q, p3d_matrix4 objectPos, p3d_matrix4 tAtt, gpGrasp& grasp, int armId, int nbTests);
double optimizeRedundentJointConfigDist(p3d_rob* robot, int redJntId, configPt q, p3d_matrix4 objectPos, p3d_matrix4 tAtt, configPt refConf, int armId, int nbTests);
#endif

double sampleRobotArmsRest(p3d_rob* robot, p3d_matrix4 objectPos, int armToActivate, p3d_matrix4 Tatt, configPt restConf, configPt conf);
void sampleTwoArmsRobotGraspAndApproachPosWithHold(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2, configPt* graspConf, configPt* approachConf);
configPt sampleTwoArmsRobotGraspApproachPosWithHold(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2, int cntrtToActivate);
configPt sampleTwoArmsRobotGraspPosWithHold(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2, int cntrtToActivate);
configPt sampleTwoArmsRobotGraspPosWithoutBase(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2, int shootObject, int cntrtToActivate, bool nonUsedCntrtDesactivation);
configPt sampleTwoArmsRobotGraspApproachPosWithoutBase(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2, int cntrtToActivate, bool nonUsedCntrtDesactivation);
configPt sampleRobotCloseToConfGraspApproachOrExtract(p3d_rob* robot, configPt refConf, p3d_matrix4 objectPos, p3d_matrix4 tAtt, int shootObject, int armId, bool nonUsedCntrtDesactivation);

configPt sampleRobotGraspPosWithBase(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 tAtt, int shootObjectPos, int shootObjectRot, int armId, bool nonUsedCntrtDesactivation);
configPt sampleRobotGraspPosWithoutBase(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 tAtt, int shootObjectPos, int shootObjectRot, int armId, bool nonUsedCntrtDesactivation);
#endif
