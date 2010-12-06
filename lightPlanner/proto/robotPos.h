#ifndef ROBOTPOS_H
#define ROBOTPOS_H
#include "P3d-pkg.h"

#include <map>

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
  
  double _x;//! x the object x coordinate
  double _y; //! y the object y coordinate
  double _z; //! z the object z coordinate
  double _rx; //! rx the object rotation around x axis
  double _ry; //! ry the object rotation around y axis
  double _rz; //! rz the object rotation around z axis
};

extern configPt p3d_getRobotBaseConfigAroundTheObject(p3d_rob* robot, p3d_jnt* baseJnt, p3d_jnt* objectJnt, p3d_objectPos& objPos, double minRadius, double maxRadius, int shootBase, int shootObjectPos, int shootObjectRot, int cntrtToActivate, bool nonUsedCntrtDesactivation, bool gaussianShoot = false);
extern configPt setBodyConfigForBaseMovement(p3d_rob * robot, configPt baseConfig, configPt bodyConfig);
extern void adaptClosedChainConfigToBasePos(p3d_rob *robot, p3d_matrix4 base, configPt refConf);
extern configPt setTwoArmsRobotGraspPosWithoutBase(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2, int shootObject, int cntrtToActivate, bool nonUsedCntrtDesactivation);
extern configPt setTwoArmsRobotGraspApproachPosWithoutBase(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2, int cntrtToActivate, bool nonUsedCntrtDesactivation);
extern configPt setTwoArmsRobotGraspApproachPosWithHold(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2, int cntrtToActivate);
extern configPt setTwoArmsRobotGraspPosWithHold(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2, int cntrtToActivate);
extern void setTwoArmsRobotGraspAndApproachPosWithHold(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2, configPt* graspConf, configPt* approachConf);
extern configPt setRobotGraspPosWithoutBase(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 tAtt, int shootObjectPos, int shootObjectRot, int armId, bool nonUsedCntrtDesactivation);
extern configPt setRobotCloseToConfGraspApproachOrExtract(p3d_rob* robot, configPt refConf, p3d_matrix4 objectPos, p3d_matrix4 tAtt, int shootObject, int armId, bool nonUsedCntrtDesactivation);
extern double computeRobotConfCostSpecificArm(p3d_rob* robot, configPt refConfig, configPt q, int whichArm);
extern double computeFreeArmsConfigCost(p3d_rob* robot, int armToActivate, configPt restConf, configPt conf);
extern double setRobotArmsRest(p3d_rob* robot, p3d_matrix4 objectPos, int armToActivate, p3d_matrix4 Tatt, configPt restConf, configPt conf);
extern double computeRobotConfCost(p3d_rob* robot, configPt q);
extern std::map<double, configPt, std::less<double> > * searchForLowCostNode(p3d_rob* robot, configPt startConfig, int whichArm);
extern double computeForwardCostSpecificArm(p3d_rob* robot, int whichArm);
extern void correctGraphForNewFixedJoints(p3d_graph* graph, configPt refConf, int nbJoints, p3d_jnt** joints);
extern void removeAloneNodesInGraph(p3d_rob* robot, p3d_graph* graph);
#ifdef GRASP_PLANNING
#include "GraspPlanning-pkg.h"
extern void correctGraphForHandsAndObject(p3d_rob* robot, p3d_graph* graph, int rightHandStatus, gpGrasp rightGrasp, int leftHandStatus, gpGrasp leftGrasp, bool carryobject, int whichArm);
extern double computeRobotGraspArmCost(p3d_rob* robot, int whichArm, gpGrasp grasp, configPt q, configPt refConfig, p3d_matrix4 objectPos);
#endif

#endif
