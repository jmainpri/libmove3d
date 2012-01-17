#include "Collision-pkg.h"
#include "P3d-pkg.h"
#include "Graphic-pkg.h"
#include "Localpath-pkg.h"

#include "robotPos.h"
#include "lightPlannerApi.h"
#include "lightPlanner.h"
#include "ManipulationUtils.hpp"
#include "ManipulationArmData.hpp"

#include "env.hpp"

using namespace std;

extern double SAFETY_DIST;
extern double APROACH_OFFSET;
extern double ROBOT_MAX_LENGTH;

//! @brief The maximal number of shoot to try before returning false
static int MaxNumberOfTry = 10000;
static int MaxNumberOfCollision = 10;

//! @brief Helps debugging configuration generation
static bool debugConfAroundTheObject = true;

void setMaxNumberOfTryForIK(int value)
{
  MaxNumberOfTry = value;
}

int getMaxNumberOfTryForIK()
{
  return  MaxNumberOfTry;
}

void setDebugConfAroundTheObject(bool value)
{
  debugConfAroundTheObject = value;
}

int getDebugConfAroundTheObject()
{
  return  debugConfAroundTheObject;
}

/**
 * @brief compute a configuration for the whole robot according the configuration of the base, the object and the rest of the robot. The configuration of the base and the object are given in the param baseConfig and the rest of the robot in the param bodyConfig.
 The base and object joints values are given during the initialisation (the p3d file here)
 * @param robot The robot
 * @param baseConfig The configuration of the base and the object
 * @param bodyConfig The configuration of the rest of the robot
 * @return The merged configuration
 */
configPt setBodyConfigForBaseMovement(p3d_rob * robot, configPt baseConfig, configPt bodyConfig){
  configPt conf = p3d_alloc_config(robot);
  for(int i = 0; i < robot->njoints + 1; i++){
    p3d_jnt * joint = robot->joints[i];
    for(int j = 0; j < joint->dof_equiv_nbr; j++){
      if(joint != robot->curObjectJnt && joint != robot->baseJnt){
        conf[joint->index_dof + j] = bodyConfig[joint->index_dof + j];
      }else{
        conf[joint->index_dof + j] = baseConfig[joint->index_dof + j];
      }
    }
  }
  return conf;
}

/**
 * @brief adapt the given base configuration to a the whole robot configuration. The configuration of the base and the object are given in the param base and the rest of the robot in the param refConf.
 The base and object joints values are given during the initialisation (the p3d file here)
 If the configuration using the brute method do not work (collision or do not respect the constraints), try to find a new configuration of the body satisfaying theses constraints.
 * @param robot The robot
 * @param base The configuration of the base and the object
 * @param refConf The configuration of the rest of the robot
 */
void adaptClosedChainConfigToBasePos(p3d_rob* robot, p3d_matrix4 base, configPt refConf) 
{
  p3d_matrix4 relMatrix, newObjectPos, basePos;
  p3d_objectPos objPos;
  p3d_mat4Copy(base, basePos);
  
  //On met le robot dans la configuration passee dans le P3D afin de trouver la matrice de transformation entre la base et l'objet.
  p3d_set_and_update_robot_conf(refConf);
  getObjectBaseAttachMatrix(robot->baseJnt->abs_pos, robot->curObjectJnt->abs_pos, relMatrix);
  
  //Pour la configuration courante de la base, la position de l'objet est base * relMatrix.
  p3d_mat4Mult(basePos, relMatrix, newObjectPos);
  objPos.setFromMatrix(newObjectPos);
  
  p3d_jnt_set_dof(robot->curObjectJnt, 0, objPos._x - robot->curObjectJnt->pos0[0][3]);
  p3d_jnt_set_dof(robot->curObjectJnt, 1, objPos._y - robot->curObjectJnt->pos0[1][3]);
  p3d_jnt_set_dof(robot->curObjectJnt, 2, objPos._z - robot->curObjectJnt->pos0[2][3]);
  p3d_jnt_set_dof(robot->curObjectJnt, 3, objPos._rx);
  p3d_jnt_set_dof(robot->curObjectJnt, 4, objPos._ry);
  p3d_jnt_set_dof(robot->curObjectJnt, 5, objPos._rz);
  
  //On change la configuration du joint de la base.
  objPos.setFromMatrix(basePos);
  
  p3d_jnt_set_dof(robot->baseJnt, 0, objPos._x);
  p3d_jnt_set_dof(robot->baseJnt, 1, objPos._y);
  p3d_jnt_set_dof(robot->baseJnt, 2, objPos._rz);
  
  //Test de validitee et collision.
  if (p3d_update_this_robot_pos_with_partial_reshoot(robot)) {
    if (!p3d_col_test()) {
      //Sauvegarde de la configuration.
      p3d_get_robot_config_into(robot, &refConf);
    }
  }else{
    configPt tmp = sampleTwoArmsRobotGraspPosWithoutBase(robot, robot->curObjectJnt->abs_pos, robot->ccCntrts[0]->Tatt, robot->ccCntrts[1]->Tatt,  FALSE, -1, true);
    if(tmp != NULL){
      //Sauvegarde de la configuration.
      p3d_copy_config_into(robot, tmp, &refConf);
      p3d_destroy_config(robot, tmp);
    }
  }
  p3d_get_robot_config_into(robot, &refConf);
}

#ifdef GRASP_PLANNING
double costConst = 1;
/**
 * @brief Optimize the given robot configuration using the redundent joint to get smarter configurations
 * @param robot the robot
 * @param redJntId the redundent joint
 * @param q the configuration to optimize
 * @param objectPos the object position matrix
 * @param tAtt the attach matrix
 * @param grasp the used grasp 
 * @param armId the arm grasping the object
 * @param nbTests the number of iterations
 * @return the better configuration cost
 */
double optimizeRedundentJointConfigCost(p3d_rob* robot, int redJntId, configPt q, p3d_matrix4 objectPos, p3d_matrix4 tAtt, gpGrasp& grasp, int armId, int nbTests)
{
  if(q){
    double refCost = computeRobotGraspArmCost(robot, armId, grasp, q , robot->openChainConf, objectPos)/costConst;
    double cost = P3D_HUGE, vmin = -P3D_HUGE, vmax = P3D_HUGE;
    //get the redundent joint from the id
    p3d_jnt* redJnt = robot->joints[redJntId];
    //Get the redundent joint bounds
    p3d_jnt_get_dof_bounds(redJnt, 0, &vmin, &vmax);
    //The index of the redundent joint in the configPt
    int qId = redJnt->index_dof;
    double refValue = q[qId], value = P3D_HUGE;
    p3d_matrix4 bak;
    
    activateCcCntrts(robot, armId, false);
    p3d_mat4Copy((*robot->armManipulationData)[armId].getCcCntrt()->Tatt, bak);
    p3d_mat4Copy(tAtt, (*robot->armManipulationData)[armId].getCcCntrt()->Tatt);
    
    for(int i = 0; i < nbTests; i++){
      q[qId] = p3d_random(vmin, vmax);
      if(p3d_is_collision_free(robot, q)){
        double tmpCost = computeRobotGraspArmCost(robot, armId, grasp, q , robot->openChainConf, objectPos)/costConst;
        if(tmpCost < cost){
          cost = tmpCost;
          value = q[qId];
        }
      }
    }
    if(refCost < cost){
      q[qId] = refValue;
      cost = refCost;
    }else{
      q[qId] = value;
    }
    p3d_set_and_update_this_robot_conf(robot, q);
    p3d_get_robot_config_into(robot, &q);
    deactivateCcCntrts(robot, armId);
    p3d_mat4Copy(bak, (*robot->armManipulationData)[armId].getCcCntrt()->Tatt);
    return cost;
  }
  return -1;
}

/**
 * @brief Optimize the given robot configuration using the redundent joint to get closer configuration to refConf
 * @param robot the robot
 * @param redJntId the redundent joint
 * @param q the configuration to optimize
 * @param objectPos the object position matrix
 * @param tAtt the attach matrix
 * @param refConf the reference config
 * @param armId the arm grasping the object
 * @param nbTests the number of iterations
 * @return the better configuration cost
 */
double optimizeRedundentJointConfigDist(p3d_rob* robot, int redJntId, configPt q, p3d_matrix4 objectPos, p3d_matrix4 tAtt, configPt refConf, int armId, int nbTests)
{
  if(q){
    double dist = P3D_HUGE, refDist = P3D_HUGE, vmin = -P3D_HUGE, vmax = P3D_HUGE;
    p3d_jnt* redJnt = robot->joints[redJntId];
    p3d_jnt_get_dof_bounds(redJnt, 0, &vmin, &vmax);
    int qId = redJnt->index_dof; //The index of the redundent joint in the configPt
    double refValue = q[qId], value = P3D_HUGE;
    p3d_matrix4 bak;
    activateCcCntrts(robot, armId, false);
    p3d_mat4Copy((*robot->armManipulationData)[armId].getCcCntrt()->Tatt, bak);
    p3d_mat4Copy(tAtt, (*robot->armManipulationData)[armId].getCcCntrt()->Tatt);   
    q[qId] = refConf[qId];
    if(!p3d_is_collision_free(robot, q) || !p3d_connectable_confs(robot, refConf, q, &dist)){
      refDist = dist;
      for(int i = 0; i < nbTests; i++){
        q[qId] = p3d_random(vmin, vmax);
        double tmpDist = P3D_HUGE;
        if(p3d_is_collision_free(robot, q) && p3d_connectable_confs(robot, refConf, q, &tmpDist)){
          if(tmpDist < dist){
            dist = tmpDist;
            //             printf("Dist = %f\n", dist);
            value = q[qId];
          }
        }
      }
      if(refDist <= dist){
        q[qId] = refValue;
        dist = refDist;
      }else{
        q[qId] = value;
      }
    }
    //     printf("Selected Dist = %f\n", dist);
    p3d_set_and_update_this_robot_conf(robot, q);
    p3d_get_robot_config_into(robot, &q);
    deactivateCcCntrts(robot, armId);
    p3d_mat4Copy(bak, (*robot->armManipulationData)[armId].getCcCntrt()->Tatt);
    if(dist == P3D_HUGE){
      return -2;
    }
    return dist;
  }
  return -1;
}

#endif

//! @brief Compute the cost between the configuration q and a The RefConfig. 
//! This cost is computed using the joint distance of the selected arm between the two configs. 
//! If the arm is KUKA LWR a potential cost is added to avoid finding configuration too high 
//!  from the rest configuration. It is preferable to have normalized costs. For LWR the costs are normalized
//! @param robot the robot
//! @param q the configuration to compute the cost
//! @param whichArm The selected arm. This correspond to the Id of the arm Ik constraint stored in robot->ccCntrts
//! @return the configuration cost. For LWR 0 < cost < 1
double computeRobotConfCostSpecificArm(p3d_rob* robot, configPt refConfig, configPt q, int whichArm)
{
  double armMediumJointCost = 0;
  p3d_cntrt* ct = robot->ccCntrts[whichArm];
  //MediumJointCost
  for(int j = 0; j < ct->npasjnts; j++){
    armMediumJointCost += SQR(q[ct->pasjnts[j]->index_dof] - refConfig[ct->pasjnts[j]->index_dof]);
  }
  if(!strcmp(ct->namecntrt, "p3d_kuka_arm_ik")){
    //normalize ArmMediumJointCost
    armMediumJointCost /= 50;
    //PotentialCost
    static double refHeight[3] = {P3D_HUGE, P3D_HUGE, P3D_HUGE};
    if(refHeight[0] == P3D_HUGE){
      p3d_set_and_update_this_robot_conf_without_cntrt(robot, refConfig);
      refHeight[0] = ct->pasjnts[0]->abs_pos[2][3];
      refHeight[1] = ct->pasjnts[2]->abs_pos[2][3];
      refHeight[2] = ct->pasjnts[4]->abs_pos[2][3];
      p3d_set_and_update_this_robot_conf(robot, q);
    }
    double armPotentialCost = 0;
    double configHeight[3] = {ct->pasjnts[0]->abs_pos[2][3], ct->pasjnts[2]->abs_pos[2][3], ct->pasjnts[4]->abs_pos[2][3]};
    armPotentialCost = ABS((configHeight[1] - configHeight[0]) - (refHeight[1] - refHeight[0])) + ABS((configHeight[2] -configHeight[0]) - (refHeight[2] - refHeight[0]));
    //normalize ArmPotentialCost
    armPotentialCost /= 1.94;
    if (robot->nbCcCntrts == 2) {
      //ForwardCost
      double forwardCost = computeForwardCostSpecificArm(robot, whichArm);
      if (forwardCost < 0) {
        forwardCost = -forwardCost;
        forwardCost = forwardCost < 0.5 ? 1 - forwardCost : forwardCost;
        return (armMediumJointCost + armPotentialCost + forwardCost) / 3;
      }else {
        return (armMediumJointCost + armPotentialCost) / 2;
      }
    }else {
      return (armMediumJointCost + armPotentialCost) / 2;
    }
  }else{
    return armMediumJointCost;
  }
}

//!  @brief Compute the cost between the configuration q and a previously declared configuration
//!  stored in robot->openChainConf. This cost is computed using the joint distance of the robot arms
//!  between the two configs.
double computeFreeArmsConfigCost(p3d_rob* robot, int armToActivate, configPt restConf, configPt conf)
{
  double minconfCost = 0;
  
  for(int i = 0; i < (int)robot->armManipulationData->size(); i++){
    if(i != armToActivate){
      minconfCost += computeRobotConfCostSpecificArm(robot, restConf, conf, i);
    }
  }
  return minconfCost;
}

//!  @brief Compute the cost between the configuration q and a previously declared configuration
//!  stored in robot->openChainConf. This cost is computed using the joint distance of the robot arms
//!  between the two configs.
//! @param robot the robot
//! @param q the configuration to compute the cost
//! @return the configuration cost. For LWR 0 < cost < 1
double computeRobotConfCost(p3d_rob* robot, configPt q)
{
  //Check the cost for each arm then combine the two costs
  double armCost = 0;
  for(int i = 0; i < robot->nbCcCntrts; i++){
    armCost += computeRobotConfCostSpecificArm(robot, robot->openChainConf, q, i);
  }
  static int baseDofId = 0;//only the rotation around Z is taken into account
  if(baseDofId == 0){
    int baseJntId = robot->baseJnt->num != 0 ? robot->baseJnt->num : 1;
    if(robot->joints[baseJntId]->type == P3D_ROTATE){
      baseDofId = robot->joints[baseJntId]->index_dof;
    }else if(robot->joints[baseJntId]->type == P3D_PLAN){
      baseDofId = robot->joints[baseJntId]->index_dof + 2;
    }else if(robot->joints[baseJntId]->type == P3D_FREEFLYER){
      baseDofId = robot->joints[baseJntId]->index_dof + 5;
    }else{
      printf("Error: Not supported base type in light planner Cost\n");
    }
  }
  //  double baseCost = SQR(q[baseDofId] - robot->openChainConf[baseDofId]) / 12.1848;
  //   return (armCost / robot->nbCcCntrts + baseCost) / 2;
  return armCost/ robot->nbCcCntrts;
}

#ifdef GRASP_PLANNING
double computeRobotGraspArmCost(p3d_rob* robot, int whichArm, gpGrasp grasp, configPt q, configPt refConfig, p3d_matrix4 objectPos)
{
  double armMediumJointCost = 0.0, thumbDirectionCost = 0.0, mediusDirectionCost = 0.0, distConf = 0.0;
  p3d_cntrt* ct = (*robot->armManipulationData)[whichArm].getCcCntrt();
  p3d_jnt* baseJnt = robot->baseJnt;
  int weight = 1;
  p3d_set_and_update_this_robot_conf(robot, q);
  
  //MediumJointCost Max 270
  for(int j = 0; j < ct->npasjnts; j++){
    if((!strcmp(ct->namecntrt, "p3d_kuka_arm_ik") || !strcmp(ct->namecntrt, "p3d_lwr_arm_ik")) && j >= 2){
      weight = (ct->npasjnts - j);
    }else{
      weight = (ct->npasjnts - j + 1);
    }
    armMediumJointCost += weight * SQR(q[ct->pasjnts[j]->index_dof] - refConfig[ct->pasjnts[j]->index_dof]);
  }
  if(!strcmp(ct->namecntrt, "p3d_kuka_arm_ik") || !strcmp(ct->namecntrt, "p3d_lwr_arm_ik")){
    armMediumJointCost += 5 * SQR(q[robot->joints[ct->argu_i[0]]->index_dof] - refConfig[robot->joints[ct->argu_i[0]]->index_dof]);
  }
  
  distConf = p3d_dist_config(robot, q, refConfig);
  
  /* Base orientation cost (Better is on the refConf base/obj line)*/  //!!!!!Not Working
  //   p3d_set_and_update_this_robot_conf(robot, refConfig);
  //   p3d_vector3 obj, current, ref, objCurrent, refCurrent;
  //   p3d_mat4ExtractColumnX(robot->baseJnt->abs_pos, ref);
  // 
  //   obj[0] = objectPos[0][3] - robot->baseJnt->abs_pos[0][3];
  //   obj[1] = objectPos[1][3] - robot->baseJnt->abs_pos[1][3];
  //   obj[2] = objectPos[2][3] - robot->baseJnt->abs_pos[2][3];
  // 
  //   p3d_set_and_update_this_robot_conf(robot, q);
  //   p3d_mat4ExtractColumnX(robot->baseJnt->abs_pos, current);
  // 
  //   p3d_vectXprod(obj, current, objCurrent);
  //   p3d_vectXprod(ref, current, refCurrent);
  // 
  //   if(p3d_vectDotProd(objCurrent,refCurrent) <= 0){
  //     weight = 5;
  //   }else{
  //     weight = 10;
  //   }
  //   armMediumJointCost += weight* SQR(q[baseJnt->index_dof] - refConfig[baseJnt->index_dof]);
  
  
  // Get the thumb in base direction. Max 1.5
  if((!strcmp(ct->namecntrt, "p3d_kuka_arm_ik") || !strcmp(ct->namecntrt, "p3d_lwr_arm_ik")) ){
    p3d_vector3 frontDir, leftDir, upDir = {0,0,1};
    frontDir[0] = objectPos[0][3] - robot->baseJnt->abs_pos[0][3];
    frontDir[1] = objectPos[1][3] - robot->baseJnt->abs_pos[1][3];
    frontDir[2] = objectPos[2][3] - robot->baseJnt->abs_pos[2][3];
    p3d_matrix4 wristPose;
    p3d_mat4Copy(ct->pasjnts[ct->npasjnts -1]->abs_pos , wristPose);
    p3d_vector3 thumbDirection, mediusDirection;
    
    p3d_mat4ExtractColumnX(wristPose, thumbDirection);
    if(!strcmp(ct->namecntrt, "p3d_kuka_arm_ik") ){
      p3d_mat4ExtractColumnY(wristPose, mediusDirection);
    }else if(!strcmp(ct->namecntrt, "p3d_lwr_arm_ik")){
      p3d_mat4ExtractColumnZ(wristPose, mediusDirection);
    }
    p3d_vectNormalize(frontDir, frontDir);
    p3d_vectXprod(upDir, frontDir, leftDir);
    
    thumbDirectionCost = 1 - p3d_vectDotProd(leftDir, thumbDirection);
    mediusDirectionCost = 1 - p3d_vectDotProd(frontDir, mediusDirection);
  }
  
  return (/*armMediumJointCost/270 + */(thumbDirectionCost + mediusDirectionCost)/4 + distConf/16)/2;
}
#endif

//! @brief Compute the forward cost of the specified arm. This function works only with two arms. 
//! This cost give negative cost for arm configurations when the wrist is behind the torso. 
//! LIMITATION : Its works only with two arms
//! @param robot the robot
//! @param whichArm the arm to compute the cost
//! @return the arm forward cost
double computeForwardCostSpecificArm(p3d_rob* robot, int whichArm)
{
  p3d_vector3 rightShoulder, leftShoulder, base, wrist;
  p3d_mat4ExtractTrans((*robot->armManipulationData)[0].getCcCntrt()->pasjnts[0]->abs_pos, rightShoulder);
  p3d_mat4ExtractTrans((*robot->armManipulationData)[1].getCcCntrt()->pasjnts[0]->abs_pos, leftShoulder);
  p3d_jnt* baseJnt = robot->baseJnt;
  if (baseJnt || (baseJnt && baseJnt->num == 0)) {
    baseJnt = robot->joints[1];
  }
  p3d_mat4ExtractTrans(baseJnt->abs_pos, base);
  p3d_plane plane = p3d_plane_from_points(leftShoulder, rightShoulder, base);
  //wrist Position
  p3d_mat4ExtractTrans(robot->ccCntrts[whichArm]->pasjnts[robot->ccCntrts[whichArm]->npasjnts - 1]->abs_pos, wrist);
  return (p3d_vectDotProd(plane.normale, wrist) + plane.d)/0.79;
}

//! @brief Sample the base joint in a semi circle
//! @param robot the robot
//! @param minRadius the minimum radius to shoot the base
//! @param maxRadius the maximum radius to shoot the base
//! @return the configuration
void sampleBaseJoint(p3d_rob* robot, p3d_jnt* baseJnt, double minRadius, double maxRadius, p3d_objectPos& objectPos, configPt q)
{
  if( baseJnt->type != P3D_PLAN && baseJnt->type != P3D_FREEFLYER )
  {
    cout << "Error in " << __func__ << endl;
    return;
  }
  
  int index_dof_rz;
  
  if( baseJnt->type == P3D_PLAN )
    index_dof_rz = 2;
  else
    index_dof_rz = 5;
  
  const bool allAround = true;
  
  double x = objectPos._x;
  double y = objectPos._y;
  double rz = objectPos._rz;
  
  double randX, randY, randRZ;
  
  if(allAround)
  {
    const double margin = 0.10;
   
    // Samples a point within a croun
    double l =  p3d_random( minRadius , maxRadius );
    double r =  p3d_random( 0 , 2*M_PI );
    randX = l*cos(r);
    randY = l*sin(r);
    
    // Orientation is chosen toward the object
    randRZ = p3d_random( angle_limit_2PI(M_PI+r-margin) , 
                         angle_limit_2PI(M_PI+r+margin) );
    
    q[baseJnt->index_dof + 0] = x + randX;
    q[baseJnt->index_dof + 1] = y + randY;
    q[baseJnt->index_dof + index_dof_rz] = angle_limit_PI(randRZ);
  }
  else
  {
    const double nominalRadius = 0.17; // 10 Deg
    
    randX =  p3d_random( minRadius , maxRadius );
    randY =  p3d_random(-(maxRadius * tan(nominalRadius)), (maxRadius * tan(nominalRadius)));
    randRZ = p3d_random(rz + robot->relativeZRotationBaseObject - nominalRadius, 
                        rz + robot->relativeZRotationBaseObject + nominalRadius);
    
    q[baseJnt->index_dof + 0] = x - cos(rz)*(randX) + sin(rz)*(randY);
    q[baseJnt->index_dof + 1] = y - sin(rz)*(randX) + cos(rz)*(randY);
    q[baseJnt->index_dof + index_dof_rz] = randRZ;
  }

  // Check X,Y DoFs limits
  if(baseJnt->dof_data[0].vmin > q[baseJnt->index_dof]){
    q[baseJnt->index_dof] = baseJnt->dof_data[0].vmin;
  }
  if(baseJnt->dof_data[0].vmax < q[baseJnt->index_dof]){
    q[baseJnt->index_dof] = baseJnt->dof_data[0].vmax;
  }
  if(baseJnt->dof_data[1].vmin > q[baseJnt->index_dof + 1]){
    q[baseJnt->index_dof + 1] = baseJnt->dof_data[1].vmin;
  }
  if(baseJnt->dof_data[1].vmax < q[baseJnt->index_dof + 1]){
    q[baseJnt->index_dof + 1] = baseJnt->dof_data[1].vmax;
  }
}

//! When the robot is carrying an object
//! the constraint is set active and if specified
//! the carried-object robot reference configuration is set to the argument parameters
//! and it loops until a configuration is found collision free for the object
//! @param objectPos the object translation parameter for each axis
//! @return q the robot configuration
void sampleObjectConfiguration( p3d_rob* robot, p3d_objectPos& objPos, int cntrtToActivate, configPt q, int shootObjectPos, int shootObjectRot)
{
  double robotSize = 0;
  double translationFactor = 0;
  double rotationFactor = 0;
  int nbTry = 0, objIdx = 6;
  
  if(cntrtToActivate == -1){
    cntrtToActivate = 0;
  }
  
  p3d_rob* carriedObject = (*robot->armManipulationData)[cntrtToActivate].getCarriedObject();
  if(!carriedObject){
    carriedObject = robot;
    objIdx = (*robot->armManipulationData)[cntrtToActivate].getManipulationJnt()->index_dof;
  }
  configPt carriedObjectRefConf = p3d_alloc_config(carriedObject);
  configPt carriedObjectConf = p3d_get_robot_config(carriedObject);
  
  carriedObjectRefConf[objIdx + 0] = objPos._x;
  carriedObjectRefConf[objIdx + 1] = objPos._y;
  carriedObjectRefConf[objIdx + 2] = objPos._z;
  carriedObjectRefConf[objIdx + 3] = objPos._rx;
  carriedObjectRefConf[objIdx + 4] = objPos._ry;
  carriedObjectRefConf[objIdx + 5] = objPos._rz;
  
  p3d_sel_desc_num(P3D_ROBOT,carriedObject->num);
  p3d_set_and_update_this_robot_conf(carriedObject, carriedObjectConf);
  p3d_get_BB_rob_max_size(carriedObject, &robotSize);
  if(robotSize >= P3D_HUGE || robotSize <= -P3D_HUGE){
    printf("Warning in %s !! BB size of %s is not well intialized. Value : %f. Setting the size to 1\n", __func__, carriedObject->name, robotSize);
    robotSize = 1;
  }
  translationFactor = robotSize/5;
  rotationFactor = 1;
  
  do{
    // This loop continues 
    // until a configuration is found collision free for the carried object
//     g3d_draw_allwin_active();
    if(carriedObject){
      p3d_gaussian_config2_specific(carriedObject, carriedObjectRefConf, carriedObjectConf, translationFactor, rotationFactor, true);
    }else{
      p3d_gaussian_config2_Joint_specific(robot, (*robot->armManipulationData)[cntrtToActivate].getManipulationJnt(), carriedObjectRefConf, carriedObjectConf, translationFactor, rotationFactor, true);
    }
    //Bit computation to get the positions and rotations to sample.
    //When the Bit = 1 the corresponding position or rotation has to be sampled
    if(~shootObjectPos & 1){
      carriedObjectConf[objIdx + 0]  = objPos._x;
    }
    if(~shootObjectPos & 2){
      carriedObjectConf[objIdx + 1]  = objPos._y;
    }
    if(~shootObjectPos & 4){
      carriedObjectConf[objIdx + 2]  = objPos._z;
    }
    if(~shootObjectRot & 1){
      carriedObjectConf[objIdx + 3]  = objPos._rx;
    }
    if(~shootObjectRot & 2){
      carriedObjectConf[objIdx + 4] = objPos._ry;
    }
    if(~shootObjectRot & 4){
      carriedObjectConf[objIdx + 5] = objPos._rz;
    }
    nbTry++;
  }while(nbTry < MaxNumberOfTry/100 && !p3d_set_and_update_this_robot_conf_with_partial_reshoot(carriedObject, carriedObjectConf) && p3d_col_test());
  
  p3d_sel_desc_num(P3D_ROBOT,robot->num);
  
  int ffjntIndex = (*robot->armManipulationData)[cntrtToActivate].getManipulationJnt()->index_dof;
  
  q[ffjntIndex] = carriedObjectConf[objIdx];
  q[ffjntIndex + 1] = carriedObjectConf[objIdx + 1];
  q[ffjntIndex + 2] = carriedObjectConf[objIdx + 2];
  q[ffjntIndex + 3] = carriedObjectConf[objIdx + 3];
  q[ffjntIndex + 4] = carriedObjectConf[objIdx + 4];
  q[ffjntIndex + 5] = carriedObjectConf[objIdx + 5];
  
  p3d_destroy_config(carriedObject, carriedObjectRefConf);
  p3d_destroy_config(carriedObject, carriedObjectConf);
}

//! @brief Computes the Min and Max Base sampling radius
//! If not given in input compute the radius for base sampling
//! Given the Bounding box of the base and the object
//! @param robot the robot
//! @param baseJnt the robot base jnt
//! @param objectJnt the object jnt
//! @param minRadius the minimum radius to shoot the base. If = -1 compute it automatically
//! @param maxRadius the maximum radius to shoot the base. If = -1 compute it automatically
void setBaseMinAndMaxSamplingRadius(p3d_rob* robot, p3d_jnt* baseJnt, p3d_jnt* objectJnt,int cntrtToActivate,double& minRadius, double& maxRadius)
{
  if(maxRadius == -1){
    if(!robot->isCarryingObject && objectJnt->o){
      maxRadius = MAX(baseJnt->o->BB0.xmax - baseJnt->o->BB0.xmin, 
                      baseJnt->o->BB0.ymax - baseJnt->o->BB0.ymin) + 
                  MAX(objectJnt->o->BB0.xmax - objectJnt->o->BB0.xmin, 
                      objectJnt->o->BB0.ymax - objectJnt->o->BB0.ymin) / 2;
    }else{
      if(!(*robot->armManipulationData)[cntrtToActivate].getCarriedObject()){
        printf("sampleRobotConfigAroundTheObject : Error, No object loaded");
      }else{
        maxRadius = MAX(baseJnt->o->BB0.xmax - baseJnt->o->BB0.xmin, 
                        baseJnt->o->BB0.ymax - baseJnt->o->BB0.ymin) + 
        MAX(objectJnt->o->BB0.xmax - objectJnt->o->BB0.xmin, 
            objectJnt->o->BB0.ymax - objectJnt->o->BB0.ymin) / 2;
      }
    }
  }
  if(minRadius == -1){
    if(!robot->isCarryingObject && objectJnt->o){
      minRadius = MAX(objectJnt->o->BB0.xmax - objectJnt->o->BB0.xmin, 
                      objectJnt->o->BB0.ymax - objectJnt->o->BB0.ymin) / 2;
    }else{
      if(!(*robot->armManipulationData)[cntrtToActivate].getCarriedObject()){
        printf("sampleRobotConfigAroundTheObject : Error, No object loaded");
      }else{
        minRadius = MAX(objectJnt->o->BB0.xmax - objectJnt->o->BB0.xmin, 
                        objectJnt->o->BB0.ymax - objectJnt->o->BB0.ymin) / 2;
      }
    }
  }
  
  if( debugConfAroundTheObject )
  {
    cout << "shoot base : " << minRadius << " , " << maxRadius << endl;
    cout << "objectJnt->name : " << objectJnt->name << endl;
    cout << "objectJnt->o->BB0.xmax : " << objectJnt->o->BB0.xmax << endl;
    cout << "objectJnt->o->BB0.xmin : " << objectJnt->o->BB0.xmin << endl;
    cout << "objectJnt->o->BB0.ymax : " << objectJnt->o->BB0.ymax << endl;
    cout << "objectJnt->o->BB0.ymin : " << objectJnt->o->BB0.ymin << endl;
    cout << "baseJnt->name : " << baseJnt->name << endl;
    cout << "baseJnt->o->BB0.xmax : " << baseJnt->o->BB0.xmax << endl;
    cout << "baseJnt->o->BB0.xmin : " << baseJnt->o->BB0.xmin << endl;
    cout << "baseJnt->o->BB0.ymax : " << baseJnt->o->BB0.ymax << endl;
    cout << "baseJnt->o->BB0.ymin : " << baseJnt->o->BB0.ymin << endl;
  }
}

//! @brief Function for sampling a valid robot configuration given an object position. 
//! We assume that the center of the object is the center of object Joint.
//! @param robot the robot
//! @param baseJnt the robot base jnt
//! @param objectJnt the object jnt
//! @param objectPos the object translation parameter for each axis
//! @param minRadius the minimum radius to shoot the base. If = -1 compute it automatically
//! @param maxRadius the maximum radius to shoot the base. If = -1 compute it automatically
//! @param shootBase Shoot the base if = 1. Shoot all exept the base when = 0
//! @param cntrtToActivate what arm to sample. if -1 all arms are sampled
//! @param nonUsedCntrtDesactivation when one arm is active make the other active or not
//! @return the robot config or NULL if fail
configPt sampleRobotConfigAroundTheObject(p3d_rob* robot, p3d_jnt* baseJnt, p3d_jnt* objectJnt, p3d_objectPos& objPos, 
                                          double minRadius, double maxRadius, 
                                          int shootBase, int shootObjectPos, int shootObjectRot, int cntrtToActivate, 
                                          bool nonUsedCntrtDesactivation , bool gaussianShoot)
{
  configPt q = NULL;
  
  // Safe test that robot, oject and base exist
  if(robot && objectJnt && baseJnt)
  {
    q = p3d_alloc_config(robot);
    configPt qInit = p3d_get_robot_config(robot);
    
    // Computes the Min and Max sampling raduis of the base joint
    // According to bounding boxes radius
    setBaseMinAndMaxSamplingRadius(robot, baseJnt, objectJnt, cntrtToActivate, minRadius, maxRadius);
    
    // Activate arms constraints
    activateCcCntrts(robot, cntrtToActivate, nonUsedCntrtDesactivation);
    
    int nbTry = 0;
    int nbTryColliding = 0;
    bool collision = false;
    bool isKukaBoundOff = false;
    do { 
      // Continues until there is a 
      // collision free configuration
      //if (nbTry != 0) { //there is a collision
      //  nbTry += MaxNumberOfTry*5/100;
      //}
      do {
//        if ( debugConfAroundTheObject ) {
//          g3d_draw_allwin_active();
//        }
        
        // Continues until a configuration
        // that respects kinematic constraints is found
        if(!gaussianShoot)
        {
          p3d_shoot(robot, q, 0);
        }
        else 
        {
          double robotSize = 0;
          double translationFactor = 0;
          double rotationFactor = 0;
          p3d_get_BB_rob_max_size(robot, &robotSize);
          translationFactor = robotSize/5;
          rotationFactor = robotSize/2;
          p3d_gaussian_config2_specific(robot, qInit, q , translationFactor, rotationFactor, true);
        }
        
        if(shootBase == true)
        {
          // Sample the base joint within a circle
          // of minRadius and maxRadius
          sampleBaseJoint(robot, baseJnt, minRadius, maxRadius, objPos, q);
          
          if ( debugConfAroundTheObject ) {
            cout << "baseJnt->index_dof = " << baseJnt->index_dof << endl;
            for(int i=0; i<baseJnt->dof_equiv_nbr; i++) {
              cout << "q[baseJnt->index_dof+" <<i<< "] = " << q[baseJnt->index_dof+i] << endl;
            }
          }
        }
        else if (baseJnt->type != P3D_ROTATE)
        {
          for(int i = 0; i<baseJnt->dof_equiv_nbr; i++)
          {
            q[baseJnt->index_dof+i] = qInit[baseJnt->index_dof+i];
          }
        }
        
        if(shootObjectPos || shootObjectRot)
        {
          sampleObjectConfiguration(robot,objPos,cntrtToActivate,q, shootObjectPos, shootObjectRot);
        }
        else
        {
          // Gets the free flyer DoFs of the robot
          int ffjntIndex = (*robot->armManipulationData)[cntrtToActivate].getManipulationJnt()->index_dof;
          
          // When not sampling the object confguration
          // the free flyer DoFs are set to the argument parameters (objPos)
          q[ffjntIndex + 0] = objPos._x;
          q[ffjntIndex + 1] = objPos._y;
          q[ffjntIndex + 2] = objPos._z;
          q[ffjntIndex + 3] = objPos._rx;
          q[ffjntIndex + 4] = objPos._ry;
          q[ffjntIndex + 5] = objPos._rz;
        }
        nbTry++;
	//printf("NbTry : %d and Max %d\n", nbTry, MaxNumberOfTry );
        // The update function re-sample the robot configuration
        // to get a valid configuration regarding the kinematic constraints
      } while (!p3d_set_and_update_this_robot_conf_with_partial_reshoot(robot, q) && (nbTry < MaxNumberOfTry));
  
      nbTryColliding++;
      collision = p3d_col_test();
      
      if ( debugConfAroundTheObject ) {
        //g3d_draw_allwin_active();
        if( collision ) {
	  p3d_print_col_pair();
        }
      }
    } while ( collision && ( nbTry < MaxNumberOfCollision ) );
    
    if(nbTry >= MaxNumberOfTry)
    {
      if (debugConfAroundTheObject) {
        cout << "NbTry = " << nbTry << endl;
        cout << "nbTryColliding = " << nbTryColliding << endl;
      }
      p3d_destroy_config(robot, qInit);
      p3d_destroy_config(robot, q);
      deactivateCcCntrts(robot, cntrtToActivate);
      return NULL;
    }
    
    p3d_get_robot_config_into( robot, &q );
    p3d_destroy_config(robot, qInit);
    deactivateCcCntrts(robot, cntrtToActivate);
  }
  return q;
}

//! @brief Get the robot grasp configuration given the attach matrix
//! @param robot the robot
//! @param objectPos the object position matrix
//! @param att the attach matrix. The number of matrix is robot->nbCcCntrts
//! @return the robot config
static configPt sampleRobotGraspConf(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 *att, 
                                     int shootBase, int shootObjectPosition, int shootObjectRotation, 
                                     int cntrtToActivate, bool nonUsedCntrtDesactivation, bool gaussianShoot) 
{
  configPt q = NULL;
  p3d_objectPos objPos(objectPos);
  
  // Backup and set the attach matrix
  p3d_matrix4 bakTatt[(int)robot->armManipulationData->size()];
	
  for (int i = 0; i < (int)robot->armManipulationData->size(); i++) 
  {
    p3d_mat4Copy((*robot->armManipulationData)[i].getCcCntrt()->Tatt, bakTatt[i]);
    p3d_mat4Copy(att[i], (*robot->armManipulationData)[i].getCcCntrt()->Tatt);
  }
  
  // Compute the min and max values of the base circle
  // when sampling the base joint (fifty centimeters
  p3d_jnt* baseJnt = robot->baseJnt;
  double maxRadius = -1;
  double minRadius = -1;
  
  if( shootBase )
  {
    // Two times the size of the base
    maxRadius = std::max(baseJnt->o->BB0.xmax - baseJnt->o->BB0.xmin, 
                         baseJnt->o->BB0.ymax - baseJnt->o->BB0.ymin) * 1.5;
    
    // Half the size of the base
    minRadius = std::max(baseJnt->o->BB0.xmax - baseJnt->o->BB0.xmin, 
                         baseJnt->o->BB0.ymax - baseJnt->o->BB0.ymin) / 2;
  }
    
  // Main function to sample a robot configuration 
	q = sampleRobotConfigAroundTheObject(robot, baseJnt, robot->curObjectJnt, objPos, 
                                       minRadius, maxRadius, 
                                       shootBase, shootObjectPosition, shootObjectRotation, 
                                       cntrtToActivate, nonUsedCntrtDesactivation, gaussianShoot);
  
	// Restore the attach matrix
  for (int i = 0; i < (int)robot->armManipulationData->size(); i++) 
  {
    p3d_mat4Copy(bakTatt[i], (*robot->armManipulationData)[i].getCcCntrt()->Tatt);
  }
  return q;
}

//! @brief Get the robot grasp approach an the robot grasp configuration given an attach matrix
//! for each arm and the object position. Verify also the hold configuration for the transfert
//! @param robot the robot
//! @param objectPos the object position matrix
//! @param att1 the attach matrix for the first arm
//! @param att2 the attach matrix for the second arm
//! @param graspConf the retruned grasp config of the robot
//! @param approachConf the retruned approach config of the robot
void sampleTwoArmsRobotGraspAndApproachPosWithHold(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2, configPt* graspConf, configPt* approachConf) 
{
  if (robot->nbCcCntrts > 2) {
    printf("There is more than 2 arms\n");
    return;
  }

  p3d_matrix4 * att = MY_ALLOC(p3d_matrix4, 2);
  p3d_mat4Copy(att1, att[0]);
  p3d_mat4Copy(att2, att[1]);
  switchBBActivationForGrasp();
  do{
    do{
      p3d_col_activate_obj_env(robot->curObjectJnt->o);
      setSafetyDistance(robot, 0);
      *graspConf = sampleRobotGraspConf(robot, objectPos, att, TRUE, FALSE, FALSE, -1, true, false);
      if(graspConf == NULL){
        return;
      }
      setSafetyDistance(robot, (double)SAFETY_DIST);
      p3d_col_deactivate_obj_env(robot->curObjectJnt->o);
      deactivateCcCntrts(robot, -1);
      configPt conf = setBodyConfigForBaseMovement(robot, *graspConf, robot->openChainConf);
      p3d_set_and_update_robot_conf(conf);
      p3d_destroy_config(robot, conf);
    } while (p3d_col_test());
    
    p3d_col_activate_obj_env(robot->curObjectJnt->o);
    configPt adaptedConf = p3d_copy_config(robot, robot->closedChainConf);
    adaptClosedChainConfigToBasePos(robot, robot->baseJnt->abs_pos, adaptedConf);
    p3d_set_and_update_robot_conf(adaptedConf);
    p3d_destroy_config(robot, adaptedConf);
  } while (p3d_col_test());
//  p3d_col_activate_obj_env(robot->curObjectJnt->o);
  setSafetyDistance(robot, 0);
  /*Shift attach position over wrist X axis*/
  att[0][1][3] += -APROACH_OFFSET;
  att[1][1][3] += APROACH_OFFSET;
  p3d_set_and_update_robot_conf(*graspConf);
  *approachConf = sampleRobotGraspConf(robot, objectPos, att, FALSE, FALSE, FALSE, -1, true, false);
  MY_FREE(att, p3d_matrix4, 2);
  switchBBActivationForGrasp();
  return;
}

//! @brief Get the robot grasp approach configuration given an attach matrix for each 
//! arm and the object position. Verify also the hold configuration for the transfert
//! @param robot the robot
//! @param objectPos the object position matrix
//! @param att1 the attach matrix for the first arm
//! @param att2 the attach matrix for the second arm
//! @return the robot config
configPt sampleTwoArmsRobotGraspApproachPosWithHold(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2, int cntrtToActivate) 
{
  if (robot->nbCcCntrts > 2) 
  {
    printf("There is more than 2 arms\n");
    return NULL;
  }
  /*Shift attach position over wrist X axis*/
  p3d_matrix4 * att = MY_ALLOC(p3d_matrix4, 2);
  p3d_mat4Copy(att1, att[0]);
  p3d_mat4Copy(att2, att[1]);
//  att[0][1][3] += -APROACH_OFFSET;
//  att[1][1][3] += APROACH_OFFSET;
  configPt q;
//  switchBBActivationForGrasp();
  do{
    do{
      p3d_col_activate_obj_env(robot->curObjectJnt->o);
      setSafetyDistance(robot, 0);
      q = sampleRobotGraspConf(robot, objectPos, att, TRUE, FALSE, FALSE, cntrtToActivate, true, false);
      if(q == NULL){
        return NULL;
      }
      setSafetyDistance(robot, (double)SAFETY_DIST);
      p3d_col_deactivate_obj_env(robot->curObjectJnt->o);
      deactivateCcCntrts(robot, -1);
      configPt conf = setBodyConfigForBaseMovement(robot, q, robot->openChainConf);
      p3d_set_and_update_robot_conf(conf);
      p3d_destroy_config(robot, conf);
    }while (p3d_col_test());
    
    p3d_col_activate_obj_env(robot->curObjectJnt->o);
    configPt adaptedConf = p3d_copy_config(robot, robot->closedChainConf);
    adaptClosedChainConfigToBasePos(robot, robot->baseJnt->abs_pos, adaptedConf);
    p3d_set_and_update_robot_conf(adaptedConf);
    p3d_destroy_config(robot, adaptedConf);
  }while (p3d_col_test());
  MY_FREE(att, p3d_matrix4, 2); 
  setSafetyDistance(robot, 0);
//  switchBBActivationForGrasp();
  return q;
}

/**
 * @brief Get the robot grasp configuration given an attach matrix for each arm and the object position. Verify also the hold configuration for the transfert
 * @param robot the robot
 * @param objectPos the object position matrix
 * @param att1 the attach matrix for the first arm
 * @param att2 the attach matrix for the second arm
 * @return the robot config
 */
configPt sampleTwoArmsRobotGraspPosWithHold(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2, int cntrtToActivate)
{
  if (robot->nbCcCntrts > 2) {
    printf("There is more than 2 arms\n");
    return NULL;
  }
  p3d_matrix4 * att = MY_ALLOC(p3d_matrix4, 2);
  p3d_mat4Copy(att1, att[0]);
  p3d_mat4Copy(att2, att[1]);
  configPt q;
//  switchBBActivationForGrasp();
  deactivateHandsVsObjectCol(robot);
  do{
//     p3d_col_activate_obj_env(robot->curObjectJnt->o);
    setSafetyDistance(robot, 0);
    q = sampleRobotGraspConf(robot, objectPos, att, true, false, false, -1, true, false);
    if(q == NULL){
      //  switchBBActivationForGrasp();
//       activateHandsVsObjectCol(robot);
      return NULL;
    }
    setSafetyDistance(robot, (double)SAFETY_DIST);
//     p3d_col_deactivate_obj_env(robot->curObjectJnt->o);
    configPt adaptedConf = p3d_copy_config(robot, robot->closedChainConf);
    adaptClosedChainConfigToBasePos(robot, robot->baseJnt->abs_pos, adaptedConf);
    p3d_set_and_update_robot_conf(adaptedConf);
    p3d_destroy_config(robot, adaptedConf);
  }while (p3d_col_test());
//   p3d_col_activate_obj_env(robot->curObjectJnt->o);
  MY_FREE(att, p3d_matrix4, 2);
  setSafetyDistance(robot, 0);
  //  switchBBActivationForGrasp();
  activateHandsVsObjectCol(robot);
  return q;
}

/**
 * @brief Get the robot grasp without reshooting the base configuration given an attach matrix for each arm and the object position
 * @param robot the robot
 * @param objectPos the object position matrix
 * @param att1 the attach matrix for the first arm
 * @param att2 the attach matrix for the second arm
 * @return the robot config
 */
configPt sampleTwoArmsRobotGraspPosWithoutBase(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2, int shootObject, int cntrtToActivate, bool nonUsedCntrtDesactivation) 
{
  int shootObjectPos = false, shootObjectRot = false;
  if (robot->nbCcCntrts > 2) {
    printf("There is more than 2 arms\n");
    return NULL;
  }
#ifndef GRASP_PLANNING
  deactivateHandsVsObjectCol(robot);
#endif
  p3d_matrix4 * att = MY_ALLOC(p3d_matrix4, 2);
  p3d_mat4Copy(att1, att[0]);
  p3d_mat4Copy(att2, att[1]);
  if(shootObject){
    shootObjectPos = true;
    shootObjectRot = true;
  }
  configPt q = sampleRobotGraspConf(robot, objectPos, att, FALSE, shootObjectPos, shootObjectRot, cntrtToActivate, nonUsedCntrtDesactivation, false);
#ifndef GRASP_PLANNING
  activateHandsVsObjectCol(robot);
#endif
  MY_FREE(att, p3d_matrix4, 2);
  return q;
}

configPt sampleTwoArmsRobotGraspApproachPosWithoutBase(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2, int cntrtToActivate, bool nonUsedCntrtDesactivation) 
{
  if (robot->nbCcCntrts > 2) {
    printf("There is more than 2 arms\n");
    return NULL;
  }
  p3d_matrix4 * att = MY_ALLOC(p3d_matrix4, 2);
  p3d_mat4Copy(att1, att[0]);
  p3d_mat4Copy(att2, att[1]);
  configPt q = sampleRobotGraspConf(robot, objectPos, att, FALSE, FALSE, FALSE, cntrtToActivate, nonUsedCntrtDesactivation, false);
  MY_FREE(att, p3d_matrix4, 2);
  return q;
}

//! @brief Samples the robot grasp reshooting the base configuration 
//! given an attach matrix for each arm and the object position
//! @param robot the robot
//! @param objectPos the object position matrix
//! @param att1 array of attach matrix for the arm
//! @return the robot config
configPt sampleRobotGraspPosWithBase(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 tAtt, int shootObjectPos, int shootObjectRot, int armId, bool nonUsedCntrtDesactivation) 
{
  const int shootBase = true;
  
#ifndef GRASP_PLANNING
  deactivateHandsVsObjectCol(robot);
#endif
  p3d_matrix4* att = MY_ALLOC(p3d_matrix4, robot->armManipulationData->size());
  
  // Get the Attach Matrix in the armManipulationData
  for(int i = 0; i < (int)robot->armManipulationData->size(); i++)
  {
    if(i == armId){
      p3d_mat4Copy(tAtt, att[i]);
    }else{
      att[i][0][0] = att[i][0][1] = att[i][0][2] = 0;
    }
  }
  
  // Get a robot grasp config from the object Pos
  configPt q = sampleRobotGraspConf(robot, objectPos, att, 
                                    shootBase, shootObjectPos, shootObjectRot,
                                    armId, nonUsedCntrtDesactivation, false);
  
#ifndef GRASP_PLANNING
  activateHandsVsObjectCol(robot);
#endif
  MY_FREE(att, p3d_matrix4, robot->armManipulationData->size());
  return q;
}

//! @brief Samples the robot grasp without reshooting the base configuration 
//! given an attach matrix for each arm and the object position
//! @param robot the robot
//! @param objectPos the object position matrix
//! @param att1 array of attach matrix for the arm
//! @return the robot config
configPt sampleRobotGraspPosWithoutBase(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 tAtt, int shootObjectPos, int shootObjectRot, int armId, bool nonUsedCntrtDesactivation) 
{
#ifndef GRASP_PLANNING
  deactivateHandsVsObjectCol(robot);
#endif
  p3d_matrix4* att = MY_ALLOC(p3d_matrix4, robot->armManipulationData->size());
  
  // Get the Attach Matrix in the armManipulationData
  for(int i = 0; i < (int)robot->armManipulationData->size(); i++)
  {
    if(i == armId){
      p3d_mat4Copy(tAtt, att[i]);
    }else{
      att[i][0][0] = att[i][0][1] = att[i][0][2] = 0;
    }
  }
  
  // Get a robot grasp config from the object Pos
  configPt q = sampleRobotGraspConf(robot, objectPos, att, 
                                    false, shootObjectPos, shootObjectRot, 
                                    armId, nonUsedCntrtDesactivation, false);
  
#ifndef GRASP_PLANNING
  activateHandsVsObjectCol(robot);
#endif
  MY_FREE(att, p3d_matrix4, robot->armManipulationData->size());
  return q;
}


/**
 * @brief Get the robot grasp without reshooting the base configuration 
          given an attach matrix for each arm and the object position
 * @param refConf the reference configuration 
 * @param robot the robot
 * @param objectPos the object position matrix
 * @param att1 array of attach matrix for the arm
 * @return the robot config
 */
configPt sampleRobotCloseToConfGraspApproachOrExtract(p3d_rob* robot, configPt refConf, p3d_matrix4 objectPos, p3d_matrix4 tAtt, int shootObject, int armId, bool nonUsedCntrtDesactivation) 
{
  int  shootObjectPos = false, shootObjectRot = false;
  
#ifndef GRASP_PLANNING
  deactivateHandsVsObjectCol(robot);
#endif
  p3d_set_and_update_this_robot_conf(robot, refConf);
  // Fix all the robot for sampling
  double ** jointSamplingState = saveJointSamplingState(robot); 
  fixAllJointsExceptBaseAndObject(robot, refConf);
  fixJoint(robot, robot->baseJnt, robot->baseJnt->abs_pos);
  p3d_matrix4* att = MY_ALLOC(p3d_matrix4, robot->armManipulationData->size());
  
  // Make arms active for sampling
  for(int i = 0; i < (int)robot->armManipulationData->size(); i++){
    if(i == armId){
      p3d_mat4Copy(tAtt, att[i]);
      for(int j = 0; j < (*robot->armManipulationData)[i].getCcCntrt()->npasjnts; j++){
       unFixJoint(robot, (*robot->armManipulationData)[i].getCcCntrt()->pasjnts[i]);
      }
      if(!strcmp((*robot->armManipulationData)[i].getCcCntrt()->namecntrt, "p3d_kuka_arm_ik")){
       unFixJoint(robot, robot->joints[(*robot->armManipulationData)[i].getCcCntrt()->argu_i[0]]);
      }
      if(!strcmp((*robot->armManipulationData)[i].getCcCntrt()->namecntrt, "p3d_lwr_arm_ik")){
       unFixJoint(robot, robot->joints[(*robot->armManipulationData)[i].getCcCntrt()->argu_i[0]]);
      }
    }else{
      att[i][0][0] = att[i][0][1] = att[i][0][2] = 0;
    }
    fixJoint(robot, (*robot->armManipulationData)[i].getManipulationJnt(), (*robot->armManipulationData)[i].getManipulationJnt()->abs_pos);
  }
  
  // Sample a configuration
  if(shootObject){
    shootObjectPos = true;
    shootObjectRot = true;
  }
  
  configPt q = sampleRobotGraspConf(robot, objectPos, att, false, shootObjectPos, shootObjectRot, armId, nonUsedCntrtDesactivation,true);
  
#ifndef GRASP_PLANNING
  activateHandsVsObjectCol(robot);
#endif
  unFixAllJointsExceptBaseAndObject(robot);
  unFixJoint(robot, robot->baseJnt);
  for(int i = 0; i < (int)robot->armManipulationData->size(); i++){
    unFixJoint(robot, (*robot->armManipulationData)[i].getManipulationJnt());
  }
  restoreJointSamplingState(robot, jointSamplingState);
  destroyJointSamplingState(robot, jointSamplingState);
  MY_FREE(att, p3d_matrix4, robot->armManipulationData->size());
  return q;
}

//! @brief Compute The free Arms configuration given a reference configuration
//! @param robot the robot
//! @param objectPos the object position matrix
//! @param att the attach matrix. The number of matrix is robot->nbCcCntrts
//! @return the robot config
double sampleRobotArmsRest(p3d_rob* robot, p3d_matrix4 objectPos, int armToActivate, p3d_matrix4 att, 
                           configPt restConf, configPt conf)
{
  configPt q = p3d_copy_config(robot, conf);
  double minconfCost = computeFreeArmsConfigCost(robot, armToActivate, restConf, conf);
  configPt bestConf = p3d_copy_config(robot, conf);
  p3d_matrix4 bakTatt;
  p3d_mat4Copy((*robot->armManipulationData)[armToActivate].getCcCntrt()->Tatt, bakTatt);
  p3d_mat4Copy(att, (*robot->armManipulationData)[armToActivate].getCcCntrt()->Tatt);
  activateCcCntrts(robot, armToActivate, true);
  
  for(int i = 0; i < (int)robot->armManipulationData->size(); i++)
  {
    if (i != armToActivate) 
    {
      p3d_cntrt* ct = (*robot->armManipulationData)[i].getCcCntrt();
      for(int j = 0; j < ct->npasjnts; j++){
        p3d_jnt* jnt = ct->pasjnts[j];
        for(int k = 0; k < jnt->dof_equiv_nbr; k++){
          q[jnt->index_dof + k] = restConf[jnt->index_dof + k];
        }
      }
      //contraintes spéciales
      //TODO Check for LWR4
      if(!strcmp(ct->namecntrt, "p3d_kuka_arm_ik")) {
        p3d_jnt* jnt = robot->joints[ct->argu_i[0]]; 
        q[jnt->index_dof] = restConf[jnt->index_dof];
      }
    }
  }
  p3d_set_and_update_this_robot_conf(robot, q);
  
  if (p3d_col_test()) 
  {
    configPt newConf = p3d_alloc_config(robot);
    double robotSize = 0, translationFactor = 0, rotationFactor = 0;
    p3d_get_BB_rob_max_size(robot, &robotSize);
    translationFactor = robotSize/10;
    rotationFactor = robotSize/5;
    fixJoint(robot, (*robot->armManipulationData)[armToActivate].getManipulationJnt(), objectPos);
    int nTry = 0;
    int shootTry = 0;
    int nbColTests = 0;
    
    while (nTry < 30 && shootTry < MaxNumberOfTry/2) 
    {
      p3d_gaussian_config2_specific(robot, q, newConf, translationFactor, rotationFactor, true);
      if (p3d_set_and_update_this_robot_conf(robot, newConf)) 
      {
        if(!p3d_col_test()){//collision free
          double confCost = computeFreeArmsConfigCost(robot, armToActivate, restConf, newConf);
          if (minconfCost > confCost) {//better config
            minconfCost = confCost;
            p3d_copy_config_into(robot, newConf, &bestConf);
          }else {
            nTry++;
          }
        }else {
          shootTry+= MaxNumberOfTry*5/100;
        }
        nbColTests++;
      }else {
        shootTry++;
      }
    }
    p3d_destroy_config(robot,newConf);
  }else {
    p3d_copy_config_into(robot, q, &bestConf);
    minconfCost = 0;
  }
  unFixJoint(robot, (*robot->armManipulationData)[armToActivate].getManipulationJnt());
  p3d_set_and_update_this_robot_conf(robot, bestConf);
  p3d_get_robot_config_into(robot, &conf);
  deactivateCcCntrts(robot, armToActivate);
  //Restore the attach matrix
  p3d_mat4Copy(bakTatt, (*robot->armManipulationData)[armToActivate].getCcCntrt()->Tatt);
  p3d_destroy_config(robot,q);
  p3d_destroy_config(robot,bestConf);
  return minconfCost;
}
