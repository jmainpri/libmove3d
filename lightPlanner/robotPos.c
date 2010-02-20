#include "Collision-pkg.h"
#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Graphic-pkg.h"
#include "Localpath-pkg.h"
#include "robotPos.h"
#include "lightPlannerApi.h"
#include "lightPlanner.h"

static configPt getRobotGraspConf(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 *att, int shootObject, int cntrtToActivate, bool nonUsedCntrtDesactivation);

extern double SAFETY_DIST;
extern double APROACH_OFFSET;
extern double ROBOT_MAX_LENGTH;

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
void adaptClosedChainConfigToBasePos(p3d_rob *robot, p3d_matrix4 base, configPt refConf) {
  p3d_matrix4 relMatrix, newObjectPos, basePos;
  double x = 0, y = 0, z = 0, rx = 0, ry = 0, rz = 0;
  p3d_mat4Copy(base, basePos);
  //On met le robot dans la configuration passee dans le P3D afin de trouver la matrice de transformation entre la base et l'objet.
  p3d_set_and_update_robot_conf(refConf);
  getObjectBaseAttachMatrix(robot->baseJnt->abs_pos, robot->curObjectJnt->abs_pos, relMatrix);
  //Pour la configuration courante de la base, la position de l'objet est base * relMatrix.
  p3d_mat4Mult(basePos, relMatrix, newObjectPos);
  p3d_mat4ExtractPosReverseOrder(newObjectPos, &x, &y, &z, &rx, &ry, &rz);
  p3d_jnt_set_dof(robot->curObjectJnt, 0, x - robot->curObjectJnt->pos0[0][3]);
  p3d_jnt_set_dof(robot->curObjectJnt, 1, y - robot->curObjectJnt->pos0[1][3]);
  p3d_jnt_set_dof(robot->curObjectJnt, 2, z - robot->curObjectJnt->pos0[2][3]);
  p3d_jnt_set_dof(robot->curObjectJnt, 3, rx);
  p3d_jnt_set_dof(robot->curObjectJnt, 4, ry);
  p3d_jnt_set_dof(robot->curObjectJnt, 5, rz);
  //On change la configuration du joint de la base.
  p3d_mat4ExtractPosReverseOrder(basePos, &x, &y, &z, &rx, &ry, &rz);
  p3d_jnt_set_dof(robot->baseJnt, 0, x);
  p3d_jnt_set_dof(robot->baseJnt, 1, y);
  p3d_jnt_set_dof(robot->baseJnt, 2, rz);
  //Test de validitee et collision.
  if (p3d_update_this_robot_pos_with_partial_reshoot(robot)) {
    if (!p3d_col_test()) {
      //Sauvegarde de la configuration.
      p3d_get_robot_config_into(robot, &refConf);
    }
  }else{
    configPt tmp = setTwoArmsRobotGraspPosWithoutBase(robot, robot->curObjectJnt->abs_pos, robot->ccCntrts[0]->Tatt, robot->ccCntrts[1]->Tatt, -1, true);
    if(tmp != NULL){
      //Sauvegarde de la configuration.
      p3d_copy_config_into(robot, tmp, &refConf);
      p3d_destroy_config(robot, tmp);
    }
  }
  p3d_get_robot_config_into(robot, &refConf);
}

/** @brief The maximal number of shoot to try before returning false*/
static int MaxNumberOfTry = 10000;
/**
 * @brief Function for sampling a valid robot configuration given an object position. We assume that the center of the object is the center of object Joint.
 * @param robot the robot
 * @param x the object x coordinate
 * @param y the object y coordinate
 * @param z the object z coordinate
 * @param rx the object rotation around x axis
 * @param ry the object rotation around y axis
 * @param rz the object rotation around z axis
 * @param minRadius the minimum radius to shoot the base. If = -1 compute it automatically
 * @param maxRadius the maximum radius to shoot the base. If = -1 compute it automatically
 * @param shootBase Shoot the base if = 1. Shoot all exept the base when = 0
 * @return the robot config or NULL if fail
 */
configPt p3d_getRobotBaseConfigAroundTheObject(p3d_rob* robot, p3d_jnt* baseJnt, p3d_jnt* objectJnt, double x, double y, double z, double rx, double ry, double rz, double minRadius, double maxRadius, int shootBase, int cntrtToActivate, bool nonUsedCntrtDesactivation){
  double nominalRadius = 0.17; // 10 Deg
  configPt q = NULL;
  if(robot && objectJnt && baseJnt){
    q = p3d_alloc_config(robot);
    configPt qInit = p3d_get_robot_config(robot);
    if(maxRadius == -1){
      if(!robot->isCarryingObject && objectJnt->o){
        maxRadius = MAX(baseJnt->o->BB0.xmax - baseJnt->o->BB0.xmin, baseJnt->o->BB0.ymax - baseJnt->o->BB0.ymin) + MAX(objectJnt->o->BB0.xmax - objectJnt->o->BB0.xmin, objectJnt->o->BB0.ymax - objectJnt->o->BB0.ymin) / 2;
      }else{
        if(!robot->carriedObject){
          printf("p3d_getRobotBaseConfigAroundTheObject : Error, No object loaded");
        }else{
          maxRadius = MAX(baseJnt->o->BB0.xmax - baseJnt->o->BB0.xmin, baseJnt->o->BB0.ymax - baseJnt->o->BB0.ymin) + MAX(robot->carriedObject->joints[1]->o->BB0.xmax - robot->carriedObject->joints[1]->o->BB0.xmin, robot->carriedObject->joints[1]->o->BB0.ymax - robot->carriedObject->joints[1]->o->BB0.ymin) / 2;
        }
      }
    }
    if(minRadius == -1){
      if(!robot->isCarryingObject && objectJnt->o){
      minRadius = MAX(objectJnt->o->BB0.xmax - objectJnt->o->BB0.xmin, objectJnt->o->BB0.ymax - objectJnt->o->BB0.ymin) / 2;
      }else{
        if(!robot->carriedObject){
          printf("p3d_getRobotBaseConfigAroundTheObject : Error, No object loaded");
        }else{
          minRadius = MAX(robot->carriedObject->joints[1]->o->BB0.xmax - robot->carriedObject->joints[1]->o->BB0.xmin, robot->carriedObject->joints[1]->o->BB0.ymax - robot->carriedObject->joints[1]->o->BB0.ymin) / 2;
        }
      }
    }
    activateCcCntrts(robot, cntrtToActivate, nonUsedCntrtDesactivation);
    double bakJntBoundMin[robot->nbCcCntrts], bakJntBoundMax[robot->nbCcCntrts];
    for(int i = 0; i < robot->nbCcCntrts; i++){
      p3d_cntrt* ct = robot->ccCntrts[i];
      if(!strcmp(ct->namecntrt, "p3d_kuka_arm_ik")){//if it is a kuka arm
        //restrict the third joint
        p3d_jnt_get_dof_bounds(robot->joints[ct->argu_i[0]], 0, &(bakJntBoundMin[i]), &(bakJntBoundMax[i]));
        p3d_jnt_set_dof_rand_bounds_deg(robot->joints[ct->argu_i[0]], 0, -120, 0);
      }
    }
    int nbTry = 0;
    bool isKukaBoundOff = false;
    do {
      do {
        p3d_shoot(robot, q, 0);
        if(shootBase == TRUE){
          double randX = p3d_random(minRadius , maxRadius);
          double randY = p3d_random(-(maxRadius * tan(nominalRadius)), (maxRadius * tan(nominalRadius)));
          q[baseJnt->index_dof] = x - cos(rz)*(randX) + sin(rz)*(randY);
          q[baseJnt->index_dof + 1] = y - sin(rz)*(randX) + cos(rz)*(randY) ;
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
          double randRZ = p3d_random(rz + robot->relativeZRotationBaseObject - nominalRadius, rz + robot->relativeZRotationBaseObject + nominalRadius);
          q[baseJnt->index_dof + 2] = randRZ;
        }else{
          for(int i = 0; i < baseJnt->dof_equiv_nbr; i++){
            q[baseJnt->index_dof + i] = qInit[baseJnt->index_dof + i];
          }
        }
        q[objectJnt->index_dof] = x;
        q[objectJnt->index_dof + 1] = y;
        q[objectJnt->index_dof + 2] = z;
        q[objectJnt->index_dof + 3] = rx;
        q[objectJnt->index_dof + 4] = ry;
        q[objectJnt->index_dof + 5] = rz;
        nbTry++;
        if(!isKukaBoundOff && nbTry > MaxNumberOfTry / 2){
          for(int i = 0; i < robot->nbCcCntrts; i++){
            p3d_cntrt* ct = robot->ccCntrts[i];
            if(!strcmp(ct->namecntrt, "p3d_kuka_arm_ik")){//if it is a kuka arm
              //unrestrict the third joint
              p3d_jnt_set_dof_rand_bounds(robot->joints[ct->argu_i[0]], 0, bakJntBoundMin[i], bakJntBoundMax[i]);
            }
          }
          isKukaBoundOff = true;
        }
      } while (!p3d_set_and_update_this_robot_conf_with_partial_reshoot(robot, q) && nbTry < MaxNumberOfTry);
      g3d_draw_allwin_active();
    }while (p3d_col_test()  && nbTry < MaxNumberOfTry);
    if(nbTry >= MaxNumberOfTry){
      return NULL;
    }
    for(int i = 0; i < robot->nbCcCntrts; i++){
      p3d_cntrt* ct = robot->ccCntrts[i];
      if(!strcmp(ct->namecntrt, "p3d_kuka_arm_ik")){//if it is a kuka arm
        //unrestrict the third joint
        p3d_jnt_set_dof_rand_bounds(robot->joints[ct->argu_i[0]], 0, bakJntBoundMin[i], bakJntBoundMax[i]);
      }
    }
    p3d_get_robot_config_into(robot, &q);
  }
  return q;
}

/**
 * @brief Get the robot grasp approach an the robot grasp configuration given an attach matrix for each arm and the object position. Verify also the hold configuration for the transfert
 * @param robot the robot
 * @param objectPos the object position matrix
 * @param att1 the attach matrix for the first arm
 * @param att2 the attach matrix for the second arm
 * @param graspConf the retruned grasp config of the robot
 * @param approachConf the retruned approach config of the robot
 */
void setTwoArmsRobotGraspAndApproachPosWithHold(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2, configPt* graspConf, configPt* approachConf) {
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
      *graspConf = getRobotGraspConf(robot, objectPos, att, TRUE, -1, true);
      if(graspConf == NULL){
        return;
      }
      setSafetyDistance(robot, (double)SAFETY_DIST);
      p3d_col_deactivate_obj_env(robot->curObjectJnt->o);
      deactivateCcCntrts(robot, -1);
      configPt conf = setBodyConfigForBaseMovement(robot, *graspConf, robot->openChainConf);
      p3d_set_and_update_robot_conf(conf);
      p3d_destroy_config(robot, conf);
    }while (p3d_col_test());
    p3d_col_activate_obj_env(robot->curObjectJnt->o);
    configPt adaptedConf = p3d_copy_config(robot, robot->closedChainConf);
    adaptClosedChainConfigToBasePos(robot, robot->baseJnt->abs_pos, adaptedConf);
    p3d_set_and_update_robot_conf(adaptedConf);
    p3d_destroy_config(robot, adaptedConf);
  }while (p3d_col_test());
//  p3d_col_activate_obj_env(robot->curObjectJnt->o);
  setSafetyDistance(robot, 0);
  /*Shift attach position over wrist X axis*/
  att[0][1][3] += -APROACH_OFFSET;
  att[1][1][3] += APROACH_OFFSET;
  p3d_set_and_update_robot_conf(*graspConf);
  *approachConf = getRobotGraspConf(robot, objectPos, att, FALSE, -1, true);
  MY_FREE(att, p3d_matrix4, 2);
  switchBBActivationForGrasp();
  return;
}

/**
 * @brief Get the robot grasp approach configuration given an attach matrix for each arm and the object position. Verify also the hold configuration for the transfert
 * @param robot the robot
 * @param objectPos the object position matrix
 * @param att1 the attach matrix for the first arm
 * @param att2 the attach matrix for the second arm
 * @return the robot config
 */
configPt setTwoArmsRobotGraspApproachPosWithHold(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2, int cntrtToActivate) {
  if (robot->nbCcCntrts > 2) {
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
      q = getRobotGraspConf(robot, objectPos, att, TRUE, cntrtToActivate, true);
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
configPt setTwoArmsRobotGraspPosWithHold(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2, int cntrtToActivate) {
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
    q = getRobotGraspConf(robot, objectPos, att, TRUE, -1, true);
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
configPt setTwoArmsRobotGraspPosWithoutBase(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2, int cntrtToActivate, bool nonUsedCntrtDesactivation) {
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
  configPt q = getRobotGraspConf(robot, objectPos, att, FALSE, cntrtToActivate, nonUsedCntrtDesactivation);
#ifndef GRASP_PLANNING
  activateHandsVsObjectCol(robot);
#endif
  MY_FREE(att, p3d_matrix4, 2);
  return q;
}

configPt setTwoArmsRobotGraspApproachPosWithoutBase(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2, int cntrtToActivate, bool nonUsedCntrtDesactivation) {
  if (robot->nbCcCntrts > 2) {
    printf("There is more than 2 arms\n");
    return NULL;
  }
  p3d_matrix4 * att = MY_ALLOC(p3d_matrix4, 2);
  p3d_mat4Copy(att1, att[0]);
  p3d_mat4Copy(att2, att[1]);
  configPt q = getRobotGraspConf(robot, objectPos, att, FALSE, cntrtToActivate, nonUsedCntrtDesactivation);
  MY_FREE(att, p3d_matrix4, 2);
  return q;
}

/**
 * @brief Get the robot grasp configuration given the attach matrix
 * @param robot the robot
 * @param objectPos the object position matrix
 * @param att the attach matrix. The number of matrix is robot->nbCcCntrts
 * @return the robot config
 */
static configPt getRobotGraspConf(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 *att, int shootObject, int cntrtToActivate, bool nonUsedCntrtDesactivation) {
  double x = 0, y = 0, z = 0, rx = 0, ry = 0, rz = 0;
  configPt q = NULL;
  p3d_mat4ExtractPosReverseOrder(objectPos, &x, &y, &z, &rx, &ry, &rz);
  //Backup and set the attach matrix
  p3d_matrix4 bakTatt[robot->nbCcCntrts];
  for (int i = 0; i < robot->nbCcCntrts; i++) {
    p3d_mat4Copy(robot->ccCntrts[i]->Tatt, bakTatt[i]);
    p3d_mat4Copy(att[i], robot->ccCntrts[i]->Tatt);
  }
  q = p3d_getRobotBaseConfigAroundTheObject(robot, robot->baseJnt, robot->curObjectJnt, x, y, z, rx, ry, rz, -1, ROBOT_MAX_LENGTH, shootObject, cntrtToActivate, nonUsedCntrtDesactivation);
  //Restore the attach matrix
  for (int i = 0; i < robot->nbCcCntrts; i++) {
    p3d_mat4Copy(bakTatt[i], robot->ccCntrts[i]->Tatt);
  }
  return q;
}

/**
 * @brief Compute the cost between the configuration q and a previously declared configuration stored in robot->openChainConf. This cost is computed using the joint distance of the selected arm between the two configs. If the arm is KUKA LWR a potential cost is added to avoid finding configuration too high from the rest configuration. It is preferable to have normalized costs. For LWR the costs are normalized
 * @param robot the robot
 * @param q the configuration to compute the cost
 * @param whichArm The selected arm. This correspond to the Id of the arm Ik constraint stored in robot->ccCntrts
 * @return the configuration cost. For LWR 0 < cost < 1
 */
double computeRobotConfCostSpecificArm(p3d_rob* robot, configPt q, int whichArm){
  double armMediumJointCost = 0;
  p3d_cntrt* ct = robot->ccCntrts[whichArm];
  //MediumJointCost
  for(int j = 0; j < ct->npasjnts; j++){
    armMediumJointCost += SQR(q[ct->pasjnts[j]->index_dof] - robot->openChainConf[ct->pasjnts[j]->index_dof]);
  }
  if(!strcmp(ct->namecntrt, "p3d_kuka_arm_ik")){
    //normalize ArmMediumJointCost
    armMediumJointCost /= 50;
    //PotentialCost
    static double refHeight[3] = {P3D_HUGE, P3D_HUGE, P3D_HUGE};
    if(refHeight[0] == P3D_HUGE){
      p3d_set_and_update_this_robot_conf_without_cntrt(robot, robot->openChainConf);
      refHeight[0] = ct->pasjnts[0]->abs_pos[2][3];
      refHeight[1] = ct->pasjnts[2]->abs_pos[2][3];
      refHeight[2] = ct->pasjnts[4]->abs_pos[2][3];
      p3d_set_and_update_this_robot_conf(robot, q);
    }
    double armPotentialCost = 0;
    double configHeight[3] = {ct->pasjnts[0]->abs_pos[2][3], ct->pasjnts[2]->abs_pos[2][3], ct->pasjnts[4]->abs_pos[2][3]};
    armPotentialCost = ABS((configHeight[1] -configHeight[0]) - (refHeight[1] - refHeight[0])) + ABS((configHeight[2] -configHeight[0]) - (refHeight[2] - refHeight[0]));
    //normalize ArmPotentialCost
    armPotentialCost /= 1.94;
    return (armMediumJointCost + armPotentialCost) / 2;
  }else{
    return armMediumJointCost;
  }
}

/**
 * @brief Compute the cost between the configuration q and a previously declared configuration stored in robot->openChainConf. This cost is computed using the joint distance of the robot arms between the two configs.
 * @param robot the robot
 * @param q the configuration to compute the cost
 * @return the configuration cost. For LWR 0 < cost < 1
 */
double computeRobotConfCost(p3d_rob* robot, configPt q){
  //Check the cost for each arm then combine the two costs
  double armCost = 0;
  for(int i = 0; i < robot->nbCcCntrts; i++){
    armCost += computeRobotConfCostSpecificArm(robot, q, i);
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
  double baseCost = SQR(q[baseDofId] - robot->openChainConf[baseDofId]) / 12.1848;
//   return (armCost / robot->nbCcCntrts + baseCost) / 2;
  return armCost/ robot->nbCcCntrts;
}

std::map<double, configPt, std::less<double> > * searchForLowCostNode(p3d_rob* robot, configPt startConfig, int whichArm){
  switch(whichArm){
    case 0:{
      deactivateCcCntrts(robot, -1);
      break;
    }
    case 1:{
      activateCcCntrts(robot, 0, true);
      break;
    }
    case 2:{
      activateCcCntrts(robot, 1, true);
      break;
    }
    case 3:{
      activateCcCntrts(robot, -1, true);
      break;
    }
  }
  p3d_copy_config_into(robot, startConfig, &robot->ROBOT_POS);
  deleteAllGraphs();
  p3d_set_RANDOM_CHOICE(P3D_RANDOM_SAMPLING);
  p3d_set_SAMPLING_CHOICE(P3D_UNIFORM_SAMPLING);
  p3d_set_MOTION_PLANNER(P3D_DIFFUSION);
  ENV.setBool(Env::isCostSpace,true);
  double oldExtensionStep = ENV.getDouble(Env::extensionStep);
  ENV.setDouble(Env::extensionStep,20);
  ENV.setBool(Env::biDir,false);
  ENV.setBool(Env::expandToGoal,false);
  ENV.setBool(Env::findLowCostConf,true);
  ENV.setInt(Env::tRrtNbtry, 0);
  ENV.setDouble(Env::bestCost, P3D_HUGE);
  ENV.setDouble(Env::findLowCostThreshold, 0.05);
  p3d_specific_search((char*)"");
  ENV.setBool(Env::findLowCostConf,false);
  ENV.setBool(Env::isCostSpace,false);
  ENV.setDouble(Env::extensionStep,oldExtensionStep);
  ENV.setBool(Env::biDir,true);
  ENV.setBool(Env::expandToGoal,true);
  std::map<double, configPt, std::less<double> > * configs = new std::map<double, configPt, std::less<double> >();
  for(p3d_list_node *cur = XYZ_GRAPH->nodes; cur->next; cur = cur->next){
    configs->insert(std::pair<double, configPt>(cur->N->cost, cur->N->q));
  }
  deleteAllGraphs();
  return configs;
}

void correctGraphForNewFixedJoints(p3d_graph* graph, configPt refConf, int nbJoints, p3d_jnt** joints){
  if(!graph || nbJoints == 0){
    return;
  }
  //remove all edge from graph
  for(p3d_list_edge* lEdge = graph->edges, *tmp = NULL; lEdge; lEdge = tmp){
    tmp = lEdge->next;
    MY_FREE(lEdge->E, p3d_edge, 1);
    lEdge->E = NULL;
    MY_FREE(lEdge, p3d_list_edge, 1);
    if(tmp){
      tmp->prev = NULL;
    }
  }
  graph->nedge = 0;
  graph->edges = NULL;
  graph->last_edge = NULL;
  //correct all nodes
  for(p3d_list_node* lNode = graph->nodes; lNode; lNode = lNode->next){
    for(int i = 0; i < nbJoints; i++){
      for(int j = 0; j < joints[i]->dof_equiv_nbr; j++){
        lNode->N->q[joints[i]->index_dof + j] = refConf[joints[i]->index_dof + j];
      }
    }
    //Delete this node's edges list
    for(p3d_list_edge* lEdge = lNode->N->edges, *tmp = NULL; lEdge; lEdge = tmp){
      tmp = lEdge->next;
      lEdge->E = NULL;
      MY_FREE(lEdge, p3d_list_edge, 1);
      if(tmp){
        tmp->prev = NULL;
      }
    }
    lNode->N->nedge = 0;
    lNode->N->edges = NULL;
    //reconstruct the edges using the nodes neigbours
    p3d_list_node* lNeig = lNode->N->neighb, *save = lNode->N->neighb;
    lNode->N->neighb = NULL;
    lNode->N->nneighb = 0;
    for(; lNeig; lNeig = lNeig->next){
      p3d_create_one_edge(graph, lNode->N, lNeig->N, -1);
    }
    //destroy the neighbor list
    lNeig = save;
    for(p3d_list_node* tmp = NULL; lNeig; lNeig = tmp){
      tmp = lNeig->next;
      MY_FREE(lNeig, p3d_list_node, 1);
      if(tmp){
        tmp->prev = NULL;
      }
    }
  }
}

void validateColGraph(p3d_graph* graph){
  int nbUNodes = 0, nbUEdges = 0, nbVEdges = 0;
  if(!graph){
    return;
  }

  p3d_edge* unvalidatedEdges[graph->nedge];
  int nbUnvalidatedEdges = 0;
  //Check all nodes and tag the invalid edges
  for(p3d_list_node* lNode = graph->nodes; lNode; lNode = lNode->next){
    p3d_set_and_update_this_robot_conf_multisol(graph->rob, lNode->N->q, NULL, 0, lNode->N->iksol);
    if(p3d_col_test()){//there is collision => set the edges invalid and store them
      nbUNodes++;
      //look if its necessary to add a flag valid in the node structure
      for(p3d_list_edge* lEdge = lNode->N->edges; lEdge; lEdge = lEdge->next){
        nbUEdges++;
        p3d_unvalid_edge(graph, lEdge->E);
        unvalidatedEdges[nbUnvalidatedEdges] = lEdge->E;
        nbUnvalidatedEdges++;
      }
    }
  }
  //Check all edges execpt
  int ntests = 0;
  for(p3d_list_edge* lEdge = graph->edges; lEdge; lEdge = lEdge->next){
    int i = 0;
    for(; i < nbUnvalidatedEdges; i++){
      if(unvalidatedEdges[i] == lEdge->E){
        break;
      }
    }
    if (i != nbUnvalidatedEdges && p3d_unvalid_localpath_test(graph->rob, lEdge->E->path, &ntests)){
      p3d_unvalid_edge(graph, lEdge->E);
    }else if (lEdge->E->unvalid == 0){
      nbVEdges++;
      p3d_valid_edge(graph, lEdge->E);
    }
  }
  p3d_separate_graph_for_unvalid_edges(graph);
}

#ifdef GRASP_PLANNING
// HandStatus 0 = grasp, 1 = open, 2 = rest
void correctGraphForHandsAndObject(p3d_rob* robot, p3d_graph* graph, int rightHandStatus, gpGrasp rightGrasp, int leftHandStatus, gpGrasp leftGrasp, bool carryobject, int whichArm, p3d_matrix4 tAtt){
  if(!graph){
    return;
  }
  //remove all edge from graph
  for(p3d_list_edge* lEdge = graph->edges, *tmp = NULL; lEdge; lEdge = tmp){
    tmp = lEdge->next;
    MY_FREE(lEdge->E, p3d_edge, 1);
    lEdge->E = NULL;
    MY_FREE(lEdge, p3d_list_edge, 1);
    if(tmp){
      tmp->prev = NULL;
    }
  }
  graph->nedge = 0;
  graph->edges = NULL;
  graph->last_edge = NULL;
  gpHand_properties leftHand, rightHand;
  leftHand.initialize(GP_SAHAND_LEFT);
  rightHand.initialize(GP_SAHAND_RIGHT);
  p3d_matrix4 newObjPos, endJntAbsPos, invTatt;
  p3d_jnt* endEffJnt = NULL;
  bool updatedObject = true;
  if(carryobject){
    p3d_cntrt* ct = robot->ccCntrts[whichArm - 1];
    p3d_matInvertXform((ct->actjnts[0])->pos0_obs, newObjPos); //if the initial manipulated jnt matrix != Id
    p3d_matInvertXform(tAtt, invTatt);
    endEffJnt = ct->pasjnts[ct->npasjnts - 1];
  }
  //correct all nodes
  for(p3d_list_node* lNode = graph->nodes; lNode; lNode = lNode->next){
    switch(rightHandStatus){
      case 0:{
        gpSet_grasp_configuration(robot, rightHand, rightGrasp, lNode->N->q, 1);
        break;
      }
      case 1:{
        gpSet_grasp_open_configuration(robot, rightHand, rightGrasp, lNode->N->q, 1);
        break;
      }
      case 2:{
        gpSet_hand_rest_configuration(robot, rightHand, lNode->N->q, 1);
        break;
      }
    }
    switch(leftHandStatus){
      case 0:{
        gpSet_grasp_configuration(robot, leftHand, leftGrasp, lNode->N->q, 2);
        break;
      }
      case 1:{
        gpSet_grasp_open_configuration(robot, leftHand, leftGrasp, lNode->N->q, 2);
        break;
      }
      case 2:{
        gpSet_hand_rest_configuration(robot, leftHand, lNode->N->q, 2);
        break;
      }
    }
    if(carryobject){
      updatedObject = true;
      p3d_mat4Mult(newObjPos, endEffJnt->abs_pos , endJntAbsPos);
      p3d_mat4Mult(endJntAbsPos, invTatt, newObjPos);
      double v[6] = {0,0,0,0,0,0}, vMin = 0, vMax = 0;
      p3d_mat4ExtractPosReverseOrder(newObjPos,&v[0],&v[1],&v[2],&v[3],&v[4],&v[5]);
      for(int i = 0; i < 6; i++){
        p3d_jnt_get_dof_bounds(robot->curObjectJnt, i, &vMin, &vMax);
        if (v[i] > vMin && v[i] < vMax){
          lNode->N->q[robot->curObjectJnt->index_dof + i] = v[i];
        }else{
          printf("Can not update this configuration : The carried object is out of bounds\n");
          updatedObject = false;
          break;
        }
      }
    }
    //Delete this node's edges list
    for(p3d_list_edge* lEdge = lNode->N->edges, *tmp = NULL; lEdge; lEdge = tmp){
      tmp = lEdge->next;
      lEdge->E = NULL;
      MY_FREE(lEdge, p3d_list_edge, 1);
      if(tmp){
        tmp->prev = NULL;
      }
    }
    lNode->N->nedge = 0;
    lNode->N->edges = NULL;
    lNode->N->IsDiscarded = false;
    if(!updatedObject){
      //remove the node from the graph
      lNode->N->IsDiscarded = true;
      break;
    }
  }
  for(p3d_list_node* lNode = graph->nodes; lNode; lNode = lNode->next){
    if(lNode->N->IsDiscarded){
      break;
    }
    //reconstruct the edges using the nodes neigbours
    p3d_list_node* lNeig = lNode->N->neighb, *save = lNode->N->neighb;
    lNode->N->neighb = NULL;
    lNode->N->nneighb = 0;
    for(; lNeig; lNeig = lNeig->next){
      if(!lNeig->N->IsDiscarded){
        p3d_create_one_edge(graph, lNode->N, lNeig->N, -1);
      }
    }
    //destroy the neighbor list
    lNeig = save;
    for(p3d_list_node* tmp = NULL; lNeig; lNeig = tmp){
      tmp = lNeig->next;
      MY_FREE(lNeig, p3d_list_node, 1);
      if(tmp){
        tmp->prev = NULL;
      }
    }
  }
}
#endif
