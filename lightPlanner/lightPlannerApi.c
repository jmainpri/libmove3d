#include "../lightPlanner/proto/lightPlannerApi.h"
#include "Move3d-pkg.h"
#include "P3d-pkg.h"
#include "Collision-pkg.h"

///////////////////////////
//// Static functions /////
///////////////////////////
static void switchObjectsTypes(void);
static void disableAutoCol(p3d_rob* robot);
static void enableAutoCol(p3d_rob* robot);
static p3d_cntrt * findTwoJointsFixCntrt(p3d_rob* robot, p3d_jnt* passiveJnt, p3d_jnt* activeJnt);


/** @brief Offset usef to compute the approach configuration of the robot */
double APROACH_OFFSET = 0.120;
/** @brief The robot lenght (length of the arms + lenght of the torso)*/
double ROBOT_MAX_LENGTH  = 1.462;
/** @brief Distance to prevent the platform localisation errors*/
double SAFETY_DIST = 0.05;
/** @brief Use Linear lp or R&s lp for the base*/
int USE_LIN = 0;

/**
 * @brief Activate the constraints declared in the initialisation to grasp the objects.
 * @param robot The robot to which the constraints are attached
 * @param cntrtNum the constraint number. If -1 activate all constraints
 */
void activateCcCntrts(p3d_rob * robot, int cntrtNum){
  if(cntrtNum == -1){
    for(int i = 0; i < robot->nbCcCntrts; i++){
      p3d_activateCntrt(robot, robot->ccCntrts[i]);
    }
  }else{
    for(int i = 0; i < robot->nbCcCntrts; i++){
      if(i == cntrtNum){
        p3d_activateCntrt(robot, robot->ccCntrts[i]);
      }else{
        p3d_desactivateCntrt(robot, robot->ccCntrts[i]);
      }
    }
  }

  #ifdef FK_CNTRT
  //deactivate the forward kinematics constraints (duals of the closed chains constraints):
  if(cntrtNum == -1){
    for(int i = 0; i < robot->nbFkCntrts; i++){
      p3d_desactivateCntrt(robot, robot->fkCntrts[i]);
    }
  }else{
    for(int i = 0; i < robot->nbFkCntrts; i++){
      if(i == cntrtNum){
        p3d_desactivateCntrt(robot, robot->fkCntrts[i]);
      }else{
        p3d_activateCntrt(robot, robot->fkCntrts[i]);
      }
    }
  }
  #endif
}
/**
 * @brief Deactivate the constraints declared in the initialisation to grasp the objects.
 * @param robot The robot to which the constraints are attached
 * @param cntrtNum the constraint number. If -1 activate all constraints
 */
void deactivateCcCntrts(p3d_rob * robot, int cntrtNum){
  if(cntrtNum == -1){
    for(int i = 0; i < robot->nbCcCntrts; i++){
      p3d_desactivateCntrt(robot, robot->ccCntrts[i]);
    }
  }else{
    p3d_desactivateCntrt(robot, robot->ccCntrts[cntrtNum]);
  }

  #ifdef FK_CNTRT
  //activate the forward kinematics constraints (duals of the closed chains constraints):
  if(cntrtNum == -1){
    for(int i = 0; i < robot->nbFkCntrts; i++){
      p3d_activateCntrt(robot, robot->fkCntrts[i]);
    }
  }else{
    for(int i = 0; i < robot->nbFkCntrts; i++){
      if(i == cntrtNum){
        p3d_activateCntrt(robot, robot->fkCntrts[i]);
      }else{
        p3d_desactivateCntrt(robot, robot->fkCntrts[i]);
      }
    }
  }
  #endif
}

/**
 * @brief Activate or desactivate the defined objects against collision
 */
static void switchObjectsTypes(void) {
  p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  for (int i = 0 ; i < robotPt->graspNbJoints; i++) {
    p3d_obj* jntObj = (robotPt->graspJoints[i])->o;
    for (int j = 0; j < jntObj->np; j++) {
      if (jntObj->pol[j]->TYPE == P3D_GRAPHIC) {
        jntObj->pol[j]->TYPE = P3D_GHOST;
      } else if (jntObj->pol[j]->TYPE == P3D_GHOST) {
        jntObj->pol[j]->TYPE = P3D_GRAPHIC;
      }
    }
  }
}

/**
 * @brief Activate or desactivate the defined objects against collision and restart the collision checker to take this modification into account
 */
void switchBBActivationForGrasp(void) {
  switchObjectsTypes();
  p3d_col_stop();
  p3d_col_start(p3d_col_mode_kcd);
}

void setSafetyDistance(p3d_rob* robot, double dist){
  if(dist != 0){
    disableAutoCol(robot);
  }else{
    enableAutoCol(robot);
  }
  p3d_set_env_object_tolerance(dist);
  p3d_col_set_tolerance(dist);
}

/**
 * @brief Disable the autocollision of the given robot
 * @param robot The robot to deactive
 */
static void disableAutoCol(p3d_rob* robot){
  p3d_col_deactivate_rob(robot);
}


/**
 * @brief Enable the autocollision of the given robot
 * @param robot The robot to set active
 */
static void enableAutoCol(p3d_rob* robot){
  p3d_col_activate_rob(robot);
}

/**
 * @brief Get the trasformation matrix between the object and the base positions.
 * @param base The base position
 * @param object The object position
 * @param result The resulting matrix
 */
void getObjectBaseAttachMatrix(p3d_matrix4 base, p3d_matrix4 object, p3d_matrix4 result) {
  p3d_matrix4 tmp;
  p3d_matInvertXform(base, tmp);
  p3d_mat4Mult(tmp, object, result);
}

/**
 * @brief Desactivate the collisions between the hands and the object. All theses objects are initialised in the startup of the application (reading the P3d file here)
 * @param robot The robot
 */
void deactivateHandsVsObjectCol(p3d_rob* robot) {
  for (int i = 0; i < robot->graspNbJoints; i++) {
    for(int j = 0; j < robot->no; j++){
      if(robot->o[j]->jnt->num == robot->graspJoints[i]->num){
        p3d_col_deactivate_obj_obj(robot->o[j], robot->curObjectJnt->o);
        p3d_col_deactivate_obj_env(robot->o[j]);
        p3d_col_deactivate_obj_all_rob(robot->o[j]);
      }
    }
  }
}

/**
 * @brief Activate the collisions between the hands and the object. All theses objects are initialised in the startup of the application (reading the P3d file here)
 * @param robot The robot
 */
void activateHandsVsObjectCol(p3d_rob* robot) {
  for (int i = 0; i < robot->graspNbJoints; i++) {
    for(int j = 0; j < robot->no; j++){
      if(robot->o[j]->jnt->num == robot->graspJoints[i]->num){
        p3d_col_activate_obj_obj(robot->o[j], robot->curObjectJnt->o);
        p3d_col_activate_obj_env(robot->o[j]);
        p3d_col_activate_obj_all_rob(robot->o[j]);
      }
    }
  }
}

/**
 * @brief Desactivate the collisions between the object and the world.
 * @param robot The robot
 */
void deactivateObjectCol(p3d_rob* robot) {
  p3d_col_deactivate_obj_env(robot->curObjectJnt->o);
}

/**
 * @brief Activate the collisions between the object and the world.
 * @param robot The robot
 */
void activateObjectCol(p3d_rob* robot) {
  p3d_col_activate_obj_env(robot->curObjectJnt->o);
}

/**
 * @brief Set the robot at the given configuration and do not shoot all joints except those of the base and the object declared in the initialisation of the program (here in the p3d file). The fixed joints will not be planned.
 * @param robot The robot
 * @param conf The configuration to fix the joints
 */
void fixAllJointsExceptBaseAndObject(p3d_rob * robot, configPt conf) {
  p3d_set_and_update_robot_conf(conf);
  for (int i = 0; i < robot->njoints + 1; i++) {
    p3d_jnt * joint = robot->joints[i];
    if (joint->type != P3D_BASE && joint->type != P3D_FIXED && joint != robot->curObjectJnt && joint != robot->baseJnt) {
      fixJoint(robot, joint, joint->jnt_mat);
    }
  }
  p3d_update_this_robot_pos(robot);
}

/**
 * @brief Unfix all the joints of the given robot exept the base and the object joints. The joints will be sampled in the next planning step
 * @param robot
 */
void unFixAllJointsExceptBaseAndObject(p3d_rob * robot) {
  for (int i = 0; i < robot->njoints + 1; i++) {
    p3d_jnt * joint = robot->joints[i];
    if (joint->type != P3D_BASE && joint->type != P3D_FIXED && joint != robot->curObjectJnt && joint != robot->baseJnt) {
      unFixJoint(robot, joint);
    }
  }
}

/**
 * @brief Set the robot jiont at the given position and do not shoot it. The fixed joints will not be planned.
 * @param robot The robot
 * @param joint The joint to fix
 * @param initPos The position or the value of the joint
 */
void fixJoint(p3d_rob * robot, p3d_jnt * joint, p3d_matrix4 initPos) {
  double * dVal = getJntDofValue(robot, joint, initPos);
  for (int i = 0; i < joint->dof_equiv_nbr; i++) {
    if (robot->isUserDof[joint->index_dof + i]) {
      p3d_jnt_set_dof(joint, i, dVal[i]);
      joint->dof_data[i].is_user = FALSE;
    }
  }
  MY_FREE(dVal, double, joint->dof_equiv_nbr);
}

/**
 * @brief Unfix the given joint. The joint will be sampled in the next planning step
 * @param robot The robot
 * @param joint The joint to unfix
 */
void unFixJoint(p3d_rob * robot, p3d_jnt * joint) {
  for (int i = 0; i < joint->dof_equiv_nbr; i++) {
    if (robot->isUserDof[joint->index_dof + i]) {
      joint->dof_data[i].is_user = TRUE;
    }
  }
}

/**
 * @brief Trasform a homogenious matrix to euler parameter depending on the type of the joint.
 * @param robot The robot
 * @param joint The joint to get it type
 * @param initPos The matrix to convert
 * @return Array containing the euler parameter for this joint
 */
double* getJntDofValue(p3d_rob * robot, p3d_jnt * joint, p3d_matrix4 initPos){
  double x = 0, y = 0, z = 0, rx = 0, ry = 0, rz = 0;
  p3d_mat4ExtractPosReverseOrder(initPos, &x, &y, &z, &rx, &ry, &rz);
  switch(joint->type){
    case P3D_BASE:{
      double * dVal = MY_ALLOC(double, 6);
      dVal[0] = x;
      dVal[1] = y;
      dVal[2] = z;
      dVal[3] = rx;
      dVal[4] = ry;
      dVal[5] = rz;
      return dVal;
    }
    case P3D_ROTATE:{
      double * dVal = MY_ALLOC(double, 1);
      dVal[0] = rz;
      return dVal;
    }
    case P3D_TRANSLATE:{
      double * dVal = MY_ALLOC(double, 1);
      dVal[0] = x;
      return dVal;
    }
    case P3D_PLAN:{
      double * dVal = MY_ALLOC(double, 3);
      dVal[0] = x;
      dVal[1] = y;
      dVal[2] = rz;
      return dVal;
    }
    case P3D_FREEFLYER:{
      double * dVal = MY_ALLOC(double, 6);
      dVal[0] = x;
      dVal[1] = y;
      dVal[2] = z;
      dVal[3] = rx;
      dVal[4] = ry;
      dVal[5] = rz;
      return dVal;
    }
    case P3D_FIXED:{
      return NULL;
    }
    case P3D_KNEE:{
      double * dVal = MY_ALLOC(double, 3);
      dVal[0] = rx;
      dVal[1] = ry;
      dVal[2] = rz;
      return dVal;
    }
    default:{
      return NULL;
    }
  }
}

/**
 * @brief find a constraint that link the two given joints
 * @param robot The robot
 * @param passiveJnt The passive joint in the constraint
 * @param activeJnt The active joint in the constraint
 * @return The constraint if found, NULL otherwise
 */
static p3d_cntrt * findTwoJointsFixCntrt(p3d_rob* robot, p3d_jnt* passiveJnt, p3d_jnt* activeJnt){
  for (int i = 0; i < robot->cntrt_manager->ncntrts; i++) {
    //Check if the constraint is already created
    p3d_cntrt *cntrt = robot->cntrt_manager->cntrts[i];
    if (cntrt->npasjnts == 1 && cntrt->nactjnts == 1 && cntrt->pasjnts[0]->num == passiveJnt->num && cntrt->actjnts[0]->num == activeJnt->num) {
      return cntrt;
    }
  }
  return NULL;
}

/**
 * @brief Activate the fix constraint between the two given joint. If the constraint do not exists create new one.
 * @param robot The robot
 * @param passiveJnt The passive joint in the constraint
 * @param activeJnt The active joint in the constraint
 */
void setAndActivateTwoJointsFixCntrt(p3d_rob * robot, p3d_jnt* passiveJnt, p3d_jnt* activeJnt) {
  int passiveJntId[1] = {passiveJnt->num}, activeJntId[1] = {activeJnt->num};
  p3d_cntrt * cntrt = findTwoJointsFixCntrt(robot, passiveJnt, activeJnt);
  //If the constraint is already created
  if (cntrt != NULL) {
    //Activate it
    p3d_activateCntrt(robot, cntrt);
  } else if (!p3d_constraint("p3d_fix_jnts_relpos", -1, passiveJntId, -1, activeJntId, -1, NULL, -1, NULL, -1, 1)) {
    printf("Error in creating the p3d_fix_jnts_relpos\n");
  } else {
    cntrt = findTwoJointsFixCntrt(robot, passiveJnt, activeJnt);
    //reinitialize iksols
    p3d_realloc_iksol(robot->cntrt_manager);
  }
  //set the attach Matrix
  getObjectBaseAttachMatrix(activeJnt->abs_pos, passiveJnt->abs_pos, cntrt->Tatt);
  return;
}

/**
 * @brief Deactivate the fix constraint between the two joints given.
 * @param robot The robot
 * @param passiveJnt The passive joint in the constraint
 * @param activeJnt The active joint in the constraint
 */
void desactivateTwoJointsFixCntrt(p3d_rob * robot, p3d_jnt* passiveJnt, p3d_jnt* activeJnt) {
  p3d_cntrt * cntrt = findTwoJointsFixCntrt(robot, passiveJnt, activeJnt);
  p3d_desactivateCntrt(robot, cntrt);
}

/**
 * @brief Set the object bounds to be shooted near the base
 * @param robot The robot
 * @param baseJnt The base joint
 * @param objectJnt The object joint
 * @param radius The radius around the base where to shoot the object -1 if the function have to compute it and -2 if the function takes the ROBOT_MAX_LENGTH
 */
void shootTheObjectArroundTheBase(p3d_rob* robot, p3d_jnt* baseJnt, p3d_jnt* objectJnt, double radius){
  if(radius == -1){
    radius = MAX(baseJnt->o->BB0.xmax - baseJnt->o->BB0.xmin, baseJnt->o->BB0.ymax - baseJnt->o->BB0.ymin) + MAX(objectJnt->o->BB0.xmax - objectJnt->o->BB0.xmin, objectJnt->o->BB0.ymax - objectJnt->o->BB0.ymin) / 2;
  }else if(radius == -2){
    radius = ROBOT_MAX_LENGTH;
  }
  //take only x and y composantes of the base
  double dof[2][2];
  for(int i = 0; i < 2; i++){
    dof[i][0] = p3d_jnt_get_dof(baseJnt, i) - radius;
    dof[i][1] = p3d_jnt_get_dof(baseJnt, i) + radius;
  }
  for(int i = 0; i < 2; i++){
    p3d_jnt_set_dof_rand_bounds(objectJnt, i, dof[i][0], dof[i][1]);
  }
}

/**
 * @brief Set the original object bounds
 * @param robot The robot
 * @param objectJnt The object joint
 */
void shootTheObjectInTheWorld(p3d_rob* robot, p3d_jnt* objectJnt){
  for(int i = 0; i < objectJnt->dof_equiv_nbr; i++){
    double vmin = 0, vmax = 0;
    p3d_jnt_get_dof_bounds(objectJnt, i, &vmin, &vmax);
    p3d_jnt_set_dof_rand_bounds(objectJnt, i, vmin, vmax);
  }
}
