#include "../lightPlanner/proto/lightPlannerApi.h"
#include "../lightPlanner/proto/robotPos.h"
#include "Move3d-pkg.h"
#include "P3d-pkg.h"
#include "Collision-pkg.h"

///////////////////////////
//// Static functions /////
///////////////////////////
static void switchObjectsTypes(void);
static void disableAutoCol(p3d_rob* robot);
static void enableAutoCol(p3d_rob* robot);

/** @brief Offset usef to compute the approach configuration of the robot */
double APROACH_OFFSET = 0.120;
/** @brief The robot lenght (length of the arms + lenght of the torso)*/
double ROBOT_MAX_LENGTH  = 1.462;
/** @brief Distance to prevent the platform localisation errors*/
double SAFETY_DIST = 0.05;
/** @brief Use Linear lp or R&s lp for the base*/
int USE_LIN = 0;
/** @brief Max nb collision check for each grasp*/
int MAX_COL_GRASP = 10;


/**
 * @brief Activate the constraints declared in the initialisation to grasp the objects.
 * @param robot The robot to which the constraints are attached
 * @param cntrtNum the constraint number. If -1 activate all constraints
 */
void activateCcCntrts(p3d_rob * robot, int cntrtNum, bool nonUsedCntrtDesactivation){
  if(cntrtNum == -1){
    for(int i = 0; i < robot->nbCcCntrts; i++){
      p3d_activateCntrt(robot, robot->ccCntrts[i]);
    }
  }else{
    for(int i = 0; i < robot->nbCcCntrts; i++){
      if(i == cntrtNum){
        p3d_activateCntrt(robot, robot->ccCntrts[i]);
      }else if (nonUsedCntrtDesactivation) {
        p3d_desactivateCntrt(robot, robot->ccCntrts[i]);
      }
    }
  }

  #if defined(LIGHT_PLANNER) && defined(FK_CNTRT)
  //deactivate the forward kinematics constraints (duals of the closed chains constraints):
  if(cntrtNum == -1){
    for(int i = 0; i < robot->nbFkCntrts; i++){
      p3d_desactivateCntrt(robot, robot->fkCntrts[i]);
    }
  }else{
      p3d_desactivateCntrt(robot, robot->fkCntrts[cntrtNum]);
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

  #if defined(LIGHT_PLANNER) && defined(FK_CNTRT)
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
p3d_cntrt * findTwoJointsFixCntrt(p3d_rob* robot, p3d_jnt* passiveJnt, p3d_jnt* activeJnt){
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

int getGraspingArm(p3d_rob* robot, bool cartesian){
  int activeArm = 0;
  if(cartesian){ //checkwich one of the ccCntrt is active
    for(int i = 0, id = 0; i < robot->nbCcCntrts; i++, id = id*2 + 1){
      if(robot->ccCntrts[i]->active){
        activeArm += id + 1;
      }
    }
  }else{
    for(int i = 0, id = 0; i < robot->nbCcCntrts; i++, id = id*2 + 1){
      p3d_cntrt* fkCntrt = findTwoJointsFixCntrt(robot, robot->curObjectJnt, robot->ccCntrts[i]->actjnts[robot->ccCntrts[i]->nactjnts - 1]);
      if(fkCntrt && fkCntrt->active){
        activeArm += id + 1;
      }
    }
  }
  return activeArm;
}

int getClosestWristToTheObject(p3d_rob* robot){
  int closestArmId = -1;
  double minDist = P3D_HUGE;
  for(int i = 0; i < robot->nbCcCntrts; i++){
    p3d_matrix4 wristPos;
    p3d_mat4Copy(robot->ccCntrts[i]->pasjnts[robot->ccCntrts[i]->npasjnts -1]->abs_pos, wristPos);
//     double dist = p3d_mat4Distance(wristPos, robot->curObjectJnt->abs_pos, 0, 1); //euclidian distance
    p3d_vector3 d;
    d[0]= wristPos[0][3] - robot->curObjectJnt->abs_pos[0][3];
    d[1]= wristPos[1][3] - robot->curObjectJnt->abs_pos[1][3];
    d[2]= wristPos[2][3] - robot->curObjectJnt->abs_pos[2][3];
    double dist = p3d_vectNorm(d);
    if(dist < minDist){
      minDist = dist;
      closestArmId = i + 1;
    }
  }
  return closestArmId;
}

#ifdef GRASP_PLANNING
int getCollisionFreeGraspAndApproach(p3d_rob* robot, p3d_matrix4 objectPos, gpHand_properties handProp, gpGrasp grasp, int whichArm, p3d_matrix4 tAtt, configPt* graspConfig, configPt* approachConfig){
  
  p3d_matrix4 handFrame, fictive;
  p3d_mat4Mult(grasp.frame, handProp.Tgrasp_frame_hand, handFrame);
  p3d_mat4Mult(handFrame, robot->ccCntrts[whichArm -1]->Tatt2, tAtt);
  fictive[0][0] = fictive[0][1] = fictive[0][2] = 0;
  //Check if there is a valid configuration of the robot using this graspFrame
  int maxColGrasps = 0;
  configPt q = NULL;
  gpSet_grasp_configuration(robot, handProp, grasp, whichArm);
  gpFix_hand_configuration(robot, handProp, whichArm);
  bool checkNextGrasp = false;
  do{
    if(q){
      p3d_destroy_config(robot, q);
    }
    if(whichArm == 1){
      q = setTwoArmsRobotGraspPosWithoutBase(robot, objectPos, tAtt, fictive, whichArm - 1, false);
    }else if(whichArm == 2){
      q = setTwoArmsRobotGraspPosWithoutBase(robot, objectPos, fictive, tAtt, whichArm - 1, false);
    }
    maxColGrasps ++;
    if(q){
      p3d_set_and_update_this_robot_conf(robot, q);
      gpSet_grasp_configuration(robot, handProp, grasp, whichArm);
      checkNextGrasp = false;
    }else{
      checkNextGrasp = true;
      continue;
    }
  }while(p3d_col_test() && maxColGrasps <= MAX_COL_GRASP);
  //Save the Grasp Configuration
  if(!checkNextGrasp && maxColGrasps < MAX_COL_GRASP){
    showConfig(q);
    p3d_get_robot_config_into(robot, graspConfig);
    //Check the rest configuration of the hand
    gpSet_grasp_open_configuration(robot, handProp, grasp, whichArm);
    g3d_draw_allwin_active();
    if(!p3d_col_test()){
      p3d_get_robot_config_into(robot, approachConfig);
      p3d_desactivateCntrt(robot, robot->ccCntrts[whichArm - 1]);
      return 0; //success
    }
  }
  return 1;
}

int getBetterCollisionFreeGraspAndApproach(p3d_rob* robot, p3d_matrix4 objectPos, gpHand_type handType, p3d_matrix4 tAtt, configPt* graspConfig, configPt* approachConfig, gpGrasp * grasp){
  int whichArm = 0;
  switch (handType){
    case GP_GRIPPER:{
      whichArm = 0;
      break;
    }
    case GP_SAHAND_RIGHT:{
      whichArm = 1;
      break;
    }
    case GP_SAHAND_LEFT:{
      whichArm = 2;
      break;
    }
    case GP_HAND_NONE:{
      whichArm = 0;
      break;
    }
    default:{
      whichArm = 0;
      break;
    }
  }

  std::list<gpGrasp> graspList;
  gpGet_grasp_list_SAHand(GP_OBJECT_NAME_DEFAULT, whichArm, graspList);
  gpHand_properties handProp;
  handProp.initialize(handType);
  //Activate the corresponding constraint
  p3d_activateCntrt(robot, robot->ccCntrts[whichArm - 1]);
  //For each grasp, get the tAtt and check the collision
  for(std::list<gpGrasp>::iterator iter = graspList.begin(); iter != graspList.end(); iter++){
    if(!getCollisionFreeGraspAndApproach(robot, objectPos, handProp, (*iter), whichArm, tAtt, graspConfig, approachConfig)){
      *grasp = (*iter);
      p3d_desactivateCntrt(robot, robot->ccCntrts[whichArm - 1]);
      return 0;
    }
  }
  p3d_desactivateCntrt(robot, robot->ccCntrts[whichArm - 1]);
  return 1; //fail
}

int selectHandAndGetGraspApproachConfigs(p3d_rob* robot, p3d_matrix4 tAtt, configPt* graspConfig, configPt* approachConfig, gpGrasp* grasp, int* whichArm, bool cartesian){
  gpHand_properties leftHand, rightHand;
  leftHand.initialize(GP_SAHAND_LEFT);
  rightHand.initialize(GP_SAHAND_RIGHT);  
  //Detect which arm is grasping the object.
  switch((*whichArm)){
    case 0:{//no arm is used
      //get closest arm to the object.
      switch(getClosestWristToTheObject(robot)){
        case 1:{
          if (getBetterCollisionFreeGraspAndApproach(robot, robot->curObjectJnt->abs_pos, GP_SAHAND_RIGHT , tAtt, graspConfig, approachConfig, grasp)){
            if (getBetterCollisionFreeGraspAndApproach(robot, robot->curObjectJnt->abs_pos, GP_SAHAND_LEFT , tAtt, graspConfig, approachConfig, grasp)){
              printf("No valid Grasp Found\n");
              return 1;
            }else{
              gpSet_hand_rest_configuration(robot, rightHand, 1);
              gpUnFix_hand_configuration(robot, leftHand, 2);
              *whichArm = 2;
            }
          }else{
            gpSet_hand_rest_configuration(robot, leftHand, 2);
            gpUnFix_hand_configuration(robot, rightHand, 1);
            *whichArm = 1;
          }
          break;
        }
        case 2:{
          if (getBetterCollisionFreeGraspAndApproach(robot, robot->curObjectJnt->abs_pos, GP_SAHAND_LEFT , tAtt, graspConfig, approachConfig, grasp)){
            if (getBetterCollisionFreeGraspAndApproach(robot, robot->curObjectJnt->abs_pos, GP_SAHAND_RIGHT , tAtt, graspConfig, approachConfig, grasp)){
              printf("No valid Grasp Found\n");
              return NULL;
            }else{
              gpSet_hand_rest_configuration(robot, leftHand, 2);
              gpUnFix_hand_configuration(robot, rightHand, 1);
              *whichArm = 1;
            }
          }else{
            gpSet_hand_rest_configuration(robot, rightHand, 1);
            gpUnFix_hand_configuration(robot, leftHand, 2);
            *whichArm = 2;
          }
          break;
        }
        default:{
          printf("There is more than two arms\n");
          return 1;
        }
      }
      break;
    }
    case 1:{//right arm
      if (getBetterCollisionFreeGraspAndApproach(robot, robot->curObjectJnt->abs_pos, GP_SAHAND_RIGHT , tAtt, graspConfig, approachConfig, grasp)){
        printf("No valid Grasp Found\n");
        return 1;
      }else{
        gpUnFix_hand_configuration(robot, rightHand, 1);
        *whichArm = 1;
      }
      break;
    }
    case 2:{//left arm
      if (getBetterCollisionFreeGraspAndApproach(robot, robot->curObjectJnt->abs_pos, GP_SAHAND_LEFT , tAtt, graspConfig, approachConfig, grasp)){
        printf("No valid Grasp Found\n");
        return 1;
      }else{
        gpUnFix_hand_configuration(robot, leftHand, 2);
        *whichArm = 2;
      }
      break;
    }
    default:{
      printf("ERROR: No arm available to grasp or the robot has more than two arms\n");
      return 1;
    }
  }
  return 0;
}
#endif
