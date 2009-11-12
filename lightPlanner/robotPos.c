#include "Collision-pkg.h"
#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Graphic-pkg.h"
#include "../lightPlanner/proto/robotPos.h"
#include "../lightPlanner/proto/lightPlannerApi.h"


static configPt getRobotGraspConf(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 *att, int shootObject, int cntrtToActivate);

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
    configPt tmp = setTwoArmsRobotGraspPosWithoutBase(robot, robot->curObjectJnt->abs_pos, robot->ccCntrts[0]->Tatt, robot->ccCntrts[1]->Tatt, -1);
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
configPt p3d_getRobotBaseConfigAroundTheObject(p3d_rob* robot, p3d_jnt* baseJnt, p3d_jnt* objectJnt, double x, double y, double z, double rx, double ry, double rz, double minRadius, double maxRadius, int shootBase, int cntrtToActivate){
  double nominalRadius = 0.17; // 10 Deg
  configPt q = NULL;
  if(robot && objectJnt && baseJnt){
    q = p3d_alloc_config(robot);
    configPt qInit = p3d_get_robot_config(robot);
    if(maxRadius == -1){
      maxRadius = MAX(baseJnt->o->BB0.xmax - baseJnt->o->BB0.xmin, baseJnt->o->BB0.ymax - baseJnt->o->BB0.ymin) + MAX(objectJnt->o->BB0.xmax - objectJnt->o->BB0.xmin, objectJnt->o->BB0.ymax - objectJnt->o->BB0.ymin) / 2;
    }
    if(minRadius == -1){
      minRadius = MAX(objectJnt->o->BB0.xmax - objectJnt->o->BB0.xmin, objectJnt->o->BB0.ymax - objectJnt->o->BB0.ymin) / 2;
    }
    activateCcCntrts(robot, cntrtToActivate);
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
      } while (!p3d_set_and_update_this_robot_conf_with_partial_reshoot(robot, q) && nbTry < MaxNumberOfTry);
//       g3d_draw_allwin_active();
      if(nbTry > MaxNumberOfTry / 2){
        for(int i = 0; i < robot->nbCcCntrts; i++){
          p3d_cntrt* ct = robot->ccCntrts[i];
          if(!strcmp(ct->namecntrt, "p3d_kuka_arm_ik")){//if it is a kuka arm
            //unrestrict the third joint
            p3d_jnt_set_dof_rand_bounds(robot->joints[ct->argu_i[0]], 0, bakJntBoundMin[i], bakJntBoundMax[i]);
          }
        }
      }
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
      *graspConf = getRobotGraspConf(robot, objectPos, att, TRUE, -1);
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
    configPt adaptedConf = p3d_copy_config(robot, robot->closedChainConf);
    adaptClosedChainConfigToBasePos(robot, robot->baseJnt->abs_pos, adaptedConf);
    p3d_set_and_update_robot_conf(adaptedConf);
    p3d_destroy_config(robot, adaptedConf);
  }while (p3d_col_test());
  p3d_col_activate_obj_env(robot->curObjectJnt->o);
  setSafetyDistance(robot, 0);
  /*Shift attach position over wrist X axis*/
  att[0][1][3] += -APROACH_OFFSET;
  att[1][1][3] += APROACH_OFFSET;
  p3d_set_and_update_robot_conf(*graspConf);
  *approachConf = getRobotGraspConf(robot, objectPos, att, FALSE, -1);
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
      q = getRobotGraspConf(robot, objectPos, att, TRUE, cntrtToActivate);
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
    q = getRobotGraspConf(robot, objectPos, att, TRUE, -1);
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
configPt setTwoArmsRobotGraspPosWithoutBase(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2, int cntrtToActivate) {
  if (robot->nbCcCntrts > 2) {
    printf("There is more than 2 arms\n");
    return NULL;
  }
  deactivateHandsVsObjectCol(robot);
  p3d_matrix4 * att = MY_ALLOC(p3d_matrix4, 2);
  p3d_mat4Copy(att1, att[0]);
  p3d_mat4Copy(att2, att[1]);
  configPt q = getRobotGraspConf(robot, objectPos, att, FALSE, cntrtToActivate);
  activateHandsVsObjectCol(robot);
  MY_FREE(att, p3d_matrix4, 2);
  return q;
}

configPt setTwoArmsRobotGraspApproachPosWithoutBase(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2, int cntrtToActivate) {
  if (robot->nbCcCntrts > 2) {
    printf("There is more than 2 arms\n");
    return NULL;
  }
  p3d_matrix4 * att = MY_ALLOC(p3d_matrix4, 2);
  p3d_mat4Copy(att1, att[0]);
  p3d_mat4Copy(att2, att[1]);
  configPt q = getRobotGraspConf(robot, objectPos, att, FALSE, cntrtToActivate);
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
static configPt getRobotGraspConf(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 *att, int shootObject, int cntrtToActivate) {
  double x = 0, y = 0, z = 0, rx = 0, ry = 0, rz = 0;
  configPt q;
  p3d_mat4ExtractPosReverseOrder(objectPos, &x, &y, &z, &rx, &ry, &rz);
  //Backup and set the attach matrix
  p3d_matrix4 bakTatt[robot->nbCcCntrts];
  for (int i = 0; i < robot->nbCcCntrts; i++) {
    p3d_mat4Copy(robot->ccCntrts[i]->Tatt, bakTatt[i]);
    p3d_mat4Copy(att[i], robot->ccCntrts[i]->Tatt);
  }
  q = p3d_getRobotBaseConfigAroundTheObject(robot, robot->baseJnt, robot->curObjectJnt, x, y, z, rx, ry, rz, -1, ROBOT_MAX_LENGTH, shootObject, cntrtToActivate);
  //Restore the attach matrix
  for (int i = 0; i < robot->nbCcCntrts; i++) {
    p3d_mat4Copy(bakTatt[i], robot->ccCntrts[i]->Tatt);
  }
  return q;
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
