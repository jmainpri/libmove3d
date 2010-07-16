#include "Util-pkg.h"
#include "P3d-pkg.h"

#ifdef P3D_COLLISION_CHECKING
#include "Collision-pkg.h"
#endif

#ifdef P3D_PLANNER
#include "Planner-pkg.h"
#endif
	
#include "Graphic-pkg.h"

#ifdef DPG
#include "../planner/dpg/proto/DpgGrid.h"
#endif

#define DEBUG_SETPOS 0

static void p3d_set_thing_pos(int type, char name[20], double tx, double ty,
                              double tz, double rx, double ry, double rz);
static int IS_ABSOLUTE = FALSE;
/* BEGIN *** CARL update NT *** */
/* extern void p3d_vec3Mat4Mult(double a[3], p3d_matrix4, p3d_matrix_type c[3]); */
/* extern int COLLISION_BY_OBJECT; */
/*  END  *** CARL update NT *** */

/************************************************************/

/*--------------------------------------------------------------------------*/
/*!
 * \brief Set position of given robot.
 *
 *  This function use the joint0 as a placement joint.
 *
 *  Note:
 *     - angles are in radian.
 *     - modify the bounds of the robot to have the value between them.
 *       If the joint is fixed, change both bounds to the new value.
 *
 *  \param r:  the robot r
 *  \param q:  the position of the first joint (we use only 6 dof: 
 *              x, y, z, Rx, Ry, Rz).
 */
void p3d_set_this_robot_pos(pp3d_rob r,configPt q) {
  pp3d_jnt jntPt = r->joints[0];
  int i;
  double vmin, vmax;

  for(i=0; i<jntPt->dof_equiv_nbr; i++) {
    p3d_jnt_get_dof_bounds(jntPt, i, &vmin, &vmax);
    if (EQ(vmin, vmax)) { vmin = vmax = q[i]; } else {
      if (vmin>q[i]) { vmin = q[i]; }
      if (vmax<q[i]) { vmax = q[i]; }
    }
    p3d_jnt_set_dof_bounds(jntPt, i, vmin, vmax);
    p3d_jnt_get_dof_rand_bounds(jntPt, i, &vmin, &vmax);
    if (EQ(vmin, vmax)) { vmin = vmax = q[i]; } else {
      if (vmin>q[i]) { vmin = q[i]; }
      if (vmax<q[i]) { vmax = q[i]; }
    }
    p3d_jnt_set_dof_rand_bounds(jntPt, i, vmin, vmax);
    p3d_jnt_set_dof(jntPt, i, q[i]);
  }
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Set position of given robot.
 *
 *  This function use the joint0 as a placement joint.
 *
 *  Note:
 *     - angles are in degree.
 *     - modify the bounds of the robot to have the value between them.
 *       If the joint is fixed, change both bounds to the new value.
 *
 *  \param r:  the robot r
 *  \param q:  the position of the first joint (we use only 6 dof: 
 *              x, y, z, Rx, Ry, Rz).
 */
void p3d_set_this_robot_pos_deg(pp3d_rob r, configPt q) {
  int i;
  for (i=NDOF_BASE_TRANSLATE; i<NDOF_BASE; i++) {
    q[i] = DTOR(q[i]);
  }
  p3d_set_this_robot_pos(r, q);
}


/*
 *  Set position of current robot.
 *
 *  Input:  position x, y, z and orientation theta.
 *
 *  Notice: input angles in radian.
 */
void p3d_set_robot_pos(configPt q) {
  pp3d_rob robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);

  p3d_set_this_robot_pos(robotPt,q);
}

/*
 *  Set position of current robot.
 *
 *  Input:  position x, y, z and orientation theta.
 *
 *  Notice: input angles in degree.
 */

void p3d_set_robot_pos_deg(configPt q) {
  int i;
  for (i=NDOF_BASE_TRANSLATE; i<NDOF_BASE; i++) {
    q[i] = DTOR(q[i]);
  }
  p3d_set_robot_pos(q);
}

/*
 * Set the position  of a joint of the given robot.
 *
 * Input:  the robot,
 *         the joint index in the robot,
 *         the new value.
 *
 * Notice: The value for rotoid joints is expressed in radian.
 */

void p3d_set_this_robot_jnt(p3d_rob *robotPt, int i, double v) {
  pp3d_jnt j = robotPt->joints[i];

  p3d_jnt_set_dof(j, 0, v);
}

/*
 * Set the position  of a joint of the current robot.
 *
 * Input:  the joint index in the current robot,
 *         the new value.
 *
 * Notice: The value for rotoid joints is expressed in radian.
 */

void p3d_set_robot_jnt(int i, double v) {
  pp3d_rob robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);

  p3d_set_this_robot_jnt(robotPt,i,v);
}

/*
 * Set the position  of a joint of the current robot.
 *
 * Input:  the joint index in the current robot,
 *         the new value.
 *
 * Notice: The value for rotoid joints is expressed in degree.
 */

void p3d_set_robot_jnt_deg(int i, double v) {
  pp3d_rob robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  pp3d_jnt j = robotPt->joints[i];

  p3d_jnt_set_dof_deg(j, 0, v);
}


/*
 * Set the position of degree of freedom of the given robot.
 *
 * Input:  the robot,
 *         the degree of freedom index in the robot,
 *         the new value.
 *
 * Notice: The value for rotoid joints is expressed in radian.
 */
void p3d_set_robot_dof(p3d_rob *robotPt, int i, double v) {
  int i_dof;
  pp3d_jnt jntPt;

  jntPt = p3d_robot_dof_to_jnt(robotPt, i, &i_dof);
  p3d_jnt_set_dof(jntPt, i_dof, v);
}


/*
 * Set the position of degree of freedom of the given robot.
 *
 * Input:  the robot,
 *         the degree of freedom index in the robot,
 *         the new value.
 *
 * Notice: The value for rotoid joints is expressed in degree.
 */
void p3d_set_robot_dof_deg(p3d_rob *robotPt, int i, double v) {
  int i_dof;
  pp3d_jnt jntPt;

  jntPt = p3d_robot_dof_to_jnt(robotPt, i, &i_dof);
  p3d_jnt_set_dof_deg(jntPt, i_dof, v);
}

/*
 *  set the position of the current robot to the config given as input
 *  
 *  Input:  the robot,
 *          the new configuration.
 *
 *  Notice: angles in radian
 *
 */
void p3d_set_robot_config(p3d_rob *robotPt, configPt config) {
  int i, j;
  int njnt = robotPt->njoints;
  p3d_jnt * jntPt;

  for(i=0; i<=njnt; i++) {
    jntPt = robotPt->joints[i];
    for(j=0; j<jntPt->dof_equiv_nbr; j++) {
      p3d_jnt_set_dof(jntPt, j, config[jntPt->index_dof+j]); }
  }
}

/*
 *  change the values corresponding to a singularity for the flagged constraint
 *  only one contraint can be flagged at
 *  Input:  the robot,
 *          the new configuration.
 *
 *  Notice: angles in radian
 *
 */
/**
 * @brief Sample the robot singularity
 * @param robotPt the current robot
 * @param cntrtNum the contraint to put at singular value
 * @return 0 if error 1 if success.
 */
int p3d_set_robot_singularity(p3d_rob *robotPt, int cntrtNum, int *singNum) {
  p3d_cntrt_management * cntrt_manager = robotPt->cntrt_manager;
  p3d_cntrt * ct = NULL;
  int i = 0,j = 0,jntNum = 0;
  double v[6] = {0,0,0,0,0,0}, vMin = 0.0, vMax = 0.0;
  p3d_matrix4 endJntAbsPos, newObjPos;
  p3d_singularity * singularity = NULL;

  ct = cntrt_manager->cntrts[cntrtNum];
#ifdef P3D_CONSTRAINTS
  if(*singNum == -1){
    *singNum = p3d_get_random_singularity(ct);
  }

#endif
  singularity = ct->singularities[*singNum];
  for(i = 0; i < singularity->nJnt; i++){
    jntNum = (singularity->singJntVal[i])->jntNum;
    for (j = 0; j < robotPt->joints[jntNum]->dof_equiv_nbr; j++){
      p3d_jnt_set_dof(robotPt->joints[jntNum], j, (singularity->singJntVal[i])->val[j]);
    }
  }
  p3d_update_this_robot_pos(robotPt);
  if(DEBUG_SETPOS){
    configPt config = p3d_alloc_config(robotPt);
    g3d_refresh_allwin_active();
    p3d_get_robot_config_into(robotPt, &config);
    print_config(robotPt, config);
    printf("\n\n");
    p3d_mat4Print((ct->pasjnts[ct->npasjnts - 1])->abs_pos, "end effector");
    p3d_mat4Print((ct->actjnts[0])->pos0_obs, "object");
    p3d_mat4Print(ct->Tatt, "Tatt");
    p3d_matInvertXform(ct->Tatt, newObjPos);
    p3d_mat4Print(newObjPos, "Tsing");
    p3d_destroy_config(robotPt, config);
  }
  //update the position of the end effector
  p3d_matInvertXform((ct->actjnts[0])->pos0_obs, newObjPos); //if the initial manipulated jnt matrix != Id
  p3d_mat4Mult(newObjPos,(ct->pasjnts[ct->npasjnts - 1])->abs_pos , endJntAbsPos);
  p3d_mat4Mult(endJntAbsPos, ct->TSingularity, newObjPos);

  p3d_mat4ExtractPosReverseOrder(newObjPos,&v[0],&v[1],&v[2],&v[3],&v[4],&v[5]);

  switch((ct->actjnts[0])->type){
    case P3D_PLAN:{
      p3d_jnt_get_dof_bounds(ct->actjnts[0], 0, &vMin, &vMax);
      if (v[0] > vMin && v[0] < vMax){
        p3d_jnt_set_dof((ct->actjnts[0]), 0, v[0]);
      }else{
        return 0;
      }
      p3d_jnt_get_dof_bounds(ct->actjnts[0], 1, &vMin, &vMax);
      if (v[1] > vMin && v[1] < vMax){
        p3d_jnt_set_dof((ct->actjnts[0]), 1, v[1]);
      }else{
        return 0;
      }
      p3d_jnt_get_dof_bounds(ct->actjnts[0], 2, &vMin, &vMax);
      if (v[5] > vMin && v[5] < vMax){
        p3d_jnt_set_dof((ct->actjnts[0]), 2, v[5]);
      }else{
        return 0;
      }
      break;
    }
    case P3D_FREEFLYER:{
      for(int i = 0; i < 6; i++){
        p3d_jnt_get_dof_bounds(ct->actjnts[0], i, &vMin, &vMax);
        if (v[i] > vMin && v[i] < vMax){
          p3d_jnt_set_dof((ct->actjnts[0]), i, v[i]);
        }else{
          return 0;
        }
      }
      break;
    }
    default:{
      PrintInfo(("invalid type of active joint for singularity (p3d_setpos.c/p3d_set_robot_singularity)\n"));
      break;
    }
  }
  p3d_update_this_robot_pos_without_cntrt(robotPt);
  if(DEBUG_SETPOS){
    configPt config = p3d_alloc_config(robotPt);
    g3d_refresh_allwin_active();
    p3d_get_robot_config_into(robotPt, &config);
    print_config(robotPt, config);
    printf("\n\n");
    p3d_mat4Print((ct->actjnts[0])->abs_pos, "object");
    p3d_destroy_config(robotPt, config);
  }
  return 1;
}

/*
 *  set the position of the current robot to the config given as input
 *  
 *  Input:  the robot,
 *          the new configuration.
 *
 *  Notice: angles in degree
 *
 */

void p3d_set_robot_config_deg(p3d_rob *robotPt, configPt config) {
  int i, j;
  int njnt = robotPt->njoints;
  p3d_jnt * jntPt;

  for(i=0; i<=njnt; i++) {
    jntPt = robotPt->joints[i];
    for(j=0; j<jntPt->dof_equiv_nbr; j++) { p3d_jnt_set_dof_deg(jntPt, j, config[jntPt->index_dof+j]); }
  }
}

// modif Juan
/*---------------------------------------------------------------------------*/
/*! \brief Update the robot's joints position without constraints and object.
 *
 *  This function update the position and start with rob::first_jnt.
 *  This function compute the position only for the joints that change.
 *  The position of the object is not computed (but will be after a call to
 *  p3d_update_this_robot_pos_without_cntrt() or to
 *  p3d_update_this_robot_pos().
 *
 * \param robotPt: the robot
 */
void p3d_update_this_robot_pos_without_cntrt_and_obj(p3d_rob *robotPt) {
  pp3d_jnt  j;
  int       i;

  j = robotPt->first_joint;

  if (j == NULL) { /* Start with the joint 0 */
    j = robotPt->joints[0];
    p3d_jnt_set_mat_pos(j, NULL);
  } else { p3d_jnt_set_mat_pos(j, robotPt->first_abs_pos); }

  p3d_update_jnt_pos(j);

  robotPt->j_modif = NULL;
  for(i=0; i<=robotPt->njoints; i++) {
    j = robotPt->joints[i];
    p3d_jnt_clean_update(j);
  }

}
// fmodif Juan

/*---------------------------------------------------------------------------*/
/*! \brief Update the robot's joints position without constraints.
 *
 *  This function update the position and start with rob::first_jnt.
 *  This function compute the position only for the joints that change.
 *
 * \param robotPt: the robot
 */
void p3d_update_this_robot_pos_without_cntrt(p3d_rob *robotPt) {
  pp3d_jnt  j;
  int       i;

  j = robotPt->first_joint;

  if (j == NULL) { /* Start with the joint 0 */
    j = robotPt->joints[0];
    p3d_jnt_set_mat_pos(j, NULL);
  } else { p3d_jnt_set_mat_pos(j, robotPt->first_abs_pos); }

  p3d_update_robot_jnt_pos(j);

  p3d_BB_update_BB_rob(robotPt);

  robotPt->j_modif = NULL;
  for(i=0; i<=robotPt->njoints; i++) {
    j = robotPt->joints[i];
    p3d_jnt_clean_update(j);
  }

  #if defined(PQP) && defined(LIGHT_PLANNER)
   p3d_update_carried_object_pos(robotPt);
  #endif
  #ifdef DPG
    if(robotPt->GRAPH && robotPt->GRAPH->dpgGrid){
      robotPt->GRAPH->dpgGrid->updateRobotOccupationCells(robotPt);
    }
  #endif
}


/*---------------------------------------------------------------------------*/
/*! \brief Update the robot's joints position with constraints.
 *
 *  This function update the position and start with rob::first_jnt.
 *  This function compute the position only for the joints that change.
 *
 * \param robotPt: the robot
 */
// modif Juan
int p3d_update_this_robot_pos(p3d_rob *robotPt) {
  p3d_cntrt *ct;

#ifdef P3D_CONSTRAINTS
  /* change la configuration s'il y a des contraintes cinematiques */
  if(robotPt->cntrt_manager->cntrts != NULL) {
    for (dbl_list_goto_first(robotPt->cntrt_manager->cntrt_call_list);
         dbl_list_more(robotPt->cntrt_manager->cntrt_call_list);
         dbl_list_next(robotPt->cntrt_manager->cntrt_call_list)) {
      ct = (p3d_cntrt *) DBL_LIST_DATA(void *, robotPt->cntrt_manager->cntrt_call_list);
      if(ct->active)
        if(!(*ct->fct_cntrt)(ct,-1,NULL,0.0))
          return(FALSE);
    }
  }
#endif

  p3d_update_this_robot_pos_without_cntrt(robotPt);

  return(TRUE);
}

// fmodif Juan


/*****************************************************/
/* Fonction actualisant la position du robot courant */
/* et de toutes ses articulations                    */
/* (a partir de la premiere articulation dont la     */
/* valeur a ete changee)                             */
/* In :                                              */
/* Out :                                             */
/*****************************************************/
int p3d_update_robot_pos(void) {
  int success = FALSE;
  pp3d_rob robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);

  success = p3d_update_this_robot_pos(robotPt);
  return success;
}


/*
 * Set and update the configuration of the current robot.
 *
 * Input:  the new configuration.
 *
 * Notice: angles in radian.
 */

int p3d_set_and_update_this_robot_conf(p3d_rob * rob, configPt q) {
  int I_can;

  p3d_set_robot_config(rob, q);
  I_can = p3d_update_this_robot_pos(rob);
  return I_can;
}


/*
 * Set and update the configuration of the current robot.
 *
 * Input:  the new configuration.
 *
 * Notice: angles in radian.
 */

int p3d_set_and_update_robot_conf(configPt q) {
  int I_can;

  p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);

  p3d_set_robot_config(robotPt, q);
  I_can = p3d_update_robot_pos();
  return I_can;
}

/*
 * Set and update the configuration of the current robot Multi solutions.
 *
 * Input:  the new configuration.
 *
 * Notice: angles in radian.
 */

int p3d_set_and_update_robot_conf_multisol(configPt q, int * ikSol) {
  int I_can;

  p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  I_can = p3d_set_and_update_this_robot_conf_multisol(robotPt, q, NULL, 0.0, ikSol);
  return I_can;
}

/*
 * Set and update the configuration of the current robot.
 *
 * Input:  the new configuration.
 *
 * Notice: angles in degree.
 */

int p3d_set_and_update_robot_conf_deg(configPt q) {
  int I_can;

  p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);

  p3d_set_robot_config_deg(robotPt, q);
  I_can = p3d_update_robot_pos();
  return I_can;
}


// modif Juan
/*****************************************************************/

/*
 * Set and update the configuration of the current robot without constraints.
 *
 * Input:  the new configuration.
 *
 * Notice: angles in radian.
 */

int p3d_set_and_update_this_robot_conf_without_cntrt(p3d_rob * rob, configPt q) {
  p3d_set_robot_config(rob, q);
  p3d_update_this_robot_pos_without_cntrt(rob);
  return 1;
}


/*
 * Set and update the configuration of the current robot with partial re-shoot.
 * 
 * Input:  the new configuration.
 *
 * Notice: angles in radian.
 */


int p3d_set_and_update_this_robot_conf_with_partial_reshoot(p3d_rob * rob, configPt q) {
  int I_can;

  p3d_set_robot_config(rob, q);
  I_can = p3d_update_this_robot_pos_with_partial_reshoot(rob);
  return I_can;
}

/*---------------------------------------------------------------------------*/
/*! \brief Update the robot's joints position with constraints.
 *         The config is re-shooted several times from a cntrt
 *         in order to get a valid one
 *
 *  This function update the position and start with rob::first_jnt.
 *  This function compute the position only for the joints that change.
 *
 * \param robotPt: the robot
 */
int p3d_update_this_robot_pos_with_partial_reshoot(p3d_rob *robotPt) {
  p3d_cntrt *ct;

  /* change la configuration s'il y a des contraintes cinematiques */
  if(robotPt->cntrt_manager->cntrts != NULL) {
    for (dbl_list_goto_first(robotPt->cntrt_manager->cntrt_call_list);
         dbl_list_more(robotPt->cntrt_manager->cntrt_call_list);
         dbl_list_next(robotPt->cntrt_manager->cntrt_call_list)) {
      ct = (p3d_cntrt *) DBL_LIST_DATA(void *, robotPt->cntrt_manager->cntrt_call_list);
      if(ct->active) {
        if(!(*ct->fct_cntrt)(ct,-1,NULL,0.0)) {
          /* NOTE :
             the reshoot works when RLG is activated !!!
             but it should be made a standard shoot for the configurations
             of the active jnts of the ct when RLG is not activated 
          */
          if(p3d_get_RLG() && (ct->reshoot_ct != NULL)) {
            if(!p3d_reshoot_and_set_config_from_ct(robotPt,ct->reshoot_ct)) {
              return(FALSE);
            }
          } else {
            return(FALSE);
          }
        }
      }
    }
  }

  p3d_update_this_robot_pos_without_cntrt(robotPt);

  return(TRUE);
}

/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*! \brief Update the robot's joints position with constraints.
 *         When several solutions for cntrt are possible, one is chosen
 *         depending on previous config
 * 
 *  This function update the position and start with rob::first_jnt.
 *  This function compute the position only for the joints that change.
 *
 * \param robotPt: the robot, the previous config., dl in localpath
 */
int p3d_update_this_robot_pos_multisol(p3d_rob *robotPt, configPt qp, double dl, int* ikSol) {
  p3d_cntrt *ct = NULL;
  int sol = -1;

  /* change la configuration s'il y a des contraintes cinematiques */
  if(robotPt->cntrt_manager->cntrts != NULL) {
    for (dbl_list_goto_first(robotPt->cntrt_manager->cntrt_call_list);
         dbl_list_more(robotPt->cntrt_manager->cntrt_call_list);
         dbl_list_next(robotPt->cntrt_manager->cntrt_call_list)) {
      ct = (p3d_cntrt *) DBL_LIST_DATA(void *, robotPt->cntrt_manager->cntrt_call_list);
      if(ct->active){
        sol = ikSol != NULL ? ikSol[ct->num]:-1;
        if(!(*ct->fct_cntrt)(ct,sol,qp,dl))
          return(FALSE);
      }
    }
  }

  p3d_update_this_robot_pos_without_cntrt(robotPt);

  return(TRUE);
}


/*
 * Set and update the configuration of the current robot.
 * Set and update the configuration of the current robot.
 *
 * Input:  the new configuration.
 *
 * Notice: angles in radian.
 */

int p3d_set_and_update_this_robot_conf_multisol(p3d_rob * rob, configPt q, configPt qp, double dl, int* ikSol) {
  int I_can;

  p3d_set_robot_config(rob, q);
  I_can = p3d_update_this_robot_pos_multisol(rob,qp,dl,ikSol);
  return I_can;
}


/////////////////////////////////////////////////////////////////////////////////////////
// WARNING (Juan) : functions "..._with_partial_reshoot_multisol" are not writen yet !!!
/////////////////////////////////////////////////////////////////////////////////////////

/*****************************************************************/
// fmodif Juan


/*
 *  Set the position of a primitive
 *
 *  Angles are expressed in radian.
 */

void p3d_set_prim_pos(p3d_poly *poly, double tx, double ty, double tz,
                      double rx, double ry, double rz) {
  p3d_matrix4 newpos;

  p3d_mat4Pos(newpos, tx, ty, tz, rx, ry, rz);
  p3d_set_prim_pos_by_mat(poly, newpos);
}

/*
 *  Set the position of a primitive
 *
 *  Angles are expressed in degree.
 */

void p3d_set_prim_pos_deg(p3d_poly *poly, double tx, double ty, double tz,
                          double rx, double ry, double rz) {
  p3d_set_prim_pos(poly, tx, ty, tz, DTOR(rx), DTOR(ry), DTOR(rz));
}

/***************************************************/
/* Fonction changeant la position d'une primitive  */
/* geometrique par matrice de passage              */
/* In : le nom de la primitive, sa matrice         */
/* Out :                                           */
/***************************************************/
void p3d_set_prim_pos_by_mat(p3d_poly *poly, p3d_matrix4 newpos) {

  if (!p3d_isTransfMat(newpos)) {
    if (poly->entity_type>2) {
      double scale;
      if (p3d_extractScale(newpos, &scale)) {
        PrintInfo(("scaling primitive %s with %f\n",poly->poly->name, scale));
        p3d_scale_prim(poly, scale);
      } else {
        PrintInfo(("WARNING in p3d_set_prim_pos_by_mat : primitive %s is loosing its shape\n convert it to a polyhedre\n", poly->poly->name));
        poly->entity_type = POLYHEDRON_ENTITY;
      }
    }
  }
  if (poly->entity_type<3) {
    p3d_pos_poly_by_mat(poly->poly, newpos);
    p3d_compute_poly_BB(poly->poly,
                        &(poly->box.x1),
                        &(poly->box.x2),
                        &(poly->box.y1),
                        &(poly->box.y2),
                        &(poly->box.z1),
                        &(poly->box.z2));
  } else {
    p3d_mat4Copy(newpos,poly->pos0);
    p3d_mat4Copy(newpos,poly->pos_rel_jnt);
    p3d_set_poly_pos(poly->poly,newpos);
#ifdef P3D_COLLISION_CHECKING
    if(!COLLISION_BY_OBJECT)
      p3d_col_set_pos(poly,newpos);
#endif
  }
}

/***********************************************************/
/* 3 Fonctions changeant la position d'un obstacle ou body */
/* In : le nom de l'obstacle, sa position                  */
/* Out :                                                   */
/***********************************************************/
void set_thing_pos(int type, p3d_obj *obst, double tx, double ty,
                   double tz, double rx, double ry, double rz) {
  p3d_matrix4 newpos,pospoly,mr,mr1,mr2,mr3,mt,mtemp,*mat_ptr;
  p3d_vector3 vect;
  p3d_poly *p;
  int np,ip,i,j;
  p3d_jnt *joint;


  p3d_mat4Copy(p3d_mat4IDENTITY,newpos);

  vect[0] = 1.;
  vect[1] = 0.;
  vect[2] = 0.;

  if ((type==P3D_BODY)&&(obst->jnt->prev_jnt!=NULL)&&(obst->jnt->prev_jnt->type!=P3D_BASE)&&!IS_ABSOLUTE) {
    PrintInfo(("we get a pos from a body\n"));
    p3d_mat4Rot(mtemp,vect,rx);
    joint = obst->jnt->prev_jnt;
    while(joint && joint->o == NULL && joint->type!=P3D_BASE){
      joint = joint->prev_jnt;
    }
    if(joint->type==P3D_BASE){
      PrintError(("Check the joints in your model: No object before the body %s\n", obst->name));
      return;
    }
//     p3d_mat4Copy(obst->jnt->prev_jnt->o->opos,newpos);
    p3d_mat4Copy(joint->o->opos,newpos);
    {
      for(i=0;i<=3;i++)
        for(j=0;j<=3;j++)
          PrintInfo(("%f\n",newpos[i][j]));
    }
    p3d_matMultXform(mtemp,newpos,mr1);
  } else
    p3d_mat4Rot(mr1,vect,rx);

  vect[0] = 0.;
  vect[1] = 1.;
  vect[2] = 0.;
  p3d_mat4Rot(mr2,vect,ry);
  p3d_matMultXform(mr2,mr1,mtemp);

  vect[0] = 0.;
  vect[1] = 0.;
  vect[2] = 1.;
  p3d_mat4Rot(mr3,vect,rz);
  p3d_matMultXform(mr3,mtemp,mr);

  vect[0] = tx;
  vect[1] = ty;
  vect[2] = tz;
  p3d_mat4Trans(mt,vect);

  p3d_matMultXform(mt,mr,newpos);

#ifdef P3D_COLLISION_CHECKING
  if(COLLISION_BY_OBJECT) {
    np = obst->np;
    for(ip=0;ip<np;ip++) {
      p = obst->pol[ip];
      mat_ptr=p3d_get_poly_mat(p->poly);
      p3d_get_poly_pos(p->poly,pospoly);
      /*p3d_matMultXform(newpos,pospoly,pos);*/
      p3d_matMultXform(newpos,pospoly,*mat_ptr);
      for(i=0;i<4;i++) {
        for(j=0;j<4;j++) {
          /*  p->pos0[i][j] = pos[i][j]; */
          p->pos0[i][j]=(*mat_ptr)[i][j];
          p->pos_rel_jnt[i][j]=(*mat_ptr)[i][j];
        }
      }
      /*p3d_set_poly_pos(p->poly,pos);*/
      /* p3d_i_collide_set_pos(p,pos);*/
    }
  } else {
    np = obst->np;
    for(ip=0;ip<np;ip++) {
      p = obst->pol[ip];

      if(p->p3d_objPt == obst){//si ce n'est pas un poly concatene
        mat_ptr=p3d_get_poly_mat(p->poly);
        p3d_get_poly_pos(p->poly,pospoly);
        /*p3d_matMultXform(newpos,pospoly,pos);*/
        p3d_matMultXform(newpos,pospoly,*mat_ptr);
        for(i=0;i<4;i++) {
          for(j=0;j<4;j++) {
            /*  p->pos0[i][j] = pos[i][j]; */
            p->pos0[i][j]=(*mat_ptr)[i][j];
            p->pos_rel_jnt[i][j]=(*mat_ptr)[i][j];
          }
        }
        /*p3d_set_poly_pos(p->poly,pos);*/
        /* p3d_i_collide_set_pos(p,pos);*/
        p3d_col_set_pos(p,*mat_ptr);
      }
else
{
;
}
    }
  }
#endif
  if ((type==P3D_BODY)&&(obst->jnt!=NULL)) {
    p3d_jnt_update_rel_pos_object(obst->jnt, obst);//before p3d_jnt_set_object
    p3d_mat4Copy(newpos,obst->opos);
  }

  (*p3d_BB_update_BB_obj)(obst,newpos);
}

static void p3d_set_thing_pos(int type, char name[20], double tx, double ty,
                              double tz, double rx, double ry, double rz) {
  p3d_obj *obst;

  obst = (p3d_obj *)p3d_sel_desc_name(type,name);
  if (obst == NULL) {
    PrintError(("No such thing to position %s\n", name));
  } else {
    set_thing_pos(type,obst,tx,ty,tz,rx,ry,rz);
  }
}


void p3d_set_obst_pos(char name[20], double tx, double ty, double tz, double rx, double ry, double rz) {
  p3d_set_thing_pos(P3D_OBSTACLE,name,tx,ty,tz,rx,ry,rz);
}

void p3d_set_body_pos(char name[20], double tx, double ty, double tz, double rx, double ry, double rz) {
  IS_ABSOLUTE = FALSE;
  p3d_set_thing_pos(P3D_BODY,name,tx,ty,tz,rx,ry,rz);
}

void p3d_set_body_abs_pos(char name[20], double tx, double ty, double tz, double rx, double ry, double rz) {
  IS_ABSOLUTE = TRUE;
  p3d_set_thing_pos(P3D_BODY,name,tx,ty,tz,rx,ry,rz);
  IS_ABSOLUTE = FALSE;
}

/***************************************************/
/* Fonction changeant la position d'un obstacle    */
/* par matrice de passage                          */
/* In : le nom de l'obstacle, sa position          */
/* Out :                                           */
/***************************************************/
void set_obst_pos_by_mat(p3d_obj *obst, p3d_matrix4 newpos) {
  p3d_poly *p;
  int np,ip,i,j;
  p3d_matrix4 pospoly,*mat_ptr;

  np = obst->np;
  for(ip=0;ip<np;ip++) {
    p = obst->pol[ip];
    mat_ptr=p3d_get_poly_mat(p->poly);
    p3d_get_poly_pos(p->poly,pospoly);
    p3d_matMultXform(newpos,pospoly,*mat_ptr);
    for(i=0;i<4;i++) {
      for(j=0;j<4;j++) {
        p->pos0[i][j] = (*mat_ptr)[i][j];
        p->pos_rel_jnt[i][j] = (*mat_ptr)[i][j];
      }
    }
#ifdef P3D_COLLISION_CHECKING
    if(!COLLISION_BY_OBJECT) { p3d_col_set_pos(p,*mat_ptr); }
#endif
  }
  if (obst->jnt!=NULL) {
    p3d_jnt_update_rel_pos_object(obst->jnt, obst);//before p3d_jnt_set_object
    p3d_mat4Copy(newpos,obst->opos);
  }
  (*p3d_BB_update_BB_obj)(obst,newpos);
}

void p3d_set_obst_pos_by_mat(char name[20], p3d_matrix4 newpos) {
  p3d_obj *obst;

  obst = (p3d_obj *) p3d_sel_desc_name(P3D_OBSTACLE,name);

  set_obst_pos_by_mat(obst,newpos);
}



/***************************************************/
/* Fonction changeant la position d'un corps du    */
/* robot courant par matrice de passage            */
/* In : le nom de l'obstacle, sa position          */
/* Out :                                           */
/***************************************************/
void p3d_set_body_pos_by_mat(char name[20], p3d_matrix4 newpos) {
  p3d_obj *body;
  p3d_poly *p;
  int np,ip,i,j;
  p3d_matrix4 pospoly,*mat_ptr;
  /* p3d_matrix4 pos;*/

  body = (p3d_obj *) p3d_sel_desc_name(P3D_BODY,name);
  if(body->jnt->o == body){//si ce n'est pas un poly concatene
    np = body->np;
    for(ip=0;ip<np;ip++) {
      p = body->pol[ip];
      mat_ptr=p3d_get_poly_mat(p->poly);
      p3d_get_poly_pos(p->poly,pospoly);
      p3d_matMultXform(newpos,pospoly,*mat_ptr);
      for(i=0;i<4;i++) {
        for(j=0;j<4;j++) {
          p->pos0[i][j] = (*mat_ptr)[i][j];
          p->pos_rel_jnt[i][j] = (*mat_ptr)[i][j];
        }
      }
#ifdef P3D_COLLISION_CHECKING
      if(!COLLISION_BY_OBJECT) { p3d_col_set_pos(p,*mat_ptr); }
#endif
    }
  }else{
    p3d_mat4Copy(newpos,body->opos);
  }
  if (body->jnt!=NULL) {
    p3d_jnt_update_rel_pos_object(body->jnt, body); //before p3d_jnt_set_object
  }

#ifdef P3D_COLLISION_CHECKING
  if(COLLISION_BY_OBJECT) { p3d_col_set_pos_of_object(body,body->jnt->abs_pos); }
#endif

  (*p3d_BB_update_BB_obj)(body,newpos);
}


/*---------------------------------------------------------------------------*/
/*! \brief Update the joints position.
 *
 *  This function update the position of the joints and of the following 
 *  joints.
 *
 * \param jntPt: the joint
 *
 * \internal
 */
void p3d_update_robot_jnt_pos(p3d_jnt * jntPt) {
  int      i;

  if (jntPt->pos_obj_modified) {
    if (jntPt->o != NULL) { update_robot_obj_pos(jntPt->o); }
    jntPt->pos_obj_modified = FALSE;
  }
  for(i=0; i<jntPt->dof_equiv_nbr; i++) { p3d_jnt_set_dof_is_modified(jntPt, i, FALSE); }

  for(i=0;i<jntPt->n_link_jnt;i++) {
    if(p3d_jnt_calc_mat_pos(jntPt->link_jnt_arr[i])) {
      if (jntPt == jntPt->link_jnt_arr[i]->prev_jnt) { p3d_update_robot_jnt_pos(jntPt->link_jnt_arr[i]->next_jnt); } else { p3d_update_robot_jnt_pos(jntPt->link_jnt_arr[i]->prev_jnt); }
    }
  }
}


/*---------------------------------------------------------------------------*/
/*! \brief Update the joints position without updating the object.
 *
 *  This function update the position of the joints and of the following 
 *  joints without updating the object position.
 *
 * \param jntPt: the joint
 *
 * \internal
 */
void p3d_update_jnt_pos(p3d_jnt * jntPt) {
  int      i;

  for(i=0;i<jntPt->n_link_jnt;i++) {
    if(p3d_jnt_calc_mat_pos(jntPt->link_jnt_arr[i])) {
      if (jntPt == jntPt->link_jnt_arr[i]->prev_jnt) { p3d_update_jnt_pos(jntPt->link_jnt_arr[i]->next_jnt); } else { p3d_update_jnt_pos(jntPt->link_jnt_arr[i]->prev_jnt); }
    }
  }
}

void update_robot_obj_pos(pp3d_obj o) {
  p3d_matrix4 *mat_ptr;
  int i;
	
#ifdef P3D_COLLISION_CHECKING
  if(COLLISION_BY_OBJECT) {
    for(i=0;i<o->np;i++) {
      mat_ptr=p3d_get_poly_mat(o->pol[i]->poly);
      p3d_matMultXform(o->jnt->abs_pos,o->pol[i]->pos_rel_jnt,*mat_ptr);
    }
    p3d_col_set_pos_of_object(o,o->jnt->abs_pos);
  } else {
    for(i=0;i<o->np;i++) {
      mat_ptr=p3d_get_poly_mat(o->pol[i]->poly);
      p3d_matMultXform(o->jnt->abs_pos,o->pol[i]->pos_rel_jnt,*mat_ptr);
      p3d_col_set_pos(o->pol[i],*mat_ptr);
    }
  }
  p3d_rob* robot = o->env->cur_robot;
  (*p3d_BB_update_BB_obj)(o,o->jnt->abs_pos);
  for(i = 0; i < robot->no; i++){
    if(robot->o[i] != o && robot->o[i]->jnt == o->jnt){
      (*p3d_BB_update_BB_obj)(robot->o[i],o->jnt->abs_pos);
    }
  }
#endif
}

/************************************************/

/*
 *  rot_trans4 --
 *
 *  rotation translation matrix corresponding to respectively
 *    a rotation of angle rx about (1,0,0)
 *    a rotation of angle ry about (0,1,0)
 *    a rotation of angle rz about (0,0,1)
 *    a translation of vector (tx, ty, tz)
 *
 */

void rot_trans4(p3d_matrix4 M, double tx, double ty, double tz,
                double rx, double ry, double rz) {
  double t1 = cos(rz);
  double t2 = cos(ry);
  double t4 = sin(rz);
  double t5 = cos(rx);
  double t7 = sin(ry);
  double t8 = t1*t7;
  double t9 = sin(rx);
  double t17 = t4*t7;

  M[0][0] = t1*t2;
  M[0][1] = -t4*t5+t8*t9;
  M[0][2] = t4*t9+t8*t5;
  M[0][3] = tx;

  M[1][0] = t4*t2;
  M[1][1] = t1*t5+t17*t9;
  M[1][2] = -t1*t9+t17*t5;
  M[1][3] = ty;

  M[2][0] = -t7;
  M[2][1] = t2*t9;
  M[2][2] = t2*t5;
  M[2][3] = tz;

  M[3][0] = 0;
  M[3][1] = 0;
  M[3][2] = 0;
  M[3][3] = 1;
}

#if defined(LIGHT_PLANNER)
//! Updates the pose of the carried object with the pose of the virtual object plus the grasp matrix.
//! \return 0 in case of success, 0 otherwise
int p3d_update_carried_object_pos(p3d_rob *robotPt)
{
  if(robotPt==NULL)
  {
    printf("%s: %d: p3d_update_carried_object_pos(): input p3d_rob* is NULL.\n",__FILE__,__LINE__);
    return 1;
  }

//   p3d_matrix4 Tpose;
  configPt q= NULL;
  
  if(robotPt->carriedObject!=NULL && robotPt->isCarryingObject==TRUE)
  { 
    if(robotPt->curObjectJnt == NULL)
    {
      printf("%s: %d: p3d_update_carried_object_pos(): the robot must have a fictive object.\n",__FILE__,__LINE__);
      return 1;
    }

//     p3d_mat4Mult(robotPt->ccCntrts[0]->actjnts[0]->abs_pos, robotPt->Tgrasp, Tpose);

    q= p3d_alloc_config(robotPt->carriedObject);
    p3d_get_robot_config_into(robotPt->carriedObject, &q);
//     p3d_mat4ExtractPosReverseOrder2(Tpose, &q[6], &q[7], &q[8], &q[9], &q[10], &q[11]);
    p3d_mat4ExtractPosReverseOrder2(robotPt->curObjectJnt->abs_pos, &q[6], &q[7], &q[8], &q[9], &q[10], &q[11]);
    p3d_set_and_update_this_robot_conf(robotPt->carriedObject, q);
    p3d_destroy_config(robotPt->carriedObject, q);

  }

  return 0;
}
#endif

//! Sets the configuration of a freeflyer robot from a pose matrix.
//! NB: Values are clamped to the joint parameter bounds.
//! \param robotPt pointer to the robot
//! \param pose desired pose
//! \return 0 in case of success, 1 otherwise
int p3d_set_freeflyer_pose(p3d_rob *robotPt, p3d_matrix4 pose)
{
  if(robotPt==NULL)
  {
    printf("%s: %d: p3d_set_freeflyer_pose(): input p3d_rob* is NULL.\n",__FILE__,__LINE__);
    return 1;
  }

  double tx, ty, tz, alpha, beta, gamma;
  double tx_min, tx_max, ty_min, ty_max, tz_min, tz_max;
  double alpha_min, alpha_max, beta_min, beta_max, gamma_min, gamma_max;
  configPt q= NULL;
  p3d_jnt *firstJoint= NULL;

  p3d_mat4ExtractPosReverseOrder2(pose, &tx, &ty, &tz, &alpha, &beta, &gamma);

  firstJoint= robotPt->joints[1];

  if(firstJoint->type!=P3D_FREEFLYER)
  {
    printf("%s: %d: p3d_set_freeflyer_pose(): the first joint of robot \"%s\" is not of type P3D_FREEFLYER.\n",__FILE__,__LINE__,robotPt->name);
    return 1;
  }

  tx_min= firstJoint->dof_data[0].vmin;
  tx_max= firstJoint->dof_data[0].vmax;
  ty_min= firstJoint->dof_data[1].vmin;
  ty_max= firstJoint->dof_data[1].vmax;
  tz_min= firstJoint->dof_data[2].vmin;
  tz_max= firstJoint->dof_data[2].vmax;

  alpha_min= firstJoint->dof_data[3].vmin;
  alpha_max= firstJoint->dof_data[3].vmax;
  beta_min = firstJoint->dof_data[4].vmin;
  beta_max = firstJoint->dof_data[4].vmax;
  gamma_min= firstJoint->dof_data[5].vmin;
  gamma_max= firstJoint->dof_data[5].vmax;

  if(tx < tx_min)
  {  tx= tx_min;  }
  if(tx > tx_max)
  {  tx= tx_max;  }
  if(ty < ty_min)
  {  ty= ty_min;  }
  if(ty > ty_max)
  {  ty= ty_max;  }
  if(tz < tz_min)
  {  tz= tz_min;  }
  if(tz > tz_max)
  {  tz= tz_max;  }


  alpha= fmod(alpha, 2*M_PI);
  beta= fmod(beta, 2*M_PI);
  gamma= fmod(gamma, 2*M_PI);

  if(alpha < alpha_min)
  {  alpha+= 2*M_PI;  }
  if(alpha > alpha_max)
  {   alpha-= 2*M_PI;  }
  if(beta < beta_min)
  {  beta+= 2*M_PI;  }
  if(beta > beta_max)
  {   beta-= 2*M_PI;  }
  if(gamma < gamma_min)
  {  gamma+= 2*M_PI;  }
  if(gamma > gamma_max)
  {   gamma-= 2*M_PI;  }

  if(alpha < alpha_min)
  {  alpha= alpha_min;  }
  if(alpha > alpha_max)
  {  alpha= alpha_max;  }
  if(beta < beta_min)
  {  beta= beta_min;  }
  if(beta > beta_max)
  {  beta= beta_max;  }
  if(gamma < gamma_min)
  {  gamma= gamma_min;  }
  if(gamma > gamma_max)
  {  gamma= gamma_max;  }

  
  q= p3d_alloc_config(robotPt);
  p3d_get_robot_config_into(robotPt, &q);

  q[firstJoint->index_dof + 0] = tx;
  q[firstJoint->index_dof + 1] = ty;
  q[firstJoint->index_dof + 2] = tz;
  q[firstJoint->index_dof + 3] = alpha;
  q[firstJoint->index_dof + 4] = beta;
  q[firstJoint->index_dof + 5] = gamma;

  p3d_set_and_update_this_robot_conf(robotPt, q);

  p3d_destroy_config(robotPt, q);

  return 0;
}


//! Sets the configuration of a freeflyer robot from a 6 parameters.
//! NB: Values are clamped to the joint parameter bounds.
//! \param robotPt pointer to the robot
//! \param x desired x position
//! \param y desired y position
//! \param z desired z position
//! \param alpha desired first Euler angle
//! \param beta desired second Euler angle
//! \param gamma desired third Euler angle
//! \return 0 in case of success, 1 otherwise
int p3d_set_freeflyer_pose2(p3d_rob *robotPt, double x, double y, double z, double alpha, double beta, double gamma)
{
  if(robotPt==NULL)
  {
    printf("%s: %d: p3d_set_freeflyer_pose2(): input p3d_rob* is NULL.\n",__FILE__,__LINE__);
    return 1;
  }

  double tx_min, tx_max, ty_min, ty_max, tz_min, tz_max;
  double alpha_min, alpha_max, beta_min, beta_max, gamma_min, gamma_max;
  configPt q= NULL;
  p3d_jnt *firstJoint= NULL;

  firstJoint= robotPt->joints[1];

  if(firstJoint->type!=P3D_FREEFLYER)
  {
    printf("%s: %d: p3d_set_freeflyer_pose2(): the first joint of robot \"%s\" is not of type P3D_FREEFLYER.\n",__FILE__,__LINE__,robotPt->name);
    return 1;
  }

  tx_min= firstJoint->dof_data[0].vmin;
  tx_max= firstJoint->dof_data[0].vmax;
  ty_min= firstJoint->dof_data[1].vmin;
  ty_max= firstJoint->dof_data[1].vmax;
  tz_min= firstJoint->dof_data[2].vmin;
  tz_max= firstJoint->dof_data[2].vmax;

  alpha_min= firstJoint->dof_data[3].vmin;
  alpha_max= firstJoint->dof_data[3].vmax;
  beta_min = firstJoint->dof_data[4].vmin;
  beta_max = firstJoint->dof_data[4].vmax;
  gamma_min= firstJoint->dof_data[5].vmin;
  gamma_max= firstJoint->dof_data[5].vmax;

  if(x < tx_min)
  {  x= tx_min;  }
  if(x > tx_max)
  {  x= tx_max;  }
  if(y < ty_min)
  {  y= ty_min;  }
  if(y > ty_max)
  {  y= ty_max;  }
  if(z < tz_min)
  {  z= tz_min;  }
  if(z > tz_max)
  {  z= tz_max;  }

  alpha = fmod(alpha, 2*M_PI);
  beta  = fmod(beta, 2*M_PI);
  gamma = fmod(gamma, 2*M_PI);

  if(alpha < alpha_min)
  {  alpha+= 2*M_PI;  }
  if(alpha > alpha_max)
  {   alpha-= 2*M_PI;  }
  if(beta < beta_min)
  {  beta+= 2*M_PI;  }
  if(beta > beta_max)
  {   beta-= 2*M_PI;  }
  if(gamma < gamma_min)
  {  gamma+= 2*M_PI;  }
  if(gamma > gamma_max)
  {   gamma-= 2*M_PI;  }

  if(alpha < alpha_min)
  {  alpha= alpha_min;  }
  if(alpha > alpha_max)
  {  alpha= alpha_max;  }
  if(beta < beta_min)
  {  beta= beta_min;  }
  if(beta > beta_max)
  {  beta= beta_max;  }
  if(gamma < gamma_min)
  {  gamma= gamma_min;  }
  if(gamma > gamma_max)
  {  gamma= gamma_max;  }

  q= p3d_alloc_config(robotPt);
  p3d_get_robot_config_into(robotPt, &q);

  q[firstJoint->index_dof + 0] = x;
  q[firstJoint->index_dof + 1] = y;
  q[firstJoint->index_dof + 2] = z;
  q[firstJoint->index_dof + 3] = alpha;
  q[firstJoint->index_dof + 4] = beta;
  q[firstJoint->index_dof + 5] = gamma;

  p3d_set_and_update_this_robot_conf(robotPt, q);

  p3d_destroy_config(robotPt, q);

  return 0;
}


