/****************************************************************************/
/*!\file p3d_col_traj.c
  \brief dynamic collision checker
  \ingroup collision
*/
/****************************************************************************/


#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"   // <- modif Juan (for debugging)
//start path deform
#include "Localpath-pkg.h"
//end path deform
#define DEBUG_COL_TRAJ 0   // <- modif Juan

//start path deform
static int DEPTH = 0;
static p3d_config_on_edge*  p3d_allocinit_config_on_edge(p3d_rob *robotPt);
static void p3d_destroy_config_on_edge(p3d_rob *robotPt, p3d_config_on_edge* q_on_edge);
static int p3d_test_loc_face(p3d_rob *robotPt, configPt q0, configPt q1, configPt q2);
static int p3d_sort_config_on_edge(void* q_on_edge1, void* q_on_edge2);
static int p3d_test_col_face(p3d_rob *robotPt, configPt * q_list, double* R_list);
//end path deform


/*! \brief Flag to indicate the microcollisionmode
 *
 * If microcollision_avoidance == FALSE we accept microcollisions
 * but the verification of a localpath is faster
 * default value is FALSE
 *
 *  \internal
 */
static int microcollision_avoidance = FALSE;

/*
 * \brief Set ::microcollision_avoidance to given value
 */
void p3d_col_set_microcollision(int value) {
  microcollision_avoidance = value;
}

/*
 * \brief Get ::microcollision_avoidance value
 */
int p3d_col_get_microcollision() {
  return(microcollision_avoidance);
}


/***************************************************************************/
/*! \brief Place the robot at position describe by localpathPt and
 *         the parameter l
 *
 * \param robotPt:     the robot
 * \param localpathPt: the robot
 * \param l:           the parameter
 *
 * \return Whether robot respect kinematics constraints and
 *         joints limits or not
 *
 *  \internal
 */
#ifndef BIO
static
#endif
int change_position_robot(p3d_rob *robotPt, p3d_localpath *localpathPt,
                          double l) {
  configPt q = NULL;
  int i, j;
  double v, v_min, v_max;
  p3d_jnt * jntPt;

  q = localpathPt->config_at_param(robotPt, localpathPt, l);

  /* s'il y a une contrainte du robot qui est pas respectee pendant le
     chemin local (ex.: si la chaine est pas fermee) */
  if (!p3d_set_and_update_this_robot_conf(robotPt, q)) {
    p3d_destroy_config(robotPt, q);
    return(TRUE);
  }
  p3d_destroy_config(robotPt, q);

  /* test that joints keep in their bounds */
  for (i = 0;  i <= robotPt->njoints; i++) {
    jntPt = robotPt->joints[i];
    for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
      p3d_jnt_get_dof_bounds(jntPt, j, &v_min, &v_max);
      v = p3d_jnt_get_dof(jntPt, j);
      if (((v > v_max + EPS6) || (v < v_min - EPS6)) &&
          !p3d_jnt_is_dof_circular(jntPt, j)) {
        return TRUE;
      }
    }
  }
  return FALSE;
}


// modif Juan
/***************************************************************************/
/*! \brief Place the robot at position describe by localpathPt and
 *         the parameter l without constraints
 *
 * \param robotPt:     the robot
 * \param localpathPt: the robot
 * \param l:           the parameter
 *
 * \return Whether robot respect joints limits or not
 *
 *  \internal
 */
#ifndef BIO
static
#endif
int change_position_robot_without_cntrt(p3d_rob *robotPt, p3d_localpath *localpathPt,
                                        double l) {
  configPt q = NULL;
  int i, j;
  double v, v_min, v_max;
  p3d_jnt * jntPt;

  q = localpathPt->config_at_param(robotPt, localpathPt, l);

  /* s'il y a une contrainte du robot qui est pas respectee pendant le
     chemin local (ex.: si la chaine est pas fermee) */
  p3d_set_robot_config(robotPt, q);
  p3d_update_this_robot_pos_without_cntrt(robotPt);
  p3d_destroy_config(robotPt, q);

  if (DEBUG_COL_TRAJ)
    g3d_draw_allwin_active();


  /* test that joints keep in their bounds */
  for (i = 0;  i <= robotPt->njoints; i++) {
    jntPt = robotPt->joints[i];
    for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
      p3d_jnt_get_dof_bounds(jntPt, j, &v_min, &v_max);
      v = p3d_jnt_get_dof(jntPt, j);
      if (((v > v_max + EPS6) || (v < v_min - EPS6)) &&
          !p3d_jnt_is_dof_circular(jntPt, j)) {
        return TRUE;
      }
    }
  }
  return FALSE;
}


/***************************************************************************/
/*! \brief Place the robot at position describe by localpathPt and
 *         the parameter l without object (only jnts)
 *
 * \param robotPt:     the robot
 * \param localpathPt: the robot
 * \param l:           the parameter
 *
 * \return Whether robot respect kinematics constraints and
 *         joints limits or not
 *
 *  \internal
 */
int change_position_robot_without_obj(p3d_rob *robotPt, p3d_localpath *localpathPt,
                                      double l) {
  configPt q = NULL;
  int i, j;
  double v, v_min, v_max;
  p3d_jnt * jntPt;

  q = localpathPt->config_at_param(robotPt, localpathPt, l);

  /* s'il y a une contrainte du robot qui est pas respectee pendant le
     chemin local (ex.: si la chaine est pas fermee) */
  if (!p3d_check_cntrts_at_conf(robotPt, q)) {
    if (DEBUG_COL_TRAJ) {
      g3d_draw_allwin_active();
    }
    p3d_destroy_config(robotPt, q);
    return(TRUE);
  }
  p3d_destroy_config(robotPt, q);

  if (DEBUG_COL_TRAJ) {
    g3d_draw_allwin_active();
  }

  p3d_destroy_config(robotPt, q);

  /* test that joints keep in their bounds */
  for (i = 0;  i <= robotPt->njoints; i++) {
    jntPt = robotPt->joints[i];
    for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
      p3d_jnt_get_dof_bounds(jntPt, j, &v_min, &v_max);
      v = p3d_jnt_get_dof(jntPt, j);
      if (((v > v_max + EPS6) || (v < v_min - EPS6)) &&
          !p3d_jnt_is_dof_circular(jntPt, j)) {
        return TRUE;
      }
    }
  }
  return FALSE;
}


/***************************************************************************/
/*! \brief Place the robot at position describe by localpathPt and
 *         the parameter l
 *         When several solutions for cntrt are possible, one is chosen
 *         depending on previous config in localpath
 *
 * \param robotPt:     the robot
 * \param localpathPt: the robot
 * \param l:           the parameter
 * \param dl:          the increment from previous config.
 *
 * \return Whether robot respect(FALSE) kinematics constraints and
 *         joints limits or not(TRUE)
 *
 *  \internal
 */
int change_position_robot_multisol(p3d_rob *robotPt, p3d_localpath *localpathPt,
                                   double l, double dl, configPt qp) {
  configPt q = NULL;
  int i, j;
  double v, v_min, v_max;
  p3d_jnt * jntPt;

  q = localpathPt->config_at_param(robotPt, localpathPt, l);

  /* s'il y a une contrainte du robot qui est pas respectee pendant le
     chemin local (ex.: si la chaine est pas fermee) */
  if (!p3d_set_and_update_this_robot_conf_multisol(robotPt, q, qp, dl, localpathPt->ikSol)) {
    if (DEBUG_COL_TRAJ) {
      g3d_draw_allwin_active();
    }
    p3d_destroy_config(robotPt, q);
    return(TRUE);
  }
  p3d_destroy_config(robotPt, q);

  if (DEBUG_COL_TRAJ) {
    g3d_draw_allwin_active();
  }

  /* test that joints keep in their bounds */
  for (i = 0;  i <= robotPt->njoints; i++) {
    jntPt = robotPt->joints[i];
    for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
      p3d_jnt_get_dof_bounds(jntPt, j, &v_min, &v_max);
      v = p3d_jnt_get_dof(jntPt, j);
      if (((v > v_max + EPS6) || (v < v_min - EPS6)) &&
          !p3d_jnt_is_dof_circular(jntPt, j)) {
        return TRUE;
      }
    }
  }
  return FALSE;
}


/***************************************************************************/
/*! \brief Place the robot at position describe by localpathPt and
 *         the parameter l without object (only jnts)
 *         When several solutions for cntrt are possible, one is chosen
 *         depending on previous config in localpath
 *
 * \param robotPt:     the robot
 * \param localpathPt: the robot
 * \param l:           the parameter
 * \param dl:          the increment from previous config.
 *
 * \return Whether robot respect kinematics constraints and
 *         joints limits or not
 *
 *  \internal
 */
int change_position_robot_without_obj_multisol(p3d_rob *robotPt, p3d_localpath *localpathPt,
    double l, double dl, configPt qp) {
  configPt q = NULL;
  int i, j;
  double v, v_min, v_max;
  p3d_jnt * jntPt;

  q = localpathPt->config_at_param(robotPt, localpathPt, l);

  /* s'il y a une contrainte du robot qui est pas respectee pendant le
     chemin local (ex.: si la chaine est pas fermee) */
  if (!p3d_check_cntrts_at_conf_multisol(robotPt, q, qp, dl)) {
    if (DEBUG_COL_TRAJ) {
      p3d_update_this_robot_pos_without_cntrt(robotPt);
      g3d_draw_allwin_active();
    }
    p3d_destroy_config(robotPt, q);
    return(TRUE);
  }

  if (DEBUG_COL_TRAJ) {
    p3d_update_this_robot_pos_without_cntrt(robotPt);
    g3d_draw_allwin_active();
  }

  p3d_destroy_config(robotPt, q);

  /* test that joints keep in their bounds */
  for (i = 0;  i <= robotPt->njoints; i++) {
    jntPt = robotPt->joints[i];
    for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
      p3d_jnt_get_dof_bounds(jntPt, j, &v_min, &v_max);
      v = p3d_jnt_get_dof(jntPt, j);
      if (((v > v_max + EPS6) || (v < v_min - EPS6)) &&
          !p3d_jnt_is_dof_circular(jntPt, j)) {
        return TRUE;
      }
    }
  }
  return FALSE;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS TO MANAGE INFORMATION ABOUT INVALID LOCAL-PATH CONFIGURATIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////////

static int there_is_a_current_q_inv = FALSE;

static void p3d_reset_current_q_inv(p3d_rob *robotPt) {
  there_is_a_current_q_inv = FALSE;
}

void p3d_set_current_q_inv(p3d_rob *robotPt, p3d_localpath *localpathPt, configPt q_inv) {
  there_is_a_current_q_inv = TRUE;
  p3d_copy_config_into(robotPt, q_inv, &(robotPt->currect_q_inv));
}

int p3d_get_current_q_inv(p3d_rob *robotPt, configPt q_invPt) {
  if (there_is_a_current_q_inv) {
    p3d_copy_config_into(robotPt, robotPt->currect_q_inv, &q_invPt);
  }
  return(there_is_a_current_q_inv);
}


/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

int p3d_col_test_localpath_classic(p3d_rob *robotPt,
                                   p3d_localpath *localpathPt,
                                   int *ntest, double *Kpath,
                                   configPt *q_atKpath) {
  int unvalid;

  /*   if(p3d_actived_cntrts(robotPt->cntrt_manager)) { */
  /*     unvalid = p3d_col_and_cntrts_test_localpath_classic(robotPt, localpathPt, ntest, Kpath, q_atKpath); */
  /*   } */
  /*   else { */
  unvalid = p3d_onlycol_test_localpath_classic(robotPt, localpathPt, ntest, Kpath, q_atKpath);
  /*   } */

  return (unvalid);
}


int p3d_col_test_localpath_classic_multisol(p3d_rob *robotPt,
    p3d_localpath *localpathPt,
    int *ntest, double *Kpath,
    configPt *q_atKpath) {

  int unvalid;

#ifdef BIO
  extern int bio_col_test_localpath_step(p3d_rob *robotPt, p3d_localpath *localpathPt, int *ntest, double *Kpath, configPt *q_atKpath);

  if (p3d_col_get_mode() == p3d_col_mode_bio) {
    unvalid = bio_col_test_localpath_step(robotPt, localpathPt, ntest, Kpath, q_atKpath);
    return(unvalid);
  }
#endif

  /*   if(p3d_actived_cntrts(robotPt->cntrt_manager)) { */
  /*     unvalid = p3d_col_and_cntrts_test_localpath_classic_multisol(robotPt, localpathPt, ntest, Kpath, q_atKpath); */
  /*   } */
  /*   else { */
  unvalid = p3d_onlycol_test_localpath_classic(robotPt, localpathPt, ntest, Kpath, q_atKpath);
  /*   } */

  return (unvalid);
}



/* NOTE :   Juan
   Functions of collision&constraints validation are only made for classic (sequential) test.
   Similar functions can be made for dichotimic test (i.e. testing constraints
   with constatnt step between two collision tests if necessary ).
*/


/*--------------------------------------------------------------------------*/
/*! \brief Collision and Constraints checking of a local path
 *
 *  \param  robotPt:     the robot
 *  \param  localpathPt: the local path
 *
 *  \retval ntest: the sum of collision checking tests
 *  \retval Kpath: the configuration at the parameter \a Kpath *
 *                 localpathPt::range_param is the last collision free
 *                 or valid (for cntrts) tested configuration on the path.
 *
 *  \return Whether valid or not
 *
 *  Description: Compute the distances between the bounding boxes of
 *          each obstacle and the bounding boxes of the environment.
 *          If these distances are positive, go forward along the
 *          path in such a way that each point of each body does not
 *          move by more than the distance between the body and the
 *          environment.
 *          If one of these distance is zero, go forward along the
 *          in such a way that no point of the robot moves more than
 *          2*dmax.
 *          Constraints are systematically tested at least each 2*dmax
 */
int p3d_col_and_cntrts_test_localpath_classic(p3d_rob *robotPt,
    p3d_localpath *localpathPt,
    int *ntest, double *Kpath,
    configPt *q_atKpath) {
  double u = 0., du, umax; /* parameters along the local path */
  double u_ct, u_ct_p, du_ct;
  //configPt qsave;
  configPt qp;
  int njnt = robotPt->njoints;
  double *distances, *distances_ct;
  double tolerance, newtol, dmax, dist0, dist_ct;
  int end_localpath = 0;
  int j;
  int test;

  *Kpath = 0;
  p3d_col_get_dmax(&dmax);
  if (localpathPt == NULL) {
    return FALSE;
  }

  /* Some curves can be decided unvalid by the user */
  if (localpathPt->valid == FALSE) {
    return TRUE;
  }

  /* we fix the step for checking cntrts at 2*dmax */
  dist_ct = 2.0 * dmax;

  umax = localpathPt->range_param;

  if (umax < EPS6) {
    return(FALSE);
  }

  distances = MY_ALLOC(double, njnt + 1);
  distances_ct = MY_ALLOC(double, njnt + 1);
  if (p3d_col_get_tolerance(&tolerance) && microcollision_avoidance) {
    p3d_col_set_tolerance(tolerance + dmax);
    newtol = tolerance + dmax;
    /* With microcollision_avoidance the first and the last config could
       be at a distance  between tolerance and tolerance+dmax, we want the
       next config to be at more than tolerance+dmax from obstacle, and we
       want to avoid any config at less than tolerance, so we could only
       cross dmax from the bounds. */
    dist0 = dmax;
  } else {
    /* Without microcollision_avoidance the first and the last config are
       valid (a distance more than tolerance), we want the next config to be
       valid and we want to avoid any config at less than tolerance - dmax
       from the obstacles, so we could cross 2*dmax from the bounds. */
    newtol = 0;
    dist0 = 2 * dmax;
  }

  /* current position of robot is saved */
  //qp = p3d_copy_config(robotPt, localpathPt->specific.lin_data->q_init);
  qp = localpathPt->config_at_param(robotPt, localpathPt, 0.0);
  p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qp);

  /* We suppose that the fisrt and the last configurations are valid so
     we could test the configuration at dist0 from the bounds */

  u = 0;

  for (j = 0; j <= njnt; j++) {
    distances[j] = dist0;
    distances_ct[j] = dist0;   // same init. for col and cntrts
  }
  // WARNING : functions stay_within_dist do not consider constraints !!!
  //           these functions should be modified
  du = localpathPt->stay_within_dist(robotPt, localpathPt, u,
                                     FORWARD, distances);
  du_ct = du;
  u_ct_p = 0.0;
  u = u_ct = du;
  if (u > umax - EPS6) {
    u = umax;
    end_localpath++;
  }
  dist0 = 2 * dmax - newtol; /* Minimal distance we could cross at each step */

  while (end_localpath < 2) {
    while (u_ct < u) {
      if (change_position_robot_without_obj_multisol(robotPt, localpathPt, u_ct, du_ct, qp)) {
        /* The last valid conf. of the robot is recovered (NEW !) */
        /* printf("change position(u_ct < u) -> TRUE\n"); */
        p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qp);
        //p3d_destroy_config(robotPt, qsave);
        p3d_destroy_config(robotPt, qp);
        MY_FREE(distances, double, njnt + 1);
        MY_FREE(distances_ct, double, njnt + 1);
        p3d_col_set_tolerance(tolerance);
        return TRUE;
      }
      p3d_get_robot_config_into(robotPt, &qp);

      /* supose that there is not collision before u !!! */
      *Kpath = u_ct / localpathPt->range_param;

      if (q_atKpath != NULL)
        p3d_get_robot_config_into(robotPt, q_atKpath);

      for (j = 0; j <= njnt; j++) {
        distances_ct[j] += dist_ct;
      }
      // WARNING : functions stay_within_dist do not consider constraints !!!
      //           these functions should be modified
      du_ct = localpathPt->stay_within_dist(robotPt, localpathPt,
                                            u_ct, FORWARD, distances_ct);
      u_ct_p = u_ct;
      u_ct += du_ct;
    }
    u_ct = u;
    for (j = 0; j <= njnt; j++) {
      distances_ct[j] = distances[j];
    }

    /* position of the robot corresponding to parameter u */
    if (change_position_robot_multisol(robotPt, localpathPt, u, (u - u_ct_p), qp)) {
      /* The last valid conf. of the robot is recovered (NEW !) */
      /* printf("change position -> TRUE\n"); */
      p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qp);
      //p3d_destroy_config(robotPt, qsave);
      p3d_destroy_config(robotPt, qp);
      MY_FREE(distances, double, njnt + 1);
      MY_FREE(distances_ct, double, njnt + 1);
      p3d_col_set_tolerance(tolerance);
      return TRUE;
    }
    /* p3d_set_and_update_robot_conf(q); */ /* <- avant modif */

    /*     printf("change position -> FALSE\n"); */
    p3d_BB_dist_robot(robotPt, distances);
    test = FALSE;
    for (j = 0; j <= njnt; j++) {
      if (distances[j] < newtol + EPS6) {
        test = TRUE;
      }
      distances[j] += dist0;
    }
    if (test) {

      /* collision checking */
      *ntest = *ntest + 1;
      if (p3d_col_test()) {
        /*  printf("col test -> TRUE\n"); */
        /* The last valid conf. of the robot is recovered (NEW !) */
        p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qp);
        //p3d_destroy_config(robotPt, qsave);
        p3d_destroy_config(robotPt, qp);
        MY_FREE(distances, double, njnt + 1);
        MY_FREE(distances_ct, double, njnt + 1);
        p3d_col_set_tolerance(tolerance);
        return TRUE;
      }
      /* Modif. Etienne: if collision detector computed distances
      in call to p3d_col_test(), we exploit them */
      if (p3d_col_report_distance(robotPt, distances)) {
        for (j = 0; j <= njnt; j++) {
          distances[j] += dist0;
        }
      }
    }
    p3d_get_robot_config_into(robotPt, &qp);

    *Kpath = u / localpathPt->range_param;

    if (q_atKpath != NULL)
      p3d_get_robot_config_into(robotPt, q_atKpath);

    for (j = 0; j <= njnt; j++) {
      distances_ct[j] += dist_ct;
    }
    // WARNING : functions stay_within_dist do not consider constraints !!!
    //           these functions should be modified
    du_ct = localpathPt->stay_within_dist(robotPt, localpathPt,
                                          u_ct, FORWARD, distances_ct);
    u_ct = u + du_ct;

    // WARNING : functions stay_within_dist do not consider constraints !!!
    //           these functions should be modified
    du = localpathPt->stay_within_dist(robotPt, localpathPt,
                                       u, FORWARD, distances);
    u += du;

    if (u > umax - EPS6) {
      u = umax;
      end_localpath++;
    }
  }
  *Kpath = 1.0;

  /* modify q_end in localpath (if multisol) */
  // WARNING : supoose that the local path is linear !!!
  p3d_get_robot_config_into(robotPt, &localpathPt->specific.lin_data->q_end);
  if (q_atKpath != NULL) {
    p3d_copy_config_into(robotPt, localpathPt->specific.lin_data->q_end, q_atKpath);
  }

  /* The last valid conf. of the robot is recovered (NEW !) */
  p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, *q_atKpath);
  //p3d_destroy_config(robotPt, qsave);
  p3d_destroy_config(robotPt, qp);

  MY_FREE(distances, double, njnt + 1);
  MY_FREE(distances_ct, double, njnt + 1);
  p3d_col_set_tolerance(tolerance);
  return FALSE;
}

int p3d_col_and_cntrts_test_localpath_classic_multisol(p3d_rob *robotPt,
    p3d_localpath *localpathPt,
    int *ntest, double *Kpath,
    configPt *q_atKpath) {
  double u = 0., du, umax; /* parameters along the local path */
  double u_ct, u_ct_p, du_ct;
  //configPt qsave;
  configPt qp;
  int njnt = robotPt->njoints;
  double *distances, *distances_ct;
  double tolerance, newtol, dmax, dist0, dist_ct;
  int end_localpath = 0;
  int j;
  int test;

  p3d_reset_current_q_inv(robotPt);

  *Kpath = 0;
  p3d_col_get_dmax(&dmax);
  if (localpathPt == NULL) {
    return FALSE;
  }

  /* Some curves can be decided unvalid by the user */
  if (localpathPt->valid == FALSE) {
    return TRUE;
  }

  /* we fix the step for checking cntrts at 2*dmax */
  dist_ct = 2.0 * dmax;

  umax = localpathPt->range_param;

  if (umax < EPS6) {
    return(FALSE);
  }

  distances = MY_ALLOC(double, njnt + 1);
  distances_ct = MY_ALLOC(double, njnt + 1);
  if (p3d_col_get_tolerance(&tolerance) && microcollision_avoidance) {
    p3d_col_set_tolerance(tolerance + dmax);
    newtol = tolerance + dmax;
    /* With microcollision_avoidance the first and the last config could
       be at a distance  between tolerance and tolerance+dmax, we want the
       next config to be at more than tolerance+dmax from obstacle, and we
       want to avoid any config at less than tolerance, so we could only
       cross dmax from the bounds. */
    dist0 = dmax;
  } else {
    /* Without microcollision_avoidance the first and the last config are
       valid (a distance more than tolerance), we want the next config to be
       valid and we want to avoid any config at less than tolerance - dmax
       from the obstacles, so we could cross 2*dmax from the bounds. */
    newtol = 0;
    dist0 = 2 * dmax;
  }


  /* current position of robot is saved */
  //qsave = p3d_get_robot_config(robotPt);

  //qp = p3d_alloc_config(robotPt);
  qp = localpathPt->config_at_param(robotPt, localpathPt, 0.0);
  p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qp);

  /* We suppose that the fisrt and the last configurations are valid so
     we could test the configuration at dist0 from the bounds */

  u = 0;

  for (j = 0; j <= njnt; j++) {
    distances[j] = dist0;
    distances_ct[j] = dist0;   // same init. for col and cntrts
  }
  // WARNING : functions stay_within_dist do not consider constraints !!!
  //           these functions should be modified
  du = localpathPt->stay_within_dist(robotPt, localpathPt, u,
                                     FORWARD, distances);
  du_ct = du;
  u_ct_p = 0.0;
  u = u_ct = du;
  if (u > umax - EPS6) {
    u = umax;
    end_localpath++;
  }
  dist0 = 2 * dmax - newtol; /* Minimal distance we could cross at each step */

  while (end_localpath < 2) {
    while (u_ct < u) {
      if (change_position_robot_without_obj_multisol(robotPt, localpathPt, u_ct, du_ct, qp)) {
        /* The last valid conf. of the robot is recovered (NEW !) */
        /*  printf("change position(u_ct < u) -> TRUE\n"); */
        p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qp);
        //p3d_destroy_config(robotPt, qsave);
        p3d_destroy_config(robotPt, qp);
        MY_FREE(distances, double, njnt + 1);
        MY_FREE(distances_ct, double, njnt + 1);
        p3d_col_set_tolerance(tolerance);
        return TRUE;
      }
      p3d_get_robot_config_into(robotPt, &qp);

      /* supose that there is not collision before u !!! */
      *Kpath = u_ct / localpathPt->range_param;

      if (q_atKpath != NULL)
        p3d_get_robot_config_into(robotPt, q_atKpath);

      for (j = 0; j <= njnt; j++) {
        distances_ct[j] += dist_ct;
      }
      // WARNING : functions stay_within_dist do not consider constraints !!!
      //           these functions should be modified
      du_ct = localpathPt->stay_within_dist(robotPt, localpathPt,
                                            u_ct, FORWARD, distances_ct);
      u_ct_p = u_ct;
      u_ct += du_ct;
    }
    u_ct = u;
    for (j = 0; j <= njnt; j++) {
      distances_ct[j] = distances[j];
    }

    /* position of the robot corresponding to parameter u */
    if (change_position_robot_multisol(robotPt, localpathPt, u, (u - u_ct_p), qp)) {
      /* The last valid conf. of the robot is recovered (NEW !) */
      /*       printf("change position -> TRUE\n"); */
      p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qp);
      //p3d_destroy_config(robotPt, qsave);
      p3d_destroy_config(robotPt, qp);
      MY_FREE(distances, double, njnt + 1);
      MY_FREE(distances_ct, double, njnt + 1);
      p3d_col_set_tolerance(tolerance);
      return TRUE;
    }
    /* p3d_set_and_update_robot_conf(q); */ /* <- avant modif */

    /*     printf("change position -> FALSE\n"); */

    p3d_BB_dist_robot(robotPt, distances);
    test = FALSE;
    for (j = 0; j <= njnt; j++) {
      if (distances[j] < newtol + EPS6) {
        test = TRUE;
      }
      distances[j] += dist0;
    }
    if (test) {

      /* collision checking */
      *ntest = *ntest + 1;
      if (p3d_col_test()) {
        /* The last valid conf. of the robot is recovered (NEW !) */
        p3d_set_current_q_inv(robotPt, localpathPt, qp);
        p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qp);
        //p3d_destroy_config(robotPt, qsave);
        p3d_destroy_config(robotPt, qp);
        MY_FREE(distances, double, njnt + 1);
        MY_FREE(distances_ct, double, njnt + 1);
        p3d_col_set_tolerance(tolerance);
        return TRUE;
      }
      /* Modif. Etienne: if collision detector computed distances
      in call to p3d_col_test(), we exploit them */
      if (p3d_col_report_distance(robotPt, distances)) {
        for (j = 0; j <= njnt; j++) {
          distances[j] += dist0;
        }
      }
    }
    p3d_get_robot_config_into(robotPt, &qp);

    *Kpath = u / localpathPt->range_param;

    if (q_atKpath != NULL)
      p3d_get_robot_config_into(robotPt, q_atKpath);

    for (j = 0; j <= njnt; j++) {
      distances_ct[j] += dist_ct;
    }
    // WARNING : functions stay_within_dist do not consider constraints !!!
    //           these functions should be modified
    du_ct = localpathPt->stay_within_dist(robotPt, localpathPt,
                                          u_ct, FORWARD, distances_ct);
    u_ct = u + du_ct;

    // WARNING : functions stay_within_dist do not consider constraints !!!
    //           these functions should be modified
    du = localpathPt->stay_within_dist(robotPt, localpathPt,
                                       u, FORWARD, distances);
    u += du;

    if (u > umax - EPS6) {
      u = umax;
      end_localpath++;
    }
  }
  *Kpath = 1.0;

  /* modify q_end in localpath (if multisol) */
  // WARNING : supoose that the local path is linear !!!
  p3d_get_robot_config_into(robotPt, &localpathPt->specific.lin_data->q_end);
  if (q_atKpath != NULL) {
    p3d_copy_config_into(robotPt, localpathPt->specific.lin_data->q_end, q_atKpath);
  }

  /* The last valid conf. of the robot is recovered (NEW !) */
  p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, *q_atKpath);

  //p3d_destroy_config(robotPt, qsave);
  p3d_destroy_config(robotPt, qp);

  MY_FREE(distances, double, njnt + 1);
  MY_FREE(distances_ct, double, njnt + 1);
  p3d_col_set_tolerance(tolerance);
  return FALSE;
}

//! Tests the continuity between two configurations (that are supposed to be two successive configurations along a path)
int p3d_test_config_continuity(p3d_rob *robotPt, configPt qprev, configPt qcur)
{
  int i = 0;
  double xprev, xcur, xmin, xmax, dx, xf;
  double xcurm, xprevm, xminm, xmaxm;

  for(i=0; i<robotPt->njoints; i++) {
    if(robotPt->joints[i]->type!=P3D_ROTATE) {
      continue;
    }

    xmin= robotPt->joints[i]->dof_data[0].vmin;
    xmax= robotPt->joints[i]->dof_data[0].vmax;

   // if(dist_circle(xmin, xmax) < EPS6 ) {
   //	continue;
   //  }

    xprev= qprev[robotPt->joints[i]->index_dof];
    xcur = qcur[robotPt->joints[i]->index_dof];
    dx = fmod(xcur-xprev, 2*M_PI);
    xf = xprev + dx;

    if( (xcur<xmin) && (xmin<xf) )
    {
      return TRUE;
    }
    if( (xf<xmin) && (xmin<xcur) )
    {
      return TRUE;
    }
    if( (xcur<xmax) && (xmax<xf) )
    {
      return TRUE;
    }
    if( (xf<xmax) && (xmax<xcur) )
    {
      return TRUE;
    }
    qcur[robotPt->joints[i]->index_dof] = xf;
  } // end for
  return FALSE;
}

int p3d_test_localpath_pb_continuity(p3d_rob *robotPt, p3d_localpath *localpathPt)
{
  double param = 0.0;
  configPt qp = NULL, qprev = NULL;
  int I_can = 0;

  qprev = localpathPt->config_at_param(robotPt, localpathPt, 0.0);

  for(param=0.0001; param<= localpathPt->length_lp; param=param+0.0001) {

    qp = localpathPt->config_at_distance(robotPt, localpathPt, param);
    I_can = p3d_set_and_update_this_robot_conf_multisol(robotPt, qp, NULL, 0, localpathPt->ikSol);
    if(I_can == FALSE) {return TRUE;}

    if(p3d_test_config_continuity(robotPt, qprev, qp)==TRUE) {
      p3d_destroy_config(robotPt, qprev);
      qprev = NULL;
      p3d_destroy_config(robotPt, qp);
      qp = NULL;
      return TRUE;
    }

    p3d_destroy_config(robotPt, qprev);
    qprev = NULL;
    qprev= p3d_copy_config(robotPt, qp);
    p3d_destroy_config(robotPt, qp);
    qp = NULL;
  }
  return FALSE;
}

/*--------------------------------------------------------------------------*/
/*! \brief Collision checking of a local path
 *
 *  \param  robotPt:     the robot
 *  \param  localpathPt: the local path
 *
 *  \retval ntest: the sum of collision checking tests
 *  \retval Kpath: the configuration at the parameter \a Kpath *
 *                 localpathPt::range_param is the last collision free
 *                 tested configuration on the path.
 *
 *  \return Whether collision or not
 *
 *  Description: Compute the distances between the bounding boxes of
 *          each obstacle and the bounding boxes of the environment.
 *          If these distances are positive, go forward along the
 *          path in such a way that each point of each body does not
 *          move by more than the distance between the body and the
 *          environment.
 *          If one of these distance is zero, go forward along the
 *          in such a way that no point of the robot moves more than
 *          2*dmax.
 */
int p3d_onlycol_test_localpath_classic(p3d_rob *robotPt,
                                       p3d_localpath *localpathPt,
                                       int *ntest, double *Kpath,
                                       configPt *q_atKpath) {
  double u = 0., du, umax; /* parameters along the local path */
  //configPt qsave;
  configPt qp;
  int njnt = robotPt->njoints;
  double *distances;
  double tolerance, newtol, dmax, dist0;
  int end_localpath = 0;
  int j;

  p3d_reset_current_q_inv(robotPt);

  *Kpath = 0;
  p3d_col_get_dmax(&dmax);
  if (localpathPt == NULL) {
    return FALSE;
  }

  /* Some curves can be decided unvalid by the user */
  if (localpathPt->valid == FALSE) {
    return TRUE;
  }

  umax = localpathPt->range_param;
  distances = MY_ALLOC(double, njnt + 1);
  if (p3d_col_get_tolerance(&tolerance) && microcollision_avoidance) {
    p3d_col_set_tolerance(tolerance + dmax);
    newtol = tolerance + dmax;
    /* With microcollision_avoidance the first and the last config could
       be at a distance  between tolerance and tolerance+dmax, we want the
       next config to be at more than tolerance+dmax from obstacle, and we
       want to avoid any config at less than tolerance, so we could only
       cross dmax from the bounds. */
    dist0 = dmax;
  } else {
    /* Without microcollision_avoidance the first and the last config are
       valid (a distance more than tolerance), we want the next config to be
       valid and we want to avoid any config at less than tolerance - dmax
       from the obstacles, so we could cross 2*dmax from the bounds. */
    newtol = 0;
    dist0 = 2 * dmax;
  }

  /* current position of robot is saved */
  qp = localpathPt->config_at_param(robotPt, localpathPt, 0.0);

  /* We suppose that the fisrt and the last configurations are valid so
     we could test the configuration at dist0 from the bounds */
  /* Remove the end of interval */
  u = umax;
  if (change_position_robot_without_cntrt(robotPt, localpathPt, u)) {   // modif Juan
    /* The initial position of the robot is recovered */
    p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qp);
    p3d_destroy_config(robotPt, qp);
    MY_FREE(distances, double, njnt + 1);
    p3d_col_set_tolerance(tolerance);
    return TRUE;
  }
  for (j = 0; j <= njnt; j++) {
    distances[j] = dist0;
  }
  /* Compute the lenght of left interval */
  /* Compute the lenght of left interval */
  /* Modif EF: the end of the path is not removed for special use of the RRT algorithm */
  /*
  du = localpathPt->stay_within_dist(robotPt, localpathPt, u,
         BACKWARD, distances);
  umax = umax - du;
  */
  if (umax < EPS6) {
    /* The initial position of the robot is recovered */
    p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qp);
    p3d_destroy_config(robotPt, qp);
    MY_FREE(distances, double, njnt + 1);
    p3d_col_set_tolerance(tolerance);
    return(FALSE);
  }

  /* Remove the beginning of interval */
  u = 0;
  if (change_position_robot(robotPt, localpathPt, u)) {
    /* The initial position of the robot is recovered */
    p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qp);
    p3d_destroy_config(robotPt, qp);
    MY_FREE(distances, double, njnt + 1);
    p3d_col_set_tolerance(tolerance);
    return TRUE;
  }
  for (j = 0; j <= njnt; j++) {
    distances[j] = dist0;
  }

  du = localpathPt->stay_within_dist(robotPt, localpathPt, u,
                                     FORWARD, distances);
  u = du;
  if (u > umax - EPS6) {
    u = umax;
    end_localpath++;
  }
  dist0 = 2 * dmax - newtol; /* Minimal distance we could cross at each step */

  while (end_localpath < 2) {

    /* position of the robot corresponding to parameter u */
    if (change_position_robot(robotPt, localpathPt, u)) {
      /* The initial position of the robot is recovered */
      p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qp);
      //p3d_destroy_config(robotPt, qsave);
      p3d_destroy_config(robotPt, qp);
      MY_FREE(distances, double, njnt + 1);
      p3d_col_set_tolerance(tolerance);
      return TRUE;
    }
    p3d_get_robot_config_into(robotPt, &qp);

    // TEMP MODIF : PROBLEM WITH SELF COLLISION : CONSTANT STEP
         p3d_BB_dist_robot(robotPt, distances);
         int test = FALSE;
         for (j=0; j<=njnt; j++) {
           if (distances[j] < newtol + EPS6)
           {
        	   test = TRUE;
        	   }
           distances[j] += dist0;
         }
         if (test) {
    /////////////////////////

		/* collision checking */
		*ntest = *ntest + 1;
		if (p3d_col_test()) {
		  /* The initial position of the robot is recovered */
		  p3d_set_current_q_inv(robotPt, localpathPt, qp);
		  p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qp);
		  //p3d_destroy_config(robotPt, qsave);
		  p3d_destroy_config(robotPt, qp);
		  MY_FREE(distances, double, njnt + 1);
		  p3d_col_set_tolerance(tolerance);
		  return TRUE;
		}
    }

    // TEMP MODIF ///////
    /* Modif. Etienne: if collision detector computed distances
    in call to p3d_col_test(), we exploit them */

//	  if(p3d_col_report_distance(robotPt,distances)){
//      for (j=0; j<=njnt; j++)
//        {
//    	  distances[j] += dist0;
//    	  }
//	  }
    //////////////////
//    for (j = 0; j <= njnt; j++) {
//      distances[j] += dist0; // Attention += test
//    }
    /////////////////

    *Kpath = u / localpathPt->range_param;

    if (q_atKpath != NULL)
      p3d_get_robot_config_into(robotPt, q_atKpath);

//    for (j = 0; j <= njnt; j++) {
//    	printf("distances[%j] = %f\n",distances[j]);
//        }

    du = localpathPt->stay_within_dist(robotPt, localpathPt,
                                       u, FORWARD, distances);

//    printf("du[%d] = %f\n",loop++,du);

    u += du;
    if (u > umax - EPS6) {
      u = umax;
      end_localpath++;
    }

  }
//  printf("\n");

  *Kpath = 1.0;

  // WARNING : NEXT LINES ARE VALID IN THE CASE OF CONSTRAINTS ???
  if (q_atKpath != NULL) {
    p3d_destroy_config(robotPt, qp);
    qp = localpathPt->config_at_param(robotPt, localpathPt, u);
    p3d_copy_config_into(robotPt, qp, q_atKpath);
  }
  ///////////

  /* The initial position of the robot is recovered */
  //p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qsave);
  //p3d_destroy_config(robotPt, qsave);
  p3d_destroy_config(robotPt, qp);

  MY_FREE(distances, double, njnt + 1);
  p3d_col_set_tolerance(tolerance);
  return FALSE;
}


// fmodif Juan

/*--------------------------------------------------------------------------*/
/*! \brief Collision checking of a trajectory
 *
 *  \param  robotPt:     the robot
 *  \param  localpathPt: the first local path
 *
 *  \retval ntest: the sum of collision checking tests
 *
 *  \return Whether collision or not
 *
 *  Description: successively check all the local paths of a global path.
 *
 *  \internal
 */
static int p3d_col_test_traj_classic(p3d_rob *robotPt,
                                     p3d_localpath *localpathPt, int *ntest) {
  int njnt, test, j;
  configPt qsave;
  double *distances, Kpath;
  double dmax, newtol, tolerance;

  if (localpathPt == NULL) {
    return FALSE;
  }

  qsave = p3d_get_robot_config(robotPt);
  njnt = robotPt->njoints;

  /* With trajectory we need to check the bounds of localpath */
  if (change_position_robot(robotPt, localpathPt, 0)) {
    p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qsave);
    p3d_destroy_config(robotPt, qsave);
    return TRUE;
  }
  p3d_col_get_dmax(&dmax);
  if (p3d_col_get_tolerance(&tolerance) && microcollision_avoidance) {
    newtol = tolerance + dmax;
  } else {
    newtol = 0;
  }
  distances = MY_ALLOC(double, njnt + 1);

  p3d_BB_dist_robot(robotPt, distances);
  test = FALSE;
  for (j = 0; j <= njnt; j++) {
    if (distances[j] < newtol + EPS6) {
      test = TRUE;
      break;
    }
  }
  if (test) {
    /* collision checking */
    *ntest = *ntest + 1;
    if (p3d_col_test()) {
      /* The initial position of the robot is recovered */
      p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qsave);
      p3d_destroy_config(robotPt, qsave);
      MY_FREE(distances, double, njnt + 1);
      return TRUE;
    }
  }

  while (localpathPt != NULL) {
    if (p3d_col_test_localpath_classic(robotPt, localpathPt, ntest, &Kpath, NULL)) {  // <- modif Juan
      /* The initial position of the robot is recovered */
      p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qsave);
      p3d_destroy_config(robotPt, qsave);
      MY_FREE(distances, double, njnt + 1);
      return TRUE;
    }
    /* With trajectory we need to check the bounds of localpath */
    if (change_position_robot(robotPt, localpathPt, localpathPt->range_param)) {
      p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qsave);
      p3d_destroy_config(robotPt, qsave);
      MY_FREE(distances, double, njnt + 1);
      return TRUE;
    }
    p3d_BB_dist_robot(robotPt, distances);
    test = FALSE;
    for (j = 0; j <= njnt; j++) {
      if (distances[j] < newtol + EPS6) {
        test = TRUE;
        break;
      }
    }
    if (test) {
      /* collision checking */
      *ntest = *ntest + 1;
      if (p3d_col_test()) {
        /* The initial position of the robot is recovered */
        p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qsave);
        p3d_destroy_config(robotPt, qsave);
        MY_FREE(distances, double, njnt + 1);
        return TRUE;
      }
    }

    localpathPt = localpathPt->next_lp;
  }
  p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qsave);
  p3d_destroy_config(robotPt, qsave);
  MY_FREE(distances, double, njnt + 1);
  return FALSE;
}


/*--------------------------------------------------------------------------*/
/*! \brief  Collision checking of a path for auto-collision
 *
 *  \param robotPt:     the robot
 *  \param localpathPt: the local path
 *
 *  \retval ntest: the number of collision checking tests
 *
 *  \return whether collision or not
 *
 *  Description: Compute the distances between the bounding boxes of
 *          each obstacle and the bounding boxes of the environment.
 *          If these distances are positive, go forward along the
 *          path in such a way that each point of each body does not
 *          move by more than the distance between the body and the
 *          environment.
 *          If one of these distance is zero, go forward along the
 *          in such a way that no point of the robot moves more than
 *          2*dmax.
 *
 *  \note With autocolision we have to divide distance by 2. In fact both
 *        body could cross the free distance between themselves. So each
 *        one must only cross half the way to avoid collision.
 *
 *  \internal
 */
static int p3d_col_test_localpath_autocol(p3d_rob *robotPt,
    p3d_localpath *localpathPt,
    int *ntest) {
  double u = 0., du, umax; /* parameters along the local path */
  configPt qsave;
  int njnt = robotPt->njoints;
  double *distances;
  double tolerance, newtol, dmax, dist0;
  int end_localpath = 0;
  int j;
  int test;

  p3d_col_get_dmax(&dmax);
  if (localpathPt == NULL) {
    return FALSE;
  }
  /* Some curves can be decided unvalid by the user */
  if (localpathPt->valid == FALSE) {
    return TRUE;
  }

  umax = localpathPt->range_param;
  distances = MY_ALLOC(double, njnt + 1);
  if (p3d_col_get_tolerance(&tolerance) && microcollision_avoidance) {
    /* microcollision_avoidance can be very difficult. With autocolision
       the bodies are very close to each other. The p3d_col_test could
       fail too ofen */
    p3d_col_set_tolerance(tolerance + dmax);
    newtol = tolerance + dmax;
    /* With microcollision_avoidance the first and the last config could
       be at a distance  between tolerance and tolerance+dmax, we want the
       next config to be at more than tolerance+dmax from obstacle, and we
       want to avoid any config at less than tolerance, so we could only
       cross dmax from the bounds. With autocolision it is only dmax/2 */
    dist0 = dmax / 2;
  } else {
    /* Without microcollision_avoidance the first and the last config are
       valid (a distance more than tolerance), we want the next config to be
       valid and we want to avoid any config at less than tolerance - dmax
       from the obstacles, so we could cross 2*dmax from the bounds.
       With autocolision it is only dmax. */
    newtol = 0;
    dist0 = dmax;
  }

  /* current position of robot is saved */
  qsave = p3d_get_robot_config(robotPt);

  /* We suppose that the fisrt and the last configurations are valid so
     we could test the configuration at dist0 from the bounds */
  /* Remove the end of interval */
  u = umax;
  if (change_position_robot(robotPt, localpathPt, u)) {
    /* The initial position of the robot is recovered */
    p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qsave);
    p3d_destroy_config(robotPt, qsave);
    MY_FREE(distances, double, njnt + 1);
    p3d_col_set_tolerance(tolerance);
    return TRUE;
  }
  for (j = 0; j <= njnt; j++) {
    distances[j] = dist0;
  }
  /* Compute the lenght of left interval */
  du = localpathPt->stay_within_dist(robotPt, localpathPt, u,
                                     BACKWARD, distances);
  umax = umax - du;
  if (umax < EPS6) {
    /* The initial position of the robot is recovered */
    p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qsave);
    p3d_destroy_config(robotPt, qsave);
    MY_FREE(distances, double, njnt + 1);
    p3d_col_set_tolerance(tolerance);
    return(FALSE);
  }
  /* Remove the beginning of interval  */
  u = 0;
  if (change_position_robot(robotPt, localpathPt, u)) {
    /* The initial position of the robot is recovered */
    p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qsave);
    p3d_destroy_config(robotPt, qsave);
    MY_FREE(distances, double, njnt + 1);
    p3d_col_set_tolerance(tolerance);
    return TRUE;
  }
  for (j = 0; j <= njnt; j++) {
    distances[j] = dist0;
  }
  du = localpathPt->stay_within_dist(robotPt, localpathPt, u,
                                     FORWARD, distances);
  u = du;
  if (u > umax - EPS6) {
    u = umax;
    end_localpath++;
  }
  dist0 = (2 * dmax - newtol) / 2; /* Minimal distance we could cross at each step */

  while (end_localpath < 2) {
    /* position of the robot corresponding to parameter u */
    if (change_position_robot(robotPt, localpathPt, u)) {
      /* The initial position of the robot is recovered */
      p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qsave);
      p3d_destroy_config(robotPt, qsave);
      MY_FREE(distances, double, njnt + 1);
      p3d_col_set_tolerance(tolerance);
      return TRUE;
    }

    p3d_BB_dist_robot(robotPt, distances);
    test = FALSE;
    for (j = 0; j <= njnt; j++) {
      if (distances[j] < newtol + EPS6) {
        test = TRUE;
      }
      distances[j] /= 2;
      distances[j] += dist0;
    }
    if (test) {

      /* collision checking */
      *ntest = *ntest + 1;
      if (p3d_col_test()) {
        /* The initial position of the robot is recovered */
        p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qsave);
        p3d_destroy_config(robotPt, qsave);
        MY_FREE(distances, double, njnt + 1);
        p3d_col_set_tolerance(tolerance);
        return TRUE;
      }
      /* Modif. Etienne: if collision detector computed distances
      in call to p3d_col_test(), we exploit them */
      if (p3d_col_report_distance(robotPt, distances)) {
        for (j = 0; j <= njnt; j++) {
          distances[j] /= 2;
          distances[j] += dist0;
        }
      }
    }

    du = localpathPt->stay_within_dist(robotPt, localpathPt,
                                       u, FORWARD, distances);
    u += du;
    if (u > umax - EPS6) {
      u = umax;
      end_localpath++;
    }
  }
  /* The initial position of the robot is recovered */
  p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qsave);
  p3d_destroy_config(robotPt, qsave);

  MY_FREE(distances, double, njnt + 1);
  p3d_col_set_tolerance(tolerance);
  return FALSE;
}


/*--------------------------------------------------------------------------*/
/*! \brief Auto-collision checking of a trajectory
 *
 *  \param  robotPt:     the robot
 *  \param  localpathPt: the first local path
 *
 *  \retval ntest: the sum of collision checking tests
 *
 *  \return Whether collision or not
 *
 *  Description: successively check all the local paths of a trajectory.
 *
 *  \internal
 */
static int p3d_col_test_traj_autocol(p3d_rob *robotPt,
                                     p3d_localpath *localpathPt, int *ntest) {
  int njnt, test, j;
  configPt qsave;
  double *distances;
  double dmax, newtol, tolerance;

  if (localpathPt == NULL) {
    return FALSE;
  }

  qsave = p3d_get_robot_config(robotPt);
  njnt = robotPt->njoints;

  /* With trajectory we need to check the bounds of localpath */
  if (change_position_robot(robotPt, localpathPt, 0)) {
    p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qsave);
    p3d_destroy_config(robotPt, qsave);
    return TRUE;
  }
  p3d_col_get_dmax(&dmax);
  if (p3d_col_get_tolerance(&tolerance) && microcollision_avoidance) {
    newtol = tolerance + dmax;
  } else {
    newtol = 0;
  }
  distances = MY_ALLOC(double, njnt + 1);

  p3d_BB_dist_robot(robotPt, distances);
  test = FALSE;
  for (j = 0; j <= njnt; j++) {
    if (distances[j] < newtol + EPS6) {
      test = TRUE;
      break;
    }
  }
  if (test) {
    /* collision checking */
    *ntest = *ntest + 1;
    if (p3d_col_test()) {
      /* The initial position of the robot is recovered */
      p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qsave);
      p3d_destroy_config(robotPt, qsave);
      MY_FREE(distances, double, njnt + 1);
      return TRUE;
    }
  }

  while (localpathPt != NULL) {
    if (p3d_col_test_localpath_autocol(robotPt, localpathPt, ntest)) {
      /* The initial position of the robot is recovered */
      p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qsave);
      p3d_destroy_config(robotPt, qsave);
      MY_FREE(distances, double, njnt + 1);
      return TRUE;
    }
    /* With trajectory we need to check the bounds of localpath */
    if (change_position_robot(robotPt, localpathPt, localpathPt->range_param)) {
      p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qsave);
      p3d_destroy_config(robotPt, qsave);
      MY_FREE(distances, double, njnt + 1);
      return TRUE;
    }
    p3d_BB_dist_robot(robotPt, distances);
    test = FALSE;
    for (j = 0; j <= njnt; j++) {
      if (distances[j] < newtol + EPS6) {
        test = TRUE;
        break;
      }
    }
    if (test) {
      /* collision checking */
      *ntest = *ntest + 1;
      if (p3d_col_test()) {
        /* The initial position of the robot is recovered */
        p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qsave);
        p3d_destroy_config(robotPt, qsave);
        MY_FREE(distances, double, njnt + 1);
        return TRUE;
      }
    }

    localpathPt = localpathPt->next_lp;
  }
  p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qsave);
  p3d_destroy_config(robotPt, qsave);
  MY_FREE(distances, double, njnt + 1);
  return FALSE;
}


/*---------------------------------------------------------------------------
 *---------------------------------------------------------------------------
 * Test if a trajectory is valid by dichotomy */

/*!
 * \brief Structure used to describe intervals.
 *
 *  \internal
 */
typedef struct {
  /*! \brief Interval lenght
   *  \note  This is a localpath parameter value (not length value).  */
  double len;
  /*! \brief Begining of the interval
   *  \note  This is a localpath parameter value (not length value).  */
  double ldeb;
  /*! \brief Begining local path.
   *  \note  Usefull only in the old trajectory test function (which was more efficient
   *         but a too complicated).  */
  p3d_localpath * deb_lpathPt;
} type_interval;

/*! \brief Initial size for the array of interval use dichotomy test. */
#define INIT_SIZE_INTERVAL 200
/*! \brief Increment to increase the size of the array of interval when it is needed. */
#define INC_SIZE_INTERVAL 100




/*--------------------------------------------------------------------------*/
/*! \brief Increase the size of an interval array.
 *
 *  \param  interval: A pointer on the interval array.
 *  \param  nbMaxInt: The present maximum number of interval.
 *  \param  indice:   The indice of data beginning.
 *
 *  \retval interval: The new pointer on the interval array.
 *  \retval nbMaxInt: The new maximum number of interval.
 *
 *  \return TRUE if there is no memory error.
 *
 *  \internal
 */
static int p3d_col_env_realloc_interval(type_interval ** interval,
                                        unsigned long * nbMaxInt,
                                        unsigned long indice) {
  type_interval * tmp;

  tmp = MY_ALLOC(type_interval, *nbMaxInt + INC_SIZE_INTERVAL);
  if (tmp == NULL) {
    PrintError(("p3d_col_env_realloc_interval: Pas assez de m�oire"));
    return FALSE;
  }
  memcpy(tmp + indice, (*interval) + indice,
         (*nbMaxInt - indice)*sizeof(type_interval));
  MY_FREE((*interval), type_interval, (*nbMaxInt));
  *interval = tmp;
  (*nbMaxInt) += INC_SIZE_INTERVAL;
  return TRUE;
}



/*-------------------------------------------------------------------------
 * Local variables
 */

/*! \brief Array of intervals for the dichotomy test.
 *  \internal */
static type_interval *intervals = NULL;
/*! \brief Size maximal of interval table (::intervals)
 *  \internal */
static unsigned long nbMaxInt   =    0;


/*--------------------------------------------------------------------------*/
/*! \brief Initialize parameters for collision checking of a localpath.
 *
 *  \param localpathPt: the localpath
 *
 *  \note Modify the global variables ::nbMaxInt, and ::intervals
 *
 *  \internal
 */
static int p3d_col_env_init_localpath_parameters(p3d_localpath *localpathPt) {
  if (nbMaxInt == 0) {   /* The first time allocate more memory */
    nbMaxInt = INIT_SIZE_INTERVAL;
    intervals = MY_ALLOC(type_interval, nbMaxInt);
  }
  if (!(localpathPt->valid)) {
    return TRUE;
  }
  intervals[0].ldeb = 0.0;
  intervals[0].deb_lpathPt = localpathPt;
  /* Lenght of localpath */
  intervals[0].len = localpathPt->range_param;
  return FALSE;
}


/*--------------------------------------------------------------------------*/
/*! \brief Release memory allocated to work on the trajectory.
 *  \note Release the memory of ::intervals.
 */
void p3d_col_env_free_memory_traj_col_tab(void) {
  if (nbMaxInt > 0) {
    MY_FREE(intervals, type_interval, nbMaxInt);
    intervals = NULL;
    nbMaxInt = 0;
  }
}


/*--------------------------------------------------------------------------*/
/*! \brief Main function of dichotomy process.
 *
 *    This function work only on one localpath.
 *
 * \param robotPt: the robot
 * \param dmax:    the minimal distance between two tests
 * hidden param : global variable intervals (array of intervals)
 *
 * \retval ntest: the number of collision tests.
 *
 * \return True : collision along the localpath (in the hidden intervals)
 *
 * \note The local path informations are passed by global variable
 *       ::intervals
 *
 * \internal
 */
std::vector<double> aveBBDist;

static int split_curv_localpath_mobile_obst(p3d_rob * robotPt, double dmax,
    int *ntest) {
  unsigned long i;
  double dist, l, newtol, dist0;
  int test, j;
  double *distances_f, *distances_b;
  int njnt = robotPt->njoints;
  p3d_localpath * lpPt;
  double lenlp;
  unsigned long nbCurInt, nbNextInt;

//  printf("---------------------------------------------------------\n");

  if (p3d_col_get_tolerance(&newtol) && microcollision_avoidance) {
    /* With microcollision_avoidance the first and the last config could
       be at a distance  between tolerance and tolerance+dmax, we want the
       next config to be at more than tolerance+dmax from obstacle, and we
       want to avoid any config at less than tolerance, so we could only
       cross dmax from the bounds. */
    dist0 = dmax;
  } else {
    /* Without microcollision_avoidance the first and the last config are
       valid (a distance more than tolerance), we want the next config to be
       valid and we want to avoid any config at less than tolerance - dmax
       from the obstacles, so we could cross 2*dmax from the bounds. */
    newtol = 0;
    dist0 = 2 * dmax;
  }
  nbCurInt = 1;
  lpPt = intervals[0].deb_lpathPt; /* This time it is the same localpath
          for all the function */

  /* We suppose that the fisrt and the last configurations are valid so
     we could test the configuration at dist0 from the bounds */
  /* Remove the end of interval */
  lenlp = intervals[0].ldeb + intervals[0].len;
  if (change_position_robot_multisol(robotPt, lpPt, lenlp, P3D_HUGE, NULL)) { //no_multisol
    return TRUE;
  }
  distances_f = MY_ALLOC(double, njnt + 1);

  for (j = 0; j <= njnt; j++) {
    distances_f[j] = dist0;
  }
  /* Compute the lenght of left interval */
  dist = lpPt->stay_within_dist(robotPt, lpPt, lenlp, BACKWARD, distances_f);
  dist = MAX(dist, dmax);
  intervals[0].len -= dist;
  if (intervals[0].len < EPS6) {
    /* The initial position of the robot is recovered */
    MY_FREE(distances_f, double, njnt + 1);
    return(FALSE);
  }

  /* calculate the beginning of interval */
  lenlp = intervals[0].ldeb;
  if (change_position_robot_multisol(robotPt, lpPt, lenlp, P3D_HUGE, NULL)) {//no_multisol
    MY_FREE(distances_f, double, njnt + 1);
    return TRUE;
  }
  for (j = 0; j <= njnt; j++) {
    distances_f[j] = dist0;
  }

  dist = lpPt->stay_within_dist(robotPt, lpPt, 0, FORWARD, distances_f);
  //modif Mokhtar
  //dmax is the smallest unit in the discretisation dont go under
  dist = MAX(dist, dmax);
  //modif Mokhtar
  if (dist > intervals[0].len + EPS6) {
    //The final position of the robot is recovered
    intervals[0].ldeb += intervals[0].len;
    intervals[0].len = 0.;
  } else {
    intervals[0].len -= dist;
    intervals[0].ldeb += dist;
  }
  distances_b = MY_ALLOC(double, njnt + 1);

  dist0 = dmax + newtol / 2; /* Minimal distance we could cross at each step */
  newtol += EPS6;
  i = 0;
  do {
    nbNextInt = 0;
    while (i < nbCurInt) {

      /* Test the middle of interval */
      lenlp = intervals[i].ldeb + .5 * intervals[i].len;
      if (change_position_robot_multisol(robotPt, lpPt, lenlp, P3D_HUGE, NULL)) {//no_multisol
        MY_FREE(distances_f, double, njnt + 1);
        MY_FREE(distances_b, double, njnt + 1);
        return TRUE;
      }

      p3d_BB_dist_robot(robotPt, distances_b);
      test = FALSE;
      for (j = 0; j <= njnt; j++) {
        if (!test && distances_b[j] < newtol) {
          //if the distance between the Move3D BB and the environement objects including the robot itself are close or in collision, do a collision check
          test = TRUE;
        }
        distances_b[j] += dist0;
        distances_f[j] = distances_b[j];
      }

      if (test) {//if we are too close to the obstacles according to the Move3d AABB do a coll test to be sure
        *ntest = *ntest + 1;
        if (p3d_col_test()) {
          MY_FREE(distances_f, double, njnt + 1);
          MY_FREE(distances_b, double, njnt + 1);
          return(TRUE);
        }
        /* Modif. Carl: if collision detector computed distances
           in call to p3d_col_test(), we exploit them/ Theses distances are
           more precise than the move3D AABB*/
        if (p3d_col_report_distance(robotPt, distances_b)){
          for (j = 0; j <= njnt; j++) {
            distances_b[j] += dist0;
            distances_f[j] = distances_b[j];
          }
        }
      }
      //if there is no collision separate the interval into two and recompute
      /* Compute the lenght of left interval */
      dist = lpPt->stay_within_dist(robotPt, lpPt, lenlp, BACKWARD, distances_b);
      l = intervals[i].len / 2 - dist;

      if (l > EPS6) { //Add a new interval (we are not yet to the start configuration)
        if (nbCurInt + nbNextInt + 1 > nbMaxInt && !p3d_col_env_realloc_interval(&intervals, &nbMaxInt, i)) {
          MY_FREE(distances_f, double, njnt + 1);
          MY_FREE(distances_b, double, njnt + 1);
          return(TRUE);
        }
        intervals[nbCurInt+nbNextInt].len = l;
        intervals[nbCurInt+nbNextInt].ldeb = intervals[i].ldeb;
        nbNextInt++;
      }

      /* Compute the lenght of right interval */
      dist = lpPt->stay_within_dist(robotPt, lpPt, lenlp, FORWARD, distances_f);
      l = intervals[i].len / 2 - dist;
      if (l > EPS6) {
        if (nbCurInt + nbNextInt + 1 > nbMaxInt && !p3d_col_env_realloc_interval(&intervals, &nbMaxInt, i)) {
          MY_FREE(distances_f, double, njnt + 1);
          MY_FREE(distances_b, double, njnt + 1);
          return(TRUE);
        }
        intervals[nbCurInt+nbNextInt].len = l;
        intervals[nbCurInt+nbNextInt].ldeb = lenlp + dist;
        nbNextInt++;
      }
      i++;
    }
    nbCurInt += nbNextInt;
  } while (nbNextInt > 0);

  MY_FREE(distances_f, double, njnt + 1);
  MY_FREE(distances_b, double, njnt + 1);
  return FALSE;
}


/*--------------------------------------------------------------------------*/
/*! \brief Collision checking of a localpath with dichotomy
 *
 *  \param  robotPt:     the robot
 *  \param  localpathPt: the first local path
 *
 *  \retval ntest: the sum of collision checking tests
 *
 *  \return True : collision along the localpath
 *
 *  Description: initialize dichotomy parameters and run the
 *               dichotomy procedure
 *
 *  \internal
 */
static int p3d_col_test_localpath_dic(p3d_rob *robotPt, p3d_localpath *localpathPt, int *ntest) {
  int intersection;
  configPt qsave;
  pp3d_rob robcur;
  double tolerance, dmax;

  p3d_col_get_dmax(&dmax);
  robcur = (p3d_rob*)p3d_get_desc_curid(P3D_ROBOT);
  p3d_sel_desc_id(P3D_ROBOT, robotPt);

  /* initialization */
  if (p3d_col_env_init_localpath_parameters(localpathPt)) {
    p3d_sel_desc_id(P3D_ROBOT, robcur);
    return TRUE;
  }

  qsave = p3d_get_robot_config(robotPt);
  if (p3d_col_get_tolerance(&tolerance) && microcollision_avoidance)
    p3d_col_set_tolerance(tolerance + dmax);

  /* Test collision */
  intersection = split_curv_localpath_mobile_obst(robotPt, dmax, ntest);

  p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qsave);
  p3d_destroy_config(robotPt, qsave);
  p3d_sel_desc_id(P3D_ROBOT, robcur);
  p3d_col_set_tolerance(tolerance);
  return(intersection);
}


/*--------------------------------------------------------------------------*/
/*! \brief Collision checking of a trajectory with dichotomy
 *
 *  \param  robotPt:     the robot
 *  \param  localpathPt: the first local path
 *
 *  \retval ntest: the sum of collision checking tests
 *
 *  \return Whether collision or not
 *
 *  Description: successively check all the local paths of a trajectory.
 *
 *  \internal
 */
static int p3d_col_test_traj_dic(p3d_rob *robotPt,  p3d_localpath *localpathPt, int *ntest) {
  int njnt, test, j;
  configPt qsave;
  double *distances;
  double dmax, newtol, tolerance;

  if (localpathPt == NULL) {
    return FALSE;
  }

  qsave = p3d_get_robot_config(robotPt);
  njnt = robotPt->njoints;

  /* With trajectory we need to check the bounds of localpath */
  if (change_position_robot(robotPt, localpathPt, 0)) {
    p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qsave);
    p3d_destroy_config(robotPt, qsave);
    return TRUE;
  }
  p3d_col_get_dmax(&dmax);
  if (p3d_col_get_tolerance(&tolerance) && microcollision_avoidance) {
    newtol = tolerance + dmax;
  } else {
    newtol = 0;
  }
  distances = MY_ALLOC(double, njnt + 1);

  p3d_BB_dist_robot(robotPt, distances);
  test = FALSE;
  for (j = 0; j <= njnt; j++) {
    if (distances[j] < newtol + EPS6) {
      test = TRUE;
      break;
    }
  }
  if (test) {
    /* collision checking */
    *ntest = *ntest + 1;
    if (p3d_col_test()) {
      /* The initial position of the robot is recovered */
      p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qsave);
      p3d_destroy_config(robotPt, qsave);
      MY_FREE(distances, double, njnt + 1);
      return TRUE;
    }
  }

  while (localpathPt != NULL) {
    if (p3d_col_test_localpath_dic(robotPt, localpathPt, ntest)) {
      /* The initial position of the robot is recovered */
      p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qsave);
      p3d_destroy_config(robotPt, qsave);
      MY_FREE(distances, double, njnt + 1);
      return TRUE;
    }
    /* With trajectory we need to check the bounds of localpath */
    if (change_position_robot(robotPt, localpathPt, localpathPt->range_param)) {
      p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qsave);
      p3d_destroy_config(robotPt, qsave);
      MY_FREE(distances, double, njnt + 1);
      return TRUE;
    }
    p3d_BB_dist_robot(robotPt, distances);
    test = FALSE;
    for (j = 0; j <= njnt; j++) {
      if (distances[j] < newtol + EPS6) {
        test = TRUE;
        break;
      }
    }
    if (test) {
      /* collision checking */
      *ntest = *ntest + 1;
      if (p3d_col_test()) {
        /* The initial position of the robot is recovered */
        p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qsave);
        p3d_destroy_config(robotPt, qsave);
        MY_FREE(distances, double, njnt + 1);
        return TRUE;
      }
    }

    localpathPt = localpathPt->next_lp;
  }
  p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qsave);
  p3d_destroy_config(robotPt, qsave);
  MY_FREE(distances, double, njnt + 1);
  return FALSE;
}


/*--------------------------------------------------------------------------*/
/*! \brief Varaible for the loscal path test selesction.
 *  \internal */
static p3d_traj_test_type choose_test_traj = TEST_TRAJ_CLASSIC_ALL; // TEST_TRAJ_DICHOTOMIE_ALL; // TEST_TRAJ_OTHER_ROBOTS_DICHOTOMIE;



/*--------------------------------------------------------------------------*/
/*! \brief Choose the path collision checking method
 *
 *  \param type  the collision checking method type
 */
void p3d_col_env_set_traj_method(p3d_traj_test_type type) {
  if ((type >= TEST_TRAJ_CLASSIC) &&
      (type <= TEST_TRAJ_CONTEXT_DICHOTOMIE)) {
    choose_test_traj = type;
  }
}

/*--------------------------------------------------------------------------*/
/*! \brief Get the choice of path collision checking method
 *
 *  \return  the collision checking method type
 */
p3d_traj_test_type p3d_col_env_get_traj_method(void) {
  return choose_test_traj;
}


/*--------------------------------------------------------------------------*/
/*! \brief  Collision checking of a local path
 *
 *  \param  robotPt:     the robot
 *  \param  localpathPt: the local path
 *
 *  \retval ntest: the sum of collision checking tests
 *
 *  \return Whether collision or not
 *
 *  Description: test a local path according to the type of test selected by
 *               p3d_col_env_set_traj_method()
 */
int p3d_col_test_localpath(p3d_rob *robotPt, p3d_localpath *localpathPt, int *ntest) {
  double Kpath;
  p3d_type_col_choice mode_compute_dist;
  int collision = FALSE;

#ifdef BIO
  extern int bio_col_test_localpath_step(p3d_rob *robotPt, p3d_localpath *localpathPt, int *ntest, double *Kpath, configPt *q_atKpath);

  if (p3d_col_get_mode() == p3d_col_mode_bio) {
    collision = bio_col_test_localpath_step(robotPt, localpathPt, ntest, &Kpath, NULL);
    return(collision);
  }
#endif

  mode_compute_dist = get_kcd_which_test();
  set_kcd_which_test(P3D_KCD_ROB_ALL);
  switch (choose_test_traj)
  {
    case TEST_TRAJ_CLASSIC :
      p3d_col_env_switch_robot(robotPt, P3D_COL_ROBOT_ALL_WITHOUT_OTHER);
      collision = p3d_col_test_localpath_classic(robotPt, localpathPt,
                  ntest, &Kpath, NULL);  // <- modif Juan
      break;
    case TEST_TRAJ_DICHOTOMIE :
      p3d_col_env_switch_robot(robotPt, P3D_COL_ROBOT_ALL_WITHOUT_OTHER);
      collision = p3d_col_test_localpath_dic(robotPt, localpathPt, ntest);
      break;
    case TEST_TRAJ_CLASSIC_ALL :
      p3d_col_env_switch_robot(robotPt, P3D_COL_ROBOT_ENV);
      collision = p3d_col_test_localpath_classic(robotPt, localpathPt,
                  ntest, &Kpath, NULL);  // <- modif Juan

      if (!collision)
      {
        p3d_col_env_restore();
        p3d_col_env_switch_robot(robotPt, P3D_COL_ROBOT_AUTOCOL);
        collision = p3d_col_test_localpath_autocol(robotPt, localpathPt, ntest);
      }
      break;
    case TEST_TRAJ_DICHOTOMIE_ALL :
      p3d_col_env_switch_robot(robotPt, P3D_COL_ROBOT_ALL);
      collision = p3d_col_test_localpath_dic(robotPt, localpathPt, ntest);
      // Warning the test was done agaist env and then autocol
      // I changed it to All and then nothing
//      if (!collision) {
//        p3d_col_env_restore();
//        p3d_col_env_switch_robot(robotPt, P3D_COL_ROBOT_AUTOCOL);
//        collision = p3d_col_test_localpath_autocol(robotPt, localpathPt, ntest);
//      }
      break;
    case TEST_TRAJ_OTHER_ROBOTS_CLASSIC :
      p3d_col_env_switch_robot(robotPt, P3D_COL_ROBOT_ALL);
      collision = p3d_col_test_localpath_classic(robotPt, localpathPt,
                  ntest, &Kpath, NULL);  // <- modif Juan
      break;
    case TEST_TRAJ_OTHER_ROBOTS_DICHOTOMIE :
      p3d_col_env_switch_robot(robotPt, P3D_COL_ROBOT_ALL);
      collision = p3d_col_test_localpath_dic(robotPt, localpathPt, ntest);
      break;
    case TEST_TRAJ_OTHER_ROBOTS_CLASSIC_ALL :
      p3d_col_env_switch_robot(robotPt, P3D_COL_ROBOT_ALL_WITHOUT_AUTOCOL);
      collision = p3d_col_test_localpath_classic(robotPt, localpathPt,
                  ntest, &Kpath, NULL);  // <- modif Juan
      if (!collision) {
        p3d_col_env_restore();
        p3d_col_env_switch_robot(robotPt, P3D_COL_ROBOT_AUTOCOL);
        collision = p3d_col_test_localpath_autocol(robotPt, localpathPt, ntest);
      }
      break;
    case TEST_TRAJ_OTHER_ROBOTS_DICHOTOMIE_ALL :
      p3d_col_env_switch_robot(robotPt, P3D_COL_ROBOT_ALL_WITHOUT_AUTOCOL);
      collision = p3d_col_test_localpath_dic(robotPt, localpathPt, ntest);
      if (!collision) {
        p3d_col_env_restore();
        p3d_col_env_switch_robot(robotPt, P3D_COL_ROBOT_AUTOCOL);
        collision = p3d_col_test_localpath_autocol(robotPt, localpathPt, ntest);
      }
      break;
    case TEST_TRAJ_CONTEXT_CLASSIC :
      collision = p3d_col_test_localpath_classic(robotPt, localpathPt,
                  ntest, &Kpath, NULL);  // <- modif Juan
      set_kcd_which_test(mode_compute_dist);
      return collision;
    case TEST_TRAJ_CONTEXT_DICHOTOMIE :
      collision = p3d_col_test_localpath_dic(robotPt, localpathPt, ntest);
      set_kcd_which_test(mode_compute_dist);
      return collision;
    default:
      PrintWarning(("WARNING: p3d_col_test_localpath mode unknowed !!!\n"));
  }
  set_kcd_which_test(mode_compute_dist);
  p3d_col_env_restore();
  return collision;
}


/*--------------------------------------------------------------------------*/
/*! \brief  Collision checking of a trajectory
 *
 *  \param  robotPt:     the robot
 *  \param  localpathPt: the first local path
 *
 *  \retval ntest: the sum of collision checking tests
 *
 *  \return Whether collision or not
 *
 *  Description: test a trajectory according to the type of test selected
 *               by p3d_col_env_set_traj_method()
 */
int p3d_col_test_traj(p3d_rob *robotPt, p3d_localpath *localpathPt, int *ntest) {
  int collision = FALSE;
  p3d_type_col_choice mode_compute_dist;
  mode_compute_dist = get_kcd_which_test();
  set_kcd_which_test(P3D_KCD_ROB_ALL);
  switch (choose_test_traj) {
    case TEST_TRAJ_CLASSIC :
      p3d_col_env_switch_robot(robotPt, P3D_COL_ROBOT_ALL_WITHOUT_OTHER);
      collision = p3d_col_test_traj_classic(robotPt, localpathPt, ntest);
      break;
    case TEST_TRAJ_DICHOTOMIE :
      p3d_col_env_switch_robot(robotPt, P3D_COL_ROBOT_ALL_WITHOUT_OTHER);
      collision = p3d_col_test_traj_dic(robotPt, localpathPt, ntest);
      break;
    case TEST_TRAJ_CLASSIC_ALL :
      p3d_col_env_switch_robot(robotPt, P3D_COL_ROBOT_ENV);
      collision = p3d_col_test_traj_classic(robotPt, localpathPt, ntest);
      if (!collision) {
        p3d_col_env_restore();
        p3d_col_env_switch_robot(robotPt, P3D_COL_ROBOT_AUTOCOL);
        collision = p3d_col_test_traj_autocol(robotPt, localpathPt, ntest);
      }
      break;
    case TEST_TRAJ_DICHOTOMIE_ALL :
      p3d_col_env_switch_robot(robotPt, P3D_COL_ROBOT_ENV);
      collision = p3d_col_test_traj_dic(robotPt, localpathPt, ntest);
      if (!collision) {
        p3d_col_env_restore();
        p3d_col_env_switch_robot(robotPt, P3D_COL_ROBOT_AUTOCOL);
        collision = p3d_col_test_traj_autocol(robotPt, localpathPt, ntest);
      }
      break;
    case TEST_TRAJ_OTHER_ROBOTS_CLASSIC :
      p3d_col_env_switch_robot(robotPt, P3D_COL_ROBOT_ALL);
      collision = p3d_col_test_traj_classic(robotPt, localpathPt, ntest);
      break;
    case TEST_TRAJ_OTHER_ROBOTS_DICHOTOMIE :
      p3d_col_env_switch_robot(robotPt, P3D_COL_ROBOT_ALL);
      collision = p3d_col_test_traj_dic(robotPt, localpathPt, ntest);
      break;
    case TEST_TRAJ_OTHER_ROBOTS_CLASSIC_ALL :
      p3d_col_env_switch_robot(robotPt, P3D_COL_ROBOT_ALL_WITHOUT_AUTOCOL);
      collision = p3d_col_test_traj_classic(robotPt, localpathPt, ntest);
      if (!collision) {
        p3d_col_env_restore();
        p3d_col_env_switch_robot(robotPt, P3D_COL_ROBOT_AUTOCOL);
        collision = p3d_col_test_traj_autocol(robotPt, localpathPt, ntest);
      }
      break;
    case TEST_TRAJ_OTHER_ROBOTS_DICHOTOMIE_ALL :
      p3d_col_env_switch_robot(robotPt, P3D_COL_ROBOT_ALL_WITHOUT_AUTOCOL);
      collision = p3d_col_test_traj_dic(robotPt, localpathPt, ntest);
      if (!collision) {
        p3d_col_env_restore();
        p3d_col_env_switch_robot(robotPt, P3D_COL_ROBOT_AUTOCOL);
        collision = p3d_col_test_traj_autocol(robotPt, localpathPt, ntest);
      }
      break;
    case TEST_TRAJ_CONTEXT_CLASSIC :
      collision = p3d_col_test_traj_classic(robotPt, localpathPt, ntest);
      set_kcd_which_test(mode_compute_dist);
      return collision;
    case TEST_TRAJ_CONTEXT_DICHOTOMIE :
      collision = p3d_col_test_traj_dic(robotPt, localpathPt, ntest);
      set_kcd_which_test(mode_compute_dist);
      return collision;
    default:
      PrintWarning(("WARNING: p3d_col_test_traj mode unknowed !!!\n"));
  }
  set_kcd_which_test(mode_compute_dist);
  p3d_col_env_restore();
  return collision;
}
//start path deform
int p3d_test_visibility_edge(p3d_rob *robotPt, configPt q0, configPt q_edge1, configPt q_edge2) {
  dbl_list* list_config_on_edge = NULL;
  p3d_jnt* jntPt,  *jjntPt = NULL;
  int i, j, k, njnt, ii, jj, kk, test, nb_test = 0;
  double qnew_k, p;
  configPt q1, q2;
  p3d_config_on_edge * q_on_edge = NULL;

  list_config_on_edge = dbl_list_pointer_init();
  // we add q_edge1 and q_edge2 in the list_config_on_edge
  q_on_edge = p3d_allocinit_config_on_edge(robotPt);
  q_on_edge->q = p3d_copy_config(robotPt, q_edge1);
  q_on_edge->p = 0.;
  dbl_list_concat(list_config_on_edge, q_on_edge);
  q_on_edge = p3d_allocinit_config_on_edge(robotPt);
  q_on_edge->q = p3d_copy_config(robotPt, q_edge2);
  q_on_edge->p = 1.;
  dbl_list_concat(list_config_on_edge, q_on_edge);

  njnt = robotPt->njoints;
  for (i = 0; i <= njnt; i++) {
    jntPt = robotPt->joints[i];
    for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
      k = jntPt->index_dof + j;
      if (p3d_jnt_is_dof_circular(jntPt, j)) {
        if (fabs(q_edge2[k] - q_edge1[k]) < M_PI) {
          if (((fabs(q_edge2[k] - q0[k]) < M_PI) && (fabs(q_edge1[k] - q0[k]) > M_PI)) ||
              ((fabs(q_edge1[k] - q0[k]) < M_PI) && (fabs(q_edge2[k] - q0[k]) > M_PI))) {
            qnew_k = angle_limit_PI(q0[k] - M_PI);
            p = fabs(qnew_k - q_edge1[k]) / fabs(q_edge2[k] - q_edge1[k]);
            q_on_edge = p3d_allocinit_config_on_edge(robotPt);
            q_on_edge->p = p;
            for (ii = 0; ii <= njnt; ii++) {
              jjntPt = robotPt->joints[ii];
              for (jj = 0; jj < jjntPt->dof_equiv_nbr; jj++) {
                kk = jjntPt->index_dof + jj;
                if (p3d_jnt_is_dof_circular(jjntPt, jj)) {
                  q_on_edge->q[kk] = angle_limit_PI(q_edge1[kk] + p * diff_angle(q_edge1[kk], q_edge2[kk]));
                } else {
                  q_on_edge->q[kk] = q_edge1[kk] + p * (q_edge2[kk] - q_edge1[kk]);
                }
              }
            }
            dbl_list_concat(list_config_on_edge, q_on_edge);
          }
        } else {
          if ((fabs(q_edge2[k] - q0[k]) < M_PI) && (fabs(q_edge1[k] - q0[k]) < M_PI)) {
            qnew_k = angle_limit_PI(q0[k] - M_PI);
            p = dist_circle(qnew_k, q_edge1[k]) / dist_circle(q_edge2[k], q_edge1[k]);
            q_on_edge = p3d_allocinit_config_on_edge(robotPt);
            q_on_edge->p = p;
            for (ii = 0; ii <= njnt; ii++) {
              jjntPt = robotPt->joints[ii];
              for (jj = 0; jj < jjntPt->dof_equiv_nbr; jj++) {
                kk = jjntPt->index_dof + jj;
                if (p3d_jnt_is_dof_circular(jjntPt, jj)) {
                  q_on_edge->q[kk] = angle_limit_PI(q_edge1[kk] + p * diff_angle(q_edge1[kk], q_edge2[kk]));
                } else {
                  q_on_edge->q[kk] = q_edge1[kk] + p * (q_edge2[kk] - q_edge1[kk]);
                }
              }
            }
            dbl_list_concat(list_config_on_edge, q_on_edge);
          }
        }
      }
    }
  }
  //p3d_destroy_config_on_edge(robotPt, q_on_edge);
  dbl_list_sort(list_config_on_edge,
                p3d_sort_config_on_edge);
  dbl_list_push(list_config_on_edge);
  dbl_list_goto_first(list_config_on_edge);
  dbl_list_next(list_config_on_edge);
  while (dbl_list_more(list_config_on_edge)) {
    q1 = (DBL_LIST_PREV(p3d_config_on_edge , list_config_on_edge))->q;
    q2 = (DBL_LIST_DATA(p3d_config_on_edge , list_config_on_edge))->q;
    // d0 =dist_circle(q1[8],q2[8]);
    //d1 =dist_circle(q0[8],q2[8]);
    //d2 =dist_circle(q0[8],q1[8]);
    /*PrintInfo(("d0: %f, d1: %f, d2: %f\n",d0,d1,d2));*/
    nb_test++;
    test = p3d_test_loc_face(robotPt, q0, q1, q2);
    //if(nb_test>1) {
    //  PrintInfo(("Warning : nb test < 1\n"));
    // }
    if (!test) {
      //destroy des elements dans la liste?
      dbl_list_pop(list_config_on_edge);
      dbl_list_goto_first(list_config_on_edge);
      while (dbl_list_more(list_config_on_edge)) {
        q_on_edge = DBL_LIST_DATA(p3d_config_on_edge , list_config_on_edge);
        p3d_destroy_config_on_edge(robotPt, q_on_edge);
        dbl_list_remove_link(list_config_on_edge);
      }
      //dbl_list_destroy(list_config_on_edge);
      return test;
    }
    dbl_list_next(list_config_on_edge);
  }
  dbl_list_pop(list_config_on_edge);
  while (dbl_list_more(list_config_on_edge)) {
    q_on_edge = DBL_LIST_DATA(p3d_config_on_edge , list_config_on_edge);
    p3d_destroy_config_on_edge(robotPt, q_on_edge);
    dbl_list_remove_link(list_config_on_edge);
  }
  //  dbl_list_destroy(list_config_on_edge);
  return TRUE;
}
static int p3d_sort_config_on_edge(void* q_on_edge1, void* q_on_edge2) {
  if (((p3d_config_on_edge*)q_on_edge1)->p < ((p3d_config_on_edge*)q_on_edge2)->p)
    return TRUE;
  return FALSE;
}

static p3d_config_on_edge*  p3d_allocinit_config_on_edge(p3d_rob *robotPt) {
  p3d_config_on_edge* q_on_edge;
  q_on_edge = MY_ALLOC(p3d_config_on_edge, 1);
  q_on_edge->q = p3d_alloc_config(robotPt);
  return q_on_edge;
}
static void p3d_destroy_config_on_edge(p3d_rob *robotPt,
                                       p3d_config_on_edge* q_on_edge) {
  p3d_destroy_config(robotPt, q_on_edge->q);
  MY_FREE(q_on_edge, p3d_config_on_edge, 1);
}
/*! \brief  check the validity of a local face
 *
 *
 *  \param  robotPt:     the robot
 *  \param  q0 q1 q2 :   the 3 verticies of the local face.
 *
 *  \return True if the local face is free of collisions
 *
 *  Description: Based on the p3d_test_col_face function. Based on the number
 *               of intersection between discs.
 *
 *
 */
static int p3d_test_loc_face(p3d_rob *robotPt, configPt q0, configPt q1, configPt q2) {
  configPt* q_list = MY_ALLOC(configPt, 3);
  double *R_list =  MY_ALLOC(double, 3), *distances = MY_ALLOC(double, robotPt->njoints + 1);
  int test = 0, j;
  double *copy_distances;
  p3d_localpath *path;
  int unvalid;
  int microcol;


  q_list[0] = q0;
  q_list[1] = q1;
  q_list[2] = q2;

  for (j = 0; j < 3; j++) {
    if (p3d_equal_config(robotPt, q_list[j%3], q_list[(j+1)%3])) {
      path = p3d_local_planner(robotPt, q_list[j%3], q_list[(j+2)%3]);
      if (path == NULL) {
        PrintInfo(("Error : impossible de planifier\n"));
        return FALSE;
      }
      microcol = p3d_col_get_microcollision();
      p3d_col_set_microcollision(TRUE);
      unvalid = p3d_unvalid_localpath_test(robotPt, path, &(robotPt->GRAPH->nb_test_coll));
      p3d_col_set_microcollision(microcol);
      path->destroy(robotPt, path);
      return (!unvalid);
    }
  }
  ChronoOn();
  p3d_set_and_update_robot_conf(q0);
  p3d_update_robot_pos();
  if (p3d_col_test()) {
    PrintInfo(("collision détectée pour les verticies initiaux!\n"));
    MY_FREE(q_list, configPt, 3);
    MY_FREE(R_list, double, 3);
    MY_FREE(distances, double, 3);
    ChronoPrint("");
    ChronoOff();
    return FALSE;
  }
  if (! p3d_col_report_distance_bodies_obst(robotPt, distances)) {
    PrintInfo(("Pb distance information\n"));
    MY_FREE(q_list, configPt, 3);
    MY_FREE(R_list, double, 3);
    MY_FREE(distances, double, 3);
    ChronoPrint("");
    ChronoOff();
    return FALSE;
  }
  copy_distances = MY_ALLOC(double, robotPt->njoints + 1);
  for (j = 0; j <= robotPt->njoints; j++) {
    copy_distances[j] = distances[j];
  }
  R_list[0] = p3d_stay_within_sphere(robotPt, distances);

  p3d_set_and_update_robot_conf(q1);
  p3d_update_robot_pos();
  if (p3d_col_test()) {
    PrintInfo(("collision détectée pour les verticies initiaux!\n"));
    MY_FREE(q_list, configPt, 3);
    MY_FREE(R_list, double, 3);
    MY_FREE(distances, double, 3);
    ChronoPrint("");
    ChronoOff();
    return FALSE;
  }
  if (! p3d_col_report_distance_bodies_obst(robotPt, distances)) {
    PrintInfo(("Pb distance information\n"));
    MY_FREE(q_list, configPt, 3);
    MY_FREE(R_list, double, 3);
    MY_FREE(distances, double, 3);
    ChronoPrint("");
    ChronoOff();
    return FALSE;
  }
  R_list[1] = p3d_stay_within_sphere(robotPt, distances);

  p3d_set_and_update_robot_conf(q2);
  p3d_update_robot_pos();
  if (p3d_col_test()) {
    PrintInfo(("collision détectée pour les verticies initiaux!\n"));
    MY_FREE(q_list, configPt, 3);
    MY_FREE(R_list, double, 3);
    MY_FREE(distances, double, 3);
    ChronoPrint("");
    ChronoOff();
    return FALSE;
  }
  if (! p3d_col_report_distance_bodies_obst(robotPt, distances)) {
    PrintInfo(("Pb distance information\n"));
    MY_FREE(q_list, configPt, 3);
    MY_FREE(R_list, double, 3);
    MY_FREE(distances, double, 3);
    ChronoPrint("");
    ChronoOff();
    return FALSE;
  }
  R_list[2] = p3d_stay_within_sphere(robotPt, distances);

  DEPTH++;
  test =   p3d_test_col_face(robotPt, q_list, R_list);
  MY_FREE(q_list, configPt, 3);
  MY_FREE(R_list, double, 3);
  MY_FREE(distances, double, 3);

  ChronoOff();
  return test;
}
/*! \brief  check if a local face is recovered by discs centered
 *          on the verticies of the face
 *
 *  \param  robotPt:     the robot
 *  \param  q_list :   the list of config of the face
 *  \param  R_list : the list of radii associated to the verticies
 *
 *  \return True if the local face is recovered by the discs
 *
 *  Description: Recursive test based on the number of intersection between discs.
 *               A geometric approach is used.
 *
 */
int p3d_test_col_face(p3d_rob *robotPt, configPt * q_list,
                      double* R_list) {

  double* e = NULL, *distances = NULL, *R_list_new;
  double d0, d1, d2, xi, yi, xc, yc, d_square, qnew1_k, qnew2_k, qnew1_ak, qnew2_ak, qnew1_bk, qnew2_bk;
  double  Ra, Rb, Rc, Rnew;
  int *ind = NULL;
  int njnt = robotPt->njoints, i, j, k;
  int nb_intersect = 0, test1 = 0, test2 = 0, unvalid = 0;
  configPt qnew = NULL, qnew_a = NULL, qnew_b = NULL, qa = NULL;
  configPt qb = NULL, qc = NULL,  *q_list_new = NULL;
  p3d_jnt * jntPt = NULL;
  p3d_localpath *path = NULL;
  if (DEPTH > 100) {
    PrintInfo(("Big depth!!\n"));
    DEPTH--;
    return FALSE;
  }


  /* test if a facet is reduced to a path */

  /* Compute the number of intersections between spheres : if d(qk,ql) - (Rk+Rl) > 0
     the discs centered on qk and dl do not intersect */
  e = MY_ALLOC(double, 3);
  for (i = 0; i < 3; i++) {
    e[i] = p3d_dist_config(robotPt, q_list[(i+1)%3], q_list[(i+2)%3]) - (R_list[(i+1)%3] + R_list[(i+2)%3]);
    if (e[i] < 0) {
      nb_intersect++;
    }
  }
  /* ind[] gives the indices of the e[i] in order of value */
  ind = MY_ALLOC(int, 3);
  if ((e[0] <= e[1]) && (e[0] <= e[2])) {
    if (e[1] <= e[2]) {
      for (i = 0; i < 3; i++) {
        ind[i] = i;
      }
    } else {
      ind[0] = 0;
      ind[1] = 2;
      ind[2] = 1;
    }
  } else if ((e[1] <= e[0]) && (e[1] <= e[2])) {
    if (e[0] <= e[2]) {
      ind[0] = 1;
      ind[1] = 0;
      ind[2] = 2;
    } else {
      ind[0] = 1;
      ind[1] = 2;
      ind[2] = 0;
    }
  } else {
    if (e[0] <= e[1]) {
      ind[0] = 2;
      ind[1] = 0;
      ind[2] = 1;
    } else {
      ind[0] = 2;
      ind[1] = 1;
      ind[2] = 0;
    }
  }


  if (nb_intersect < 2) {  //0 or 1 intersections between pair of discs
    qnew = p3d_alloc_config(robotPt);
    qa = q_list[(ind[2] + 1) % 3];
    Ra = R_list[(ind[2] + 1) % 3];
    qb = q_list[(ind[2] + 2) % 3];
    Rb = R_list[(ind[2] + 2) % 3];
    qc = q_list[(ind[2]) % 3];

    for (i = 0; i <= njnt; i++) {
      jntPt = robotPt->joints[i];
      for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
        k = jntPt->index_dof + j;
        if (p3d_jnt_is_dof_circular(jntPt, j)) {
          //if(dist_circle(qa[k],qb[k])- M_PI<EPS6) {
          qnew1_k = angle_limit_PI(qa[k] + ((Ra + 0.5 * e[ind[2]]) / (p3d_dist_config(robotPt, qb, qa))) * diff_angle(qa[k], qb[k]));
          qnew2_k = angle_limit_PI(qa[k] - ((Ra + 0.5 * e[ind[2]]) / (p3d_dist_config(robotPt, qb, qa))) * diff_angle(qa[k], qb[k]));
          if (dist_circle(qnew1_k, qc[k]) < dist_circle(qnew2_k, qc[k])) {
            qnew[k] = qnew1_k;
          } else {
            qnew[k] = qnew2_k;
          }
        } else {
          qnew[k] = qa[k] + (1 / 2. + (Rb - Ra) / (2.*p3d_dist_config(robotPt, qb, qa))) * (qb[k] - qa[k]);
        }
      }
    }
    p3d_set_and_update_robot_conf(qnew);
    p3d_update_robot_pos();
    if (p3d_col_test()) {
      MY_FREE(ind, int, 3);
      MY_FREE(e, double, 3);
      if (qnew != NULL) {
        p3d_destroy_config(robotPt, qnew);
        qnew = NULL;
      }
      DEPTH--;
      return FALSE;
    }
    distances = MY_ALLOC(double, robotPt->njoints + 1);
    if (! p3d_col_report_distance_bodies_obst(robotPt, distances)) {
      PrintInfo(("Pb distance information\n"));
      if (qnew != NULL) {
        p3d_destroy_config(robotPt, qnew);
        qnew = NULL;
      }
      MY_FREE(ind, int, 3);
      MY_FREE(e, double, 3);
      MY_FREE(distances, double, robotPt->njoints + 1);
      DEPTH--;
      return FALSE;
    }
    Rnew = p3d_stay_within_sphere(robotPt, distances);

    q_list_new = MY_ALLOC(configPt, 3);
    R_list_new = MY_ALLOC(double, 3);

    q_list_new[0] =  q_list[ind[2]];
    q_list_new[1] =  qa;
    q_list_new[2] =  qnew;

    R_list_new[0] =  R_list[ind[2]];
    R_list_new[1] =  Ra;
    R_list_new[2] =  Rnew;

    for (i = 0; i < 3; i++) {
      if (p3d_equal_config(robotPt, q_list_new[i%3], q_list_new[(i+1)%3])) {
        PrintInfo(("2 confs of the facet are equal, test local path\n"));
        path = p3d_local_planner(robotPt, q_list_new[i%3], q_list_new[(i+2)%3]);
        if (path == NULL) {
          PrintInfo(("Error : impossible de planifier\n"));
          DEPTH--;
          return FALSE;
        }
        unvalid = p3d_unvalid_localpath_test(robotPt, path, &(robotPt->GRAPH->nb_test_coll));
        path->destroy(robotPt, path);
        path = NULL;
        DEPTH--;
        return (!unvalid);
      }
    }
    DEPTH++;
    test1 = p3d_test_col_face(robotPt, q_list_new, R_list_new);
    if (test1 == FALSE) {
      if (qnew != NULL) {
        p3d_destroy_config(robotPt, qnew);
        qnew = NULL;
      }
      MY_FREE(ind, int, 3);
      MY_FREE(e, double, 3);
      MY_FREE(distances, double, robotPt->njoints + 1);
      MY_FREE(q_list_new, configPt, 3);
      MY_FREE(R_list_new, double, 3);
      DEPTH--;
      return FALSE;
    }
    q_list_new[0] =  q_list[ind[2]];
    q_list_new[1] =  qb;
    q_list_new[2] =  qnew;

    R_list_new[0] =  R_list[ind[2]];
    R_list_new[1] =  Rb;
    R_list_new[2] =  Rnew;
    for (i = 0; i < 3; i++) {
      if (p3d_equal_config(robotPt, q_list_new[i%3], q_list_new[(i+1)%3])) {
        PrintInfo(("2 confs of the facet are equal, test local path\n"));
        path = p3d_local_planner(robotPt, q_list_new[i%3], q_list_new[(i+2)%3]);
        if (path == NULL) {
          PrintInfo(("Error : impossible de planifier\n"));
          DEPTH--;
          return FALSE;
        }
        unvalid = p3d_unvalid_localpath_test(robotPt, path, &(robotPt->GRAPH->nb_test_coll));
        path->destroy(robotPt, path);
        path = NULL;
        DEPTH--;
        return (!unvalid);
      }
    }
    DEPTH++;
    test2 = p3d_test_col_face(robotPt, q_list_new, R_list_new);
    if (qnew != NULL) {
      p3d_destroy_config(robotPt, qnew);
      qnew = NULL;
    }
    MY_FREE(ind, int, 3);
    MY_FREE(e, double, 3);
    MY_FREE(distances, double, robotPt->njoints + 1);
    MY_FREE(q_list_new, configPt, 3);
    MY_FREE(R_list_new, double, 3);
    DEPTH--;
    return (test2);
  } else if (nb_intersect == 2) {        //2  intersections between pair of discs
    qnew_a  = p3d_alloc_config(robotPt);
    qnew_b  = p3d_alloc_config(robotPt);

    qa = q_list[(ind[2] + 1) % 3];
    Ra = R_list[(ind[2] + 1) % 3];
    qb = q_list[(ind[2] + 2) % 3];
    Rb = R_list[(ind[2] + 2) % 3];
    qc = q_list[ind[2] % 3];
    Rc = R_list[ind[2] % 3];
    for (i = 0; i <= njnt; i++) {
      jntPt = robotPt->joints[i];
      for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
        k = jntPt->index_dof + j;
        if (p3d_jnt_is_dof_circular(jntPt, j)) {
          qnew1_ak = angle_limit_PI(qa[k] + (Ra / p3d_dist_config(robotPt, qa, qb)) * diff_angle(qa[k], qb[k]));
          qnew2_ak = angle_limit_PI(qa[k] - (Ra / p3d_dist_config(robotPt, qa, qb)) * diff_angle(qa[k], qb[k]));
          if (dist_circle(qnew1_ak, qc[k]) < dist_circle(qnew2_ak, qc[k])) {
            qnew_a[k] = qnew1_ak;
          } else {
            qnew_a[k] = qnew2_ak;
          }
          qnew1_bk = angle_limit_PI(qb[k] + (Rb / p3d_dist_config(robotPt, qa, qb)) * diff_angle(qb[k], qa[k]));
          qnew2_bk = angle_limit_PI(qb[k] - (Rb / p3d_dist_config(robotPt, qa, qb)) * diff_angle(qb[k], qa[k]));
          if (dist_circle(qnew1_bk, qc[k]) < dist_circle(qnew2_bk, qc[k])) {
            qnew_b[k] = qnew1_bk;
          } else {
            qnew_b[k] = qnew2_bk;
          }
        } else {
          qnew_a[k] = qa[k] + (Ra / p3d_dist_config(robotPt, qa, qb)) * (qb[k] - qa[k]);
          qnew_b[k] = qb[k] + (Rb / p3d_dist_config(robotPt, qa, qb)) * (qa[k] - qb[k]);
        }
      }
    }

    if ((p3d_dist_config(robotPt, qc, qnew_a) < Rc) &&
        (p3d_dist_config(robotPt, qc, qnew_b) < Rc)) {

      p3d_destroy_config(robotPt, qnew_a);
      qnew_a = NULL;
      p3d_destroy_config(robotPt, qnew_b);
      qnew_b = NULL;
      MY_FREE(ind, int, 3);
      MY_FREE(e, double, 3);

      DEPTH--;
      return TRUE;
    }

    p3d_destroy_config(robotPt, qnew_a);
    qnew_a = NULL;
    p3d_destroy_config(robotPt, qnew_b);
    qnew_b = NULL;
    qnew = p3d_alloc_config(robotPt);

    for (i = 0; i <= njnt; i++) {
      jntPt = robotPt->joints[i];
      for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
        k = jntPt->index_dof + j;
        if (p3d_jnt_is_dof_circular(jntPt, j)) {
          qnew1_k = angle_limit_PI(qa[k] + ((Ra + 0.5 * e[ind[2]]) / p3d_dist_config(robotPt, qa, qb)) * diff_angle(qa[k], qb[k]));
          qnew2_k = angle_limit_PI(qa[k] - ((Ra + 0.5 * e[ind[2]]) / p3d_dist_config(robotPt, qa, qb)) * diff_angle(qa[k], qb[k]));
          if (dist_circle(qnew1_k, qc[k]) < dist_circle(qnew2_k, qc[k])) {
            qnew[k] = qnew1_k;
          } else {
            qnew[k] = qnew2_k;
          }
        } else {
          qnew[k] = qa[k] + ((Ra + 0.5 * e[ind[2]]) / p3d_dist_config(robotPt, qa, qb)) * (qb[k] - qa[k]);
        }
      }
    }

    p3d_set_and_update_robot_conf(qnew);
    p3d_update_robot_pos();
    if (p3d_col_test()) {
      MY_FREE(ind, int, 3);
      MY_FREE(e, double, 3);

      if (qnew != NULL) {
        p3d_destroy_config(robotPt, qnew);
        qnew = NULL;
      }

      DEPTH--;
      return FALSE;
    }
    distances = MY_ALLOC(double, robotPt->njoints + 1);
    if (! p3d_col_report_distance_bodies_obst(robotPt, distances)) {
      PrintInfo(("Pb distance information\n"));
      if (qnew != NULL) {
        p3d_destroy_config(robotPt, qnew);
        qnew = NULL;
      }
      MY_FREE(ind, int, 3);
      MY_FREE(distances, double, robotPt->njoints + 1);
      MY_FREE(e, double, 3);
      DEPTH--;
      return FALSE;
    }
    Rnew = p3d_stay_within_sphere(robotPt, distances);

    q_list_new = MY_ALLOC(configPt, 3);
    R_list_new = MY_ALLOC(double, 3);

    q_list_new[0] =  q_list[ind[2]];
    q_list_new[1] =  qa;
    q_list_new[2] =  qnew;

    R_list_new[0] =  R_list[ind[2]];
    R_list_new[1] =  Ra;
    R_list_new[2] =  Rnew;

    for (i = 0; i < 3; i++) {
      if (p3d_equal_config(robotPt, q_list_new[i%3], q_list_new[(i+1)%3])) {
        PrintInfo(("2 confs of the facet are equal, test local path\n"));
        path = p3d_local_planner(robotPt, q_list_new[i%3], q_list_new[(i+2)%3]);
        if (path == NULL) {
          PrintInfo(("Error : impossible de planifier\n"));
          DEPTH--;
          return FALSE;
        }
        unvalid = p3d_unvalid_localpath_test(robotPt, path, &(robotPt->GRAPH->nb_test_coll));
        path->destroy(robotPt, path);
        path = NULL;
        DEPTH--;
        return (!unvalid);
      }
    }

    DEPTH++;
    test1 = p3d_test_col_face(robotPt, q_list_new, R_list_new);
    if (test1 == FALSE) {
      if (qnew != NULL) {
        p3d_destroy_config(robotPt, qnew);
        qnew = NULL;
      }
      MY_FREE(ind, int, 3);
      MY_FREE(e, double, 3);
      MY_FREE(distances, double, robotPt->njoints + 1);
      MY_FREE(q_list_new, configPt, 3);
      MY_FREE(R_list_new, double, 3);
      DEPTH--;
      return FALSE;
    }
    q_list_new[0] =  q_list[ind[2]];
    q_list_new[1] =  qb;
    q_list_new[2] =  qnew;

    R_list_new[0] =  R_list[ind[2]];
    R_list_new[1] =  Rb;
    R_list_new[2] =  Rnew;

    DEPTH++;
    test2 = p3d_test_col_face(robotPt, q_list_new, R_list_new);
    if (qnew != NULL) {
      p3d_destroy_config(robotPt, qnew);
      qnew = NULL;
    }
    MY_FREE(ind, int, 3);
    MY_FREE(e, double, 3);
    MY_FREE(distances, double, robotPt->njoints + 1);
    MY_FREE(q_list_new, configPt, 3);
    MY_FREE(R_list_new, double, 3);
    DEPTH--;
    return (test2);

  } else {//all  intersections between pair of discs

    d0 = p3d_dist_config(robotPt, q_list[1], q_list[2]);
    d1 = p3d_dist_config(robotPt, q_list[0], q_list[2]);
    d2 = p3d_dist_config(robotPt, q_list[0], q_list[1]);

    xc = (SQR(d2) + SQR(d0) - SQR(d1)) / (2 * d0);
    if ((SQR(d2) - SQR(xc)) < 0) {
      PrintInfo(("warning xc2 > d32\n"));
      DEPTH--;
      return FALSE;
    }
    yc = sqrt(SQR(d2) - SQR(xc));

    if ((R_list[2] > d0 + R_list[1]) || (R_list[1] > d0 + R_list[2])) {
      MY_FREE(ind, int, 3);
      MY_FREE(e, double, 3);
      DEPTH--;
      return TRUE;
    }
    xi = (SQR(R_list[1]) + SQR(d0) - SQR(R_list[2])) / (2 * d0);
    if ((SQR(R_list[1]) - SQR(xi)) < 0) {
      PrintInfo(("warning xi2 > R1\n"));
      DEPTH--;
      return FALSE;
    }
    yi = sqrt(SQR(R_list[1]) - SQR(xi));
    d_square = SQR(xi - xc) + SQR(yi - yc);

    if (d_square < SQR(R_list[0])) {
      MY_FREE(ind, int, 3);
      MY_FREE(e, double, 3);
      DEPTH--;
      return TRUE;
    }

    qnew = p3d_alloc_config(robotPt);
    qa = q_list[(ind[2] + 1) % 3];
    Ra = R_list[(ind[2] + 1) % 3];
    qb = q_list[(ind[2] + 2) % 3];
    Rb = R_list[(ind[2] + 2) % 3];
    qc = q_list[ind[2] % 3];
    for (i = 0; i <= njnt; i++) {
      jntPt = robotPt->joints[i];
      for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
        k = jntPt->index_dof + j;
        if (p3d_jnt_is_dof_circular(jntPt, j)) {
          qnew1_k = angle_limit_PI(qa[k] + ((Ra + 0.5 * e[ind[2]]) / (p3d_dist_config(robotPt, qb, qa))) * (diff_angle(qa[k], qb[k])));
          qnew2_k = angle_limit_PI(qa[k] - ((Ra + 0.5 * e[ind[2]]) / (p3d_dist_config(robotPt, qb, qa))) * (diff_angle(qa[k], qb[k])));
          if (dist_circle(qnew1_k, qc[k]) < dist_circle(qnew2_k, qc[k])) {
            qnew[k] = qnew1_k;
          } else {
            qnew[k] = qnew2_k;
          }
        } else {
          qnew[k] = qa[k] + ((Ra + 0.5 * e[ind[2]]) / (p3d_dist_config(robotPt, qb, qa))) * (qb[k] - qa[k]);
        }
      }
    }



    p3d_set_and_update_robot_conf(qnew);
    p3d_update_robot_pos();
    if (p3d_col_test()) {
      MY_FREE(ind, int, 3);
      MY_FREE(e, double, 3);

      if (qnew != NULL) {
        p3d_destroy_config(robotPt, qnew);
        qnew = NULL;
      }

      DEPTH--;
      return FALSE;
    }
    distances = MY_ALLOC(double, robotPt->njoints + 1);
    if (! p3d_col_report_distance_bodies_obst(robotPt, distances)) {
      PrintInfo(("Pb distance information\n"));
      if (qnew != NULL) {
        p3d_destroy_config(robotPt, qnew);
        qnew = NULL;
      }
      MY_FREE(ind, int, 3);
      MY_FREE(e, double, 3);
      MY_FREE(distances, double, robotPt->njoints + 1);

      DEPTH--;
      return FALSE;
    }
    Rnew = p3d_stay_within_sphere(robotPt, distances);


    q_list_new = MY_ALLOC(configPt, 3);
    R_list_new = MY_ALLOC(double, 3);

    q_list_new[0] =  q_list[ind[2]];
    q_list_new[1] =  qa;
    q_list_new[2] =  qnew;

    R_list_new[0] =  R_list[ind[2]];
    R_list_new[1] =  Ra;
    R_list_new[2] =  Rnew;

    for (i = 0; i < 3; i++) {
      if (p3d_equal_config(robotPt, q_list_new[i%3], q_list_new[(i+1)%3])) {
        PrintInfo(("2 confs of the facet are equal, test local path\n"));
        path = p3d_local_planner(robotPt, q_list_new[i%3], q_list_new[(i+2)%3]);
        if (path == NULL) {
          PrintInfo(("Error : impossible de planifier\n"));
          DEPTH--;
          return FALSE;
        }
        unvalid = p3d_unvalid_localpath_test(robotPt, path, &(robotPt->GRAPH->nb_test_coll));
        path->destroy(robotPt, path);
        path = NULL;
        DEPTH--;
        return (!unvalid);
      }
    }

    DEPTH++;
    test1 = p3d_test_col_face(robotPt, q_list_new, R_list_new);
    if (test1 == FALSE) {
      if (qnew != NULL) {
        p3d_destroy_config(robotPt, qnew);
        qnew = NULL;
      }
      MY_FREE(ind, int, 3);
      MY_FREE(e, double, 3);
      MY_FREE(distances, double, robotPt->njoints + 1);
      MY_FREE(q_list_new, configPt, 3);
      MY_FREE(R_list_new, double, 3);
      DEPTH--;
      return FALSE;
    }
    q_list_new[0] =  q_list[ind[2]];
    q_list_new[1] =  qb;
    q_list_new[2] =  qnew;

    R_list_new[0] =  R_list[ind[2]];
    R_list_new[1] =  Rb;
    R_list_new[2] =  Rnew;

    DEPTH++;
    test2 = p3d_test_col_face(robotPt, q_list_new, R_list_new);
    if (qnew != NULL) {
      p3d_destroy_config(robotPt, qnew);
      qnew = NULL;
    }
    MY_FREE(ind, int, 3);
    MY_FREE(e, double, 3);
    MY_FREE(distances, double, robotPt->njoints + 1);
    MY_FREE(q_list_new, configPt, 3);
    MY_FREE(R_list_new, double, 3);
    DEPTH--;
    return (test2);

  }
}
//end path deform
