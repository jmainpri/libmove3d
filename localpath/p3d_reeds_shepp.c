#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Localpath-pkg.h"
#include "Graphic-pkg.h"

enum {Q1_NOT_FOUND, Q1_ON_CURRENT_SEG, Q1_FOUND_NOT_Q2, Q2_FOUND};


/* Definition for the Reeds and Shepp parameter */
#define DOF_RS_X     0
#define DOF_RS_Y     1
#define DOF_RS_THETA 2
#define DOF_RS_Z     3

/* Definition for the trailer parameter */
#define DOF_X     0
#define DOF_Y     1
#define DOF_THETA 2
#define DOF_PHI   3
#define DOF_DC_DS 4

#define JNT_BASE    0
#define JNT_TRAILER 1

/* computes the range of the parameter of a Reeds and Shepp local path */
double p3d_rs_compute_range_param(p3d_localpath *localpathPt) {
  double range = 0;
  p3d_rs_data * rs_segmentPt = localpathPt->specific.rs_data;

  while (rs_segmentPt != NULL) {
    range += rs_segmentPt->val_rs;
    rs_segmentPt = rs_segmentPt->next_rs;
  }
  return range;
}

/* allocation of a data structure specific to a local path of type
   Reeds and Shepp */
p3d_rs_data * p3d_alloc_spec_rs_localpath(configPt q_i, configPt q_f,
    double xcenter, double ycenter,
    double radius, whichway dir_rs,
    double val_rs, int type_rs,
    pp3d_rs_data next_rs,
    pp3d_rs_data prev_rs) {
  p3d_rs_data * rs_data;

  if ((rs_data = MY_ALLOC(p3d_rs_data, 1)) == NULL)
    return NULL;

  rs_data->q_init = q_i;
  rs_data->q_end = q_f;
  rs_data->centre_x = xcenter;
  rs_data->centre_y = ycenter;
  rs_data->radius = radius;
  rs_data->dir_rs = dir_rs;
  rs_data->val_rs = val_rs;
  rs_data->type_rs = type_rs;
  rs_data->next_rs = next_rs;
  rs_data->prev_rs = prev_rs;

  return rs_data;
}

/* Allocation of a local path of type Reeds and Shepp
 *
 * The fields prev_lp and next_lp are set to NULL.
 */
p3d_localpath *p3d_alloc_rs_localpath(p3d_rob *robotPt,
                                      configPt q_i, configPt q_f,
                                      double xcenter, double ycenter,
                                      double radius, whichway dir_rs,
                                      double val_rs, int type_rs,
                                      int lp_id, pp3d_rs_data next_rs,
                                      pp3d_rs_data prev_rs,
                                      int is_valid) {
  p3d_localpath * localpathPt = NULL;

  if ((localpathPt = MY_ALLOC(p3d_localpath, 1)) == NULL)
    return NULL;

  /* allocation specific part */
  localpathPt->specific.rs_data =
    p3d_alloc_spec_rs_localpath(q_i, q_f, xcenter,
                                ycenter, radius,
                                dir_rs, val_rs,
                                type_rs, next_rs,
                                prev_rs);

  if (localpathPt->specific.rs_data == NULL) {
    /* allocation failed: free allocated structures and return NULL*/
    MY_FREE(localpathPt, p3d_localpath, 1);
    return NULL;
  }
  /* initialization of the generic part */
  /* fields */
  localpathPt->type_lp = REEDS_SHEPP;
  localpathPt->valid = is_valid;
  localpathPt->range_param = p3d_rs_compute_range_param(localpathPt);
  localpathPt->lp_id = lp_id;
  localpathPt->next_lp = NULL;
  localpathPt->prev_lp = NULL;

#ifdef MULTILOCALPATH
	localpathPt->mlpID = -1;

	for(int j=0; j< MAX_MULTILOCALPATH_NB ; j++) {
		localpathPt->mlpLocalpath[j] = NULL;
	}
#endif

  /* methods associated to the local path */
  /* compute the length of the local path */
  localpathPt->length =
    (double(*)(p3d_rob*, p3d_localpath*))(p3d_rs_dist);
  /* extract from a local path a sub local path starting from length
     l1 and ending at length l2 */
  localpathPt->extract_sub_localpath =
    (p3d_localpath * (*)(p3d_rob*, p3d_localpath * localpathPt,
                         double l1, double l2))(p3d_extract_rs);
  /* extract from a local path a sub local path starting from parameter
     u1 and ending at parameter u2 */
  localpathPt->extract_by_param =
    (p3d_localpath * (*)(p3d_rob*, p3d_localpath*,
                         double, double))(p3d_extract_rs_by_param);
  /* destroy the localpath */
  localpathPt->destroy =
    (void(*)(p3d_rob*, p3d_localpath*))(p3d_rs_destroy);
  /*copy the local path */
  localpathPt->copy =
    (p3d_localpath * (*)(p3d_rob*,
                         p3d_localpath*))(p3d_copy_rs_localpath);
  /* computes the configuration at given distance along the path */
  localpathPt->config_at_distance =
    (configPt(*)(p3d_rob*,
                 p3d_localpath*, double))(p3d_rs_config_at_distance);
  /* computes the configuration at given parameter along the path */
  localpathPt->config_at_param =
    (configPt(*)(p3d_rob*, p3d_localpath*,
                 double))(p3d_rs_config_at_param);
  /* from a configuration on a local path, this function computes an
     interval of parameter on the local path on which all the points
     of the robot move by less than the distance given as input.
     The interval is centered on the configuration given as input. The
     function returns the half length of the interval */
  localpathPt->stay_within_dist =
    (double(*)(p3d_rob*, p3d_localpath*,
               double, whichway, double*))(p3d_rs_stay_within_dist);
  /* compute the cost of a local path */
  localpathPt->cost =
    (double(*)(p3d_rob*, p3d_localpath*))(p3d_rs_cost);
  /* function that simplifies the sequence of two local paths: valid
     only for RS curves */
  localpathPt->simplify =
    (p3d_localpath * (*)(p3d_rob*, p3d_localpath*, int*))(p3d_simplify_rs);
  /* write the local path in a file */
  localpathPt->write =
    (int(*)(FILE * file, p3d_rob*, p3d_localpath*))(p3d_write_rs);

  /* update length and range of parameter */
  localpathPt->length_lp = p3d_rs_dist(robotPt, localpathPt);
  localpathPt->range_param = p3d_rs_compute_range_param(localpathPt);
  localpathPt->ikSol = NULL;
    //save the active constraints
  localpathPt->activeCntrts = p3d_getActiveCntrts(robotPt,&(localpathPt->nbActiveCntrts));
  return localpathPt;
}

/* put a Reeds and Shepp local path into a localpath structure.
 *
 */
p3d_localpath * p3d_rs_data_into_localpath(p3d_rob *robotPt,
    p3d_rs_data *rs_dataPt,
    int is_valid, int lp_id) {
  p3d_localpath *localpathPt = NULL;

  if ((localpathPt = MY_ALLOC(p3d_localpath, 1)) == NULL)
    return NULL;

  /* initialization of pointer to specific part */
  localpathPt->specific.rs_data = rs_dataPt;

  /* initialization of the generic part */
  localpathPt->type_lp = REEDS_SHEPP;
  localpathPt->prev_lp = NULL;
  localpathPt->next_lp = NULL;
  localpathPt->valid = is_valid;
  localpathPt->lp_id = lp_id;
  localpathPt->range_param = p3d_rs_compute_range_param(localpathPt);
  localpathPt->ikSol = NULL;

  /* methods associated to the local path */
  /* compute the length of the local path */
  localpathPt->length =
    (double(*)(p3d_rob*, p3d_localpath*))(p3d_rs_dist);
  /* extract from a local path a sub local path starting from length
     l1 and ending at length l2 */
  localpathPt->extract_sub_localpath =
    (p3d_localpath * (*)(p3d_rob*, p3d_localpath*,
                         double, double))(p3d_extract_rs);
  /* extract from a local path a sub local path starting from parameter
     u1 and ending at parameter u2 */
  localpathPt->extract_by_param =
    (p3d_localpath * (*)(p3d_rob*, p3d_localpath*,
                         double, double))(p3d_extract_rs_by_param);
  /* destroy the localpath */
  localpathPt->destroy =
    (void(*)(p3d_rob*, p3d_localpath*))(p3d_rs_destroy);
  /*copy the local path */
  localpathPt->copy =
    (p3d_localpath * (*)(p3d_rob*,
                         p3d_localpath*))(p3d_copy_rs_localpath);
  /* computes the configuration at given distance along the path */
  localpathPt->config_at_distance =
    (configPt(*)(p3d_rob*,
                 p3d_localpath*, double))(p3d_rs_config_at_distance);
  /* computes the configuration at given parameter along the path */
  localpathPt->config_at_param =
    (configPt(*)(p3d_rob*, p3d_localpath*,
                 double))(p3d_rs_config_at_param);
  /* from a configuration on a local path, this function computes an
     interval of parameter on the local path on which all the points
     of the robot move by less than the distance given as input.
     The interval is centered on the configuration given as input. The
     function returns the half length of the interval */
  localpathPt->stay_within_dist =
    (double(*)(p3d_rob*, p3d_localpath*,
               double, whichway, double*))(p3d_rs_stay_within_dist);
  /* compute the cost of a local path */
  localpathPt->cost =
    (double(*)(p3d_rob*, p3d_localpath*))(p3d_rs_cost);
  /* function that simplifies the sequence of two local paths: valid
     only for RS curves */
  localpathPt->simplify =
    (p3d_localpath * (*)(p3d_rob*, p3d_localpath*, int*))(p3d_simplify_rs);
  /* write the local path in a file */
  localpathPt->write =
    (int(*)(FILE *, p3d_rob*, p3d_localpath*))(p3d_write_rs);

  /* update length and range of parameter */
  localpathPt->length_lp = p3d_rs_dist(robotPt, localpathPt);
  localpathPt->range_param = p3d_rs_compute_range_param(localpathPt);
  localpathPt->nbActiveCntrts = 0;
  localpathPt->activeCntrts = NULL;
  return localpathPt;
}

/*
 * Compute the length of a Reeds and Shepp segment: the length is
 * the sum of the RS length and of the euclidian distance in the joint
 * space of the arm if any
 *
 * Input:  the RS segment
 *         the number of joints
 *
 * Output: the length
 */

double p3d_length_rs_segment(p3d_rob *robotPt, p3d_rs_data* rs_dataPt) {
  double l = 0., ljnt = 0.;
  int i, j;
  lm_reeds_shepp_str *rs_paramPt =
    lm_get_reeds_shepp_lm_param(robotPt);
  pflat_trailer_str trailer_paramPt =
    lm_get_trailer_lm_param(robotPt);
  int *other_jnt;
  int nbotherjnt;
  p3d_jnt * jntPt;

  if (trailer_paramPt) {
    other_jnt = trailer_paramPt->other_jnt;
    nbotherjnt = trailer_paramPt->nb_other_jnt;
  } else {
    other_jnt = rs_paramPt->other_jnt;
    nbotherjnt = rs_paramPt->nb_other_jnt;
  }
  if (rs_dataPt != NULL) {

    l = rs_dataPt->val_rs;
    for (i = 0; i < nbotherjnt; i++) {
      jntPt = robotPt->joints[other_jnt[i]];
      for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
        ljnt += SQR(p3d_jnt_calc_dof_dist(jntPt, j, rs_dataPt->q_init,
                                          rs_dataPt->q_end));
      }
    }
    l = l + sqrt(ljnt);
  }
  return l;
}


/*
 * Compute the distance for the Reeds and Shepp local method:
 * the distance is the sum of the field val_rs of the local path
 * and the euclidian distance in the joint space of the arm if any
 */
double p3d_rs_dist(p3d_rob *robotPt, p3d_localpath *localpathPt) {
  double l = 0.;
  /* cast of the pointer toward union p3d_lm_specific to a pointer
     toward p3d_rs_data */
  p3d_rs_data *specificPt = (p3d_rs_data *) localpathPt->specific.rs_data;

  /* test whether the type of local method is the expected one */
  if (localpathPt->type_lp != REEDS_SHEPP) {
    PrintError(("p3d_rs_dist: Reeds and Shepp local path expected\n"));
    return 0;
  }
  /* length is sum of all segment lengths */
  while (specificPt != NULL) {
    l += p3d_length_rs_segment(robotPt, specificPt);
    specificPt = specificPt->next_rs;
  }
  return(l);
}

/*
 * Copy one localpath of type Reeds and Shepp. Set the fields prev_lp and
 * next_lp to NULL.
 */
p3d_localpath *p3d_copy_rs_localpath(p3d_rob* robotPt,
                                     p3d_localpath *localpathPt) {
  p3d_localpath *copy_localpathPt = NULL;
  p3d_rs_data *rs_specificPt = NULL, *new_rs_specificPt = NULL,
                               *copy_rs_specificPt = NULL;
  configPt q1, q2;

  if (localpathPt == NULL) {
    return NULL;
  }
  /* test whether the type of local method is the expected one */
  if (localpathPt->type_lp != REEDS_SHEPP) {
    PrintError(("p3d_copy_rs_localpath: Reeds and Shepp local path expected\n"));
    return NULL;
  }

  rs_specificPt = localpathPt->specific.rs_data;

  while (rs_specificPt != NULL) {
    q1 = p3d_copy_config(robotPt, rs_specificPt->q_init);
    q2 = p3d_copy_config(robotPt, rs_specificPt->q_end);

    copy_rs_specificPt =
      p3d_alloc_spec_rs_localpath(q1, q2,
                                  rs_specificPt->centre_x,
                                  rs_specificPt->centre_y,
                                  rs_specificPt->radius,
                                  rs_specificPt->dir_rs,
                                  rs_specificPt->val_rs,
                                  rs_specificPt->type_rs,
                                  NULL, NULL);
    if (new_rs_specificPt == NULL) {
      /*first segment of the local path */
      copy_localpathPt = p3d_rs_data_into_localpath(robotPt,
                         copy_rs_specificPt,
                         localpathPt->valid,
                         localpathPt->lp_id);
      new_rs_specificPt = copy_rs_specificPt;
    } else {
      new_rs_specificPt->next_rs = copy_rs_specificPt;
      copy_rs_specificPt->prev_rs = new_rs_specificPt;
      new_rs_specificPt = copy_rs_specificPt;
    }

    rs_specificPt = rs_specificPt->next_rs;
  }
  /* update length and range of parameter */
  copy_localpathPt->length_lp = p3d_rs_dist(robotPt, copy_localpathPt);
  copy_localpathPt->range_param = p3d_rs_compute_range_param(copy_localpathPt);
  p3d_copy_iksol(robotPt->cntrt_manager, localpathPt->ikSol, &(copy_localpathPt->ikSol));
  copy_localpathPt->nbActiveCntrts = localpathPt->nbActiveCntrts;
  copy_localpathPt->activeCntrts = MY_ALLOC(int, copy_localpathPt->nbActiveCntrts);
  for(int i = 0; i < copy_localpathPt->nbActiveCntrts; i++){
    copy_localpathPt->activeCntrts[i] = localpathPt->activeCntrts[i];
  }
  return copy_localpathPt;
}

/*
 *  Computes the configuration along a Reeds and Shepp segment corresponding
 *  to a given length. The length includes the motion of the arm if any
 *
 *  Input:  a Reeds and Shepp portion
 *          the number of joints
 *          the length
 *
 *  Output: the configuration
 *
 *  Allocation: the resulting configuration
 */

static configPt p3d_get_conf_along_rs_by_length(p3d_rob * robotPt,
    p3d_rs_data *rs_specificPt,
    double l, int car_config) {
  double a, a1, a2, steer_angle;
  configPt q = NULL;
  int i, j;
  double lmaill = 0.;
  double radius = rs_specificPt->radius;
  double theta_init, theta_end;
  lm_reeds_shepp_str *rs_paramPt =
    lm_get_reeds_shepp_lm_param(robotPt);
  pflat_trailer_str trailer_paramPt =
    lm_get_trailer_lm_param(robotPt);
  int x_coord, y_coord, theta_coord, z_coord;
  int *other_jnt, nb_other_jnt;
  p3d_jnt * jntPt;

  if (rs_paramPt == NULL) {
    PrintWarning(("  No Reeds and Shepp local method specified\n"));
    return NULL;
  }

  if ((trailer_paramPt != NULL) && (car_config == TRUE)) {
    other_jnt = trailer_paramPt->other_jnt;
    nb_other_jnt = trailer_paramPt->nb_other_jnt;
    x_coord = trailer_paramPt->numdof[DOF_X];
    y_coord = trailer_paramPt->numdof[DOF_Y];
    theta_coord = trailer_paramPt->numdof[DOF_THETA];
  } else {
    other_jnt = rs_paramPt->other_jnt;
    nb_other_jnt = rs_paramPt->nb_other_jnt;
    x_coord = rs_paramPt->numdof[DOF_RS_X];
    y_coord = rs_paramPt->numdof[DOF_RS_Y];
    theta_coord = rs_paramPt->numdof[DOF_RS_THETA];
  }

  z_coord = rs_paramPt->numdof[DOF_RS_Z];

  /* allocation of a structure that will store the resulting
     configuration */
  q = p3d_copy_config(robotPt, rs_specificPt->q_init);

  lmaill = p3d_length_rs_segment(robotPt, rs_specificPt);
  /* debug florent */
  if (lmaill < EPS6) {
    PrintInfo(("  p3d_get_conf_along_rs_by_length: segment of length 0\n"));
    return q;
  }

  if ((l / lmaill) - 1 > EPS6) {
    l = lmaill;
  }
  if (l < EPS6) {
    l = 0.0;
  }

  theta_init = rs_specificPt->q_init[theta_coord];
  theta_end = rs_specificPt->q_end[theta_coord];

  switch (rs_specificPt->type_rs) {
    case RIGHT:
      steer_angle = -1.0;
      if (rs_specificPt->dir_rs > 0) {
        a1 = angle_limit_2PI(theta_init + M_PI_2);
        a2 = angle_limit_2PI(theta_end + M_PI_2);
        if (a1 < a2) {
          a1 = a1 + 2 * M_PI;
        }
      } else {
        a1 = angle_limit_2PI(theta_init + M_PI_2);
        a2 = angle_limit_2PI(theta_end + M_PI_2);
        if (a1 > a2) {
          a1 = a1 - 2 * M_PI;
        }
      }
      a = (1 - (l / lmaill)) * a1 + (l / lmaill) * a2;

      q[x_coord] = rs_specificPt->centre_x + radius * cos(a);
      q[y_coord] = rs_specificPt->centre_y + radius * sin(a);
      if (z_coord >= 0) {
        q[z_coord] = (1 - (l / lmaill)) * rs_specificPt->q_init[z_coord] +
                     (l / lmaill) * rs_specificPt->q_end[z_coord];
      }
      q[theta_coord] = a - M_PI_2;
      q[theta_coord] = angle_limit_PI(q[theta_coord]);

      /* other degrees of freedom */
      for (i = 0; i < nb_other_jnt; i++) {
        jntPt = robotPt->joints[other_jnt[i]];
        for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
          q[jntPt->index_dof+j] = p3d_jnt_calc_dof_value(
                                    jntPt, j, rs_specificPt->q_init,
                                    rs_specificPt->q_end, l / lmaill);
        }
      }
      break;
    case LEFT:
      steer_angle = 1.0;
      if (rs_specificPt->dir_rs > 0) {
        a1 = angle_limit_2PI(theta_init - M_PI_2);
        a2 = angle_limit_2PI(theta_end - M_PI_2);
        if (a1 > a2) {
          a1 = a1 - 2 * M_PI;
        }
      } else {
        a1 = angle_limit_2PI(theta_init - M_PI_2);
        a2 = angle_limit_2PI(theta_end - M_PI_2);
        if (a1 < a2) {
          a1 = a1 + 2 * M_PI;
        }
      }
      a = (1 - (l / lmaill)) * a1 + (l / lmaill) * a2;
      q[x_coord] = rs_specificPt->centre_x + radius * cos(a);
      q[y_coord] = rs_specificPt->centre_y + radius * sin(a);
      if (z_coord >= 0) {
        q[z_coord] = (1 - (l / lmaill)) * rs_specificPt->q_init[z_coord] +
                     (l / lmaill) * rs_specificPt->q_end[z_coord];
      }
      q[theta_coord] = a + M_PI_2;
      q[theta_coord] = angle_limit_PI(q[theta_coord]);

      /* other degrees of freedom */
      for (i = 0; i < nb_other_jnt; i++) {
        jntPt = robotPt->joints[other_jnt[i]];
        for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
          q[jntPt->index_dof+j] = p3d_jnt_calc_dof_value(
                                    jntPt, j, rs_specificPt->q_init,
                                    rs_specificPt->q_end, l / lmaill);
        }
      }
      break;
    case STRAIGHT:
      steer_angle = 0.0;
      q[x_coord] = (1 - (l / lmaill)) * rs_specificPt->q_init[x_coord] +
                   (l / lmaill) * rs_specificPt->q_end[x_coord];
      q[y_coord] = (1 - (l / lmaill)) * rs_specificPt->q_init[y_coord] +
                   (l / lmaill) * rs_specificPt->q_end[y_coord];
      if (z_coord >= 0) {
        q[z_coord] = (1 - (l / lmaill)) * rs_specificPt->q_init[z_coord] +
                     (l / lmaill) * rs_specificPt->q_end[z_coord];
      }
      q[theta_coord] = (1 - (l / lmaill)) * theta_init +
                       (l / lmaill) * theta_end;

      /* other degrees of freedom */
      for (i = 0; i < nb_other_jnt; i++) {
        jntPt = robotPt->joints[other_jnt[i]];
        for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
          q[jntPt->index_dof+j] = p3d_jnt_calc_dof_value(
                                    jntPt, j, rs_specificPt->q_init,
                                    rs_specificPt->q_end, l / lmaill);
        }
      }
      break;
    default:
      PrintError(("p3d_get_conf_along_rs_by_length : ERROR : local path not valid...\n"));
      return(NULL);
  }
  if ((trailer_paramPt != NULL) && (car_config == TRUE)) {
    /* The robot is a car modeled by a virtual car-trailer system */
    double x_rear = q[x_coord];
    double y_rear = q[y_coord];
    double theta_car = q[theta_coord];
    double length = trailer_paramPt->flat_str->distAxleToAxis.l2;
    double x_front = x_rear + length * cos(theta_car);
    double y_front = y_rear + length * sin(theta_car);
    int trailer_coord = trailer_paramPt->numdof[DOF_PHI];
    int dc_ds_coord = trailer_paramPt->numdof[DOF_DC_DS];

    steer_angle *= atan(length / radius);
    q[x_coord] = x_front;
    q[y_coord] = y_front;
    q[theta_coord] = angle_limit_PI(theta_car + steer_angle);
    q[trailer_coord] = -steer_angle;
    q[dc_ds_coord] = 0;
  }

  return(q);
}

/*
 *  Extract from a RS+arm local path the sub local path starting
 *  at length l1 and ending at length l2.
 *  The length of the extracted local path is computed
 *
 *  If l2 > length local path, return end of local path
 */

p3d_localpath *p3d_extract_rs(p3d_rob *robotPt,
                              p3d_localpath *localpathPt,
                              double l1, double l2) {
  p3d_rs_data * rs_specificPt = NULL, *cur_segPt = NULL, *first_segPt = NULL;
  int search_status = Q1_NOT_FOUND, next_segment = FALSE;
  double dist = 0, distnext = 0, val_rs, new_val_rs = 0, length;
  configPt q1 = NULL, q2 = NULL;
  p3d_localpath *resultPt = NULL;

  if (localpathPt == NULL) {
    return NULL;
  }
  if (l1 > l2)
    return NULL;

  /* test whether the type of local method is the expected one */
  if (localpathPt->type_lp != REEDS_SHEPP) {
    PrintError(("p3d_extract_rs: Reeds and Shepp local path expected\n"));
    return NULL;
  }
  rs_specificPt = localpathPt->specific.rs_data;
  length = p3d_length_rs_segment(robotPt, rs_specificPt);
  val_rs = rs_specificPt->val_rs;
  distnext = length;

  while ((rs_specificPt != NULL) && (search_status != Q2_FOUND)) {
    /* go to next segment*/
    if (next_segment) {
      rs_specificPt = rs_specificPt->next_rs;
      dist = distnext;
      length = p3d_length_rs_segment(robotPt, rs_specificPt);
      val_rs = rs_specificPt->val_rs;
      distnext += length;
    }
    next_segment = FALSE;
    switch (search_status) {
      case Q1_NOT_FOUND:
        /* q1 has not been found yet */
        if ((dist <= l1) && (l1 < distnext)) {
          /* q1 is on the current segment */
          search_status = Q1_ON_CURRENT_SEG;
          q1 = p3d_get_conf_along_rs_by_length(robotPt,
                                               rs_specificPt,
                                               l1 - dist, FALSE);
          if (robotPt->cntrt_manager->cntrts != NULL) {
            p3d_set_and_update_this_robot_conf_multisol(robotPt, q1, NULL, 0, localpathPt->ikSol);
            p3d_get_robot_config_into(robotPt, &q1);
          }
        } else {
          /* go to the next segment */
          next_segment = TRUE;
        }
        break;
      case Q1_ON_CURRENT_SEG:
        if ((dist <= l2) && (l2 <= distnext + EPS6)) {
          /* q2 is on the same segment as q1 */
          search_status = Q2_FOUND;
          q2 = p3d_get_conf_along_rs_by_length(robotPt,
                                               rs_specificPt,
                                               l2 - dist, FALSE);
          if (robotPt->cntrt_manager->cntrts != NULL) {
            p3d_set_and_update_this_robot_conf_multisol(robotPt, q2, NULL, 0, localpathPt->ikSol);
            p3d_get_robot_config_into(robotPt, &q2);
          }
          new_val_rs = (l2 - l1) * val_rs / length;
        } else {
          /* q2 is on a different segment as q1 */
          search_status = Q1_FOUND_NOT_Q2;
          q2 = p3d_copy_config(robotPt, rs_specificPt->q_end);
          if (robotPt->cntrt_manager->cntrts != NULL) {
            p3d_set_and_update_this_robot_conf_multisol(robotPt, q2, NULL, 0, localpathPt->ikSol);
            p3d_get_robot_config_into(robotPt, &q2);
          }
          new_val_rs = (distnext - l1) * val_rs / length;

          /* go to the next segment */
          next_segment = TRUE;
        }
        break;

      case Q1_FOUND_NOT_Q2:
        if ((dist <= l2) && (l2 < distnext + EPS6)) {
          /* q2 is on the current segment */
          search_status = Q2_FOUND;
          q1 = p3d_copy_config(robotPt, rs_specificPt->q_init);
          if (robotPt->cntrt_manager->cntrts != NULL) {
            p3d_set_and_update_this_robot_conf_multisol(robotPt, q1, NULL, 0, localpathPt->ikSol);
            p3d_get_robot_config_into(robotPt, &q1);
          }
          q2 = p3d_get_conf_along_rs_by_length(robotPt,
                                               rs_specificPt,
                                               l2 - dist, FALSE);
          if (robotPt->cntrt_manager->cntrts != NULL) {
            p3d_set_and_update_this_robot_conf_multisol(robotPt, q2, NULL, 0, localpathPt->ikSol);
            p3d_get_robot_config_into(robotPt, &q2);
          }
          new_val_rs = (l2 - dist) * val_rs / length;
        } else {
          /* q2 is on a different segment as q1 */
          search_status = Q1_FOUND_NOT_Q2;
          q1 = p3d_copy_config(robotPt, rs_specificPt->q_init);
          if (robotPt->cntrt_manager->cntrts != NULL) {
            p3d_set_and_update_this_robot_conf_multisol(robotPt, q1, NULL, 0, localpathPt->ikSol);
            p3d_get_robot_config_into(robotPt, &q1);
          }
          q2 = p3d_copy_config(robotPt, rs_specificPt->q_end);
          if (robotPt->cntrt_manager->cntrts != NULL) {
            p3d_set_and_update_this_robot_conf_multisol(robotPt, q2, NULL, 0, localpathPt->ikSol);
            p3d_get_robot_config_into(robotPt, &q2);
          }

          new_val_rs = rs_specificPt->val_rs;

          /* go to the next segment */
          next_segment = TRUE;
        }
        break;
    }
    /* copy extracted segments */
    if ((q1 != NULL) && (q2 != NULL)) {
      if (first_segPt == NULL) {
        first_segPt = p3d_alloc_spec_rs_localpath(q1, q2,
                      rs_specificPt->centre_x,
                      rs_specificPt->centre_y,
                      rs_specificPt->radius,
                      rs_specificPt->dir_rs,
                      new_val_rs,
                      rs_specificPt->type_rs,
                      NULL, NULL);
        cur_segPt = first_segPt;
      } else {
        cur_segPt->next_rs =
          p3d_alloc_spec_rs_localpath(q1, q2,
                                      rs_specificPt->centre_x,
                                      rs_specificPt->centre_y,
                                      rs_specificPt->radius,
                                      rs_specificPt->dir_rs,
                                      new_val_rs,
                                      rs_specificPt->type_rs,
                                      NULL, cur_segPt);
        cur_segPt = cur_segPt->next_rs;
      }
    }

  } /* end of loop over the RS segments */

  if (search_status == Q2_FOUND) {
    /* if search is successful, put the RS segment list into
       a local path and compute length */
    resultPt = p3d_rs_data_into_localpath(robotPt, first_segPt,
                                          localpathPt->valid,
                                          localpathPt->lp_id);
    /* update length and range of parameter */
    resultPt->length_lp = resultPt->length(robotPt, resultPt);
    resultPt->range_param = p3d_rs_compute_range_param(resultPt);
    p3d_copy_iksol(robotPt->cntrt_manager, localpathPt->ikSol, &(resultPt->ikSol));
    resultPt->nbActiveCntrts = localpathPt->nbActiveCntrts;
    resultPt->activeCntrts = MY_ALLOC(int, resultPt->nbActiveCntrts);
    for(int i = 0; i < resultPt->nbActiveCntrts; i++){
      resultPt->activeCntrts[i] = localpathPt->activeCntrts[i];
    }
  }
  return resultPt;
}

/*
 *  Extract from a RS+arm local path the sub local path starting
 *  at parameter u1 and ending at parameter u2.
 *  The length of the extracted local path is computed
 *
 *  If u2 > range_param local path, return end of local path
 */

p3d_localpath *p3d_extract_rs_by_param(p3d_rob *robotPt,
                                       p3d_localpath *localpathPt,
                                       double u1, double u2) {
  double length = localpathPt->length_lp;
  double range_param = localpathPt->range_param;
  double l1, l2;

  l1 = length * u1 / range_param;
  l2 = length * u2 / range_param;

  return p3d_extract_rs(robotPt, localpathPt, l1, l2);

}


/*
 * destroy a structure of type p3d_rs_data and the structures pointed by
 * the successive fields next_rs of this structure. The initial and final
 * configurations of the path are also destroyed.
 */
void p3d_destroy_rs_data(p3d_rob* robotPt, p3d_rs_data* rs_dataPt) {
  p3d_rs_data *prev_rs_dataPt = NULL,
                                *destr_rs_dataPt = NULL;

  if (rs_dataPt == NULL)
    return;

  /* if rs_dataPt is in the middle of a list, cut the list after the
     previous element */
  if (rs_dataPt->prev_rs != NULL) {
    prev_rs_dataPt = rs_dataPt->prev_rs;
    prev_rs_dataPt->next_rs = NULL;
  }

  destr_rs_dataPt = rs_dataPt;

  while (destr_rs_dataPt != NULL) {
    rs_dataPt = destr_rs_dataPt->next_rs;

    p3d_destroy_config(robotPt, destr_rs_dataPt->q_init);
    p3d_destroy_config(robotPt, destr_rs_dataPt->q_end);
    MY_FREE(destr_rs_dataPt, p3d_rs_data, 1);

    destr_rs_dataPt = rs_dataPt;
  }
}

/*
 * destroy a Reeds and Shepp local path
 */
void p3d_rs_destroy(p3d_rob* robotPt, p3d_localpath* localpathPt) {
  if (localpathPt != NULL) {

    /* test whether the type of local method is the expected one */
    if (localpathPt->type_lp != REEDS_SHEPP) {
      PrintError(("p3d_rs_destroy: Reeds and Shepp local path expected\n"));
      return;
    }
    /* destroy the specific part */
    if (localpathPt->specific.rs_data != NULL) {
      p3d_destroy_rs_data(robotPt, localpathPt->specific.rs_data);
    }
    localpathPt->next_lp = NULL;
    localpathPt->prev_lp = NULL;
    if (localpathPt->ikSol) {
      p3d_destroy_specific_iksol(robotPt->cntrt_manager, localpathPt->ikSol);
      localpathPt->ikSol = NULL;
    }
    MY_FREE(localpathPt->activeCntrts, int, localpathPt->nbActiveCntrts);
    MY_FREE(localpathPt, p3d_localpath, 1);
  }
}

/*
 *  Computes the configuration along a Reeds and Shepp segment corresponding
 *  to a given parameter. The parameter corresponds to the length along
 *  the Reeds and Shepp curve, without taking into account the motion of
 *  the arm
 *
 *  Input:  a Reeds and Shepp portion
 *          the number of joints
 *          the parameter
 *
 *  Output: the configuration
 *
 *  Allocation: the resulting configuration
 */

static configPt p3d_get_conf_along_rs_by_param(p3d_rob* robotPt,
    p3d_rs_data *rs_specificPt,
    double u, int njnt) {
  double a, a1, a2, steer_angle;
  configPt q = NULL;
  int i, j;
  double val_rs;
  double radius = rs_specificPt->radius;
  double theta_init, theta_end;
  lm_reeds_shepp_str *rs_paramPt =
    lm_get_reeds_shepp_lm_param(robotPt);
  pflat_trailer_str trailer_paramPt =
    lm_get_trailer_lm_param(robotPt);
  p3d_jnt * jntPt;
  int x_coord, y_coord, z_coord, theta_coord;
  int *other_jnt, nb_other_jnt;

  if (rs_paramPt == NULL) {
    PrintWarning(("  No Reeds and Shepp local method specified\n"));
    return NULL;
  }

  if (trailer_paramPt != NULL) {
    other_jnt = trailer_paramPt->other_jnt;
    nb_other_jnt = trailer_paramPt->nb_other_jnt;
    x_coord = trailer_paramPt->numdof[DOF_X];
    y_coord = trailer_paramPt->numdof[DOF_Y];
    theta_coord = trailer_paramPt->numdof[DOF_THETA];
  } else {
    other_jnt = rs_paramPt->other_jnt;
    nb_other_jnt = rs_paramPt->nb_other_jnt;
    x_coord = rs_paramPt->numdof[DOF_RS_X];
    y_coord = rs_paramPt->numdof[DOF_RS_Y];
    theta_coord = rs_paramPt->numdof[DOF_RS_THETA];
  }

  z_coord = rs_paramPt->numdof[DOF_RS_Z];

  /* allocation of a structure that will store the resulting
     configuration */
  q = p3d_copy_config(robotPt, rs_specificPt->q_init);

  val_rs = rs_specificPt->val_rs;

  if ((u / val_rs) - 1 > EPS6) {
    u = val_rs;
  }
  if (u < EPS6) {
    u = 0.0;
  }

  theta_init = rs_specificPt->q_init[theta_coord];
  theta_end = rs_specificPt->q_end[theta_coord];

  switch (rs_specificPt->type_rs) {
    case RIGHT:
      steer_angle = -1.0;
      if (rs_specificPt->dir_rs > 0) {
        a1 = angle_limit_2PI(theta_init + M_PI_2);
        a2 = angle_limit_2PI(theta_end + M_PI_2);
        if (a1 < a2) {
          a1 = a1 + 2 * M_PI;
        }
      } else {
        a1 = angle_limit_2PI(theta_init + M_PI_2);
        a2 = angle_limit_2PI(theta_end + M_PI_2);
        if (a1 > a2) {
          a1 = a1 - 2 * M_PI;
        }
      }
      a = (1 - (u / val_rs)) * a1 + (u / val_rs) * a2;

      q[x_coord] = rs_specificPt->centre_x + radius * cos(a);
      q[y_coord] = rs_specificPt->centre_y + radius * sin(a);
      if (z_coord >= 0) {
        q[z_coord] = (1 - (u / val_rs)) * rs_specificPt->q_init[z_coord] +
                     (u / val_rs) * rs_specificPt->q_end[z_coord];
      }
      q[theta_coord] = a - M_PI_2;
      q[theta_coord] = angle_limit_PI(q[theta_coord]);

      /* other degrees of freedom */
      for (i = 0; i < nb_other_jnt; i++) {
        jntPt = robotPt->joints[other_jnt[i]];
        for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
          q[jntPt->index_dof+j] = p3d_jnt_calc_dof_value(
                                    jntPt, j, rs_specificPt->q_init,
                                    rs_specificPt->q_end, u / val_rs);
        }
      }
      break;
    case LEFT:
      steer_angle = 1.0;
      if (rs_specificPt->dir_rs > 0) {
        a1 = angle_limit_2PI(theta_init - M_PI_2);
        a2 = angle_limit_2PI(theta_end - M_PI_2);
        if (a1 > a2) {
          a1 = a1 - 2 * M_PI;
        }
      } else {
        a1 = angle_limit_2PI(theta_init - M_PI_2);
        a2 = angle_limit_2PI(theta_end - M_PI_2);
        if (a1 < a2) {
          a1 = a1 + 2 * M_PI;
        }
      }
      a = (1 - (u / val_rs)) * a1 + (u / val_rs) * a2;
      q[x_coord] = rs_specificPt->centre_x + radius * cos(a);
      q[y_coord] = rs_specificPt->centre_y + radius * sin(a);
      if (z_coord >= 0) {
        q[z_coord] = (1 - (u / val_rs)) * rs_specificPt->q_init[z_coord] +
                     (u / val_rs) * rs_specificPt->q_end[z_coord];
      }
      q[theta_coord] = a + M_PI_2;
      q[theta_coord] = angle_limit_PI(q[theta_coord]);

      /* other degrees of freedom */
      for (i = 0; i < nb_other_jnt; i++) {
        jntPt = robotPt->joints[other_jnt[i]];
        for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
          q[jntPt->index_dof+j] = p3d_jnt_calc_dof_value(
                                    jntPt, j, rs_specificPt->q_init,
                                    rs_specificPt->q_end, u / val_rs);
        }
      }
      break;
    case STRAIGHT:
      steer_angle = 0.0;
      q[x_coord] = (1 - (u / val_rs)) * rs_specificPt->q_init[x_coord] +
                   (u / val_rs) * rs_specificPt->q_end[x_coord];
      q[y_coord] = (1 - (u / val_rs)) * rs_specificPt->q_init[y_coord] +
                   (u / val_rs) * rs_specificPt->q_end[y_coord];
      if (z_coord >= 0) {
        q[z_coord] = (1 - (u / val_rs)) * rs_specificPt->q_init[z_coord] +
                     (u / val_rs) * rs_specificPt->q_end[z_coord];
      }
      q[theta_coord] = (1 - (u / val_rs)) * theta_init +
                       (u / val_rs) * theta_end;

      for (i = 0; i < nb_other_jnt; i++) {
        jntPt = robotPt->joints[other_jnt[i]];
        for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
          q[jntPt->index_dof+j] = p3d_jnt_calc_dof_value(
                                    jntPt, j, rs_specificPt->q_init,
                                    rs_specificPt->q_end, u / val_rs);
        }
      }
      break;
    default:
      PrintError(("p3d_get_conf_along_rs_by_param : ERROR : local path not valid...\n"));
      return(NULL);
  }

  if (trailer_paramPt != NULL) {
    /* The robot is a car modeled by a virtual car-trailer system */
    double x_rear = q[x_coord];
    double y_rear = q[y_coord];
    double theta_car = q[theta_coord];
    double length = trailer_paramPt->flat_str->distAxleToAxis.l2;
    double x_front = x_rear + length * cos(theta_car);
    double y_front = y_rear + length * sin(theta_car);
    int trailer_coord = trailer_paramPt->numdof[DOF_PHI];
    int dc_ds_coord = trailer_paramPt->numdof[DOF_DC_DS];

    steer_angle *= atan(length / radius);
    q[x_coord] = x_front;
    q[y_coord] = y_front;
    q[theta_coord] = angle_limit_PI(theta_car + steer_angle);
    q[trailer_coord] = -steer_angle;
    q[dc_ds_coord] = 0;
  }

  return(q);
}


/*
 *  Compute the configuration situated at given distance on the local path.
 *
 *  Input:  the robot, the distance.
 *
 *  Output: the configuration.
 *
 *  Allocation: the resulting configuration.
 */

configPt p3d_rs_config_at_distance(p3d_rob *robotPt,
                                   p3d_localpath *localpathPt,
                                   double length) {
  p3d_rs_data *rs_specificPt = NULL, *last_segmentPt = NULL;
  double l1 = 0, l2 = 0;
  int config_found = FALSE;
  configPt q = NULL;
  pflat_trailer_str trailer_paramPt =
    lm_get_trailer_lm_param(robotPt);

  if (localpathPt == NULL)
    return NULL;

  rs_specificPt = localpathPt->specific.rs_data;

  while ((rs_specificPt != NULL) && (config_found == FALSE)) {
    l2 += p3d_length_rs_segment(robotPt, rs_specificPt);

    if (l2 >= length) {
      /* the segment containing the configuration has been found */
      config_found = TRUE;
      q = p3d_get_conf_along_rs_by_length(robotPt, rs_specificPt,
                                          length - l1, TRUE);
    }
    l1 = l2;
    last_segmentPt = rs_specificPt;
    rs_specificPt = rs_specificPt->next_rs;
  }
  if (config_found == FALSE) {
    if (length - l2 < EPS6) {
      /* the config is close to the end of the local path */
      q = p3d_copy_config(robotPt, last_segmentPt->q_end);
      if (trailer_paramPt != NULL) {
        /* The robot is a car modeled by a virtual car-trailer system */
        double steer_angle = 0.0;
        double x_rear = q[trailer_paramPt->numdof[DOF_X]];
        double y_rear = q[trailer_paramPt->numdof[DOF_Y]];
        double theta_car = q[trailer_paramPt->numdof[DOF_THETA]];
        double length = trailer_paramPt->flat_str->distAxleToAxis.l2;
        double x_front = x_rear + length * cos(theta_car);
        double y_front = y_rear + length * sin(theta_car);

        q[trailer_paramPt->numdof[DOF_X]] = x_front;
        q[trailer_paramPt->numdof[DOF_Y]] = y_front;
        q[trailer_paramPt->numdof[DOF_THETA]] = angle_limit_PI(theta_car + steer_angle);
        q[trailer_paramPt->numdof[DOF_PHI]] = -steer_angle;
        q[trailer_paramPt->numdof[DOF_DC_DS]] = 0;
      }
    } else {
      /* the config is not on the local path */
      PrintError(("  p3d_rs_config_at_distance: config is not on the local path\n"));
    }
  }
  return q;
}

/*
 *
 *  Input:  the robot, the parameter.
 *
 *  Output: the configuration.
 *
 *  Description: Compute the configuration situated at given parameter
 *          on the local path.  parameter corresponds to length along
 *          Reeds and Shepp curve (without arm)
 */

configPt p3d_rs_config_at_param(p3d_rob *robotPt,
                                p3d_localpath *localpathPt,
                                double param) {
  p3d_rs_data *rs_specificPt = NULL, *last_segmentPt = NULL;
  double l1 = 0, l2 = 0;
  int njnt = robotPt->njoints;
  int config_found = FALSE;
  configPt q = NULL;
  pflat_trailer_str trailer_paramPt = lm_get_trailer_lm_param(robotPt);

  if (localpathPt == NULL)
    return NULL;

  rs_specificPt = localpathPt->specific.rs_data;

  while ((rs_specificPt != NULL) && (config_found == FALSE)) {
    l2 += rs_specificPt->val_rs;

    if (l2 >= param) {
      /* the segment containing the configuration has been found */
      config_found = TRUE;
      q = p3d_get_conf_along_rs_by_param(robotPt, rs_specificPt,
                                         param - l1, njnt);
    }
    l1 = l2;
    last_segmentPt = rs_specificPt;
    rs_specificPt = rs_specificPt->next_rs;
  }
  if (config_found == FALSE) {
    if (param - l2 < EPS6) {
      /* the config is close to the end of the local path */
      q = p3d_copy_config(robotPt, last_segmentPt->q_end);
      if (trailer_paramPt != NULL) {
        /* The robot is a car modeled by a virtual car-trailer system */
        double steer_angle = 0.0;
        double x_rear = q[trailer_paramPt->numdof[DOF_X]];
        double y_rear = q[trailer_paramPt->numdof[DOF_Y]];
        double theta_car = q[trailer_paramPt->numdof[DOF_THETA]];
        double length = trailer_paramPt->flat_str->distAxleToAxis.l2;
        double x_front = x_rear + length * cos(theta_car);
        double y_front = y_rear + length * sin(theta_car);

        q[trailer_paramPt->numdof[DOF_X]] = x_front;
        q[trailer_paramPt->numdof[DOF_Y]] = y_front;
        q[trailer_paramPt->numdof[DOF_THETA]] = angle_limit_PI(theta_car + steer_angle);
        q[trailer_paramPt->numdof[DOF_PHI]] = -steer_angle;
        q[trailer_paramPt->numdof[DOF_DC_DS]] = 0;
      }
    } else {
      /* the config is not on the local path */
      PrintError(("  p3d_rs_config_at_param: config is not on the local path\n"));
    }
  }
  return q;
}


/*
 *  rs_segment_at_parameter
 *
 *  Input:  the robot,
 *          the local path,
 *          the parameter.
 *
 *  Output: the RS segment,
 *          the local parameter on this RS segment.
 *
 *  Description: returns the RS segment corresponding to the input parameter
 *      on a RS local path. The parameter is changed into the local parameter
 *      along the segment.
 */


p3d_rs_data *rs_segment_at_parameter(p3d_rob *robotPt,
                                     p3d_rs_data *rs_segmentPt,
                                     double *parameterPt) {
  double param = *parameterPt;
  double val_rs = rs_segmentPt->val_rs;

  if (param < 0) {
    param = 0;
  }
  while (param > val_rs) {
    param = param - val_rs;
    if (rs_segmentPt->next_rs == NULL) {
      /* the end of the local path is reached */
      param = val_rs;
    } else {
      rs_segmentPt = rs_segmentPt->next_rs;
      val_rs = rs_segmentPt->val_rs;
    }
  }
  *parameterPt = param;
  return rs_segmentPt;
}

/*
 *  This function computes an interval of parameter which all the points
 *  of the joint move by less than the distance given as input.
 *
 *  Input: speed of the previous joint (prev_data),
 *         the robotPt (robotPt),
 *         the sub-structure of localpath (rs_localpathPt)
 *         the maximal distance (distance),
 *         the initial and final configuration (q_init, q_max_param),
 *         the value of the parameter in the final configuration (max_param)
 *         the current maximal parameter that could be reach (reach_param)
 *
 * Output: speed of the joints (data),
 *         the distances that the joint couldn't cross
 *                       (distance),
 *         the new maximal parameter that could be reach (reach_param)
 */
static void p3d_jnt_rs_stay_within_dist(
  p3d_stay_within_dist_data * prev_data,
  p3d_rob * robotPt, p3d_rs_data * rs_localpathPt,
  p3d_stay_within_dist_data * data,
  double * distance, configPt q_init,
  configPt q_max_param,
  double max_param, double * reach_param) {
  double wmax_rel = 0, vmax_rel = 0;
  p3d_point p_min, p_max;
  double r, val_rs;
  double dist, range, velocity_max;
  lm_reeds_shepp_str *rs_paramPt    = lm_get_reeds_shepp_lm_param(robotPt);
  p3d_jnt *jntPt, *jnt_xPt, *jnt_yPt, *jnt_zPt;
  int x_coord, y_coord, z_coord, theta_coord;
  int i_dof_x, i_dof_y, i_dof_z;

  jntPt = robotPt->joints[rs_paramPt->numjnt];
  p3d_jnt_get_point(jntPt, &(data->p));

  if (*reach_param < EPS6) {
    data->vmax = prev_data->vmax;
    data->wmax = prev_data->wmax;
  } else {
    x_coord = rs_paramPt->numdof[DOF_RS_X];
    y_coord = rs_paramPt->numdof[DOF_RS_Y];
    z_coord = rs_paramPt->numdof[DOF_RS_Z];
    theta_coord = rs_paramPt->numdof[DOF_RS_THETA];

    jnt_xPt       = p3d_robot_dof_to_jnt(robotPt, x_coord, &i_dof_x);
    jnt_yPt       = p3d_robot_dof_to_jnt(robotPt, y_coord, &i_dof_y);
    jnt_zPt       = NULL;
    if (z_coord >= 0) {
      jnt_zPt   = p3d_robot_dof_to_jnt(robotPt, z_coord, &i_dof_z);
    }

    r = rs_localpathPt->radius;
    val_rs = rs_localpathPt->val_rs;

    if ((rs_localpathPt->type_rs == RIGHT) ||
        (rs_localpathPt->type_rs == LEFT)) {
      wmax_rel = 1.0 / r;
      dist = SQR(max_param);
    } else {
      wmax_rel = 0.;
      dist = SQR(q_max_param[x_coord] - q_init[x_coord]) +
             SQR(q_max_param[y_coord] - q_init[y_coord]);
    }
    if (z_coord >= 0) {
      dist +=  SQR(q_max_param[z_coord] - q_init[z_coord]);
    }
    vmax_rel = sqrt(dist) / max_param;

    /* distance between the reference point of the previous body
       and the point the current joint is attached to
       (this is only an approximation) */
    if (prev_data->wmax < EPS6) { /* We don't need to compute this distance */
      dist = 0.;
    } else {
      p_min.x = data->p.x +
                q_init[x_coord] * jnt_xPt->dof_data[i_dof_x].axis[0] +
                q_init[y_coord] * jnt_yPt->dof_data[i_dof_y].axis[0];
      p_min.y = data->p.y +
                q_init[x_coord] * jnt_xPt->dof_data[i_dof_x].axis[1] +
                q_init[y_coord] * jnt_yPt->dof_data[i_dof_y].axis[1];
      p_min.z = data->p.z +
                q_init[x_coord] * jnt_xPt->dof_data[i_dof_x].axis[1] +
                q_init[y_coord] * jnt_yPt->dof_data[i_dof_y].axis[1];

      p_max.x = data->p.x +
                q_max_param[x_coord] * jnt_xPt->dof_data[i_dof_x].axis[0] +
                q_max_param[y_coord] * jnt_yPt->dof_data[i_dof_y].axis[0];
      p_max.y = data->p.y +
                q_max_param[x_coord] * jnt_xPt->dof_data[i_dof_x].axis[1] +
                q_max_param[y_coord] * jnt_yPt->dof_data[i_dof_y].axis[1];
      p_max.z = data->p.z +
                q_max_param[x_coord] * jnt_xPt->dof_data[i_dof_x].axis[2] +
                q_max_param[y_coord] * jnt_yPt->dof_data[i_dof_y].axis[2];

      if (z_coord >= 0) {
        p_min.x += q_init[z_coord] * jnt_zPt->dof_data[i_dof_z].axis[0];
        p_min.y += q_init[z_coord] * jnt_zPt->dof_data[i_dof_z].axis[1];
        p_min.z += q_init[z_coord] * jnt_zPt->dof_data[i_dof_z].axis[2];

        p_max.x += q_max_param[z_coord] * jnt_zPt->dof_data[i_dof_z].axis[0];
        p_max.y += q_max_param[z_coord] * jnt_zPt->dof_data[i_dof_z].axis[1];
        p_max.z += q_max_param[z_coord] * jnt_zPt->dof_data[i_dof_z].axis[2];
      }
      dist = MAX(p3d_point_dist(prev_data->p, p_min),
                 p3d_point_dist(prev_data->p, p_max));
    }

    data->wmax = prev_data->wmax + wmax_rel;
    data->vmax = prev_data->vmax + dist * (prev_data->wmax) + vmax_rel;

    /* maximal velocity of all the points of the current body */
    velocity_max = data->vmax + (jntPt->dist) * (data->wmax);

    if (velocity_max > EPS6) {
      range = (*distance) / velocity_max;
    } else {
      range = P3D_HUGE;
    }
    if (range < *reach_param) {
      *reach_param = range;
    }
    *distance -= velocity_max * (*reach_param);
  }
}


/*
 *  This function computes an interval of parameter which all the points
 *  of the joint move by less than the distance given as input for a trailer.
 *
 *  Input: speed of the previous joint (prev_data),
 *         the robotPt (robotPt),
 *         the sub-structure of localpath (rs_localpathPt)
 *         the maximal distances (distance_base, distance_trailer),
 *         the initial and final configuration (q_init, q_max_param),
 *         the value of the parameter in the final configuration (max_param)
 *         the current maximal parameter that could be reach (reach_param)
 *
 * Output: speed of the joints (base_data, trailer_data),
 *         the distances that the joint couldn't cross
 *                       (distance_base, distance_trailer),
 *         the new maximal parameter that could be reach (reach_param)
 */
static void p3d_jnt_rs_trailer_stay_within_dist(
  p3d_stay_within_dist_data * prev_data,
  p3d_rob * robotPt, p3d_rs_data * rs_localpathPt,
  p3d_stay_within_dist_data * base_data,
  double * distance_base,
  p3d_stay_within_dist_data *trailer_data,
  double * distance_trailer, configPt q_init,
  configPt q_max_param,
  double max_param, double * reach_param) {
  double wmax_base_rel = 0, vmax_base_rel = 0;
  double wmax_trailer_rel = 0, vmax_trailer_rel = 0;
  double r, val_rs;
  p3d_point p_min, p_max;
  double dist_base, dist_trailer, length;
  double velocity_max_base, velocity_max_trailer;
  lm_reeds_shepp_str *rs_paramPt    = lm_get_reeds_shepp_lm_param(robotPt);
  pflat_trailer_str trailer_paramPt = lm_get_trailer_lm_param(robotPt);
  p3d_jnt *jntPt, *jnt_xPt, *jnt_yPt, *jnt_zPt, *jnt_trailerPt;
  int x_coord, y_coord, z_coord, theta_coord, phi_coord, dc_ds_coord;
  int i_dof_x, i_dof_y, i_dof_z;

  jntPt = robotPt->joints[trailer_paramPt->numjnt[JNT_BASE]];
  jnt_trailerPt = robotPt->joints[trailer_paramPt->numjnt[JNT_TRAILER]];
  p3d_jnt_get_point(jntPt, &(base_data->p));
  p3d_jnt_get_point(jnt_trailerPt, &(trailer_data->p));
  length = trailer_paramPt->flat_str->distAxleToAxis.l2;

  if (*reach_param < EPS6) {
    base_data->vmax = prev_data->vmax;
    base_data->wmax = prev_data->wmax;
    trailer_data->vmax = prev_data->vmax;
    trailer_data->wmax = prev_data->wmax;
  } else {
    x_coord = trailer_paramPt->numdof[DOF_X];
    y_coord = trailer_paramPt->numdof[DOF_Y];
    z_coord = rs_paramPt->numdof[DOF_RS_Z];
    theta_coord = trailer_paramPt->numdof[DOF_THETA];
    phi_coord = trailer_paramPt->numdof[DOF_PHI];
    dc_ds_coord = trailer_paramPt->numdof[DOF_DC_DS];

    jnt_xPt       = p3d_robot_dof_to_jnt(robotPt, x_coord, &i_dof_x);
    jnt_yPt       = p3d_robot_dof_to_jnt(robotPt, y_coord, &i_dof_y);
    jnt_zPt = NULL;
    if (z_coord >= 0) {
      jnt_zPt   = p3d_robot_dof_to_jnt(robotPt, z_coord, &i_dof_z);
    }

    r = rs_localpathPt->radius;
    val_rs = rs_localpathPt->val_rs;

    if ((rs_localpathPt->type_rs == RIGHT) ||
        (rs_localpathPt->type_rs == LEFT)) {
      wmax_base_rel = 1.0 / r;
      dist_base = SQR(max_param / r) * (SQR(r) + SQR(length));
    } else {
      wmax_base_rel = 0.;
      dist_base = SQR(q_max_param[x_coord] - q_init[x_coord]) +
                  SQR(q_max_param[y_coord] - q_init[y_coord]);
    }
    if (z_coord >= 0) {
      dist_base +=  SQR(q_max_param[z_coord] - q_init[z_coord]);
    }
    vmax_base_rel = sqrt(dist_base) / max_param;
    wmax_trailer_rel = 0.;
    vmax_trailer_rel = 0.;

    /* distance between the reference point of the previous body
       and the point the current joint is attached to
       (this is only an approximation) */
    if (prev_data->wmax < EPS6) { /* We don't need to compute this distance */
      dist_base = 0.;
    } else {
      p_min.x = base_data->p.x +
                q_init[x_coord] * jnt_xPt->dof_data[i_dof_x].axis[0] +
                q_init[y_coord] * jnt_yPt->dof_data[i_dof_y].axis[0];
      p_min.y = base_data->p.y +
                q_init[x_coord] * jnt_xPt->dof_data[i_dof_x].axis[1] +
                q_init[y_coord] * jnt_yPt->dof_data[i_dof_y].axis[1];
      p_min.z = base_data->p.z +
                q_init[x_coord] * jnt_xPt->dof_data[i_dof_x].axis[1] +
                q_init[y_coord] * jnt_yPt->dof_data[i_dof_y].axis[1];

      p_max.x = base_data->p.x +
                q_max_param[x_coord] * jnt_xPt->dof_data[i_dof_x].axis[0] +
                q_max_param[y_coord] * jnt_yPt->dof_data[i_dof_y].axis[0];
      p_max.y = base_data->p.y +
                q_max_param[x_coord] * jnt_xPt->dof_data[i_dof_x].axis[1] +
                q_max_param[y_coord] * jnt_yPt->dof_data[i_dof_y].axis[1];
      p_max.z = base_data->p.z +
                q_max_param[x_coord] * jnt_xPt->dof_data[i_dof_x].axis[2] +
                q_max_param[y_coord] * jnt_yPt->dof_data[i_dof_y].axis[2];

      if (z_coord >= 0) {
        p_min.x += q_init[z_coord] * jnt_zPt->dof_data[i_dof_z].axis[0];
        p_min.y += q_init[z_coord] * jnt_zPt->dof_data[i_dof_z].axis[1];
        p_min.z += q_init[z_coord] * jnt_zPt->dof_data[i_dof_z].axis[2];

        p_max.x += q_max_param[z_coord] * jnt_zPt->dof_data[i_dof_z].axis[0];
        p_max.y += q_max_param[z_coord] * jnt_zPt->dof_data[i_dof_z].axis[1];
        p_max.z += q_max_param[z_coord] * jnt_zPt->dof_data[i_dof_z].axis[2];
      }
      dist_base = MAX(p3d_point_dist(prev_data->p, p_min),
                      p3d_point_dist(prev_data->p, p_max));
    }
    dist_trailer = p3d_point_dist(base_data->p, trailer_data->p);

    base_data->wmax = prev_data->wmax + wmax_base_rel;
    base_data->vmax = prev_data->vmax + dist_base * (prev_data->wmax)
                      + vmax_base_rel;

    /* maximal velocity of all the points of the current body */
    velocity_max_base = base_data->vmax + (jntPt->dist) * (base_data->wmax);

    if (velocity_max_base > EPS6) {
      *reach_param = MIN(*reach_param, (*distance_base) / velocity_max_base);
    }

    trailer_data->wmax = base_data->wmax + wmax_trailer_rel;
    trailer_data->vmax = base_data->vmax + dist_trailer * (base_data->wmax) +
                         vmax_trailer_rel;

    /* maximal velocity of all the points of the current body */
    velocity_max_trailer = trailer_data->vmax +
                           (jnt_trailerPt->dist) * (trailer_data->wmax);

    if (velocity_max_trailer > EPS6) {
      *reach_param = MIN(*reach_param,
                         (*distance_trailer) / velocity_max_trailer);
    }
    *distance_base -= velocity_max_base * (*reach_param);
    *distance_trailer -= velocity_max_trailer * (*reach_param);
  }
}


/*
 *  Input:  the robot,
 *          the local path,
 *          the parameter along the curve,
 *          the direction of motion,
 *          the array of maximal distances moved by all the points of
 *          each body of the robot .
 *
 *  Output: distance the robot can move forward along the path without
 *          moving by more than the input distance
 *
 *  Description:
 *          From a configuration on a local path, this function
 *          computes an interval of parameter on the local path on
 *          which all the points of the robot move by less than the
 *          distance given as input.  The interval starts at the
 *          configuration given as input. The function returns the
 *          length of the interval.
 *          Sketch of algorithm: the function computes sucessively
 *          for the plateform and then for each body an upper
 *          bound of the linear and angular velocities at the reference
 *          point. For each body, the maximal range of parameter along
 *          the curve is given by the minimal distance of the body to
 *          the obstacles (stored in array distances) divided by the upper
 *          bound of the velocity of all its points. The minimal value of
 *          these parameter ranges is returned.
 *
 *
 *  CAUTION:   here parameter is the length along RS curve and not
 *           distance along RS+arm path. As a consequence, val_rs
 *           must have non-zero length for all RS segments.
 *             Before this function, we need a p3d_set_and_update_config
 *           for the configuration at u_lp. This allow to take into
 *           account a part of the constraints.
 */

double p3d_rs_stay_within_dist(struct rob* robotPt,
                               struct localpath* localpathPt,
                               double parameter, whichway dir,
                               double* distances) {
  int base_joint = 0;
  int trailer_joint = 1;

  double max_param, min_param, range_param, tot_max_range;
  p3d_rs_data *rs_localpathPt = NULL;
  /* array of maximal linear and angular velocities of each body wrt
     the body it is attached to */
  int i, j, k;
  p3d_jnt *cur_jntPt, *prev_jntPt;
  configPt q_max_param, q_param;
  p3d_stay_within_dist_data * stay_within_dist_data;
  lm_reeds_shepp_str *rs_paramPt    = lm_get_reeds_shepp_lm_param(robotPt);
  pflat_trailer_str trailer_paramPt = lm_get_trailer_lm_param(robotPt);
  int njnt = robotPt->njoints;
  int *other_jnt, nb_other_jnt;
  int x_coord, y_coord, z_coord;

  /* local path has to be of type Reeds and Shepp */
  if (localpathPt->type_lp != REEDS_SHEPP) {
    PrintError(("p3d_rs_stay_within_dist: Reeds and Shepp local path expected\n"));
    return 0;
  }

  rs_localpathPt = localpathPt->specific.rs_data;
// //Moky Debug
//   print_config(robotPt,rs_localpathPt->q_init);
//   p3d_set_and_update_this_robot_conf_with_partial_reshoot(robotPt, rs_localpathPt->q_init);
//   g3d_draw_allwin();
// //   sleep(1);
//   print_config(robotPt,rs_localpathPt->q_end);
//   p3d_set_and_update_this_robot_conf_with_partial_reshoot(robotPt, rs_localpathPt->q_end);
//   g3d_draw_allwin();
// //   sleep(1);
// //Moky Debug
  /* store the data to compute the maximal velocities at the
     joint for each body of the robot */
  stay_within_dist_data = MY_ALLOC(p3d_stay_within_dist_data, njnt + 2);
  p3d_init_stay_within_dist_data(stay_within_dist_data);

  if (trailer_paramPt != NULL) {
    other_jnt = trailer_paramPt->other_jnt;
    nb_other_jnt = trailer_paramPt->nb_other_jnt;
    base_joint = trailer_paramPt->numjnt[JNT_BASE];
    trailer_joint = trailer_paramPt->numjnt[JNT_TRAILER];
    x_coord = trailer_paramPt->numdof[DOF_X];
    y_coord = trailer_paramPt->numdof[DOF_Y];
  } else {
    other_jnt = rs_paramPt->other_jnt;
    nb_other_jnt = rs_paramPt->nb_other_jnt;
    base_joint = rs_paramPt->numjnt;
    x_coord = rs_paramPt->numdof[DOF_RS_X];
    y_coord = rs_paramPt->numdof[DOF_RS_Y];
  }
  z_coord = rs_paramPt->numdof[DOF_RS_Z];

  /* Get the current config to have the modifications of the constraints */
  q_param = p3d_get_robot_config(robotPt);

  /* get the RS segment on which the input configuration is */
  rs_localpathPt = rs_segment_at_parameter(robotPt, rs_localpathPt,
                   &parameter);
  range_param = rs_localpathPt->val_rs;
  tot_max_range = 0.;
  /* range_max is the maximal range possible whitout reaching bounds
     of local path. Notive that ranges are always positive */
  if (dir == FORWARD) {
    min_param = max_param = range_param - parameter;
    q_max_param = rs_localpathPt->q_end;
  } else {
    min_param = max_param = parameter;
    q_max_param = rs_localpathPt->q_init;
  }

  /* loop on the Reeds and Shepp segments */
  while ((min_param >= 0) && (rs_localpathPt != NULL)) {
    k = 0;
    for (i = 0; i <= njnt; i++) {
      cur_jntPt = robotPt->joints[i];
      prev_jntPt = cur_jntPt->prev_jnt;

      if (i == base_joint) {
        /* We could compute the stay_within_dist for the Reed & Shepp method */

        /* j = index of the joint to which the current joint is attached */
        prev_jntPt = cur_jntPt;
        j = -2;
        do {
          if (prev_jntPt == NULL) {
            j = -1;
          } else {
            if ((prev_jntPt->index_dof <= x_coord) &&
                (prev_jntPt->index_dof <= y_coord) &&
                ((prev_jntPt->index_dof <= z_coord) || (z_coord < 0))) {
              if (prev_jntPt->prev_jnt == NULL) {
                j = -1;
              } else {
                j = prev_jntPt->prev_jnt->num;
              }
            } else {
              prev_jntPt = prev_jntPt->prev_jnt;
            }
          }
        } while (j == -2);
        if (trailer_paramPt != NULL) {
          p3d_jnt_rs_trailer_stay_within_dist(&(stay_within_dist_data[j+1]),
                                              robotPt, rs_localpathPt,
                                              &(stay_within_dist_data[base_joint+1]),
                                              &(distances[base_joint]),
                                              &(stay_within_dist_data[trailer_joint+1]),
                                              &(distances[trailer_joint]),
                                              q_param, q_max_param, max_param, &min_param);
        } else {
          p3d_jnt_rs_stay_within_dist(&(stay_within_dist_data[j+1]),
                                      robotPt, rs_localpathPt,
                                      &(stay_within_dist_data[base_joint+1]),
                                      &(distances[base_joint]),
                                      q_param, q_max_param, max_param, &min_param);
        }
        /* All the joints which compose the trailer_joint have the same
           stay_within_dist (this majoration is too strong, but normaly,
           we couldn't have other link than base_joint, so we don't care) */
        while ((cur_jntPt != prev_jntPt) && (cur_jntPt->prev_jnt != NULL)) {
          cur_jntPt = cur_jntPt->prev_jnt;
          stay_within_dist_data[cur_jntPt->num+1] =
            stay_within_dist_data[base_joint+1];
        }
      } else  if ((k < nb_other_jnt) && (i == other_jnt[k])) {
        k ++;

        /* j = index of the joint to which the current joint is attached */
        if (prev_jntPt == NULL) {
          j = -1;  /* environment */
        } else {
          j = prev_jntPt->num;
        }

        p3d_jnt_stay_within_dist(&(stay_within_dist_data[j+1]), cur_jntPt,
                                 &(stay_within_dist_data[i+1]), &(distances[i]),
                                 q_param, q_max_param, max_param, &min_param);
        /* Rem: stay_within_dist_data[0] is bound to the environment */
      }
    }

    tot_max_range += min_param;
    if ((dir == FORWARD) && (parameter + min_param > range_param - EPS6)) {
      /* Test du chemin local suivant */
      rs_localpathPt = rs_localpathPt->next_rs;
      if (rs_localpathPt != NULL) {
        range_param = rs_localpathPt->val_rs;
        min_param = max_param = range_param;
        parameter = 0.;
        p3d_copy_config_into(robotPt, rs_localpathPt->q_init, &q_param);
        q_max_param = rs_localpathPt->q_end;
      } else {
        max_param = -1.;
      }
    } else if ((dir == BACKWARD) && (parameter - min_param < EPS6)) {
      /* Test du chemin local suivant */
      rs_localpathPt = rs_localpathPt->prev_rs;
      if (rs_localpathPt != NULL) {
        range_param = rs_localpathPt->val_rs;
        min_param = max_param = range_param;
        parameter = range_param;
        p3d_copy_config_into(robotPt, rs_localpathPt->q_end, &q_param);
        q_max_param = rs_localpathPt->q_init;
      } else {
        min_param = -1.;
      }
    } else {
      min_param = -1.;
    }
  }
  MY_FREE(stay_within_dist_data, p3d_stay_within_dist_data, njnt + 2);
  p3d_destroy_config(robotPt, q_param);

  return tot_max_range;
}


/*
 *  Cost of a local path
 *
 *  Input:  the local path
 *
 *  Output: the cost
 */
double p3d_rs_cost(p3d_rob *robotPt, p3d_localpath *localpathPt) {
  return localpathPt->length_lp;
}

/*
 *  Simplify Reeds and Shepp trajectories
 *
 *  Input:  the robot,
 *          two RS local path,
 *
 *  Output: the RS local path modified
 *
 *  Description: After optimization, trajectories composed of Reeds and
 *          Shepp local paths can contain parts where the robot goes
 *          forward and then backward along the same path. This function
 *          removes such parts on the localpath given as input and
 *          the next one.
 *            - If the input local path and the next one have
 *              empty intersection, the output is the same local path
 *              as the input.
 *            - If the intersection is not empty, the position of the
 *              remaining joints, if any, is the average value between
 *              the positions at the end of each trucated local path.
 *              As a result, the path returned may be in collision.
 */

p3d_localpath *p3d_simplify_rs(p3d_rob *robotPt, p3d_localpath *localpathPt,
                               int *need_colcheck) {
  int simplify = FALSE, config_different = FALSE;
  p3d_rs_data *rs_segment1Pt, *rs_segment2Pt;
  double common_param = 0, l1, l2, val1, val2;
  configPt q1 = NULL, q2 = NULL, q = NULL;
  int i, j, k;
  p3d_localpath *new_localpath1Pt, *new_localpath2Pt;
  /* the second localpath */
  p3d_localpath *localpath2Pt = localpathPt->next_lp;
  lm_reeds_shepp_str *rs_paramPt = lm_get_reeds_shepp_lm_param(robotPt);
  pflat_trailer_str trailer_paramPt = lm_get_trailer_lm_param(robotPt);
  int *other_jnt, nbotherjnt, z_coord;
  p3d_jnt * jntPt;

  z_coord = rs_paramPt->numdof[DOF_RS_Z];
  if (trailer_paramPt) {
    other_jnt = trailer_paramPt->other_jnt;
    nbotherjnt = trailer_paramPt->nb_other_jnt;
  } else {
    other_jnt = rs_paramPt->other_jnt;
    nbotherjnt = rs_paramPt->nb_other_jnt;;
  }

  if (localpathPt->type_lp != REEDS_SHEPP) {
    PrintError(("p3d_simplify_rs: Reeds and Shepp local path expected\n"));
    return localpathPt;
  }
  if (localpath2Pt == NULL) {
    return localpathPt;
  }
  if (localpath2Pt->type_lp != REEDS_SHEPP) {
    return localpathPt;
  }

  /* look for the last segment of localpathPt */
  rs_segment1Pt = localpathPt->specific.rs_data;

  while (rs_segment1Pt->next_rs != NULL) {
    rs_segment1Pt = rs_segment1Pt->next_rs;
  }
  rs_segment2Pt = localpath2Pt->specific.rs_data;

  /* is there something to  simplify ? */
  simplify = (rs_segment2Pt->type_rs == rs_segment1Pt->type_rs) &&
             (rs_segment2Pt->dir_rs != rs_segment1Pt->dir_rs) &&
             (fabs(rs_segment2Pt->radius - rs_segment1Pt->radius) < EPS6);

  while (simplify) {
    /* which segment is longer ? */
    if (fabs(rs_segment1Pt->val_rs - rs_segment2Pt->val_rs) < EPS6) {
      /* segments have same length */
      common_param += rs_segment1Pt->val_rs;
      rs_segment1Pt = rs_segment1Pt->prev_rs;
      rs_segment2Pt = rs_segment2Pt->next_rs;

      if ((rs_segment1Pt == NULL) || (rs_segment2Pt == NULL)) {
        simplify = FALSE;
      } else {
        simplify = ((rs_segment2Pt->type_rs == rs_segment1Pt->type_rs) &&
                    (rs_segment2Pt->dir_rs != rs_segment1Pt->dir_rs));
      }
    } else if (rs_segment1Pt->val_rs < rs_segment2Pt->val_rs) {
      /*rs_segment 2 is longer */
      common_param += rs_segment1Pt->val_rs;
      simplify = FALSE;
    } else {
      /*rs_segment 1 is longer */
      common_param += rs_segment2Pt->val_rs;
      simplify = FALSE;
    }
  }

  if (common_param < EPS6) {
    return localpathPt;
  }

  /* replace common part */
  l1 = localpathPt->length_lp;
  l2 = localpath2Pt->length_lp;

  val1 = localpathPt->range_param;
  val2 = localpath2Pt->range_param;

  if (fabs(val1 - common_param) > EPS6) {
    new_localpath1Pt =
      localpathPt->extract_sub_localpath(robotPt, localpathPt, 0,
                                         (val1 - common_param) * l1 / val1);
  } else {
    new_localpath1Pt = NULL;
  }

  if (fabs(val2 - common_param) > EPS6) {
    new_localpath2Pt =
      localpath2Pt->extract_sub_localpath(robotPt, localpath2Pt,
                                          common_param * l2 / val2, l2);
  } else {
    new_localpath2Pt = NULL;
  }

  /* attach new local paths together */
  new_localpath1Pt = concat_liste_localpath(new_localpath1Pt,
                     new_localpath2Pt);

  if (nbotherjnt == 0) {
    return new_localpath1Pt;
  }

  /* Now we have to take care of the other joints */
  q1 = localpathPt->config_at_param(robotPt, localpathPt,
                                    val1 - common_param);
  q2 = localpath2Pt->config_at_param(robotPt, localpath2Pt,
                                     common_param);

  q = p3d_copy_config(robotPt, q1);

  /* cas de z */
  if (z_coord >= 0) {
    q[z_coord] = (q1[z_coord] + q2[z_coord]) / 2;
    if (fabs(q1[z_coord] - q2[z_coord]) > EPS6) {
      config_different = TRUE;
    }
  }

  for (i = 0; i < nbotherjnt; i++) {
    jntPt = robotPt->joints[other_jnt[i]];
    for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
      k = jntPt->index_dof + j;
      q[k] = p3d_jnt_calc_dof_value(jntPt, j, q1, q2, 0.5);
      if (p3d_jnt_is_dof_circular(jntPt, j)) {
        if (dist_circle(q1[k], q2[k]) > EPS6) {
          config_different = TRUE;
        }
      } else {
        if (fabs(q1[k] - q2[k]) > EPS6) {
          config_different = TRUE;
        }
      }
    }
  }

  p3d_destroy_config(robotPt, q1);
  p3d_destroy_config(robotPt, q2);

  /* if the position of the other joints are the same at end of
     the first new local path and at the beginning of the second
     new local path, return these local paths */

  if (!config_different) {
    return new_localpath1Pt;
  }

  /* otherwise the local planner has to be called between the beginning of
     localpathPt q1, the end of the common portion q and between q and
     the end of localpath2Pt q2 */
  q1 = localpathPt->config_at_param(robotPt, new_localpath1Pt, 0);
  q2 = localpath2Pt->config_at_param(robotPt, localpath2Pt, val2);
  /* destroy the extracted local paths */
	if(new_localpath1Pt){
		new_localpath1Pt->destroy(robotPt, new_localpath1Pt);
	}
	if(new_localpath2Pt){
  	new_localpath2Pt->destroy(robotPt, new_localpath2Pt);
	}
  /* call local planner between new configurations */
  new_localpath1Pt = p3d_rsarm_localplanner(robotPt, q1, q, localpathPt->ikSol);
  new_localpath2Pt = p3d_rsarm_localplanner(robotPt, q, q2, localpathPt->ikSol);

  new_localpath1Pt->next_lp = new_localpath2Pt;
  new_localpath2Pt->prev_lp = new_localpath1Pt;

  p3d_destroy_config(robotPt, q1);
  p3d_destroy_config(robotPt, q2);
  p3d_destroy_config(robotPt, q);

  *need_colcheck = TRUE;

  return new_localpath1Pt;
}


/*
 *  p3d_write_rs --
 *
 *  write a localpath of type Reeds and Shepp in a file
 *
 *  ARGS IN  : a file descriptor,
 *             a robot,
 *             a localpath
 *
 *  ARGS OUT : TRUE if success,
 *             FALSE if input local path is not a Reeds and Shepp one.
 */

int p3d_write_rs(FILE *file, p3d_rob* robotPt, p3d_localpath* localpathPt) {
  p3d_rs_data *rs_dataPt = NULL;

  if (localpathPt->type_lp != REEDS_SHEPP) {
    return FALSE;
  }

  fprintf(file, "\n\np3d_add_localpath REEDS_SHEPP\n");

  rs_dataPt = (pp3d_rs_data)localpathPt->specific.rs_data;
  /* write each RS segment */
  while (rs_dataPt != NULL) {
    fprintf(file, "conf_init");
    fprint_config_one_line(file, robotPt, rs_dataPt->q_init);
    fprintf(file, "\n");
    fprintf(file, "conf_end ");
    fprint_config_one_line(file, robotPt, rs_dataPt->q_end);
    fprintf(file, "\n");
    fprintf(file, "centre_x \t%f\n", rs_dataPt->centre_x);
    fprintf(file, "centre_y \t%f\n", rs_dataPt->centre_y);
    fprintf(file, "radius \t%f\n", rs_dataPt->radius);
    fprintf(file, "dir_rs \t%d\n", rs_dataPt->dir_rs);
    fprintf(file, "val_rs \t%f\n", rs_dataPt->val_rs);
    fprintf(file, "type_rs \t%d\n", rs_dataPt->type_rs);
    fprintf(file, "\n");
    rs_dataPt = rs_dataPt->next_rs;
  }
  fprintf(file, "\np3d_end_local_path\n");

  return TRUE;
}



/*
 *  Debugging functions
 */

void print_RS(p3d_rob *robotPt, p3d_localpath *localpathPt) {
  int i, j;

  p3d_rs_data *rs_specificPt = NULL;

  if (localpathPt == NULL) {
    PrintInfo(("Local path null\n"));
    return;
  }

  /* local path has to be of type Reeds and Shepp */
  if (localpathPt->type_lp != REEDS_SHEPP) {
    PrintInfo(("print_RS: Reeds and Shepp local path expected\n"));
    return;
  }

  i = 1;
  while (localpathPt != NULL) {
    PrintInfo(("\n *********************************\n  Local path number %d\n\n", i));
    rs_specificPt = localpathPt->specific.rs_data;
    j = 1;

    while (rs_specificPt != NULL) {
      PrintInfo(("    RS segment number %d\n", j));
      PrintInfo(("    q_init\n"));
      print_config(robotPt, rs_specificPt->q_init);

      PrintInfo(("      centre_x %g\n", rs_specificPt->centre_x));
      PrintInfo(("      centre_y %g\n", rs_specificPt->centre_y));
      PrintInfo(("      radius %g\n", rs_specificPt->radius));
      PrintInfo(("      dir_rs %d\n", rs_specificPt->dir_rs));
      PrintInfo(("      val_rs %g\n", rs_specificPt->val_rs));
      PrintInfo(("      type_rs %d\n", rs_specificPt->type_rs));
      PrintInfo(("\n"));

      PrintInfo(("    q_end\n"));
      print_config(robotPt, rs_specificPt->q_end);
      PrintInfo(("\n"));

      rs_specificPt = rs_specificPt->next_rs;
      j++;
    }
    localpathPt = localpathPt->next_lp;
    i++;
  }
}


/*
 *  Reeds and Shepp + arm local planner
 *
 *  Input:  the robot,
 *          the initial and end configurations,
 *
 *  Output: the local path
 *
 *  Description: Computes a local path between an initial and end
 *          configurations. The platform follows a Reeds and Shepp
 *          path and the arm a linear path.
 */

p3d_localpath *p3d_rsarm_localplanner(p3d_rob *robotPt, configPt qi,
                                      configPt qf, int* ikSol) {
  configPt q_i, q_f, q_n1, q_n2;
  int i, j, k, nmaill, irs;
  int nb_dof = robotPt->nb_dof;
  double radius, dist = 0, distc = 0, disr_rs = 0, frac1 = 0, frac2 = 0;
  pStconfig c_i, c_f;
  Stcourbe RStraj[6];
  p3d_localpath *localpathPt;
  p3d_jnt * jntPt;
  p3d_rs_data *courbePt = NULL, *rs_partPt = NULL, *deb_courbePt = NULL;
  lm_reeds_shepp_str *rs_paramPt = lm_get_reeds_shepp_lm_param(robotPt);
  pflat_trailer_str trailer_paramPt = lm_get_trailer_lm_param(robotPt);
  int x_coord, y_coord, theta_coord, z_coord, phi_coord;
  double x_dof, y_dof, theta_dof, z_dof = 0., phi_dof = 0;
  int *other_jnt, nbotherjnt;

  if (rs_paramPt == NULL) {
    PrintWarning(("  No Reeds and Shepp local method specified\n"));
    return NULL;
  }

  if (trailer_paramPt) {
    other_jnt = trailer_paramPt->other_jnt;
    nbotherjnt = trailer_paramPt->nb_other_jnt;
    x_coord = trailer_paramPt->numdof[DOF_X];
    y_coord = trailer_paramPt->numdof[DOF_Y];
    theta_coord = trailer_paramPt->numdof[DOF_THETA];
    phi_coord = trailer_paramPt->numdof[DOF_PHI];
  } else {
    other_jnt = rs_paramPt->other_jnt;
    nbotherjnt = rs_paramPt->nb_other_jnt;;
    x_coord = rs_paramPt->numdof[DOF_RS_X];
    y_coord = rs_paramPt->numdof[DOF_RS_Y];
    theta_coord = rs_paramPt->numdof[DOF_RS_THETA];
    phi_coord = -1;
  }

  z_coord = rs_paramPt->numdof[DOF_RS_Z];

  radius = rs_paramPt->radius;

  for (i = 0;i < 5;i++) {
    RStraj[i].type = 0;
  }

  /* verify initial and end configurations exist */
  if (qi == NULL) {
    PrintInfo((("MP: p3d_rsarm_localplanner: no start configuration...\n")));
    p3d_set_search_status(P3D_ILLEGAL_START);
    return(NULL);
  }
  if (qf == NULL) {
    PrintInfo((("MP: p3d_rsarm_localplanner: no goal configuration...\n")));
    p3d_set_search_status(P3D_ILLEGAL_GOAL);
    return(NULL);
  }

  /* copy initial and end configurations */
  q_i = p3d_copy_config(robotPt, qi);
  q_f = p3d_copy_config(robotPt, qf);

  /* freeze dof between the trailer kinematic chain */

  x_dof = qf[x_coord];
  y_dof = qf[y_coord];
  theta_dof = qf[theta_coord];
  if (z_coord >= 0) {
    z_dof = qf[z_coord];
  }
  if (phi_coord >= 0)
    phi_dof = qf[phi_coord];

  j = 0;
  for (i = 0; i <= robotPt->njoints; i++) {
    if ((j < nbotherjnt) &&
        (i == other_jnt[j])) {
      j++;
    } else {
      jntPt = robotPt->joints[i];
      for (k = 0; k < jntPt->dof_equiv_nbr; k++) {
        q_f[k+jntPt->index_dof] = q_i[k+jntPt->index_dof];
      }
    }
  }
  q_f[x_coord] = x_dof;
  q_f[y_coord] = y_dof;
  q_f[theta_coord] = theta_dof;
  if (z_coord >= 0) {
    qf[z_coord] = z_dof;
  }
  if (phi_coord >= 0)
    qf[phi_coord] = phi_dof;


  if (p3d_get_search_verbose()) {
    PrintInfo(("MP: p3d_rsarm_localplanner : "));
    PrintInfo(("qi=("));
    for (i = 0;i < nb_dof;i++) {
      PrintInfo(("%f,", q_i[i]));
    }
    PrintInfo((") ; "));
    PrintInfo(("qf=("));
    for (i = 0;i < nb_dof;i++) {
      PrintInfo(("%f,", q_f[i]));
    }
    PrintInfo((")\n"));
  }

  /* verify that qi != qf */
  if (p3d_equal_config(robotPt, q_i, q_f)) {
    p3d_destroy_config(robotPt, q_i);
    p3d_destroy_config(robotPt, q_f);
    PrintInfo((("MP: p3d_rsarm_localplanner: q_init = q_goal!\n")));
    p3d_set_search_status(P3D_CONFIG_EQUAL);
    return(NULL);
  }

  /* initial and end configurations of the platform */
  c_i = MY_ALLOC(Stconfig, 1);
  c_f = MY_ALLOC(Stconfig, 1);

  /* The Reeds and Shepp local planner works only for turning
     radius=1.  We need to express the configuration with the turning
     radius as unit of length */

  if (trailer_paramPt == NULL) {
    c_i->x = q_i[x_coord] / radius;
    c_i->y = q_i[y_coord] / radius;
    c_i->z = q_i[theta_coord]; /* theta in radian */
    c_i->suiv = NULL;
    c_f->x = q_f[x_coord] / radius;
    c_f->y = q_f[y_coord] / radius;
    c_f->z = q_f[theta_coord]; /* theta in radian */
    c_f->suiv = NULL;
  } else {
    int trailer_coord = trailer_paramPt->numdof[DOF_PHI];
    double theta_front_wheel, theta_car;
    double l = trailer_paramPt->flat_str->distAxleToAxis.l2;
    /* if the robot is a car modeled by a virtual robot-trailer system,
    the center of the car is different */
    theta_front_wheel = q_i[theta_coord];
    theta_car = angle_limit_2PI(q_i[trailer_coord] + theta_front_wheel);
    c_i->x = (q_i[x_coord] - l * cos(theta_car)) / radius;
    c_i->y = (q_i[y_coord] - l * sin(theta_car)) / radius;
    c_i->z = theta_car;

    theta_front_wheel = q_f[theta_coord];
    theta_car = angle_limit_2PI(q_f[trailer_coord] + theta_front_wheel);
    c_f->x = (q_f[x_coord] - l * cos(theta_car)) / radius;
    c_f->y = (q_f[y_coord] - l * sin(theta_car)) / radius;
    c_f->z = theta_car;
  }

  /* Reeds and Shepp curve between initial and end configurations of
     the platform */
  if (sqrt(SQR(c_i->x - c_f->x) + SQR(c_i->y - c_f->y) + SQR(c_i->z - c_f->z)) > EPS6) {//Modif Mokhtar compare with EPS6 to avoid the float error computation.
    /* the initial and final configurations are different */
    dist = RS_curve(c_i, c_f, 1.0, RStraj);

    /*length of the Reeds and Shepp path */
    for (irs = 0;irs < 5;irs++) {
      if (RStraj[irs].type == RIGHT ||
          RStraj[irs].type == LEFT  ||
          RStraj[irs].type == STRAIGHT) {

        /* multiply lengths by radius to come back to
           initial unit of length*/
        RStraj[irs].cd.x *= radius;
        RStraj[irs].cd.y *= radius;
        RStraj[irs].cf.x *= radius;
        RStraj[irs].cf.y *= radius;
        RStraj[irs].centre_x *= radius;
        RStraj[irs].centre_y *= radius;
        RStraj[irs].r = radius;
        RStraj[irs].val *= radius;

        disr_rs = disr_rs + RStraj[irs].val;
      }
    }
    nmaill = 0;

    for (irs = 0; irs < 5; irs++) {
      if (RStraj[irs].type == RIGHT ||
          RStraj[irs].type == LEFT ||
          RStraj[irs].type == STRAIGHT) {

        /* end configurations stored in each RS segment */
        q_n1 = p3d_copy_config(robotPt, qi);
        q_n2 = p3d_copy_config(robotPt, qi);

        nmaill++;

        /* store the Reeds and Shepp segment ends in q_n1 and q_n2 */
        q_n1[x_coord] = RStraj[irs].cd.x;
        q_n2[x_coord] = RStraj[irs].cf.x;
        q_n1[y_coord] = RStraj[irs].cd.y;
        q_n2[y_coord] = RStraj[irs].cf.y;
        q_n1[theta_coord] = RStraj[irs].cd.z; /* theta in radian */
        q_n2[theta_coord] = RStraj[irs].cf.z;  /* theta in radian */
        /* theta between -Pi and Pi */
        q_n1[theta_coord] = angle_limit_PI(q_n1[theta_coord]);
        q_n2[theta_coord] = angle_limit_PI(q_n2[theta_coord]);

        frac1 = distc / disr_rs;
        distc = distc + RStraj[irs].val;
        frac2 = distc / disr_rs;

        if (z_coord >= 0) {
          jntPt = p3d_robot_dof_to_jnt(robotPt, z_coord, &j);
          q_n1[z_coord] = p3d_jnt_calc_dof_value(jntPt, j, q_i, q_f, frac1);
          q_n2[z_coord] = p3d_jnt_calc_dof_value(jntPt, j, q_i, q_f, frac2);
        }
        for (i = 0; i < nbotherjnt; i++) {
          jntPt = robotPt->joints[other_jnt[i]];
          for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
            k = jntPt->index_dof + j;
            q_n1[k] = p3d_jnt_calc_dof_value(jntPt, j, q_i, q_f, frac1);
            q_n2[k] = p3d_jnt_calc_dof_value(jntPt, j, q_i, q_f, frac2);
          }
        }
        if (robotPt->cntrt_manager->cntrts != NULL) {
          p3d_set_and_update_this_robot_conf(robotPt, q_n1);
          p3d_get_robot_config_into(robotPt, &q_n1);
          p3d_set_and_update_this_robot_conf(robotPt, q_n2);
          p3d_get_robot_config_into(robotPt, &q_n2);
        }

        rs_partPt = p3d_alloc_spec_rs_localpath(q_n1, q_n2,
                                                RStraj[irs].centre_x,
                                                RStraj[irs].centre_y,
                                                RStraj[irs].r,
                                                RStraj[irs].sens,
                                                RStraj[irs].val,
                                                RStraj[irs].type,
                                                NULL, NULL);


        if (nmaill == 1) {
          courbePt = rs_partPt;
          deb_courbePt = courbePt;
        } else {
          courbePt->next_rs = rs_partPt;
          rs_partPt->prev_rs = courbePt;
          courbePt = courbePt->next_rs;
        }
      }
    }

    /* put the Reeds and Shepp local path in a localpath structure */
    localpathPt = p3d_rs_data_into_localpath(robotPt, deb_courbePt, TRUE, 0);
    /* store length and range of parameter of local path */
    localpathPt->length_lp = localpathPt->length(robotPt, localpathPt);
    localpathPt->range_param = distc;

    p3d_set_search_status(P3D_SUCCESS);

    p3d_destroy_config(robotPt, q_i);
    p3d_destroy_config(robotPt, q_f);
  } else {
    /* if the platform does not move, build a linear local path */
    localpathPt = p3d_alloc_lin_localpath(robotPt, q_i, q_f,
                                          0, TRUE);
  }
  MY_FREE(c_i, Stconfig, 1);
  MY_FREE(c_f, Stconfig, 1);

  localpathPt->ikSol = ikSol;
  localpathPt->activeCntrts = p3d_getActiveCntrts(robotPt,&(localpathPt->nbActiveCntrts));
  return(localpathPt);
}


/* Debut modif Fabien */

/*--------------------------------------------------------------------------*/
/*! \brief Local planner for Dubins methods.
 *
 *  \param  robotPt: the robot
 *  \param  qi:      the initial configuration
 *  \param  qf:      the final configuration
 *
 *  \return the local path
 *
 *  Computes a local path between an initial and end
 *  configurations. The platform follows a Dubins
 *  path and the arm a linear path. Dubins path use only 4 of the 48
 *  possible Reeds and Shepp path. This method is not reversible.
 *  It is use with oriented graph.
 */

p3d_localpath *p3d_dubins_localplanner(p3d_rob *robotPt, configPt qi,
                                       configPt qf, int* ikSol) {
  configPt q_i, q_f, q_n1, q_n2;
  int i, j, k, nmaill, irs;
  int nb_dof = robotPt->nb_dof;
  int curve_num;
  double radius, dist = 0, distc = 0, disr_rs = 0, frac1 = 0, frac2 = 0;
  pStconfig c_i, c_f;
  Stcourbe RStraj[6];
  p3d_localpath *localpathPt;
  p3d_jnt * jntPt;
  p3d_rs_data *courbePt = NULL, *rs_partPt = NULL, *deb_courbePt = NULL;
  lm_reeds_shepp_str *rs_paramPt = lm_get_reeds_shepp_lm_param(robotPt);
  pflat_trailer_str trailer_paramPt = lm_get_trailer_lm_param(robotPt);
  int x_coord, y_coord, theta_coord, z_coord, phi_coord;
  double x_dof, y_dof, theta_dof, z_dof = 0., phi_dof = 0;
  int *other_jnt, nbotherjnt;

  if (rs_paramPt == NULL) {
    PrintWarning(("  No Reeds and Shepp local method specified\n"));
    return NULL;
  }

  if (trailer_paramPt) {
    other_jnt = trailer_paramPt->other_jnt;
    nbotherjnt = trailer_paramPt->nb_other_jnt;
    x_coord = trailer_paramPt->numdof[DOF_X];
    y_coord = trailer_paramPt->numdof[DOF_Y];
    theta_coord = trailer_paramPt->numdof[DOF_THETA];
    phi_coord = trailer_paramPt->numdof[DOF_PHI];
  } else {
    other_jnt = rs_paramPt->other_jnt;
    nbotherjnt = rs_paramPt->nb_other_jnt;;
    x_coord = rs_paramPt->numdof[DOF_RS_X];
    y_coord = rs_paramPt->numdof[DOF_RS_Y];
    theta_coord = rs_paramPt->numdof[DOF_RS_THETA];
    phi_coord = -1;
  }

  z_coord = rs_paramPt->numdof[DOF_RS_Z];

  radius = rs_paramPt->radius;

  for (i = 0;i < 5;i++) {
    RStraj[i].type = 0;
  }

  /* verify initial and end configurations exist */
  if (qi == NULL) {
    PrintInfo((("MP: p3d_rsarm_localplanner: no start configuration...\n")));
    p3d_set_search_status(P3D_ILLEGAL_START);
    return(NULL);
  }
  if (qf == NULL) {
    PrintInfo((("MP: p3d_rsarm_localplanner: no goal configuration...\n")));
    p3d_set_search_status(P3D_ILLEGAL_GOAL);
    return(NULL);
  }

  /* copy initial and end configurations */
  q_i = p3d_copy_config(robotPt, qi);
  q_f = p3d_copy_config(robotPt, qf);

  /* freeze dof between the trailer kinematic chain */
  x_dof = qf[x_coord];
  y_dof = qf[y_coord];
  theta_dof = qf[theta_coord];
  if (z_coord >= 0) {
    z_dof = qf[z_coord];
  }
  if (phi_coord >= 0)
    phi_dof = qf[phi_coord];

  j = 0;
  for (i = 0; i <= robotPt->njoints; i++) {
    if ((j < nbotherjnt) &&
        (i == other_jnt[j])) {
      j++;
    } else {
      jntPt = robotPt->joints[i];
      for (k = 0; k < jntPt->dof_equiv_nbr; k++) {
        q_f[k+jntPt->index_dof] = q_i[k+jntPt->index_dof];
      }
    }
  }
  q_f[x_coord] = x_dof;
  q_f[y_coord] = y_dof;
  q_f[theta_coord] = theta_dof;
  if (z_coord >= 0) {
    qf[z_coord] = z_dof;
  }
  if (phi_coord >= 0) {
    qf[phi_coord] = phi_dof;
  }


  if (p3d_get_search_verbose()) {
    PrintInfo(("MP: p3d_rsarm_localplanner : "));
    PrintInfo(("qi=("));
    for (i = 0;i < nb_dof;i++) {
      PrintInfo(("%f,", q_i[i]));
    }
    PrintInfo((") ; "));
    PrintInfo(("qf=("));
    for (i = 0;i < nb_dof;i++) {
      PrintInfo(("%f,", q_f[i]));
    }
    PrintInfo((")\n"));
  }

  /* verify that qi != qf */
  if (p3d_equal_config(robotPt, q_i, q_f)) {
    p3d_destroy_config(robotPt, q_i);
    p3d_destroy_config(robotPt, q_f);
    PrintInfo((("MP: p3d_rsarm_localplanner: q_init = q_goal!\n")));
    p3d_set_search_status(P3D_CONFIG_EQUAL);
    return(NULL);
  }

  /* initial and end configurations of the platform */
  c_i = MY_ALLOC(Stconfig, 1);
  c_f = MY_ALLOC(Stconfig, 1);

  /* The Reeds and Shepp local planner works only for turning
     radius=1.  We need to express the configuration with the turning
     radius as unit of length */

  if (trailer_paramPt == NULL) {
    c_i->x = q_i[x_coord] / radius;
    c_i->y = q_i[y_coord] / radius;
    c_i->z = q_i[theta_coord]; /* theta in radian */
    c_i->suiv = NULL;
    c_f->x = q_f[x_coord] / radius;
    c_f->y = q_f[y_coord] / radius;
    c_f->z = q_f[theta_coord]; /* theta in radian */
    c_f->suiv = NULL;
  } else {
    int trailer_coord = trailer_paramPt->numdof[DOF_PHI];
    double theta_front_wheel, theta_car;
    double l = trailer_paramPt->flat_str->distAxleToAxis.l2;
    /* if the robot is a car modeled by a virtual robot-trailer system,
    the center of the car is different */
    theta_front_wheel = q_i[theta_coord];
    theta_car = angle_limit_2PI(q_i[trailer_coord] + theta_front_wheel);
    c_i->x = (q_i[x_coord] - l * cos(theta_car)) / radius;
    c_i->y = (q_i[y_coord] - l * sin(theta_car)) / radius;
    c_i->z = theta_car;

    theta_front_wheel = q_f[theta_coord];
    theta_car = angle_limit_2PI(q_f[trailer_coord] + theta_front_wheel);
    c_f->x = (q_f[x_coord] - l * cos(theta_car)) / radius;
    c_f->y = (q_f[y_coord] - l * sin(theta_car)) / radius;
    c_f->z = theta_car;
  }

  /* Reeds and Shepp curve between initial and end configurations of
     the platform */
  if (sqrt(SQR(c_i->x - c_f->x) + SQR(c_i->y - c_f->y) + SQR(c_i->z - c_f->z)) != 0.) {
    /* the initial and final configurations are different */
    RS_get_curve_info(&curve_num, &x_dof, &y_dof, &theta_dof);
    /* Note, do not use x_dof, y_dof, theta_dof */

    /* Select Dubins method */
    RS_select_curve(RS_DUBINS);
    dist = RS_curve(c_i, c_f, 1.0, RStraj);
    RS_select_curve(curve_num);

    /*length of the Reeds and Shepp path */
    for (irs = 0;irs < 5;irs++) {
      if (RStraj[irs].type == RIGHT ||
          RStraj[irs].type == LEFT  ||
          RStraj[irs].type == STRAIGHT) {

        /* multiply lengths by radius to come back to
           initial unit of length*/
        RStraj[irs].cd.x *= radius;
        RStraj[irs].cd.y *= radius;
        RStraj[irs].cf.x *= radius;
        RStraj[irs].cf.y *= radius;
        RStraj[irs].centre_x *= radius;
        RStraj[irs].centre_y *= radius;
        RStraj[irs].r = radius;
        RStraj[irs].val *= radius;

        disr_rs = disr_rs + RStraj[irs].val;
      }
    }
    nmaill = 0;

    for (irs = 0; irs < 5; irs++) {
      if (RStraj[irs].type == RIGHT ||
          RStraj[irs].type == LEFT ||
          RStraj[irs].type == STRAIGHT) {

        /* end configurations stored in each RS segment */
        q_n1 = p3d_copy_config(robotPt, qi);
        q_n2 = p3d_copy_config(robotPt, qi);

        nmaill++;

        /* store the Reeds and Shepp segment ends in q_n1 and q_n2 */
        q_n1[x_coord] = RStraj[irs].cd.x;
        q_n2[x_coord] = RStraj[irs].cf.x;
        q_n1[y_coord] = RStraj[irs].cd.y;
        q_n2[y_coord] = RStraj[irs].cf.y;
        q_n1[theta_coord] = RStraj[irs].cd.z; /* theta in radian */
        q_n2[theta_coord] = RStraj[irs].cf.z;  /* theta in radian */
        /* theta between -Pi and Pi */
        q_n1[theta_coord] = angle_limit_PI(q_n1[theta_coord]);
        q_n2[theta_coord] = angle_limit_PI(q_n2[theta_coord]);

        frac1 = distc / disr_rs;
        distc = distc + RStraj[irs].val;
        frac2 = distc / disr_rs;

        if (z_coord >= 0) {
          jntPt = p3d_robot_dof_to_jnt(robotPt, z_coord, &j);
          q_n1[z_coord] = p3d_jnt_calc_dof_value(jntPt, j, q_i, q_f, frac1);
          q_n2[z_coord] = p3d_jnt_calc_dof_value(jntPt, j, q_i, q_f, frac2);
        }
        for (i = 0; i < nbotherjnt; i++) {
          jntPt = robotPt->joints[other_jnt[i]];
          for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
            k = jntPt->index_dof + j;
            q_n1[k] = p3d_jnt_calc_dof_value(jntPt, j, q_i, q_f, frac1);
            q_n2[k] = p3d_jnt_calc_dof_value(jntPt, j, q_i, q_f, frac2);
          }
        }
        if (robotPt->cntrt_manager->cntrts != NULL) {
          p3d_set_and_update_this_robot_conf(robotPt, q_n1);
          p3d_get_robot_config_into(robotPt, &q_n1);
          p3d_set_and_update_this_robot_conf(robotPt, q_n2);
          p3d_get_robot_config_into(robotPt, &q_n2);
        }

        rs_partPt = p3d_alloc_spec_rs_localpath(q_n1, q_n2,
                                                RStraj[irs].centre_x,
                                                RStraj[irs].centre_y,
                                                RStraj[irs].r,
                                                RStraj[irs].sens,
                                                RStraj[irs].val,
                                                RStraj[irs].type,
                                                NULL, NULL);


        if (nmaill == 1) {
          courbePt = rs_partPt;
          deb_courbePt = courbePt;
        } else {
          courbePt->next_rs = rs_partPt;
          rs_partPt->prev_rs = courbePt;
          courbePt = courbePt->next_rs;
        }
      }
    }

    /* put the Dubins local path in a localpath structure */
    localpathPt = p3d_rs_data_into_localpath(robotPt, deb_courbePt, TRUE, 0);
    /* store length and range of parameter of local path */
    localpathPt->length_lp = localpathPt->length(robotPt, localpathPt);
    localpathPt->range_param = distc;

    p3d_set_search_status(P3D_SUCCESS);

    p3d_destroy_config(robotPt, q_i);
    p3d_destroy_config(robotPt, q_f);
  } else {
    /* if the platform does not move, build a linear local path */
    localpathPt = p3d_alloc_lin_localpath(robotPt, q_i, q_f,
                                          0, TRUE);
  }
  MY_FREE(c_i, Stconfig, 1);
  MY_FREE(c_f, Stconfig, 1);

  localpathPt->ikSol = ikSol;
  localpathPt->activeCntrts = p3d_getActiveCntrts(robotPt,&(localpathPt->nbActiveCntrts));
  return(localpathPt);
}

/* Fin modif Fabien */



/*
 *  p3d_create_reeds_shepp_local_method --
 *
 *  create data-structure to store parameters of reeds_shepp local method
 *  and precomputed arrays .
 */

int p3d_create_reeds_shepp_local_method_for_robot(p3d_rob *robotPt, double *dtab, int *itab) {
  plm_reeds_shepp_str rs_paramPt =
    lm_get_reeds_shepp_lm_param(robotPt);

  /* test that reeds_shepp local method has not been already initialized
     for this robot */
  if (rs_paramPt != NULL) {
    PrintWarning(("  reeds_shepp already initialized\n"));
    return FALSE;
  }

  robotPt->lpl_type = P3D_RSARM_PLANNER;

  rs_paramPt = lm_create_reeds_shepp(robotPt, dtab, itab);

  if (rs_paramPt != NULL) {
    robotPt->local_method_params =
      lm_append_to_list(robotPt->local_method_params,
                        (void*)rs_paramPt,
                        P3D_RSARM_PLANNER);
  }
  return TRUE;
}

/*
 *  p3d_create_reeds_shepp_local_method --
 *
 *  create data-structure to store parameters of reeds_shepp local method
 *  and precomputed arrays .
 */

int p3d_create_reeds_shepp_local_method(double *dtab, int *itab) {
  p3d_rob *robotPt = (p3d_rob*)p3d_get_desc_curid(P3D_ROBOT);
  plm_reeds_shepp_str rs_paramPt =
    lm_get_reeds_shepp_lm_param(robotPt);

  /* test that reeds_shepp local method has not been already initialized
     for this robot */
  if (rs_paramPt != NULL) {
    PrintWarning(("  reeds_shepp already initialized\n"));
    return FALSE;
  }

  robotPt->lpl_type = P3D_RSARM_PLANNER;

  rs_paramPt = lm_create_reeds_shepp(robotPt, dtab, itab);

  if (rs_paramPt != NULL) {
    robotPt->local_method_params =
      lm_append_to_list(robotPt->local_method_params,
                        (void*)rs_paramPt,
                        P3D_RSARM_PLANNER);
  }
  return TRUE;
}


/*
 * Check the joint that must be freezed between x,y,z,theta
 */
static void unfree_x_y(p3d_jnt *jntPt, int num_theta, int * free_joints) {
  int i;

  if (jntPt->num >= num_theta) {
    return;
  }
  free_joints[jntPt->num] = FALSE;
  for (i = 0; i < jntPt->n_next_jnt; i++) {
    unfree_x_y(jntPt->next_jnt[i], num_theta, free_joints);
  }
}


/*
 *  lm_create_reeds_shepp
 *
 *  create a Reeds and Shepp local method structure
 *  set radius,
 *  set Steering method to Reeds and Shepp
 *
 *  Input: dtab[0] = radius
 *         itab[0, 1, 2] are the degrees of freedom over which
 *         RS works.
 */

lm_reeds_shepp_str *lm_create_reeds_shepp(p3d_rob *robotPt, double *dtab,
    int *itab) {
  lm_reeds_shepp_str* rs_paramPt = NULL;
  double radius = dtab[0];
  int i, j;
  int njnt = robotPt->njoints;
  int dof_x, dof_y, dof_z, dof_theta;
  p3d_jnt *jnt_thetaPt, *jnt_xPt, *jnt_yPt, *jnt_zPt, *jntPt, *jnt_firstPt;
  int *free_joints;

  /* test that radius is positive */
  if (radius <= EPS6) {
    PrintWarning(("lm_create_reeds_shepp: radius negative\n"));
    return NULL;
  }
  /* test that dof exist */
  if ((itab[0] < 0) || (itab[0] > robotPt->njoints)) {
    PrintWarning(("  lm_create_reeds_shepp: indice of joint not valid\n"));
    return NULL;
  }

  /* check the joint */
  jnt_xPt = jnt_yPt = jnt_thetaPt = robotPt->joints[itab[0]];
  dof_theta = dof_x = dof_y = dof_z = -1;
  jnt_zPt = NULL;
  switch (jnt_thetaPt->type) {
    case P3D_ROTATE: /* Check on sevral joints */
      if (!p3d_jnt_get_dof_is_user(jnt_thetaPt, 0)) {
        PrintWarning(("  lm_create_reeds_shepp: theta not controlable\n"));
        return NULL;
      }
      dof_theta = jnt_thetaPt->index_dof;
      /* Check on previous joint */
      jntPt = jnt_thetaPt->prev_jnt;
      while ((jntPt != NULL) && (dof_x < 0)) {
        for (i = jntPt->dof_equiv_nbr - 1; i >= 0; i--) {
          if (p3d_jnt_get_dof_is_user(jntPt, i) &&
              !p3d_jnt_is_dof_angular(jntPt, i)) {
            if (dof_y < 0) {
              dof_y = jntPt->index_dof + i;
              jnt_yPt = jntPt;
            } else if (dof_x < 0) {
              dof_x = jntPt->index_dof + i;
              jnt_xPt = jntPt;
            } else {
              break;
            }
          }
        }
      }
      if (dof_x < 0) {
        PrintWarning(("  lm_create_reeds_shepp: couldn't found x degree of freedom\n"));
        return NULL;
      }
      break;
    case P3D_FREEFLYER:
    case P3D_BASE: /* We use x, y, Rz */
      if (!p3d_jnt_get_dof_is_user(jnt_thetaPt, 5) ||
          !p3d_jnt_get_dof_is_user(jnt_thetaPt, 1) ||
          !p3d_jnt_get_dof_is_user(jnt_thetaPt, 0)) {
        PrintWarning(("  lm_create_reeds_shepp: x, y or Rz not controlable\n"));
        return NULL;
      }
      dof_theta = jnt_thetaPt->index_dof + 5;
      jnt_yPt = jnt_thetaPt;
      dof_y = jnt_thetaPt->index_dof + 1;
      jnt_xPt = jnt_thetaPt;
      dof_x = jnt_thetaPt->index_dof;
      if (p3d_jnt_get_dof_is_user(jnt_thetaPt, 3)) {
        jnt_zPt = jnt_thetaPt;
        dof_z = jnt_thetaPt->index_dof + 2;
      }
      break;
    case P3D_PLAN: /* We use x, y, Rz */
      if (!p3d_jnt_get_dof_is_user(jnt_thetaPt, 2) ||
          !p3d_jnt_get_dof_is_user(jnt_thetaPt, 1) ||
          !p3d_jnt_get_dof_is_user(jnt_thetaPt, 0)) {
        PrintWarning(("  lm_create_reeds_shepp: x, y or Rz not controlable\n"));
        return NULL;
      }
      dof_theta = jnt_thetaPt->index_dof + 2;
      jnt_yPt = jnt_thetaPt;
      dof_y = jnt_thetaPt->index_dof + 1;
      jnt_xPt = jnt_thetaPt;
      dof_x = jnt_thetaPt->index_dof;
      break;
    case P3D_TRANSLATE:
    case P3D_FIXED:
    case P3D_KNEE:
      PrintWarning(("  lm_create_reeds_shepp: theta is not a rotation\n"));
      return NULL;
      break;
  }

  free_joints = MY_ALLOC(int, njnt + 1);
  for (i = 0; i <= njnt; i++) {
    free_joints[i] = TRUE;
  }

  if (dof_z <= 0) {
    i = 1;
  } else {
    i = 0;
  }
  jnt_firstPt = jntPt = jnt_thetaPt;
  do {
    free_joints[jntPt->num] = FALSE;
    if (jntPt == jnt_xPt) {
      i++;
    }
    if (jntPt == jnt_yPt) {
      i++;
    }
    if (jntPt == jnt_zPt) {
      i++;
    }
    jnt_firstPt = jntPt;
    jntPt = jntPt->prev_jnt;
  } while ((jntPt != NULL) &&
           ((jntPt->num >= jnt_xPt->num) || (jntPt->num >= jnt_yPt->num) ||
            ((jntPt->index_dof >= dof_z) && (dof_z > 0))));
  if (i != 3) {
    MY_FREE(free_joints, int, njnt + 1);
    PrintWarning((" lm_create_reeds_shepp: chaine between phi and x or y is not connected\n"));
    return NULL;
  }
  unfree_x_y(jnt_firstPt, jnt_thetaPt->num, free_joints);

  if ((rs_paramPt = MY_ALLOC(lm_reeds_shepp_str, 1)) == NULL) {
    MY_FREE(free_joints, int, njnt + 1);
    PrintWarning(("  lm_create_reeds_shepp: allocation failed\n"));
    return (NULL);
  }

  rs_paramPt->numdof[DOF_RS_X]     = dof_x;
  rs_paramPt->numdof[DOF_RS_Y]     = dof_y;
  rs_paramPt->numdof[DOF_RS_THETA] = dof_theta;
  rs_paramPt->numdof[DOF_RS_Z]     = dof_z;
  rs_paramPt->numjnt               = jnt_thetaPt->num;

  rs_paramPt->nb_other_jnt = 0;
  for (i = 0; i <= njnt; i++) {
    if (free_joints[i]) {
      rs_paramPt->nb_other_jnt ++;
    }
  }
  rs_paramPt->other_jnt = MY_ALLOC(int, rs_paramPt->nb_other_jnt);

  /* store dof that are not used by trailer local method in array
     other_jnt */
  j = 0;
  for (i = 0; i <= njnt; i++) {
    if (free_joints[i]) {
      rs_paramPt->other_jnt[j] = i;
      j ++;
    }
  }

  MY_FREE(free_joints, int, njnt + 1);
  rs_paramPt->radius = radius;
  return(rs_paramPt);
}


/*
 *  lm_get_reeds_shepp_lm_param --
 *
 *  find the first occurence of reeds_shepp local method parameters.
 */

lm_reeds_shepp_str *lm_get_reeds_shepp_lm_param(p3d_rob* robotPt) {
  lm_list_param_str *list_paramPt = robotPt->local_method_params;
  lm_reeds_shepp_str *resultPt = NULL;

  while (list_paramPt) {
    if (list_paramPt->lpl_type != P3D_RSARM_PLANNER) {
      list_paramPt = list_paramPt->next;
    } else {
      resultPt = (plm_reeds_shepp_str)(list_paramPt->lm_param);
      list_paramPt = NULL;
    }
  }
  return resultPt;
}


/*
 *  lm_destroy_reeds_shepp_params --
 *
 *  destroy data-structure specific to reeds_shepp local method parameters
 */

void lm_destroy_reeds_shepp_params(p3d_rob *robotPt,
                                   void *local_method_params) {
  plm_reeds_shepp_str rs_paramPt =
    (plm_reeds_shepp_str)local_method_params;

  if (rs_paramPt != NULL) {
    MY_FREE(rs_paramPt->other_jnt, int, rs_paramPt->nb_other_jnt);
    MY_FREE(rs_paramPt, lm_reeds_shepp_str, 1);
  }
}


/*
 *  p3d_read_reeds_shepp_segment --
 *
 * build a Reeds and Shepp local path and read the data specific this local
 * path in a file.
 *
 *  ARGS IN  : the file descriptor
 *
 * ARGS OUT : a local path or NULL if error */

static
p3d_rs_data *p3d_read_reeds_shepp_segment(p3d_rob *robotPt, FILE *file,
    double version) {
  p3d_rs_data *rs_dataPt = NULL;
  int size_max_line = 0;
  char *pos = NULL, *line = NULL, *name = NULL;
  int size_max_dtab = 0;
  double *dtab = NULL;
  configPt q_init = NULL, q_end = NULL;
  int success = TRUE;
  double centre_x, centre_y, radius, val_rs;
  int dir_rs, type_rs;
  int num_line = 0;


  static int save_line_size = 0;
  static char *save_line = NULL;

  if ((size_max_line = p3d_read_line_next_function(file, &line,
                       size_max_line,
                       &num_line)) == 0) {
    PrintWarning(("line %d: expecting initial configuration or end of local path\n", num_line));
    success = FALSE;
  }
  /* copy line to save it */
  save_line = p3d_copy_line(line, save_line, size_max_line);
  save_line_size = size_max_line;

  pos = line;

  /*
   * check it is not the end of the local path */
  if (success) {
    if (p3d_read_string_name(&pos, &name) != TRUE) {
      PrintWarning(("line %d: expecting initial configuration or end of local path\n", num_line));
      success = FALSE;
    }
  }

  if (success) {
    if (strcmp(name, "p3d_end_local_path") == 0) {
      return NULL;
    }
  }

  /*
   *  look for conf_init
   */
  /* if not end local path, restore line */
  line = p3d_copy_line(save_line, line, size_max_line);
  size_max_line = save_line_size;

  if (success) {
    if ((q_init = p3d_read_word_and_config(robotPt, line,
                                           (char*)"conf_init", version)) == NULL) {
      PrintWarning(("line %d: expecting initial configuration\n", num_line));
      success = FALSE;
    }
  }

  /*
   *  look for conf_end
   */
  /* read a line */
  if ((size_max_line = p3d_read_line_next_function(file, &line,
                       size_max_line,
                       &num_line)) == 0) {
    PrintWarning(("line %d: expecting end configuration\n", num_line));
    success = FALSE;
  }
  pos = line;

  if (success) {
    if ((q_end = p3d_read_word_and_config(robotPt, line,
                                          (char*)"conf_end", version)) == NULL) {
      PrintWarning(("line %d: expecting end configuration\n", num_line));
      success = FALSE;
    }
  }

  /*
   * look for centre_x
   */

  if (success) {
    /* read next line */
    if ((size_max_line = p3d_read_line_next_function(file, &line,
                         size_max_line,
                         &num_line)) == 0) {
      PrintWarning(("line %d: expecting centre_x\n", num_line));
      success = FALSE;
    }
    pos = line;
  }

  if (success) {
    if (p3d_read_word_and_double(line, (char*)"centre_x", &centre_x) != TRUE) {
      PrintWarning(("line %d: expecting centre_x\n", num_line));
      success = FALSE;
    }
  }


  /*
   * look for centre_y
   */

  if (success) {
    /* read next line */
    if ((size_max_line = p3d_read_line_next_function(file, &line,
                         size_max_line,
                         &num_line)) == 0) {
      PrintWarning(("line %d: expecting centre_y\n", num_line));
      success = FALSE;
    }
    pos = line;
  }

  if (success) {
    if (p3d_read_word_and_double(line, (char*)"centre_y", &centre_y) != TRUE) {
      PrintWarning(("line %d: expecting centre_y\n", num_line));
      success = FALSE;
    }
  }

  /*
   * look for radius
   */

  if (success) {
    /* read next line */
    if ((size_max_line = p3d_read_line_next_function(file, &line,
                         size_max_line,
                         &num_line)) == 0) {
      PrintWarning(("line %d: expecting radius\n", num_line));
      success = FALSE;
    }
    pos = line;
  }

  if (success) {
    if (p3d_read_word_and_double(line, (char*)"radius", &radius) != TRUE) {
      PrintWarning(("line %d: expecting radius\n", num_line));
      success = FALSE;
    }
  }


  /*
   * look for dir_rs
   */

  if (success) {
    /* read next line */
    if ((size_max_line = p3d_read_line_next_function(file, &line,
                         size_max_line,
                         &num_line)) == 0) {
      PrintWarning(("line %d: expecting dir_rs\n", num_line));
      success = FALSE;
    }
    pos = line;
  }

  if (success) {
    if (p3d_read_word_and_int(line, (char*)"dir_rs", &dir_rs) != TRUE) {
      PrintWarning(("line %d: expecting dir_rs\n", num_line));
      success = FALSE;
    }
  }


  /*
   * look for val_rs
   */

  if (success) {
    /* read next line */
    if ((size_max_line = p3d_read_line_next_function(file, &line,
                         size_max_line,
                         &num_line)) == 0) {
      PrintWarning(("line %d: expecting val_rs\n", num_line));
      success = FALSE;
    }
    pos = line;
  }

  if (success) {
    if (p3d_read_word_and_double(line, (char*)"val_rs", &val_rs) != TRUE) {
      PrintWarning(("line %d: expecting val_rs\n", num_line));
      success = FALSE;
    }
  }


  /*
   * look for type_rs
   */

  if (success) {
    /* read next line */
    if ((size_max_line = p3d_read_line_next_function(file, &line,
                         size_max_line,
                         &num_line)) == 0) {
      PrintWarning(("line %d: expecting type_rs\n", num_line));
      success = FALSE;
    }
    pos = line;
  }

  if (success) {
    if (p3d_read_word_and_int(line, (char*)"type_rs", &type_rs) != TRUE) {
      PrintWarning(("line %d: expecting type_rs\n", num_line));
      success = FALSE;
    }
  }


  if (success) {
    rs_dataPt = p3d_alloc_spec_rs_localpath(q_init, q_end,
                                            centre_x, centre_y,
                                            radius, dir_rs, val_rs,
                                            type_rs, NULL, NULL);
  }

  else {
    /* error while readind local path, desallocate and return NULL */
    if (q_init != NULL) {
      p3d_destroy_config(robotPt, q_init);
    }
    if (q_end != NULL) {
      p3d_destroy_config(robotPt, q_end);
    }
  }
  if (size_max_dtab > 0) {
    MY_FREE(dtab, double, size_max_dtab);
    dtab = NULL;
  }

  return rs_dataPt;
}


/*
 *  p3d_read_reeds_shepp_localpath --
 *
 * build a Reeds and Shepp local path and read the data specific this local
 * path in a file.
 *
 *  ARGS IN  : the file descriptor
 *
 * ARGS OUT : a local path or NULL if error */

p3d_localpath *p3d_read_reeds_shepp_localpath(p3d_rob *robotPt, FILE *file,
    double version) {
  p3d_localpath *localpathPt = NULL;
  p3d_rs_data *rs_startPt = NULL, *rs_endPt = NULL;

  do {
    if (rs_startPt == NULL) {
      rs_startPt = p3d_read_reeds_shepp_segment(robotPt, file, version);
      rs_endPt = rs_startPt;
      if (rs_startPt == NULL) {
        PrintWarning(("p3d_read_reeds_shepp_localpath: empty local path\n"));
        return NULL;
      }
    } else {
      rs_endPt->next_rs = p3d_read_reeds_shepp_segment(robotPt, file,
                          version);

      if (rs_endPt->next_rs != NULL) {
        rs_endPt->next_rs->prev_rs = rs_endPt;
      }
      rs_endPt = rs_endPt->next_rs;
    }

  } while (rs_endPt != NULL);

  localpathPt = p3d_rs_data_into_localpath(robotPt, rs_startPt, TRUE, 0);
  localpathPt->activeCntrts = p3d_getActiveCntrts(robotPt,&(localpathPt->nbActiveCntrts));
  return localpathPt;
}
