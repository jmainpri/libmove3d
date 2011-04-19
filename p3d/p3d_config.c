#include "Util-pkg.h"
#include "P3d-pkg.h"

#ifdef P3D_COLLISION_CHECKING
#include "Collision-pkg.h"
#endif

#ifdef P3D_LOCALPATH
#include "Localpath-pkg.h"
#endif

/*! allocate a configuration:
 *
 * Input:  the robot DOF
 *
 * Output: a pointer to a configuration
 *
 * Allocation: the configuration
 */

configPt p3d_alloc_config_n(int nb_dof)
{
  int i;

  configPt confPt = MY_ALLOC(double, nb_dof);
  for (i = 0; i < nb_dof; i++) {
    confPt[i] = 0;
  }
  return confPt;
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Allocate a configuration for one robot
 *
 * \param robotPt: the robot
 *
 * \return The configuration allocated (all values set to 0)
 */
configPt p3d_alloc_config(p3d_rob *robotPt)
{
  int nb_dof = robotPt->nb_dof;
  configPt confPt = p3d_alloc_config_n(nb_dof);
  return confPt;
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Allocate a user configuration for one robot
 *
 * \param robotPt: the robot
 *
 * Note: The user config, is a config where there aren't the degree of
 * freedom not controled by the user (p3d_dof_data::is_user == FALSE).
 *
 * \return The configuration allocated (all values set to 0)
 */
configPt p3d_alloc_user_config(p3d_rob *robotPt)
{
  int nb_dof = robotPt->nb_user_dof;
  configPt confPt = p3d_alloc_config_n(nb_dof);
  return confPt;
}


/*! allocate a configuration for one body only (6 dofs):
 *
 * Input:
 *
 * Output: a pointer to a configuration
 *
 * Allocation: the configuration
 */

configPt p3d_alloc_body_config()
{
  int i;
  configPt confPt = MY_ALLOC(double, NDOF_BASE);

  for (i = 0; i < NDOF_BASE; i++)
    confPt[i] = 0;
  return confPt;
}



/* destroy a configuration for one body only (6 dofs):
 *
 * Input: the configuration
 *
 */

void p3d_destroy_body_config(configPt q)
{
  MY_FREE(q, double, NDOF_BASE);
}

/* Create a config copying the givennumber of DOF from the src:
 *
 *  Input:  the robot, the configuration
 *
 *  Output: a configuration
 *
 *  Allocation: the copied configuration
 */
configPt p3d_copy_config_n(int nb_dof, configPt src)
{
  configPt copy_q = MY_ALLOC(double, (size_t)nb_dof);
  memcpy((void*)copy_q, (void*)src, (size_t)nb_dof*sizeof(double));
  return copy_q;
}

/* Create a copied configuration:
 *
 *  Input:  the robot, the configuration
 *
 *  Output: a configuration with the number of DOFs same as from Robot, with values from src.
 *
 *  Allocation: the copied configuration
 */
configPt p3d_copy_config(p3d_rob * robotPt, configPt src)
{
  int nb_dof = robotPt->nb_dof;
  configPt copy_q = p3d_copy_config_n(nb_dof, src);

  return copy_q;
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Copy a user configuration to a new configuration.
 *
 * \note The user config, is a config where there aren't the degree of
 * freedom not controled by the user (p3d_dof_data::is_user == FALSE).
 * For the degree of freedom which aren't controled by the user, we set
 * their values to the current value of the joint.
 *
 * \param robotPt: the robot
 * \param q_user:  the user config
 *
 * \return The configuration created
 */
configPt p3d_copy_user_config_to_config(p3d_rob *robotPt, configPt q_user)
{
  configPt confPt;

  confPt =  p3d_get_robot_config_deg(robotPt);
  p3d_copy_user_config_into_config(robotPt, q_user, &confPt);
  return confPt;
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Copy a user configuration to a new user configuration.
 *
 * Note: The user config, is a config where there aren't the degree of
 * freedom not controled by the user (p3d_dof_data::is_user == FALSE).
 *
 * \param robotPt: the robot
 * \param q_user:  the user config
 *
 * \return The user configuration created
 */
configPt p3d_copy_user_config_to_user_config(p3d_rob *robotPt, configPt q_user)
{
  int nb_dof = robotPt->nb_user_dof;
  configPt copy_q = p3d_copy_config_n(nb_dof, q_user);

  return copy_q;
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Copy a configuration to a new user configuration.
 *
 * Note: The user config, is a config where there aren't the degree of
 * freedom not controled by the user (p3d_dof_data::is_user == FALSE).
 *
 * \param robotPt: the robot
 * \param q_user:  the config
 *
 * \return The user configuration created
 */
configPt p3d_copy_config_to_user_config(p3d_rob *robotPt, configPt q)
{
  configPt confPt;

  confPt = MY_ALLOC(double, robotPt->nb_user_dof);
  p3d_copy_config_into_user_config(robotPt, q, &confPt);
  return confPt;
}


/*!  Copy a configuration into another configuration
 *
 *  Input:  the robot, the configuration to copy,
 *          a pointer to another configuration.
 *
 *  Allocation: the copied configuration
 */

void p3d_copy_config_n_into(int nb_dof, configPt config1,
                            configPt *config2Pt)
{
  memcpy((void*)*config2Pt, (void*)config1,
         (size_t)(nb_dof*sizeof(double)));
}

void p3d_copy_config_into(p3d_rob *robotPt, configPt config1,
                          configPt *config2Pt)
{
  int nb_dof = robotPt->nb_dof;
  p3d_copy_config_n_into(nb_dof, config1, config2Pt);
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Copy a user configuration into a configuration.
 *
 * \note The user config, is a config where there aren't the degree of
 * freedom not controled by the user (p3d_dof_data::is_user == FALSE).
 * For the degree of freedom which aren't controled by the user, we set
 * their values to the default value in \a config2.
 *
 * \param robotPt: the robot
 * \param user_config1:  the user config
 *
 * \retval config2: the configuration copied
 */
void p3d_copy_user_config_into_config(p3d_rob *robotPt,
                                      configPt user_config1,
                                      configPt * config2)
{
  int i, j, k;
  p3d_jnt * jntPt;

  for (i = 0; i <= robotPt->njoints; i++) {
    jntPt = robotPt->joints[i];
    k = jntPt->index_user_dof;
    for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
      if (p3d_jnt_get_dof_is_user(jntPt, j)) {
        (*config2)[jntPt->index_dof+j] = user_config1[k++];
      }
    }
  }
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Copy a user configuration into a user configuration.
 *
 * Note: The user config, is a config where there aren't the degree of
 * freedom not controled by the user (p3d_dof_data::is_user == FALSE).
 *
 * \param robotPt: the robot
 * \param user_config1:  the user config
 *
 * \retval user_config2: The user configuration copied
 */
void p3d_copy_user_config_into_user_config(p3d_rob *robotPt,
    configPt user_config1,
    configPt * user_config2)
{
  int nb_dof = robotPt->nb_user_dof;
  p3d_copy_config_n_into(nb_dof, user_config1, user_config2);
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Copy a configuration to a new user configuration.
 *
 * Note: The user config, is a config where there aren't the degree of
 * freedom not controled by the user (p3d_dof_data::is_user == FALSE).
 *
 * \param robotPt: the robot
 * \param config1:  the config
 *
 * \retval user_config2: the user configuration copied
 */
void p3d_copy_config_into_user_config(p3d_rob *robotPt,
                                      configPt config1,
                                      configPt * user_config2)
{
  int i, j, k;
  p3d_jnt * jntPt;

  for (i = 0; i <= robotPt->njoints; i++) {
    jntPt = robotPt->joints[i];
    k = jntPt->index_user_dof;
    for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
      if (p3d_jnt_get_dof_is_user(jntPt, j)) {
        (*user_config2)[k++] = config1[jntPt->index_dof+j];
      }
    }
  }
}


/*! Copy a configuration and convert angles from radian to degree.
 *
 *  Input:  the robot, the configuration
 *
 *  Output: a configuration
 *
 *  Allocation: the copied configuration
 */

configPt p3d_copy_config_rad_to_deg(p3d_rob *robotPt, configPt q)
{
  int i, j, k;
  int njnt = robotPt->njoints;
  configPt copy_q = MY_ALLOC(double, (size_t)(robotPt->nb_dof));
  p3d_jnt *jntPt;

  k = 0;
  for (i = 0; i <= njnt; i++) {
    jntPt = robotPt->joints[i];
    for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
      if (p3d_jnt_is_dof_angular(jntPt, j)) {
        copy_q[k] = RTOD(q[k]);
      } else {
        copy_q[k] = q[k];
      }
      k ++;
    }
  }
  return copy_q;
}

/*! Copy a configuration and convert angles from degree to radian.
 *
 *  Input:  the robot, the configuration
 *
 *  Output: a configuration
 *
 *  Allocation: the copied configuration
 */

configPt p3d_copy_config_deg_to_rad(p3d_rob *robotPt, configPt q)
{
  int i, j, k;
  int njnt = robotPt->njoints;
  configPt copy_q = MY_ALLOC(double, (size_t)(robotPt->nb_dof));
  p3d_jnt *jntPt;

  k = 0;
  for (i = 0; i <= njnt; i++) {
    jntPt = robotPt->joints[i];
    for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
      if (p3d_jnt_is_dof_angular(jntPt, j)) {
        copy_q[k] = DTOR(q[k]);
      } else {
        copy_q[k] = q[k];
      }
      k ++;
    }
  }
  return copy_q;
}

/************************************************************************/
/*!
 * \brief configurations add
 *
 * \param rob the robot
 * \param q1 first config
 * \param q2 second config
 * \retval q result = q1 + q2
 *
 * \author EF
*/
/************************************************************************/
void p3d_addConfig(p3d_rob *rob, configPt q1, configPt q2, configPt q)
{
  int i, j, k;
  int njnt = rob->njoints;
  p3d_jnt *jntPt;

  k = 0;
  for (i = 0; i <= njnt; i++) {
    jntPt = rob->joints[i];
    for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
      if (p3d_jnt_is_dof_circular(jntPt, j)) {
        q[k] =  angle_limit_PI(q1[k] + q2[k]);
      } else {
        q[k] =  q1[k] + q2[k];
      }
      k ++;
    }
  }
}

/************************************************************************/
/*!
 * \brief configurations add 2PI
 *
 * \param rob the robot
 * \param q1 first config
 * \param q2 second config
 * \retval q result = q1 + q2
 *
 * \author EF
*/
/************************************************************************/
void p3d_addConfig2PI(p3d_rob *rob, configPt q1, configPt q2, configPt q)
{
  int i, j, k;
  int njnt = rob->njoints;
  p3d_jnt *jntPt;

  k = 0;
  for (i = 0; i <= njnt; i++) {
    jntPt = rob->joints[i];
    for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
      if (p3d_jnt_is_dof_circular(jntPt, j)) {
        q[k] =  angle_limit_2PI(q1[k] + q2[k]);
      } else {
        q[k] =  q1[k] + q2[k];
      }
      k ++;
    }
  }
}

/************************************************************************/
/*!
 * \brief configurations substract
 *
 * \param rob the robot
 * \param q1 first config
 * \param q2 second config
 * \retval dq dq = q2 - q1
 *
 * \author EF
 */
/************************************************************************/
void p3d_subConfig(p3d_rob *rob, configPt q1, configPt q2, configPt dq)
{
  int i, j, k;
  int njnt = rob->njoints;
  p3d_jnt *jntPt;

  k = 0;
  for (i = 0; i <= njnt; i++) {
    jntPt = rob->joints[i];
    for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
      if (p3d_jnt_is_dof_circular(jntPt, j)) {
        dq[k] =  diff_angle(q1[k], q2[k]);
      } else {
        dq[k] =  q2[k] - q1[k];
      }
      k ++;
    }
  }
}

/************************************************************************/
/*!
 * Check if two configs are equal
 *
 */
int
p3d_equal_config_n(int nb_dof, configPt q_i, configPt q_f)
{
  int i;
  for (i = 0;i < nb_dof;i++) {
    if (q_i[i] != q_f[i]) {
      return(FALSE);
    }
  }
  return(TRUE);
}

int p3d_equal_config(p3d_rob *robotPt, configPt q_i, configPt q_f)
{
  int i, j, k;
  int njnt = robotPt->njoints;
  p3d_jnt *jntPt;

  k = 0;
  for (i = 0; i <= njnt; i++) {
    jntPt = robotPt->joints[i];
    for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
      if (p3d_jnt_get_dof_is_user(jntPt, j)) {
        if (p3d_jnt_is_dof_circular(jntPt, j)) {
          if (dist_circle(q_i[k], q_f[k]) > EPS6) {
            return FALSE;
          }
        } else {
          if (fabs(q_i[k] - q_f[k]) > EPS6) {
            return FALSE;
          }
        }
      }
      k ++;
    }
  }
  return TRUE;
}


/*!
 * Check if two configs are equal without considering all the configPt
 *
 */
int p3d_equal_config_n_offset(int nb_dof, int offset, configPt q_i, configPt q_f)
{
  int i;
  for (i = offset;i < offset + nb_dof;i++) {
    if (fabs(q_i[i] - q_f[i]) > EPS6) {
      return(FALSE);
    }
  }
  return(TRUE);
}


/*!
 * Compute the square distance between two configurations using
 * only the active dofs
 *
 * Input:  The robot,
 *         the two configurations
 */
double p3d_ActiveDistConfig(p3d_rob * robotPt, configPt q_i, configPt q_f)
{
  double l = 0., ljnt = 0.;
  int i, j, njnt = robotPt->njoints;
  p3d_jnt * jntPt;

  for (i = 0; i <= njnt; i++) {
    jntPt = robotPt->joints[i];
    for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
      if ((p3d_jnt_get_dof_is_user(jntPt, j)) &&
          (p3d_jnt_get_dof_is_active_for_planner(jntPt, j)))
        ljnt += SQR(p3d_jnt_calc_dof_dist(jntPt, j, q_i, q_f));
      // } else {
      // PrintInfo((" test\n"));
      // }
    }
  }
  l = sqrt(ljnt);

  return l;
}


void p3dCopyPassive(p3d_rob*robotPt, configPt qSource, configPt qGoal)
{
  ;
  int i, j, k, njnt = robotPt->njoints;
  p3d_jnt * jntPt;

  for (i = 0; i <= njnt; i++) {
    jntPt = robotPt->joints[i];
    for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
      k = jntPt->index_dof + j;
      if ((!p3d_jnt_get_dof_is_user(jntPt, j)) ||
          (!p3d_jnt_get_dof_is_active_for_planner(jntPt, j))) {
        qGoal[k] =  qSource[k];
      }
    }
  }
}

/*!
 * Compute the classic square distance between two configurations
 *
 * Input:  The robot,
 *         the two configurations
 */
double p3d_dist_config(p3d_rob * robotPt, configPt q_i, configPt q_f)
{
  double l = 0., ljnt = 0.;
  int i, j, njnt = robotPt->njoints;
  p3d_jnt * jntPt;

  for (i = 0; i <= njnt; i++) {
    jntPt = robotPt->joints[i];
    for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
      if (robotPt->cntrt_manager->in_cntrt[jntPt->index_dof + j] != DOF_PASSIF) {
				double dist = SQR(p3d_jnt_calc_dof_dist(jntPt, j, q_i, q_f));
        if (isnan(dist)) {
          printf("Distance computation error !!!\n");
          return P3D_HUGE;
        }
				//printf(" dist[%d] = %f\n",jntPt->index_dof + j,dist);
        ljnt += dist;
      }
    }
  }
  l = sqrt(ljnt);

  return l;
}

// Same as above without weigthing
// This function computes the distance without consideration of W
double p3d_dist_config_2(p3d_rob * robotPt, configPt q_i, configPt q_f)
{
  double l = 0., ljnt = 0.;
  int i, j, njnt = robotPt->njoints;
  p3d_jnt * jntPt;
	
  for (i = 0; i <= njnt; i++) {
    jntPt = robotPt->joints[i];
    for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
        ljnt += SQR(p3d_jnt_calc_dof_dist_2(jntPt, j, q_i, q_f));
    }
  }
  l = sqrt(ljnt);
	
  return l;
}

/*!
 * Convert a configuration expressed in degree to a configuration expressed
 * in radian
 *
 * Input:  The robot,
 *         the configuration in degree,
 *         a pointer to the configuration in radian
 *
 */
void p3d_convert_config_deg_to_rad(p3d_rob *robotPt, configPt q_deg,
                                   configPt *q_rad)
{
  int i, j, k;
  int njnt = robotPt->njoints;
  p3d_jnt *jntPt;

  k = 0;
  for (i = 0; i <= njnt; i++) {
    jntPt = robotPt->joints[i];
    for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
      if (p3d_jnt_is_dof_angular(jntPt, j)) {
        (*q_rad)[k] = DTOR(q_deg[k]);
      } else {
        (*q_rad)[k] = q_deg[k];
      }
      k ++;
    }
  }
}


/*!
 * Convert a configuration expressed in radian to a configuration expressed
 * in degree
 *
 * Input:  The robot,
 *         the configuration in degree,
 *         a pointer to the configuration in radian
 *
 */

void p3d_convert_config_rad_to_deg(p3d_rob *robotPt, configPt q_rad,
                                   configPt *q_deg)
{
  int i, j, k;
  int njnt = robotPt->njoints;
  p3d_jnt *jntPt;

  k = 0;
  for (i = 0; i <= njnt; i++) {
    jntPt = robotPt->joints[i];
    for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
      if (p3d_jnt_is_dof_angular(jntPt, j)) {
        (*q_deg)[k] = RTOD(q_rad[k]);
      } else {
        (*q_deg)[k] = q_rad[k];
      }
      k ++;
    }
  }
}


/*!
 *  Destroy a configuration
 *
 *  Input:  the robot, the configuration
 *
 *  Allocation: desallocates the configuration
 */

void p3d_destroy_config_n(int nb_dof, configPt cfg)
{
  if (cfg != NULL) {
    MY_FREE(cfg, double, nb_dof);
    cfg = NULL;
  }
}

void p3d_destroy_config(p3d_rob *robotPt, configPt cfg)
{
  int nb_dof = robotPt->nb_dof;

  p3d_destroy_config_n(nb_dof, cfg);
}

void p3d_destroy_user_config(p3d_rob *robotPt, configPt cfg)
{
  int nb_dof = robotPt->nb_user_dof;

  p3d_destroy_config_n(nb_dof, cfg);
}

/*!
 *  distance between points
 */

double p3d_point_dist(p3d_point p1, p3d_point p2)
{
  double dist =
    sqrt(SQR(p2.x - p1.x) + SQR(p2.y - p1.y) + SQR(p2.z - p1.z));

  return dist;
}

/*!
 *  Set a robot in its joints limits
 *
 *  Input:  the robot
 *
 */

void p3d_set_robot_in_joint_limits(p3d_rob *robotPt, configPt q)
{
  int i, j, k;
  double vmin, vmax;
  int njnt = robotPt->njoints;
  p3d_jnt *jntPt;

  k = 0;
  for (i = 0; i <= njnt; i++) {
    jntPt = robotPt->joints[i];
    for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
      p3d_jnt_get_dof_bounds(jntPt, j, &vmin, &vmax);
      if (q[k] < vmin) {
        q[k] = vmin;
      } else if (q[k] > vmax) {
        q[k] = vmax;
      }
      k ++;
    }
  }
}

/*! print a configuration */
void print_config(p3d_rob *robotPt, configPt q)
{
  int i, nb_dof;

  if (robotPt != NULL) {
    nb_dof = robotPt->nb_dof;
  } else {
    nb_dof = 0;
  }

  for (i = 0; i < nb_dof;i++) {
    PrintInfo(("q[%d] = %f\n", i, q[i]));
  }
}

/*! print a configuration on one line */
void print_config_one_line_degrees(p3d_rob *robotPt, configPt q)
{
  if(robotPt==NULL)
  {
    printf("%s: %d: print_config_one_line_degrees(): input p3d_rob* is NULL.\n",__FILE__,__LINE__);
    return;
  }

  int i, nb_dof;
  configPt q_deg;
  
  q_deg= p3d_alloc_config(robotPt);

  p3d_convert_config_rad_to_deg(robotPt, q, &q_deg);

  nb_dof = robotPt->nb_dof;
 
  printf("q= [\n");
  for (i = 0; i < robotPt->nb_dof;i++) {
    printf("  %f", q_deg[i]);
  }
  printf("\n]\n");
  
  p3d_destroy_config(robotPt, q_deg);
}

/*! print a configuration in a file */
void fprint_config_one_line(FILE *file, p3d_rob *robotPt, configPt q)
{
  int i, nb_dof;

  if (robotPt != NULL) {
    nb_dof = robotPt->nb_dof;
  } else {
    nb_dof = 0;
  }

  for (i = 0; i < nb_dof;i++) {
    fprintf(file, "\t %f", q[i]);
  }
}



/************************************************************************/
/*!
 * \brief configurations get middle
 *
 * \param rob the robot
 * \param q1 first config
 * \param q2 second config
 * \retval q result = (q1 + q2)/2
 *
 * \author Mokhtar
*/
/************************************************************************/
void p3d_middleConfig(p3d_rob *rob, configPt q1, configPt q2, configPt q)
{
  int i, j, k;
  int njnt = rob->njoints;
  p3d_jnt *jntPt;

  k = 0;
  for (i = 0; i <= njnt; i++) {
    jntPt = rob->joints[i];
    for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
      if (p3d_jnt_is_dof_circular(jntPt, j)) {
        q[k] =  angle_limit_PI((q1[k] + q2[k]) / 2);
      } else {
        q[k] = (q1[k] + q2[k]) / 2;
      }
      k ++;
    }
  }
}

/**
 * Check if a configuration is null or not (Contains only 0)
 * @param robot The robot
 * @param q The configuration
 * @return True if the configuration is Null, False otherwise.
 */
int p3d_isNullConfig(p3d_rob* robot, configPt q)
{
  for (int i = 0; i < robot->nb_dof; i++) {
    if (q[i] != 0) {
      return FALSE;
    }
  }
  return TRUE;
}

//start path deform
/*  p3d_stay_within_sphere
 *
 *  Input:  the robot,
 *          the maximal distance moved by all the points of the
 *          robot
 *
 *  Output: linear distance the robot can move safely
 *
 *
 *  Description:
 *          This function is very close to p3d_stay_within_dist.
 *          The linear and angular velocities of each body are
 *          bounded in order to get a lower bound on the distance
 *          the robot can move
 */
double p3d_stay_within_sphere(p3d_rob* robotPt, double *distances)
{
  int i, j, njnt = robotPt->njoints;
  p3d_jnt *cur_jntPt, *prev_jntPt;
  p3d_stay_within_dist_data *stay_within_dist_data;
  double dmax, dist0 = 0.,  min_param = P3D_HUGE;

  dmax = p3d_get_env_dmax();
#ifdef P3D_COLLISION_CHECKING
  if (!p3d_col_get_microcollision()) {
    dist0 = dmax;
  }
#endif
  for (j = 0; j <= njnt; j++) {
    distances[j] += dist0;
  }

  stay_within_dist_data = MY_ALLOC(p3d_stay_within_dist_data, njnt + 2);
	
#ifdef P3D_LOCALPATH
  p3d_init_stay_within_dist_data(stay_within_dist_data);
#endif

  /* computation of the bounds for the linear and angular
     velocities of each body */
  for (i = 0; i <= njnt; i++) {
    cur_jntPt = robotPt->joints[i];
    prev_jntPt = cur_jntPt->prev_jnt;

    /* j = index of the joint to which the current joint is attached */
    if (prev_jntPt == NULL) {
      j = -1;  /* environment */
    } else {
      j = prev_jntPt->num;
    }

    p3d_jnt_stay_within_sphere(&(stay_within_dist_data[j+1]), cur_jntPt,
                               &(stay_within_dist_data[i+1]),  &(distances[i]), &min_param);
    /* Rem: All p3d_jnt_stay_within_sphere _[0] are bound to the environment */
  }
  MY_FREE(stay_within_dist_data, p3d_stay_within_dist_data, njnt + 2);
  return min_param;
}
//end path deform

/*  p3d_is_collision_free
 *
 *  Input:  the robot,
 *          the configuration
 *
 *  Output: TRUE if the configuration is collision free
 *
 *
 *  Description:
 *          This function returns true if the confuration
 *					is collision free with the side effect of leaving the robot
 *					in the configuration passed as argument
 */
int p3d_is_collision_free(p3d_rob* robotPt, configPt q)
{
#ifdef P3D_COLLISION_CHECKING
	
	if(p3d_set_and_update_this_robot_conf(robotPt, q) && !p3d_col_test())
	{
		// No collision
		return TRUE;
	}
	else 
	{
		// collisions exist or can not set the configuration
		return FALSE;
	}
#else
	return TRUE;
#endif
}

/*  p3d_is_config_inside_joints_bounds
 *
 *  Input:  the robot,
 *          the configuration
 *
 *  Output: False if the configuration is inside the joint bounds
 *
 *
 *  Description:
 *          This function returns false if the configuration
 *					is valid regarding the joints bounds */
int p3d_isOutOfBounds(p3d_rob* robot, configPt q){
  int isOutOfBounds = FALSE;
  for(int i = 0; i < robot->njoints; i++){
    p3d_jnt* joint = robot->joints[i];
    for(int j = 0; j < joint->dof_equiv_nbr; j++){
      double vmin = -P3D_HUGE, vmax = P3D_HUGE;
      p3d_jnt_get_dof_bounds(joint, j, &vmin, &vmax);
      if ((q[joint->index_dof + j] < (vmin - EPS6) ) || (q[joint->index_dof + j] > (vmax + EPS6) )) {
        printf("The joint %s is outside the joint bounds. Vmin : %f, Value : %f, Vmax = %f\n", joint->name, vmin, q[joint->index_dof + j], vmax);
        isOutOfBounds = TRUE;
      }
    }
  }
  return isOutOfBounds;
}
