/****************************************************************************/
/*!
 *  \file p3d_joints.c
 *
 *  \brief Joint function API.
 *
 *       The joints are kinematics articulations between two obstacles.
 * Into Move3D, it is possible to have virtual object (none object or
 * pur graphic object). This allow the composition of sevral joint
 * by the user to build specefic joints.
 *
 *       Each joint (::p3d_jnt) could have sevral degree of freedom to
 *  control the relative position of robot's bodies.
 *
 *       This file is the standard function interface for complex joints.
 *  For specfific functions on joints, it switches to specific functions
 *  in file p3d_jnt_*.c. Presently there are 5 type of joints:
 *     - ::P3D_FIXED:  Fixed link, no degree of freedom, describe
 *                     in p3d_jnt_fixed.c
 *     - ::P3D_ROTATE: Single rotation describe in p3d_jnt_rotate.c
 *     - ::P3D_TRANSLATE: Single translation describe in p3d_jnt_translate.c
 *     - ::P3D_BASE: A freeflying joint (3 translations, 3 rotations)
 *                   used for the first joint of a kinematic chain.
 *                   It is describe in p3d_jnt_base.c
 *     - ::P3D_PLAN: A plan joint (2 translations, 1 rotation),
 *                   describe in p3d_jnt_plan.c
 *     - ::P3D_FREEFLYER: A freeflying joint (3 translations, 3 rotations),
 *                        describe in p3d_jnt_freeflyer.c
 *     - ::P3D_KNEE: A knee joint (3 rotations),
 *                   describe in p3d_jnt_knee.c
 *
 */


#include "P3d-pkg.h"
#include "Util-pkg.h"
#include "Planner-pkg.h"

/*! \brief Maximal size for the name of a joint and its degree of freedom */
#define DOF_MAX_SIZE_NAME (JNT_MAX_SIZE_NAME+6)

double WEIGHT_ROTATIONS = 1.;



/*! \brief Default name for a joint.
 *  \internal
 */
static char * s_dof_name[DOF_MAX_SIZE_NAME];

/*--------------------------------------------------------------------------*/
/*!
 *  \brief Get the joint that hold the value of a degree of freedom
 *         for a given robot, return the index of this value in this joint.
 *
 *  \param  robotPt:  the robot
 *  \param  i_robot:  the index of the degree of freedom
 *                    in the robot configuration
 *
 *  \return The joint that holds this degree of freedom (NULL if there is no
 *          joint that holds it)
 *
 *  \retval i_joint: the index of the degree of freedom in the joint.
 */
p3d_jnt * p3d_robot_dof_to_jnt(p3d_rob * robotPt, int i_robot, int * i_joint) {
  int j, nb_joints;
  p3d_jnt *jntPt;

  nb_joints = robotPt->njoints;
  for(j=0; j<=nb_joints; j++) {
    jntPt = robotPt->joints[j];
    if (((jntPt->index_dof+jntPt->dof_equiv_nbr)>i_robot) &&
        (jntPt->index_dof<=i_robot)) {
      *i_joint = i_robot-jntPt->index_dof;
      return jntPt;
    }
  }
  *i_joint = -1;
  return NULL;
}


/*--------------------------------------------------------------------------*/
/*!
 *  \brief Get the index of the user degree of freedom in the real
 *         configuration.
 *
 *  \param  robotPt:  the robot
 *  \param  i_user_dof:  the index of the degree of freedom
 *                       in the user configuration
 *
 *  \return The index of this degree of freedom in the real configuration
 *          (-1 if there is no joint that holds it)
 */
int p3d_robot_user_dof_to_dof(p3d_rob * robotPt, int i_user_dof) {
  int j, nb_joints;
  int i, i_dof;
  p3d_jnt *jntPt;

  nb_joints = robotPt->njoints;
  for(j=0; j<=nb_joints; j++) {
    jntPt = robotPt->joints[j];
    if (((jntPt->index_user_dof+jntPt->user_dof_equiv_nbr)>i_user_dof) &&
        (jntPt->index_user_dof<=i_user_dof)) {
      i_dof = jntPt->index_dof;
      i_user_dof -= jntPt->index_user_dof;
      for(i=0; i<jntPt->dof_equiv_nbr; i++) {
        if(p3d_jnt_get_dof_is_user(jntPt, i)) {
          i_user_dof--;
          if (i_user_dof<0) { return i_dof; }
        }
        i_dof ++;
      }
      return i_dof;
    }
  }
  PrintError(("p3d_robot_user_dof_to_dof: index not valid!\n"));
  return 0;
}


/*--------------------------------------------------------------------------*/
/*!
 *  \brief Get the type of the degree of freedom for a given type of joint
 *         (angular or not)
 *
 *  This function is used to know if the degree of freedom is angular
 *  (in radian between \f$ -\pi \f$ and \f$ \pi \f$ or between \f$ 0 \f$
 *   and \f$ 2.\pi \f$, so the value doesn't depend of the environment),
 *  or if it's not (so its bounds depend of the scale factor and of the
 *  joint description).
 *  Then, to compare angular dof and not angular dof we must multiply
 *  angular dof by a distance (classicaly jnt::dist)
 *
 *  \param type:   the type of joint
 *  \param i_dof:  the index of the degree of freedom
 *
 *  \return TRUE angular, FALSE not angular
 */
int p3d_jnt_get_dof_is_angular(p3d_type_joint type, int i_dof) {
  switch(type) {
    case P3D_BASE:
    case P3D_FREEFLYER:
    if (i_dof<3) { return FALSE; } else { return TRUE; }
    case P3D_PLAN:
    if (i_dof<2) { return FALSE; } else { return TRUE; }
    case P3D_ROTATE:
    case P3D_KNEE:
      return TRUE;
    case P3D_TRANSLATE:
      return FALSE;
    case P3D_FIXED:
      return FALSE;
  }
  return FALSE;
}


/*--------------------------------------------------------------------------*/
/*!
 *  \brief Get the type of the degree of freedom for a given joint
 *         (angular or not)
 *
 *  This function is used to know if the degree of freedom is angular
 *  (in radian between \f$ -\pi \f$ and \f$ \pi \f$ or between \f$ 0 \f$
 *   and \f$ 2.\pi \f$, so the value doesn't depend of the environment),
 *  or if it's not (so its bounds depend of the scale factor and of the
 *  joint description).
 *  Then, to compare angular dof and not angular dof we must multiply
 *  angular dof by a distance (classicaly jnt::dist)
 *
 *  \param jntPt:  the joint
 *  \param i_dof:  the index of the degree of freedom
 *
 *  \return TRUE angular, FALSE not angular
 */
int p3d_jnt_is_dof_angular(p3d_jnt * jntPt, int i_dof) {
  return p3d_jnt_get_dof_is_angular(jntPt->type, i_dof);
}


/*--------------------------------------------------------------------------*/
/*!
 *  \brief Get the type of the degree of freedom for a given joint
 *         (linear or not)
 *
 *  This function is used to know if the degree of freedom is partialy
 *  in translation. For most of the joints p3d_jnt_is_dof_angular()
 *  and p3d_jnt_is_dof_linear() are opposit functions. But some joint like
 *  the helicoid joint are both linear and angular.
 *
 *  \param jntPt:  the joint
 *  \param i_dof:  the index of the degree of freedom
 *
 *  \return TRUE linear, FALSE not linear (angular)
 */
int p3d_jnt_is_dof_linear(p3d_jnt * jntPt, int i_dof) {
  return !p3d_jnt_get_dof_is_angular(jntPt->type, i_dof);
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Get the type of the degree of freedom for a given joint
 *        (circular or not)
 *
 *  This calified joint when their bounds are linked together, when, for
 *  a rotation, the joint could freely turn. In this case, the computation
 *  of the value of the joint must check what is the sens of rotation
 *  that gives the shortest path.
 *
 *  \param jntPt:  the joint
 *  \param i_dof:  the index of the degree of freedom
 *
 *  \return TRUE if circular, FALSE if not.
 */
int p3d_jnt_compute_is_dof_circular(p3d_jnt * jntPt, int i_dof) {
  double vmin, vmax;

  if (p3d_jnt_is_dof_linear(jntPt, i_dof)) { return FALSE; }
  p3d_jnt_get_dof_bounds(jntPt, i_dof, &vmin, &vmax);
  if ((dist_circle(vmin, vmax) < EPS6) && (ABS(vmin-vmax)>EPS6)) { return TRUE; }
  return FALSE;
}

/*--------------------------------------------------------------------------*/
/*!
 * \brief Get the name of a degree of freedom for a given joint
 *
 *  This function is used for printing purpose.
 *
 *  \param jntPt:  the joint
 *  \param i_dof:  the index of the degree of freedom
 *
 *  \return The name of the degree of freedom
 */
const char * p3d_jnt_get_dof_name(p3d_jnt * jntPt, int i_dof) {
  int i = 0;

  fixed_str_copy((char *)s_dof_name, jntPt->name, &i, DOF_MAX_SIZE_NAME);
  if ((i_dof>=0) && (i_dof<jntPt->dof_equiv_nbr)) {
    fixed_str_copy((char *)s_dof_name, " ", &i, DOF_MAX_SIZE_NAME);
    fixed_str_copy((char *)s_dof_name,
                   p3d_jnt_get_only_dof_name(jntPt->type, i_dof),
                   &i, DOF_MAX_SIZE_NAME);
  }

  return (const char *)s_dof_name;
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Get the name for a given a degree of freedom
 *
 *  This function is used for printing purpose.
 *
 *  \param type:   the joint type
 *  \param i_dof:  the index of the degree of freedom
 *
 *  \return The name of the degree of freedom
 */
const char * p3d_jnt_get_only_dof_name(p3d_type_joint type, int i_dof) {
  switch(type) {
    case P3D_BASE:
      return p3d_jnt_base_get_dof_name(i_dof);
    case P3D_FREEFLYER:
      return p3d_jnt_freeflyer_get_dof_name(i_dof);
    case P3D_PLAN:
      return p3d_jnt_plan_get_dof_name(i_dof);
    case P3D_KNEE:
      return p3d_jnt_knee_get_dof_name(i_dof);
    case P3D_ROTATE:
      return p3d_jnt_rotate_get_dof_name(i_dof);
    case P3D_TRANSLATE:
      return p3d_jnt_translate_get_dof_name(i_dof);
    case P3D_FIXED:
      PrintError(("No degree of freedom !!!\n"));
      return NULL;
  }
  return NULL;
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Set the name of the joint.
 *
 *  \warning The size of the name must be less than :JNT_MAX_SIZE_NAME
 *
 *  \param jntPt:  the joint
 *  \param name:   the name
 */
void p3d_jnt_set_name(p3d_jnt * jntPt, const char * name) {
  strncpy(jntPt->name, name, JNT_MAX_SIZE_NAME-1);
}


/***************************************************************************
 * Function to get the parameters of the degree of freedom
 */

/*--------------------------------------------------------------------------*/
/*!
 * \brief Get the value of a degree of freedom for the given joint
 *
 *  \param jntPt:  the joint
 *  \param i_dof:  the index of the degree of freedom
 *
 *  \return The value of the degree of freedom (expressed in radian for angles)
 */
double p3d_jnt_get_dof(p3d_jnt * jntPt, int i_dof) {
  return jntPt->dof_data[i_dof].v;
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Get the value of a degree of freedom for the given joint
 *
 *  Same function than p3d_jnt_get_dof(), but return degree for angles.
 *
 *  \param jntPt:  the joint
 *  \param i_dof:  the index of the degree of freedom
 *
 *  \return The value of the degree of freedom (expressed in degree for angles)
 */
double p3d_jnt_get_dof_deg(p3d_jnt * jntPt, int i_dof) {
  if (p3d_jnt_is_dof_angular(jntPt, i_dof)) { return RTOD(jntPt->dof_data[i_dof].v); }
  return jntPt->dof_data[i_dof].v;
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Get the flag of user value
 *
 * Note: If it isn't controled by user, then it isn't controled by p3d_shoot.
 *       This flag could be used for placement joint, passif joint or
 *       degree of freedom computed by local method. A degree of freedom
 *       not controled by user does not appeared in user config.
 *
 *  \param jntPt:  the joint
 *  \param i_dof:  the index of the degree of freedom
 *
 *  \return The status of the degree of freedom (user controled (::TRUE) or
 *          not (::FALSE))
 */
int p3d_jnt_get_dof_is_user(p3d_jnt * jntPt, int i_dof) {
  return jntPt->dof_data[i_dof].is_user;
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Get the flag is_active_for_planner
 *
 * Note: This flag maybe redundant with the flag "is_user".
 *
 *  \param jntPt:  the joint
 *  \param i_dof:  the index of the degree of freedom
 *
 *  \return The status of the degree of freedom (planner controled (::TRUE) or
 *          not (::FALSE))
 */
int p3d_jnt_get_dof_is_active_for_planner(p3d_jnt * jntPt, int i_dof) {
  return jntPt->dof_data[i_dof].is_active_for_planner;
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Get whether or not the degree of freedom as been modified.
 *
 *  \param jntPt:  the joint
 *  \param i_dof:  the index of the degree of freedom
 *
 *  \return The status of the degree of freedom (modified (::TRUE) or
 *          not (::FALSE))
 *
 *  \note A call to p3d_jnt_set_dof() can modify the degree of freedom.
 *        A call to p3d_jnt_clean_update() uncheck the change.
 */
int p3d_jnt_get_dof_is_modified(p3d_jnt * jntPt, int i_dof) {
  return jntPt->dof_data[i_dof].is_modified;
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Get the value v0 for a degree of freedom in the given joint
 *
 *  Note: This function is used in the sdk.
 *
 *  \param jntPt:  the joint
 *  \param i_dof:  the index of the degree of freedom
 *
 *  \return The value of the degree of freedom (expressed in radian for angles)
 */
double p3d_jnt_get_dof_v0(p3d_jnt * jntPt, int i_dof) {
  return jntPt->dof_data[i_dof].v0;
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Get the value v0 for a degree of freedom in the given joint
 *
 *  Same function than p3d_jnt_get_dof(), but return degree for angles.
 *
 *  Note: This function is used in the sdk.
 *
 *  \param jntPt:  the joint
 *  \param i_dof:  the index of the degree of freedom
 *
 *  \return The value of the degree of freedom (expressed in degree for angles)
 */
double p3d_jnt_get_dof_v0_deg(p3d_jnt * jntPt, int i_dof) {
  if (p3d_jnt_is_dof_angular(jntPt, i_dof)) { return RTOD(jntPt->dof_data[i_dof].v0); }
  return jntPt->dof_data[i_dof].v0;
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Get the initial axis for a degree of freedom in the given joint
 *
 *  \param jntPt:  the joint
 *  \param i_dof:  the index of the degree of freedom
 *
 *  \retval axe:   The axe of the degree of freedom in the given joint.
 */
void p3d_jnt_get_dof_axis(p3d_jnt * jntPt, int i_dof, p3d_vector3 axe) {
  axe[0] = jntPt->dof_data[i_dof].axis[0];
  axe[1] = jntPt->dof_data[i_dof].axis[1];
  axe[2] = jntPt->dof_data[i_dof].axis[2];
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Get the current axis for a degree of freedom in the given joint
 *
 *  Get the current normalized axis of the degree of freedom for \a jntPt.
 *
 *  \param jntPt:  the joint
 *  \param i_dof:  the index of the degree of freedom
 *
 *  \retval axe:   The axe of the degree of freedom in the given joint.
 */
void p3d_jnt_get_dof_cur_axis(p3d_jnt * jntPt, int i_dof, p3d_vector3 axe) {
  int i = 2;
  /* Note: the row of abs_pos are respectivly the current axis x, y, z */

  switch(jntPt->type) {
    case P3D_BASE:
    case P3D_FREEFLYER:
    case P3D_PLAN:
    case P3D_KNEE:
      i = i_dof % 3;
      break;
    case P3D_ROTATE:
    case P3D_TRANSLATE:
      i = 2;
      break;
    case P3D_FIXED:
      PrintError(("No degree of freedom !!!\n"));
      break;
  }
  axe[0] = jntPt->abs_pos[0][i];
  axe[1] = jntPt->abs_pos[1][i];
  axe[2] = jntPt->abs_pos[2][i];
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Get the current axis for a degree of freedom in the given joint
 *        before the apply of jnt::jnt_mat.
 *
 *  Get the current normalized axis of the degree of freedom for \a jntPt
 *  before the apply of jnt::jnt_mat.
 *
 *  \param jntPt:  the joint
 *  \param i_dof:  the index of the degree of freedom
 *
 *  \retval axe:   The axe of the degree of freedom in the given joint.
 */
void p3d_jnt_get_dof_cur_axis_before(p3d_jnt * jntPt, int i_dof,
                                     p3d_vector3 axe) {
  int i = 2;
  /* Note: the row of abs_pos are respectivly the current axis x, y, z */

  switch(jntPt->type) {
    case P3D_BASE:
    case P3D_FREEFLYER:
    case P3D_PLAN:
    case P3D_KNEE:
      i = i_dof % 3;
      break;
    case P3D_ROTATE:
    case P3D_TRANSLATE:
      i = 2;
      break;
    case P3D_FIXED:
      PrintError(("No degree of freedom !!!\n"));
      break;
  }
  axe[0] = jntPt->abs_pos_before_jnt[0][i];
  axe[1] = jntPt->abs_pos_before_jnt[1][i];
  axe[2] = jntPt->abs_pos_before_jnt[2][i];
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Get the value of the bounds for a given joint and degree of freedom
 *
 *  \param jntPt:  the joint
 *  \param i_dof:  the index of the degree of freedom
 *
 *  \retval vmin:  The minimum value of the degree of freedom
 *                 (expressed in radian for angles)
 *  \retval vmax:  The maximum value of the degree of freedom
 *                 (expressed in radian for angles)
 */
void p3d_jnt_get_dof_bounds(p3d_jnt * jntPt, int i_dof, double *vmin,
                            double *vmax) {
  *vmin = jntPt->dof_data[i_dof].vmin;
  *vmax = jntPt->dof_data[i_dof].vmax;
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Get the value of the bounds for a given joint and degree of freedom
 *
 *  Same function than p3d_jnt_get_dof_bounds(), but return degree for angles.
 *
 *  \param jntPt:  the joint
 *  \param i_dof:  the index of the degree of freedom
 *
 *  \retval vmin:  The minimum value of the degree of freedom
 *                 (expressed in degree for angles)
 *  \retval vmax:  The maximum value of the degree of freedom
 *                 (expressed in degree for angles)
 */
void p3d_jnt_get_dof_bounds_deg(p3d_jnt * jntPt, int i_dof,
                                double *vmin, double *vmax) {
  if (p3d_jnt_is_dof_angular(jntPt, i_dof)) {
    *vmin = RTOD(jntPt->dof_data[i_dof].vmin);
    *vmax = RTOD(jntPt->dof_data[i_dof].vmax);
  } else {
    *vmin = jntPt->dof_data[i_dof].vmin;
    *vmax = jntPt->dof_data[i_dof].vmax;
  }
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Get the value of the bounds for random shoot for
 *        a given joint and degree of freedom
 *
 *  \param jntPt:  the joint
 *  \param i_dof:  the index of the degree of freedom
 *
 *  \retval vmin:  The minimum value for random on the degree of freedom
 *                 (expressed in radian for angles)
 *  \retval vmax:  The maximum value for random on the degree of freedom
 *                 (expressed in radian for angles)
 */
void p3d_jnt_get_dof_rand_bounds(p3d_jnt * jntPt, int i_dof,
                                 double *vmin, double *vmax) {
  *vmin = jntPt->dof_data[i_dof].vmin_r;
  *vmax = jntPt->dof_data[i_dof].vmax_r;
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Get the value of the bounds for random shoot for
 *        a given joint and degree of freedom
 *
 *  Same function than p3d_jnt_get_dof_rand_bounds(),
 *  but return degree for angles.
 *
 *  \param jntPt:  the joint
 *  \param i_dof:  the index of the degree of freedom
 *
 *  \retval vmin:  The minimum value for random of the degree of freedom
 *                 (expressed in degree for angles)
 *  \retval vmax:  The maximum value for random of the degree of freedom
 *                 (expressed in degree for angles)
 */
void p3d_jnt_get_dof_rand_bounds_deg(p3d_jnt * jntPt, int i_dof,
                                     double *vmin, double *vmax) {
  if (p3d_jnt_is_dof_angular(jntPt, i_dof)) {
    *vmin = RTOD(jntPt->dof_data[i_dof].vmin_r);
    *vmax = RTOD(jntPt->dof_data[i_dof].vmax_r);
  } else {
    *vmin = jntPt->dof_data[i_dof].vmin_r;
    *vmax = jntPt->dof_data[i_dof].vmax_r;
  }
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Get the position initial of the joint
 *
 *  \param jntPt:  the joint
 *
 *  \retval pointPt:  The position of the joint
 */
void p3d_jnt_get_point(p3d_jnt * jntPt, p3d_point * pointPt) {
  pointPt->x = jntPt->pos0[0][3];
  pointPt->y = jntPt->pos0[1][3];
  pointPt->z = jntPt->pos0[2][3];
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Get the position initial of the joint
 *
 *  \param jntPt:  the joint
 *
 *  \retval pos:   The position of the joint
 */
void p3d_jnt_get_vect_point(p3d_jnt * jntPt, p3d_vector3 pos) {
  pos[0] = jntPt->pos0[0][3];
  pos[1] = jntPt->pos0[1][3];
  pos[2] = jntPt->pos0[2][3];
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Get the current absolut position of the joint
 *
 *  \param jntPt:  the joint
 *
 *  \retval pos:   The position of the joint
 */
void p3d_jnt_get_cur_vect_point(p3d_jnt * jntPt, p3d_vector3 pos) {
  pos[0] = jntPt->abs_pos[0][3];
  pos[1] = jntPt->abs_pos[1][3];
  pos[2] = jntPt->abs_pos[2][3];
}

/*--------------------------------------------------------------------------*/
/*!
 * \brief Get the current absolut position of the joint
 *        before the apply of jnt::jnt_mat.
 *
 *  \param jntPt:  the joint
 *
 *  \retval pos:   The position of the joint
 */
void p3d_jnt_get_cur_vect_point_before(p3d_jnt * jntPt, p3d_vector3 pos) {
  pos[0] = jntPt->abs_pos_before_jnt[0][3];
  pos[1] = jntPt->abs_pos_before_jnt[1][3];
  pos[2] = jntPt->abs_pos_before_jnt[2][3];
}

/*--------------------------------------------------------------------------*/
/*!
 * \brief Indicate whether or not there is an object on the joint
 *
 * Note: the object could be pur graphic. To test also pur graphic
 *       object use p3d_col_object_is_pure_graphic().
 *
 *  \param jntPt:  the joint
 *
 *  \retval TRUE if there is an object / FALSE else
 */
int p3d_jnt_is_with_object(p3d_jnt * jntPt) {
  if (jntPt->o == NULL) { return FALSE; }
  if (jntPt->o->np <= 0) { return FALSE; }
  return TRUE;
}


/***************************************************************************
 * Function to change the parameters of the degree of freedom
 */


/*--------------------------------------------------------------------------*/
/*!
 * \brief Change the value of a degree of freedom for the given joint
 *
 * Note: this function doesn't check if the value is valid
 *       (between the bounds).
 *
 *  \param jntPt:  the joint
 *  \param i_dof:  the index of the degree of freedom
 *  \param val:    the new value for this degree of freedom
 *                 (expressed in radian for angles)
 */
void p3d_jnt_set_dof(p3d_jnt * jntPt, int i_dof, double val) {
  int i;

  if (EQ(jntPt->dof_data[i_dof].old_v, val)) {
    if (jntPt->pos_updated) {
      if (EQ(val, jntPt->dof_data[i_dof].v)) { jntPt->pos_updated = FALSE; }
    } else {
      if (jntPt->mat_modified) {
        jntPt->mat_modified = FALSE;
        for(i=0; i<jntPt->dof_equiv_nbr; i++) {
          if (!EQ(jntPt->dof_data[i].old_v, jntPt->dof_data[i].v) &&
              (i!=i_dof)) {
            jntPt->mat_modified = TRUE;
            break;
          }
        }
      }
    }
  } else {
    jntPt->mat_modified = TRUE;
    p3d_jnt_set_dof_is_modified(jntPt, i_dof, TRUE);
  }
  switch(jntPt->type) {
    case P3D_BASE:
      p3d_jnt_base_set_dof(jntPt, i_dof, val);
      break;
    default:
      jntPt->dof_data[i_dof].v = val;
  }
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Change the value of a degree of freedom for the given joint
 *
 * Note:
 *     - this function doesn't check if the value is valid
 *       (between the bounds).
 *     - this function looks like p3d_jnt_set_dof(),
 *       but here angles are in degree
 *
 *  \param jntPt:  the joint
 *  \param i_dof:  the index of the degree of freedom
 *  \param val:    the new value for this degree of freedom
 *                 (expressed in degree for angles)
 */
void p3d_jnt_set_dof_deg(p3d_jnt * jntPt, int i_dof, double val) {
  if (p3d_jnt_is_dof_angular(jntPt, i_dof)) { p3d_jnt_set_dof(jntPt, i_dof, DTOR(val)); } else { p3d_jnt_set_dof(jntPt, i_dof, val); }
}

/*--------------------------------------------------------------------------*/
/*!
 * \brief Change the index of user config in joints.
 *
 * Note: This function is used when a joint changes its nuber of degree of
 *       freedom controled by user. In this case, we have to change the
 *       value of jnt::index_user_dof to have the new indice of the degree
 *       of freedom in a user configuration.
 *
 *  \param robotPt:  the robot
 *  \param i_jnt:    the index of the joint that has changed the number
 *                   of user degree of freedom.
 *  \param val:      The change in the number of user degree of freedom.
 *                   (>0 add controled user degree of freedom)
 *
 *  \internal
 */
static void p3d_rob_switch_user_joint_user_index(p3d_rob * robotPt,
                                                 int i_jnt, int val) {
  int i;

  if ((robotPt!=NULL) && (val!=0)) {
    robotPt->nb_user_dof += val;
    for(i=i_jnt+1; i<=robotPt->njoints; i++) { robotPt->joints[i]->index_user_dof += val; }
  }
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Set the flag of user value
 *
 * Note: If it isn't controled by user, then it isn't controled by p3d_shoot.
 *       This flag could be used for placement joint, passif joint or
 *       degree of freedom computed by local method. A degree of freedom
 *       not controled by user does not appeared in user config.
 *
 *  \param jntPt:  the joint
 *  \param i_dof:  the index of the degree of freedom
 *  \param flag:   The status of the degree of freedom
 *                 (user controled (::TRUE) or not (::FALSE))
 */
void p3d_jnt_set_dof_is_user(p3d_jnt * jntPt, int i_dof, int flag) {
  if ((jntPt!=NULL) && (i_dof>=0) && (i_dof<=jntPt->dof_equiv_nbr) &&
      (((jntPt->dof_data[i_dof].is_user) && !(flag)) ||
       (!(jntPt->dof_data[i_dof].is_user) && (flag)))) {
    jntPt->dof_data[i_dof].is_user = flag;
    if (flag && (jntPt->user_dof_equiv_nbr < jntPt->dof_equiv_nbr)) {
      jntPt->user_dof_equiv_nbr ++;
      p3d_rob_switch_user_joint_user_index(jntPt->rob, jntPt->num, 1);
    }
    if (!flag && (jntPt->user_dof_equiv_nbr > 0)) {
      jntPt->user_dof_equiv_nbr --;
      p3d_rob_switch_user_joint_user_index(jntPt->rob, jntPt->num, -1);
    }
  }
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Set the flag is_active_for_planner
 *
 * Note: This flag maybe redundant with the flag "is_user".
 *
 *  \param jntPt:  the joint
 *  \param i_dof:  the index of the degree of freedom
 *  \param flag:   The status of the degree of freedom
 *                 (planner controled (::TRUE) or not (::FALSE))
 */
void p3d_jnt_set_dof_is_active_for_planner(p3d_jnt * jntPt, int i_dof, int flag) {
  jntPt->dof_data[i_dof].is_active_for_planner = flag;
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Set the flag of user value for all degree of freedom of the joint.
 *
 * Note: If it isn't controled by user, then it isn't controled by p3d_shoot.
 *       This flag could be used for placement joint, passif joint or
 *       degree of freedom computed by local method. A degree of freedom
 *       not controled by user does not appeared in user config.
 *
 *  \param jntPt:  the joint
 *  \param flag:   The status of the degree of freedom
 *                 (user controled (::TRUE) or not (::FALSE))
 */
void p3d_jnt_set_is_user(p3d_jnt * jntPt, int flag) {
  int i;

  for(i=0; i<jntPt->dof_equiv_nbr; i++) { p3d_jnt_set_dof_is_user(jntPt, i, flag); }
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Returns FALSE if at least one of the dofs is passive for the planner
 *
 * Note: This flag maybe redundant with the flag "is_user".

 *  \param jntPt:  the joint
 */
int p3d_jnt_get_is_active_for_planner(p3d_jnt * jntPt) {
  int i;

  for(i=0; i<jntPt->dof_equiv_nbr; i++)
    if(!p3d_jnt_get_dof_is_active_for_planner(jntPt,i))
      return FALSE;

  return TRUE;
}

/*--------------------------------------------------------------------------*/
/*!
 * \brief Set the flag is_active_for_planner for all degree of freedom of the joint.
 *
 * Note: This flag maybe redundant with the flag "is_user".

 *  \param jntPt:  the joint
 *  \param flag:   The status of the degree of freedom
 *                 (planner controled (::TRUE) or not (::FALSE))
 */
void p3d_jnt_set_is_active_for_planner(p3d_jnt * jntPt, int flag) {
  int i;

  for(i=0; i<jntPt->dof_equiv_nbr; i++) { p3d_jnt_set_dof_is_active_for_planner(jntPt, i, flag); }
}


/**
 * \brief Desactive a joint for the planner with its dof
 * @param robotPt
 * @param jnt
 */
void p3d_jnt_set_is_active_for_planner2(p3d_jnt * jnt, int flag) {
	for(int j=0; j< jnt->dof_equiv_nbr;j++) {
		p3d_jnt_set_dof_is_user(jnt, j, flag);
	}
	p3d_jnt_set_is_active_for_planner(jnt, flag);
}

/*--------------------------------------------------------------------------*/
/*!
 * \brief Set whether or not the degree of freedom as been modified.
 *
 *  \param jntPt:  the joint
 *  \param i_dof:  the index of the degree of freedom
 *  \param val:    the state of the degree of freedom (TRUE or FALSE)
 */
void p3d_jnt_set_dof_is_modified(p3d_jnt * jntPt, int i_dof, int val) {
  if (val) { jntPt->dof_data[i_dof].is_modified = TRUE; } else { jntPt->dof_data[i_dof].is_modified = FALSE; }
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Force the joint to be updated the next time
 *
 *  \param jntPt:  the joint
 */
void p3d_jnt_force_update(p3d_jnt * jntPt) {
  int i_dof;

  jntPt->mat_modified = TRUE;
  for(i_dof=0; i_dof<jntPt->dof_equiv_nbr; i_dof++) {
    p3d_jnt_set_dof_is_modified(jntPt, i_dof, TRUE);
    jntPt->dof_data[i_dof].old_v = P3D_HUGE;
  }
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Change the value of v0 for a degree of freedom in the given joint
 *
 * Note: this function doesn't check if the value is valid
 *       (between the bounds). This function is used in the sdk.
 *
 *  \param jntPt:  the joint
 *  \param i_dof:  the index of the degree of freedom
 *  \param val:    the new value for this degree of freedom
 *                 (expressed in radian for angles)
 */
void p3d_jnt_set_dof_v0(p3d_jnt * jntPt, int i_dof, double val) {
  jntPt->dof_data[i_dof].v0 = val;
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Change the value of v0 for a degree of freedom in the given joint
 *
 * Note:
 *     - this function doesn't check if the value is valid
 *       (between the bounds).
 *     - this function looks like p3d_jnt_set_dof(),
 *       but here angles are in degree
 *     - this function is mainly used by the sdk.
 *
 *  \param jntPt:  the joint
 *  \param i_dof:  the index of the degree of freedom
 *  \param val:    the new value for this degree of freedom
 *                 (expressed in degree for angles)
 */
void p3d_jnt_set_dof_v0_deg(p3d_jnt * jntPt, int i_dof, double val) {
  if (p3d_jnt_is_dof_angular(jntPt, i_dof)) { p3d_jnt_set_dof_v0(jntPt, i_dof, DTOR(val)); } else { p3d_jnt_set_dof_v0(jntPt, i_dof, val); }
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Change the value of the bounds for a given joint and
 *        degree of freedom
 *
 * Note: this function doesn't check if the values are valid
 *       (vmin<vmax).
 *
 *  \param jntPt:  the joint
 *  \param i_dof:  the index of the degree of freedom
 *  \param vmin:   the new minimum bound for this degree of freedom
 *                 (expressed in radian for angles)
 *  \param vmax:   the new maximum bound for this degree of freedom
 *                 (expressed in radian for angles)
 */
void p3d_jnt_set_dof_bounds(p3d_jnt * jntPt, int i_dof,
                            double vmin, double vmax) {
  switch(jntPt->type) {
    case P3D_BASE:
      p3d_jnt_base_set_dof_bounds(jntPt, i_dof, vmin, vmax);
      break;
    default:
      jntPt->dof_data[i_dof].vmin = vmin;
      jntPt->dof_data[i_dof].vmax = vmax;
  }
  jntPt->dof_data[i_dof].circular = p3d_jnt_compute_is_dof_circular(jntPt, i_dof);
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Change the value of the bounds for a given joint and
 *        degree of freedom
 *
 * Note:
 *     - this function doesn't check if the values are valid
 *       (vmin<vmax).
 *     - this function looks like p3d_jnt_set_dof_bounds(),
 *       but here angles are in degree
 *
 *  \param jntPt:  the joint
 *  \param i_dof:  the index of the degree of freedom
 *  \param vmin:   the new minimum bound for this degree of freedom
 *                 (expressed in degree for angles)
 *  \param vmax:   the new maximum bound for this degree of freedom
 *                 (expressed in degree for angles)
 */
void p3d_jnt_set_dof_bounds_deg(p3d_jnt * jntPt, int i_dof,
                                double vmin, double vmax) {
  if (p3d_jnt_is_dof_angular(jntPt, i_dof)) { p3d_jnt_set_dof_bounds(jntPt, i_dof, DTOR(vmin), DTOR(vmax)); } else { p3d_jnt_set_dof_bounds(jntPt, i_dof, vmin, vmax); }
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Change the value of the random bounds for a given joint and
 *        degree of freedom
 *
 * Note: this function doesn't check if the values are valid
 *       (jnt::vmin <= vmin <= vmax <= jnt::vmax).
 *
 *  \param jntPt:  the joint
 *  \param i_dof:  the index of the degree of freedom
 *  \param vmin:   the new minimum random bound for this degree of freedom
 *                 (expressed in radian for angles)
 *  \param vmax:   the new maximum random bound for this degree of freedom
 *                 (expressed in radian for angles)
 */
void p3d_jnt_set_dof_rand_bounds(p3d_jnt * jntPt, int i_dof,
                                 double vmin, double vmax) {
  jntPt->dof_data[i_dof].vmin_r = vmin;
  jntPt->dof_data[i_dof].vmax_r = vmax;
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Change the value of the random bounds for a given joint and
 *        degree of freedom
 *
 * \note
 *     - this function doesn't check if the values are valid
 *       (jnt::vmin <= vmin <= vmax <= jnt::vmax).
 *     - this function looks like p3d_jnt_set_dof_rand_bounds(),
 *       but here angles are in degree
 *
 *  \param jntPt:  the joint
 *  \param i_dof:  the index of the degree of freedom
 *  \param vmin:   the new minimum random bound for this degree of freedom
 *                 (expressed in degree for angles)
 *  \param vmax:   the new maximum random bound for this degree of freedom
 *                 (expressed in degree for angles)
 */
void p3d_jnt_set_dof_rand_bounds_deg(p3d_jnt * jntPt, int i_dof,
                                     double vmin, double vmax) {
  if (p3d_jnt_is_dof_angular(jntPt, i_dof)) { p3d_jnt_set_dof_rand_bounds(jntPt, i_dof, DTOR(vmin), DTOR(vmax)); } else { p3d_jnt_set_dof_rand_bounds(jntPt, i_dof, vmin, vmax); }
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Change the object attached to a joint
 *
 *  \param jntPt:  the joint
 *  \param objPt:  the object.
 *
 * \note This function update the relative position matrix between
 *       the ::p3d_poly of the object and the joint.
 */

void p3d_jnt_update_rel_pos_object(p3d_jnt * jntPt, p3d_obj * objPt) {
  int ip;
  p3d_matrix4 inv_pos0;
  p3d_poly *p;

  if (jntPt != NULL) {
    if (objPt != NULL) {
      p3d_matInvertXform(jntPt->pos0_obs, inv_pos0);
      /* Change the relative position of each polyhedre */
      for(ip=0;ip<objPt->np;ip++) {
        p = objPt->pol[ip];
        p3d_matMultXform(inv_pos0, p->pos0, p->pos_rel_jnt);
      }
    }
  }
}

/*!
 * \brief Change the object attached to a joint
 *
 *  \param jntPt:  the joint
 *  \param objPt:  the object.
 *  \return 0 if there is not concat 1 otherwise
 * \note This function update the relative position matrix between
 *       the ::p3d_poly of the object and the joint. If there is many
 *       objects associated to one joint, concat them to one object.
 */

// modif Juan
int p3d_jnt_set_object(p3d_jnt * jntPt, p3d_obj * objPt) {
  int ip = 0, newnp = 0, concat = 0;
//   p3d_matrix4 inv_pos0;
//   p3d_poly *p;

  if (jntPt != NULL) {
    p3d_jnt_update_rel_pos_object(jntPt,objPt);

    if (jntPt->o != NULL) {
      // c'etait :
      /* Change the relative position of each polyhedre */
      /*       for(ip=0;ip<jntPt->o->np;ip++){ */
      /*  p = jntPt->o->pol[ip]; */
      /*  p3d_mat4Copy(p->pos0, p->pos_rel_jnt); */
      /*       } */
      // -> a koi ca sert !!???
      // new :
      // on fait un object compose avec le precedant et le nouveau body
      //faut peut etre verifier si l'objet existe avant de l'ajouter Mokhtar
      if (objPt != NULL) {
        newnp = jntPt->o->np + objPt->np;
        jntPt->o->pol = MY_REALLOC(jntPt->o->pol,p3d_poly *,jntPt->o->np,newnp);
        for(ip=0; ip<objPt->np; ip++) {
          jntPt->o->pol[jntPt->o->np+ip] = objPt->pol[ip];
        }
        jntPt->o->np = newnp;
        concat = 1;
      }
      // ON DOIT LIBERER MEMOIRE  ???
      // WARNING : POSSIBLE MULTIPLICITY OF OBJECTS IN ENV AND ROB !!!
      // MODIFS ARE NECCESARY IN p3d_env.c (function : p3d_end_obj)
    } else {
      jntPt->o = objPt;
    }
  }
  return concat;
}
// fmodif Juan



/*--------------------------------------------------------------------------*/
/*!
 *  \brief Compute the distance and the speed that the joint could reach
 *
 *  From the configuration q_init of the joint to q_max_param,
 *  this function computes an interval of parameter which all the points
 *  of the joint move by less than the distance given as input.
 *
 *  Note: The joint looks directly in the robot configurations
 *        the degree of freedom that it needs.
 *
 * \param  prev_data:   speed of the previous joint
 * \param  jntPt:       the joint
 * \param  distance:    the maximal distance
 * \param  qinit:       the initial configuration
 * \param  q_max_param: the final configuration
 * \param  max_param:   the value of the delta parameter in the final
 *                      configuration (it gives the range param needed
 *                                     to reach the final configuration)
 * \param  reach_param: the actual maximal range that could be reach
 *                      (previous joint can limits the range parameter)
 *
 * \retval data:        speed of the joint
 *         distance:    the distance that the joint couldn't cross
 *         reach_param: the new maximal range parameter
 *                      that could be reach
 */
void p3d_jnt_stay_within_dist(p3d_stay_within_dist_data * prev_data,
                                     p3d_jnt * jntPt,
                                     p3d_stay_within_dist_data * data,
                                     double * distance, configPt q_init,
                                     configPt q_max_param, double max_param,
                                     double * reach_param) {
  switch(jntPt->type) {
    case P3D_BASE:
      p3d_jnt_base_stay_within_dist(prev_data, jntPt, data, distance,
                                    q_init, q_max_param, max_param, reach_param);
      break;
    case P3D_FREEFLYER:
      p3d_jnt_freeflyer_stay_within_dist(prev_data, jntPt, data, distance,
                                         q_init, q_max_param, max_param,
                                         reach_param);
      break;
    case P3D_PLAN:
      p3d_jnt_plan_stay_within_dist(prev_data, jntPt, data, distance,
                                    q_init, q_max_param, max_param, reach_param);
      break;
    case P3D_KNEE:
      p3d_jnt_knee_stay_within_dist(prev_data, jntPt, data, distance,
                                    q_init, q_max_param,
                                    max_param, reach_param);
      break;
    case P3D_ROTATE:
      p3d_jnt_rotate_stay_within_dist(prev_data, jntPt, data, distance,
                                      q_init, q_max_param,
                                      max_param, reach_param);
      break;
    case P3D_TRANSLATE:
      p3d_jnt_translate_stay_within_dist(prev_data, jntPt, data, distance,
                                         q_init, q_max_param,
                                         max_param, reach_param);
      break;
    case P3D_FIXED:
      p3d_jnt_fixed_stay_within_dist(prev_data, jntPt, data, distance,
                                     q_init, q_max_param,
                                     max_param, reach_param);
      break;
  }
}

/**
 * p3d_GetWeightRotations()
 * Get the value of the weight given to the rotations
 * for distance computations. The total weight of rotations
 * is given by WeightRota*jntPt->dist
 * @return:  the value of the weight given to the rotations
 */
double p3d_GetWeightRotations(void) {
  return WEIGHT_ROTATIONS;
}

/**
 * p3d_SetWeightRotations()
 * Set the value of the weight given to the rotations
 * for distance computations. The total weight of rotations
 * is given by WeightRota*jntPt->dist
 * @param:  the value given to weight the rotations
 */
void p3d_SetWeightRotations(double RotationWeight) {
  WEIGHT_ROTATIONS = RotationWeight;
}



/*--------------------------------------------------------------------------*/
/*!
 *  \brief Compute the distance between two configurations for a joint
 *
 *  Note:
 *      - The joint looks directly in the robot configurations
 *        the degree of freedom that it needs.
 *      - This function test if the degree of freedom is angular.
 *        In this case it multiply it by jnt::dist to compare this distance
 *        with translation distances.
 *      - This function test also if the degree of freedom is circular to
 *        give the shortest distance.
 *
 *      - The weight between rotations and translations is given by WeightRota*jntPt->dist
 *
 * \param jntPt:  the joint
 * \param i_dof:  the index of the degree of freedom
 * \param  qinit: the initial robot configuration
 * \param  q_end: the final robot configuration
 *
 * \return the distance between q_init and q_end for the degree of freedom
 */
double p3d_jnt_calc_dof_dist(p3d_jnt * jntPt, int i_dof,
                             configPt q_init, configPt q_end) {
  int k = jntPt->index_dof + i_dof;
  double WeightRota = 1.;
#ifdef P3D_PLANNER
  if(p3d_GetIsWeightedRotations()) {
    WeightRota = p3d_GetWeightRotations();
  }
#endif
  if (p3d_jnt_is_dof_angular(jntPt, i_dof)) {
    /* multiply by jntPt->dist to be able to compare angle with length */
    if (p3d_jnt_is_dof_circular(jntPt, i_dof))  {
      return fabs(WeightRota*jntPt->dist*dist_circle(q_init[k], q_end[k]));
    }
    else {
      return fabs(WeightRota*jntPt->dist*(q_end[k] - q_init[k]));
    }
  }
  return fabs(q_end[k] - q_init[k]);
}

// Same as above without weigthing
// This function computes the distance without consideration of W
double p3d_jnt_calc_dof_dist_2(p3d_jnt * jntPt, int i_dof,
                             configPt q_init, configPt q_end) 
{
  int k = jntPt->index_dof + i_dof;
	
  if (p3d_jnt_is_dof_angular(jntPt, i_dof)) 
	{
    if (p3d_jnt_is_dof_circular(jntPt, i_dof)) 
      return fabs(dist_circle(q_init[k], q_end[k]));
    else
      return fabs(q_end[k] - q_init[k]);
  }
  return fabs(q_end[k] - q_init[k]);
}


/*--------------------------------------------------------------------------*/
/*!
 *  \brief Compute the value of a joint in a linear methode
 *         between two configurations
 *
 *  Note:
 *      - The joint looks directly in the robot configurations
 *        the degree of freedom that it needs.
 *      - This function test also if the degree of freedom is circular to
 *        give the shortest distance.
 *
 * \param jntPt:  the joint
 * \param i_dof:  the index of the degree of freedom
 * \param  qinit: the initial robot configuration
 * \param  q_end: the final robot configuration
 * \param  alpha: the parameter between 0 and 1.
 *
 * \return the value of the degree of freedom
 */
double p3d_jnt_calc_dof_value(p3d_jnt * jntPt, int i_dof,
                              configPt q_init, configPt q_end, double alpha) {
  // modif Juan
  int k = jntPt->index_dof + i_dof;
  double vmin,vmax;

  alpha = MAX(0.,MIN(1.,alpha));
  if (p3d_jnt_is_dof_circular(jntPt, i_dof)) 
	{
    p3d_jnt_get_dof_bounds_deg(jntPt, i_dof, &vmin, &vmax);
    if(vmin < 0.0) {
      return angle_limit_PI(q_init[k] + alpha *
                            diff_angle(q_init[k], q_end[k]));
    } else {
      return angle_limit_2PI(q_init[k] + alpha *
                             diff_angle(q_init[k], q_end[k]));
    }
    // fmodif Juan
  }
  return (q_init[k] + alpha * (q_end[k] - q_init[k]));
}


/*--------------------------------------------------------------------------*/
/*!
 *  \brief Compute the transformation matrix of the joint.
 *
 * \param jntPt:  the joint.
 *
 *  This function use the value for the degree of freedom set in the joint
 *  by p3d_jnt_set_dof().
 */
void p3d_jnt_calc_jnt_mat(p3d_jnt * jntPt) {
  switch(jntPt->type) {
    case P3D_BASE:
    case P3D_FREEFLYER:
      p3d_jnt_freeflyer_calc_jnt_mat(jntPt);
      break;
    case P3D_PLAN:
      p3d_jnt_plan_calc_jnt_mat(jntPt);
      break;
    case P3D_KNEE:
      p3d_jnt_knee_calc_jnt_mat(jntPt);
      break;
    case P3D_ROTATE:
      p3d_jnt_rotate_calc_jnt_mat(jntPt);
      break;
    case P3D_TRANSLATE:
      p3d_jnt_translate_calc_jnt_mat(jntPt);
      break;
    case P3D_FIXED:
      /* Nothing to do, always identity */
      break;
  }
}


/*--------------------------------------------------------------------------*/
/*!
 *  \brief Compute the absolute position matrix of the joint.
 *
 * \param link_jntPt:  the link between two joints.
 *
 * \return FALSE if both joint are already updated, TRUE in other case.
 *
 *  This function use the value for the degree of freedom set in the joint
 *  by p3d_jnt_set_dof().
 *  This function is able to follow the kinematic chain or to compute
 *  the inverse model.
 *
 * \note It checks if the joints need an update.
 */
int p3d_jnt_calc_mat_pos(p3d_link_between_joint * link_jntPt) {
  int sens_direct = TRUE;
  p3d_matrix4 inv_rel_pos;
  p3d_jnt * jntPt;
  p3d_jnt * jnt_prevPt;

  if (link_jntPt->prev_jnt->pos_updated) {
    if (link_jntPt->next_jnt->pos_updated) { return FALSE; }
  } else {
    if (link_jntPt->next_jnt->pos_updated) { sens_direct = FALSE; } else { return FALSE; }
  }
  if (sens_direct) {
    jntPt      = link_jntPt->next_jnt;
    jnt_prevPt = link_jntPt->prev_jnt;
  } else {
    jnt_prevPt = link_jntPt->next_jnt;
    jntPt      = link_jntPt->prev_jnt;
  }
  if (jntPt->mat_modified) {
    if (jntPt->type != P3D_BASE) { p3d_jnt_calc_jnt_mat(jntPt); }
  }
  if (sens_direct) {
    if (jnt_prevPt->abs_pos_modified) {
      if (jntPt->type == P3D_BASE) {
        p3d_matMultXform(jnt_prevPt->abs_pos, link_jntPt->rel_pos,
                         jntPt->abs_pos);
        jntPt->abs_pos_modified = TRUE;
        jntPt->pos_obj_modified = TRUE;
        p3d_jnt_base_calc_dof(jntPt);
      } else {
        p3d_matMultXform(jnt_prevPt->abs_pos, link_jntPt->rel_pos,
                         jntPt->abs_pos_before_jnt);
        jntPt->abs_pos_before_jnt_modified = TRUE;
        p3d_matMultXform(jntPt->abs_pos_before_jnt, jntPt->jnt_mat,
                         jntPt->abs_pos);
        jntPt->abs_pos_modified = TRUE;
        jntPt->pos_obj_modified = TRUE;
      }
    } else if (jntPt->mat_modified) {
      p3d_matMultXform(jntPt->abs_pos_before_jnt, jntPt->jnt_mat,
                       jntPt->abs_pos);
      jntPt->abs_pos_modified = TRUE;
      jntPt->pos_obj_modified = TRUE;
    }
  } else {
    if (jnt_prevPt->type == P3D_BASE) {
      if (jnt_prevPt->abs_pos_modified) {
        p3d_matInvertXform(link_jntPt->rel_pos, inv_rel_pos);
        p3d_matMultXform(jnt_prevPt->abs_pos, inv_rel_pos,
                         jntPt->abs_pos);
        jntPt->abs_pos_modified = TRUE;
        jntPt->pos_obj_modified = TRUE;
        if (jntPt->type == P3D_BASE) { p3d_jnt_base_calc_dof(jntPt); } else {
          p3d_matInvertXform(jntPt->jnt_mat, inv_rel_pos);
          p3d_matMultXform(jntPt->abs_pos, inv_rel_pos,
                           jntPt->abs_pos_before_jnt);
          jntPt->abs_pos_before_jnt_modified = TRUE;
        }
      } else if (jntPt->mat_modified) {
        if (jntPt->type == P3D_BASE) { p3d_jnt_base_calc_dof(jntPt); } else {
          p3d_matInvertXform(jntPt->jnt_mat, inv_rel_pos);
          p3d_matMultXform(jntPt->abs_pos, inv_rel_pos,
                           jntPt->abs_pos_before_jnt);
          jntPt->abs_pos_before_jnt_modified = TRUE;
        }
      }
    } else {
      if (jnt_prevPt->abs_pos_before_jnt_modified) {
        p3d_matInvertXform(link_jntPt->rel_pos, inv_rel_pos);
        p3d_matMultXform(jnt_prevPt->abs_pos_before_jnt, inv_rel_pos,
                         jntPt->abs_pos);
        jntPt->abs_pos_modified = TRUE;
        jntPt->pos_obj_modified = TRUE;
        if (jntPt->type == P3D_BASE) { p3d_jnt_base_calc_dof(jntPt); } else {
          p3d_matInvertXform(jntPt->jnt_mat, inv_rel_pos);
          p3d_matMultXform(jntPt->abs_pos, inv_rel_pos,
                           jntPt->abs_pos_before_jnt);
          jntPt->abs_pos_before_jnt_modified = TRUE;
        }
      } else if (jntPt->mat_modified) {
        if (jntPt->type == P3D_BASE) { p3d_jnt_base_calc_dof(jntPt); } else {
          p3d_matInvertXform(jntPt->jnt_mat, inv_rel_pos);
          p3d_matMultXform(jntPt->abs_pos, inv_rel_pos,
                           jntPt->abs_pos_before_jnt);
          jntPt->abs_pos_before_jnt_modified = TRUE;
        }
      }
    }
  }
  jntPt->pos_updated = TRUE;
  return TRUE;
}


/*--------------------------------------------------------------------------*/
/*!
 *  \brief Set the absolute position of a joint.
 *
 * \param jntPt:   the joints.
 * \param abs_pos: the absolute position of the joint (NULL if we want
 *        to set the joint matrix (jnt::jnt_mat) as the absolute position).
 *
 * \remarks This function make the update of this joint.
 */
void p3d_jnt_set_mat_pos(p3d_jnt * jntPt, p3d_matrix4 * abs_pos) {
  p3d_matrix4 inv_pos;

  if (abs_pos == NULL) {
    if (jntPt->mat_modified) {
      p3d_jnt_calc_jnt_mat(jntPt);
      jntPt->abs_pos_modified = TRUE;
      jntPt->pos_obj_modified = TRUE;
    }
    if (!p3d_mat4IsEqual(jntPt->abs_pos, jntPt->jnt_mat)) {
      p3d_mat4Copy(jntPt->jnt_mat, jntPt->abs_pos);
      jntPt->abs_pos_modified = TRUE;
      jntPt->pos_obj_modified = TRUE;
    }
    if (!p3d_mat4IsEqual(jntPt->abs_pos_before_jnt, p3d_mat4IDENTITY)) {
      p3d_mat4Copy(p3d_mat4IDENTITY, jntPt->abs_pos_before_jnt);
      jntPt->abs_pos_before_jnt_modified = TRUE;
    }
  } else {
    if (!p3d_mat4IsEqual(jntPt->abs_pos, *abs_pos)) {
      p3d_mat4Copy(*abs_pos, jntPt->abs_pos);
      jntPt->abs_pos_modified = TRUE;
      jntPt->pos_obj_modified = TRUE;
    }
    if (jntPt->mat_modified) {
      if (jntPt->type != P3D_BASE) { p3d_jnt_calc_jnt_mat(jntPt); }
    }
    if (jntPt->type == P3D_BASE) { p3d_jnt_base_calc_dof(jntPt); } else if ((jntPt->mat_modified) || (jntPt->abs_pos_modified)) {
      p3d_matInvertXform(jntPt->jnt_mat, inv_pos);
      p3d_matMultXform(jntPt->abs_pos, inv_pos, jntPt->abs_pos_before_jnt);
      jntPt->abs_pos_before_jnt_modified = TRUE;
    }
  }
  jntPt->pos_updated = TRUE;
}


/*--------------------------------------------------------------------------*/
/*!
 *  \brief Compute the absolute position matrix of the joint.
 *
 * \param link_jntPt:  the link between two joints.
 *
 * \return FALSE if both joint are already updated, TRUE in other case.
 *
 *  This function use the value for the degree of freedom set in the joint
 *  by p3d_jnt_set_dof().
 *  This function is able to follow the kinematic chain or to compute
 *  the inverse model.
 *
 * \note It checks if the joints need an update.
 */
int p3d_jnt_calc_mat_pos_and_jac(p3d_link_between_joint * link_jntPt) {
  int sens_direct = TRUE;
  p3d_jnt * jntPt;

  if (link_jntPt->prev_jnt->pos_updated) {
    if (link_jntPt->next_jnt->pos_updated) { return FALSE; }
  } else {
    if (link_jntPt->next_jnt->pos_updated) { sens_direct = FALSE; } else { return TRUE; }
  }
  if (sens_direct) {
    jntPt      = link_jntPt->next_jnt;
  } else {
    jntPt      = link_jntPt->prev_jnt;
  }
  p3d_jnt_calc_mat_pos(link_jntPt);

  if ((jntPt->abs_pos_modified) || (jntPt->mat_modified)) {
    switch(jntPt->type) {
      case P3D_BASE:
      case P3D_FREEFLYER:
        p3d_jnt_freeflyer_calc_mat_jac(jntPt);
        break;
      case P3D_PLAN:
        p3d_jnt_plan_calc_mat_jac(jntPt);
        break;
      case P3D_KNEE:
        p3d_jnt_knee_calc_mat_jac(jntPt);
        break;
      case P3D_ROTATE:
        p3d_jnt_rotate_calc_mat_jac(jntPt);
        break;
      case P3D_TRANSLATE:
        p3d_jnt_translate_calc_mat_jac(jntPt);
        break;
      case P3D_FIXED:
        /* Nothing to do, no jacobian */
        break;
    }
  }

  return TRUE;
}


/*--------------------------------------------------------------------------*/
/*!
 *  \brief Set the absolute position of a joint.
 *
 * \param jntPt:   the joints.
 * \param abs_pos: the absolute position of the joint (NULL if we want
 *        to set the joint matrix (jnt::jnt_mat) as the absolute position).
 *
 * \remarks This function make the update of this joint.
 */
void p3d_jnt_set_mat_pos_and_jac(p3d_jnt * jntPt, p3d_matrix4 * abs_pos) {
  p3d_jnt_set_mat_pos(jntPt, abs_pos);

  if ((jntPt->abs_pos_modified) || (jntPt->mat_modified)) {
    switch(jntPt->type) {
      case P3D_BASE:
      case P3D_FREEFLYER:
        p3d_jnt_freeflyer_calc_mat_jac(jntPt);
        break;
      case P3D_PLAN:
        p3d_jnt_plan_calc_mat_jac(jntPt);
        break;
      case P3D_KNEE:
        p3d_jnt_knee_calc_mat_jac(jntPt);
        break;
      case P3D_ROTATE:
        p3d_jnt_rotate_calc_mat_jac(jntPt);
        break;
      case P3D_TRANSLATE:
        p3d_jnt_translate_calc_mat_jac(jntPt);
        break;
      case P3D_FIXED:
        /* Nothing to do, no jacobian */
        break;
    }
  }
}


/*--------------------------------------------------------------------------*/
/*!
 *  \brief Clean up the flags used to check if the joint is updated.
 *
 * \param jntPt:  the joint.
 *
 * \warning The joint must be updated before.
 */
void p3d_jnt_clean_update(p3d_jnt * jntPt) {
  int i;
  for(i=0; i<jntPt->dof_equiv_nbr; i++) { jntPt->dof_data[i].old_v = jntPt->dof_data[i].v; }
  jntPt->pos_updated = FALSE;
  jntPt->abs_pos_modified = FALSE;
  jntPt->abs_pos_before_jnt_modified = FALSE;
  jntPt->mat_modified = FALSE;
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Create a new joint partialy defined
 *
 * This function create and initialize the structure common to all joints.
 * It is used in the funtion that creates joints, but must be not used
 * otherwhere.
 *
 * \param  type: the type of the joint
 * \param  pos:  the position matrix of the joint
 *
 * \return the new joint.
 *
 * \internal
 */
p3d_jnt * p3d_jnt_create_common(p3d_matrix4 pos) {
  p3d_jnt * jntPt;

  jntPt = MY_ALLOC(p3d_jnt,1);
  if(!jntPt) {
    PrintWarning(("p3d_jnt_create_common: can't alloc a new joint !!!\n"));
    return NULL;
  }
  memset(jntPt, 0, sizeof(p3d_jnt));

  jntPt->num         = -1;

  strcpy(jntPt->name, "Joint");

  jntPt->index_dof          = -1;
  jntPt->index_user_dof     = -1;

  p3d_mat4Copy(pos, jntPt->pos0);
  p3d_mat4Copy(pos, jntPt->pos0_obs);
  /* compatibility */
  jntPt->relative_p0.x = pos[0][3];
  jntPt->relative_p0.y = pos[1][3];
  jntPt->relative_p0.z = pos[2][3];
  jntPt->p0.x = pos[0][3];
  jntPt->p0.y = pos[1][3];
  jntPt->p0.z = pos[2][3];

  p3d_mat4Copy(pos, jntPt->abs_pos_before_jnt);
  jntPt->abs_pos_modified            = TRUE;
  jntPt->abs_pos_before_jnt_modified = TRUE;
  jntPt->pos_updated                 = FALSE;
  jntPt->mat_modified                = TRUE;
  jntPt->pos_obj_modified            = TRUE;

  return jntPt;
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Create a new joint with angles in radian
 *
 * \param  type: the type of the joint
 * \param  pos:  the position matrix of the joint
 * \param  v:    the values of the degree of freedom for the joint
 * \param  vmin & vmax:  the bounds values of
 *                       the degree of freedom for the joint
 * \param  vmin_rand & vmax_rand: the random (or user) bounds values of
 *                                the degree of freedom for the joint
 * \param  velocity_max & torque_max: the maximal values of joint velocity and torque of the joint: only for P3D_ROTATE
 *
 * \param  param: the array of the parameters for the joint
 *
 * \return the new joint.
 */
p3d_jnt * p3d_jnt_create(p3d_type_joint type, p3d_matrix4 pos, double * v,
                         double * vmin, double * vmax, double * vmin_rand, double *velocity_max, double *acceleration_max, double *jerk_max, double *torque_max,
                         double * vmax_rand, double * param) {
  switch(type) {
    case P3D_BASE:
      return p3d_jnt_base_create(pos, v, vmin, vmax,
                                 vmin_rand, vmax_rand, velocity_max, acceleration_max, jerk_max, param);
    case P3D_FREEFLYER:
      return p3d_jnt_freeflyer_create(pos, v, vmin, vmax,
                                      vmin_rand, vmax_rand, velocity_max, acceleration_max, jerk_max, param);
    case P3D_PLAN:
      return p3d_jnt_plan_create(pos, v, vmin, vmax,
                                 vmin_rand, vmax_rand, param);
    case P3D_KNEE:
      return p3d_jnt_knee_create(pos, v, vmin, vmax,
                                 vmin_rand, vmax_rand, param);
    case P3D_ROTATE:
      return p3d_jnt_rotate_create(pos, v, vmin, vmax,
                                   vmin_rand, vmax_rand, velocity_max, acceleration_max, jerk_max, torque_max, param);
    case P3D_TRANSLATE:
      return p3d_jnt_translate_create(pos, v, vmin, vmax,
                                      vmin_rand, vmax_rand, param);
    case P3D_FIXED:
      return p3d_jnt_fixed_create(pos, v, vmin, vmax,
                                  vmin_rand, vmax_rand, param);
  }
  PrintWarning(("p3d_jnt_create: type unknown\n"));
  return NULL;
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Create a new joint with angles in degree.
 *
 * \param  type: the type of the joint
 * \param  pos:  the position matrix of the joint
 * \param  v:    the values of the degree of freedom for the joint
 * \param  vmin & vmax:  the bounds values of
 *                       the degree of freedom for the joint
 * \param  vmin_rand & vmax_rand: the random (or user) bounds values of
 *                                the degree of freedom for the joint
 * \param  velocity_max & torque_max: the maximal values of joint velocity and torque of the joint: only for P3D_ROTATE
 * \param  param: the array of the parameters for the joint
 *
 * \return the new joint.
 */
p3d_jnt * p3d_jnt_create_deg(int type, p3d_matrix4 pos, double * v,
                             double * vmin, double * vmax,
                             double * vmin_rand, double * vmax_rand,
                             double *velocity_max, double *acceleration_max, double *jerk_max,
			     double *torque_max, double * param) {
  switch(type) {
    case P3D_BASE:
      return p3d_jnt_base_create_deg(pos, v, vmin, vmax,
                                     vmin_rand, vmax_rand, velocity_max, acceleration_max, jerk_max, param);
    case P3D_FREEFLYER:
      return p3d_jnt_freeflyer_create_deg(pos, v, vmin, vmax,
                                          vmin_rand, vmax_rand, velocity_max, acceleration_max, jerk_max, param);
    case P3D_PLAN:
      return p3d_jnt_plan_create_deg(pos, v, vmin, vmax,
                                     vmin_rand, vmax_rand, param);
    case P3D_KNEE:
      return p3d_jnt_knee_create_deg(pos, v, vmin, vmax,
                                     vmin_rand, vmax_rand, param);
    case P3D_ROTATE:
      return p3d_jnt_rotate_create_deg(pos, v, vmin, vmax,
                                       vmin_rand, vmax_rand, velocity_max, acceleration_max, jerk_max, torque_max, param);
    case P3D_TRANSLATE:
      return p3d_jnt_translate_create(pos, v, vmin, vmax,
                                      vmin_rand, vmax_rand, param);
    case P3D_FIXED:
      return p3d_jnt_fixed_create(pos, v, vmin, vmax,
                                  vmin_rand, vmax_rand, param);
  }
  PrintWarning(("p3d_jnt_create: type unknown\n"));
  return NULL;
}


/*--------------------------------------------------------------------------*/
/*!
 *  \brief Get the number of degree of freedom, and the number of
 *         parameters needed to create a joint.
 *
 * \param  type: the type of the joint (::p3d_type_joint),
 *
 * \retval nb_dof:   the number of degree of freedom.
 * \retval nb_param: the number of parameters.
 */
void p3d_jnt_get_nb_param(p3d_type_joint type, int * nb_dof, int * nb_param) {
  switch(type) {
    case P3D_BASE:
      *nb_dof = 6;
      *nb_param = 0;
      break;
    case P3D_FREEFLYER:
      *nb_dof = 6;
      *nb_param = 0;
      break;
    case P3D_PLAN:
      *nb_dof = 3;
      *nb_param = 0;
      break;
    case P3D_KNEE:
      *nb_dof = 3;
      *nb_param = 0;
      break;
    case P3D_ROTATE:
      *nb_dof = 1;
      *nb_param = 0;
      break;
    case P3D_TRANSLATE:
      *nb_dof = 1;
      *nb_param = 0;
      break;
    case P3D_FIXED:
      *nb_dof = 0;
      *nb_param = 0;
      break;
    default:
      PrintWarning(("p3d_jnt_get_nb_param: type unknown\n"));
  }
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Change the scale factor of a joint.
 *
 * \param jntPt:  the joint
 * \param scale:  the scale factor.
 *
 * \note Update the matrice in the links owned by the joint.
 * \note Update the matrice of relative position with the object.
 */
void p3d_jnt_scale(p3d_jnt * jntPt, double scale) {
  int i, j;
  int ip;
  p3d_poly *p;

  for(i=0; i<3; i++) {
    jntPt->jnt_mat[i][3] *= scale;
    jntPt->abs_pos[i][3] *= scale;
    jntPt->abs_pos_before_jnt[i][3] *= scale;
    jntPt->pos0[i][3] *= scale;
    jntPt->pos0_obs[i][3] *= scale;
  }
  /* compatibility */
  jntPt->relative_p0.x *= scale;
  jntPt->relative_p0.y *= scale;
  jntPt->relative_p0.z *= scale;
  jntPt->p0.x          *= scale;
  jntPt->p0.y          *= scale;
  jntPt->p0.z          *= scale;
  jntPt->dist          *= scale;
  for(i=0; i<jntPt->dof_equiv_nbr; i++) {
    if (!p3d_jnt_is_dof_angular(jntPt, i)) {
      jntPt->dof_data[i].v *= scale;
      jntPt->dof_data[i].v0 *= scale;
      jntPt->dof_data[i].old_v *= scale;
      jntPt->dof_data[i].vmin *= scale;
      jntPt->dof_data[i].vmax *= scale;
      jntPt->dof_data[i].vmin_r *= scale;
      jntPt->dof_data[i].vmax_r *= scale;
    }
  }
  /* Relative position with other joints */
  for(j=0; j<jntPt->n_link_jnt_owned; j++) {
    for(i=0; i<3; i++) {
      jntPt->link_jnt_owned_arr[j]->rel_pos[i][3] *= scale;
    }
  }
  /* Relative position with the object */
  if (jntPt->o != NULL) {
    for(ip=0;ip<jntPt->o->np;ip++) {
      p = jntPt->o->pol[ip];
      for(i=0; i<3; i++) { p->pos_rel_jnt[i][3] *= scale; }
    }
  }
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Change the initial position of a joint.
 *
 * \param jntPt:  the joint
 * \param pos0:   the initial position matrix.
 *
 * \note Update the matrice in the links owned by the joint.
 * \note Does not update the matrice of relative position with the object
 *       (but not the absolute position).
 */
void p3d_jnt_change_pos0(p3d_jnt * jntPt, p3d_matrix4 pos0) {
  int j;
  //int ip;
  //p3d_poly *p;
  p3d_matrix4 inv_pos0, delta_pos0, tmp_mat;

  p3d_matInvertXform(jntPt->pos0, inv_pos0);
  p3d_matMultXform(inv_pos0, pos0, delta_pos0);
  p3d_matMultXform(jntPt->abs_pos_before_jnt, delta_pos0, tmp_mat);
  p3d_mat4Copy(tmp_mat, jntPt->abs_pos_before_jnt);
  p3d_mat4Copy(pos0, jntPt->pos0);
  p3d_matMultXform(tmp_mat, jntPt->jnt_mat, jntPt->abs_pos);

  /* compatibility */
  jntPt->relative_p0.x = pos0[0][3];
  jntPt->relative_p0.y = pos0[1][3];
  jntPt->relative_p0.z = pos0[2][3];
  jntPt->p0.x          = pos0[0][3];
  jntPt->p0.y          = pos0[1][3];
  jntPt->p0.z          = pos0[2][3];

  /* Relative position with other joints */
  for(j=0; j<jntPt->n_link_jnt_owned; j++) {
    if (jntPt->link_jnt_arr[j]->prev_jnt == jntPt) { p3d_matMultXform(delta_pos0,jntPt->link_jnt_arr[j]->rel_pos,tmp_mat); } else { p3d_matMultXform(jntPt->link_jnt_arr[j]->rel_pos,delta_pos0,tmp_mat); }
    p3d_mat4Copy(tmp_mat, jntPt->link_jnt_arr[j]->rel_pos);
  }
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Attach a joint to an other.
 *
 * \param prev_jntPt:  the previous joint
 * \param jntPt:       the joint
 *
 * \note Update the next joint array of the other joint.
 * \note Create a link_between_joint.
 */
void p3d_jnt_attach_to_jnt(p3d_jnt * jnt_prevPt, p3d_jnt * jntPt) {
  int n;
  p3d_link_between_joint * linkPt;
  p3d_matrix4 inv_pos0;

  if (jntPt->prev_jnt != NULL) { p3d_jnt_deattach_jnt(jntPt, jntPt->prev_jnt); }  // modif Juan (bug correction)
  jntPt->prev_jnt = jnt_prevPt;
  if (jnt_prevPt!=NULL) {
    n = jnt_prevPt->n_next_jnt;
    jnt_prevPt->next_jnt = MY_REALLOC(jnt_prevPt->next_jnt, p3d_jnt *, n, n+1);
    if (jnt_prevPt->next_jnt == NULL) {
      jnt_prevPt->n_next_jnt = 0;
      PrintError(("Not enough memory !!!\n"));
      return;
    }
    jnt_prevPt->next_jnt[n] = jntPt;
    jnt_prevPt->n_next_jnt ++;

    linkPt = MY_ALLOC(p3d_link_between_joint, 1);
    if (linkPt == NULL) {
      PrintError(("Not enough memory !!!\n"));
      return;
    }
    linkPt->prev_jnt = jnt_prevPt;
    linkPt->next_jnt = jntPt;
    p3d_matInvertXform(jnt_prevPt->pos0_obs, inv_pos0);
    p3d_matMultXform(inv_pos0, jntPt->pos0_obs, linkPt->rel_pos);

    n = jntPt->n_link_jnt_owned;
    jntPt->link_jnt_owned_arr = MY_REALLOC(jntPt->link_jnt_owned_arr,
                                           p3d_link_between_joint *, n, n+1);
    if (jntPt->link_jnt_owned_arr == NULL) {
      jntPt->n_link_jnt_owned = 0;
      PrintError(("Not enough memory !!!\n"));
      return;
    }
    jntPt->link_jnt_owned_arr[n] = linkPt;
    jntPt->n_link_jnt_owned ++;

    n = jntPt->n_link_jnt;
    jntPt->link_jnt_arr = MY_REALLOC(jntPt->link_jnt_arr,
                                     p3d_link_between_joint *, n, n+1);
    if (jntPt->link_jnt_arr == NULL) {
      jntPt->n_link_jnt = 0;
      PrintError(("Not enough memory !!!\n"));
      return;
    }
    jntPt->link_jnt_arr[n] = linkPt;
    jntPt->n_link_jnt ++;

    n = jnt_prevPt->n_link_jnt;
    jnt_prevPt->link_jnt_arr = MY_REALLOC(jnt_prevPt->link_jnt_arr,
                                          p3d_link_between_joint *, n, n+1);
    if (jnt_prevPt->link_jnt_arr == NULL) {
      jnt_prevPt->n_link_jnt = 0;
      PrintError(("Not enough memory !!!\n"));
      return;
    }
    jnt_prevPt->link_jnt_arr[n] = linkPt;
    jnt_prevPt->n_link_jnt ++;
  }
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Attach a joint to an other but store the data only in the
 *        first joint.
 *
 * \param jnt1Pt:    the joint that store the link
 * \param jnt2Pt:    the joint which is attached to \a jnt1Pt
 * \param rel_pos:   the relative position between the previous and the
 *                   next joint
 * \param flag_prev: TRUE if \a jnt2Pt is before \a jnt1Pt.
 *
 * \note Create a link_between_joint.
 */
void p3d_jnt_attachment(p3d_jnt * jnt1Pt, p3d_jnt * jnt2Pt,
                        p3d_matrix4 rel_pos, int flag_prev) {
  int n;
  p3d_link_between_joint * linkPt;

  linkPt = MY_ALLOC(p3d_link_between_joint, 1);
  if (linkPt == NULL) {
    PrintError(("Not enough memory !!!\n"));
    return;
  }
  p3d_mat4Copy(rel_pos, linkPt->rel_pos);
  if (flag_prev) {
    linkPt->prev_jnt = jnt2Pt;
    linkPt->next_jnt = jnt1Pt;
    jnt1Pt->prev_jnt = jnt2Pt;
  } else {
    linkPt->prev_jnt = jnt1Pt;
    linkPt->next_jnt = jnt2Pt;
    n = jnt1Pt->n_next_jnt;
    jnt1Pt->next_jnt = MY_REALLOC(jnt1Pt->next_jnt, p3d_jnt *, n, n+1);
    if (jnt1Pt->next_jnt == NULL) {
      jnt1Pt->n_next_jnt = 0;
      PrintError(("Not enough memory !!!\n"));
      return;
    }
    jnt1Pt->next_jnt[n] = jnt2Pt;
    jnt1Pt->n_next_jnt ++;
  }

  n = jnt1Pt->n_link_jnt_owned;
  jnt1Pt->link_jnt_owned_arr = MY_REALLOC(jnt1Pt->link_jnt_owned_arr,
                                          p3d_link_between_joint *, n, n+1);
  if (jnt1Pt->link_jnt_owned_arr == NULL) {
    jnt1Pt->n_link_jnt_owned = 0;
    PrintError(("Not enough memory !!!\n"));
    return;
  }
  jnt1Pt->link_jnt_owned_arr[n] = linkPt;
  jnt1Pt->n_link_jnt_owned ++;

  n = jnt1Pt->n_link_jnt;
  jnt1Pt->link_jnt_arr = MY_REALLOC(jnt1Pt->link_jnt_arr,
                                    p3d_link_between_joint *, n, n+1);
  if (jnt1Pt->link_jnt_arr == NULL) {
    jnt1Pt->n_link_jnt = 0;
    PrintError(("Not enough memory !!!\n"));
    return;
  }
  jnt1Pt->link_jnt_arr[n] = linkPt;
  jnt1Pt->n_link_jnt ++;

  n = jnt2Pt->n_link_jnt;
  jnt2Pt->link_jnt_arr = MY_REALLOC(jnt2Pt->link_jnt_arr,
                                    p3d_link_between_joint *, n, n+1);
  if (jnt2Pt->link_jnt_arr == NULL) {
    jnt2Pt->n_link_jnt = 0;
    PrintError(("Not enough memory !!!\n"));
    return;
  }
  jnt2Pt->link_jnt_arr[n] = linkPt;
  jnt2Pt->n_link_jnt ++;
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Destroy a link between two joints in an array.
 *
 * \param list_link:      The array of links between joints.
 * \param size_list_link: The size of \a list_link.
 * \param jntPt1:         The first joint.
 * \param jntPt2:         The second joint.
 *
 * \retval list_link:      The new array of links between joints.
 * \retval size_list_link: The new size of \a list_link.
 *
 * \return The link found between \a jntPt1 and \a jntPt2.
 *
 * \note Do not destroy the link, just the pointer in the arry.
 */
static p3d_link_between_joint * s_p3d_jnt_suppress_link(
  p3d_link_between_joint *** list_link,
  int * size_list_link, p3d_jnt * jntPt1,
  p3d_jnt * jntPt2) {
  int i_found, i;
  int test = FALSE;
  p3d_link_between_joint * linkPt = NULL;
  p3d_link_between_joint ** new_link_arr;

  for (i_found = 0; i_found<*size_list_link; i_found++) {
    if ((((*list_link)[i_found]->prev_jnt == jntPt1) &&
         ((*list_link)[i_found]->next_jnt == jntPt2)) ||
        (((*list_link)[i_found]->prev_jnt == jntPt2) &&
         ((*list_link)[i_found]->next_jnt == jntPt1))) {
      linkPt = (*list_link)[i_found];
      test = TRUE;
      break;
    }
  }
  if (!test) { return NULL; }
  new_link_arr = MY_ALLOC(p3d_link_between_joint *, (*size_list_link)-1);
  if (((*size_list_link)>1) && (new_link_arr == NULL)) {
    PrintError(("Not enough memory !!!\n"));
    return NULL;
  }
  for(i=0; i<i_found; i++) { new_link_arr[i] = (*list_link)[i]; }
  for(i=i_found+1; i<(*size_list_link); i++) { new_link_arr[i-1] = (*list_link)[i]; }
  MY_FREE((*list_link), p3d_link_between_joint *, (*size_list_link));
  (*list_link) = new_link_arr;
  (*size_list_link) --;
  return linkPt;
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Deattach a joint to an other.
 *
 * \param jntPt1:  the first joint
 * \param jntPt2:  the second joint
 *
 * \note The order between \a jntPt1 and jntPt is not important.
 * \note Destroy a ::link_between_joint only if it is owned by one
 *       of the joints.
 */
void p3d_jnt_deattach_jnt(p3d_jnt * jntPt1, p3d_jnt * jntPt2) {
  p3d_link_between_joint * linkPt;
  p3d_link_between_joint * tmp_linkPt;
  p3d_jnt ** new_next_arr;
  int i, j;

  s_p3d_jnt_suppress_link(&(jntPt1->link_jnt_arr), &(jntPt1->n_link_jnt),
                          jntPt1, jntPt2);
  s_p3d_jnt_suppress_link(&(jntPt2->link_jnt_arr), &(jntPt2->n_link_jnt),
                          jntPt1, jntPt2);
  linkPt = s_p3d_jnt_suppress_link(&(jntPt1->link_jnt_owned_arr),
                                   &(jntPt1->n_link_jnt_owned), jntPt1,jntPt2);
  tmp_linkPt = s_p3d_jnt_suppress_link(&(jntPt2->link_jnt_owned_arr),
                                       &(jntPt2->n_link_jnt_owned),
                                       jntPt1, jntPt2);
  if (linkPt == NULL) { linkPt = tmp_linkPt; }
  if (linkPt != NULL) { MY_FREE(linkPt, p3d_link_between_joint, 1); }
  if (jntPt1->prev_jnt == jntPt2) { jntPt1->prev_jnt = NULL; } else {
    for (i=0; i<jntPt1->n_next_jnt; i++) {
      if (jntPt1->next_jnt[i] == jntPt2) { break; }
    }
    if (i<jntPt1->n_next_jnt) {
      new_next_arr = MY_ALLOC(p3d_jnt *, jntPt1->n_next_jnt-1);
      if ((new_next_arr == NULL) && (jntPt1->n_next_jnt>1)) {
        PrintError(("Not enough memory !!!\n"));
        return;
      }
      for(j=0; j<i; j++) { new_next_arr[j] = jntPt1->next_jnt[j]; }
      for(j=i+1; j<jntPt1->n_next_jnt; j++) { new_next_arr[j-1] = jntPt1->next_jnt[j]; }
      MY_FREE(jntPt1->next_jnt, p3d_jnt *, jntPt1->n_next_jnt);
      jntPt1->next_jnt = new_next_arr;
      jntPt1->n_next_jnt --;
    }
  }
  if (jntPt2->prev_jnt == jntPt1) { jntPt2->prev_jnt = NULL; } else {
    for (i=0; i<jntPt2->n_next_jnt; i++) {
      if (jntPt2->next_jnt[i] == jntPt1) { break; }
    }
    if (i<jntPt2->n_next_jnt) {
      new_next_arr = MY_ALLOC(p3d_jnt *, jntPt2->n_next_jnt-1);
      if ((new_next_arr == NULL) && (jntPt2->n_next_jnt>1)) {
        PrintError(("Not enough memory !!!\n"));
        return;
      }
      for(j=0; j<i; j++) { new_next_arr[j] = jntPt2->next_jnt[j]; }
      for(j=i+1; j<jntPt2->n_next_jnt; j++) { new_next_arr[j-1] = jntPt2->next_jnt[j]; }
      MY_FREE(jntPt2->next_jnt, p3d_jnt *, jntPt2->n_next_jnt);
      jntPt2->next_jnt = new_next_arr;
      jntPt2->n_next_jnt --;
    }
  }
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Destroy a joint.
 *
 * \param  jntPt: the joint.
 *
 * \note Release the memory and destroy all the links with this joint.
 *
 * \warning Do not change the robot structure.
 */
void p3d_jnt_destroy(p3d_jnt * jntPt) {
  p3d_link_between_joint * linkPt;

  while(jntPt->n_link_jnt>0) {
    linkPt = jntPt->link_jnt_arr[0];
    p3d_jnt_deattach_jnt(linkPt->prev_jnt, linkPt->next_jnt);
  }
  p3d_jnt_set_object(jntPt, NULL);
  MY_FREE(jntPt, p3d_jnt, 1);
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Indicate whether or not there is an object on or after the joint
 *
 * Note: the object could be pur graphic. To test also pur graphic
 *       object use p3d_col_object_is_pure_graphic().
 *
 *  \param jntPt:  the joint
 *
 *  \retval TRUE if there is an object / FALSE else
 */

int p3d_jnt_object_after_joint(p3d_jnt * jntPt) {
  int i;

  if (p3d_jnt_is_with_object(jntPt)) { return TRUE; }
  for(i=0; i<jntPt->n_next_jnt; i++) {
    if (p3d_jnt_object_after_joint(jntPt->next_jnt[i])) { return TRUE; }
  }
  return FALSE;
}
//start path deform
/*!
 *  \brief Compute the distance and the speed that the joint could reach for
 *          a fixed sphere.
 *
 *  From the bounds of the environement and based on the maximal speed,
 *  computes an interval (~radius) for  which all the points
 *  of the joint move by less than the distance given as input.
 *
 *  Note: The joint looks directly in the robot configurations
 *        the degree of freedom that it needs.
 *
 * \param  prev_data:   speed of the previous joint
 * \param  jntPt:       the joint
 * \param  distance:    the  distance for body linked to the
 *                      joint to the nearest obstacle
 *
 * \retval data:        speed of the joint
 *         distance:    the distance that the joint couldn't cross
 *         reach_param: the new maximal range parameter
 *                      that could be reach
 */
void p3d_jnt_stay_within_sphere(
  p3d_stay_within_dist_data * prev_data,
  p3d_jnt * jntPt,
  p3d_stay_within_dist_data * data,
  double * distance,
  double * reach_param) {
  switch(jntPt->type) {
    case P3D_BASE:
      p3d_jnt_base_stay_within_sphere(prev_data, jntPt, data, distance,
                                      reach_param);
      break;
    case P3D_FREEFLYER:
      p3d_jnt_freeflyer_stay_within_sphere(prev_data, jntPt, data, distance,
                                           reach_param);
      break;
    case P3D_PLAN:
      p3d_jnt_plan_stay_within_sphere(prev_data, jntPt, data, distance,
                                      reach_param);
      break;
    case P3D_KNEE:
      p3d_jnt_knee_stay_within_sphere(prev_data, jntPt, data, distance,
                                      reach_param);
      break;
    case P3D_ROTATE:
      p3d_jnt_rotate_stay_within_sphere(prev_data, jntPt, data, distance,
                                        reach_param);
      break;
    case P3D_TRANSLATE:
      p3d_jnt_translate_stay_within_sphere(prev_data, jntPt, data, distance,
                                           reach_param);
      break;
    case P3D_FIXED:
      p3d_jnt_fixed_stay_within_sphere(prev_data, jntPt, data, distance,
                                       reach_param);
      break;
  }
}
//end path deform
