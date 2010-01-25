/****************************************************************************/
/*!
 *  \file p3d_col_env.c
 *
 *    \brief Management of collisions in the environment.
 *
 *    These functions allow the selection of collision tested.
 *    They could manage sevral context of collisions.
 */
/****************************************************************************/


#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Collision-pkg.h"


/***************************************************************************
 ***************************************************************************
 * Variables and structure for the management of the collision context.
 */

/*! \ingroup collision_context_env_group
 *  \brief Struture that store sevral context of collisions for one robot.
 *  \internal
 */
typedef struct col_env_robot_context {

  /*! \brief Collision context for all collision.
   *
   *  There are the auto-collision, collision with other robots and
   *  collision with the environment. This is selected by ::P3D_COL_ROBOT_ALL.
   *  \internal
   */
  p3d_collision_pair * col_robot_all;

  /*! \brief Collision context for all collision with the static environment.
   *
   *  There are only the collision with the static environment.
   *  This is selected by ::P3D_COL_ROBOT_ENV.
   *  \internal
   */
  p3d_collision_pair * col_robot_env;

  /*! \brief Collision context for all collision without autocol.
   *
   *  There are the collision with other robots and collision with the
   *  environment. This is selected by ::P3D_COL_ROBOT_ALL_WITHOUT_AUTOCOL.
   *  \internal
   */
  p3d_collision_pair * col_robot_all_without_autocol;

  /*! \brief Collision context for all collision without other robots.
   *
   *  There are the auto-collision and collision with the static
   *  environment. But the autocollision distance is not computed.
   *  This is selected by ::P3D_COL_ROBOT_ALL_WITHOUT_OTHER.
   *  \internal
   */
  p3d_collision_pair * col_robot_all_without_other;

  /*! \brief Collision context for only auto-collision.
   *
   *  There are the auto-collision. This is selected by
   *  ::P3D_COL_ROBOT_AUTOCOL.
   *  \internal
   */
  p3d_collision_pair * col_robot_autocol;

} p3d_col_env_robot_context;


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_context_env_group
 *  \brief Struture that store sevral context of collisions for one robot.
 *  \internal
 */
typedef struct col_env_context {

  /*! \brief The number of robot.
   *  \internal
   */
  int nb_robot;

  /*! \brief Collision context for all collisions.
   *
   *  There are all the collisions between robots or with the static
   *  environment. This is selected by ::P3D_COL_ENV_ALL.
   *  \internal
   */
  p3d_collision_pair * col_env_all;

  /*! \brief Collision context with no collisions.
   *
   *  In this context all the collisions are deactivated.
   *  This is selected by ::P3D_COL_ENV_NONE.
   *  \internal
   */
  p3d_collision_pair * col_env_none;

  /*! \brief Collision context for collision for one robots.
   *
   *  There are sevral context for one robot.
   *  \internal
   */
  p3d_col_env_robot_context * col_env_robot;
} p3d_col_env_context;


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_context_env_group
 *  \brief Structure to store the data of each environment
 *  \internal
 */
typedef struct col_env_context_env {
  /*! \brief Number of the environment */
  int p3d_env_id;

  /*! \brief Store the current collision context
   *  (::cur_col_env_context)
   */
  p3d_col_env_context * cur_col_env_context;

} p3d_col_env_context_env;


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_context_env_group
 *  \brief Array to store the collision context data for each environment.
 *  \internal
 */
static p3d_col_env_context_env * col_env_context_envs = NULL;

/*! \ingroup collision_context_env_group
 *  \brief Number of environment stored in ::col_env_context_envs.
 *  \internal
 */
static int nof_col_env_context_envs = 0;

/*! \ingroup collision_context_env_group
 *  \brief Index of the current environment used in ::col_env_context_envs.
 *  \internal
 */
static int index_cur_col_env_context_envs = -1;

/*! \ingroup collision_context_env_group
 *  \brief Current data for the collision context.
 *  \internal
 */
static p3d_col_env_context * cur_col_env_context = NULL;



/***************************************************************************
 ***************************************************************************
 * Management of the collision context.
 */

/*--------------------------------------------------------------------------*/
/*! \brief Return TRUE if it is possible to use the environment
 *         collision context.
 *
 *  \return TRUE if it is possible to use the environment
 *          collision context, FALSE otherwise.
 *
 *  \note If the collision context is note create, create it.
 *
 *  \internal
 */
int s_p3d_col_env_active_context(void) {
  int i;

  if (cur_col_env_context != NULL) {
    return TRUE;
  }
  if (p3d_col_get_mode() != p3d_col_mode_kcd) {
    return FALSE;  /* Could not use the collision context */
  }
  cur_col_env_context = MY_ALLOC(p3d_col_env_context, 1);
  if (cur_col_env_context == NULL) {
    PrintError(("Not enough memory !!!\n"));
    return FALSE;
  }
  cur_col_env_context->nb_robot      = p3d_get_desc_number(P3D_ROBOT);
  cur_col_env_context->col_env_all   = p3d_col_pair_create();
  cur_col_env_context->col_env_none  = p3d_col_pair_create();
  cur_col_env_context->col_env_robot = MY_ALLOC(p3d_col_env_robot_context,
                                       cur_col_env_context->nb_robot);
  if ((cur_col_env_context->col_env_all   == NULL) ||
      (cur_col_env_context->col_env_none  == NULL) ||
      (cur_col_env_context->col_env_robot == NULL)) {
    PrintError(("Not enough memory !!!\n"));
    MY_FREE(cur_col_env_context, p3d_col_env_context, 1);
    cur_col_env_context = NULL;
    return FALSE;
  }
  p3d_col_pair_push_and_put(cur_col_env_context->col_env_all);
  for (i = 0; i < cur_col_env_context->nb_robot; i++) {
    cur_col_env_context->col_env_robot[i].col_robot_all
    = p3d_col_pair_create();
    cur_col_env_context->col_env_robot[i].col_robot_env
    = p3d_col_pair_create();
    cur_col_env_context->col_env_robot[i].col_robot_all_without_autocol
    = p3d_col_pair_create();
    cur_col_env_context->col_env_robot[i].col_robot_all_without_other
    = p3d_col_pair_create();
    cur_col_env_context->col_env_robot[i].col_robot_autocol
    = p3d_col_pair_create();
    if ((cur_col_env_context->col_env_robot[i].col_robot_all == NULL) ||
        (cur_col_env_context->col_env_robot[i].col_robot_env == NULL) ||
        (cur_col_env_context->col_env_robot[i].
         col_robot_all_without_autocol == NULL) ||
        (cur_col_env_context->col_env_robot[i].
         col_robot_all_without_other == NULL) ||
        (cur_col_env_context->col_env_robot[i].col_robot_autocol == NULL)) {
      PrintError(("Not enough memory !!!\n"));
      MY_FREE(cur_col_env_context, p3d_col_env_context, 1);
      cur_col_env_context = NULL;
      return FALSE;
    }
  }
  return TRUE;
}


/*--------------------------------------------------------------------------*/
/*! \brief Destroy the current collision context.
 *
 *  \internal
 */
static void s_p3d_col_env_context_clear(void) {
  int i;

  if (cur_col_env_context != NULL) {
    for (i = 0; i < cur_col_env_context->nb_robot; i++) {
      p3d_col_pair_destroy(cur_col_env_context->col_env_robot[i].
                           col_robot_all);
      p3d_col_pair_destroy(cur_col_env_context->col_env_robot[i].
                           col_robot_env);
      p3d_col_pair_destroy(cur_col_env_context->col_env_robot[i].
                           col_robot_all_without_autocol);
      p3d_col_pair_destroy(cur_col_env_context->col_env_robot[i].
                           col_robot_all_without_other);
      p3d_col_pair_destroy(cur_col_env_context->col_env_robot[i].
                           col_robot_autocol);
    }
    MY_FREE(cur_col_env_context->col_env_robot, p3d_col_env_robot_context,
            cur_col_env_context->nb_robot);
    p3d_col_pair_destroy(cur_col_env_context->col_env_all);
    p3d_col_pair_destroy(cur_col_env_context->col_env_none);
    MY_FREE(cur_col_env_context, p3d_col_env_context, 1);
    cur_col_env_context = NULL;
  }
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_context_env_group
 *  \brief Select a new collision context.
 *
 *  \param  type: The type of context selected. It could be ::P3D_COL_ENV_ALL
 *                or ::P3D_COL_ENV_NONE.
 *
 *  \note Use p3d_col_env_switch_robot() to select a context specific to
 *        one robot.
 *
 *  \warning Do not forget to restore the previous context with
 *           p3d_col_env_restore()
 */
void p3d_col_env_switch_env(p3d_type_col_env_context type) {
  if (s_p3d_col_env_active_context()) {
    switch (type) {
      case P3D_COL_ENV_ALL:
        p3d_col_pair_push_and_put(cur_col_env_context->col_env_all);
        break;
      case P3D_COL_ENV_NONE:
        p3d_col_pair_push_and_put(cur_col_env_context->col_env_none);
        break;
      default:
        PrintWarning(("!!! p3d_col_env_switch_env: type not valid !!!\n"));
    }
  } else {
    switch (type) {
      case P3D_COL_ENV_ALL:
        p3d_col_activate_env();
        break;
      case P3D_COL_ENV_NONE:
        p3d_col_deactivate_all();
        break;
      default:
        PrintWarning(("!!! p3d_col_env_switch_env: type not valid !!!\n"));
    }
  }
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_context_env_group
 *  \brief Select a new collision context.
 *
 *  \param  robotPt: The robot.
 *  \param  type:    The type of context selected. It could be
 *                   ::P3D_COL_ROBOT_ALL, ::P3D_COL_ROBOT_ENV,
 *                   ::P3D_COL_ROBOT_ALL_WITHOUT_AUTOCOL,
 *                   ::P3D_COL_ROBOT_ALL_WITHOUT_OTHER
 *                   or ::P3D_COL_ROBOT_AUTOCOL.
 *
 *  \note Use p3d_col_env_switch_env() to select a context specific to
 *        all robots.
 *
 *  \warning Do not forget to restore the previous context with
 *           p3d_col_env_restore()
 */
void p3d_col_env_switch_robot(p3d_rob * robotPt, p3d_type_col_env_context type) {
  p3d_col_env_robot_context * robot_contPt;

	#if  defined(PQP) && defined(LIGHT_PLANNER)
          if(p3d_col_get_mode()==p3d_col_mode_pqp)
	  {  return;  }
	#endif

  if (s_p3d_col_env_active_context()) {
    if ((robotPt == NULL) || (robotPt->num < 0) ||
        (robotPt->num > cur_col_env_context->nb_robot)) {
      PrintWarning(("!!! p3d_col_env_switch_robot: robot not valid !!!\n"));
    } else {
      robot_contPt = &(cur_col_env_context->col_env_robot[robotPt->num]);
      switch (type) {
        case P3D_COL_ROBOT_ALL:
          p3d_col_pair_push_and_put(robot_contPt->col_robot_all);
          break;
        case P3D_COL_ROBOT_ENV:
          p3d_col_pair_push_and_put(robot_contPt->col_robot_env);
          break;
        case P3D_COL_ROBOT_ALL_WITHOUT_AUTOCOL:
          p3d_col_pair_push_and_put(robot_contPt->col_robot_all_without_autocol);
          break;
        case P3D_COL_ROBOT_ALL_WITHOUT_OTHER:
          p3d_col_pair_push_and_put(robot_contPt->col_robot_all_without_other);
          break;
        case P3D_COL_ROBOT_AUTOCOL:
          p3d_col_pair_push_and_put(robot_contPt->col_robot_autocol);
          break;
        default:
          PrintWarning(("!!! p3d_col_env_switch_robot: type not valid !!!\n"));
      }
    }
  } else {
    switch (type) {
      case P3D_COL_ROBOT_ALL:
        p3d_col_deactivate_all();
        p3d_col_activate_rob_env(robotPt);
        p3d_col_activate_rob(robotPt);
        p3d_col_activate_rob_all_rob(robotPt);
        break;
      case P3D_COL_ROBOT_ENV:
        p3d_col_deactivate_all();
        p3d_col_activate_rob_env(robotPt);
        break;
      case P3D_COL_ROBOT_ALL_WITHOUT_AUTOCOL:
        p3d_col_deactivate_all();
        p3d_col_activate_rob_env(robotPt);
        p3d_col_activate_rob_all_rob(robotPt);
        break;
      case P3D_COL_ROBOT_ALL_WITHOUT_OTHER:
        p3d_col_deactivate_all();
        p3d_col_activate_rob_env(robotPt);
        p3d_col_activate_rob(robotPt);
        break;
      case P3D_COL_ROBOT_AUTOCOL:
        p3d_col_deactivate_all();
        p3d_col_activate_rob(robotPt);
        break;
      default:
        PrintWarning(("!!! p3d_col_env_switch_robot: type not valid !!!\n"));
    }
  }
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_context_env_group
 *  \brief Restore a previous collision context.
 *
 *  \warning This function must be used after a call to
 *           p3d_col_env_switch_env() or to p3d_col_env_switch_robot()
 */
void p3d_col_env_restore(void) {
  if (s_p3d_col_env_active_context()) {
    p3d_col_pair_pop();
  } else {
    p3d_col_activate_env();
  }
}



/***************************************************************************
 ***************************************************************************
 * Management of multi-environment context.
 */


/*--------------------------------------------------------------------------*/
/*! \brief Store the global variable in ::col_env_context_envs.
 *
 *  \internal
 */
static void s_p3d_deconnect_col_env_global_vars(void) {
  if (index_cur_col_env_context_envs >= 0) {
    col_env_context_envs[index_cur_col_env_context_envs].
    cur_col_env_context = cur_col_env_context;
    cur_col_env_context = NULL;
  } else {
    s_p3d_col_env_context_clear();
  }
}


/*--------------------------------------------------------------------------*/
/*! \brief Restore the data from an environment backup.
 *
 *  \param col_env_context_envPt: Data for collision context
 *                                in a specific environment.
 *
 *  \internal
 */
static void s_p3d_connect_col_env_global_vars(p3d_col_env_context_env *
    col_env_context_envPt) {
  cur_col_env_context = col_env_context_envPt->cur_col_env_context;
}


/*--------------------------------------------------------------------------*/
/*! \brief Create a new case to store the new envirenment \a p3d_env_id.
 *
 *  \param p3d_env_id: Number of the environment.
 *
 *  \internal
 */
static void s_p3d_init_col_env_global_vars(int p3d_env_id) {
  col_env_context_envs = MY_REALLOC(col_env_context_envs,
                                    p3d_col_env_context_env,
                                    nof_col_env_context_envs,
                                    nof_col_env_context_envs + 1);
  if (col_env_context_envs == NULL) {
    PrintError(("Not enough memory !!!\n"));
    nof_col_env_context_envs = 0;
    index_cur_col_env_context_envs = -1;
  } else {
    index_cur_col_env_context_envs = nof_col_env_context_envs;
    nof_col_env_context_envs ++;
    col_env_context_envs[index_cur_col_env_context_envs].
    p3d_env_id = p3d_env_id;
    col_env_context_envs[index_cur_col_env_context_envs].
    cur_col_env_context = NULL;
  }
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_context_env_group
 *  \brief Initialize collision context to start a new environment.
 */
void p3d_col_env_start(void) {
  int p3d_env_id;
  int i;

  s_p3d_deconnect_col_env_global_vars();
  p3d_env_id = XYZ_ENV->num;
  for (i = 0; i < nof_col_env_context_envs; i++) {
    if (col_env_context_envs[i].p3d_env_id == p3d_env_id) {
      break;
    }
  }
  if ((nof_col_env_context_envs > i) &&
      (col_env_context_envs[i].p3d_env_id == p3d_env_id)) {
    /* Already stored */
    s_p3d_connect_col_env_global_vars(&(col_env_context_envs[i]));
  } else {
    /* Create this new environment */
    s_p3d_init_col_env_global_vars(p3d_env_id);
  }
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_context_env_group
 *  \brief Release the memory for collision context in all environment.
 */
void p3d_col_env_stop(void) {
  int i;

  s_p3d_deconnect_col_env_global_vars();
  for (i = 0; i < nof_col_env_context_envs; i++) {
    s_p3d_connect_col_env_global_vars(&(col_env_context_envs[i]));
    s_p3d_col_env_context_clear();
  }
  MY_FREE(col_env_context_envs, p3d_col_env_context_env,
          nof_col_env_context_envs);
  nof_col_env_context_envs = 0;
  index_cur_col_env_context_envs = -1;
  col_env_context_envs = NULL;
}



/***************************************************************************
 ***************************************************************************
 * Activation/Deactivation of the collision context.
 */

/*--------------------------------------------------------------------------*/
/*! \ingroup collision_context_env_group
 *  \brief Function to activate the collision between a pair of objets
 *         in all the context concerned.
 *
 *  \param  obj1: The first object
 *  \param  obj2: The second object
 *
 *  \warning The management of the context must be valid.
 *  \warning The objects must be valid.
 *
 *  \internal
 */
static void s_p3d_col_env_activate_obj_obj(p3d_obj *obj1, p3d_obj *obj2) {
  int i_rob, i_rob2;

  if ((obj1->jnt != NULL) && (obj1->jnt->rob != NULL)) {
    i_rob = obj1->jnt->rob->num;
    if ((obj2->jnt != NULL) && (obj2->jnt->rob != NULL)) {
      i_rob2 = obj2->jnt->rob->num;
      p3d_col_pair_activate_pair(cur_col_env_context->col_env_all, obj1, obj2);
      if ((i_rob >= 0) || (i_rob < cur_col_env_context->nb_robot) ||
          (i_rob2 >= 0) || (i_rob2 < cur_col_env_context->nb_robot)) {
        if (i_rob != i_rob2) {
          p3d_col_pair_activate_pair(cur_col_env_context->col_env_robot[i_rob].col_robot_all, obj1, obj2);
          p3d_col_pair_activate_pair(cur_col_env_context->col_env_robot[i_rob2].col_robot_all, obj1, obj2);
          p3d_col_pair_activate_pair(cur_col_env_context->col_env_robot[i_rob].col_robot_all_without_autocol, obj1, obj2);
          p3d_col_pair_activate_pair(cur_col_env_context->col_env_robot[i_rob2].col_robot_all_without_autocol, obj1, obj2);
        } else {
          p3d_col_pair_activate_pair(cur_col_env_context->col_env_robot[i_rob].col_robot_all, obj1, obj2);
          p3d_col_pair_activate_pair(cur_col_env_context->col_env_robot[i_rob].col_robot_autocol, obj1, obj2);
          p3d_col_pair_activate_pair(cur_col_env_context->col_env_robot[i_rob].col_robot_all_without_other,obj1, obj2);
        }
      }
    } else {
      p3d_col_pair_activate_env(cur_col_env_context->col_env_all, obj1);
      if ((i_rob >= 0) || (i_rob < cur_col_env_context->nb_robot)) {
        p3d_col_pair_activate_env(cur_col_env_context->col_env_robot[i_rob].col_robot_all, obj1);
        p3d_col_pair_activate_env(cur_col_env_context->col_env_robot[i_rob].col_robot_env, obj1);
        p3d_col_pair_activate_env(cur_col_env_context->col_env_robot[i_rob].col_robot_all_without_autocol, obj1);
        p3d_col_pair_activate_env(cur_col_env_context->col_env_robot[i_rob].col_robot_all_without_other, obj1);
      }
    }
  } else if ((obj2->jnt != NULL) && (obj2->jnt->rob != NULL)) {
    p3d_col_pair_activate_env(cur_col_env_context->col_env_all, obj2);
    i_rob = obj2->jnt->rob->num;
    if ((i_rob >= 0) || (i_rob < cur_col_env_context->nb_robot)) {
      p3d_col_pair_activate_env(cur_col_env_context->col_env_robot[i_rob].col_robot_all, obj2);
      p3d_col_pair_activate_env(cur_col_env_context->col_env_robot[i_rob].col_robot_env, obj2);
      p3d_col_pair_activate_env(cur_col_env_context->col_env_robot[i_rob].col_robot_all_without_autocol, obj2);
      p3d_col_pair_activate_env(cur_col_env_context->col_env_robot[i_rob].col_robot_all_without_other, obj2);
    }
  }
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_context_env_group
 *  \brief Function to deactivate the collision between a pair of objets
 *         in all the context concerned.
 *
 *  \param  obj: The object
 *
 *  \warning The management of the context must be valid.
 *  \warning The objects must be valid.
 *
 *  \internal
 */
static void s_p3d_col_env_deactivate_obj_obj(p3d_obj *obj1, p3d_obj *obj2) {
  int i_rob, i_rob2;

  if ((obj1->jnt != NULL) && (obj1->jnt->rob != NULL)) {
    i_rob = obj1->jnt->rob->num;
    if ((obj2->jnt != NULL) && (obj2->jnt->rob != NULL)) {
      i_rob2 = obj2->jnt->rob->num;
      p3d_col_pair_deactivate_pair(cur_col_env_context->col_env_all, obj1, obj2);
      if ((i_rob >= 0) || (i_rob < cur_col_env_context->nb_robot) ||
          (i_rob2 >= 0) || (i_rob2 < cur_col_env_context->nb_robot)) {
        if (i_rob != i_rob2) {
          p3d_col_pair_deactivate_pair(cur_col_env_context->
                                       col_env_robot[i_rob].col_robot_all,
                                       obj1, obj2);
          p3d_col_pair_deactivate_pair(cur_col_env_context->col_env_robot
                                       [i_rob2].col_robot_all, obj1, obj2);
          p3d_col_pair_deactivate_pair(cur_col_env_context->col_env_robot
                                       [i_rob].col_robot_all_without_autocol,
                                       obj1, obj2);
          p3d_col_pair_deactivate_pair(cur_col_env_context->col_env_robot
                                       [i_rob2].col_robot_all_without_autocol,
                                       obj1, obj2);
        } else {
          p3d_col_pair_deactivate_pair(cur_col_env_context->col_env_robot
                                       [i_rob].col_robot_autocol, obj1, obj2);
          p3d_col_pair_deactivate_col_pair(cur_col_env_context->
                                           col_env_robot[i_rob].col_robot_all,
                                           obj1, obj2);
          p3d_col_pair_deactivate_col_pair(cur_col_env_context->col_env_robot
                                           [i_rob].col_robot_all_without_other,
                                           obj1, obj2);
        }
      }
    } else {
      p3d_col_pair_deactivate_env(cur_col_env_context->col_env_all, obj1);
      if ((i_rob >= 0) || (i_rob < cur_col_env_context->nb_robot)) {
        p3d_col_pair_deactivate_env(cur_col_env_context->col_env_robot[i_rob].
                                    col_robot_all, obj1);
        p3d_col_pair_deactivate_env(cur_col_env_context->col_env_robot[i_rob].
                                    col_robot_env, obj1);
        p3d_col_pair_deactivate_env(cur_col_env_context->col_env_robot[i_rob].
                                    col_robot_all_without_autocol, obj1);
        p3d_col_pair_deactivate_env(cur_col_env_context->col_env_robot[i_rob].
                                    col_robot_all_without_other, obj1);
      }
    }
  } else if ((obj2->jnt != NULL) && (obj2->jnt->rob != NULL)) {
    p3d_col_pair_deactivate_env(cur_col_env_context->col_env_all, obj2);
    i_rob = obj2->jnt->rob->num;
    if ((i_rob >= 0) || (i_rob < cur_col_env_context->nb_robot)) {
      p3d_col_pair_deactivate_env(cur_col_env_context->col_env_robot[i_rob].
                                  col_robot_all, obj2);
      p3d_col_pair_deactivate_env(cur_col_env_context->col_env_robot[i_rob].
                                  col_robot_env, obj2);
      p3d_col_pair_deactivate_env(cur_col_env_context->col_env_robot[i_rob].
                                  col_robot_all_without_autocol, obj2);
      p3d_col_pair_deactivate_env(cur_col_env_context->col_env_robot[i_rob].
                                  col_robot_all_without_other, obj2);
    }
  }
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_context_env_group
 *  \brief Function to activate the collision between one objet and
 *         the environment in all the context concerned.
 *
 *  \param  obj:  The object (movable object)
 *
 *  \warning The management of the context must be valid.
 *  \warning The object must be valid.
 *
 *  \internal
 */
static void s_p3d_col_env_activate_obj_env(p3d_obj *obj) {
  int i_rob;

  p3d_col_pair_activate_env(cur_col_env_context->col_env_all, obj);
  if ((obj->jnt != NULL) && (obj->jnt->rob != NULL)) {
    i_rob = obj->jnt->rob->num;
    if ((i_rob >= 0) || (i_rob < cur_col_env_context->nb_robot)) {
      p3d_col_pair_activate_env(cur_col_env_context->col_env_robot[i_rob].
                                col_robot_all, obj);
      p3d_col_pair_activate_env(cur_col_env_context->col_env_robot[i_rob].
                                col_robot_env, obj);
      p3d_col_pair_activate_env(cur_col_env_context->col_env_robot[i_rob].
                                col_robot_all_without_autocol, obj);
      p3d_col_pair_activate_env(cur_col_env_context->col_env_robot[i_rob].
                                col_robot_all_without_other, obj);
    }
  }
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_context_env_group
 *  \brief Function to deactivate the collision between one objet and
 *         the environment in all the context concerned.
 *
 *  \param  obj:  The object (movable object)
 *
 *  \warning The management of the context must be valid.
 *  \warning The object must be valid.
 *
 *  \internal
 */
static void s_p3d_col_env_deactivate_obj_env(p3d_obj *obj) {
  int i_rob;

  p3d_col_pair_deactivate_env(cur_col_env_context->col_env_all, obj);
  if ((obj->jnt != NULL) && (obj->jnt->rob != NULL)) {
    i_rob = obj->jnt->rob->num;
    if ((i_rob >= 0) || (i_rob < cur_col_env_context->nb_robot)) {
      p3d_col_pair_deactivate_env(cur_col_env_context->col_env_robot[i_rob].
                                  col_robot_all, obj);
      p3d_col_pair_deactivate_env(cur_col_env_context->col_env_robot[i_rob].
                                  col_robot_env, obj);
      p3d_col_pair_deactivate_env(cur_col_env_context->col_env_robot[i_rob].
                                  col_robot_all_without_autocol, obj);
      p3d_col_pair_deactivate_env(cur_col_env_context->col_env_robot[i_rob].
                                  col_robot_all_without_other, obj);
    }
  }
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_context_env_group
 *  \brief Function to deactivate the collision between all objects pairs
 *         in all the context concerned.
 *
 *  \param  obj:  The object (movable object)
 *
 *  \warning The management of the context must be valid.
 *
 *  \internal
 */
static void s_p3d_col_env_deactivate_all(void) {
  int i;

  p3d_col_pair_deactivate_all(cur_col_env_context->col_env_all);
  for (i = 0; i < cur_col_env_context->nb_robot; i++) {
    p3d_col_pair_deactivate_all(cur_col_env_context->col_env_robot[i].
                                col_robot_all);
    p3d_col_pair_deactivate_all(cur_col_env_context->col_env_robot[i].
                                col_robot_env);
    p3d_col_pair_deactivate_all(cur_col_env_context->col_env_robot[i].
                                col_robot_all_without_autocol);
    p3d_col_pair_deactivate_all(cur_col_env_context->col_env_robot[i].
                                col_robot_all_without_other);
    p3d_col_pair_deactivate_all(cur_col_env_context->col_env_robot[i].
                                col_robot_autocol);
  }
}



/***************************************************************************
 ***************************************************************************
 * Activation/Deactivation of the object.
 */

/*--------------------------------------------------------------------------*/
/*! \ingroup collision_context_env_group
 *  \brief Function to activate the collision between a pair of objets.
 *
 *  \param  obj1: The first object
 *  \param  obj2: The second object
 *
 *  \note   This function set the activate pair in all the context of
 *          the environment.
 */
void p3d_col_activate_obj_obj(p3d_obj *obj1, p3d_obj *obj2) {
  int i, j, np1, np2;

  if ((!p3d_col_object_is_pure_graphic(obj1)) &&
      (!p3d_col_object_is_pure_graphic(obj2))) {

    if (s_p3d_col_env_active_context()) {
      s_p3d_col_env_activate_obj_obj(obj1, obj2);
    } else {
      if (COLLISION_BY_OBJECT) {
        p3d_col_activate_pair_of_objects(obj1, obj2);
      } else {
        p3d_BB_activate_pair((p3d_BB_handle *)NULL, obj1, obj2);
        np1 = obj1->np;
        np2 = obj2->np;
        for (i = 0;i < np1;i++) {
          for (j = 0;j < np2;j++) {
            /* CAROLE : on n'active la paire que si les deux polyhedres
               ne sont pas uniquement graphiques */
            if ((obj1->pol[i]->TYPE != P3D_GRAPHIC) &&
                (obj2->pol[j]->TYPE != P3D_GRAPHIC)) {
              p3d_col_activate_pair(obj1->pol[i], obj2->pol[j]);
            }
          }
        }
      }
    }
  }
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_context_env_group
 *  \brief Function to activate the collision between one objet
 *         and the environment.
 *
 *  \param  obj: The object
 *
 *  \note   This function set the activate pair in all the context of
 *          the environment.
 */
void p3d_col_activate_obj_env(p3d_obj *obj) {
  int i, np;

  if (!p3d_col_object_is_pure_graphic(obj)) {

    if (s_p3d_col_env_active_context()) {
      s_p3d_col_env_activate_obj_env(obj);
    } else {
      if (COLLISION_BY_OBJECT) {
        p3d_col_activate_object_to_env(obj);
      } else {
        p3d_BB_activate_env((p3d_BB_handle *)NULL, obj);
        np = obj->np;
        for (i = 0;i < np;i++) {
          /* CAROLE : on n'active la paire que si les deux polyhedres
             ne sont pas uniquement graphiques */
          if (obj->pol[i]->TYPE != P3D_GRAPHIC) {
            p3d_col_activate_full(obj->pol[i]);
          }
        }
      }
    }
  }
}


/*********************************************************/
/* Fonction d'activation de toutes les paires d'un robot */
/* et d'un objet d'un environnement                      */
/* In : le robot et l'objet                              */
/* Out :                                                 */
/*********************************************************/
void p3d_col_activate_rob_obj(p3d_rob *rob, p3d_obj *obj) {
  int ib, nb;
  p3d_obj *o;

  nb = rob->no;

  if (!p3d_col_object_is_pure_graphic(obj)) {
    for (ib = 0;ib < nb;ib++) {
      p3d_filter_set_current_active_link(ib);
      o = rob->o[ib];
      if (!p3d_col_object_is_pure_graphic(o)) {
        p3d_col_activate_body_obj(o, obj);
      }
    }
  }
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_context_env_group
 *  \brief Function to activate the collision between one robot
 *         and the environment.
 *
 *  \param  robotPt: The robot
 *
 *  \note   This function set the activate pair in all the context of
 *          the environment.
 */
void p3d_col_activate_rob_env(p3d_rob *robotPt) {
  int i;

  p3d_filter_set_current_robot_nr(robotPt->num);
  for (i = 0; i < robotPt->no; i++) {
    p3d_col_activate_obj_env(robotPt->o[i]);
  }
}


/*--------------------------------------------------------------------------*/
/*! \brief Function to activate the collision between two robots.
 *
 *  \param  rob1: The first robot
 *  \param  rob2: The second robot
 *
 *  \note   This function set the activate pair in all the context of
 *          the environment.
 */
void p3d_col_activate_rob_rob(p3d_rob *rob1, p3d_rob *rob2) {
  int i, j;

  for (i = 0; i < rob1->no; i++) {
    for (j = 0; j < rob2->no; j++) {
      p3d_col_activate_obj_obj(rob1->o[i], rob2->o[j]);
    }
  }
}

/*--------------------------------------------------------------------------*/
/*! \brief Function to activate the collision between a movable object and a robot.
 *
 *  \param  obj: The object
 *  \param  rob: The robot
 *
 *  \note   This function set the activate pair in all the context of
 *          the environment.
 */
void p3d_col_activate_obj_rob(p3d_obj *obj, p3d_rob *rob) {
  int i;
  for (i = 0; i < rob->no; i++) {
    p3d_col_activate_obj_obj(obj, rob->o[i]);
  }
}

/*--------------------------------------------------------------------------*/
/*! \ingroup collision_context_env_group
 *  \brief Function to activate the collision between one robot and the other.
 *
 *  \param  robotPt: The robot
 *
 *  \note   This function set the activate pair in all the context of
 *          the environment.
 */
void p3d_col_activate_rob_all_rob(p3d_rob *robotPt) {
  int r, nr, ir;
  p3d_rob * rob;

  r = p3d_get_desc_curnum(P3D_ROBOT);
  nr = p3d_get_desc_number(P3D_ROBOT);

  /* Pour tous les robots */
  for (ir = 0; ir < nr; ir++) {

    p3d_sel_desc_num(P3D_ROBOT, ir);
    rob = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);

    if (rob != robotPt) {
      p3d_col_activate_rob_rob(robotPt, rob);
    }
  }
  p3d_sel_desc_num(P3D_ROBOT, r);
}
/*--------------------------------------------------------------------------*/
/*! \ingroup collision_context_env_group
 *  \brief Function to activate the collision between one Movable object of the current robot and all the other robots.
 *
 *  \param  obj: The movable object
 *
 *  \note   This function set the activate pair in all the context of
 *          the environment.
 */
void p3d_col_activate_obj_all_rob(p3d_obj *obj) {
  int r, nr, ir;
  p3d_rob * rob, *curRobot;

  r = p3d_get_desc_curnum(P3D_ROBOT);
  curRobot = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  nr = p3d_get_desc_number(P3D_ROBOT);

  /* Pour tous les robots */
  for (ir = 0; ir < nr; ir++) {

    p3d_sel_desc_num(P3D_ROBOT, ir);
    rob = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);

    if (rob != curRobot) {
      p3d_col_activate_obj_rob(obj, rob);
    }
  }
  p3d_sel_desc_num(P3D_ROBOT, r);
}
/*--------------------------------------------------------------------------*/
/*! \ingroup collision_context_env_group
 *  \brief Function to activate the auto-collision for one robot.
 *
 *  \param  rob: The robot
 *
 *  \note   This function set the activate pair in all the context of
 *          the environment.
 */
void p3d_col_activate_rob(p3d_rob *rob) {
  /*** DEBUT MODIF J.PETTRE 06/06/01 : nouvelle methode
       pour autocollision d'un robot ***/
  p3d_autocol_activate_rob(rob);
  /*** FIN MODIF J.PETTRE 06/06/01 : nouvelle methode
       pour autocollision d'un robot ***/
  p3d_col_deactivate_cntrt_pairs();
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_context_env_group
 *  \brief Function to deactivate collision between all objects pair.
 *
 *  \note   This function deactivate and then activate all pairs
 *          in all the context of the environment.
 */
void p3d_col_deactivate_all(void) {
  if (s_p3d_col_env_active_context()) {
    s_p3d_col_env_deactivate_all();
  } else {
    p3d_col_cur_deactivate_all();
  }
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_context_env_group
 *  \brief Function to activate all the collisions.
 *
 *  \note   This function deactivate and then activate all pairs
 *          in all the context of the environment.
 */
void p3d_col_activate_env(void) {
  p3d_col_deactivate_all();
  p3d_col_activate_robots();
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_context_env_group
 *  \brief Function to activate all the collisions.
 *
 *  \note   This function does not deactive previous pair before
 *          the beginnig of the activation.
 */
void p3d_col_activate_robots(void) {
  int r, nr, ir, jr;
  p3d_rob *rob, *other_rob;

  r = p3d_get_desc_curnum(P3D_ROBOT);
  nr = p3d_get_desc_number(P3D_ROBOT);

  /* Pour tous les robots */
  for (ir = 0; ir < nr; ir++) {

    p3d_sel_desc_num(P3D_ROBOT, ir);
    rob = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);

    /* activation du robot par rapport aux obstacles */
    p3d_col_activate_rob_env(rob);

    /* activation des bodies du robot courant */
    p3d_col_activate_rob(rob);

    /* activation du robot par rapport aux autres robots */
    for (jr = ir + 1;jr < nr;jr++) {
      p3d_sel_desc_num(P3D_ROBOT, jr);
      other_rob = (p3d_rob*)p3d_get_desc_curid(P3D_ROBOT);
      p3d_col_activate_rob_rob(rob, other_rob);
    }

  }
  p3d_sel_desc_num(P3D_ROBOT, r);
}


/*--------------------------------------------------------------------------*/
/*! \brief Function to deactivate the collision between a pair of objets.
 *
 *  \param  obj1: The first object
 *  \param  obj2: The second object
 *
 *  \note   This function set the activate pair in all the context of
 *          the environment.
 */
void p3d_col_deactivate_obj_obj(p3d_obj *obj1, p3d_obj *obj2) {
  int i, j, np1, np2;

  if ((!p3d_col_object_is_pure_graphic(obj1)) &&
      (!p3d_col_object_is_pure_graphic(obj2))) {

    if (s_p3d_col_env_active_context()) {
      s_p3d_col_env_deactivate_obj_obj(obj1, obj2);
    } else {
      if (COLLISION_BY_OBJECT) {
        p3d_col_deactivate_pair_of_objects(obj1, obj2);
      } else {
        p3d_BB_deactivate_pair((p3d_BB_handle *)NULL, obj1, obj2);
        np1 = obj1->np;
        np2 = obj2->np;
        for (i = 0;i < np1;i++) {
          for (j = 0;j < np2;j++) {
            /* CAROLE : on ne desactive la paire que si les deux polyhedres
               ne sont pas uniquement graphiques */
            if ((obj1->pol[i]->TYPE != P3D_GRAPHIC) &&
                (obj2->pol[j]->TYPE != P3D_GRAPHIC)) {
              p3d_col_deactivate_pair(obj1->pol[i], obj2->pol[j]);
            }
          }
        }
      }
    }
  }
}


/*--------------------------------------------------------------------------*/
/*! \brief Function to deactivate the collision between one objet
 *         and the environment.
 *
 *  \param  obj: The object
 *
 *  \note   This function set the activate pair in all the context of
 *          the environment.
 */
void p3d_col_deactivate_obj_env(p3d_obj *obj) {
  int i, np;

  if (!p3d_col_object_is_pure_graphic(obj)) {

    if (s_p3d_col_env_active_context()) {
      s_p3d_col_env_deactivate_obj_env(obj);
    } else {
      if (COLLISION_BY_OBJECT) {
        p3d_col_deactivate_object_to_env(obj);
      } else {
        p3d_BB_deactivate_env((p3d_BB_handle *)NULL, obj);
        np = obj->np;
        for (i = 0;i < np;i++) {
          /* CAROLE : on n'active la paire que si les deux polyhedres
             ne sont pas uniquement graphiques */
          if (obj->pol[i]->TYPE != P3D_GRAPHIC) {
            p3d_col_deactivate_full(obj->pol[i]);
          }
        }
      }
    }
  }
}


/*************************************************************/
/* Fonction de desactivation de toutes les paires d'un robot */
/* et d'un objet d'un environnement                          */
/* In : le robot et l'objet                                  */
/* Out :                                                     */
/*************************************************************/
void p3d_col_deactivate_rob_obj(p3d_rob *rob, p3d_obj *obj) {
  int ib, nb;
  p3d_obj *o;

  nb = rob->no;

  for (ib = 0;ib < nb;ib++) {
    o = rob->o[ib];
    p3d_col_deactivate_obj_obj(o, obj);
  }
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_context_env_group
 *  \brief Function to deactivate the collision between one robot
 *         and the environment.
 *
 *  \param  robotPt: The robot
 *
 *  \note   This function set the activate pair in all the context of
 *          the environment.
 */
void p3d_col_deactivate_rob_env(p3d_rob *robotPt) {
  int i;

  for (i = 0; i < robotPt->no; i++) {
    p3d_col_deactivate_obj_env(robotPt->o[i]);
  }
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_context_env_group
 *  \brief Function to deactivate the collision between two robots.
 *
 *  \param  rob1: The first robot
 *  \param  rob2: The second robot
 *
 *  \note   This function set the activate pair in all the context of
 *          the environment.
 */
void p3d_col_deactivate_rob_rob(p3d_rob *rob1, p3d_rob *rob2) {
  int i, j;

  for (i = 0; i < rob1->no; i++) {
    for (j = 0; j < rob2->no; j++) {
      p3d_col_deactivate_obj_obj(rob1->o[i], rob2->o[j]);
    }
  }
}

/*--------------------------------------------------------------------------*/
/*! \ingroup collision_context_env_group
 *  \brief Function to deactivate the collision between a movable object and a robot.
 *
 *  \param  obj: The movable object
 *  \param  rob: The robot
 *
 *  \note   This function set the activate pair in all the context of
 *          the environment.
 */
void p3d_col_deactivate_obj_rob(p3d_obj *obj, p3d_rob *rob) {
  int i;
    for (i = 0; i < rob->no; i++) {
      p3d_col_deactivate_obj_obj(obj, rob->o[i]);
    }
}

/*--------------------------------------------------------------------------*/
/*! \ingroup collision_context_env_group
 *  \brief Function to deactivate all the auto-collision for one robot.
 *
 *  \param  rob: The robot
 *
 *  \note   This function set the activate pair in all the context of
 *          the environment.
 */
void p3d_col_deactivate_rob(p3d_rob *rob) {
  int ib, jb, nb;
  p3d_obj *bod, *bodc;

  nb = rob->no;

  /* Pour tout corps */
  for (ib = 0;ib < nb;ib++) {
    bodc = rob->o[ib];
    /* On active les paires des corps non-adjacents */
    for (jb = ib + 1;jb < nb;jb++) {
      bod = rob->o[jb];
      p3d_col_deactivate_obj_obj(bodc, bod);
    }
  }
}

/*--------------------------------------------------------------------------*/
/*! \ingroup collision_context_env_group
 *  \brief Function to deactivate all the collision between a Movable object and the other robots
 *
 *  \param  obj: The object
 *
 *  \note   This function set the activate pair in all the context of
 *          the environment.
 */
void p3d_col_deactivate_obj_all_rob(p3d_obj *obj) {
  int r, nr, ir;
  p3d_rob * rob, *curRobot;

  r = p3d_get_desc_curnum(P3D_ROBOT);
  curRobot = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  nr = p3d_get_desc_number(P3D_ROBOT);

  /* Pour tous les robots */
  for (ir = 0; ir < nr; ir++) {

    p3d_sel_desc_num(P3D_ROBOT, ir);
    rob = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);

    if (rob != curRobot) {
      p3d_col_deactivate_obj_rob(obj, rob);
    }
  }
  p3d_sel_desc_num(P3D_ROBOT, r);
}