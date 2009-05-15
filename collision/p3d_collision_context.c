/****************************************************************************/
/*!
 *  \file p3d_collision_context.c
 *
 *    \brief Collision context methodes
 *
 *    These functions are the interface of the Collision pair structure 
 *    (::p3d_collision_pair) and to the Collision context
 *    (::p3d_collision_context).
 */
/****************************************************************************/


#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Collision-pkg.h"


/***************************************************************************
 ***************************************************************************
 * Variables for the management of the current collision pair
 */

/*! \ingroup current_collision_pair_group
 *  \brief Structure to store the data of each environment
 *  \internal
 */
typedef struct collision_pair_env {
  /*! \brief Number of the environment */
  int p3d_env_id;

  /*! \brief Store the default collision pair
   *  (::default_current_collision_pair)
   */
  p3d_collision_pair * default_current_collision_pair;

  /*! \brief Store the current stack of collision pair
   *  (::current_collision_pair_stack)
   */
  p3d_collision_pair ** current_collision_pair_stack;

  /*! \brief Store the size of the current stack of collision pair
   *  (::size_current_collision_pair_stack)
   */
  int size_current_collision_pair_stack;

  /*! \brief Store the index of the current collision pair in the stack
   *  (::index_current_collision_pair)
   */
  int index_current_collision_pair;
} p3d_collision_pair_env;

/*! \ingroup current_collision_pair_group
 *  \brief Array to store the data for each environment.
 *  \internal
 */
static p3d_collision_pair_env * collision_pair_envs = NULL;

/*! \ingroup current_collision_pair_group
 *  \brief Number of environment stored in ::collision_pair_envs.
 *  \internal
 */
static int nof_collision_pair_envs = 0;

/*! \ingroup current_collision_pair_group
 *  \brief Index of the current environment used in ::collision_pair_envs.
 *  \internal
 */
static int index_cur_collision_pair_envs = -1;

/*! \ingroup current_collision_pair_group
 *  \brief Default stack size.
 *  \internal
 */
#define COL_PAIR_STACK_MIN_SIZE 10

/*! \ingroup current_collision_pair_group
 *  \brief Collision pairs that can be used when none current pair 
 *  is defined
 *  \internal
 */
static p3d_collision_pair * default_current_collision_pair = NULL;

/*! \ingroup current_collision_pair_group
 *  \brief Stack for the gestion of the current collision pairs.
 *  \internal
 */
static p3d_collision_pair ** current_collision_pair_stack = NULL;

/*! \ingroup current_collision_pair_group
 *  \brief Size of the stack ::current_collision_pair_stack.
 *  \internal
*/
static int size_current_collision_pair_stack = 0;

/*! \ingroup current_collision_pair_group
 *  \brief Index of the current collision pairs in the stack
 * ::current_collision_pair_stack.
 *  \internal
 */
static int index_current_collision_pair = -1;

static void s_p3d_col_pair_init_current(void);

/***************************************************************************
 ***************************************************************************
 * Modification of a collision pairs
 */

/*--------------------------------------------------------------------------*/
/*! \ingroup collision_pair_group
 *
 *  \brief Add the collision test of a movable object
 *         with the environment
 *
 *  \param   pair:    the collision pair.
 *  \param   obj:     the Move3D object. This must be a movable object (body).
 *
 *  \note  This function does not change the distance computation status.
 *  \note  If \a pair == ::NULL then use the current collision pair.
 */
void p3d_col_pair_activate_col_env(p3d_collision_pair * pair,
				   p3d_obj * obj)
{
  if (pair == NULL) 
    { pair = p3d_col_pair_get_cur(); }
  if (!p3d_col_pair_is_test_col_env(pair, obj))
    { kcd_add_collision_object_to_env(pair->kcd_handlePt, obj); }
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_pair_group
 *
 *  \brief Add the distance computation of a movable object
 *         with the environment
 *
 *  \param   pair:    the collision pair.
 *  \param   obj:     the Move3D object. This must be a movable object (body).
 *
 *  \note  This function does not change the collision test status.
 *  \note  If \a pair == ::NULL then use the current collision pair.
 */
void p3d_col_pair_activate_dist_env(p3d_collision_pair * pair,
				    p3d_obj * obj)
{
  if (pair == NULL) 
    { pair = p3d_col_pair_get_cur(); }
  if (!p3d_col_pair_is_calc_dist_env(pair, obj)) {
    p3d_BB_activate_env(pair->BB_handlePt, obj);
    kcd_add_distance_object_to_env(pair->kcd_handlePt, obj);
  }
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_pair_group
 *
 *  \brief Add the test and the distance computation of a movable object
 *         with the environment
 *
 *  \param   pair:    the collision pair.
 *  \param   obj:     the Move3D object. This must be a movable object (body).
 *
 *  \note  If \a pair == ::NULL then use the current collision pair.
 */
void p3d_col_pair_activate_env(p3d_collision_pair * pair,
			       p3d_obj * obj)
{
  if (pair == NULL) 
    { pair = p3d_col_pair_get_cur(); }
  p3d_col_pair_activate_col_env(pair, obj);
  p3d_col_pair_activate_dist_env(pair, obj);
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_pair_group
 *
 *  \brief Add the collision test between two movable objects.
 *
 *  \param   pair:    the collision pair.
 *  \param   obj1:    the first movable object. 
 *  \param   obj2:    the secon movable object. 
 *
 *  \note  This function does not change the distance computation status.
 *  \note  If \a pair == ::NULL then use the current collision pair.
 */
void p3d_col_pair_activate_col_pair(p3d_collision_pair * pair,
				    p3d_obj * obj1, p3d_obj * obj2)
{
  if (pair == NULL) 
    { pair = p3d_col_pair_get_cur(); }
  if (!p3d_col_pair_is_test_col_pair(pair, obj1, obj2))
    { kcd_add_collision_pair_of_objects(pair->kcd_handlePt, obj1, obj2); }
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_pair_group
 *
 *  \brief Add the distance computation between two movable objects.
 *
 *  \param   pair:    the collision pair.
 *  \param   obj1:    the first movable object. 
 *  \param   obj2:    the secon movable object. 
 *
 *  \note  This function does not change the collision test status.
 *  \note  If \a pair == ::NULL then use the current collision pair.
 */
void p3d_col_pair_activate_dist_pair(p3d_collision_pair * pair,
				     p3d_obj * obj1, p3d_obj * obj2)
{
  if (pair == NULL) 
    { pair = p3d_col_pair_get_cur(); }
  if (!p3d_col_pair_is_calc_dist_pair(pair, obj1, obj2)) {
    p3d_BB_activate_pair(pair->BB_handlePt, obj1, obj2);
    kcd_add_distance_pair_of_objects(pair->kcd_handlePt, obj1, obj2);
  }
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_pair_group
 *
 *  \brief Add the test and the distance computation
 *         between two movable objects.
 *
 *  \param   pair:    the collision pair.
 *  \param   obj1:    the first movable object. 
 *  \param   obj2:    the secon movable object. 
 *
 *  \note  If \a pair == ::NULL then use the current collision pair.
 */
void p3d_col_pair_activate_pair(p3d_collision_pair * pair,
				p3d_obj * obj1, p3d_obj * obj2)
{
  if (pair == NULL) 
    { pair = p3d_col_pair_get_cur(); }
  p3d_col_pair_activate_col_pair(pair, obj1, obj2);
  p3d_col_pair_activate_dist_pair(pair, obj1, obj2);
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_pair_group
 *
 *  \brief Deactive the test of a movable object with the environment
 *
 *  \param   pair:    the collision pair.
 *  \param   obj:     the Move3D object. This must be a movable object (body).
 *
 *  \note  This function does not change the distance computation status.
 *  \note  If \a pair == ::NULL then use the current collision pair.
 */
void p3d_col_pair_deactivate_col_env(p3d_collision_pair * pair,
				     p3d_obj * obj)
{
  if (pair == NULL) 
    { pair = p3d_col_pair_get_cur(); }
  if (p3d_col_pair_is_test_col_env(pair, obj))
    { kcd_deact_collision_object_to_env(pair->kcd_handlePt, obj); }
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_pair_group
 *
 *  \brief Deactive the distance computation of a movable object
 *         with the environment
 *
 *  \param   pair:    the collision pair.
 *  \param   obj:     the Move3D object. This must be a movable object (body).
 *
 *  \note  This function does not change the collision test status.
 *  \note  If \a pair == ::NULL then use the current collision pair.
 */
void p3d_col_pair_deactivate_dist_env(p3d_collision_pair * pair,
				      p3d_obj * obj)
{
  if (pair == NULL) 
    { pair = p3d_col_pair_get_cur(); }
  if (p3d_col_pair_is_calc_dist_env(pair, obj)) {
    p3d_BB_deactivate_env(pair->BB_handlePt, obj);
    kcd_deact_distance_object_to_env(pair->kcd_handlePt, obj);
  }
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_pair_group
 *
 *  \brief Deactive the test and the distance computation of a movable object
 *         with the environment
 *
 *  \param   pair:    the collision pair.
 *  \param   obj:     the Move3D object. This must be a movable object (body).
 *
 *  \note  If \a pair == ::NULL then use the current collision pair.
 */
void p3d_col_pair_deactivate_env(p3d_collision_pair * pair,
				 p3d_obj * obj)
{
  if (pair == NULL) 
    { pair = p3d_col_pair_get_cur(); }
  p3d_col_pair_deactivate_col_env(pair, obj);
  p3d_col_pair_deactivate_dist_env(pair, obj);
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_pair_group
 *
 *  \brief Deactive the test between two movable objects.
 *
 *  \param   pair:    the collision pair.
 *  \param   obj1:    the first movable object. 
 *  \param   obj2:    the secon movable object. 
 *
 *  \note  This function does not change the distance computation status.
 *  \note  If \a pair == ::NULL then use the current collision pair.
 */
void p3d_col_pair_deactivate_col_pair(p3d_collision_pair * pair,
				      p3d_obj * obj1, p3d_obj * obj2)
{
  if (pair == NULL) 
    { pair = p3d_col_pair_get_cur(); }
  if (p3d_col_pair_is_test_col_pair(pair, obj1, obj2))
    { kcd_deact_collision_pair_of_objects(pair->kcd_handlePt, obj1, obj2); }
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_pair_group
 *
 *  \brief Deactive the distance computation between two movable objects.
 *
 *  \param   pair:    the collision pair.
 *  \param   obj1:    the first movable object. 
 *  \param   obj2:    the secon movable object. 
 *
 *  \note  This function does not change the collision test status.
 *  \note  If \a pair == ::NULL then use the current collision pair.
 */
void p3d_col_pair_deactivate_dist_pair(p3d_collision_pair * pair,
				       p3d_obj * obj1, p3d_obj * obj2)
{
  if (pair == NULL) 
    { pair = p3d_col_pair_get_cur(); }
  if (p3d_col_pair_is_calc_dist_pair(pair, obj1, obj2)) {
    p3d_BB_deactivate_pair(pair->BB_handlePt, obj1, obj2);
    kcd_deact_distance_pair_of_objects(pair->kcd_handlePt, obj1, obj2);
  }
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_pair_group
 *
 *  \brief Deactive the test and the distance computation
 *         between two movable objects.
 *
 *  \param   pair:    the collision pair.
 *  \param   obj1:    the first movable object. 
 *  \param   obj2:    the secon movable object. 
 *
 *  \note  If \a pair == ::NULL then use the current collision pair.
 */
void p3d_col_pair_deactivate_pair(p3d_collision_pair * pair,
				  p3d_obj * obj1, p3d_obj * obj2)
{
  if (pair == NULL) 
    { pair = p3d_col_pair_get_cur(); }
  p3d_col_pair_deactivate_col_pair(pair, obj1, obj2);
  p3d_col_pair_deactivate_dist_pair(pair, obj1, obj2);
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_pair_group
 *
 *  \brief Deactive all the test of collisions.
 *
 *  \param   pair: the collision pair.
 *
 *  \note  If \a pair == ::NULL then use the current collision pair.
 */
void p3d_col_pair_deactivate_all(p3d_collision_pair * pair)
{
  if (pair == NULL) 
    { pair = p3d_col_pair_get_cur(); }
  p3d_BB_deactivate_all(pair->BB_handlePt);
  kcd_deactivate_all_mo(pair->kcd_handlePt);
}



/***************************************************************************
 ***************************************************************************
 * Information on a collision pair.
 */

/*--------------------------------------------------------------------------*/
/*! \ingroup collision_pair_group
 *
 *  \brief Show if there is a test of the movable object with the environment.
 *
 *  \param   pair:    the collision pair.
 *  \param   obj:     the Move3D object. This must be a movable object (body).
 *
 *  \return TRUE/FALSE, TRUE if the pair check the collision.
 *
 *  \note Only use the data of Kcd
 *  \note  If \a pair == ::NULL then use the current collision pair.
 */
int p3d_col_pair_is_test_col_env(p3d_collision_pair * pair, p3d_obj * obj)
{
  if (pair == NULL) 
    { pair = p3d_col_pair_get_cur(); }
  return kcd_is_collision_object_to_env(pair->kcd_handlePt, obj);
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_pair_group
 *
 *  \brief Show if there is a distance computed of the movable object
 *         with the environment.
 *
 *  \param   pair:    the collision pair.
 *  \param   obj:     the Move3D object. This must be a movable object (body).
 *
 *  \return TRUE/FALSE, TRUE if the pair check the collision.
 *
 *  \note Only use the data of Kcd
 *  \note  If \a pair == ::NULL then use the current collision pair.
 */
int p3d_col_pair_is_calc_dist_env(p3d_collision_pair * pair, p3d_obj * obj)
{
  if (pair == NULL) 
    { pair = p3d_col_pair_get_cur(); }
  return kcd_is_distance_object_to_env(pair->kcd_handlePt, obj);
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_pair_group
 *
 *  \brief Show if there is a test between the two movable objects.
 *
 *  \param   pair:    the collision pair.
 *  \param   obj1:    the first movable object. 
 *  \param   obj2:    the secon movable object. 
 *
 *  \return TRUE/FALSE, TRUE if the pair check the collision.
 *
 *  \note Only use the data of Kcd
 *  \note  If \a pair == ::NULL then use the current collision pair.
 */
int p3d_col_pair_is_test_col_pair(p3d_collision_pair * pair,
				  p3d_obj * obj1, p3d_obj * obj2)

{
  if (pair == NULL) 
    { pair = p3d_col_pair_get_cur(); }
  return kcd_is_collision_pair_of_objects(pair->kcd_handlePt, obj1, obj2);
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_pair_group
 *
 *  \brief Show if there is a distance computed between
 *         the two movable objects.
 *
 *  \param   pair:    the collision pair.
 *  \param   obj1:    the first movable object. 
 *  \param   obj2:    the secon movable object. 
 *
 *  \return TRUE/FALSE, TRUE if the pair check the collision.
 *
 *  \note Only use the data of Kcd
 *  \note  If \a pair == ::NULL then use the current collision pair.
 */
int p3d_col_pair_is_calc_dist_pair(p3d_collision_pair * pair,
				  p3d_obj * obj1, p3d_obj * obj2)

{
  if (pair == NULL) 
    { pair = p3d_col_pair_get_cur(); }
  return kcd_is_distance_pair_of_objects(pair->kcd_handlePt, obj1, obj2);
}



/***************************************************************************
 ***************************************************************************
 * Operation on a collision pairs structure
 */

/*--------------------------------------------------------------------------*/
/*! \ingroup collision_pair_group
 *
 *  \brief Add the test of collision in \a src_pair into the
 *         collision pair \a dest_pair.
 *
 *  \param   src_pair:  the collision pair that is added to \a dest_pair.
 *  \param   dest_pair: the collision pair in which \a src_pair is added.
 *
 *  This is the incrementation function (+=).
 *
 *  It is use for example to add new collision test valid to a localpath.
 *
 *  \note  If \a src_pair == ::NULL then use the current collision pair.
 *  \note  If \a dest_pair == ::NULL then use the current collision pair.
 */
void p3d_col_pair_add_into(p3d_collision_pair * src_pair,
			   p3d_collision_pair * dest_pair)
{
  if (src_pair == NULL) 
    { src_pair = p3d_col_pair_get_cur(); }
  if (dest_pair == NULL) 
    { dest_pair = p3d_col_pair_get_cur(); }
  if (src_pair != dest_pair) {
    p3d_BB_handle_add_into(src_pair->BB_handlePt, dest_pair->BB_handlePt);
    kcd_add_handle_into(src_pair->kcd_handlePt, dest_pair->kcd_handlePt);
  }
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_pair_group
 *
 *  \brief Substract the test of collision in \a src_pair into the
 *         collision pair \a dest_pair.
 *
 *  \param   src_pair:  the collision pair that subtracts \a dest_pair.
 *  \param   dest_pair: the collision pair in which \a src_pair is subtracted.
 *
 *  This is the decrementation function (-=).
 *
 *  It is used, for example, to know what are the test needed to valid a 
 *  localpath in the environment. In this case, the collision needed
 *  are \a dest_pair, but we have already tested the localpath in the
 *  \a src_pair, so we only need to test (\a dest_pair - \a src_pair).
 *
 *  \note  If \a src_pair == ::NULL then use the current collision pair.
 *  \note  If \a dest_pair == ::NULL then use the current collision pair.
 */
void p3d_col_pair_sub_into(p3d_collision_pair * src_pair,
			   p3d_collision_pair * dest_pair)
{
  if (src_pair == NULL) 
    { src_pair = p3d_col_pair_get_cur(); }
  if (dest_pair == NULL) 
    { dest_pair = p3d_col_pair_get_cur(); }
  if (src_pair != dest_pair) {
    p3d_BB_handle_sub_into(src_pair->BB_handlePt, dest_pair->BB_handlePt);
    kcd_sub_handle_into(src_pair->kcd_handlePt, dest_pair->kcd_handlePt);  
  }
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_pair_group
 *
 *  \brief Copy the test of collision in \a src_pair into the
 *         collision pair \a dest_pair.
 *
 *  \param   src_pair:  the collision pair that copied into \a dest_pair.
 *  \param   dest_pair: the collision pair in which \a src_pair is copied.
 *
 *  This is the affectation function (=).
 *
 *  \note  If \a src_pair == ::NULL then use the current collision pair.
 *  \note  If \a dest_pair == ::NULL then use the current collision pair.
 */
void p3d_col_pair_copy_into(p3d_collision_pair * src_pair,
			    p3d_collision_pair * dest_pair)
{
  if (src_pair == NULL) 
    { src_pair = p3d_col_pair_get_cur(); }
  if (dest_pair == NULL) 
    { dest_pair = p3d_col_pair_get_cur(); }
  if (src_pair != dest_pair) {
    p3d_BB_handle_copy_into(src_pair->BB_handlePt, dest_pair->BB_handlePt);
    kcd_copy_handle_into(src_pair->kcd_handlePt, dest_pair->kcd_handlePt);  
  }
}


/***************************************************************************
 ***************************************************************************
 * Create / destroy on a collision pairs structure
 */

/*--------------------------------------------------------------------------*/
/*! \ingroup collision_pair_group
 *
 *  \brief Create a new collision pairs structure.
 *
 *  \return The new collision pairs (::p3d_collision_pair).
 *
 *  \remarks In this new pair, there is no test of collisions.
 */
p3d_collision_pair * p3d_col_pair_create(void)
{
  p3d_collision_pair * new_pair;
  
  new_pair = MY_ALLOC(p3d_collision_pair, 1);
  if (new_pair == NULL) {
    PrintError(("Not enough memory !!!\n"));
    return NULL;
  }
  new_pair->BB_handlePt  = p3d_BB_handle_create();
  new_pair->kcd_handlePt = kcd_create_handle();
  if ((new_pair->BB_handlePt == NULL) || (new_pair->kcd_handlePt == NULL))
    { return NULL; }
  return new_pair;
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_pair_group
 *
 *  \brief Create a new collision pair.
 *
 *  \param   pair: the collision pair that copied into the new contex.
 *
 *  \return The new collision pair (::p3d_collision_pair).
 *
 *  In this new pair, there is the same test of collisions than in 
 *  \a pair.
 *
 *  \note  If \a pair == ::NULL then copy the current collision pairs.
 */
p3d_collision_pair * p3d_col_pair_copy(p3d_collision_pair * pair)
{
  p3d_collision_pair * new_pair;
  
  if (pair == NULL)
    { pair = p3d_col_pair_get_cur(); }
  new_pair = MY_ALLOC(p3d_collision_pair, 1);
  if (new_pair == NULL) {
    PrintError(("Not enough memory !!!\n"));
    return NULL;
  }
  new_pair->BB_handlePt  = p3d_BB_handle_copy(pair->BB_handlePt);
  new_pair->kcd_handlePt = kcd_copy_handle(pair->kcd_handlePt);
  if ((new_pair->BB_handlePt == NULL) || (new_pair->kcd_handlePt == NULL))
    { return NULL; }
  return new_pair;
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_pair_group
 *
 *  \brief Release the memory of a collision pair.
 *
 *  \param   pair: the collision pair to be deleted.
 *
 *  \note Do not use this function for the default collision pair.
 *  \note If the collision pair is in the stack, all the stack is switched.
 */
void p3d_col_pair_destroy(p3d_collision_pair * pair)
{
  int i, i_switch;
  
  if ((pair != NULL) && (pair != default_current_collision_pair)) {
    p3d_BB_handle_destroy(pair->BB_handlePt);
    kcd_destroy_handle(pair->kcd_handlePt);
    MY_FREE(pair, p3d_collision_pair, 1);
    i_switch = 0;
    for(i=0; i<=index_current_collision_pair; i++) {
      if (current_collision_pair_stack[i] == pair)
	{ i_switch ++; }
      else if (i_switch>0) {
	current_collision_pair_stack[i-i_switch] = 
	  current_collision_pair_stack[i];
      }
    }
    if (i_switch>0) {
      index_current_collision_pair -= i_switch;
      if (index_current_collision_pair < 0) {
	if (size_current_collision_pair_stack <= 0)
	  { s_p3d_col_pair_init_current(); }
	else {
	  current_collision_pair_stack[0] = default_current_collision_pair;
	  index_current_collision_pair = 0;
	}
      }
      kcd_set_cur_handle(current_collision_pair_stack
			 [index_current_collision_pair]->kcd_handlePt);
      p3d_BB_set_cur_handle(current_collision_pair_stack
			    [index_current_collision_pair]->BB_handlePt);
    }
  }
}



/****************************************************************************
 ***************************************************************************
 * Manipulating current collision pair
 */


/*--------------------------------------------------------------------------*/
/*! \ingroup current_collision_pair_group
 *
 *  \brief Initialize the data for the current collision pair.
 *
 *  Initialize the stack ::current_collision_pair_stack and
 *  ::default_current_collision_pair.
 *
 *  \internal
 */
static void s_p3d_col_pair_init_current(void)
{
  if (default_current_collision_pair != NULL)
    { p3d_col_pair_clear(); }
  default_current_collision_pair = p3d_col_pair_create();
  current_collision_pair_stack = 
    MY_ALLOC(p3d_collision_pair*, COL_PAIR_STACK_MIN_SIZE);
  current_collision_pair_stack[0] = default_current_collision_pair;
  kcd_set_cur_handle(default_current_collision_pair->kcd_handlePt);
  p3d_BB_set_cur_handle(default_current_collision_pair->BB_handlePt);
  size_current_collision_pair_stack = COL_PAIR_STACK_MIN_SIZE;
  index_current_collision_pair = 0;
}


/*--------------------------------------------------------------------------*/
/*! \ingroup current_collision_pair_group
 *
 *  \brief Get the current collision pair.
 *
 *  \return  the current collision pair.
 *
 *  \note If there is no current collision pair in the stack. 
 *        This function returns ::default_current_collision_pair
 *        (and creates it if necessary s_p3d_col_pair_init_current()).
 *
 *  \warning Do not destroy this current pair. Any modifications on
 *           this pair will be apply on the stack.
 */
p3d_collision_pair * p3d_col_pair_get_cur(void)
{
  if (size_current_collision_pair_stack<=0)
    { s_p3d_col_pair_init_current(); }
  if (current_collision_pair_stack[index_current_collision_pair] == NULL) {
    current_collision_pair_stack[index_current_collision_pair] =
      default_current_collision_pair;
    kcd_set_cur_handle(default_current_collision_pair->kcd_handlePt);
    p3d_BB_set_cur_handle(default_current_collision_pair->BB_handlePt);
  }
  return current_collision_pair_stack[index_current_collision_pair];
}


/*--------------------------------------------------------------------------*/
/*! \ingroup current_collision_pair_group
 *
 *  \brief Copy the current collision pair into \a pair
 *
 *  \retval pair: The collision pair in which the current collision
 *                  pair is copied.
 */
void p3d_col_pair_copy_current_into(p3d_collision_pair * pair)
{
  p3d_col_pair_copy_into(p3d_col_pair_get_cur(), pair);
}


/*--------------------------------------------------------------------------*/
/*! \ingroup current_collision_pair_group
 *
 *  \brief Create a new collision pair.
 *
 *  \return The new collision pair (::p3d_collision_pair).
 *
 *  In this new pair, there is the same test of collisions than in 
 *  the current pair.
 */
p3d_collision_pair * p3d_col_pair_copy_current(void)
{
  return p3d_col_pair_copy(p3d_col_pair_get_cur());
}


/*--------------------------------------------------------------------------*/
/*! \ingroup current_collision_pair_group
 *
 *  \brief Put the \a pair in the current pair.
 *
 *  \param   pair: the collision pair that become current
 *
 *  The \a pair becomes the new current pair. The old current
 *  pair is not destroy. If we want to destroy it, it is necessary
 *  to use p3d_col_pair_destroy().
 */
void p3d_col_pair_put(p3d_collision_pair * pair)
{
  if (pair!=NULL) {
    if (size_current_collision_pair_stack<=0)
      { s_p3d_col_pair_init_current(); }
    current_collision_pair_stack[index_current_collision_pair] = pair; 
    kcd_set_cur_handle(pair->kcd_handlePt);
    p3d_BB_set_cur_handle(pair->BB_handlePt);
  }
}


/*--------------------------------------------------------------------------*/
/*! \ingroup current_collision_pair_group
 *
 *  \brief Push the current collision pair in the stack 
 *  ::current_collision_pair_stack.
 *
 *  Note: After being push their is no valid current collision pair. 
 *  Any function that calls the current collision pair before a 
 *  p3d_col_pair_put() or a p3d_col_pair_push() will get 
 *  ::default_current_collision_pair.
 */
void p3d_col_pair_push(void)
{
  if (size_current_collision_pair_stack<=0)
    { s_p3d_col_pair_init_current(); }
  if (size_current_collision_pair_stack<=index_current_collision_pair+1) {
    current_collision_pair_stack = 
      MY_REALLOC(current_collision_pair_stack, p3d_collision_pair *, 
		 size_current_collision_pair_stack,
		 size_current_collision_pair_stack+COL_PAIR_STACK_MIN_SIZE);
    size_current_collision_pair_stack += COL_PAIR_STACK_MIN_SIZE;
  }
  index_current_collision_pair++;
  current_collision_pair_stack[index_current_collision_pair] = NULL;
}


/*--------------------------------------------------------------------------*/
/*! \ingroup current_collision_pair_group
 *
 *  \brief Push the current collision pair in the stack 
 *  ::current_collision_pair_stack, and put \a pair as the new 
 *  current collision pair.
 *
 *  \param   pair: the collision pair that becomes current
 *
 *  \note This function is only a combination of p3d_col_pair_push()
 *  and p3d_col_pair_put(), but it could be more usefull than both 
 *  dissociated.
 */
void p3d_col_pair_push_and_put(p3d_collision_pair * pair)
{
  if (pair==NULL)
    { PrintError(("Collision pair with NULL pointor !!!\n")); }
  else {
    if (size_current_collision_pair_stack<=0)
      { s_p3d_col_pair_init_current(); }
    if (size_current_collision_pair_stack<=index_current_collision_pair+1) {
      current_collision_pair_stack = 
	MY_REALLOC(current_collision_pair_stack, p3d_collision_pair *, 
		   size_current_collision_pair_stack,
		   size_current_collision_pair_stack+COL_PAIR_STACK_MIN_SIZE);
      size_current_collision_pair_stack += COL_PAIR_STACK_MIN_SIZE;
    }
    index_current_collision_pair++;
    current_collision_pair_stack[index_current_collision_pair] = pair;
    kcd_set_cur_handle(pair->kcd_handlePt);
    p3d_BB_set_cur_handle(pair->BB_handlePt);
  }
}


/*--------------------------------------------------------------------------*/
/*! \ingroup current_collision_pair_group
 *
 *  \brief Restore a collision pair in the stack 
 *  ::current_collision_pair_stack as the new current collision pair.
 *
 *  \note The previous collision pair is not destroy. The memory
 *  management must be done by the owner of this collision pair.
 */
void p3d_col_pair_pop(void)
{
  if (size_current_collision_pair_stack<=0)
    { s_p3d_col_pair_init_current(); }
  if (index_current_collision_pair<1) {
    index_current_collision_pair = 0;
    current_collision_pair_stack[0] = default_current_collision_pair;
  } else
    { index_current_collision_pair --; }
  kcd_set_cur_handle(current_collision_pair_stack
		     [index_current_collision_pair]->kcd_handlePt);
  p3d_BB_set_cur_handle(current_collision_pair_stack
			[index_current_collision_pair]->BB_handlePt);
}


/*--------------------------------------------------------------------------*/
/*! \ingroup current_collision_pair_group
 *
 *  \brief Clear the stack ::current_collision_pair_stack.
 *
 *  \note After being clear their is no valid current collision pair. 
 *  Any function that calls the current collision pair before a 
 *  p3d_col_pair_put() will get ::default_current_collision_pair.
 */
void p3d_col_pair_flush(void)
{
  if (size_current_collision_pair_stack>0) {
    index_current_collision_pair = 0;
    current_collision_pair_stack[0] = default_current_collision_pair;
    kcd_set_cur_handle(default_current_collision_pair->kcd_handlePt);
    p3d_BB_set_cur_handle(default_current_collision_pair->BB_handlePt);
  }
}


/*--------------------------------------------------------------------------*/
/*! \ingroup current_collision_pair_group
 *
 *  \brief Release the memory used to manage current collision pair.
 *
 *  This release the memory for ::current_collision_pair_stack and
 *  ::default_current_collision_pair.
 *
 *  \note After being clear their is no valid current collision pair. 
 *  Any function that uses current collision pair will call 
 *  before p3d_col_pair_init().
 */
void p3d_col_pair_clear(void)
{
  if (default_current_collision_pair != NULL) {
    p3d_BB_handle_destroy(default_current_collision_pair->BB_handlePt);
    kcd_destroy_handle(default_current_collision_pair->kcd_handlePt);
    MY_FREE(default_current_collision_pair, p3d_collision_pair, 1);
    default_current_collision_pair = NULL;
  }
  if (size_current_collision_pair_stack>0) {
    MY_FREE(current_collision_pair_stack, p3d_collision_pair *,
	    size_current_collision_pair_stack);
  }
  size_current_collision_pair_stack =  0;
  index_current_collision_pair      = -1;
}



/****************************************************************************
 ***************************************************************************
 * Multi-environment management for collision pair.
 */


/*--------------------------------------------------------------------------*/
/*! \ingroup current_collision_pair_group
 *
 *  \brief Store the global variable in ::collision_pair_envs.
 *
 *  \internal
 */
static void s_p3d_deconnect_col_pair_global_vars(void)
{
  if (index_cur_collision_pair_envs>=0) {
    collision_pair_envs[index_cur_collision_pair_envs].
      current_collision_pair_stack = current_collision_pair_stack;
    current_collision_pair_stack = NULL;
    collision_pair_envs[index_cur_collision_pair_envs].
      size_current_collision_pair_stack = size_current_collision_pair_stack;
    size_current_collision_pair_stack = 0;
    collision_pair_envs[index_cur_collision_pair_envs].
      index_current_collision_pair = index_current_collision_pair;
    index_current_collision_pair = -1;
    collision_pair_envs[index_cur_collision_pair_envs].
      default_current_collision_pair = default_current_collision_pair;
    default_current_collision_pair = NULL;
  } else
    { p3d_col_pair_clear(); }
}


/*--------------------------------------------------------------------------*/
/*! \ingroup current_collision_pair_group
 *
 *  \brief Restore the data from an environment backup.
 *
 *  \param collision_pair_envPt: Data for collision pair 
 *                               in a specific environment.
 *
 *  \internal
 */
static void s_p3d_connect_col_pair_global_vars(p3d_collision_pair_env *
					       collision_pair_envPt)
{
  current_collision_pair_stack = 
    collision_pair_envPt->current_collision_pair_stack;
  size_current_collision_pair_stack = 
    collision_pair_envPt->size_current_collision_pair_stack;
  index_current_collision_pair = 
    collision_pair_envPt->index_current_collision_pair;
  default_current_collision_pair = 
    collision_pair_envPt->default_current_collision_pair;
}


/*--------------------------------------------------------------------------*/
/*! \ingroup current_collision_pair_group
 *
 *  \brief Create a new case to store the new envirenment \a p3d_env_id.
 *
 *  \param p3d_env_id: Number of the environment.
 *
 *  \internal
 */
static void s_p3d_init_col_pair_global_vars(int p3d_env_id)
{
  collision_pair_envs = MY_REALLOC(collision_pair_envs, p3d_collision_pair_env,
				   nof_collision_pair_envs,
				   nof_collision_pair_envs+1);
  if (collision_pair_envs == NULL) {
    PrintError(("Not enough memory !!!\n"));
    nof_collision_pair_envs = 0;
    index_cur_collision_pair_envs = -1;
  } else {
    index_cur_collision_pair_envs = nof_collision_pair_envs;
    nof_collision_pair_envs ++;
    collision_pair_envs[index_cur_collision_pair_envs].p3d_env_id = p3d_env_id;
    collision_pair_envs[index_cur_collision_pair_envs].
      current_collision_pair_stack = NULL;
    collision_pair_envs[index_cur_collision_pair_envs].
      size_current_collision_pair_stack = 0;
    collision_pair_envs[index_cur_collision_pair_envs].
      index_current_collision_pair = -1;
    collision_pair_envs[index_cur_collision_pair_envs].
      default_current_collision_pair = NULL;
  }
}


/*--------------------------------------------------------------------------*/
/*! \ingroup current_collision_pair_group
 *
 *  \brief Initialize collision pair to start a new environment.
 */
void p3d_col_pair_start(void)
{
  int p3d_env_id;
  int i;

  s_p3d_deconnect_col_pair_global_vars();
  p3d_env_id = XYZ_ENV->num;
  for(i=0; i<nof_collision_pair_envs; i++) {
    if (collision_pair_envs[i].p3d_env_id == p3d_env_id)
      { break; }
  }
  if ((nof_collision_pair_envs>i) &&
      (collision_pair_envs[i].p3d_env_id == p3d_env_id)) {
    /* Already stored */
    s_p3d_connect_col_pair_global_vars(&(collision_pair_envs[i]));
  } else {
    /* Create this new environment */
    s_p3d_init_col_pair_global_vars(p3d_env_id);
  }
}


/*--------------------------------------------------------------------------*/
/*! \ingroup current_collision_pair_group
 *
 *  \brief Release the memory for collision pair in all environment.
 */
void p3d_col_pair_stop(void)
{
  int i;

  s_p3d_deconnect_col_pair_global_vars();
  for(i=0; i<nof_collision_pair_envs; i++) {
    s_p3d_connect_col_pair_global_vars(&(collision_pair_envs[i]));
    p3d_col_pair_clear();
  }
  MY_FREE(collision_pair_envs,p3d_collision_pair_env,nof_collision_pair_envs);
  nof_collision_pair_envs = 0;
  index_cur_collision_pair_envs = -1;
  collision_pair_envs = NULL;
}



/****************************************************************************
 ***************************************************************************
 * Collision context
 */


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_context_group
 *
 *  \brief Add an element to a collision context.
 *
 *  \param  list_begin: The address of the pointer on the beginnig of the list.
 *  \param  param:      The pointer to a parameter that identify the
 *                      conditions to the context of collision.
 *  \param  pair:       The collision pair.
 *
 *  \retval list_begin: If the list is void (\a *list_begin == NULL),
 *                      it initializes it.  
 *
 *  \note It does not merge with element with the same parameter
 *        (but do not copy two time if there is the same ::p3d_collision_pair).
 *  \note It does not copy the context, it is just a link.
 *  \note If the list is not initialized (\a *list_begin == NULL), it
 *        changes the value of \a *list_begin to point to the new element.
 */
void p3d_col_context_add_elem(p3d_collision_context ** list_begin,
			      void * param, p3d_collision_pair * pair)
{
  p3d_collision_context ** prev_elem;
  p3d_collision_context *  next_elem;
  p3d_collision_context *  new_elem;

  prev_elem = list_begin;
  next_elem = *list_begin;
  while ((next_elem != NULL) && (param>next_elem->param)) {
    prev_elem = &(next_elem->next);
    next_elem = next_elem->next;
  }
  while ((next_elem != NULL) && (param==next_elem->param)) {
    if (next_elem->collision_pair == pair)
      { return; }
    prev_elem = &(next_elem->next);
    next_elem = next_elem->next;    
  }
  new_elem = MY_ALLOC(p3d_collision_context, 1);
  if (new_elem==NULL) 
    { PrintError(("Not enough memory !!!\n")); }
  else {
    new_elem->param = param;
    new_elem->collision_pair = pair;
    (*prev_elem) = new_elem;
    new_elem->next = next_elem;
  }
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_context_group
 *
 *  \brief Add an element to a list of collision context.
 *
 *  \param  list_begin: The address of the pointer on the beginnig of the list.
 *  \param  param:      The pointer to a parameter that identify the
 *                      conditions to the context of collision.
 *  \param  pair:       The collision pair.
 *
 *  \retval list_begin: If the list is void (\a *list_begin == NULL),
 *                      it initializes it.  
 *
 *  \note It merges with element with the same parameter.
 *  \note It does not copy the context, it is just a link.
 *  \note If the list is not initialized (\a *list_begin == NULL), it
 *        changes the value of \a *list_begin to point to the new element.
 */
void p3d_col_context_merge_elem(p3d_collision_context ** list_begin,
				void * param, p3d_collision_pair * pair)
{
  p3d_collision_context ** prev_elem;
  p3d_collision_context *  next_elem;
  p3d_collision_context *  new_elem;

  prev_elem = list_begin;
  next_elem = *list_begin;
  while ((next_elem != NULL) && (param>next_elem->param)) {
    prev_elem = &(next_elem->next);
    next_elem = next_elem->next;
  }
  if ((next_elem != NULL) && (param==next_elem->param)) {
    if (next_elem->collision_pair != pair)
      { p3d_col_pair_add_into(pair, next_elem->collision_pair); }
    return;
  }
  new_elem = MY_ALLOC(p3d_collision_context, 1);
  if (new_elem==NULL) 
    { PrintError(("Not enough memory !!!\n")); }
  else {
    new_elem->param = param;
    new_elem->collision_pair = pair;
    (*prev_elem) = new_elem;
    new_elem->next = next_elem;
  }
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_context_group
 *
 *  \brief Delete an element to a list of collision context.
 *
 *  \param  list_begin: The address of the pointer on the beginnig of the list.
 *  \param  param:      The pointer to a parameter that identify the
 *                      conditions to the context of collision.
 *  \param  pair:       The collision pair.
 *
 *  \retval list_begin: The new beginning.
 *                      If the list becomes void (\a *list_begin = NULL).
 */
void p3d_col_context_del_elem(p3d_collision_context ** list_begin,
			      void * param, p3d_collision_pair * pair)
{
  p3d_collision_context ** prev_elem;
  p3d_collision_context *  next_elem;
  p3d_collision_context *  tmp_elem;

  prev_elem = list_begin;
  next_elem = *list_begin;
  while ((next_elem != NULL) && (param>next_elem->param)) {
    prev_elem = &(next_elem->next);
    next_elem = next_elem->next;
  }
  while ((next_elem != NULL) && (param==next_elem->param)) {
    if (next_elem->collision_pair == pair) {
      tmp_elem = next_elem;
      next_elem = next_elem->next;
      (*prev_elem) = next_elem;
      MY_FREE(next_elem, p3d_collision_context, 1);
    } else {
      prev_elem = &(next_elem->next);
      next_elem = next_elem->next;
    }
  }
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_context_group
 *
 *  \brief Delete all the element to a list of collision context 
 *         with the given parameter.
 *
 *  \param  list_begin: The address of the pointer on the beginnig of the list.
 *  \param  param:      The pointer to a parameter that identify the
 *                      conditions to the context of collision.
 *
 *  \retval list_begin: The new beginning.
 *                      If the list becomes void (\a *list_begin = NULL).
 */
void p3d_col_context_del_param(p3d_collision_context ** list_begin,
			       void * param)
{
  p3d_collision_context ** prev_elem;
  p3d_collision_context *  next_elem;
  p3d_collision_context *  tmp_elem;

  prev_elem = list_begin;
  next_elem = *list_begin;
  while ((next_elem != NULL) && (param>next_elem->param)) {
    prev_elem = &(next_elem->next);
    next_elem = next_elem->next;
  }
  while ((next_elem != NULL) && (param==next_elem->param)) {
    tmp_elem = next_elem;
    next_elem = next_elem->next;
    (*prev_elem) = next_elem;
    MY_FREE(next_elem, p3d_collision_context, 1);
  }
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_context_group
 *
 *  \brief Copy a list of collision context 
 *
 *  \param  list_begin: The beginnig of the list.
 *
 *  \return The copy of the list
 */
p3d_collision_context * p3d_col_context_copy(p3d_collision_context *list_begin)
{
  
  p3d_collision_context * copy_begin = NULL;
  p3d_collision_context ** next_elem;

  next_elem = &copy_begin;
  while(list_begin != NULL) {
    (*next_elem) = MY_ALLOC(p3d_collision_context, 1);
    if ((*next_elem) == NULL) {
      PrintError(("Not enough memory !!!\n"));
      return copy_begin;
    }
    (*next_elem)->param          = list_begin->param;
    (*next_elem)->collision_pair = list_begin->collision_pair;
    (*next_elem)->next           = NULL;
    next_elem  = &((*next_elem)->next);
    list_begin = list_begin->next;
  }
  return copy_begin;
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_context_group
 *
 *  \brief Destroy a list of collision context 
 *
 *  \param  list_begin: The address of the pointer on the beginnig of the list.
 */
void p3d_col_context_destroy(p3d_collision_context ** list_begin)
{
  p3d_collision_context *  next_elem;
  p3d_collision_context *  tmp_elem;

  next_elem = *list_begin;
  while(next_elem != NULL) {
    tmp_elem = next_elem;
    next_elem = tmp_elem->next;
    MY_FREE(tmp_elem, p3d_collision_context, 1);
  }
  (*list_begin) = NULL;
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_context_group
 *
 *  \brief Add two collision contexts into \a dest_context.
 *
 *  \param  src_context:  The context that must be copied into \a dest_context.
 *  \param  dest_context: The address of the pointer on the beginnig of the 
 *                        context in which \a src_context is copied.
 *
 *  \retval dest_context: The sum of the two contexts.
 *
 *  \note It does not merge with element with the same parameter
 *        (but do not copy two time if there is the same ::p3d_collision_pair).
 *  \note It does not copy the context, it is just a link.
 *  \note If the list is not initialized (\a *list_begin == NULL), it
 *        changes the value of \a *list_begin to point to the new element.
 */
void p3d_col_context_add(p3d_collision_context *  src_context,
			 p3d_collision_context ** dest_context)
{
  p3d_collision_context ** prev_elem, ** save_prev_elem;
  p3d_collision_context *  next_elem;
  p3d_collision_context *  new_elem;
  p3d_collision_context *  src_elem;

  prev_elem = dest_context;
  next_elem = *dest_context;
  src_elem  = src_context;
  while(src_elem != NULL) {
    while ((next_elem != NULL) && (src_elem->param>next_elem->param)) {
      prev_elem = &(next_elem->next);
      next_elem = next_elem->next;
    }
    save_prev_elem = prev_elem;
    while ((next_elem != NULL) && (src_elem->param==next_elem->param)) {
      if (next_elem->collision_pair == src_elem->collision_pair)
	{ break; }
      prev_elem = &(next_elem->next);
      next_elem = next_elem->next;    
    }
    if ((next_elem == NULL) || (src_elem->param<next_elem->param)) {
      new_elem = MY_ALLOC(p3d_collision_context, 1);
      if (new_elem==NULL) 
	{ PrintError(("Not enough memory !!!\n")); }
      else {
	new_elem->param = src_elem->param;
	new_elem->collision_pair = src_elem->collision_pair;
	(*prev_elem) = new_elem;
	new_elem->next = next_elem;
      } 
    }
    prev_elem = save_prev_elem;
    next_elem = *prev_elem;
    src_elem = src_elem->next;
  }
}


/*--------------------------------------------------------------------------*/
/*! \ingroup collision_context_group
 *
 *  \brief Substract two collision contexts into \a dest_context.
 *
 *  \param  src_context:  The context that must be substracted from
 *                        \a dest_context.
 *  \param  dest_context: The address of the pointer on the beginnig of the 
 *                        context in which \a src_context is substracted.
 *
 *  \return : The result of the substraction of the two contexts
 *            (\a dest_context - \a src_context).
 *
 *  \note It does not merge with element with the same parameter
 *        (but do not copy two time if there is the same ::p3d_collision_pair).
 *  \note It does not copy the context, it is just a link.
 *  \note If the list is not initialized (\a *list_begin == NULL), it
 *        changes the value of \a *list_begin to point to the new element.
 */
p3d_collision_pair * p3d_col_context_sub(p3d_collision_context * src_context,
					 p3d_collision_context * dest_context)
{
  p3d_collision_context *  dest_elem;
  p3d_collision_pair    *  new_pair, * tmp_pair;
  p3d_collision_context *  src_elem, * save_src_elem;

  dest_elem = dest_context;
  src_elem  = src_context;
  new_pair  = p3d_col_pair_create();
  tmp_pair  = p3d_col_pair_create();
  while(dest_elem != NULL) {
    while ((src_elem != NULL) && (src_elem->param<dest_elem->param)) 
      { src_elem = src_elem->next; }
    save_src_elem = src_elem;
    while ((src_elem != NULL) && (src_elem->param==dest_elem->param)) {
      if (dest_elem->collision_pair == src_elem->collision_pair)
	{ break; }
      src_elem = src_elem->next;
    }
    if ((src_elem == NULL) || (src_elem->param>dest_elem->param)) {
      /* We don't have the same collision pair.
	 We must compute the result */
      src_elem = save_src_elem;
      p3d_col_pair_copy_into(dest_elem->collision_pair, tmp_pair);
      while((src_elem != NULL) && (src_elem->param==dest_elem->param)) {
	p3d_col_pair_sub_into(src_elem->collision_pair, tmp_pair);
	src_elem = src_elem->next;
      }
      p3d_col_pair_add_into(tmp_pair, new_pair);
    }
    src_elem = save_src_elem;
    dest_elem = dest_elem->next;
  }
  p3d_col_pair_destroy(tmp_pair);

  return new_pair;
}
