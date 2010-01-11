#include "Util-pkg.h"
#include "Collision-pkg.h"

obj_prim_interval_table *kcd_o_prm_table;
prim_obj_table      *kcd_obj_prim_table;
poly_bb_table       *kcd_poly_bb_table;
mov_obj_bb_table    *kcd_mo_bb_table;   /* for each mo, the bb around it */
link_bb_table_p     *kcd_body_bb_table; /* for each robot or mo-group,the mo-id and bb-id of the links or mo-s in the group */


int *nof_grp_elts; /* for each group of movable objects, the number of movable objects in the group */
int *grp_is_active; /* for each group of movable objects, TRUE iff group must be tested for collision */

int *ith_static_obj; /* entry nof_sos is input index considerung all objects */

/*! \brief Current collision handle. */
static kcd_col_handle * kcd_cur_col_handle = NULL;

/*! \brief The default collision handle. */
static kcd_col_handle * kcd_default_col_handle = NULL;


/* the new API tables of KCD */
/** *********************************************************** **
 ** *********************************************************** **
 **     Functions for collision pairs of groupe of movable objects 
 **     for pair i,j where i<=j, we have the number of movable
 **     object to be tested. The sign shows if it is activate
 **     act_grp_pair_arr[i][k-1-j] > 0 when activated
 ** *********************************************************** **
 ** *********************************************************** **/
/*---------------------------------------------------------------*/
/*!
 * \brief Initialize the array of collisions between groups
 *        of movable objects.
 *
 * Note: this function doesn't set the value of this array. Use
 *       kcd_deactivate_all_grp_col_pairs() to do that.
 *
 * \param  handlePt: The kcd collision handle.
 *
 * \return FALSE if there isn't enough memory.
 *
 * \internal
 */
static int kcd_init_grp_col_pairs(kcd_col_handle * handlePt)
{
  int i;
  int nof_grps = handlePt->nof_grps;

  handlePt->act_grp_pair_arr = MY_ALLOC(grp_act_type *,nof_grps+1);
  if (handlePt->act_grp_pair_arr == NULL) {
    PrintError(("Not enough memory\n"));
    return FALSE;
  }
  handlePt->act_grp_pair_arr[0] = MY_ALLOC(grp_act_type,
					  (nof_grps+2)*(nof_grps+1)/2);
  if (handlePt->act_grp_pair_arr[0] == NULL) {
    PrintError(("Not enough memory\n"));
    return FALSE;
  }
  for(i=0;i<nof_grps;i++) {
    handlePt->act_grp_pair_arr[i+1] = handlePt->act_grp_pair_arr[i] +
      (nof_grps-i+1);
  }
  return TRUE;
}


/*---------------------------------------------------------------*/
/*!
 * \brief Change the activation between group of movable objects.
 *
 * \param  handlePt:   The kcd collision handle.
 * \param  grp_id1:    The indice of the first group of movable objects.
 * \param  grp_id2:    The indice of the first group of movable objects.
 * \param  kcd_activate: TRUE to activate the collision, 
 *                       FALSE to deactivate it.
 */
void kcd_fill_act_grp_pair_arr(kcd_col_handle * handlePt, int grp_id1, 
			       int grp_id2, int kcd_activate)
{
  int nof_grps = handlePt->nof_grps;
  int tmp;

  if(grp_id1 > grp_id2) {
    tmp = grp_id2;
    grp_id2 = grp_id1;
    grp_id1 = tmp;
  }
  if ((kcd_activate && 
       (handlePt->act_grp_pair_arr[grp_id1+1][nof_grps-1-grp_id2]<0)) ||
      (!kcd_activate && 
       (handlePt->act_grp_pair_arr[grp_id1+1][nof_grps-1-grp_id2]>0))) {
    handlePt->act_grp_pair_arr[grp_id1+1][nof_grps-1-grp_id2] =
      - handlePt->act_grp_pair_arr[grp_id1+1][nof_grps-1-grp_id2];
  }
}


/*---------------------------------------------------------------*/
/*!
 * \brief Return the activation between two group of movable objects.
 *
 * \param  handlePt:   The kcd collision handle.
 * \param  grp_id1:    The indice of the first group of movable objects.
 * \param  grp_id2:    The indice of the first group of movable objects.
 *
 * \return TRUE if the collision is activate, 
 *         FALSE if the collision is deactivate.
 */
int kcd_get_grp_col_pair_is_act(kcd_col_handle * handlePt, int grp_id1,
			       int grp_id2)
{
  int nof_grps = handlePt->nof_grps;
  int tmp;

  if(grp_id1 > grp_id2) {
    tmp = grp_id2;
    grp_id2 = grp_id1;
    grp_id1 = tmp;
  }
  if (handlePt->act_grp_pair_arr[grp_id1+1][nof_grps-1-grp_id2] > 0)
    { return TRUE; }

  return FALSE; 
}


/*---------------------------------------------------------------*/
/*!
 * \brief Return the activation between group of movable objects for the 
 *        current collision handle ::kcd_cur_col_handle.
 *
 * \param  grp_id1:     The indice of the first group of movable objects.
 * \param  grp_id2:     The indice of the first group of movable objects.
 *
 * \return TRUE if the collision is activate, 
 *         FALSE if the collision is deactivate.
 */
int kcd_get_grp_col_pair_cur_is_act(int grp_id1, int grp_id2)
{
  return kcd_get_grp_col_pair_is_act(kcd_cur_col_handle, grp_id1, grp_id2);
}


/*---------------------------------------------------------------*/
/*!
 * \brief Deactive all collision test between groups of movable objects.
 *
 * \param  handlePt:   The kcd collision handle.
 *
 * \internal
 */
static void kcd_deactivate_all_grp_col_pairs(kcd_col_handle * handlePt)
{
  int nof_grps = handlePt->nof_grps;

  memset(handlePt->act_grp_pair_arr[0], 0, 
	 (nof_grps+1)*(nof_grps+2)/2*sizeof(grp_act_type));
}


/*---------------------------------------------------------------*/
/*!
 * \brief Release the memory of the array of collisions between
 *        group of movable objects.
 *
 * \param  handlePt: The kcd collision handle.
 *
 * \internal
 */
static void kcd_clean_up_grp_col_pairs(kcd_col_handle * handlePt)
{
  int nof_grps = handlePt->nof_grps;

  MY_FREE(handlePt->act_grp_pair_arr[0],grp_act_type, (nof_grps+1)*(nof_grps+2)/2);
  MY_FREE(handlePt->act_grp_pair_arr,grp_act_type *,nof_grps+1);
  handlePt->act_grp_pair_arr = NULL;
}


/** *********************************************************** **
 ** *********************************************************** **
 **     Functions for collision pairs of movable objects 
 **     with the environment. The sign shows if it is activate
 **     act_grp_env_arr[i] > 0  when activated
 ** *********************************************************** **
 ** *********************************************************** **/
/*---------------------------------------------------------------*/
/*!
 * \brief Initialize the array of collisions between group of
 *        movable objects and the environment.
 *
 * Note: this function doesn't set the value of this array. Use
 *       kcd_deactivate_all_grp_col_env() to do that.
 *
 * \param  handlePt: The kcd collision handle.
 *
 * \return FALSE if there isn't enough memory.
 *
 * \internal
 */
static int kcd_init_grp_col_env(kcd_col_handle * handlePt)
{
  int nof_grps = handlePt->nof_grps;

  handlePt->act_grp_env_arr = MY_ALLOC(grp_act_type, nof_grps+1);
  if (handlePt->act_grp_env_arr == NULL) {
    PrintError(("Not enough memory\n"));
    return FALSE;
  }
  return TRUE;
}


/*---------------------------------------------------------------*/
/*!
 * \brief Change the activation between group of movable object
 *        and the environment.
 *
 * \param  handlePt:   The kcd collision handle.
 * \param  grp_id:     The indice of the group of movable object.
 * \param  kcd_activate: TRUE to activate the collision, 
 *                       FALSE to deactivate it.
 */
void kcd_fill_act_grp_env_arr(kcd_col_handle * handlePt, int grp_id, 
			     int kcd_activate)
{
  if ((kcd_activate && (handlePt->act_grp_env_arr[grp_id+1]<0)) ||
      (!kcd_activate && (handlePt->act_grp_env_arr[grp_id+1]>0))) {
    handlePt->act_grp_env_arr[grp_id+1] =
      - handlePt->act_grp_env_arr[grp_id+1];
  }
}


/*---------------------------------------------------------------*/
/*!
 * \brief Return the activation between group of movable objects
 *        and the environment.
 *
 * \param  handlePt:   The kcd collision handle.
 * \param  grp_id:     The indice of the group of movable object.
 *
 * \return TRUE if the collision is activate, 
 *         FALSE if the collision is deactivate.
 */
int kcd_get_grp_col_env_is_act(kcd_col_handle * handlePt, int grp_id)
{
  if (handlePt->act_grp_env_arr[grp_id+1] > 0)
    { return TRUE; }
  return FALSE;
}


/*---------------------------------------------------------------*/
/*!
 * \brief Return the activation between group of movable objects and the 
 *        environment for the current collision handle ::kcd_cur_col_handle.
 *
 * \param  grp_id:      The indice of the group of movable objects.
 *
 * \return TRUE if the collision is activate, 
 *         FALSE if the collision is deactivate.
 */
int kcd_get_grp_col_env_cur_is_act(int grp_id)
{
  if (kcd_cur_col_handle->act_grp_env_arr[grp_id+1] > 0)
    { return TRUE; }
  return FALSE;
}


/*---------------------------------------------------------------*/
/*!
 * \brief Deactive all collision test between group of movable objects 
 *        and the environment.
 *
 * \param  handlePt:   The kcd collision handle.
 *
 * \internal
 */
static void kcd_deactivate_all_grp_col_env(kcd_col_handle * handlePt)
{
  int nof_grps = handlePt->nof_grps;

  memset(handlePt->act_grp_env_arr, 0, (nof_grps+1)*sizeof(grp_act_type));
}


/*---------------------------------------------------------------*/
/*!
 * \brief Release the memory of the array of collisions between
 *        group of movable objectS and the environment.
 *
 * \param  handlePt: The kcd collision handle.
 *
 * \internal
 */
static void kcd_clean_up_grp_col_env(kcd_col_handle * handlePt)
{
  int nof_grps = handlePt->nof_grps;

  MY_FREE(handlePt->act_grp_env_arr,grp_act_type, nof_grps+1);
  handlePt->act_grp_env_arr = NULL;
}


/** *********************************************************** **
 ** *********************************************************** **
 **     Functions for collision pairs of movable objects 
 **     for pair i,j where i<j, we have boolean value 
 **     in act_mo_pair_arr[i][k-1-j] == TRUE when activated
 ** *********************************************************** **
 ** *********************************************************** **/
/*---------------------------------------------------------------*/
/*!
 * \brief Initialize the array of collisions between movable object.
 *
 * Note: this function doesn't set the value of this array. Use
 *       kcd_deactivate_all_mo_col_pairs() to do that.
 *
 * \param  handlePt: The kcd collision handle.
 *
 * \return FALSE if there isn't enough memory.
 *
 * \internal
 */
static int kcd_init_mo_col_pairs(kcd_col_handle * handlePt)
{
  int i;
  int nof_mos = handlePt->nof_mos;

  if (nof_mos>1) {
    handlePt->act_mo_pair_arr = MY_ALLOC(mo_act_type *,nof_mos-1);
    if (handlePt->act_mo_pair_arr == NULL) {
      PrintError(("Not enough memory\n"));
      return FALSE;
    }
    handlePt->act_mo_pair_arr[0] = MY_ALLOC(mo_act_type,
					    (nof_mos)*(nof_mos-1)/2);
    if (handlePt->act_mo_pair_arr[0] == NULL) {
      PrintError(("Not enough memory\n"));
      return FALSE;
    }
    for(i=1;i<nof_mos-1;i++) {
      handlePt->act_mo_pair_arr[i] = handlePt->act_mo_pair_arr[i-1] +
	(nof_mos-i) * sizeof(mo_act_type);
    }
  } else 
    { handlePt->act_mo_pair_arr = NULL; }
  return TRUE;
}


/*---------------------------------------------------------------*/
/*!
 * \brief Change the activation between movable objects.
 *
 * \param  handlePt:   The kcd collision handle.
 * \param  mo_id1:     The indice of the first movable object.
 * \param  mo_id2:     The indice of the first movable object.
 * \param  kcd_activate: ::KCD_TABLE_COL_ACT to activate only the collision, 
 *                       ::KCD_TABLE_DIST_ACT to activate only the distance.
 *                       ::KCD_TABLE_COL_DIST_ACT to activate both the
 *                                             distance and the collision.
 *                       ::KCD_TABLE_COL_DIST_DEACT to deactivate both the
 *                                             distance and the collision.
 */
void kcd_fill_act_mo_pair_arr(kcd_col_handle * handlePt, int mo_id1, 
			      int mo_id2, mo_act_type kcd_activate)
{
  int nof_mos = handlePt->nof_mos;
  int nof_grps = handlePt->nof_grps;
  int tmp, grp1, grp2;

  if(mo_id1 > mo_id2) {
    tmp = mo_id2;
    mo_id2 = mo_id1;
    mo_id1 = tmp;
  }
  if ((mo_id1!=mo_id2) && (mo_id1>=0) && (mo_id2<nof_mos)) {
    if (kcd_activate &&!(handlePt->act_mo_pair_arr[mo_id1][nof_mos-1-mo_id2])){
      grp1 = kcd_mo_bb_table[mo_id1].grp_id;
      grp2 = kcd_mo_bb_table[mo_id2].grp_id;
      if(grp1 > grp2) {
	tmp = grp2;
	grp2 = grp1;
	grp1 = tmp;
      }
      if (handlePt->act_grp_pair_arr[grp1+1][nof_grps-1-grp2]<0)
	{ handlePt->act_grp_pair_arr[grp1+1][nof_grps-1-grp2] --; }
      else
	{ handlePt->act_grp_pair_arr[grp1+1][nof_grps-1-grp2] ++; }
    } else if (!kcd_activate && 
	       (handlePt->act_mo_pair_arr[mo_id1][nof_mos-1-mo_id2])) {
      grp1 = kcd_mo_bb_table[mo_id1].grp_id;
      grp2 = kcd_mo_bb_table[mo_id2].grp_id;
      if(grp1 > grp2) {
	tmp = grp2;
	grp2 = grp1;
	grp1 = tmp;
      }
      if (handlePt->act_grp_pair_arr[grp1+1][nof_grps-1-grp2]<0)
	{ handlePt->act_grp_pair_arr[grp1+1][nof_grps-1-grp2] ++; }
      else
	{ handlePt->act_grp_pair_arr[grp1+1][nof_grps-1-grp2] --; }
    }
    handlePt->act_mo_pair_arr[mo_id1][nof_mos-1-mo_id2] = kcd_activate;
  }
  /* if mo_id1 == mo_id2, we ignore */
}


/*---------------------------------------------------------------*/
/*!
 * \brief Return the activation between movable objects.
 *
 * \param  handlePt:   The kcd collision handle.
 * \param  mo_id1:     The indice of the first movable object.
 * \param  mo_id2:     The indice of the first movable object.
 *
 * \return ::KCD_TABLE_COL_ACT to activate only the collision, 
 *         ::KCD_TABLE_DIST_ACT to activate only the distance.
 *         ::KCD_TABLE_COL_DIST_ACT to activate both the
 *                                  distance and the collision.
 *         ::KCD_TABLE_COL_DIST_DEACT to deactivate both the
 *                                    distance and the collision.
 */
mo_act_type kcd_get_mo_col_pair_is_act(kcd_col_handle * handlePt, int mo_id1,
					 int mo_id2)
{
  int nof_mos = handlePt->nof_mos;

  if(mo_id1 < mo_id2)
    { return handlePt->act_mo_pair_arr[mo_id1][nof_mos-1-mo_id2]; }
  else if(mo_id1 > mo_id2)
    { return handlePt->act_mo_pair_arr[mo_id2][nof_mos-1-mo_id1]; }

  /* mo_id1 == mo_id2, we ignore */
  return KCD_TABLE_COL_DIST_DEACT; 
}


/*---------------------------------------------------------------*/
/*!
 * \brief Return the activation between movable objects for the 
 *        current collision handle ::kcd_cur_col_handle.
 *
 * \param  mo_id1:     The indice of the first movable object.
 * \param  mo_id2:     The indice of the first movable object.
 *
 * \return ::KCD_TABLE_COL_ACT to activate only the collision, 
 *         ::KCD_TABLE_DIST_ACT to activate only the distance.
 *         ::KCD_TABLE_COL_DIST_ACT to activate both the
 *                                  distance and the collision.
 *         ::KCD_TABLE_COL_DIST_DEACT to deactivate both the
 *                                    distance and the collision.
 */
mo_act_type kcd_get_mo_col_pair_cur_is_act(int mo_id1, int mo_id2)
{
  int nof_mos = kcd_cur_col_handle->nof_mos;

  if(mo_id1 < mo_id2)
    { return kcd_cur_col_handle->act_mo_pair_arr[mo_id1][nof_mos-1-mo_id2]; }
  else if(mo_id1 > mo_id2)
    { return kcd_cur_col_handle->act_mo_pair_arr[mo_id2][nof_mos-1-mo_id1]; }

  /* mo_id1 == mo_id2, we ignore */
  return KCD_TABLE_COL_DIST_DEACT; 
}


/*---------------------------------------------------------------*/
/*!
 * \brief Deactive all collision test between movable objects.
 *
 * \param  handlePt:   The kcd collision handle.
 *
 * \internal
 */
static void kcd_deactivate_all_mo_col_pairs(kcd_col_handle * handlePt)
{
  int nof_mos = handlePt->nof_mos;

  if (nof_mos>1) {
    memset(handlePt->act_mo_pair_arr[0], KCD_TABLE_COL_DIST_DEACT, 
	   nof_mos*(nof_mos-1)/2*sizeof(mo_act_type));
  }
}


/*---------------------------------------------------------------*/
/*!
 * \brief Release the memory of the array of collisions between
 *        movable object.
 *
 * \param  handlePt: The kcd collision handle.
 *
 * \internal
 */
static void kcd_clean_up_mo_col_pairs(kcd_col_handle * handlePt)
{
  int nof_mos = handlePt->nof_mos;

  if (nof_mos>1) {
    MY_FREE(handlePt->act_mo_pair_arr[0],mo_act_type,
	    (nof_mos)*(nof_mos-1)/2);
    MY_FREE(handlePt->act_mo_pair_arr,mo_act_type *,nof_mos-1);
  }
  handlePt->act_mo_pair_arr = NULL;
}


/** *********************************************************** **
 ** *********************************************************** **
 **     Functions for collision pairs of movable objects 
 **     with the environment, we have boolean value 
 **     in act_mo_env_arr[i] == TRUE when activated
 ** *********************************************************** **
 ** *********************************************************** **/
/*---------------------------------------------------------------*/
/*!
 * \brief Initialize the array of collisions between movable object
 *        and the environment.
 *
 * Note: this function doesn't set the value of this array. Use
 *       kcd_deactivate_all_mo_col_env() to do that.
 *
 * \param  handlePt: The kcd collision handle.
 *
 * \return FALSE if there isn't enough memory.
 *
 * \internal
 */
static int kcd_init_mo_col_env(kcd_col_handle * handlePt)
{
  int nof_mos = handlePt->nof_mos;

  if (nof_mos>0) {
    handlePt->act_mo_env_arr = MY_ALLOC(mo_act_type, nof_mos);
    if (handlePt->act_mo_env_arr == NULL) {
      PrintError(("Not enough memory\n"));
      return FALSE;
    }
  } else
    { handlePt->act_mo_env_arr = NULL; }
  return TRUE;
}


/*---------------------------------------------------------------*/
/*!
 * \brief Change the activation between movable object and the environment.
 *
 * \param  handlePt:   The kcd collision handle.
 * \param  mo_id:      The indice of the movable object.
 * \param  kcd_activate: ::KCD_TABLE_COL_ACT to activate only the collision, 
 *                       ::KCD_TABLE_DIST_ACT to activate only the distance.
 *                       ::KCD_TABLE_COL_DIST_ACT to activate both the
 *                                             distance and the collision.
 *                       ::KCD_TABLE_COL_DIST_DEACT to deactivate both the
 *                                             distance and the collision.
 */
void kcd_fill_act_mo_env_arr(kcd_col_handle * handlePt, int mo_id, 
			     mo_act_type kcd_activate)
{
  int grp_id;

  if (kcd_activate && !(handlePt->act_mo_env_arr[mo_id])) {
    grp_id = kcd_mo_bb_table[mo_id].grp_id;
    if (handlePt->act_grp_env_arr[grp_id+1]<0)
      { handlePt->act_grp_env_arr[grp_id+1] --; }
    else
      { handlePt->act_grp_env_arr[grp_id+1] ++; }
  } else if (!kcd_activate && (handlePt->act_mo_env_arr[mo_id])) {
    grp_id = kcd_mo_bb_table[mo_id].grp_id;
    if (handlePt->act_grp_env_arr[grp_id+1]<0)
      { handlePt->act_grp_env_arr[grp_id+1] ++; }
    else
      { handlePt->act_grp_env_arr[grp_id+1] --; }
  }
  handlePt->act_mo_env_arr[mo_id] = kcd_activate;
}


/*---------------------------------------------------------------*/
/*!
 * \brief Return the activation between movable object and the environment.
 *
 * \param  handlePt:   The kcd collision handle.
 * \param  mo_id:      The indice of the movable object.
 *
 * \return ::KCD_TABLE_COL_ACT to activate only the collision, 
 *         ::KCD_TABLE_DIST_ACT to activate only the distance.
 *         ::KCD_TABLE_COL_DIST_ACT to activate both the
 *                                  distance and the collision.
 *         ::KCD_TABLE_COL_DIST_DEACT to deactivate both the
 *                                    distance and the collision.
 */
mo_act_type kcd_get_mo_col_env_is_act(kcd_col_handle * handlePt, int mo_id)
{
  return handlePt->act_mo_env_arr[mo_id];
}


/*---------------------------------------------------------------*/
/*!
 * \brief Return the activation between movable object and the 
 *        environment for the current collision handle ::kcd_cur_col_handle.
 *
 * \param  mo_id:      The indice of the movable object.
 *
 * \return ::KCD_TABLE_COL_ACT to activate only the collision, 
 *         ::KCD_TABLE_DIST_ACT to activate only the distance.
 *         ::KCD_TABLE_COL_DIST_ACT to activate both the
 *                                  distance and the collision.
 *         ::KCD_TABLE_COL_DIST_DEACT to deactivate both the
 *                                    distance and the collision.
 */
mo_act_type kcd_get_mo_col_env_cur_is_act(int mo_id)
{
  return kcd_cur_col_handle->act_mo_env_arr[mo_id];
}


/*---------------------------------------------------------------*/
/*!
 * \brief Deactive all collision test between movable object 
 *        and the environment.
 *
 * \param  handlePt:   The kcd collision handle.
 *
 * \internal
 */
static void kcd_deactivate_all_mo_col_env(kcd_col_handle * handlePt)
{
  int nof_mos = handlePt->nof_mos;

  if (nof_mos>0) {
    memset(handlePt->act_mo_env_arr, KCD_TABLE_COL_DIST_DEACT, 
	   nof_mos*sizeof(mo_act_type));
  }
}


/*---------------------------------------------------------------*/
/*!
 * \brief Release the memory of the array of collisions between
 *        movable object and the environment.
 *
 * \param  handlePt: The kcd collision handle.
 *
 * \internal
 */
static void kcd_clean_up_mo_col_env(kcd_col_handle * handlePt)
{
  int nof_mos = handlePt->nof_mos;

  if (nof_mos>0)
    { MY_FREE(handlePt->act_mo_env_arr,mo_act_type, nof_mos); }
  handlePt->act_mo_env_arr = NULL;
}


/** *********************************************************** **
 ** *********************************************************** **
 **     Functions to manage the kcd collision handle.
 ** *********************************************************** **
 ** *********************************************************** **/
/*---------------------------------------------------------------*/
/*!
 * \brief Create and initialize a kcd collision handle.
 *
 * Note: this function deactivate all tesst of collisions.
 *
 * \param  nof_mos: The number of movable object.
 *
 * \return The kcd collision handle. NULL if there isn't enough memory.
 */
kcd_col_handle * kcd_create_col_handle(int nof_mos, int nof_grps)
{
  kcd_col_handle * handlePt;

  handlePt = MY_ALLOC(kcd_col_handle, 1);
  if (handlePt == NULL) {
    PrintError(("Not enough memory\n"));
    return NULL;
  }
  handlePt->nof_mos  = nof_mos;
  handlePt->nof_grps = nof_grps;
  if (!kcd_init_grp_col_pairs(handlePt) || !kcd_init_grp_col_env(handlePt) ||
      !kcd_init_mo_col_pairs(handlePt) || !kcd_init_mo_col_env(handlePt)) {
    PrintError(("Not enough memory\n"));
    return NULL;
  }
  kcd_deactivate_all_col_handle(handlePt);
  return handlePt;
}


/*---------------------------------------------------------------*/
/*!
 * \brief Copy a kcd collision handle to a new structure.
 *
 * \param  handlePt: The kcd collision handle that must be copied.
 *
 * \return The kcd collision handle. NULL if there isn't enough memory.
 */
kcd_col_handle * kcd_copy_col_handle(kcd_col_handle * handlePt)
{
  kcd_col_handle * copy_handlePt;

  copy_handlePt = MY_ALLOC(kcd_col_handle, 1);
  if (copy_handlePt == NULL) {
    PrintError(("Not enough memory\n"));
    return NULL;
  }
  copy_handlePt->nof_mos = handlePt->nof_mos;
  copy_handlePt->nof_grps = handlePt->nof_grps;
  if (!kcd_init_grp_col_pairs(copy_handlePt) || 
      !kcd_init_grp_col_env(copy_handlePt) ||
      !kcd_init_mo_col_pairs(copy_handlePt) || 
      !kcd_init_mo_col_env(copy_handlePt)) {
    PrintError(("Not enough memory\n"));
    return NULL;
  }
  kcd_copy_col_handle_into(handlePt, copy_handlePt);
  return copy_handlePt;
}


/*---------------------------------------------------------------*/
/*!
 * \brief Copy a kcd collision handle into an other structure.
 *
 * \warning The number of movable object must be the same in both handle.
 *
 * \param src_handlePt:   The kcd collision handle that must be copied.
 * \param dest_handlePt:  The kcd collision handle that must store the copy.
 */
void kcd_copy_col_handle_into(kcd_col_handle * src_handlePt,
			      kcd_col_handle * dest_handlePt)
{
  int nof_mos = dest_handlePt->nof_mos;
  int nof_grps = dest_handlePt->nof_grps;

  if (nof_mos>0) {
    memcpy(dest_handlePt->act_mo_env_arr, src_handlePt->act_mo_env_arr,
	   nof_mos*sizeof(mo_act_type));
    if (nof_mos>1) {
      memcpy(dest_handlePt->act_mo_pair_arr[0], 
	     src_handlePt->act_mo_pair_arr[0],
	     nof_mos*(nof_mos-1)/2*sizeof(mo_act_type));
    }
  }
  memcpy(dest_handlePt->act_grp_env_arr, src_handlePt->act_grp_env_arr,
	 (nof_grps+1)*sizeof(grp_act_type));
  memcpy(dest_handlePt->act_grp_pair_arr[0], src_handlePt->act_grp_pair_arr[0],
	 (nof_grps+1)*(nof_grps+2)/2*sizeof(grp_act_type));
}


/*---------------------------------------------------------------*/
/*!
 * \brief Merge two kcd collision handle (sum of collision tests)
 *
 * Note: It is an "or" opperation between all flags.
 *
 * \warning The number of movable object must be the same in both handle.
 *
 * \param src_handlePt:   The source kcd collision handle.
 * \param dest_handlePt:  The kcd collision handle that must store the sum.
 */
void kcd_add_col_handle_into(kcd_col_handle * src_handlePt,
			     kcd_col_handle * dest_handlePt)
{
  int i, j, k, l, tmp;
  int nof_links1, nof_links2, mo_id1, mo_id2;
  int nof_grps = dest_handlePt->nof_grps;
  int nof_mos  = dest_handlePt->nof_mos;

  for(i=0; i<=nof_grps; i++) {
    /* Group vs environment */
    if (src_handlePt->act_grp_env_arr[i]>0) {
      if (dest_handlePt->act_grp_env_arr[i]>0) {
	dest_handlePt->act_grp_env_arr[i] = 0;
	nof_links1 = nof_grp_elts[i];
	for(l=0; l<nof_links1; l++) {
	  mo_id1 = kcd_body_bb_table[i][l].kcd_mo_id;
	  dest_handlePt->act_mo_env_arr[mo_id1] |= 
	    src_handlePt->act_mo_env_arr[mo_id1];
	  if (dest_handlePt->act_mo_env_arr[mo_id1])
	    { dest_handlePt->act_grp_env_arr[i] ++; }
	}
      } else {
	dest_handlePt->act_grp_env_arr[i] = src_handlePt->act_grp_env_arr[i];
	nof_links1 = nof_grp_elts[i];
	for(l=0; l<nof_links1; l++) {
	  mo_id1 = kcd_body_bb_table[i][l].kcd_mo_id ;
	  dest_handlePt->act_mo_env_arr[mo_id1] = 
	    src_handlePt->act_mo_env_arr[mo_id1];
	}
      }
    } 
  }
  for(i=0; i<=nof_grps; i++) {
    /* Autocollision */
    if (src_handlePt->act_grp_pair_arr[i][nof_grps-i]>0) {
      if (dest_handlePt->act_grp_pair_arr[i][nof_grps-i]>0) {
	dest_handlePt->act_grp_pair_arr[i][nof_grps-i] = 0;
	nof_links1 = nof_grp_elts[i];
	for (l=0; l<nof_links1-1; l++) {
	  mo_id1 = kcd_body_bb_table[i][l].kcd_mo_id;
	  for(k=l+1; k<nof_links1; k++) {
	    mo_id2 = kcd_body_bb_table[i][k].kcd_mo_id;
	    if (mo_id1 > mo_id2) {
	      tmp = mo_id2;
	      mo_id2 = mo_id1;
	      mo_id1 = tmp;
	    }
	    dest_handlePt->act_mo_pair_arr[mo_id1][nof_mos-1-mo_id2] |=
	      src_handlePt->act_mo_pair_arr[mo_id1][nof_mos-1-mo_id2];
	    if (dest_handlePt->act_mo_pair_arr[mo_id1][nof_mos-1-mo_id2])
	      { dest_handlePt->act_grp_pair_arr[i][nof_grps-i] ++; }
	  }
	}
      } else {
	dest_handlePt->act_grp_pair_arr[i][nof_grps-i] =
	  src_handlePt->act_grp_pair_arr[i][nof_grps-i];
	nof_links1 = nof_grp_elts[i];
	for(l=0; l<nof_links1; l++) {
	  mo_id1 = kcd_body_bb_table[i][l].kcd_mo_id ;
	  for(k=l+1; k<nof_links1; k++) {
	    mo_id2 = kcd_body_bb_table[i][k].kcd_mo_id ;
	    if (mo_id1 > mo_id2) {
	      tmp = mo_id2;
	      mo_id2 = mo_id1;
	      mo_id1 = tmp;
	    }
	    dest_handlePt->act_mo_pair_arr[mo_id1][nof_mos-1-mo_id2] =
	      src_handlePt->act_mo_pair_arr[mo_id1][nof_mos-1-mo_id2];
	  }
	}
      }
    } 
    for(j=i+1; j<=nof_grps; j++) {
      /* Group vs group */
      if (src_handlePt->act_grp_pair_arr[i][nof_grps-j]>0) {
	if (dest_handlePt->act_grp_pair_arr[i][nof_grps-j]>0) {
	  dest_handlePt->act_grp_pair_arr[i][nof_grps-j] = 0;
	  nof_links1 = nof_grp_elts[i];
	  nof_links2 = nof_grp_elts[j];
	  for(l=0; l<nof_links1; l++) {
	    mo_id1 = kcd_body_bb_table[i][l].kcd_mo_id ;
	    for(k=0; k<nof_links2; k++) {
	      mo_id2 = kcd_body_bb_table[j][k].kcd_mo_id ;
	      if (mo_id1 > mo_id2) {
		tmp = mo_id2;
		mo_id2 = mo_id1;
		mo_id1 = tmp;
	      }
	      dest_handlePt->act_mo_pair_arr[mo_id1][nof_mos-1-mo_id2] |=
		src_handlePt->act_mo_pair_arr[mo_id1][nof_mos-1-mo_id2];
	      if (dest_handlePt->act_mo_pair_arr[mo_id1][nof_mos-1-mo_id2])
		{ dest_handlePt->act_grp_pair_arr[i][nof_grps-j] ++; }
	    }
	  }
	} else {
	  dest_handlePt->act_grp_pair_arr[i][nof_grps-j] =
	    src_handlePt->act_grp_pair_arr[i][nof_grps-j];
	  nof_links1 = nof_grp_elts[i];
	  nof_links2 = nof_grp_elts[j];
	  for(l=0; l<nof_links1; l++) {
	    mo_id1 = kcd_body_bb_table[i][l].kcd_mo_id ;
	    for(k=0; k<nof_links2; k++) {
	      mo_id2 = kcd_body_bb_table[j][k].kcd_mo_id ;
	      if (mo_id1 > mo_id2) {
		tmp = mo_id2;
		mo_id2 = mo_id1;
		mo_id1 = tmp;
	      }
	      dest_handlePt->act_mo_pair_arr[mo_id1][nof_mos-1-mo_id2] =
		src_handlePt->act_mo_pair_arr[mo_id1][nof_mos-1-mo_id2];
	    }
	  }
	}
      }
    } 
  }
}


/*---------------------------------------------------------------*/
/*!
 * \brief Merge two kcd collision handle (subtraction of collision tests)
 *
 * Note: It is an "dest and not src" opperation between all flags.
 *
 * \warning The number of movable object must be the same in both handle.
 *
 * \param src_handlePt:   The source kcd collision handle.
 * \param dest_handlePt:  The kcd collision handle that must store the sum.
 */
void kcd_sub_col_handle_into(kcd_col_handle * src_handlePt,
			     kcd_col_handle * dest_handlePt)
{
  int i, j, k, l;
  int nof_links1, nof_links2, mo_id1, mo_id2, tmp;
  int nof_grps = dest_handlePt->nof_grps;
  int nof_mos  = dest_handlePt->nof_mos;

  for(i=0; i<=nof_grps; i++) {
    /* Group vs environment */
    if (dest_handlePt->act_grp_env_arr[i]>0) {
      if (src_handlePt->act_grp_env_arr[i]>0) {
	dest_handlePt->act_grp_env_arr[i] = 0;
	nof_links1 = nof_grp_elts[i];
	for(l=0; l<nof_links1; l++) {
	  mo_id1 = kcd_body_bb_table[i][l].kcd_mo_id;
	  dest_handlePt->act_mo_env_arr[mo_id1] &= 
	    ~(src_handlePt->act_mo_env_arr[mo_id1]);
	  if (dest_handlePt->act_mo_env_arr[mo_id1]) 
	    { dest_handlePt->act_grp_env_arr[i] ++; }
	}
      }
    } 
  }
  for(i=0; i<=nof_grps; i++) {
    /* Autocollision */
    if (dest_handlePt->act_grp_pair_arr[i][nof_grps-i]>0) {
      if (src_handlePt->act_grp_pair_arr[i][nof_grps-i]>0) {
	dest_handlePt->act_grp_pair_arr[i][nof_grps-i] = 0;
	nof_links1 = nof_grp_elts[i];
	for(l=0; l<nof_links1-1; l++) {
	  mo_id1 = kcd_body_bb_table[i][l].kcd_mo_id ;
	  for(k=l+1; k<nof_links1; k++) {
	    mo_id2 = kcd_body_bb_table[i][k].kcd_mo_id ;
	    if (mo_id1 > mo_id2) {
		tmp = mo_id2;
		mo_id2 = mo_id1;
		mo_id1 = tmp;
	    }
	    dest_handlePt->act_mo_pair_arr[mo_id1][nof_mos-1-mo_id2] =
	      ~(src_handlePt->act_mo_pair_arr[mo_id1][nof_mos-1-mo_id2]);
	    if (dest_handlePt->act_mo_pair_arr[mo_id1][nof_mos-1-mo_id2])
	      { dest_handlePt->act_grp_pair_arr[i][nof_grps-i] ++; }
	  }
	}
      }
    }
    for(j=i+1; j<=nof_grps; j++) {
      /* Group vs group */
      if (dest_handlePt->act_grp_pair_arr[i][nof_grps-j]>0) {
	if (src_handlePt->act_grp_pair_arr[i][nof_grps-j]>0) {
	  dest_handlePt->act_grp_pair_arr[i][nof_grps-j] = 0;
	  nof_links1 = nof_grp_elts[i];
	  nof_links2 = nof_grp_elts[j];
	  for(l=0; l<nof_links1; l++) {
	    mo_id1 = kcd_body_bb_table[i][l].kcd_mo_id ;
	    for(k=0; k<nof_links2; k++) {
	      mo_id2 = kcd_body_bb_table[j][k].kcd_mo_id ;
	      if (mo_id1 > mo_id2) {
		tmp = mo_id2;
		mo_id2 = mo_id1;
		mo_id1 = tmp;
	      }
	      dest_handlePt->act_mo_pair_arr[mo_id1][nof_mos-1-mo_id2] =
		~(src_handlePt->act_mo_pair_arr[mo_id1][nof_mos-1-mo_id2]);
	      if (dest_handlePt->act_mo_pair_arr[mo_id1][nof_mos-1-mo_id2])
		{ dest_handlePt->act_grp_pair_arr[i][nof_grps-i] ++; }
	    }
	  }
	}
      }
    } 
  }
}


/*---------------------------------------------------------------*/
/*!
 * \brief Deactive all collision test between movable object 
 *        and the environment.
 *
 * \param  handlePt:   The kcd collision handle.
 */
void kcd_deactivate_all_col_handle(kcd_col_handle * handlePt)
{
  kcd_deactivate_all_mo_col_pairs(handlePt);
  kcd_deactivate_all_mo_col_env(handlePt);
  kcd_deactivate_all_grp_col_pairs(handlePt);
  kcd_deactivate_all_grp_col_env(handlePt);
}


/*---------------------------------------------------------------*/
/*!
 * \brief Release the memory of the kcd collisions handle.
 *
 * \param  handlePt: The kcd collision handle.
 *
 * \note This function cannot destroy the default collision handle.
 *       To do that you must use kcd_clean_up_col_table().
 */
void kcd_clean_up_col_handle(kcd_col_handle * handlePt)
{
  if ((handlePt != NULL) && (handlePt != kcd_default_col_handle)) {
    kcd_clean_up_mo_col_pairs(handlePt);
    kcd_clean_up_mo_col_env(handlePt);
    kcd_clean_up_grp_col_pairs(handlePt);
    kcd_clean_up_grp_col_env(handlePt);
    MY_FREE(handlePt, kcd_col_handle, 1);
    if (handlePt == kcd_cur_col_handle)
      { kcd_cur_col_handle = kcd_default_col_handle; }
  }
}


/** *********************************************************** **
 ** *********************************************************** **
 **     Functions to manage the kcd collision handle.
 ** *********************************************************** **
 ** *********************************************************** **/
/*---------------------------------------------------------------*/
/*!
 * \brief Initialize the current and the default kcd collision handle.
 *
 * \param  nof_mos:  The number of movable objects.
 * \param  nof_grps: The number of group of movable objects.
 *
 * \return FALSE if there isn't enough memory.
 */
int kcd_init_col_table(int nof_mos, int nof_grps)
{
  if (kcd_default_col_handle != NULL)
    { kcd_clean_up_col_table(); }
  kcd_default_col_handle = kcd_create_col_handle(nof_mos, nof_grps);
  kcd_cur_col_handle = kcd_default_col_handle;
  if (kcd_default_col_handle == NULL)
    { return FALSE; }
  return TRUE;
}


/*---------------------------------------------------------------*/
/*!
 * \brief Put a new collision handle as the current collision handle.
 *
 * \param  handlePt: The kcd collision handle.
 */
void kcd_set_cur_col_handle(kcd_col_handle * handlePt)
{
  if (handlePt!=NULL)
    { kcd_cur_col_handle = handlePt; }
}


/*---------------------------------------------------------------*/
/*!
 * \brief Get the current collision handle.
 *
 * \return  The current kcd collision handle.
 */
kcd_col_handle * kcd_get_cur_col_handle(void)
{
  return kcd_cur_col_handle;
}


/*---------------------------------------------------------------*/
/*!
 * \brief Release the memory of the default kcd collision handle.
 *
 * \warning Don't call kcd_get_mo_col_env_cur_is_act() or
 * kcd_get_mo_col_pair_cur_is_act() just after that.
 */
void kcd_clean_up_col_table(void)
{
  if (kcd_default_col_handle != NULL) {
    kcd_clean_up_mo_col_pairs(kcd_default_col_handle);
    kcd_clean_up_mo_col_env(kcd_default_col_handle);
    kcd_clean_up_grp_col_pairs(kcd_default_col_handle);
    kcd_clean_up_grp_col_env(kcd_default_col_handle);
    MY_FREE(kcd_default_col_handle, kcd_col_handle, 1);
  }
  kcd_default_col_handle = NULL;
  kcd_cur_col_handle = NULL;
}


/** **************************************************************** **
 ** **************************************************************** **
 ** Functions: these manipulate data-table kcd_o_prm_table
 **            in which is stored two indices k and k+l for each 
 **            external object
 **            - all values from k to k+l are the indices of 
 **              the primitives belonging to the same external object
 ** **************************************************************** **
 ** **************************************************************** **/

/* ****************************************************************** *
 * Function: kcd_init_cd_o_prm_table
 *           initializes table of given size
 * ARG IN  : nof_ext_objs         size (total number of  objects 
 *                                in the scene for the external
 *                                model, both static and movable)
 * ****************************************************************** */
void kcd_init_kcd_o_prm_table(int nof_ext_objs)
{
  kcd_o_prm_table = NULL;
  kcd_o_prm_table = MY_ALLOC(obj_prim_interval_table,nof_ext_objs);
}

void kcd_resize_kcd_o_prm_table(int nof_ext_objs, int nof_extra_ext_objs)
{
  kcd_o_prm_table = MY_REALLOC(kcd_o_prm_table, obj_prim_interval_table,
			       nof_ext_objs, nof_ext_objs+nof_extra_ext_objs);
}

/* ****************************************************************** *
 * Function: kcd_fill_kcd_o_prm_table
 *           fills entry of table kcd_o_prm_table
 * ARG IN  : kcd_ext_o    the external object was given as kcd_ext_o-th
 *                        object (both static and movable)
 *           m_or_s_id    the rank index as movable or static object
 *                        (this is the m_or_s_id-th movable object or
 *                        the m_or_s_id-th static object
 *           is_movable   TRUE or FALSE
 *           sml_i        index of first primitive belonging to the 
 *                        object, sml_i-th primitive given by user
 *           big_i        index of last primitive belonging to the 
 *                        object, sml_i-th primitive given by user
 * REMARKS : if sml_i > big_i, the external object does not contain
 *           any primitive (for instance, a graphical object)
 *           sets is_active (collision must be tested) and is_moved
 *           (mo changed place since last collision test) to TRUE 
 * ****************************************************************** */
void kcd_fill_kcd_o_prm_table(int kcd_ext_o,int m_or_s_id,int is_movable,int sml_i,int big_i)
{
  kcd_o_prm_table[kcd_ext_o].ext_obj_id = kcd_ext_o;
  kcd_o_prm_table[kcd_ext_o].m_or_s_id = m_or_s_id;
  kcd_o_prm_table[kcd_ext_o].is_movable = is_movable;
  kcd_o_prm_table[kcd_ext_o].is_active = TRUE;        /* DEFAULT */
  kcd_o_prm_table[kcd_ext_o].is_moved  = TRUE;        /* DEFAULT */
  kcd_o_prm_table[kcd_ext_o].sml_index  = sml_i;      /* ext_p_id ! */
  kcd_o_prm_table[kcd_ext_o].big_index  = big_i;      /* ext_p_id ! */
  /* kcd_o_prm_table[kcd_ext_o].is_active is TRUE if the object must be taken into
     account for collision test. PRECOND (5 Feb. 2001): object is movable */
}

/* B Kineo Carl 27.02.20002 */
/* ****************************************************************** *
 * Function: kcd_set_active_kcd_o_prm_table
 *           set field is_active to given value for given ext_obj_id
 * ARG IN  : ext_obj_id   the external object was given as kcd_ext_o-th
 *                        object (both static and movable)
 *           set_value    the new value (TRUE for active object, 
 *                        FALSE for deactivated object)
 * ****************************************************************** */
void kcd_set_active_kcd_o_prm_table(int ext_obj_id,int set_value) 
{
  kcd_o_prm_table[ext_obj_id].is_active = set_value;
}

/* ****************************************************************** *
 * Function: kcd_get_active_kcd_o_prm_table
 *           set field is_active to given value for given ext_obj_id
 * ARG IN  : ext_obj_id   the external object was given as kcd_ext_o-th
 *                        object (both static and movable)
 * RETURNS : set_value    the current value (TRUE for active object, 
 *                        FALSE for deactivated object) of is_active,
 *                        and TRUE if ext_obj_id is unvalid (i.e. 
 *                        when equal to -1)
 * ****************************************************************** */
int kcd_get_active_kcd_o_prm_table(int ext_obj_id) 
{
  int ret_val = TRUE;

  if(ext_obj_id != -1)
    ret_val = kcd_o_prm_table[ext_obj_id].is_active;;

/*   if(ext_obj_id == -1) */
/*     { */
/*       printf("ext_obj_id unvalid\n"); */
/*     } */
/*   else */
/*     { */
/*       if(kcd_o_prm_table[ext_obj_id].is_active) */
/* 	printf("obj. %i is active (TRUE)\n",ext_obj_id); */
/*       else */
/* 	printf("obj. %i is deactivated (FALSE)\n",ext_obj_id); */
/*     } */
  return ret_val;
}
/* E Kineo Carl 27.02.20002 */

/* ****************************************************************** *
 * Function: kcd_get_nof_prm_of_ext_o
 *           returns the number of primitives in a given external object
 * ARG IN  : kcd_ext_o    the external object was given as kcd_ext_o-th
 *                        object (both static and movable)
 * RETURNS :              number of primitives in the external object
 * ****************************************************************** */
int kcd_get_nof_prm_of_ext_o(int kcd_ext_o)
{
  return (kcd_o_prm_table[kcd_ext_o].big_index - kcd_o_prm_table[kcd_ext_o].sml_index + 1);
}

/* ****************************************************************** *
 * Function: kcd_clean_up_cd_o_prm_table
 *           cleans up table of given size
 * ARG IN  : nof_ext_objs         size (total number of  objects 
 *                                in the scene for the external
 *                                model, both static and movable)
 * ****************************************************************** */
void kcd_clean_up_kcd_o_prm_table(int nof_ext_objs)
{
  MY_FREE(kcd_o_prm_table,obj_prim_interval_table,nof_ext_objs);
  kcd_o_prm_table = NULL;
}

/** **************************************************************** **
 ** **************************************************************** **
 ** Functions: these manipulate data-table kcd_obj_prim_table
 **            each external object primitive has an entry
 **            in which is stored:
 **            - primitive belongs to movable object (true 
 **              or false)
 **            - primitive was entered as the ext_p_id-th 
 **              of the scene
 **            - primitive belongs to the object entered as  
 **              the ext_o_id-th of the scene
 **            - kcd_p_id is the bb_id of root obb around 
 **              moving object, or -1 
 **            - kcd_p_id is the bb_id of root obb around
 **              polyhedron or solid
 ** **************************************************************** **
 ** **************************************************************** **/

/* ****************************************************************** *
 * Function: kcd_init_prim_obj_table
 *           initializes table of given size
 * ARG IN  : tot_nof_prims        size (total number of solids
 *                                and polyhedrons in the scene,
 *                                both static and movable)
 * ****************************************************************** */
void kcd_init_prim_obj_table(int tot_nof_prims)
{
  int i;

  kcd_obj_prim_table = NULL;
  kcd_obj_prim_table = MY_ALLOC(prim_obj_table,tot_nof_prims);
  for(i=0;i<tot_nof_prims;i++)
    {
      kcd_obj_prim_table[i].is_movable = 0;
      kcd_obj_prim_table[i].ext_prim_id = -1;
      kcd_obj_prim_table[i].ext_obj_id = -1;
      kcd_obj_prim_table[i].kcd_obj_id = -1;
      kcd_obj_prim_table[i].kcd_prim_bb_id = -1;
    }
}

void kcd_resize_prim_obj_table(int tot_nof_prims, int tot_extra_nof_prims)
{
  int i;

  kcd_obj_prim_table = MY_REALLOC(kcd_obj_prim_table, prim_obj_table,
				tot_nof_prims, tot_nof_prims+tot_extra_nof_prims);
  for(i=tot_nof_prims;i<(tot_nof_prims+tot_extra_nof_prims);i++)
    {
      kcd_obj_prim_table[i].is_movable = 0;
      kcd_obj_prim_table[i].ext_prim_id = -1;
      kcd_obj_prim_table[i].ext_obj_id = -1;
      kcd_obj_prim_table[i].kcd_obj_id = -1;
      kcd_obj_prim_table[i].kcd_prim_bb_id = -1;
    }
}


/* ****************************************************************** *
 * Function: kcd_fill_prim_obj_table
 *           fills table with given data
 * ARG IN  : ext_o_id    ext_o_id-th object of the scene
 *           ext_p_id    ext_p_id-th primitive of the scene
 *           kcd_o_id    root obb around moving object, or -1
 *           kcd_p_id    root obb around polyhedron or solid
 *           is_movable  belongs to movable object (true or false)
 * ****************************************************************** */
void kcd_fill_prim_obj_table(int ext_o_id, int ext_p_id, int kcd_o_id, 
			     int kcd_p_id, int is_movable)
{
  kcd_obj_prim_table[ext_p_id].is_movable = is_movable;
  /* primitive was entered as the ext_p_id-th of the scene */
  kcd_obj_prim_table[ext_p_id].ext_prim_id = ext_p_id; 
  /* primitive belongs to the object entered as the ext_o_id-th of the scene */
  kcd_obj_prim_table[ext_p_id].ext_obj_id = ext_o_id;
  /* kcd_p_id is the bb_id of root obb around moving object, or -1 */
  kcd_obj_prim_table[ext_p_id].kcd_obj_id = kcd_o_id;
  /* kcd_p_id is the bb_id of root obb around polyhedron or solid */
  kcd_obj_prim_table[ext_p_id].kcd_prim_bb_id = kcd_p_id; 
}

/* ****************************************************************** *
 * Function: kcd_clean_up_prim_obj_table
 *           clean up data-table kcd_obj_prim_table
 * ARG IN  : tot_nof_prims        size (total number of solids
 *                                and polyhedrons in the scene,
 *                                both static and movable)
 * ****************************************************************** */
void kcd_clean_up_prim_obj_table(int tot_nof_prims)
{
  MY_FREE(kcd_obj_prim_table,prim_obj_table,tot_nof_prims);
  kcd_obj_prim_table = NULL;
}

/* ****************************************************************** *
 * Function: kcd_belong_to_same_object
 *           returns TRUE if given primitives belong to same 
 *           external data object (decision made by a look-up in 
 *           data-table kcd_obj_prim_table), FALSE otherwise
 * ARG IN  : ext_p_id1  ext_p_id1-th primitive of the scene
 *           ext_p_id2  ext_p_id2-th primitive of the scene
 * ****************************************************************** */
int new_kcd_belong_to_same_object(int ext_p_id1, int ext_p_id2)
{
  return (kcd_obj_prim_table[ext_p_id1].ext_obj_id == kcd_obj_prim_table[ext_p_id2].ext_obj_id);
}




/** **************************************************************** **
 ** **************************************************************** **
 ** Functions: these manipulate data-table ith_static_obj
 **            which stores the input index for each static object
 **            (allows to look for the i-th static object)
 ** **************************************************************** **
 ** **************************************************************** **/
void kcd_init_ith_static_obj(int nof_st_obj)
{
  ith_static_obj = NULL;
  ith_static_obj = MY_ALLOC(int,nof_st_obj);
}

void kcd_resize_ith_static_obj(int nof_st_obj, int extra_size)
{
  ith_static_obj = MY_REALLOC(ith_static_obj,int,nof_st_obj,nof_st_obj+extra_size);
}


void kcd_fill_ith_static_obj(int kcd_so_id, int ext_obj_id)
{
  ith_static_obj[kcd_so_id] = ext_obj_id;
}

void kcd_clean_up_ith_static_obj(int nof_st_obj)
{
  MY_FREE(ith_static_obj,int,nof_st_obj);
  ith_static_obj = NULL;
}


/** **************************************************************** **
 ** **************************************************************** **
 ** Functions: these manipulate data-table nof_grp_elts
 **            which stores the number of movable objects in each
 **            of the groups
 ** **************************************************************** **
 ** **************************************************************** **/
void kcd_init_grps(int nof_mo_grps)
{
  nof_grp_elts = NULL; 
  nof_grp_elts = MY_ALLOC(int,nof_mo_grps+1);
  grp_is_active = NULL;
  grp_is_active = MY_ALLOC(int,nof_mo_grps+1);
}

void kcd_fill_grps(int grp_id, int nof_mos_in_grp)
{
  nof_grp_elts[grp_id+1] = nof_mos_in_grp;
  grp_is_active[grp_id+1] = TRUE; /* DEFAULT */
}

int kcd_get_nof_grp_elts(int grp_id)
{
  return nof_grp_elts[grp_id+1];
}

int kcd_grp_is_active(int grp_id)
{
  return grp_is_active[grp_id+1];
}

void kcd_clean_up_grps(int nof_mo_grps)
{
  MY_FREE(grp_is_active,int,nof_mo_grps+1);
  grp_is_active = NULL; 
  MY_FREE(nof_grp_elts,int,nof_mo_grps+1);
  nof_grp_elts = NULL; 
}



/** **************************************************************** **
 ** **************************************************************** **
 ** Functions: these manipulate data-table kcd_mo_bb_table
 **            each external movable object has an entry
 **            in which is stored:
 **            - movable object was entered as the ext_o_id-th 
 **              of the scene
 **            - kcd_o_id is the bb_id of root obb around
 **              movable object
 ** **************************************************************** **
 ** **************************************************************** **/

/* ****************************************************************** *
 * Function: kcd_init_mo_bb_table
 *           initializes table of given size
 * ARG IN  : tot_nof_mos          size (total number of 
 *                                movable objects in the scene)
 * ****************************************************************** */
void kcd_init_mo_bb_table(int tot_nof_mos)
{
  kcd_mo_bb_table = NULL;
  kcd_mo_bb_table = MY_ALLOC(mov_obj_bb_table,tot_nof_mos);
}

/* ****************************************************************** *
 * Function: kcd_fill_mo_bb_table
 *           fills table with given data
 * ARG IN  : ext_o_id    ext_o_id-th object of the scene
 *           mobj_id     mobj_id-th movable(!) object of the scene
 *           bb_id       root obb around movable object
 *           grp_id      id of group to which mo belongs, or -1 if 
 *                       movable object does not belong to any group
 * ****************************************************************** */
void kcd_fill_kcd_mo_bb_table(int ext_o_id, int mobj_id, int bb_id, int grp_id)
{
  if(kcd_mo_bb_table)
    {
      kcd_mo_bb_table[mobj_id].ext_o_id = ext_o_id;
      kcd_mo_bb_table[mobj_id].obj_id = mobj_id;
      kcd_mo_bb_table[mobj_id].bb_id = bb_id;
      kcd_mo_bb_table[mobj_id].grp_id = grp_id;
    }
}

/* ****************************************************************** *
 * Function: kcd_clean_up_mo_bb_table
 *           clean up data-table kcd_mo_bb_table
 * ARG IN  : tot_nof_mos          size (total number of 
 *                                movable objects in the scene)
 * ****************************************************************** */
void kcd_clean_up_mo_bb_table(int tot_nof_mos)
{
  MY_FREE(kcd_mo_bb_table,mov_obj_bb_table,tot_nof_mos);
  kcd_mo_bb_table = NULL;
}


/* the original tables of KCD: */

/** **************************************************************** **
 ** **************************************************************** **
 ** Functions: these manipulate data-table kcd_poly_bb_table
 **            each external object primitive has an entry
 **            in which is stored:
 **            - primitive was entered as the ext_p_id-th 
 **              of the scene
 **            - kcd_p_id is the bb_id of root obb around
 **              polyhedron or solid (initialized as -1)
 ** **************************************************************** **
 ** **************************************************************** **/

/* ****************************************************************** *
 * Function: kcd_init_poly_bb_table
 *           initializes table of given size
 *           sets field bb_id -1 for each entry (this allows look-up
 *           telling whether primitive was initialized)
 * ARG IN  : tot_nof_prims        size (total number of solids
 *                                and polyhedrons in the scene,
 *                                both static and movable)
 * ****************************************************************** */
void kcd_init_kcd_poly_bb_table(int tot_nof_prims)
{
  int i;

  kcd_poly_bb_table = NULL;
  kcd_poly_bb_table = MY_ALLOC(poly_bb_table,tot_nof_prims);
  for(i=0;i<tot_nof_prims;i++)
    {
      kcd_poly_bb_table[i].p3d_poly_id = i;
      kcd_poly_bb_table[i].bb_id = -1;
     }
}

void kcd_resize_kcd_poly_bb_table(int old_size, int extra_size)
{
  int i;

  kcd_poly_bb_table = MY_REALLOC(kcd_poly_bb_table,poly_bb_table,
				 old_size,old_size+extra_size);
  for(i=old_size;i<old_size+extra_size;i++)
    {
      kcd_poly_bb_table[i].p3d_poly_id = i;
      kcd_poly_bb_table[i].bb_id = -1;
    }  
}

/* ****************************************************************** *
 * Function: kcd_fill_poly_bb_table
 *           fills table with given data
 * ARG IN  : ext_p_id    ext_p_id-th primitive of the scene
 *           prim_bb_id  root obb around polyhedron or solid
 * REMARKS : prim_bb_id stays -1 (initial value) iff this function
 *           wasn't called through an object-definition of the kcd-api
 * ****************************************************************** */
void kcd_fill_kcd_poly_bb_table(int ext_p_id, int prim_bb_id)
{
  if(kcd_poly_bb_table)
    {
      kcd_poly_bb_table[ext_p_id].p3d_poly_id = ext_p_id;
      kcd_poly_bb_table[ext_p_id].bb_id = prim_bb_id;
    }
}

/* ****************************************************************** *
 * Function: kcd_clean_up_poly_bb_table
 *           clean up data-table kcd_poly_bb_table
 * ARG IN  : tot_nof_prims        size (total number of solids
 *                                and polyhedrons in the scene,
 *                                both static and movable)
 * ****************************************************************** */
void kcd_clean_up_poly_bb_table(int tot_nof_prims)
{
  MY_FREE(kcd_poly_bb_table,poly_bb_table,tot_nof_prims);
  kcd_poly_bb_table = NULL;
}

/** **************************************************************** **
 ** **************************************************************** **
 ** Functions: these manipulate data-table kcd_body_bb_table
 **            is a matrix [grp_id][mo_id]
 **            in which is stored:
 **            - was entered as the mobj_id-th mov. obj. of the scene
 **            - the kcd-id of the mov. obj.
 **            - the bb_id of root obb around the mov. obj.
 ** **************************************************************** **
 ** **************************************************************** **/

/* ****************************************************************** *
 * Function: kcd_init_body_bb_table
 *           initialize the table of movable object groups 
 * ARG IN  : nof_grps     number of groups to initialize
 * ****************************************************************** */
void kcd_init_body_bb_table(int nof_grps)
{
  kcd_body_bb_table = NULL;
  kcd_body_bb_table = MY_ALLOC(link_bb_table_p,nof_grps+1);
}

/* ****************************************************************** *
 * Function: kcd_reinit_body_bb_table
 *           extend the table of movable object groups 
 * ARG IN  : orig_nof_grps     number of existing entries in 
 *                             kcd_body_bb_table for groups
 *           nof_to_be_added   number of entries for groups to be added
 * PRECOND : kcd_init_body_bb_table called before 
 * ****************************************************************** */
void kcd_reinit_body_bb_table(int orig_nof_grps, int nof_to_be_added)
{
  kcd_body_bb_table =
    MY_REALLOC(kcd_body_bb_table,link_bb_table_p,orig_nof_grps+1,
	       orig_nof_grps+nof_to_be_added+1);
}

/* ****************************************************************** *
 * Function: kcd_init_body_bb_table_entry
 *           initializes for a given group the table of movable objects
 * ARG IN  : this_entry     id of group to initialize
 *           nof_mos        number of movable objects contained 
 *                          in the group
 * PRECOND : kcd_init_body_bb_table called before 
 * ****************************************************************** */
void kcd_init_body_bb_table_entry(int this_entry, int nof_mos)
{
  kcd_body_bb_table[this_entry+1] = MY_ALLOC(link_bb_table,nof_mos);
} 


/*--------------------------------------------------------------------------*/
/*!
 * \brief Fills data for all movable objects of a given group.
 *
 * \param  grp_id:      id of the given group
 * \param  arr_kcd_ids: array of ids of the movable objects 
 *                       in the group (array contains object-ids)
 * \param  nof_mo_ids:  number of movable objects in the array
 *
 * REMARKS:
 *           - ::kcd_mo_bb_table [arr_mo_ids[i]].grp_id is set as well, 
 *             for each of the movable objects in the array
 *           - movable objects already in another group are ignored
 *             in group \a grp_id !
 *           - group -1 must not be initialized.
 */
void kcd_fill_body_bb_table_entry(int grp_id,int *arr_kcd_ids,int nof_mo_ids)
{
  int mo_it,kcd_mo_id;

  for(mo_it=0;mo_it<nof_mo_ids;mo_it++)
    {
      kcd_mo_id = kcd_o_prm_table[arr_kcd_ids[mo_it]].m_or_s_id;
      if(kcd_mo_bb_table[kcd_mo_id].grp_id == -1)
	{
	  /* keep track of the group to which mo belongs */
	  kcd_mo_bb_table[kcd_mo_id].grp_id = grp_id;
	  kcd_mo_bb_table[kcd_mo_id].ith_mo = mo_it;
	  /* the position in the array: */
	  kcd_body_bb_table[grp_id+1][mo_it].mo_id = mo_it;
	  /* KCD-id of the movable object */
	  kcd_body_bb_table[grp_id+1][mo_it].kcd_mo_id = kcd_mo_id;
	  /* KCD-id of the root of the obb-tree on the movable object */
	  kcd_body_bb_table[grp_id+1][mo_it].bb_id = kcd_mo_bb_table[kcd_mo_id].bb_id;
	}
      else
	{
	  PrintInfo(("KCD: WARNING: movable object %i belongs already to a group \n",kcd_mo_id));
	  PrintInfo(("              and thus movable object not added to a second group!\n"));
	  kcd_body_bb_table[grp_id+1][mo_it].mo_id = -1;
	  kcd_body_bb_table[grp_id+1][mo_it].kcd_mo_id = -1;
	  kcd_body_bb_table[grp_id+1][mo_it].bb_id = -1; 
	}
    }
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Fills data for all movable objects without group and store
 *        them in the group -1.
 *
 * \return The number of movable object without group.
 *
 * REMARKS:
 *           - All objects with ::kcd_mo_bb_table [arr_mo_ids[i]].grp_id == -1
 *             are used
 *           - group -1 must not be initialized.
 */
int kcd_fill_no_body_bb_table_entry(void)
{
  int mo_it,kcd_mo_id;
  int nof_mos = kcd_get_nof_mos();
  int nof_no_grp, mo_no_grp_it;

  nof_no_grp = 0;
  for(mo_it=0;mo_it<nof_mos;mo_it++) {
    kcd_mo_id = kcd_o_prm_table[mo_it].m_or_s_id;
    if(kcd_mo_bb_table[kcd_mo_id].grp_id == -1)
      { nof_no_grp ++; }
  }
  kcd_init_body_bb_table_entry(-1, nof_no_grp);
  if (nof_no_grp == 0)
    { return 0; }
  mo_no_grp_it = 0;
  for(mo_it=0;mo_it<nof_mos;mo_it++) {
    kcd_mo_id = kcd_o_prm_table[mo_it].m_or_s_id;
    if(kcd_mo_bb_table[kcd_mo_id].grp_id == -1) {
      /* keep track of the group to which mo belongs */
      kcd_mo_bb_table[kcd_mo_id].ith_mo = mo_no_grp_it;
      /* the position in the array: */
      kcd_body_bb_table[0][mo_no_grp_it].mo_id = mo_no_grp_it;
      /* KCD-id of the movable object */
      kcd_body_bb_table[0][mo_no_grp_it].kcd_mo_id = kcd_mo_id;
      /* KCD-id of the root of the obb-tree on the movable object */
      kcd_body_bb_table[0][mo_no_grp_it].bb_id =
	kcd_mo_bb_table[kcd_mo_id].bb_id;
      mo_no_grp_it ++;
    }
  }
  return nof_no_grp;
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Return the value of one table element.
 *
 * \param  robot_id: The robot identifiant (-1 for movable object 
 *                   without robot)
 * \param  body_id:  The body identifiant
 *
 * \return The link_bb_table element.
 */
link_bb_table kcd_get_body_bb_table(int robot_id, int body_id)
{
  return kcd_body_bb_table[robot_id+1][body_id];
}

/* ****************************************************************** *
 * Function: kcd_clean_up_body_bb_table_entry
 *           cleans up for a given group the table of movable objects
 * ARG IN  : this_entry     id of group to clean
 *           nof_mos        number of movable objects contained 
 *                          in the group
 * ****************************************************************** */
void kcd_clean_up_body_bb_table_entry(int this_entry, int nof_mos)
{
  MY_FREE(kcd_body_bb_table[this_entry+1],link_bb_table,nof_mos);
} 

/* ****************************************************************** *
 * Function: kcd_clean_up_body_bb_table
 *           cleans up the table of movable object groups 
 * ARG IN  : nof_grps     number of groups to clean
 * REMARKS : kcd_clean_up_body_bb_table should be called before 
 *           on all entries
 * ****************************************************************** */
void kcd_clean_up_body_bb_table(int nof_grps)
{
  int i,nof_entries;

  for(i=-1;i<nof_grps;i++)
    {
      nof_entries = nof_grp_elts[i+1];
      kcd_clean_up_body_bb_table_entry(i,nof_entries);
    }

  MY_FREE(kcd_body_bb_table,link_bb_table_p,nof_grps+1);
  kcd_body_bb_table = NULL;
}
