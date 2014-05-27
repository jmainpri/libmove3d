/*
 * Copyright (c) 2001-2014 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 */
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"   // <- modif Juan
#include "Planner-pkg.h"

static int   SEARCH_STATUS =  P3D_FAILURE;
static int   SEARCH_VERBOSE=  FALSE;


/* Array of pointers to localplanner functions. The indices of the array
   are the elements of the p3d_localpath_type enumeration. */
ptr_to_localplanner array_localplanner[]=
  {
    (pp3d_localpath (*)(p3d_rob*, configPt, configPt, int*))(p3d_rsarm_localplanner),
    (pp3d_localpath (*)(p3d_rob*, configPt, configPt, int*))(p3d_linear_localplanner),
    (pp3d_localpath (*)(p3d_rob*, configPt, configPt, int*))(p3d_manh_localplanner),
    (pp3d_localpath (*)(p3d_rob*, configPt, configPt, int*))(p3d_trailer_localplanner),
    (pp3d_localpath (*)(p3d_rob*, configPt, configPt, int*))(p3d_nocusp_trailer_localplanner),
    (pp3d_localpath (*)(p3d_rob*, configPt, configPt, int*))(p3d_hilflat_localplanner),
    (pp3d_localpath (*)(p3d_rob*, configPt, configPt, int*))(p3d_nocusp_hilflat_localplanner),
    (pp3d_localpath (*)(p3d_rob*, configPt, configPt, int*))(p3d_dubins_localplanner)
  };

char * array_localplanner_name[] =
  {
    (char*)"R&S+linear",
    (char*)"Linear",
    (char*)"Manhattan",
    (char*)"Trailer",
    (char*)"Trailer-Forward",
    (char*)"Flat-Hilare",
    (char*)"Flat-Hilare-Forward",
    (char*)"Dubins"
#ifdef MULTILOCALPATH
    ,(char*)"Soft-Motion"  /* XB */
    ,(char*)"Multi-Localpath" // it's not a planner explicitely, it call other planner which will control all localplanner
#endif
  };
#ifdef MULTILOCALPATH
int P3D_NB_LOCAL_PLANNER = 10;
#else
int P3D_NB_LOCAL_PLANNER = 8;
#endif
/*
 *  To add a local method, add the corresponding function
 *  in array_localplanner[] and its name in array_localplanner_name[] and do
 *  not forget to add a line in
 *  typedef enum {
 *      ...
 *    } p3d_localpath_type;
 *
 *  in the file localpath.h
 */

/*
 * Initialize a p3d_stay_within_dist_data to the joints which have not
 * previous joint
 */
void p3d_init_stay_within_dist_data(p3d_stay_within_dist_data * data)
{
  data->vmax = 0.;
  data->wmax = 0.;
  data->p.x  = 0.;
  data->p.y  = 0.;
  data->p.z  = 0.;
}


/**********************************************************************/

int   p3d_search_status(void) {
  return(SEARCH_STATUS);
}

void p3d_set_search_status(int search_status) {
  SEARCH_STATUS = search_status;
}

void  p3d_search_verbose(int flag){
  SEARCH_VERBOSE = flag;
}

int  p3d_get_search_verbose(){
  return(SEARCH_VERBOSE);
}


/**********************************************************************/

p3d_localpath * p3d_local_planner(p3d_rob *robotPt, configPt q1, configPt q2)
{
  configPt q[2];
  q[0] = q1;
  q[1] = q2;
  return p3d_local_planner_array(robotPt, q);
}

p3d_localpath *p3d_local_planner_multisol(p3d_rob *robotPt, configPt q1, configPt q2, int* ikSol){
  configPt q[2];
  q[0] = q1;
  q[1] = q2;
  return p3d_local_planner_array_multisol(robotPt, q, ikSol);
}


p3d_localpath *p3d_local_planner_array(p3d_rob *robotPt, configPt* q)
{
  pp3d_localpath localpathPt;
  p3d_localpath_type lpl_type = robotPt->lpl_type;

#ifdef  MULTILOCALPATH
  //  p3d_jnt * jntPt = NULL;
  //  double vmin=0.0,vmax=0.0;
  //  double diff=0.0;
  //  /* Fix the circular jnt */
  //  int k, njnt = robotPt->njoints, angle =0.0;
  //  /* translation parameters of main body */
  //  for (int i=0; i<=njnt; i++) {
  //    jntPt = robotPt->joints[i];
  //    if(jntPt->type == P3D_ROTATE) {
  //      for (int j=0; j<jntPt->dof_equiv_nbr; j++) {
  //        k = jntPt->index_dof+j;
  //        if (p3d_jnt_is_dof_circular(jntPt, j)){
  //         // printf("k %d q[0][k] %f q[1][k] %f",k, q[0][k], q[1][k]);
  //         q[0][k] =  angle_limit_PI(q[0][k]);
  //         // printf(" q[0][k]_modif %f",q[0][k]);
  //         diff = diff_angle(q[0][k], q[1][k]);
  //         q[1][k] = q[0][k] + diff_angle(q[0][k], q[1][k]);
  //         // printf(" diff %f q[1][k] %f\n",diff, q[1][k] );
  //        }
  //      }
  //    }
  //  }

  p3d_adaptConfigsForCircularDofs(robotPt, &(q[0]), &(q[1]));


  //   printf("After\n");
  //   p3d_set_and_update_this_robot_conf(robotPt, q[1]);
  //   p3d_get_robot_config_into(robotPt, &q[1]);
  //   //print_config(robotPt, q[1]); 
  //   int indexfreeflyer = 37;
  //   if(fabs(q[1][indexfreeflyer]-q1Tmp[indexfreeflyer]) > EPS6 ||
  //      fabs(q[1][indexfreeflyer+1]-q1Tmp[indexfreeflyer+1]) > EPS6 ||
  //      fabs(q[1][indexfreeflyer+2]-q1Tmp[indexfreeflyer+2]) > EPS6) { 
  //      printf("config differs\n"); 
  //   }
  //   p3d_destroy_config(robotPt, q1Tmp);
  //   p3d_set_and_update_this_robot_conf(robotPt, qTmp);
  //   p3d_destroy_config(robotPt, qTmp);





  if (lpl_type == MULTI_LOCALPATH) {
    int nblpGp = 0;
    nblpGp = robotPt->mlp->nblpGp;
    p3d_softMotion_data     *softMotion_data[nblpGp];
    for(int i=0; i<nblpGp;i++) {
      if( p3d_multiLocalPath_get_value_groupToPlan(robotPt, i)) {
	if(robotPt->mlp->mlpJoints[i]->lplType == SOFT_MOTION) {
	  softMotion_data[i] = NULL;
	  softMotion_data[i] = p3d_create_softMotion_data_multilocalpath(robotPt, i);
	} else {
	  softMotion_data[i] = NULL;
	}
      } else {
	softMotion_data[i] = NULL;
      }
    }
    localpathPt = p3d_multiLocalPath_localplanner(robotPt, softMotion_data, q[0], q[1], q[1], NULL);

  } else if (lpl_type == SOFT_MOTION) {
    PrintError(("You can't call Soft-Motion planner in this way, call Multi-Graph and set a group with Soft-Motion"));
  } else {
#endif
    localpathPt = array_localplanner[lpl_type](robotPt, q[0], q[1], NULL);
#ifdef  MULTILOCALPATH
  }
#endif

  /* When retrieving statistics;
     Commit Jim; date: 01/10/2008 */
  if(getStatStatus()){
    XYZ_GRAPH->stat->planLpNum++;
    XYZ_GRAPH->stat->planLpLenght += localpathPt->length_lp;
  }

  return(localpathPt);
}


p3d_localpath *p3d_local_planner_array_multisol(p3d_rob *robotPt, configPt* q, int* ikSol)
{
  pp3d_localpath localpathPt = NULL;
  p3d_localpath_type lpl_type = robotPt->lpl_type;

#ifdef  MULTILOCALPATH


  p3d_adaptConfigsForCircularDofs(robotPt, &(q[0]), &(q[1]));


  //   printf("After\n");
  //   p3d_set_and_update_this_robot_conf(robotPt, q[1]);
  //   p3d_get_robot_config_into(robotPt, &q[1]);
  //   //print_config(robotPt, q[1]); 
  //   int indexfreeflyer = 37;
  //   if(fabs(q[1][indexfreeflyer]-q1Tmp[indexfreeflyer]) > EPS6 ||
  //      fabs(q[1][indexfreeflyer+1]-q1Tmp[indexfreeflyer+1]) > EPS6 ||
  //      fabs(q[1][indexfreeflyer+2]-q1Tmp[indexfreeflyer+2]) > EPS6) { 
  //      printf("config differs\n"); 
  //   }
  //   p3d_destroy_config(robotPt, q1Tmp);
  //   p3d_set_and_update_this_robot_conf(robotPt, qTmp);
  //   p3d_destroy_config(robotPt, qTmp);

  if (lpl_type == MULTI_LOCALPATH) {
    int nblpGp = 0;
    nblpGp = robotPt->mlp->nblpGp;
    pp3d_softMotion_data     softMotion_data[nblpGp];
    for(int i=0; i<nblpGp;i++) {
     if( p3d_multiLocalPath_get_value_groupToPlan(robotPt, i)) {
      if(robotPt->mlp->mlpJoints[i]->lplType == SOFT_MOTION) {
	softMotion_data[i] = NULL;
	softMotion_data[i] = p3d_create_softMotion_data_multilocalpath(robotPt, i);
      } else {
	softMotion_data[i] = NULL;
      }
     } else {
	softMotion_data[i] = NULL;
      }
    }
    localpathPt = p3d_multiLocalPath_localplanner(robotPt, softMotion_data, q[0], q[1], q[1], ikSol);

  } else if (lpl_type == SOFT_MOTION) {
    PrintError(("You can't call Soft-Motion planner in this way, call Multi-Graph and set a group with Soft-Motion"));
  } else {
#endif
    localpathPt = array_localplanner[lpl_type](robotPt, q[0], q[1], ikSol);
#ifdef  MULTILOCALPATH
  }
#endif

  /* When retrieving statistics;
     Commit Jim; date: 01/10/2008 */
  if(getStatStatus()){
    // //     XYZ_GRAPH->stat->planLpNum++;
    // //     if(localpathPt){
    // //       XYZ_GRAPH->stat->planLpLenght += localpathPt->length_lp;
    // //     }
  }

  return(localpathPt);
}

int p3d_local_set_planner(p3d_localpath_type type)
{
  /* the local method of the current robot is changed */
  XYZ_ENV->cur_robot->lpl_type = type;
  return(type);
}

p3d_localpath_type p3d_local_get_planner(void)
{
  p3d_rob * robotPt = (p3d_rob *)p3d_get_desc_curid(P3D_ROBOT);
  p3d_localpath_type lpl_type;

  lpl_type = robotPt->lpl_type;

  return(lpl_type);
}

char *
p3d_local_getname_planner(p3d_localpath_type lpl_type)
{
  return(array_localplanner_name[lpl_type]);
}

/* D�ut modification Fabien */
p3d_localpath_type
p3d_local_getid_planner(const char * name)
{
  int i;

  for(i=0; i<P3D_NB_LOCAL_PLANNER; i++) {
    if (strcmp(name, array_localplanner_name[i]) == 0)
      { return (p3d_localpath_type)i; }
  }
  return (p3d_localpath_type)P3D_NULL_OBJ;
}
/* Fin modification Fabien */


/*
 *  replace a part of trajectory
 *
 *  Input:  pointer to the beginning of the trajectory
 *          the first local path of the part of trajectory to replace,
 *          the last local path of the part of trajectory to replace,
 *          the new part.
 *
 *  Output: pointer to the beginning of the trajectory
 *
 *  Description: replaces in a list of local path a sub-list by
 *          another list of localpath.
 *
 */
p3d_localpath* p3d_replace_traj_part(p3d_localpath* start_trajPt,
				     p3d_localpath* start_old_partPt,
				     p3d_localpath* last_old_partPt,
				     p3d_localpath* new_partPt)
{
  p3d_localpath *last_new_partPt = new_partPt;

  /* last local path of the new part */
  while (last_new_partPt->next_lp != NULL){
    last_new_partPt = last_new_partPt->next_lp;
  }

  if (start_old_partPt->prev_lp != NULL){
    /* the old part does not contain the beginning of the
       trajectory */
    start_old_partPt->prev_lp->next_lp = new_partPt;
  }
  else {
    /* the first local path has changed, return pointer to the
       new one */
    start_trajPt = new_partPt;
  }
  new_partPt->prev_lp = start_old_partPt->prev_lp;

  if (last_old_partPt->next_lp != NULL){
    /* the old part does not contain the end of the trajectory */
    last_old_partPt->next_lp->prev_lp = last_new_partPt;
  }
  last_new_partPt->next_lp = last_old_partPt->next_lp;

  start_old_partPt->prev_lp = NULL;
  last_old_partPt->next_lp = NULL;

  return start_trajPt;
}

/*
 *  distance between two configurations of a robot
 *
 *  Input:  the robot,
 *          two configurationsl.
 *
 *  Output: the distance
 *
 *  The function computes a local path between q1 and q2 using
 *  the default local method of the robot and returns the length
 *  of the local path.
 */

double p3d_dist_q1_q2(p3d_rob *robotPt, configPt q1, configPt q2)
{
  pp3d_localpath localpathPt;
  double length;
  //  p3d_localpath_type lpl_type = robotPt->lpl_type;
	
  localpathPt = p3d_local_planner(robotPt, q1, q2);
  //localpathPt = array_localplanner[lpl_type](robotPt, q1, q2, NULL);

  //  localpathPt = array_localplanner[lpl_type](robotPt, q1, q2, NULL);
  if (localpathPt != NULL){
    length = localpathPt->length_lp;
    localpathPt->destroy(robotPt, localpathPt);
  }
  else {
    length = P3D_HUGE;
  }
  return length;
}

/*
 *  distance between two configurations of a robot
 *
 *  Input:  the robot,
 *          two configurationsl.
 *          the ikSol Vector
 *
 *  Output: the distance
 *
 *  The function computes a local path between q1 and q2 using
 *  the default local method of the robot and returns the length
 *  of the local path.
 */

double p3d_dist_q1_q2_multisol(p3d_rob *robotPt, configPt q1, configPt q2, int* ikSol)
{
  pp3d_localpath localpathPt;
  double length;

  localpathPt = p3d_local_planner_multisol(robotPt, q1, q2, ikSol);

  if (localpathPt != NULL){
    length = localpathPt->length_lp;
    localpathPt->destroy(robotPt, localpathPt);
  }
  else {
    if(p3d_equal_config(robotPt, q1, q2)){
      length = 0;
    }else{
      length = P3D_HUGE;
    }
  }
  return length;
}


/*********************************************************************/


/*
 * destroy a list of localpaths
 *
 * Input: the first local path of the list
 *        the robot
 *
 */
void destroy_list_localpath(p3d_rob *robotPt,
			    p3d_localpath *localpathPt)
{
  p3d_localpath *current_lpPt = localpathPt,
    *next_lpPt;

  while (current_lpPt != NULL){
    next_lpPt = current_lpPt->next_lp;
    current_lpPt->destroy(robotPt, current_lpPt);
    current_lpPt = next_lpPt;
  }
}

/*
 * Concatenation of two lists of local paths
 *
 */

p3d_localpath *concat_liste_localpath(p3d_localpath *list1Pt,
				      p3d_localpath *list2Pt)
{
  p3d_localpath *localpathPt;

  if (list1Pt){
    localpathPt = list1Pt;
    while (localpathPt->next_lp){
      localpathPt = localpathPt->next_lp;
    }
    localpathPt->next_lp = list2Pt;
    if (list2Pt)
      list2Pt->prev_lp = localpathPt;
    return(list1Pt);
  }
  else  return(list2Pt);
}


/*
 *  lm_destroy_params --
 *
 *  destroy the data-structures containing the parameter relative to
 *  each local method
 */

void lm_destroy_params(p3d_rob *robotPt,
		       lm_list_param_str *lm_list_paramPt)
{
  lm_list_param_str *cur_listPt = lm_list_paramPt;
  lm_list_param_str *next_listPt;

  while (cur_listPt != NULL){
    lm_destroy_one_params(robotPt, cur_listPt);
    next_listPt = cur_listPt->next;
    cur_listPt->next = NULL;
    MY_FREE(cur_listPt, lm_list_param_str, 1);
    cur_listPt = next_listPt;
  }
}

/*
 *  lm_destroy_one_params --
 *
 *  destroy the data-structures containing the parameter relative to
 *  one local method
 */

void lm_destroy_one_params(p3d_rob *robotPt,
			   lm_list_param_str *lm_list_paramPt)
{
  void *paramPt = lm_list_paramPt->lm_param;

  switch(lm_list_paramPt->lpl_type) {
  case DUBINS:
  case REEDS_SHEPP:
    lm_destroy_reeds_shepp_params(robotPt, paramPt);
    break;
  case LINEAR:
    lm_destroy_linear_params(robotPt, paramPt);
    break;
  case MANHATTAN:
    lm_destroy_manhattan_params(robotPt, paramPt);
    break;
  case TRAILER:
    lm_destroy_trailer_params(robotPt, paramPt);
    break;
  case HILFLAT:
    lm_destroy_hilflat_params(robotPt, paramPt);
    break;
#ifdef MULTILOCALPATH
  case SOFT_MOTION:
    lm_destroy_softMotion_params(robotPt, paramPt);
    break;
#endif
  case NBLP_TYPE:
    break;
  case TRAILER_FORWARD:
    break;
  case HILFLAT_FORWARD:
    break;
  }
  lm_list_paramPt->lm_param=NULL;
}

/*
 *  lm_append_to_list --
 *
 *  Add a local method at end of a list
 *
 *  Input:  a list,
 *          an element,
 *          a localplanner_type
 *
 *  Output: the input list if it was not empty
 *          a new list otherwise.
 */

lm_list_param_str *lm_append_to_list(lm_list_param_str *listPt, void* eltPt,
				     p3d_localpath_type lpl_type)
{
  lm_list_param_str *new_listPt = NULL;
  lm_list_param_str *end_of_listPt = listPt;

  new_listPt = MY_ALLOC(lm_list_param_str, 1);
  new_listPt->lpl_type = lpl_type;
  new_listPt->next = NULL;
  new_listPt->lm_param = eltPt;

  /* if initial list is empty, return first element */
  if (listPt == NULL){
    return new_listPt;
  }

  /* otherwise go to end of list */
  while (end_of_listPt->next != NULL) {
    end_of_listPt = end_of_listPt->next;
  }
  end_of_listPt->next = new_listPt;

  return listPt;
}

/*
 *  p3d_read_localpath --
 *
 *  read a localpath of given type in input file
 *
 *  ARGS IN  : a file descriptor,
 *             the name of local path type (LINEAR, TRAILER, ...)
 *
 *  ARGS OUT : a localpath if success, NULL otherwise.
 */

p3d_localpath *p3d_read_localpath(p3d_rob *robotPt, FILE *file,
				  char *type, double version)
{
  p3d_localpath *localpathPt=NULL;

  if (strcmp(type, "LINEAR") == 0) {
    localpathPt = p3d_read_linear_localpath(robotPt, file, version);
    return localpathPt;
  }
  else
    if (strcmp(type, "MANHATTAN") == 0) {
      localpathPt = p3d_read_manhattan_localpath(robotPt, file, version);
      return localpathPt;
    }
    else
      if (strcmp(type, "REEDS_SHEPP") == 0) {
	localpathPt = p3d_read_reeds_shepp_localpath(robotPt, file, version);
	return localpathPt;
      }
      else
	if (strcmp(type, "TRAILER") == 0) {
	  localpathPt = p3d_read_trailer_localpath_symmetric(robotPt, file, version);
	  return localpathPt;
	}
	else
	  if (strcmp(type, "TRAILER_NOT_SYMMETRIC") == 0) {
	    localpathPt = p3d_read_trailer_localpath_not_symmetric(robotPt, file,
								   version);
	    return localpathPt;
	  }
	  else
	    if (strcmp(type, "HILFLAT") == 0) {
	      localpathPt = p3d_read_hilflat_localpath_symmetric(robotPt, file, version);
	      return localpathPt;
	    }
	    else
	      if (strcmp(type, "HILFLAT_NOT_SYMMETRIC") == 0) {
		localpathPt = p3d_read_hilflat_localpath_not_symmetric(robotPt,
								       file,
								       version);
		return localpathPt;
	      }
	      else {
		PrintError(("Wrong type of localpath specified\n"));
		return NULL;
	      }
}



/*---------------------------------------------------------------------------*/

/* modif Juan */
/***********************************************************************/
/***********************************************************************/
/*   FUNCTIONS TESTING LOCAL PATHS VALIDITY (CONSTRAINTS + COLLISION)  */
/***********************************************************************/
/***********************************************************************/

int p3d_unvalid_localpath_test(p3d_rob *robotPt, p3d_localpath *localpathPt, int *ntest)
{
  int unvalid = FALSE;

  #ifdef MULTILOCALPATH
//     unvalid = p3d_test_localpath_pb_continuity(robotPt,localpathPt);
  #endif
  
  // NOTE : FUNCTIONS HANDLING MULTIPLE IK SOLUTIONS ARE ONLY MADE YET
  //        FOR CLASSIC (SEQUENTIAL) TEST
  if(unvalid==FALSE){
    unvalid = p3d_col_test_localpath(robotPt,localpathPt,ntest);
  }

  /* When retrieving statistics;Commit Jim; date: 01/10/2008 */
  if(getStatStatus() && XYZ_GRAPH){
    XYZ_GRAPH->stat->planLpColNum++;
  }

  return (unvalid);
}


int p3d_unvalid_localpath_classic_test(p3d_rob *robotPt,
				       p3d_localpath *localpathPt,
				       int *ntest, double *Kpath,
				       configPt *q_atKpath){
  int unvalid;

  unvalid = p3d_col_test_localpath_classic_multisol(robotPt, localpathPt, ntest, Kpath, q_atKpath);

  return (unvalid);
}


int p3d_unvalid_localpath_separated_classic_test(p3d_rob *robotPt,
						 p3d_localpath *localpathPt,
						 int *ntest, double *Kpath)
{
  int unvalid_cntrt=0, unvalid_col=0;
  double Kpath_cntrt=1.0, Kpath_col=1.0;

  unvalid_cntrt = !p3d_cntrt_localpath_classic_test(robotPt, localpathPt, &Kpath_cntrt);

  unvalid_col = p3d_col_test_localpath_classic_multisol(robotPt, localpathPt, ntest, &Kpath_col, NULL);  // WARNING : THIS FUNCTION HAS NOT BEEN TESTED !!!!

  if(Kpath_cntrt < Kpath_col)
    *Kpath = Kpath_cntrt;
  else
    *Kpath = Kpath_col;

  system("xrandt -o inverted");
  return (unvalid_cntrt || unvalid_col);
}

/* fmodif Juan */

void p3d_set_localpath_ikSol(p3d_localpath *localpathPt, p3d_rob* robotPt, int* iksol1, int isSing1, int* iksol2, int isSing2){
  if(iksol1 && iksol2){
    if (isSing1){
      p3d_copy_iksol(robotPt->cntrt_manager,iksol2,&localpathPt->ikSol);
    }else if(isSing2){
      p3d_copy_iksol(robotPt->cntrt_manager,iksol1,&localpathPt->ikSol);
    }else{
      p3d_copy_iksol(robotPt->cntrt_manager,iksol1,&localpathPt->ikSol);
    }
  }
}

/*
 *  append_to_localpath --
 *
 *  put a localpath after another in a list
 *
 *  Input:  the two localpaths
 *
 *  Output: the second localpath
 */
p3d_localpath *append_to_localpath(p3d_localpath *localpath1Pt,
				   p3d_localpath *localpath2Pt)
{
  localpath1Pt->next_lp = localpath2Pt;
  localpath2Pt->prev_lp = localpath1Pt;

  return localpath2Pt;
}

/**
 * @brief Test if two configuration are connectable with a local path creation and validation
 * @param robot : the considered robot
 * @param qStart : the start configuration
 * @param qGoal : the goal configuration
 * @param ntest : the number of tests durring the validation of the localpath
 * @param dist : the length of the localpath (For linear lp the lenght is the distance between the two configs)
 * 
 * @return true if the configurations are connectable
 */
int p3d_connectable_confs(p3d_rob *robot, configPt qStart, configPt qGoal, double * length){
  p3d_localpath * lp = p3d_local_planner(robot, qStart, qGoal);
  if(lp){
    int ntest = 0;
    if(!p3d_unvalid_localpath_test(robot, lp, &ntest)){
      *length = lp->length(robot, lp);
      lp->destroy(robot, lp);
      return TRUE;
    }
  }
  return FALSE;
}
