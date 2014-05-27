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
#include "Graphic-pkg.h"
#ifdef LIGHT_PLANNER
#include "ManipulationArmData.hpp"
#endif

#include <iostream> 
using namespace std;

#define DEBUG_LINEAR 0
/* allocation of a data structure specific to the linear local method */
p3d_lin_data * p3d_alloc_spec_lin_localpath(configPt q_i, configPt q_f)
{
  p3d_lin_data * lin_data;

  if ((lin_data = MY_ALLOC(p3d_lin_data,1)) == NULL)
    return NULL;

  lin_data->q_init = q_i;
  lin_data->q_end = q_f;

  return lin_data;
}

/* allocation of local path of type linear */
p3d_localpath * p3d_alloc_lin_localpath(p3d_rob *robotPt,
					configPt q_i, configPt q_f,
					int lp_id,
					int is_valid)
{
  p3d_localpath * localpathPt = NULL;

  if ((localpathPt = MY_ALLOC(p3d_localpath, 1)) == NULL)
    return NULL;

  /* allocation of the specific part */
  localpathPt->specific.lin_data =
    p3d_alloc_spec_lin_localpath(q_i, q_f);

  if (localpathPt->specific.lin_data == NULL){
    /* allocation failed free everything and return NULL*/
    MY_FREE(localpathPt, p3d_localpath, 1);
    return NULL;
  }
  /* Initialization of the generic part */
  /* fields */
  localpathPt->type_lp = LINEAR;
  localpathPt->valid = is_valid;
  localpathPt->lp_id = lp_id;
  localpathPt->prev_lp = NULL;
  localpathPt->next_lp = NULL;

#ifdef MULTILOCALPATH
	localpathPt->mlpID = -1;
	localpathPt->q_init = NULL;
	for(int j=0; j< MAX_MULTILOCALPATH_NB ; j++) {
		localpathPt->mlpLocalpath[j] = NULL;
	}
#endif

  /* methods associated to the local path */
  /* compute the length of the local path */
  localpathPt->length =
    (double (*)(p3d_rob*, p3d_localpath*))(p3d_lin_dist);
  /* extract from a local path a sub local path starting from length
     l1 and ending at length l2 */
  localpathPt->extract_sub_localpath =
    (p3d_localpath* (*)(p3d_rob*, p3d_localpath*,
			double, double))(p3d_extract_lin);
  /* extract from a local path a sub local path starting from parameter
     u1 and ending at parameter u2 */
  localpathPt->extract_by_param =
    (p3d_localpath* (*)(p3d_rob*, p3d_localpath*,
			double, double))(p3d_extract_lin);
  /* destroy the localpath */
  localpathPt->destroy =
    (void (*)(p3d_rob*, p3d_localpath*))(p3d_lin_destroy);
  /*copy the local path */
  localpathPt->copy =
    (p3d_localpath* (*)(p3d_rob*,
			p3d_localpath*))(p3d_copy_lin_localpath);
  /* computes the configuration at given distance along the path */
  localpathPt->config_at_distance =
    (configPt (*)(p3d_rob*,
		  p3d_localpath*, double))(p3d_lin_config_at_distance);
  /* computes the configuration at given parameter along the path */
  localpathPt->config_at_param =
    (configPt (*)(p3d_rob*, p3d_localpath*,
		  double))(p3d_lin_config_at_distance);
  /* from a configuration on a local path, this function computes an
     interval of parameter on the local path on which all the points
     of the robot move by less than the distance given as input.
     The interval is centered on the configuration given as input. The
     function returns the half length of the interval */
  localpathPt->stay_within_dist =
    (double (*)(p3d_rob*, p3d_localpath*,
		double, whichway, double*))(p3d_lin_stay_within_dist);
  /* compute the cost of a local path */
  localpathPt->cost =
    (double (*)(p3d_rob*, p3d_localpath*))(p3d_lin_cost);
  /* function that simplifies the sequence of two local paths: valid
     only for RS curves */
  localpathPt->simplify =
    (p3d_localpath* (*)(p3d_rob*, p3d_localpath*, int*))(p3d_simplify_lin);
  /* write the local path in a file */
  localpathPt->write =
    (int (*)(FILE *, p3d_rob*, p3d_localpath*))(p3d_write_lin);

  localpathPt->length_lp = p3d_lin_dist(robotPt, localpathPt);
  localpathPt->range_param = localpathPt->length_lp;
  localpathPt->ikSol = NULL;
  //save the active constraints
  localpathPt->nbActiveCntrts = 0;
  localpathPt->activeCntrts = NULL;

#ifdef LIGHT_PLANNER
  localpathPt->isCarryingObject = FALSE;
  for (int i = 0; i < MAX_CARRIED_OBJECTS; i++) {
    localpathPt->carriedObject[i] = NULL;
  }
#endif
  return localpathPt;
}

/* distance for the linear local method */

double p3d_lin_dist(p3d_rob *robotPt, p3d_localpath *localpathPt)
{
  /* cast the pointer to union p3d_lm_specific to a pointer
     to p3d_lin_data */
  p3d_lin_data *specificPt = localpathPt->specific.lin_data;

  /* test whether the type of local path is the expected one */
  if (localpathPt->type_lp != LINEAR){
    PrintError(("p3d_lin_dist: linear local local path expected\n"));
  }
	// Care full this distance does't take into account the 
	// Body length multiplication
  return  p3d_dist_config(robotPt, specificPt->q_init, specificPt->q_end);
}

/*
 * destroys a structure of type p3d_lin_data
 */
void p3d_destroy_lin_data(p3d_rob* robotPt, p3d_lin_data* lin_dataPt)
{
  if (lin_dataPt != NULL){
    if (lin_dataPt->q_init != NULL){
      p3d_destroy_config(robotPt, lin_dataPt->q_init);
      p3d_destroy_config(robotPt, lin_dataPt->q_end);
    }
    MY_FREE(lin_dataPt, p3d_lin_data, 1);
  }
}


/*
 * destroy a linear local path
 */
void p3d_lin_destroy(p3d_rob* robotPt, p3d_localpath* localpathPt)
{
  if (localpathPt != NULL){

    /* test whether the type of local path is the expected one */
    if (localpathPt->type_lp != LINEAR){
      PrintError(("p3d_lin_destroy: linear local path expected\n"));
    }
    /* destroy the specific part */
    if (localpathPt->specific.lin_data != NULL){
      p3d_destroy_lin_data(robotPt, localpathPt->specific.lin_data);
    }
    localpathPt->next_lp = NULL;
    localpathPt->prev_lp = NULL;
    if (localpathPt->ikSol){
      p3d_destroy_specific_iksol(robotPt->cntrt_manager, localpathPt->ikSol);
      localpathPt->ikSol = NULL;
    }
    MY_FREE(localpathPt->activeCntrts, int, localpathPt->nbActiveCntrts);
    MY_FREE(localpathPt, p3d_localpath, 1);
  }
}

/*
 *  Compute the configuration situated at given distance on the local path.
 *
 *  Input:  the robot, the distance.
 *
 *  Output: the configuration
 */
configPt p3d_lin_config_at_distance(p3d_rob *robotPt,
				    p3d_localpath *localpathPt,
				    double distance)
{
  p3d_lin_data *lin_specificPt;
  configPt q_init, q_end, q;
  int i, j, k, njnt = robotPt->njoints;
  double alpha;
  p3d_jnt *jntPt;

  if (localpathPt == NULL)
    return NULL;

  if (localpathPt->type_lp != LINEAR){
    PrintError(("p3d_lin_config_at_distance: local path must be linear\n"));
    return NULL;
  }
  lin_specificPt = localpathPt->specific.lin_data;

  q_init = lin_specificPt->q_init;
  q_end = lin_specificPt->q_end;

//  printf("q[%d] = %f\n", 14, lin_specificPt->q_init[14] );
//  printf("q[%d] = %f\n", 14, lin_specificPt->q_end[14] );

  q = p3d_alloc_config(robotPt);

  if (distance < 0)
    distance = 0;
  if (distance > localpathPt->length_lp)
    distance = localpathPt->length_lp;
  if(localpathPt->length_lp == 0){
    p3d_destroy_config(robotPt, q);
    return p3d_copy_config(robotPt, q_init);
  }
  alpha = distance / localpathPt->length_lp;

#if defined(MULTILOCALPATH)
	/* translation parameters of main body */
	Gb_dep dep_i;
	Gb_dep dep_e;
	Gb_quat quat_i;
	Gb_quat quat_e;
	Gb_quat quat_o;
	p3d_matrix4 m_i, m_e;
	p3d_matrix4 mat_o;
	Gb_th th_i, th_e, th_o;


  /* translation parameters of main body */
  for (i=0; i<=njnt; i++) {
    jntPt = robotPt->joints[i];
		if(jntPt->type == P3D_FREEFLYER) {
			k = jntPt->index_dof;
      //configPt qsave = ((p3d_rob *) p3d_get_desc_curid(P3D_ROBOT))->ROBOT_POS;
			p3d_mat4PosReverseOrder(m_i, q_init[k], q_init[k+1], q_init[k+2], q_init[k+3], q_init[k+4], q_init[k+5]);
			p3d_mat4PosReverseOrder(m_e, q_end[k], q_end[k+1], q_end[k+2], q_end[k+3], q_end[k+4], q_end[k+5]);
      //p3d_set_and_update_robot_conf(q_init);
			lm_convert_p3dMatrix_To_GbTh(m_i,&th_i);
			lm_convert_p3dMatrix_To_GbTh(m_e,&th_e);
			Gb_th_dep(&th_i, &dep_i);
			Gb_th_dep(&th_e, &dep_e);
			Gb_dep_quat(&dep_i, &quat_i);
			Gb_dep_quat(&dep_e, &quat_e);
//	  printf("quat_e %f, %f, %f, %f\n", quat_e.vx, quat_e.vy, quat_e.vz, quat_e.w);
// 		printf("alpha  %f\n", alpha);
			Gb_quat_interpole(&quat_i, &quat_e, alpha, &quat_o);
// 		printf("quat_o %f, %f, %f, %f, %f, %f, %f\n",  quat_o.x, quat_o.y, quat_o.z,quat_o.vx, quat_o.vy, quat_o.vz, quat_o.w);
			Gb_quat_th(&quat_o, &th_o);
			lm_convert_GbTh_To_p3dMatrix(&th_o, mat_o);
// 		printf("mat_ox %f, %f, %f, %f\n", mat_o[0][0], mat_o[0][1], mat_o[0][2], mat_o[0][3]);
// 		printf("mat_oy %f, %f, %f, %f\n", mat_o[1][0], mat_o[1][1], mat_o[1][2], mat_o[1][3]);
//  	printf("mat_oz %f, %f, %f, %f\n", mat_o[2][0], mat_o[2][1], mat_o[2][2], mat_o[2][3]);
// 		printf("mat_op %f, %f, %f\n", mat_o[0][3], mat_o[1][3], mat_o[2][3]);
			p3d_mat4ExtractPosReverseOrder(mat_o, q+k, q+k+1, q+k+2, q+k+3, q+k+4, q+k+5);
// 		printf("q_k  %f, %f, %f, %f, %f, %f\n -------\n",q[k], q[k+1], q[k+2], q[k+3], q[k+4], q[k+5]);
		}
		else {
			for (j=0; j<jntPt->dof_equiv_nbr; j++) {
				k = jntPt->index_dof+j;
				q[k] = p3d_jnt_calc_dof_value(jntPt, j, q_init, q_end, alpha);
			}
		}
	}
#else
	/* translation parameters of main body */
	for (i=0; i<=njnt; i++) {
		jntPt = robotPt->joints[i];
		for (j=0; j<jntPt->dof_equiv_nbr; j++) {
			k = jntPt->index_dof+j;
			q[k] = p3d_jnt_calc_dof_value(jntPt, j, q_init, q_end, alpha);
			//cout << "Joint " << k << " q : " << q[k] << endl;
		}
	}
#endif
  return q;
}


/*  p3d_lin_stay_within_dist
 *
 *  Input:  the robot,
 *          the local path,
 *          the parameter along the curve,
 *          the direction of motion
 *          the maximal distance moved by all the points of the
 *          robot
 *
 *  Output: length of the interval of parameter the robot can
 *          stay on without any body moving by more than the input distance
 *
 *  Description:
 *          From a configuration on a local path, this function
 *          computes an interval of parameter on the local path on
 *          which all the points of the robot move by less than the
 *          distance given as input.  The interval is centered on the
 *          configuration given as input. The function returns the
 *          half length of the interval
 */
double p3d_lin_stay_within_dist(p3d_rob* robotPt,
				p3d_localpath* localpathPt,
				double parameter, whichway dir,
				double *distances)
{
  p3d_lin_data *lin_localpathPt=NULL;
  int i, j, njnt = robotPt->njoints;
  p3d_jnt *cur_jntPt, *prev_jntPt;
  configPt q_max_param, q_param;
  double max_param, min_param;
  double range_param = localpathPt->length_lp;
  p3d_stay_within_dist_data * stay_within_dist_data;

  /* local path has to be of type linear */
  if (localpathPt->type_lp != LINEAR){
    PrintError(("p3d_lin_stay_within_dist: linear local path expected\n"));
    return 0;
  }
  //on recupere les donnees du localpath
  lin_localpathPt = localpathPt->specific.lin_data;

  /* store the data to compute the maximal velocities at the
     joint for each body of the robot */
  stay_within_dist_data = MY_ALLOC(p3d_stay_within_dist_data, njnt+2); /** ALLOC **/
  p3d_init_stay_within_dist_data(stay_within_dist_data);

  if (dir == FORWARD)
  {
    q_max_param = lin_localpathPt->q_end;
    min_param = max_param = range_param - parameter;
  } else
  {
    q_max_param = lin_localpathPt->q_init;
    min_param = max_param = parameter;
  }
  /* Get the current config to have the modifications of the constraints */
  /* Supose that q_init and q_goal respect cronstraints */
  q_param = p3d_get_robot_config(robotPt);  /** ALLOC **/
//  q_param = localpathPt->config_at_param(robotPt,localpathPt,parameter);

  /* computation of the bounds for the linear and angular
     velocities of each body */

  int minJnt = 0;
  for(i=0; i<=njnt; i++) {
    cur_jntPt = robotPt->joints[i];
    prev_jntPt = cur_jntPt->prev_jnt;

    /* j = index of the joint to which the current joint is attached */
    if (prev_jntPt==NULL)
      { j = -1; } /* environment */
    else
      { j = prev_jntPt->num; }
    double bakMinParam = min_param;

    p3d_jnt_stay_within_dist(&(stay_within_dist_data[j+1]), cur_jntPt,
			     &(stay_within_dist_data[i+1]), &(distances[i]),
			     q_param, q_max_param, max_param, &min_param);
    if (min_param < bakMinParam){
      minJnt = cur_jntPt->num;
    }


    /* Rem: stay_within_dist_data[0] is bound to the environment */
  }
  if (DEBUG_LINEAR == 1){
    printf("minjnt = %d\n", minJnt);
  }
  MY_FREE(stay_within_dist_data, p3d_stay_within_dist_data, njnt+2);  /** FREE **/
  p3d_destroy_config(robotPt, q_param);  /** FREE **/

  return min_param;
}


/*
 *  Copy one local path.
 *
 *  Input:  the robot, the local path.
 *
 *  Output: the copied local path
 */

p3d_localpath *p3d_copy_lin_localpath(p3d_rob* robotPt,
				      p3d_localpath* localpathPt)
{
  p3d_localpath *lin_localpathPt;
  configPt q_i, q_f;
  int lp_id = localpathPt->lp_id;
  int is_valid = localpathPt->valid;

  q_i = p3d_copy_config(robotPt, localpathPt->specific.lin_data->q_init);
  q_f = p3d_copy_config(robotPt, localpathPt->specific.lin_data->q_end);

  lin_localpathPt = p3d_alloc_lin_localpath(robotPt, q_i, q_f,
					    lp_id, is_valid);

  /* update length and range of parameter */
  lin_localpathPt->length_lp = p3d_lin_dist(robotPt, lin_localpathPt);
  lin_localpathPt->range_param = lin_localpathPt->length_lp;
  p3d_copy_iksol(robotPt->cntrt_manager, localpathPt->ikSol, &(lin_localpathPt->ikSol));
  lin_localpathPt->nbActiveCntrts = localpathPt->nbActiveCntrts;
  lin_localpathPt->activeCntrts = MY_ALLOC(int, lin_localpathPt->nbActiveCntrts);
  for(int i = 0; i < lin_localpathPt->nbActiveCntrts; i++){
    lin_localpathPt->activeCntrts[i] = localpathPt->activeCntrts[i];
  }
  
#ifdef LIGHT_PLANNER
  lin_localpathPt->isCarryingObject = localpathPt->isCarryingObject;
  for (int i = 0; i < MAX_CARRIED_OBJECTS; i++) {
    lin_localpathPt->carriedObject[i] = localpathPt->carriedObject[i];
  }
#endif
  
  return lin_localpathPt;
}

/*
 *  Extract from a linear local path the sub local path starting
 *  at length l1 and ending at length l2.
 *  The length of the extracted local path is computed
 *
 *  If l2 > length local path, return end of local path
 */
p3d_localpath *p3d_extract_lin(p3d_rob *robotPt,
				p3d_localpath *localpathPt,
				double l1, double l2)
{
  configPt q1, q2;
  p3d_localpath *sub_localpathPt;

  q1 = p3d_lin_config_at_distance(robotPt, localpathPt, l1);
  if (robotPt->cntrt_manager->cntrts != NULL) {
    p3d_set_and_update_this_robot_conf_multisol(robotPt, q1, NULL, 0, localpathPt->ikSol);
    p3d_get_robot_config_into(robotPt, &q1);
  }
  q2 = p3d_lin_config_at_distance(robotPt, localpathPt, l2);
  if (robotPt->cntrt_manager->cntrts != NULL) {
    p3d_set_and_update_this_robot_conf_multisol(robotPt, q2, NULL, 0, localpathPt->ikSol);
    p3d_get_robot_config_into(robotPt, &q2);
  }

  sub_localpathPt = p3d_alloc_lin_localpath(robotPt, q1, q2, FORWARD,
					    localpathPt->valid);
  p3d_copy_iksol(robotPt->cntrt_manager, localpathPt->ikSol, &(sub_localpathPt->ikSol));
  sub_localpathPt->nbActiveCntrts = localpathPt->nbActiveCntrts;
  sub_localpathPt->activeCntrts = MY_ALLOC(int, sub_localpathPt->nbActiveCntrts);
  for(int i = 0; i < sub_localpathPt->nbActiveCntrts; i++){
    sub_localpathPt->activeCntrts[i] = localpathPt->activeCntrts[i];
  }
  return sub_localpathPt;
}

/*
 *  Cost of a local path
 *
 *  Input:  the local path
 *
 *  Output: the cost
 */
double p3d_lin_cost(p3d_rob *robotPt, p3d_localpath *localpathPt)
{
  return localpathPt->length_lp;
}

/*
 *  does nothing
 */

p3d_localpath *p3d_simplify_lin(p3d_rob *robotPt, p3d_localpath *localpathPt,
				int *need_colcheck)
{
  return localpathPt;
}


/*
 *  p3d_write_lin --
 *
 *  write a localpath of type linear in a file
 *
 *  ARGS IN  : a file descriptor,
 *             a robot,
 *             a localpath
 *
 *  ARGS OUT : TRUE if success,
 *             FALSE if input local path is not a linear one.
 */

int p3d_write_lin(FILE *file, p3d_rob* robotPt, p3d_localpath* localpathPt)
{
  p3d_lin_data *lin_dataPt = NULL;

  if (localpathPt->type_lp != LINEAR) {
    return FALSE;
  }

  fprintf(file, "\n\np3d_add_localpath LINEAR\n");

  lin_dataPt = (pp3d_lin_data)localpathPt->specific.lin_data;
  /* write each RS segment */
  fprintf(file, "conf_init");
  fprint_config_one_line(file, robotPt, lin_dataPt->q_init);
  fprintf(file, "\nconf_end ");
  fprint_config_one_line(file, robotPt, lin_dataPt->q_end);
  fprintf(file, "\n");

  fprintf(file, "\np3d_end_local_path\n");

  return TRUE;
}

/*
 *  Debugging functions
 */

void print_LIN(p3d_rob *robotPt, p3d_localpath *localpathPt)
{
  int i, j;

  p3d_lin_data *lin_specificPt=NULL;

  if (localpathPt == NULL){
    PrintInfo(("Local path null\n"));
    return;
  }

  /* local path has to be of type linear */
  if (localpathPt->type_lp != LINEAR){
    PrintInfo(("print_LIN: linear local path expected\n"));
    return;
  }

  i=1;
  while(localpathPt != NULL){
    PrintInfo(("\n *********************************\n  Local path number %d\n\n", i));
    lin_specificPt = localpathPt->specific.lin_data;
    j=1;

    PrintInfo(("    q_init\n"));
    print_config(robotPt, lin_specificPt->q_init);

    PrintInfo(("    q_end\n"));
    print_config(robotPt, lin_specificPt->q_end);
    PrintInfo(("\n"));

    localpathPt = localpathPt->next_lp;
    i++;
  }
}

/*
 * Linear local planner
 *
 * Input:  the robot, two configurations
 *
 * Output: a local path.
 *
 * Allocation: the initial and goal config are copied
 */
p3d_localpath *p3d_linear_localplanner(p3d_rob *robotPt, configPt qi,
				       configPt qf, int* ikSol)
{
  configPt initconfPt, goalconfPt;
  p3d_localpath *localpathPt;

  /* on verifie que les configurations de depart et d'arrivee existent */
  if(qi == NULL){
    PrintInfo((("MP: p3d_linear_localplanner: no start configuration...\n")));
    p3d_set_search_status(P3D_ILLEGAL_START);
    return(NULL);
  }
  if(qf == NULL){
    PrintInfo((("MP: p3d_linear_localplanner: no goal configuration...\n")));
    p3d_set_search_status(P3D_ILLEGAL_GOAL);
    return(NULL);
  }

  /* copy of qi into initconfPt and of qf into goalconfPt */
  initconfPt = p3d_copy_config(robotPt, qi);
  goalconfPt = p3d_copy_config(robotPt, qf);
//     initconfPt = qi;
//     goalconfPt = qf;
  if(p3d_get_search_verbose()){
      PrintInfo(("MP: p3d_linear_localplanner : "));
      PrintInfo(("qi=("));
      print_config(robotPt, initconfPt);
      PrintInfo((") ; "));
      PrintInfo(("qf=("));
      print_config(robotPt, goalconfPt);
      PrintInfo((")\n"));
  }

  /* If initconfPt == goalconfPt, free initconfPt and goalconfPt
     and return NULL */
  if(p3d_equal_config(robotPt,initconfPt, goalconfPt)) {
    PrintInfo((("MP: p3d_linear_localplanner: q_init = q_goal!\n")));
    p3d_set_search_status(P3D_CONFIG_EQUAL);
    p3d_destroy_config(robotPt, initconfPt);
    p3d_destroy_config(robotPt, goalconfPt);
    return(NULL);
  }

  /* Modif NIC
  on verifie que les initconfPt et goalconfPt sont valides
  p3d_set_robot_config(robotPt, initconfPt);
  p3d_update_robot_pos();
  if(p3d_col_test()) {
    PrintInfo(("MP: p3d_linear_localplanner: Illegal q_init\n"));
  }

  p3d_set_robot_config(robotPt, goalconfPt);
  p3d_update_robot_pos();
  if(p3d_col_test()){
    PrintInfo(("MP: p3d_linear_localplanner: Illegal q_goal\n"));
  }
  */

  localpathPt = p3d_alloc_lin_localpath(robotPt, initconfPt, goalconfPt,
					0, TRUE);

  p3d_set_search_status(P3D_SUCCESS);

  localpathPt->ikSol = ikSol;
  localpathPt->activeCntrts = p3d_getActiveCntrts(robotPt,&(localpathPt->nbActiveCntrts));
  
#if defined(LIGHT_PLANNER)
  localpathPt->isCarryingObject = robotPt->isCarryingObject;
  for (int i = 0; i < MAX_CARRIED_OBJECTS; i++){
    if(i < robotPt->armManipulationData->size() ){
      localpathPt->carriedObject[i] = (*robotPt->armManipulationData)[i].getCarriedObject(); /*!< pointer to the carried object (obstacle environment or robot body) */
    }
  }
	//p3d_mat4Copy(robotPt->Tgrasp, localpathMg->Tgrasp);
#endif
  
  return(localpathPt);
}

void lm_destroy_linear_params(p3d_rob *robotPt, void *paramPt)
{
}

/*
 *  read_linear_localpath --
 *
 *  build a linear local path and read the data specific this  local path
 *  in a file.
 *
 *  ARGS IN  : the file descriptor
 *
 *  ARGS OUT : a local path or NULL if error
 */

p3d_localpath *p3d_read_linear_localpath(p3d_rob *robotPt, FILE *file,
					 double version)
{
  p3d_localpath *localpathPt = NULL;
  int size_max_line=0;
  char *pos=NULL, *line=NULL, *name=NULL;
  configPt q_init=NULL, q_end=NULL;
  int success=TRUE;
  int num_line=0;

  /*
   *  look for conf_init
   */

  /* read a line */
  if ((size_max_line = p3d_read_line_next_function(file, &line,
						   size_max_line,
						   &num_line)) == 0) {
    PrintWarning(("line %d: expecting initial configuration\n", num_line));
    success=FALSE;
  }
  pos = line;

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

  if (success) {
    /* read next line */
    if ((size_max_line = p3d_read_line_next_function(file, &line,
						     size_max_line,
						     &num_line)) == 0) {
      PrintWarning(("line %d: expecting end configuration\n", num_line));
      success=FALSE;
    }
    pos = line;
  }

  if (success) {
    if ((q_end = p3d_read_word_and_config(robotPt, line,
					  (char*)"conf_end", version)) == NULL) {
      PrintWarning(("line %d: expecting end configuration\n", num_line));
      success = FALSE;
    }
  }

  /* look for p3d_end_local_path */
  if (success) {
    if ((size_max_line = p3d_read_line_next_function(file, &line,
						     size_max_line,
						     &num_line)) == 0) {
      PrintWarning(("line %d: expecting command p3d_end_local_path\n",
		  num_line));
      success=FALSE;
    }
  }
  if (success) {
    if (p3d_read_string_name(&pos, &name) != TRUE) {
      PrintWarning(("line %d: expecting command p3d_end_local_path\n",
		  num_line));
      success=FALSE;
    }
  }
  if (success) {
    if (strcmp(name, "p3d_end_local_path") != 0) {
      PrintWarning(("line %d: expecting command p3d_end_local_path\n",
		    num_line));
      success=FALSE;
    }
  }

  if (success == TRUE) {
    localpathPt = p3d_alloc_lin_localpath(robotPt, q_init, q_end, 0, TRUE);
  }

  if (success == FALSE) {
    /* there has been a problem desallocate configurations */
    if (q_init != NULL) {
      p3d_destroy_config(robotPt, q_init);
    }
    if (q_end != NULL) {
      p3d_destroy_config(robotPt, q_end);
    }
  }

  return localpathPt;

}
