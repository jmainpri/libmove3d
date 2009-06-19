#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Localpath-pkg.h"

/* allocation of a data structure specific to the manhattan local path */
p3d_manh_data * p3d_alloc_spec_manh_localpath(configPt q_i, configPt q_f,
				     configPt crit_q_i, configPt crit_q_f)
{
  p3d_manh_data * manh_data;

  if ((manh_data = MY_ALLOC(p3d_manh_data,1)) == NULL)
    return NULL;

  manh_data->crit_q_init = crit_q_i;
  manh_data->crit_q_end = crit_q_f;

  manh_data->q_init = q_i;
  manh_data->q_end = q_f;

  return manh_data;
}

/* allocation of local path of type manhattan */
p3d_localpath * p3d_alloc_manh_localpath(p3d_rob *robotPt,
					 configPt q_i, configPt q_f,
					 configPt crit_q_i, configPt crit_q_f,
					 int lp_id,
					 int is_valid)
{
  p3d_localpath * localpathPt = NULL;

  if ((localpathPt = MY_ALLOC(p3d_localpath,1)) == NULL)
    return NULL;

  /* allocation of the specific part */
  localpathPt->specific.manh_data =
    p3d_alloc_spec_manh_localpath(q_i,q_f,crit_q_i,crit_q_f);

  if (localpathPt->specific.manh_data == NULL){
    /* allocation failed free everything and return NULL*/
    MY_FREE(localpathPt, p3d_localpath, 1);
    return NULL;
  }
  /* allocation of the generic part */
  localpathPt->type_lp = MANHATTAN;
  localpathPt->valid = is_valid;
  localpathPt->lp_id = lp_id;
  localpathPt->prev_lp = NULL;
  localpathPt->next_lp = NULL;

#ifdef MULTILOCALPATH
	localpathPt->mlpID = -1;

	for(int j=0; j< MAX_MULTILOCALPATH_NB ; j++) {
		localpathPt->mlpLocalpath[j] = NULL;
	}
#endif

  /* methods associated to the local path */
  /* compute the length of the local path */
  localpathPt->length =
    (double (*)(p3d_rob*, p3d_localpath*))(p3d_manh_dist);
  /* extract from a local path a sub local path starting from length
     l1 and ending at length l2 */
  localpathPt->extract_sub_localpath =
    (p3d_localpath* (*)(p3d_rob*, p3d_localpath*,
			double, double))(p3d_extract_manh);
  /* extract from a local path a sub local path starting from parameter
     u1 and ending at parameter u2 */
  localpathPt->extract_by_param =
    (p3d_localpath* (*)(p3d_rob*, p3d_localpath*,
			double, double))(p3d_extract_manh);
  /* destroy the localpath */
  localpathPt->destroy =
    (void (*)(p3d_rob*, p3d_localpath*))(p3d_manh_destroy);
  /*copy the local path */
  localpathPt->copy =
    (p3d_localpath* (*)(p3d_rob*,
			p3d_localpath*))(p3d_copy_manh_localpath);
  /* computes the configuration at given distance along the path */
  localpathPt->config_at_distance =
    (configPt (*)(p3d_rob*,
		  p3d_localpath*, double))(p3d_manh_config_at_distance);
  /* computes the configuration at given parameter along the path */
  localpathPt->config_at_param =
    (configPt (*)(p3d_rob*, p3d_localpath*,
		  double))(p3d_manh_config_at_distance);
  /* from a configuration on a local path, this function computes an
     interval of parameter on the local path on which all the points
     of the robot move by less than the distance given as input.
     The interval is centered on the configuration given as input. The
     function returns the half length of the interval */
  localpathPt->stay_within_dist =
    (double (*)(p3d_rob*, p3d_localpath*,
		double, whichway, double*))(p3d_manh_stay_within_dist);
  /* compute the cost of a local path */
  localpathPt->cost =
    (double (*)(p3d_rob*, p3d_localpath*))(p3d_manh_cost);
  /* function that simplifies the sequence of two local paths: valid
     only for RS curves */
  localpathPt->simplify =
    (p3d_localpath* (*)(p3d_rob*, p3d_localpath*, int*))(p3d_simplify_manh);
  /* write the local path in a file */
  localpathPt->write =
    (int (*)(FILE *, p3d_rob*, p3d_localpath*))(p3d_write_manh);

  localpathPt->length_lp = p3d_manh_dist(robotPt, localpathPt);
  localpathPt->range_param = localpathPt->length_lp;
  localpathPt->ikSol = NULL;
  localpathPt->nbActiveCntrts = 0;
  localpathPt->activeCntrts = NULL;
  return localpathPt;
}

/* distance for the manhattan local method */

double p3d_manh_dist(p3d_rob *robotPt, p3d_localpath *localpathPt)
{
  double ljnt=0;
  int i, j, njnt = robotPt->njoints;
  p3d_manh_data *specificPt = NULL;
  p3d_jnt * jntPt;

  /* cast the pointer to union p3d_lm_specific to a pointer
     to p3d_manh_data */
  specificPt = localpathPt->specific.manh_data;

  /* test whether the type of local path is the expected one */
  if (localpathPt->type_lp != MANHATTAN){
    PrintError(("p3d_manh_dist: manhattan local local path expected\n"));
  }
  ljnt = 0.;
  for(i=0; i<=njnt; i++) {
    jntPt = robotPt->joints[i];
    for(j=0; j<jntPt->dof_equiv_nbr; j++) {
      ljnt += p3d_jnt_calc_dof_dist(jntPt, j, specificPt->q_init,
				    specificPt->q_end);
    }
  }
  return(ljnt);
}

/*
 * destroys a structure of type p3d_rs_data and the structures pointed by
 * the successive fields next_rs of this structure
 */
void p3d_destroy_manh_data(p3d_rob* robotPt, p3d_manh_data* manh_dataPt)
{
  if (manh_dataPt != NULL){
    if (manh_dataPt->q_init != NULL){
      p3d_destroy_config(robotPt, manh_dataPt->q_init);
      p3d_destroy_config(robotPt, manh_dataPt->q_end);
      p3d_destroy_config(robotPt, manh_dataPt->crit_q_init);
      p3d_destroy_config(robotPt, manh_dataPt->crit_q_end);
    }
    MY_FREE(manh_dataPt, p3d_manh_data, 1);
  }
}


/*
 * destroy a manhattan local path
 */
void p3d_manh_destroy(p3d_rob* robotPt, p3d_localpath* localpathPt)
{
  if (localpathPt != NULL){

    /* test whether the type of local path is the expected one */
    if (localpathPt->type_lp != MANHATTAN){
      PrintError(("p3d_manh_destroy: Manhattan local path expected\n"));
      return;
    }
    /* destroy the specific part */
    if (localpathPt->specific.manh_data != NULL){
      p3d_destroy_manh_data(robotPt, localpathPt->specific.manh_data);
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


configPt p3d_manh_config_at_distance(p3d_rob *robotPt,
				     p3d_localpath *localpathPt,
				     double length)
{
  p3d_manh_data *manh_specificPt;
  configPt crit_q_init, crit_q_end, q_init, q_end, q;
  int i, j, k, njnt = robotPt->njoints;
  int nb_dof = robotPt->nb_dof;
  double qifirst=0, qffirst=0;
  double dl;
  int found = FALSE;
  p3d_jnt * jntPt;

  if (localpathPt->type_lp != MANHATTAN){
    PrintError(("p3d_manh_config_at_distance: local path must be manhattan\n"));
    return NULL;
  }
  manh_specificPt = localpathPt->specific.manh_data;

  q_init = manh_specificPt->q_init;
  q_end  = manh_specificPt->q_end ;
  crit_q_init = manh_specificPt->crit_q_init;
  crit_q_end  = manh_specificPt->crit_q_end ;

  /* the first moving joint determines the order of motions of the
     joints */
  found = FALSE;
  for(i=0;(i<nb_dof)&&(!found);i++){
    if(fabs(crit_q_end[i]-crit_q_init[i])>EPS6){
      qifirst = crit_q_init[i];
      qffirst = crit_q_end[i];
      found = TRUE;
    }
  }
  if(!found)
    { PrintError(("MANHATTAN LOCAL PLANNER: sens not found\n")); }


  q = p3d_copy_config(robotPt, q_init);

  if(fabs(length)<EPS6)
    { return q; }

  if(qifirst<=qffirst){
    for(i=0; i<=njnt; i++) {
      jntPt = robotPt->joints[i];
      for(j=0; j<jntPt->dof_equiv_nbr; j++) {
	k = jntPt->index_dof+j;
	dl = p3d_jnt_calc_dof_dist(jntPt, j, q_init, q_end);
	if (length > dl) {
	  length -= dl;
	  q[k] = q_end[k];
	} else {
	  q[k] = p3d_jnt_calc_dof_value(jntPt, j, q_init, q_end, length/dl);
	  return q;
	}
      }
    }
  } else {
    for(i=njnt; i>=0; i--) {
      jntPt = robotPt->joints[i];
      for(j=jntPt->dof_equiv_nbr-1; j>=0; j--) {
	k = jntPt->index_dof+j;
	dl = p3d_jnt_calc_dof_dist(jntPt, j, q_init, q_end);
	if (length > dl) {
	  length -= dl;
	  q[k] = q_end[k];
	} else {
	  q[k] = p3d_jnt_calc_dof_value(jntPt, j, q_init, q_end, length/dl);
	  return q;
	}
      }
    }
  }

  return q;
}

/*
 *  Copy one local path.
 *
 *  Input:  the robot, the local path.
 *
 *  Output: the copied local path
 */

p3d_localpath *p3d_copy_manh_localpath(p3d_rob* robotPt,
				       p3d_localpath* localpathPt)
{
  p3d_localpath *copy_localpathPt;
  p3d_manh_data *manh_specificPt=NULL;
  configPt q1, q2, crit_q1, crit_q2;

  if (localpathPt == NULL){
    return NULL;
  }
  /* test whether the type of local method is the expected one */
  if (localpathPt->type_lp != MANHATTAN){
    PrintError(("p3d_copy_manh_localpath: manhattan local path expected\n"));
    return NULL;
  }

  manh_specificPt = localpathPt->specific.manh_data;
  q1 = p3d_copy_config(robotPt, manh_specificPt->q_init);
  q2 = p3d_copy_config(robotPt, manh_specificPt->q_end);
  crit_q1 = p3d_copy_config(robotPt, manh_specificPt->crit_q_init);
  crit_q2 = p3d_copy_config(robotPt, manh_specificPt->crit_q_end);

  copy_localpathPt = p3d_alloc_manh_localpath(robotPt, q1, q2,
					      crit_q1, crit_q2, 0,
					      localpathPt->valid);
  p3d_copy_iksol(robotPt->cntrt_manager, localpathPt->ikSol, &(copy_localpathPt->ikSol));
  copy_localpathPt->nbActiveCntrts = localpathPt->nbActiveCntrts;
  copy_localpathPt->activeCntrts = MY_ALLOC(int, copy_localpathPt->nbActiveCntrts);
  for(int i = 0; i < copy_localpathPt->nbActiveCntrts; i++){
    copy_localpathPt->activeCntrts[i] = localpathPt->activeCntrts[i];
  }
  return copy_localpathPt;
}

/*
 *  Extract from a manhattan local path the sub local path starting
 *  at length l1 and ending at length l2.
 *  The length of the extracted local path is computed
 *
 *  If l2 > length local path, return initial local path
 */

p3d_localpath *p3d_extract_manh(p3d_rob *robotPt,
				p3d_localpath *localpathPt,
				double l1, double l2)
{
  configPt q1, q2, q3, q4;
  p3d_manh_data *manh_specificPt=NULL;
  p3d_localpath *sub_localpathPt;

  manh_specificPt = localpathPt->specific.manh_data;
  q3 = p3d_copy_config(robotPt, manh_specificPt->crit_q_init);
  q4 = p3d_copy_config(robotPt, manh_specificPt->crit_q_end);

  q1 = p3d_manh_config_at_distance(robotPt, localpathPt, l1);
  if (robotPt->cntrt_manager->cntrts != NULL) {
    p3d_set_and_update_this_robot_conf_multisol(robotPt, q1, NULL, 0, localpathPt->ikSol);
    p3d_get_robot_config_into(robotPt, &q1);
  }
  q2 = p3d_manh_config_at_distance(robotPt, localpathPt, l2);
  if (robotPt->cntrt_manager->cntrts != NULL) {
    p3d_set_and_update_this_robot_conf_multisol(robotPt, q2, NULL, 0, localpathPt->ikSol);
    p3d_get_robot_config_into(robotPt, &q2);
  }

  sub_localpathPt = p3d_alloc_manh_localpath(robotPt, q1, q2, q3, q4, 0,
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
 * Initialize some parameters for p3d_manh_stay_within_dist
 * return the current actif joint
 */
static int p3d_init_stay_within_dist_parameters(
			 p3d_rob * robotPt, int sens, configPt q_param,
			 configPt q_max_param, double * parameter,
			 double * len_param, configPt q_init, configPt q_end)
{
  int i, j, k;
  int njnt = robotPt->njoints;
  p3d_jnt * jntPt;

  if (sens>0) {
    for(i=0; i<=njnt; i++) {
      jntPt = robotPt->joints[i];
      for(j=0; j<jntPt->dof_equiv_nbr; j++) {
	k = jntPt->index_dof+j;
	if ((len_param[k]<=EPS6) || (*parameter>=len_param[k])) {
	  *parameter -= len_param[k];
	  q_param[k] = q_max_param[k] = q_end[k];
	} else {
	  q_param[k] = p3d_jnt_calc_dof_value(jntPt, j, q_init, q_end,
					      *parameter/len_param[k]);
	  q_max_param[k] = q_end[k];
	  return k;
	}
      }
    }
  } else {
    for(i=njnt; i>=0; i--) {
      jntPt = robotPt->joints[i];
      for(j=jntPt->dof_equiv_nbr-1; j>=0 ; j--) {
	k = jntPt->index_dof+j;
	if ((len_param[k]<=EPS6) || (*parameter>=len_param[k])) {
	  *parameter -= len_param[k];
	  q_param[k] = q_max_param[k] = q_end[k];
	} else {
	  q_param[k] = p3d_jnt_calc_dof_value(jntPt, j, q_init, q_end,
					      *parameter/len_param[k]);
	  q_max_param[k] = q_end[k];
	  return k;
	}
      }
    }
  }
  return -1;
}


/*  p3d_manh_stay_within_dist
 *
 *  Input:  the robot,
 *          the local path,
 *          the parameter along the curve,
 *          the maximal distance moved by all the points of the
 *          robot
 *
 *  Output: half length of the interval of parameter the robot can
 *          describe without moving by more than the input distance
 *
 *  Description: This function is a copy of the same function for
 *          linear local paths.
 *
 */
double p3d_manh_stay_within_dist(p3d_rob* robotPt,
				p3d_localpath* localpathPt,
				double parameter, whichway dir,
				double *distances)
{
  p3d_manh_data *manh_localpathPt=NULL;
  double * len_param;
  int i, j, k, njnt = robotPt->njoints;
  int nb_dof = robotPt->nb_dof;
  p3d_jnt *cur_jntPt, *prev_jntPt, * jntPt;
  configPt q_end, q_init;
  configPt q_max_param, q_param;
  double max_param, min_param, tot_max_param;
  double range_param = localpathPt->length_lp;
  p3d_stay_within_dist_data * stay_within_dist_data;
  int sens, actif_dof;

  /* local path has to be of type manhattan */
  if (localpathPt->type_lp != MANHATTAN){
    PrintError(("p3d_manh_stay_within_dist: manhattan local path expected\n"));
    return 0;
  }

  manh_localpathPt = localpathPt->specific.manh_data;

  /* store the data to compute the maximal velocities at the
     joint for each body of the robot */
  stay_within_dist_data = MY_ALLOC(p3d_stay_within_dist_data, njnt+2);
  p3d_init_stay_within_dist_data(stay_within_dist_data);

  len_param = MY_ALLOC(double, nb_dof);
  for(i=0; i<nb_dof; i++)
    { len_param[i] = 0.; }

  q_init = manh_localpathPt->q_init;
  q_end = manh_localpathPt->q_end;

  sens = 0;
  for(i=0; i<=njnt; i++) {
    jntPt = robotPt->joints[i];
    for(j=0; j<jntPt->dof_equiv_nbr; j++) {
      k = jntPt->index_dof+j;
      len_param[k] = p3d_jnt_calc_dof_dist(jntPt, j, q_init, q_end);
      if (!sens && (len_param[k]>EPS6)) {
	if (q_end[k]>=q_init[k])
	  { sens = FORWARD; }
	else
	  { sens = BACKWARD; }
      }
    }
  }

  if (dir == BACKWARD) {
    q_init = manh_localpathPt->q_init;
    q_end = manh_localpathPt->q_end;
    parameter = range_param-parameter;
  }
  q_param = p3d_copy_config(robotPt, q_init);
  q_max_param = p3d_copy_config(robotPt, q_init);
  sens *= dir;

  actif_dof = p3d_init_stay_within_dist_parameters(
			 robotPt, sens, q_param, q_max_param,
			 &parameter, len_param, q_init, q_end) ;

  min_param = max_param = len_param[actif_dof] - parameter;
  tot_max_param = 0.;

  while((min_param>=0) && (actif_dof>=0) && (actif_dof<nb_dof)) {
    /* computation of the bounds for the linear and angular
       velocities of each body */
    for(i=0; i<=njnt; i++) {
      cur_jntPt = robotPt->joints[i];
      prev_jntPt = cur_jntPt->prev_jnt;

      /* j = index of the joint to which the current joint is attached */
      if (prev_jntPt==NULL)
	{ j = -1; } /* environment */
      else
	{ j = prev_jntPt->num; }

      p3d_jnt_stay_within_dist(&(stay_within_dist_data[j+1]), cur_jntPt,
			       &(stay_within_dist_data[i+1]),
			       &(distances[i]), q_param, q_max_param,
			       max_param, &min_param);
      /* Rem: stay_within_dist_data[0] is bound to the environment */
    }
    tot_max_param += min_param;
    if (parameter+min_param>len_param[actif_dof]-EPS6) {
      /* Test du chemin local suivant */
      q_param[actif_dof] = q_end[actif_dof];
      do { actif_dof += sens; }
      while((actif_dof>=0)&&(actif_dof<nb_dof)&&(len_param[actif_dof]<EPS6));
      if ((actif_dof>=0) && (actif_dof<nb_dof)) {
	q_max_param[actif_dof] = q_end[actif_dof];
	min_param = max_param = len_param[actif_dof];
      }
      parameter = 0.;
    } else
      { min_param = -1.; }
  }
  MY_FREE(len_param, double, nb_dof);
  MY_FREE(stay_within_dist_data, p3d_stay_within_dist_data, njnt+2);
  p3d_destroy_config(robotPt, q_max_param);
  p3d_destroy_config(robotPt, q_param);
  return tot_max_param;
}


/*
 *  Cost of a local path
 *
 *  Input:  the local path
 *
 *  Output: the cost
 */
double p3d_manh_cost(p3d_rob *robotPt, p3d_localpath *localpathPt)
{
  return localpathPt->length_lp;
}

/*
 *  does nothing
 */

p3d_localpath *p3d_simplify_manh(p3d_rob *robotPt,
				 p3d_localpath *localpathPt,
				 int *need_colcheck)
{
  p3d_localpath *newnextlocalpathPt = NULL;
  p3d_localpath *nextlocalpathPt = NULL;
  p3d_localpath *returnpathPt = NULL;
  p3d_manh_data *manh_data1Pt = NULL;
  p3d_manh_data *manh_data2Pt = NULL;
  configPt crit_q_init, crit_q_end, q1_init, q2_init, q1_end, q2_end;
  double qifirst=0, qffirst=0;
  int movingDOF1, movingDOF2;
  int i, j, ndofs = robotPt->nb_dof;
  int o1,o2;
  int *order1;
  int *order2;
  int sens1 = 0; /* +1 if increasing order of index, -1 if decreasing */
  int sens2 = 0; /* +1 if increasing order of index, -1 if decreasing */
  int found = FALSE;
  int dof_equal = TRUE;


  movingDOF1 = movingDOF2 = 0;
  order1 = order2 = NULL;
  nextlocalpathPt = localpathPt->next_lp;
  if(nextlocalpathPt == NULL)
    {
      returnpathPt =  localpathPt;
    }
  else
    {
      /* collect data specific for manhattan local path */
      /* ============================================== */
      manh_data1Pt = (pp3d_manh_data)localpathPt->specific.manh_data;
      manh_data2Pt = (pp3d_manh_data)nextlocalpathPt->specific.manh_data;


      /* look for sense of path creation, for this local path */
      /* ==================================================== */
      crit_q_init = manh_data1Pt->crit_q_init;
      crit_q_end  = manh_data1Pt->crit_q_end ;

      /* (the first moving joint determines the order of motions
	 of the joints) */
      found = FALSE;
      for(i=0;(i<ndofs)&&(!found);i++){
	if(fabs(crit_q_end[i]-crit_q_init[i])>EPS6){
	  qifirst = crit_q_init[i];
	  qffirst = crit_q_end[i];
	  found = TRUE;
	}
      }

      if(!found)
	{ PrintError(("p3d_simplify_manh: corrupted local path\n")); }

      /* (keep track of the order in which they move) */
      order1 = MY_ALLOC(int, ndofs);

      if(qifirst<=qffirst){
	/* (the joints are moved in increasing order of index) */
	sens1 = 1;
	for (i=0; i<ndofs; i++)
	  order1[i] = i;
      }
      else {
	/* (the joints are moved in decreasing order of index) */
	sens1 = -1;
	for (i=0; i<ndofs; i++)
	  order1[i] = ndofs-1-i;
      }


      /* look for sense of path creation, for next local path */
      /* ==================================================== */
      crit_q_init = manh_data2Pt->crit_q_init;
      crit_q_end  = manh_data2Pt->crit_q_end ;

      /* (the first moving joint determines the order of motions
	 of the joints) */
      found = FALSE;
      for(i=0;(i<ndofs)&&(!found);i++){
	if(fabs(crit_q_end[i]-crit_q_init[i])>EPS6){
	  qifirst = crit_q_init[i];
	  qffirst = crit_q_end[i];
	  found = TRUE;
	}
      }

      if(!found)
	{ PrintError(("p3d_simplify_manh: corrupted local path\n")); }

     /* (keep track of the order in which they move) */
      order2 = MY_ALLOC(int, ndofs);

      if(qifirst<=qffirst){
	/* (the joints are moved in increasing order of index) */
	sens2 = 1;
	for (i=0; i<ndofs; i++)
	  order2[i] = i;
      }
      else {
	/* (the joints are moved in decreasing order of index) */
	sens2 = -1;
	for (i=0; i<ndofs; i++)
	  order2[i] = ndofs-1-i;
      }

      /* collect other data, for overlap test between this
	 and next localpath */
      /* =================================================== */
      q1_init = manh_data1Pt->q_init;
      q1_end  = manh_data1Pt->q_end ;
      q2_init = manh_data2Pt->q_init;
      q2_end  = manh_data2Pt->q_end ;

      /* do overlap test, replace if overlapping */
      /* ======================================= */
      if(sens1 == sens2)
	{
	  /* computed in same sens */

	  /* look for last  moving DOF of this path */
	  o1 = -1; /* in case all are identical */
	  found = FALSE;
	  for(i=ndofs-1;(i>=0)&&(!found);i--)
	    {
	      if(fabs(q1_end[order1[i]]-q1_init[order1[i]]) > EPS6)
		{
		  /* found */
		  found = TRUE;
		  o1 = order1[i];
		  movingDOF1 = i;
		}
	    }
	  /* look for first moving DOF of next path */
	  o2 = ndofs; /* in case all are identical */
	  found = FALSE;
	  for(i=0;(i<ndofs)&&(!found);i++)
	    {
	      if(fabs(q2_end[order2[i]]-q2_init[order2[i]]) > EPS6)
		{
		  /* found */
		  found = TRUE;
		  o2 = order2[i];
		  movingDOF2 = i;
		}
	    }

	  /* first part of overlap test */
	  if(o1 != o2)
	    {
	      /* they don't overlap, or movingDOF1 == -1 or movingDOF2 == ndofs */
	      returnpathPt = localpathPt;
	    }
	  else
	    {
	      /* they might overlap */

	      q1_end[order1[movingDOF1]]  = q2_end[order2[movingDOF2]];
	      q2_init[order2[movingDOF2]] = q1_end[order1[movingDOF1]];

	      returnpathPt = localpathPt;
	    }
	}
      else
	{
	  /* computed in different sens */
	  dof_equal = TRUE;

	  if(order1[0] == 0)
	    {
	      i = ndofs;
	      while( (i>0) && (dof_equal) )
		{
		  i--;
		  dof_equal = (fabs(q1_init[i] - q2_end[i]) <= EPS6);
		}
	    }
	  else
	    {
	      i = 0;
	      while( (i<ndofs) && (dof_equal) )
		{
		  dof_equal = (fabs(q1_init[i] - q2_end[i]) <= EPS6);
		  if(dof_equal)
		    i++;
		}
	    }

	  /* q1_end : first i-1 that are moved, must move as foreseen */
	  /* q1_end : i-th becomes q2_end[i] since we must go there
	     rather than to q1_end[i] : note that it does not matter
	     whether or not q2_end[i] lies between q1_init[i]
	     and q1_end[i] since the non-overlapping part is done
	     anyway by the next localpath */
	  q1_end[i]  = q2_end[i];
	  q2_init[i] = q1_end[i];

	  /* q1_end : rest becomes q1_init, we don't have to move any
	     further because the second localpath goes back on same track */
	  if(order1[0] == 0)
	    {
	      for(j=i+1;j<ndofs;j++)
		{
		  q1_end[j]  = q1_init[j];
		  q2_init[j] = q1_end[j];
		}
	    }
	  else
	    {
	      for(j=0;j<i;j++)
		{
		  q1_end[j]  = q1_init[j];
		  q2_init[j] = q1_end[j];
		}

	    }
	  returnpathPt = localpathPt;
	}

      /* if next localpath is reduced to q2_init == q2_end: remove it */
      /* ============================================================ */
      if( p3d_equal_config(robotPt,q2_init,q2_end) )
	{
	  /* note that we can remove the localpath because in
	     p3d_simplify_path there is a loop testing on
	     "localpathPt->next_lp != NULL" and at the end
	     it counts the total number of localpaths in the
	     trajectory again */

	  newnextlocalpathPt = nextlocalpathPt->next_lp;
	  if (newnextlocalpathPt != NULL)
	  {
		newnextlocalpathPt->prev_lp = localpathPt;
	  }
	  localpathPt->next_lp = newnextlocalpathPt;

	  p3d_manh_destroy(robotPt,nextlocalpathPt);
	}
    }

  /* clean up */
  /* ======== */
  MY_FREE(order1,int,ndofs);
  MY_FREE(order2,int,ndofs);


  /* check for terrible error */
  /* ======================== */
  if((localpathPt != NULL) && (returnpathPt == NULL))
    {
      /* something went terribly wrong */
      PrintError(("p3d_simplify_manh(): did not simplify properly\n"));

      returnpathPt =  localpathPt;
    }

  /* update range_param and length_lp */
  /* ================================ */
  localpathPt->length_lp = p3d_manh_dist(robotPt, localpathPt);
  localpathPt->range_param = localpathPt->length_lp;
  if(localpathPt->next_lp != NULL)
    {
      localpathPt->next_lp->length_lp = p3d_manh_dist(robotPt, localpathPt->next_lp);
      localpathPt->next_lp->range_param = localpathPt->next_lp->length_lp;
    }

  /* return resulting localpath and next one */
  /* ======================================= */
  return returnpathPt;
}

/*
 *  p3d_write_manh --
 *
 *  write a localpath of type manhattan in a file
 *
 *  ARGS IN  : a file descriptor,
 *             a robot,
 *             a localpath
 *
 *  ARGS OUT : TRUE if success,
 *             FALSE if input local path is not a Manhattan one.
 */

int p3d_write_manh(FILE *file, p3d_rob* robotPt, p3d_localpath* localpathPt)
{
  p3d_manh_data *manh_dataPt = NULL;

  if (localpathPt->type_lp != MANHATTAN) {
    return FALSE;
  }

  fprintf(file, "\n\np3d_add_localpath MANHATTAN\n");

  manh_dataPt = (pp3d_manh_data)localpathPt->specific.manh_data;
  /* write each RS segment */
  fprintf(file, "conf_init");
  fprint_config_one_line(file, robotPt, manh_dataPt->q_init);
  fprintf(file, "\n");
  fprintf(file, "conf_end ");
  fprint_config_one_line(file, robotPt, manh_dataPt->q_end);
  fprintf(file, "\n");

  fprintf(file, "crit_q_init");
  fprint_config_one_line(file, robotPt, manh_dataPt->crit_q_init);
  fprintf(file, "\n");
  fprintf(file, "crit_q_end ");
  fprint_config_one_line(file, robotPt, manh_dataPt->crit_q_end);
  fprintf(file, "\n");

  fprintf(file, "\np3d_end_local_path\n");

  return TRUE;
}


/*
 * Manhattan local planner
 *
 * Input:  the robot, two configurations
 *
 * Output: a local path.
 *
 * Allocation: the initial and goal config are copied
 */
p3d_localpath *p3d_manh_localplanner(p3d_rob *robotPt, configPt qi,
				     configPt qf, int* ikSol)
{
  configPt q_i, q_f;
  configPt qi_crit, qf_crit;
  int i,ndof = robotPt->nb_dof;
  p3d_localpath *localpathPt;

  /* on verifie que les configurations de depart et d'arrivee existent */
  if(qi == NULL){
    PrintInfo(("MP: p3d_manh_localplanner: no start configuration...\n"));
    p3d_set_search_status(P3D_ILLEGAL_START);
    return(NULL);
  }
  if(qf == NULL){
    PrintInfo(("MP: p3d_manh_localplanner: no goal configuration...\n"));
    p3d_set_search_status(P3D_ILLEGAL_GOAL);
    return(NULL);
  }

  /* copy initial and end configurations */
  q_i = p3d_copy_config(robotPt, qi);
  q_f = p3d_copy_config(robotPt, qf);

  if (p3d_get_search_verbose()){
    PrintInfo(("MP: p3d_manh_localplanner : "));
    PrintInfo(("qi=("));
    for(i=0;i<ndof;i++){
      PrintInfo(("%f,",q_i[i]));
    }
    PrintInfo((") ; "));
    PrintInfo(("qf=("));
    for(i=0;i<ndof;i++){
      PrintInfo(("%f,",q_f[i]));
    }
    PrintInfo((")\n"));
  }

  /* check that qi != qf */
  if(p3d_equal_config(robotPt,q_i,q_f)) {
    PrintInfo((("MP: p3d_manh_localplanner: q_init = q_goal!\n")));
    p3d_set_search_status(P3D_CONFIG_EQUAL);
    return(NULL);
  }

  /* test collision of initial and end configurations */
  /* Modif NIC
  p3d_set_robot_config(robotPt, q_i);
  p3d_update_robot_pos();
  if(p3d_col_test()){
    PrintInfo(("MP: p3d_manh_localplanner (1): Illegal q_init\n"));
  }

  p3d_set_robot_config(robotPt, q_f);
  p3d_update_robot_pos();
  if(p3d_col_test()){
    PrintInfo(("MP: p3d_manh_localplanner: Illegal q_goal\n"));
  }
  */
  qi_crit = p3d_copy_config(robotPt, q_i);
  qf_crit = p3d_copy_config(robotPt, q_f);
  localpathPt = p3d_alloc_manh_localpath(robotPt, q_i, q_f, qi_crit, qf_crit, 0,TRUE);
  /* stores length of local path */
  localpathPt->length_lp = localpathPt->length(robotPt, localpathPt);
  localpathPt->ikSol = ikSol;
  localpathPt->activeCntrts = p3d_getActiveCntrts(robotPt,&(localpathPt->nbActiveCntrts));
  return localpathPt;
}


void lm_destroy_manhattan_params(p3d_rob * robotPt, void *paramPt)
{
}

/*
 *  read_manhattan_localpath --
 *
 * build a manhattan local path and read the data specific this local
 * path in a file.
 *
 *  ARGS IN  : the file descriptor
 *
 * ARGS OUT : a local path or NULL if error */

p3d_localpath *p3d_read_manhattan_localpath(p3d_rob* robotPt, FILE *file,
					    double version)
{
  p3d_localpath *localpathPt = NULL;
  int size_max_line=0;
  char *pos=NULL, *line=NULL, *name=NULL;
  configPt q_init=NULL, q_end=NULL, crit_q_init=NULL, crit_q_end=NULL;
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
					   "conf_init", version)) == NULL) {
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
					  "conf_end", version)) == NULL) {
      PrintWarning(("line %d: expecting end configuration\n", num_line));
      success = FALSE;
    }
  }

  /*
   *  look for crit_q_init
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
    if ((crit_q_init = p3d_read_word_and_config(robotPt, line,
					   "crit_q_init", version)) == NULL) {
      PrintWarning(("line %d: expecting initial configuration\n", num_line));
      success = FALSE;
    }
  }

  /*
   *  look for crit_q_end
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
    if ((crit_q_end = p3d_read_word_and_config(robotPt, line,
					  "crit_q_end", version)) == NULL) {
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
    localpathPt = p3d_alloc_manh_localpath(robotPt, q_init, q_end,
					   crit_q_init, crit_q_end,
					   0, TRUE);
  }

  if (success == FALSE) {
    /* there has been a problem desallocate configurations */
    if (q_init != NULL) {
      p3d_destroy_config(robotPt, q_init);
    }
    if (q_end != NULL) {
      p3d_destroy_config(robotPt, q_end);
    }
    if (crit_q_init != NULL) {
      p3d_destroy_config(robotPt, crit_q_init);
    }
    if (crit_q_end != NULL) {
      p3d_destroy_config(robotPt, crit_q_end);
    }
  }

  return localpathPt;
}



