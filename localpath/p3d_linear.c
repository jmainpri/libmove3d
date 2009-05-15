#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Localpath-pkg.h"
//#include "Collision-pkg.h"

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

  q = p3d_alloc_config(robotPt);

  if (distance < 0)
    distance = 0;
  if (distance > localpathPt->length_lp)
    distance = localpathPt->length_lp;

  alpha = distance / localpathPt->length_lp;

  /* translation parameters of main body */
  for (i=0; i<=njnt; i++) {
    jntPt = robotPt->joints[i];
    for (j=0; j<jntPt->dof_equiv_nbr; j++) {
      k = jntPt->index_dof+j;
      q[k] = p3d_jnt_calc_dof_value(jntPt, j, q_init, q_end, alpha);
    }
  }
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

  lin_localpathPt = localpathPt->specific.lin_data;

  /* store the data to compute the maximal velocities at the 
     joint for each body of the robot */
  stay_within_dist_data = MY_ALLOC(p3d_stay_within_dist_data, njnt+2);
  p3d_init_stay_within_dist_data(stay_within_dist_data);
  
  if (dir == FORWARD) {
    q_max_param = lin_localpathPt->q_end;
    min_param = max_param = range_param - parameter;
  } else {
    q_max_param = lin_localpathPt->q_init;
    min_param = max_param = parameter;
  }
  /* Get the current config to have the modifications of the constraints */
  q_param = p3d_get_robot_config(robotPt);
  
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
			     &(stay_within_dist_data[i+1]), &(distances[i]), 
			     q_param, q_max_param, max_param, &min_param);
    /* Rem: stay_within_dist_data[0] is bound to the environment */
  }
  
  MY_FREE(stay_within_dist_data, p3d_stay_within_dist_data, njnt+2);
  p3d_destroy_config(robotPt, q_param);

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
