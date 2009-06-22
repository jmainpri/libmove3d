#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
//#include "Collision-pkg.h"




int p3d_compute_traj_nloc(p3d_traj *trajPt)
{
  p3d_localpath *localpathPt = trajPt->courbePt;
  int nloc = 0;

  while (localpathPt != NULL){
    nloc++;
    localpathPt= localpathPt->next_lp;
  }
  return(nloc);
}

/*  
 *  p3d_compute_traj_rangeparam --  
 *
 *  Input:  the trajectory
 *
 *  Output: the parameter range of the trajectory.
 *
 *  Description: a trajectory is a curve parameterized by a parameter which is 
 *          not necessarily arc-length. The range of this parameter is the 
 *          sum of the ranges of the parameter along the localpaths of the 
 *          trajectory.
 */

double p3d_compute_traj_rangeparam(p3d_traj *trajPt) 
{
  double range_param = 0;
  p3d_localpath *localpathPt;

  if(trajPt != NULL){
    localpathPt = trajPt->courbePt;
    while (localpathPt != NULL){
      range_param += localpathPt->range_param;
      localpathPt = localpathPt->next_lp;
    }
    return(range_param);
  }
  else  return(0);
}

/*
 *  p3d_compute_traj_length --
 *
 *  Input:  the trajectory
 *
 *  Output: the length of the trajectory.
 *
 *  Description: compute the length of a trajectory
 */

double p3d_compute_traj_length(p3d_traj *trajPt) 
{
  double length = 0;
  p3d_localpath *localpathPt;

  if(trajPt != NULL){
    localpathPt = trajPt->courbePt;
    while (localpathPt != NULL){
      length += localpathPt->length_lp;
      localpathPt = localpathPt->next_lp;
    }
    return(length);
  }
  else  return(0);
}

/*  
 *  compute configuration at given parameter along a trajectory
 *
 *  Input:  the trajectory,
 *          the parameter.
 *
 *  Output: the configuration
 *
 *  Description: a trajectory is a curve parameterized by a parameter which is 
 *          not necessarily arc-length. This function computes the 
 *          configuration at the given parameter along a trajectory.
 */
configPt p3d_config_at_param_along_traj(p3d_traj *trajPt,
					double parameter)
{
  configPt q;
  p3d_rob *robotPt = trajPt->rob;
  p3d_localpath *localpathPt = trajPt->courbePt;

  if (!localpathPt){
    printf("the traj has no localPath\n");
    return NULL;
  }
  while (parameter > localpathPt->range_param){
    if (localpathPt->next_lp != NULL){
      parameter -= localpathPt->range_param;
      localpathPt = localpathPt->next_lp;
    }
    else{
      parameter = localpathPt->range_param;
    }
  }
  q = localpathPt->config_at_param(robotPt, localpathPt, parameter);
  return q;
}

/*  
 *  compute ikSol at given parameter along a trajectory
 *
 *  Input:  the trajectory,
 *          the parameter.
 *
 *  Output: the ikSol
 *
 *  Description: a trajectory is a curve parameterized by a parameter which is 
 *          not necessarily arc-length. This function computes the 
 *          ikSol at the given parameter along a trajectory.
 */
void p3d_ikSol_at_param_along_traj(p3d_traj *trajPt, double parameter, int ** ikSol){
  p3d_rob *robotPt = trajPt->rob;
  p3d_localpath *localpathPt = trajPt->courbePt;

  while (parameter > localpathPt->range_param){
    if (localpathPt->next_lp != NULL){
      parameter -= localpathPt->range_param;
      localpathPt = localpathPt->next_lp;
    }
    else{
      parameter = localpathPt->range_param;
    }
  }
  p3d_copy_iksol(robotPt->cntrt_manager, localpathPt->ikSol, ikSol);
}

/*  
 *  compute configuration at given distance along a trajectory
 *
 *  Input:  the trajectory,
 *          the distance
 *
 *  Output: the configuration
 *
 *  Description: This function computes the 
 *          configuration at the given distance along a trajectory.
 */
configPt p3d_config_at_distance_along_traj(p3d_traj *trajPt,
					   double distance)
{
  configPt q;
  p3d_rob *robotPt = trajPt->rob;
  p3d_localpath *localpathPt = trajPt->courbePt;

  while (distance > localpathPt->length_lp){
    if (localpathPt->next_lp != NULL){
      distance -= localpathPt->length_lp;
      localpathPt = localpathPt->next_lp;
    }
    else{
      distance = localpathPt->length_lp;
    }
  }
  q = localpathPt->config_at_distance(robotPt, localpathPt, distance);
  return q;
}
      
/*
 *  Stay within distance
 *
 *  Input:  a trajectory, 
 *          a direction
 *          an array of maximal distance for each body, 
 *          position on the trajectory.
 *
 *  Output: new position on the trajectory
 *
 *  Description: move forward or backward along a trajectory in such a way
 *          that each point of the robot moves by less than the distance 
 *          given as input. 
 */

void p3d_traj_stay_within_dist(p3d_traj  *trajPt,
			       whichway dir,
			       double *distances,
			       double *localparamPt)
{
  p3d_rob *robotPt=trajPt->rob;
  p3d_localpath *localpathPt = NULL;
  double upval=*localparamPt, upval0 = 0;
  int end_motion=FALSE;
  double du;
  int i, njnt;
	
  /* look for local path corresponding to given parameter */
  localpathPt = trajPt->courbePt;
	
  while (upval > localpathPt->range_param){
    if (localpathPt->next_lp != NULL){
      upval -= localpathPt->range_param;
      upval0 += localpathPt->range_param;
      localpathPt = localpathPt->next_lp;
    }
    else if (upval - localpathPt->range_param < EPS6*trajPt->range_param) {
      upval = localpathPt->range_param;
    }
    else {
      return;
    }
  }
	
  njnt = robotPt->njoints;
  /* upval is now the parameter along localpathPt */
  while (!end_motion) {
    du = localpathPt->stay_within_dist(robotPt, localpathPt, 
				       upval, dir, distances);
    if (dir == FORWARD) {
      upval += du;
      if (upval > localpathPt->range_param - EPS10) {
	/* end of localpath is reached */
	if (localpathPt->next_lp != NULL) {
	  upval = 0;
	  upval0 += localpathPt->range_param;
	  localpathPt = localpathPt->next_lp;
	}
	else {
	  upval = localpathPt->range_param;
	  end_motion = TRUE;
	}
      }
    }
    else {
      /* backward motion */
      upval -= du;
      if (upval < EPS10) {
	/* the beginning of the local path is reached */
	if (localpathPt->prev_lp != NULL){
	  localpathPt=localpathPt->prev_lp;
	  upval = localpathPt->range_param;
	  upval0 -= localpathPt->range_param;
	}
	else{
	  /* the beginning of the trajectory is reached */
	  upval = 0;
	  end_motion = TRUE;
	}
      }
    }
    /* test if a body reached the limit */
    for (i=0; i <= njnt; i++){
      if (distances[i] < EPS6){
	end_motion = TRUE;
      }
    }
  }
  if(p3d_get_ik_choice() != IK_NORMAL){
    printf("p3d_traj_stay_within_dist ");
    p3d_print_iksol(robotPt->cntrt_manager, localpathPt->ikSol);
  }
  *localparamPt = upval0 + upval;
}



/* compute the length of a trajectory by summing the lengths of each 
 * local path and extract the initial and goal configurations
 *
 * Input:  the trajectory
 *
 * Output: the length, the initial and final configurations
 */

double p3d_ends_and_length_traj(p3d_traj* trajPt, configPt *qinitPt,
				configPt *qgoalPt)
{
  p3d_localpath *localpathPt = trajPt->courbePt;
  p3d_localpath *last_localpathPt=NULL;
  double ltot = 0.0;
  p3d_rob *robotPt = trajPt->rob;

  *qinitPt = localpathPt->config_at_distance(robotPt, localpathPt,0);

  while (localpathPt != NULL){
    ltot += localpathPt->length_lp;
    last_localpathPt = localpathPt;
    localpathPt = localpathPt->next_lp;
  }
  *qgoalPt = 
    last_localpathPt->config_at_distance(robotPt, last_localpathPt,
					 last_localpathPt->length_lp);
  return ltot;
}

/* create an empty trajectory in the workspace
 *
 * Input a robot
 *
 */


p3d_traj *p3d_create_empty_trajectory(p3d_rob *robotPt)
{
  p3d_traj *t;
  int currob = p3d_get_desc_curnum(P3D_ROBOT);
  int curtraj = p3d_get_desc_curnum(P3D_TRAJ);

  p3d_sel_desc_id(P3D_ROBOT,robotPt);

  p3d_beg_desc(P3D_TRAJ,"");
  p3d_end_desc();

  t = (p3d_traj*)p3d_get_desc_curid(P3D_TRAJ);

  p3d_sel_desc_num(P3D_ROBOT, currob);
  if(curtraj != -1) {
    p3d_sel_desc_num(P3D_TRAJ, curtraj);
  }
  return t;
}

/* create a trajectory by copy
 *
 * Input the source trajectory
 *
 * Output the new trajectory. NULL if error
 *
 */

p3d_traj *p3d_create_traj_by_copy(p3d_traj *source)
{
  p3d_traj *dest = NULL;
  p3d_localpath *lp = source->courbePt, *newlp = NULL;
  int currob = p3d_get_desc_curnum(P3D_ROBOT);
  int curtraj = p3d_get_desc_curnum(P3D_TRAJ);

  p3d_sel_desc_id(P3D_ROBOT,source->rob);

  p3d_beg_desc(P3D_TRAJ,"");
  
  while (lp)
  {
    newlp = lp->copy(source->rob,lp);
    p3d_add_desc_courbe(newlp);
    lp = lp->next_lp;
  }
  
  p3d_end_desc();

  dest = (p3d_traj*)p3d_get_desc_curid(P3D_TRAJ);
  
  p3d_sel_desc_num(P3D_ROBOT, currob);
  p3d_sel_desc_num(P3D_TRAJ, curtraj);
  return dest;
}

/* concat two trajectory t1 = t1 + t2
 *
 * Input the first trajectory (modified) and the second one (unchanged)
 *
 * Output FALSE in case of success
 *
 * note the end of the first trajectory must be the beginning of the second one
 *
 */

int p3d_concat_traj(p3d_traj *traj1Pt, p3d_traj *traj2Pt)
{
  p3d_localpath *localpath1Pt = NULL, *localpath2Pt = NULL;
  configPt q1_end=NULL, q2_start=NULL;
  p3d_rob *robotPt = traj1Pt->rob;
  
  /* check that end of first trajectory and beginning of second are equal */
  
  q1_end = p3d_config_at_param_along_traj(traj1Pt, traj1Pt->range_param);
  q2_start = p3d_config_at_param_along_traj(traj2Pt, 0);
  if (! p3d_equal_config(robotPt, q1_end, q2_start)) {
    PrintError(("concat: end of first trajectory different from beginning of second one\n"));
    return TRUE;
  }

  p3d_destroy_config(robotPt, q1_end);
  p3d_destroy_config(robotPt, q2_start);
  localpath1Pt = traj1Pt->courbePt;
  localpath2Pt = traj2Pt->courbePt;	
  
  /* go to end of first trajectory */
  while (localpath1Pt->next_lp != NULL) {
    localpath1Pt = localpath1Pt->next_lp;
  }
  
  /* copy second trajectory at end of first trajectory */
  while(localpath2Pt != NULL) {
    localpath1Pt->next_lp = localpath2Pt->copy(robotPt, localpath2Pt);
    localpath1Pt->next_lp->prev_lp = localpath1Pt;
    localpath1Pt = localpath1Pt->next_lp;
    localpath2Pt = localpath2Pt->next_lp;
  }
  traj1Pt->range_param += traj2Pt->range_param;
  traj1Pt->nloc += traj2Pt->nloc;
  return FALSE;
}

/* replace a part a trajectory by a given trajectory
 * 
 * Input : a trajectory (modified)
 *         a parameter of the beginning of the part to replace
 *         a parameter of the end of the part to replace
 *         a trajectory (unchanged)
 * Output FALSE in case of success
 *
 * note the start en end configurations must be equals on the trajectories
 *
 */
int p3d_replace_traj(p3d_traj *traj1Pt, 
		     double upval1, 
		     double upval2, 
		     p3d_traj *traj2Pt)
{
  p3d_localpath *localpath1Pt = NULL, *localpath2Pt = NULL, 
    *start_q2qe_lpPt = NULL, *end_qiq1_lpPt=NULL;
  configPt q11=NULL, q12=NULL, q21=NULL, q22=NULL;
  double rp2;
  p3d_rob *robotPt = traj1Pt->rob;

  localpath1Pt = traj1Pt->courbePt;
	
  /* look for first configuration */
  while (localpath1Pt->range_param < upval1) {
    upval1 -= localpath1Pt->range_param;
    upval2 -= localpath1Pt->range_param;
    localpath1Pt = localpath1Pt->next_lp;
  }
  /* first configuration on first traj */
  q11 = localpath1Pt->config_at_param(robotPt, localpath1Pt, upval1);
	
  /* look for second configuration */
  localpath2Pt = localpath1Pt;
  while (localpath2Pt->range_param < upval2) {
    upval2 -= localpath2Pt->range_param;
    if (localpath2Pt->next_lp != NULL) {
      localpath2Pt = localpath2Pt->next_lp;
    }
    else {
      /* pval2 is beyond trajectory bounds */
      upval2 = localpath2Pt->range_param;
    }
  }
  /* second configurationon first traj */
  q12 = localpath2Pt->config_at_param(robotPt, localpath2Pt, upval2);
	
  /* q21 and q22 start and end configurations of second traj */
  p3d_ends_and_length_traj(traj2Pt, &q21, &q22);
	
  /* q11 and q21 must be equal */
  if (!p3d_equal_config(robotPt, q11, q21))
    {
      p3d_destroy_config(robotPt, q11);
      p3d_destroy_config(robotPt, q12);
      p3d_destroy_config(robotPt, q21);
      p3d_destroy_config(robotPt, q22);
      return TRUE;
    }
  
  /* q12 and q22 must be equal */
  if (!p3d_equal_config(robotPt, q12, q22)) 
    {
      p3d_destroy_config(robotPt, q11);
      p3d_destroy_config(robotPt, q12);
      p3d_destroy_config(robotPt, q21);
      p3d_destroy_config(robotPt, q22);
      return TRUE;
    }
  p3d_destroy_config(robotPt, q11);
  p3d_destroy_config(robotPt, q12);
  p3d_destroy_config(robotPt, q21);
  p3d_destroy_config(robotPt, q22);	
  
  /* end_qiq1_lpPt = part of localpath that ends at q1 */
  end_qiq1_lpPt = localpath1Pt->extract_by_param(robotPt, 
						 localpath1Pt, 
						 0, 
						 upval1);
	
  /* start_q2qe_lpPt = part of localpath that start at q2 */
  rp2 = localpath2Pt->range_param;
	
  start_q2qe_lpPt = localpath2Pt->extract_by_param(robotPt, 
						   localpath2Pt, 
						   upval2, 
						   rp2);
	
  end_qiq1_lpPt->prev_lp = localpath1Pt->prev_lp;
  if (end_qiq1_lpPt->prev_lp)
    end_qiq1_lpPt->prev_lp->next_lp = end_qiq1_lpPt;
  else
    traj1Pt->courbePt = end_qiq1_lpPt;
	
  /* extract localpath1Pt from traj1 */
  localpath1Pt->prev_lp = NULL;
	
  start_q2qe_lpPt->next_lp = localpath2Pt->next_lp;
  if (start_q2qe_lpPt->next_lp)
    start_q2qe_lpPt->next_lp->prev_lp = start_q2qe_lpPt;
  localpath2Pt->next_lp = NULL;
	
  /* destroy part of traj 1 between q1 and q2 */
  destroy_list_localpath(robotPt, localpath1Pt);//to see...
	
  localpath1Pt = NULL;
  localpath2Pt = NULL;
	
  /* copy traj 2 and insert it between q1 and q2 */
  localpath1Pt = end_qiq1_lpPt;
  localpath2Pt = traj2Pt->courbePt;
	
  while (localpath2Pt != NULL) {
    localpath1Pt->next_lp = localpath2Pt->copy(robotPt, localpath2Pt);
    localpath1Pt->next_lp->prev_lp = localpath1Pt;
    localpath1Pt = localpath1Pt->next_lp;
    localpath2Pt = localpath2Pt->next_lp;
  }
	
  localpath1Pt->next_lp = start_q2qe_lpPt;
  start_q2qe_lpPt->prev_lp = localpath1Pt;
  traj1Pt->nloc = p3d_compute_traj_nloc(traj1Pt);
  traj1Pt->range_param = p3d_compute_traj_rangeparam(traj1Pt);
  return FALSE;
}

//start path deform
int p3d_destroy_traj(p3d_rob* robotPt, p3d_traj* traj){
  int i, j, found = 0;
  for (i = 0; i<robotPt->nt; i++) {
    if(robotPt->t[i]->id == traj ->id) {
      found =1;
      for (j = i; j<(robotPt->nt-1); j++) {
        robotPt->t[j]= robotPt->t[j+1];
      }
      robotPt->nt--;
      if((robotPt->tcur!= NULL) && (robotPt->tcur->id == traj->id) ) {
        robotPt->tcur =NULL;
      }
    }
  }
  destroy_list_localpath(robotPt, traj->courbePt);
  if(traj->name != NULL) {
    free(traj->name);
  }
  if(traj->file != NULL) {
    free(traj->file);
  }
  MY_FREE(traj,p3d_traj,1);
  return found;
}

void p3d_compute_traj_project(p3d_rob* robotPt, p3d_traj* traj1Pt, p3d_traj* traj2Pt, int nstep) {
  int i,j;
  double step1, step2, param1, param2;
  FILE * oFile;
  step1 =(traj1Pt->range_param) /(double) nstep;
  step2 =(traj2Pt->range_param) /(double) nstep;
  oFile = open_file_to_save_plot(1);
  for(i=0;i<nstep+1;i++) {
    for (j =0; j<nstep+1; j++) {
      param1 = ((double)i)*step1;
      param2 = ((double)j)*step2;
      if(p3d_is_one_project(robotPt,traj1Pt, traj2Pt,param1, param2)) {
        fprintf(oFile,"%f %f\n", param1, param2);
      }
    }
  }
  close_file_to_save_plot(oFile);
}

/**
 * @brief Extract configurations on the given trajectories at the given distances and check wether a localpath between these configurations is free of obstacles
 * @param robotPt the current robot
 * @param traj1Pt first trajectory
 * @param traj2Pt second trajectory
 * @param param1 the distance between the initial configuration of the first trajectory and the configuration to compute
 * @param param2 the distance between the initial configuration of the second trajectory and the configuration to compute
 * @return True if the configurations are projectable False otherwise ie there is an obstacle.
 */
int p3d_is_one_project(p3d_rob* robotPt, p3d_traj* traj1Pt, p3d_traj* traj2Pt,
           double param1, double param2) {
  configPt q1, q2;
  p3d_localpath* current_lp;
  int ntest, res;

  q1 = p3d_config_at_param_along_traj(traj1Pt,param1);
  q2 = p3d_config_at_param_along_traj(traj2Pt,param2);

  if(!p3d_equal_config(robotPt,q1, q2)) {
    current_lp = p3d_local_planner(robotPt, q1,q2);
    res = p3d_unvalid_localpath_test(robotPt,current_lp, &ntest);
    current_lp->destroy(robotPt,current_lp);
  }else{
    res = 0;
  }
  p3d_destroy_config(robotPt, q1);
  p3d_destroy_config(robotPt, q2);
  return !res;
}

int p3d_test_projection(p3d_rob* robotPt , p3d_traj *traj1Pt,
      p3d_traj* traj2Pt, int nstep) {
  double p1, p2;
  p3d_traj *cycle_trajPt;
  int res;

  if(p3d_get_is_general_proj() == FALSE) {
    return p3d_is_projectable(robotPt , traj1Pt, traj2Pt, nstep, &p1, &p2);
  }
  cycle_trajPt = p3d_build_cycle_from_trajs(robotPt, traj1Pt, traj2Pt);
  res = p3d_is_reductible_cycle(robotPt, cycle_trajPt, nstep);
  p3d_del_traj(cycle_trajPt);//mokhtar
  return res;
}

int p3d_is_projectable(p3d_rob* robotPt , p3d_traj* traj1Pt, 
           p3d_traj* traj2Pt, int nstep,
           double *p1, double *p2) {
  int i, j, di, dj, ntest = 0, result = 0;
  double step1, step2, param1, param2, best_cost, cur_cost;
  p3d_project_point** proj_table;
  dbl_list* list_open_points;
  p3d_project_point *goal_pointPt, *best_pointPt, *current_pointPt = NULL;
  FILE* OFile;

  step1 =(traj1Pt->range_param)/(double) nstep;//discretise la premiere trajectoire
  step2 =(traj2Pt->range_param)/(double) nstep;//discretise la seconde trajectoire
  proj_table = p3d_init_proj_point_table(nstep);//initialise le graph de visibilite
  goal_pointPt = proj_table[(nstep)*(nstep+2)];//prend l'avant dernier element comme goal
  list_open_points = dbl_list_pointer_init();//initialisation de la liste des points
  best_cost = p3d_compute_proj_point_cost(proj_table[0], nstep);//calcul du cout du premier point de la liste.
  best_pointPt = proj_table[0];//initialisation de best_point
  //dbl_list_append_link(list_open_points, proj_table[0]);//Ajout du premier point dans la liste
  dbl_list_insert_sorted_link(list_open_points, proj_table[0], p3d_sort_project_point);
  if(p3d_get_gvis())
    OFile = open_file_to_save_plot(p3d_get_nretract());//creation du fichier pour sauver le graph de visibilite 
  while(dbl_list_more(list_open_points)&&(goal_pointPt->in_test == FALSE)) {
    ntest++;
    current_pointPt = DBL_LIST_FIRST(p3d_project_point,list_open_points);//selection du premier element de la liste ie qui a le cout le plus faible
    //calcul des position des configuration a extraire pour voir si elles sont mutuellement visible ie il n'y a pas d'obstacles
    param1 = ((double)current_pointPt->i)*step1;
    param2 = ((double)current_pointPt->j)*step2;
    if(p3d_is_one_project(robotPt,traj1Pt, traj2Pt,param1, param2)) {//si elle sont mutuellement visible
      if(p3d_get_gvis())
        fprintf(OFile,"%f %f\n",param1, param2);
      current_pointPt->is_valid = TRUE;// flag pour signaler qu'il n'y a pas de collision.
      //regarder si le point calcule est plus proche du goal ou pas.
      cur_cost = current_pointPt->cost;
      if(cur_cost < best_cost ) {
        best_cost = cur_cost;
        best_pointPt = current_pointPt;
      }
      switch(p3d_get_retract_search_type()) {
        case ALL_RETRACT_SEARCH:{// global search
          for(di=-1;di<2;di++) {
            for(dj=-1;dj<2;dj++) {
              i = current_pointPt->i + di;
              j = current_pointPt->j + dj;
              if((i>-1)&&(i<nstep+1)&&(j>-1)&&(j<nstep+1)&&((di!=0)||(dj!=0))) {//si les parametres sont dans les bornes du problemes
                if((proj_table[(nstep+1)*i+j])->in_test == FALSE) {//si le point n'est pas entrain d'etre teste
                  p3d_compute_proj_point_cost(proj_table[(nstep+1)*i+j], nstep);//calcul du cout du point
                  //dbl_list_append_link(list_open_points,proj_table[(nstep+1)*i+j]);//ajouter a la liste des points calcule
                  dbl_list_insert_sorted_link(list_open_points, proj_table[(nstep+1)*i+j],p3d_sort_project_point);
                  (proj_table[(nstep+1)*i+j])->in_test = TRUE;//le point est entrain d'etre teste
                }
              }
            }
          }
          break;
        }
        case FORWARD_RETRACT_SEARCH:{
          for(di=0;di<2;di++) {
            for(dj=0;dj<2;dj++) {
              i = current_pointPt->i +di;
              j = current_pointPt->j +dj;
              if((i>-1)&&(i<nstep+1)&&(j>-1)&&(j<nstep+1)&&((di!=0)||(dj!=0))) {
                if((proj_table[(nstep+1)*i+j])->in_test == FALSE) {
                  p3d_compute_proj_point_cost(proj_table[(nstep+1)*i+j], nstep);
                  //dbl_list_append_link(list_open_points,proj_table[(nstep+1)*i+j]);
                  dbl_list_insert_sorted_link(list_open_points, proj_table[(nstep+1)*i+j],p3d_sort_project_point);
                  (proj_table[(nstep+1)*i+j])->in_test = TRUE;
                }
              }
            }
          }
          break;
        }
        case LINEAR_RETRACT_SEARCH:{
          for(di=1;di<2;di++) {
            for(dj=1;dj<2;dj++) {
              i = current_pointPt->i +di;
              j = current_pointPt->j +dj;
              if((i>-1)&&(i<nstep+1)&&(j>-1)&&(j<nstep+1)&&((di!=0)||(dj!=0))) {
                if((proj_table[(nstep+1)*i+j])->in_test == FALSE) {
                  p3d_compute_proj_point_cost(proj_table[(nstep+1)*i+j], nstep);
                  //dbl_list_append_link(list_open_points,proj_table[(nstep+1)*i+j]);
                  dbl_list_insert_sorted_link(list_open_points, proj_table[(nstep+1)*i+j],p3d_sort_project_point);
                  (proj_table[(nstep+1)*i+j])->in_test = TRUE;
                }
              }
            }
          }
          break;
        }
        default:{
          PrintInfo(("RETRACT_SEARCH : ERREUR : wrong type of search\n"));
          return(0);
        }
      }
    }
    dbl_list_remove_link_data(list_open_points,current_pointPt);//enlever de la liste le point qu'on vient de traiter
    dbl_list_goto_first (list_open_points);
    //dbl_list_sort(list_open_points,p3d_sort_project_point);
  }
  (*p1) = ((double)best_pointPt->i)*step1;
  (*p2) = ((double)best_pointPt->j)*step2;
  
  result = goal_pointPt->in_test;
  if(p3d_get_gvis())
    close_file_to_save_plot(OFile);//fermer le fichier de sauvegarde
  p3d_free_proj_point_table(proj_table, nstep);
  dbl_list_destroy(list_open_points);
  return (result);
}

p3d_traj* p3d_build_cycle_from_trajs(p3d_rob* robotPt, p3d_traj* traj1Pt, p3d_traj* traj2Pt) {
 p3d_traj* inv_trajPt;
 p3d_traj* new_trajPt;
 if((traj1Pt == NULL) || (traj2Pt == NULL)) {
   return NULL;
 }
 new_trajPt = p3d_create_traj_by_copy(traj1Pt);
 inv_trajPt = p3d_invert_traj(robotPt, traj2Pt);
 p3d_concat_traj(new_trajPt,inv_trajPt);
 p3d_del_traj(inv_trajPt);//mokhtar
 return new_trajPt;
}

int p3d_is_reductible_cycle(p3d_rob* robotPt, p3d_traj* trajPt, int nstep) {
  double param1, min_param, max_param, p1, p2;
  p3d_traj *traj1Pt, *traj2Pt, *end_traj2Pt;
  p3d_traj *beg_traj1Pt, *t1Pt, *t2Pt, *t3Pt, *inv_t3Pt, *inv_end_traj2Pt;
  int res;
  configPt q1, q2;
  p3d_localpath* current_lp;

  if(trajPt == NULL) {
    return TRUE;
  }
  param1 = (trajPt->range_param)*(double)(rand()/((double)RAND_MAX+1));

  if(param1<((trajPt->range_param)/2.)) {
    min_param = param1;
    max_param = param1 + (trajPt->range_param)/2.;
  }else{
    min_param = param1 - (trajPt->range_param)/2.;
    max_param = param1;
  }
  traj1Pt = p3d_extract_traj_from_traj(trajPt, min_param, max_param);

  p3d_check_traj_continuity(robotPt,traj1Pt);
  end_traj2Pt = p3d_extract_traj_from_traj(trajPt, max_param, trajPt->range_param);
  p3d_check_traj_continuity(robotPt,end_traj2Pt);
  inv_end_traj2Pt =p3d_invert_traj(robotPt, end_traj2Pt);
  p3d_check_traj_continuity(robotPt,inv_end_traj2Pt);
  beg_traj1Pt = p3d_extract_traj_from_traj(trajPt, 0, min_param);
  p3d_check_traj_continuity(robotPt,beg_traj1Pt);
  traj2Pt = p3d_invert_traj(robotPt, beg_traj1Pt);
  p3d_check_traj_continuity(robotPt,traj2Pt);
  p3d_concat_traj(traj2Pt,inv_end_traj2Pt);
  p3d_check_traj_continuity(robotPt,traj2Pt);
  p3d_del_traj(beg_traj1Pt);//mokhtar
  p3d_del_traj(end_traj2Pt);//mokhtar
  p3d_del_traj(inv_end_traj2Pt);//mokhtar
 
  res = p3d_is_projectable(robotPt, traj1Pt, traj2Pt, nstep, &p1, &p2);
  if ((res == TRUE) || (p3d_get_nretract() >= p3d_get_nretract_max())||
      ((p1 < EPS6)&&(p2 < EPS6)) ) {
    p3d_del_traj(traj1Pt);//mokhtar
    p3d_del_traj(traj2Pt);//mokhtar
    if((res == TRUE) && p3d_get_nretract() > 1) {
      PrintInfo(("Path deformable after several tries : %d \n",p3d_get_nretract() ));
    }
    return res;
  }
  q1 =p3d_config_at_param_along_traj(traj1Pt, p1);
  q2 =p3d_config_at_param_along_traj(traj2Pt, p2);
 
  current_lp = p3d_local_planner(robotPt,q1,q2);
  t1Pt = p3d_create_empty_trajectory(robotPt);
  t1Pt->courbePt = current_lp; 
  t1Pt->nloc = p3d_compute_traj_nloc(t1Pt);
  t1Pt->nlp  = p3d_compute_traj_nloc(t1Pt);
  t1Pt->rob = robotPt; 
  t1Pt->range_param = p3d_compute_traj_rangeparam(t1Pt);

  if(abs(p2 - traj2Pt->range_param) > EPS6) {
    t2Pt = p3d_extract_traj_from_traj(traj2Pt, p2, traj2Pt->range_param);
    p3d_concat_traj(t1Pt, t2Pt);
    p3d_del_traj(t2Pt);//mokhtar
  }
  if(abs(p1 - traj1Pt->range_param) > EPS6) {
    inv_t3Pt = p3d_extract_traj_from_traj(traj1Pt, p1, traj1Pt->range_param);
    t3Pt = p3d_invert_traj(robotPt, inv_t3Pt);
    p3d_del_traj(inv_t3Pt);//mokhtar
    p3d_concat_traj(t1Pt, t3Pt);
    p3d_del_traj(t3Pt);//mokhtar
  }
  res = p3d_is_reductible_cycle(robotPt, t1Pt, nstep);
  p3d_del_traj(t1Pt);//mokhtar
  return res;
}

int p3d_sort_project_point(void* proj_point1, void* proj_point2) {
  return ((p3d_project_point*)proj_point1)->cost > ((p3d_project_point*)proj_point2)->cost;
}

p3d_project_point ** p3d_init_proj_point_table(int nstep) {
  int i,j;
  p3d_project_point *p;
  p3d_project_point **proj_table;
  
  proj_table = MY_ALLOC(p3d_project_point*,(nstep+1)*(nstep+1));//pourquoi n+1 ???
  for(i=0;i<nstep+1;i++) {
    for (j =0; j<nstep+1; j++) {
      p = MY_ALLOC(p3d_project_point,1);
      p->i = i;
      p->j = j;
      p->in_test = 0;
      p->is_valid = 0;
      p->cost = 0.0;
      proj_table[(nstep+1)*i+j] = p;
    }
  }
  return proj_table;
}

/**
 * @brief Compute the cost of the projected point given. The cost is the distance between the point and the goal (nstep,nstep).
 * @param proj_point the projected point on the matrix
 * @param nstep the position of the point
 * @return projected point cost
 */
double p3d_compute_proj_point_cost(p3d_project_point* proj_point, int nstep) {
  proj_point->cost = sqrt(SQR(nstep-proj_point->i)+SQR(nstep-proj_point->j));
  return proj_point->cost;
}

void p3d_free_proj_point_table(p3d_project_point ** proj_table, int nstep) {
 int i,j;
 for(i=0;i<nstep+1;i++) {
    for (j =0; j<nstep+1; j++) {
      MY_FREE(proj_table[(nstep+1)*i+j], p3d_project_point, 1);
      proj_table[(nstep+1)*i+j] = NULL;
    }
 }
  MY_FREE(proj_table, p3d_project_point*,(nstep+1)*(nstep+1));
}

p3d_traj* p3d_invert_traj(p3d_rob* robotPt, p3d_traj* traj) {
  p3d_traj* inv_traj;
  p3d_localpath* localpathPt, *inv_localpathPt;
  configPt q1, q2;
  if(traj == NULL) {
    return NULL;
  }
  inv_traj = p3d_create_empty_trajectory(robotPt);
  localpathPt = traj->courbePt;
  while(localpathPt!= NULL) {
    q1 =  localpathPt->config_at_param(robotPt, localpathPt, 0);
    q2 = localpathPt->config_at_param(robotPt, localpathPt, localpathPt->range_param );
    inv_localpathPt =  p3d_local_planner(robotPt,q2,q1);
    inv_traj->courbePt = concat_liste_localpath(inv_localpathPt,inv_traj->courbePt);
    localpathPt = localpathPt->next_lp;
  }
  inv_traj->nloc = p3d_compute_traj_nloc(inv_traj);
  inv_traj->nlp  = p3d_compute_traj_nloc(inv_traj);
  inv_traj->rob = robotPt;
  inv_traj->range_param = p3d_compute_traj_rangeparam(inv_traj);
  return inv_traj;
}

/* extract a trajectory from  a given trajectory
 * 
 * Input : a trajectory
 *         a parameter of the beginning of the part to extract
 *         a parameter of the end of the part to extract
 * Output the extracted trajectory
 *
 */
p3d_traj* p3d_extract_traj_from_traj(p3d_traj *traj1Pt, double upval1, double upval2) {
  p3d_localpath *localpath1Pt = NULL, *localpath2Pt = NULL, 
    *start_lp2 = NULL, *end_lp1=NULL, *current_lp = NULL;
  p3d_rob *robotPt = traj1Pt->rob;
  p3d_traj* traj2 = p3d_create_empty_trajectory(robotPt);

  localpath1Pt = traj1Pt->courbePt;
  /* look for first configuration */
  while (localpath1Pt->range_param < upval1) {
    upval1 -= localpath1Pt->range_param;
    upval2 -= localpath1Pt->range_param;
    localpath1Pt = localpath1Pt->next_lp;
  }
  /* first configuration on first traj */
  if(localpath1Pt->range_param > upval2) {
    // the second value is in the same localpath
    end_lp1 =  localpath1Pt->extract_by_param(robotPt, localpath1Pt, upval1, upval2);
    traj2->courbePt =end_lp1;
    traj2->nloc = p3d_compute_traj_nloc(traj2);
    traj2->nlp  = p3d_compute_traj_nloc(traj2);
    traj2->range_param = p3d_compute_traj_rangeparam(traj2);
    return traj2;
  }
  // the new trajectory contain at least 2 localpaths
  end_lp1 = localpath1Pt->extract_by_param(robotPt, localpath1Pt, upval1, localpath1Pt->range_param);
  traj2->courbePt = end_lp1;
  /* look for second configuration */
   upval2 -=localpath1Pt->range_param;
   localpath2Pt = localpath1Pt->next_lp;
  //localpath2Pt = localpath1Pt;
   while(( localpath2Pt!= NULL)&&(localpath2Pt->range_param < upval2)) {
     upval2 -= localpath2Pt->range_param;
     current_lp = localpath1Pt->extract_by_param(robotPt, localpath2Pt, 0, localpath2Pt->range_param);
     traj2->courbePt = concat_liste_localpath(traj2->courbePt,current_lp);
     localpath2Pt = localpath2Pt->next_lp;
   }
   if(localpath2Pt != NULL) {
     start_lp2 = localpath2Pt->extract_by_param(robotPt, localpath2Pt, 0, upval2);
     traj2->courbePt = concat_liste_localpath(traj2->courbePt,start_lp2);
   }
   traj2->nloc = p3d_compute_traj_nloc(traj2);
   traj2->nlp  = p3d_compute_traj_nloc(traj2);
   traj2->range_param = p3d_compute_traj_rangeparam(traj2);
   return traj2;
}

int p3d_check_traj_continuity(p3d_rob* robotPt, p3d_traj* trajPt) {
  p3d_localpath*  current_lp, *prev_lp;
  configPt q1, q2;

  if ((!trajPt)|| (!trajPt->courbePt)) {
    return TRUE;
  }
  current_lp = trajPt->courbePt->next_lp;
  while(current_lp != NULL) {
    prev_lp = current_lp->prev_lp;
    q1 = prev_lp->config_at_param(robotPt,prev_lp,prev_lp->range_param);
    q2 = current_lp->config_at_param(robotPt,current_lp,0.);
    if(!p3d_equal_config(robotPt, q1, q2)) {
      PrintInfo(("trajectory not continue!! \n"));
      p3d_destroy_config(robotPt, q1);
      p3d_destroy_config(robotPt, q2);
      return FALSE;
    }
    p3d_destroy_config(robotPt, q1);
    p3d_destroy_config(robotPt, q2);
    current_lp = current_lp->next_lp;
  }
  return TRUE;
}

p3d_traj*  p3d_create_traj_from_list_nodes(p3d_graph* G,dbl_list* list_node) {
  p3d_node* prev_node, *current_node;
  p3d_localpath* list_lp = NULL, *current_lp;
  p3d_traj* traj;

  dbl_list_push(list_node);
  dbl_list_goto_first(list_node);
  dbl_list_next(list_node);
  while(dbl_list_more(list_node)) {
    prev_node = DBL_LIST_PREV(p3d_node,list_node);
    current_node = DBL_LIST_DATA(p3d_node,list_node);
    current_lp = p3d_local_planner(G->rob,prev_node->q,current_node->q);
    list_lp = concat_liste_localpath(list_lp,current_lp);
    dbl_list_next(list_node);
  }
  dbl_list_pop(list_node);
  traj = p3d_create_empty_trajectory(G->rob);
  traj->courbePt = list_lp;
  traj->nloc = p3d_compute_traj_nloc(traj);
  traj->nlp  = p3d_compute_traj_nloc(traj);
  traj->rob = G->rob;
  traj->range_param = p3d_compute_traj_rangeparam(traj);
  return traj;
}
//end path deform



/**
 * p3d_GetRandSuccesConfAlongTraj
 * Get 3 successive configurations 
 * at a random parameter along a given path. 
 * The path distance between these configuration id given by 
 * dMax*extendStepParam (equal to the step in the extend mode 
 * of diffusion.process).
 * param[In] trajPt: the given trajectory
 * param[Out] qPrevPt: 1st config along the path
 * param[Out] qCurrentPt: 2nd config along the path
 * param[Out] qNextPt: 3rd config along the path
 * param[Out] prevDistPt parameter along the path of the 1st config
 * param[Out] randDistPt parameter along the path of the 2nd config
 * param[Out] nextDistPt parameter along the path of the 3rd config
 */
void p3d_GetRandSuccesConfAlongTraj(p3d_traj* trajPt,  configPt* qPrevPt, 
				    configPt*  qCurrentPt, configPt* qNextPt, 
				    double* prevDistPt, double* randDistPt, 
				    double* nextDistPt) {
  configPt qIni = NULL, qEnd= NULL;
  double  randDist, prevDist, nextDist;
  double dMax, step, extendStepParam, trajLength;
  configPt qPrev, qCurrent, qNext;
 if(trajPt == NULL) {
    PrintInfo(("Warning: no current trajectory to \
Get Random Successive Configurations\n"));
    return;
  }
  dMax =  p3d_get_env_dmax();
  extendStepParam = p3d_GetExtendStepParam();
  step = dMax*extendStepParam;

  trajLength = p3d_ends_and_length_traj(trajPt, &qIni, &qEnd);
  randDist = p3d_random(EPS6*trajLength,(1-EPS6)*trajLength);
  prevDist = MAX(EPS6, randDist - step );
  nextDist = MIN(trajLength-EPS6, randDist + step);

  qCurrent = p3d_config_at_distance_along_traj(trajPt, randDist);
  qPrev = p3d_config_at_distance_along_traj(trajPt, prevDist);
  qNext = p3d_config_at_distance_along_traj(trajPt, nextDist);
  *qPrevPt = qPrev;
  *qCurrentPt = qCurrent;
  *qNextPt = qNext;
  //PrintInfo(("qPrev[x]:%f, qPrev[y]:%f\n",qPrev[6],qPrev[7]));
  //PrintInfo(("qCurrent[x]:%f, qCurrent[y]:%f\n",qCurrent[6],qCurrent[7])); 
  //PrintInfo(("qNext[x]:%f, qNext[y]:%f\n",qNext[6],qNext[7])); 
  *prevDistPt = prevDist;
  *randDistPt = randDist;
  *nextDistPt = nextDist;
}

/* void p3d_replaceTraj(p3d_rob* robotPt, p3d_traj* trajPt, configPt qPrev,  */
/* 		     configPt qNext, configPt qNew, double lPrev, double lNext) { */
/*   p3d_localpath* c1, lp1, lp2; */
/*   configPt qStartTraj; */
/*   if(lPrev <= EPS6) { */
/*     //we assume qPrev == qStartTraj */
/*     qStartTraj = p3d_config_at_param_along_traj (trajPt , 0); */
/*     lp1 = p3d_local_planner(robotPt, qStartTraj, qNew); */
/*     lp2 = p3d_local_planner(robotPt,  qNew, qNext); */
/*     lp2 = append_to_localpath(lp1,lp2); */



/*   } */

/* } */


/**
 * p3d_createThreeConfTraj
 * create a trajectory from 3 configurations
 * Creates 2 localpaths with the local method 
 * associated to the robot 
 * param[In] robotPt: the current robot
 * param[In] qPrev: the first configuration (init of the traj)
 * param[In] qNew: the second configuration
 * param[In] qNext: the last configuration (end of the traj)
 * @return: the created trajectory
 */  
p3d_traj* p3d_createThreeConfTraj(p3d_rob* robotPt, configPt qPrev, 
				  configPt qNew, configPt qNext) {
  p3d_localpath*  lp1, * lp2; 
  p3d_traj* trajPt;
  lp1 = p3d_local_planner(robotPt, qPrev, qNew);
  lp2 = p3d_local_planner(robotPt,  qNew, qNext); 
  append_to_localpath(lp1,lp2);
  trajPt = p3d_create_empty_trajectory (robotPt);
  trajPt->courbePt = lp1;
  trajPt->nloc = 2;
  trajPt->nlp = 2;
  trajPt->range_param = p3d_compute_traj_rangeparam(trajPt);
  return trajPt;
}
