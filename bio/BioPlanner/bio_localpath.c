#include "P3d-pkg.h"
#include "Bio-pkg.h"
#include "Planner-pkg.h"
#include "Util-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"

// NOTE : extern functions in "enchufe.h" should be included with "Bio-pkg.h"
extern int bio_all_molecules_col(void);  

// number of steps for saving intermediate nodes using RRT expansion (currently constant)
#define NLPATHS_RECOPM_STEP 10

/**************************************************/
// LOCAL FUNCTION DECLARATION

static void bio_reset_current_q_inv(p3d_rob *robotPt); 


/**************************************************/

// coll step deplacement
static double bio_coll_step_deplacement = 0.1;  // default (max. penetration = 0.05 A)


void bio_col_set_step_deplacement(double deplacement)
{
  bio_coll_step_deplacement = deplacement;  // Angstroms
}


void bio_col_get_step_deplacement(double *deplacement)
{
  *deplacement = bio_coll_step_deplacement;  // Angstroms
}

/**************************************************/

/* This function computes the length of the constant steps for validating
   a localpath in such a way that no atom moves more than a given distance */
/* NOTE: THIS IS A "BAD" FIRTS IMPLEMENTATION :
   which considers that all atom displacement is linear */
double bio_compute_localpath_validation_const_step(p3d_rob *robotPt,
						   p3d_localpath *localpathPt,
						   double maxdep)
{
  configPt qi,qf;
  p3d_vector3 *array_posJ_qi;
  p3d_vector3 posJ_qf,pos_diff;
  double d,dmax;
  double step;
  int njnt,i;
  double umax = localpathPt->range_param;

  // WARNING : suppose that the localpath is linear
  if(p3d_local_get_planner() != P3D_LINEAR_PLANNER) {
    printf("ERROR : bio_compute_localpath_validation_const_step : localpath must be LINEAR\n");
    return (0.0);
  }

  qi = localpathPt->specific.lin_data->q_init;
  qf = localpathPt->specific.lin_data->q_end;
  njnt = robotPt->njoints + 1;

  array_posJ_qi = MY_ALLOC(p3d_vector3,njnt);
  
  p3d_set_robot_config(robotPt,qi);
  p3d_update_this_robot_pos_without_cntrt_and_obj(robotPt);
  
  for(i=0; i<njnt; i++) {
    p3d_jnt_get_cur_vect_point(robotPt->joints[i],array_posJ_qi[i]);
  }

  p3d_set_robot_config(robotPt,qf);
  p3d_update_this_robot_pos_without_cntrt_and_obj(robotPt);

  dmax = 0.0;
  for(i=0; i<njnt; i++) {
    p3d_jnt_get_cur_vect_point(robotPt->joints[i],posJ_qf);
    p3d_vectSub(array_posJ_qi[i],posJ_qf,pos_diff);	
    d = (double) p3d_vectNorm(pos_diff);
    if(d > dmax)
      dmax = d;
  }

  step = umax * maxdep / dmax;
  
  MY_FREE(array_posJ_qi,p3d_vector3,njnt);  

  // DEGUG
  //printf("step = %f, umax = %f , maxdep = %f, dmax = %f\n",step,umax,maxdep,dmax);

  return(step);
}

/**************************************************/

/* Collision test along a localpath with a constant resolution (defined by the user).
   NOTE : The resolution is given in Angstroms (in the workspace) and has to be 
   "translated" to a length in CS.
*/

int bio_col_test_localpath_step(p3d_rob *robotPt,
				p3d_localpath *localpathPt,
				int *ntest, double *Kpath,
				configPt *q_atKpath)
{
  double u, du, umax; /* parameters along the local path */
  configPt qp;
  int njnt = robotPt->njoints;
  double *distances;
  int end_localpath = 0;
  double step_dep;
  p3d_node *BaseNode = NULL;
  int counter_steps_rrt_N = 0; 

  BaseNode = p3d_GetCurrentNearNode();

  bio_reset_current_q_inv(robotPt);

  u = 0.0;
  *Kpath = 0.0;
  bio_col_get_step_deplacement(&step_dep);
  if (localpathPt == NULL)
    { return FALSE; }

  // current position of robot is saved
  qp = p3d_alloc_config(robotPt);
  // WARNING : suppose that the local-path is linear
  p3d_copy_config_into(robotPt, localpathPt->specific.lin_data->q_init, &qp);
  p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qp);

   umax = localpathPt->range_param;
  distances = MY_ALLOC(double, njnt+1);

  // NOTE : qi and qg in the localpath are supposed to be collision free

  // put the robot at the beginning of the localpath
/*   if (change_position_robot(robotPt, localpathPt, u)) { */
/*     /\* The initial position of the robot is recovered *\/ */
/*     p3d_set_and_update_this_robot_conf(robotPt, qp); */
/*     p3d_destroy_config(robotPt, qp); */
/*     MY_FREE(distances, double, njnt+1); */
/*     return TRUE; */
/*   } */

  // TEMPORARY MODIF : USING NEW SIMPLIFIED FUNCTION FOR COMPUTING "du"
/*   for (j=0; j<=njnt; j++) */
/*     { distances[j] = step_dep; }   */
  
  // WARNING : a function "bio_stay_within_dist" should be writen
  //           the general function is not exact since it handles 
  //           joints instead of bodies for computing distances.
/*   du = localpathPt->stay_within_dist(robotPt, localpathPt, */
/* 				     u, FORWARD, distances); */
  ///////////////
\
  du = bio_compute_localpath_validation_const_step(robotPt,localpathPt,step_dep);  

  u+=du;
  if (u > umax - EPS6){
    u = umax;
    end_localpath = 1;
  }

  while(!end_localpath) {
    /* position of the robot corresponding to parameter u */
    //if (change_position_robot_without_cntrt(robotPt, localpathPt, u)) {  // NO LOOPS !!!  
    if (change_position_robot_multisol(robotPt, localpathPt, u, du, qp)) {
      //printf("Invalid localpath because of constraints\n");
      // save q_inv only in case of collision !!!
      /* The initial position of the robot is recovered */
      p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qp);
      p3d_destroy_config(robotPt, qp);
      MY_FREE(distances, double, njnt+1);
      return TRUE;
    }
    p3d_get_robot_config_into(robotPt, &qp);

    //g3d_draw_allwin_active();

    /* collision checking */
    *ntest = *ntest + 1;
    if (bio_all_molecules_col() > 0){
      //printf("Invalid localpath because of collision ---\n");
      //bio_set_current_q_inv(robotPt,localpathPt,u);
      bio_set_current_q_inv(robotPt,localpathPt,qp);
      p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, qp);
      p3d_destroy_config(robotPt, qp);
      MY_FREE(distances, double, njnt+1);
      return TRUE;
    }      
    
    *Kpath = u / localpathPt->range_param;
    
    if(q_atKpath != NULL)
      p3d_get_robot_config_into(robotPt, q_atKpath);       
    
    counter_steps_rrt_N ++;

    // TEMPORARY MODIF : USING NEW SIMPLIFIED FUNCTION FOR COMPUTING "du"
/*     for (j=0; j<=njnt; j++) */
/*       { distances[j] = step_dep; }   */
    
/*     du = localpathPt->stay_within_dist(robotPt, localpathPt, */
/* 				       u, FORWARD, distances); */
    /////////////

    u+=du;
    if (u > umax - EPS6){
      u = umax;
      end_localpath = 1;
    }
    else {
      // saving intermediate nodes during RRT expansion
  /*     if(p3d_get_save_intermediate_rrt_N() == 1) {  */
/* 	current_n_steps_rrt_N  = (int)ceil(n_steps_rrt_N/n_inter_rrt_N); */
/* 	if (current_n_steps_rrt_N < min_n_steps_rrt_N) */
/* 	  current_n_steps_rrt_N = min_n_steps_rrt_N; */
/* 	if(counter_steps_rrt_N == current_n_steps_rrt_N) { */
/* 	  n_inter_rrt_N++;  */
/* 	  array_n_inter_rrt_N[count_lpaths-1] = n_inter_rrt_N; */
/* 	  dist = *Kpath * localpathPt->length_lp; */
/* 	  //p3d_set_and_update_this_robot_conf_without_cntrt(robotPt,qp); */
/* 	  p3d_get_iksol_vector(robotPt->cntrt_manager,iksol); */
/* 	  q = p3d_alloc_config(robotPt); */
/* 	  p3d_copy_config_into(robotPt,qp,&q);  */
/* 	  NewNode = p3d_APInode_make_multisol(robotPt->GRAPH,q,iksol); */
/* 	  NewNode->type = LINKING; */
/* 	  NewNode->weight = ffo_dist_from_root_pos(NewNode->q); */
/* 	  p3d_insert_node(robotPt->GRAPH,NewNode); */
/* 	  p3d_create_edges(robotPt->GRAPH,BaseNode,NewNode,dist); */
/* 	  p3d_add_node_compco(NewNode, BaseNode->comp);	 */
/* 	  counter_steps_rrt_N = 0; */
/* 	} */
/*       } */
    }
  }
  *Kpath = 1.0;
  if(q_atKpath != NULL)
    // WARNING : suppose that the local-path is linear
    // WARNING : NEXT LINES ARE VALID IN THE CASE OF CONSTRAINTS ???
    p3d_copy_config_into(robotPt, localpathPt->specific.lin_data->q_end, q_atKpath);

  /* The initial position of the robot is recovered */
  // WARNING : suppose that the local-path is linear
  p3d_set_and_update_this_robot_conf_without_cntrt(robotPt,localpathPt->specific.lin_data->q_end);
  p3d_destroy_config(robotPt, qp);

  MY_FREE(distances, double, njnt+1);
  return FALSE; 
}


/**
 * p3d_isBioTrajInCol
 * Test if a bio path is valide 
 * with the current Van der Waals parameters
 * @param[In] robotPt: Poiter to the robot
 * @param[In] trajPt: considered robto trajectory
 * @param[Out] nTestPt: pointer on the number of 
 * collision tests performed to cheack the path
 * @return: TRUE if the trajectory is in collision
 */
int isBioTrajInCol(p3d_rob* robotPt, p3d_traj* trajPt, 
		   int* nTestPt) {
  double u=0, du, umax, dMaxDraw; 
  pp3d_localpath localpathPt;
  int njnt = robotPt->njoints;
  double *distances;
  int i, isCollision, end_localpath =0;    
  int indCurrRobot = p3d_get_desc_curnum(P3D_ROBOT);
  configPt q;

  if(trajPt == NULL) {
    PrintInfo(("Warning: no trajectory to bio-test\n"));
    return FALSE;
  }
  localpathPt = trajPt->courbePt;
  distances = MY_ALLOC(double, njnt+1);
  while (localpathPt != NULL){
    umax = localpathPt->range_param;
    while (end_localpath < 2) {
      //warning : the step is based 
      //on the graphic dmax
      dMaxDraw = p3d_get_env_graphic_dmax();
      /* position of the robot corresponding to parameter u */
      q = localpathPt->config_at_param(robotPt, localpathPt, u);
      p3d_set_and_update_robot_conf(q);
      p3d_destroy_config(robotPt, q);
      (nTestPt++);
   
      //     bio_all_molecules_col_with_report();
      p3d_col_test_all();
      isCollision = biocol_robot_report(indCurrRobot);
      if(isCollision == TRUE) {
	MY_FREE(distances, double, njnt+1);
	return TRUE;
      }

      for (i=0; i<=njnt; i++){
	distances[i] = dMaxDraw;
      }
      du = localpathPt->stay_within_dist(robotPt, localpathPt,
					 u, FORWARD, distances);
      u+=du;
      if (u > umax-EPS6){
	u = umax;
	end_localpath++;
      }
    }
    localpathPt = localpathPt->next_lp;
    end_localpath = 0;
    u = 0;
  }
     MY_FREE(distances, double, njnt+1);
     return FALSE;
}




/////////////////////////////////////////////////////////////////////////////////////////////////////////
// TRAJECTORY EVALUATION
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void bio_evaluate_traj(FILE *contacts_file)
{
  pp3d_rob robotPt = (pp3d_rob) p3d_get_desc_curid(P3D_ROBOT);
  pp3d_localpath localpathPt;
  configPt q;
  p3d_poly **listpol1, **listpol2;
  int col_number;
  //double dist_surf;
  int nconfs;
  int MAX_SIZE_CONFS = 1000;
  int array_cnum[MAX_SIZE_CONFS];
  int total_cnum;
  int max_cnum;
  int thres1_cnum;
  int n_max_thres1;
  int indexligand;
  int i,j;
  double u=0.0;
  double dist_from_origin;
  char res1name[4],res2name[4];
  char res1part[4],res2part[4];
  int res1seq,res2seq;
  int indexletter;
  char atomname1[5],atomname2[5];

  if(robotPt->tcur == NULL)
    {
      PrintInfo(("bio_evaluate_traj : no current trajectory\n"));
      return;
    }

  // set BioCD mode
  bio_set_col_mode(SURFACE_DISTANCE_BIOCOL_MODE);
  //dist_surf =  bio_get_surface_d();
  //bio_set_surface_d(0.7);  // PROBAR OTROS VALORES !!!???
  
  localpathPt = robotPt->tcur->courbePt;
  nconfs = 0;
  total_cnum = 0.0;
  while (localpathPt != NULL){
    for(i=0;i<2;i++) {
      if(i==0)
	u = localpathPt->range_param/4.0;
      else if(i==1)
	u = localpathPt->range_param*(3.0/4.0);
      // place "robot" at intermediate conf. of the local path
      q = localpathPt->config_at_param(robotPt, localpathPt, u);
      p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, q);
      // call bio_coll
      if(is_ligand_in_robot(robotPt)== TRUE){
	// IF PROTEIN-LIGAND
	// WARNING !!! : suppose that there is only one ligand and thet it is the last "sub-robot"
      indexligand = get_number_of_bcd_robots() - 1;
      bio_molecule_col_no_autocol(indexligand);
      }
      else{
	//IF PROTEIN
	bio_all_molecules_col();
      }
      // evaluate
      biocol_report(&col_number,&listpol1,&listpol2);
      array_cnum[nconfs] = col_number;
      nconfs++;
      // compute distance of the baricenter from docking pos.
      // WARNING : suppose that the joint values 0 are at the docking pos.
      dist_from_origin = ffo_dist_from_root_pos(q);
      if(contacts_file == NULL) {
	printf("### CONF. NUM. %d : distance %f : num. heavy atoms in contact %d ###\n",
	       nconfs,dist_from_origin,col_number);
      }
      else {
	fprintf(contacts_file,"### CONF. NUM. %d : distance %f : num. heavy atoms in contact %d ###\n",
		nconfs,dist_from_origin,col_number); 	
      }
      for(j=0;j<col_number;j++) {
	get_AAtype_from_name(listpol1[j]->p3d_objPt->name,res1name);
	get_AAtype_from_name(listpol2[j]->p3d_objPt->name,res2name);
	res1seq = get_AAnumber_from_name(listpol1[j]->p3d_objPt->name);
	res2seq = get_AAnumber_from_name(listpol2[j]->p3d_objPt->name);
	indexletter=2;
        strcpy(atomname1, givemeword(listpol1[j]->poly->name,'.',&indexletter));
	indexletter=2;
        strcpy(atomname2, givemeword(listpol2[j]->poly->name,'.',&indexletter));
	//	if(listpol1[j]->p3d_objPt->jnt->bio_jnt_type == BIO_GAMMA_JNT)
	if((strcmp(atomname1,"N") == 0) ||
	   (strcmp(atomname1,"CA") == 0) ||
	   (strcmp(atomname1,"C") == 0) ||
	   (strcmp(atomname1,"O") == 0) ||
	   (strcmp(atomname1,"CB") == 0))
	  strcpy(res1part,"bkb");
	else
	  strcpy(res1part,"sch");
	//	if(listpol2[j]->p3d_objPt->jnt->bio_jnt_type == BIO_GAMMA_JNT)
	if((strcmp(atomname2,"N") == 0) ||
	   (strcmp(atomname2,"CA") == 0) ||
	   (strcmp(atomname2,"C") == 0) ||
	   (strcmp(atomname2,"O") == 0) ||
	   (strcmp(atomname2,"CB") == 0))
	  strcpy(res2part,"bkb");
	else
	  strcpy(res2part,"sch");

	if(strcmp(res1name,"LIG") == 0)
	  strcpy(res1part,"---");
	if(strcmp(res2name,"LIG") == 0)
	  strcpy(res2part,"---");

/* 	if(res1seq < 0) */
/* 	  res1seq = 1; */
/* 	if(res2seq < 0) */
/* 	  res1seq = 1; */
	if(contacts_file == NULL) {
	  printf("%s: %s-%d,%s    -     %s: %s-%d,%s\n",
		 listpol1[j]->poly->name,res1name,res1seq,res1part,
		 listpol2[j]->poly->name,res2name,res2seq,res2part);
	}    
	else {
	  fprintf(contacts_file, "%s: %s-%d,%s    -     %s: %s-%d,%s\n",
		 listpol1[j]->poly->name,res1name,res1seq,res1part,
		 listpol2[j]->poly->name,res2name,res2seq,res2part);
	}
      }
      total_cnum += col_number;
      p3d_destroy_config(robotPt, q);
    }
    localpathPt = localpathPt->next_lp;
    if(nconfs>MAX_SIZE_CONFS) {
      if(contacts_file == NULL) {
	printf("bio_evaluate_traj : too much localpaths in traj\n");
      }
      else {
	fprintf(contacts_file, "bio_evaluate_traj : too much localpaths in traj\n");
      }
      return;
    }
  }
  // average
  max_cnum = 0;
  for(i=0; i<nconfs; i++) {
    if(array_cnum[i] > max_cnum)
      max_cnum = array_cnum[i];
  }
  if(contacts_file == NULL) {
    printf("\n");
  }
  else {
    fprintf(contacts_file, "\n");
  }    
  thres1_cnum = max_cnum - (int) ceil((double)max_cnum/10.0);
  n_max_thres1 = 0;
  for(i=0; i<nconfs; i++) {
    if(array_cnum[i] > thres1_cnum)
      n_max_thres1++;
  }  
  if(contacts_file == NULL) {
    printf("Average enclosing along path : %f\n",(double)total_cnum/((double)nconfs));
    printf("Maximum enclosing value : %d\n",max_cnum);
    //printf("Enclosing value : %f\n",(double)max_cnum*(double)n_max_thres1/(double)nconfs);
  }
  else {
    fprintf(contacts_file, "Average enclosing along path : %f\n",(double)total_cnum/((double)nconfs));
    fprintf(contacts_file, "Maximum enclosing value : %d\n",max_cnum);
    //fprintf(contacts_file, "Enclosing value : %f\n",(double)max_cnum*(double)n_max_thres1/(double)nconfs);
  }
  // reset BioCD mode
  //bio_set_surface_d(dist_surf);
  bio_set_col_mode(NORMAL_BIOCOL_MODE);
  
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS TO MANAGE INFORMATION ABOUT INVALID LOCAL-PATH CONFIGURATIONS 
/////////////////////////////////////////////////////////////////////////////////////////////////////////

static int there_is_a_current_q_inv = FALSE;

static void bio_reset_current_q_inv(p3d_rob *robotPt) 
{
  there_is_a_current_q_inv = FALSE;
}

void bio_set_current_q_inv(p3d_rob *robotPt, p3d_localpath *localpathPt, configPt q_inv) 
{
  there_is_a_current_q_inv = TRUE;
  p3d_copy_config_into(robotPt,q_inv,&(robotPt->currect_q_inv));  
}

int bio_get_current_q_inv(p3d_rob *robotPt, configPt q_invPt) 
{ 
  if(there_is_a_current_q_inv) {
    p3d_copy_config_into(robotPt,robotPt->currect_q_inv,&q_invPt);
  }
  return(there_is_a_current_q_inv);
}
