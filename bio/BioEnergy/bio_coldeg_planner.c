#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Graphic-pkg.h"
#include "Localpath-pkg.h"
#include "Planner-pkg.h"
#include "Bio-pkg.h"
#include "Util-pkg.h"
#include "include/Energy-pkg.h"
#include "include/bioenergy_common.h"


/**********************************************************************/
/**********************************************************************/
/* Exploration of Collective Degrees                                  */
/**********************************************************************/
/**********************************************************************/
/* MAIN IDEAS:
   - using an RRT-like algorithm to explore the space of the collective degrees
   - sampling : sample conformations using the collective degrees for the backbone
     and random sampling for the side-chains (and the rest)
   - nearest neighbor : using a metric in the space of the collective degrees
   - local paths : 
     - bkb joints : change linearly in the space of the collective degrees
     - sch joints : change linearly in the conformation space

     * the checking step along the local paths is constant. 
       it is computed from a linear aproximation of the maximum 
       deplacement produced by the collective degrees 

OLD IDEA: the variation in the space of the collective degrees is approximated
          by a set of small linear paths in the conformation space
          (intermediate Nodes have to be saved)
*/


/**********************************************************************/

// GLOBAL VARIABLES
static double *max_step_coldeg = NULL;

static int coldeg_weighted_RRT = 0;
static double coldeg_best_weight;

#define MAX_COLDEG_N_EXPAN_WITHOUT_IMPROVE 100
static int coldeg_n_expan_without_improve = 0;

/**********************************************************************/

static double compute_jnt_xyz_dep_from_pos0(p3d_jnt *jntPt)
{
  p3d_vector3 pos0,pos,pos_diff;
  double dep;

  p3d_jnt_get_vect_point(jntPt,pos0);
  p3d_jnt_get_cur_vect_point(jntPt,pos);
  p3d_vectSub(pos,pos0,pos_diff);
  dep = (double) p3d_vectNorm(pos_diff);

  return (dep);
}


// function computing de maximum deplacement of an atom for each collective degree
static double bio_nmode_compute_max_deplacement(int n_coldeg, int coldeg)
{
  p3d_rob *robPt;
  configPt q;
  double *coldeg_q;
  int i;
  double dep,max_dep = 0;
  int ij;
  p3d_jnt *jntPt;

  // WARNING : only made for the current robot
  robPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 

  coldeg_q = bio_alloc_coldeg_config(n_coldeg);
  q = p3d_alloc_config(robPt);

  // set coldeg_i to maximum value
  for(i=0; i<n_coldeg; i++) {
    if(i == coldeg)
      coldeg_q[i] = 1.0;
    else
      coldeg_q[i] = 0.0;
  }

  bio_infer_q_from_coldeg_conf(robPt,&q,coldeg_q,n_coldeg);
  p3d_set_and_update_this_robot_conf(robPt,q);

  for (ij=0; ij < robPt->njoints; ij++){
    jntPt = robPt->joints[ij];
    dep = compute_jnt_xyz_dep_from_pos0(jntPt);
    if(dep > max_dep)
      max_dep = dep;
  }

  // set coldeg_i to mainimum value
  for(i=0; i<n_coldeg; i++) {
    if(i == coldeg)
      coldeg_q[i] = -1.0;
    else
      coldeg_q[i] = 0.0;
  }

  bio_infer_q_from_coldeg_conf(robPt,&q,coldeg_q,n_coldeg);
  p3d_set_and_update_this_robot_conf(robPt,q);

  for (ij=0; ij < robPt->njoints; ij++){
    jntPt = robPt->joints[ij];
    dep = compute_jnt_xyz_dep_from_pos0(jntPt);
    if(dep > max_dep)
      max_dep = dep;
  }
  
  bio_destroy_coldeg_config(coldeg_q,n_coldeg);
  p3d_destroy_config(robPt,q);
  
  return max_dep;
}


static int bio_nmode_compute_step_lengths(int n_coldeg)
{
  int i;
  double col_dep;
  double max_dep_coldeg_i;
  
  bio_col_get_step_deplacement(&col_dep);

  for(i=0; i<n_coldeg; i++) {
    max_dep_coldeg_i = bio_nmode_compute_max_deplacement(n_coldeg,i);
    //max_dep_coldeg_i = 0.05;
    max_step_coldeg[i] = col_dep/max_dep_coldeg_i;
  }

  return 1;
}


static void bio_compute_coldeg_coll_test_nsteps(double *coldeg_qi, double *coldeg_qf, int n_coldeg, 
						int *n_steps)
{
  int i;
  int n, max_n = 0;
  double d_mod_i;

  for(i=0; i<n_coldeg; i++) {
    d_mod_i = fabs(coldeg_qf[i] - coldeg_qi[i]);
    n = (int) ceil(d_mod_i/max_step_coldeg[i]);
    if(n > max_n)
      max_n = n;
  }
  *n_steps = max_n;
}


/**********************************************************************/

// main exploration fuction
int bio_explore_coldeg(void)
{
  int curr_planner;
  int n_coldeg;
/*   int fres, lres; */
/*   p3d_rob *robPt; */
  
  // compute legths of steps for collision checking as a function
  // of the deplacements produced by the collective degrees
  // WARNING : in the future, this should be made for each molecule

  n_coldeg = bio_get_num_collective_degrees();
  max_step_coldeg =  MY_ALLOC(double,n_coldeg); 
  bio_nmode_compute_step_lengths(n_coldeg);

  // deactivate collision for sidechains
/*   robPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);  */
/*   fres = get_AAnumber_from_jnt(robPt->joints[3]); */
/*   lres = get_AAnumber_from_jnt(robPt->joints[robPt->njoints - 1]); */
/*   printf("Fres num = %d  Lres num = %d\n",fres,lres); */
/*   supress_sc_rigid_sequence(robPt->num, fres, lres); */

  curr_planner = p3d_get_MOTION_PLANNER();
  p3d_set_MOTION_PLANNER(BIO_COLDEG_RRT);

  CB_specific_search_obj(NULL,0); 
  p3d_set_MOTION_PLANNER(curr_planner);

  // search "best node"  
  bio_search_max_weight_in_curr_rrt();

  // re-activate collision for side-chains
/*   activate_sc_rigid_sequence(robPt->num, fres, lres);  */

  MY_FREE(max_step_coldeg,double,n_coldeg);  

  return 1;
}

/**********************************************************************/
// collective degrees configuration

double *bio_alloc_coldeg_config(int n_coldeg) 
{
  double *coldeg_q;

  coldeg_q = MY_ALLOC(double, n_coldeg);
  return (coldeg_q);
}


void  bio_destroy_coldeg_config(double *coldeg_q, int n_coldeg)
{
  MY_FREE(coldeg_q,double,n_coldeg);
}


void bio_copy_coldeg_config(double *coldeg_qsrc, double *coldeg_qdst, int n_coldeg)
{
  int i;

  for(i=0; i<n_coldeg; i++) {
    coldeg_qdst[i] = coldeg_qsrc[i];
  }
}


void bio_print_coldeg_config(double *coldeg_q, int n_coldeg)
{
  int i;

  for(i=0; i<n_coldeg; i++) {
    printf("mode %d = %f ,",i,coldeg_q[i]);
  }
  printf("\n");
}


double bio_dist_coldeg_config(double *coldeg_q1, double *coldeg_q2, int n_coldeg)
{
  double d = 0.,dsqr = 0.;
  int i;

  for(i=0; i<n_coldeg; i++) {
    dsqr += SQR(coldeg_q2[i] - coldeg_q1[i]);
  }
  d = sqrt(dsqr);
  return d;
}


static void bio_compute_intermediate_coldeg_config(double *coldeg_qi, double *coldeg_qf, double *coldeg_qn, int n_coldeg,
					     double totn_steps, int n_steps)
{
  int i;

  for(i=0; i<n_coldeg; i++) {
    coldeg_qn[i] = coldeg_qi[i] + (coldeg_qf[i] - coldeg_qi[i]) * ((double)n_steps) / ((double)totn_steps);
  }
}


static p3d_node *bio_coldeg_nearest_neighbor(double *coldeg_q, int n_coldeg, p3d_compco *comp)
{
  p3d_list_node *ListNode = comp->dist_nodes;
  p3d_node *Nmin = NULL;
  double dmin,d;

  dmin = P3D_HUGE;
  while(ListNode != NULL) {
    //    if((comp->nnode == 1)||(ListNode->N->n_fail_extend < MAXN_FAILS)) {
    // ONLY measure distance in the space of the collective degrees
    d = bio_dist_coldeg_config(coldeg_q,ListNode->N->coldeg_q,n_coldeg);
    if (d < dmin) {
      dmin = d;
      Nmin = ListNode->N;
    }
    //    }
    ListNode = ListNode->next;
  }
  return Nmin;
}

static double refweight = 0.0;

void bio_coldeg_planner_init_weight(double w)
{
  refweight = w;
  coldeg_best_weight = refweight;
  coldeg_weighted_RRT = 1;
}

static p3d_node *bio_coldeg_weighted_nearest_neighbor(double *coldeg_q, int n_coldeg, p3d_compco *comp, 
						      int wsign)
{
  p3d_list_node *ListNode = comp->dist_nodes;
  p3d_node *Nmin = NULL;
  double dmin,d;
  //double weight_param = 10.0;

  dmin = P3D_HUGE;
  while(ListNode != NULL) {
    //    if((comp->nnode == 1)||(ListNode->N->n_fail_extend < MAXN_FAILS)) {
    // ONLY measure distance in the space of the collective degrees
    d = bio_dist_coldeg_config(coldeg_q,ListNode->N->coldeg_q,n_coldeg) 
      //+ ((double) wsign) * (refweight - ListNode->N->weight) * n_coldeg;
       * ( 1.0 - (((double) wsign) * (-refweight + ListNode->N->weight)/refweight));
    if (d < dmin) {
      dmin = d;
      Nmin = ListNode->N;
    }
    //    }
    ListNode = ListNode->next;
  }
  return Nmin;
}


void bio_copy_coldeg_q_in_N(p3d_node *N, double *coldeg_q, int n_coldeg)
{
  N->coldeg_q = bio_alloc_coldeg_config(n_coldeg);
  bio_copy_coldeg_config(coldeg_q,N->coldeg_q,n_coldeg);  
}


/**********************************************************************/

static void bio_coldeg_shoot(double *coldeg_q, int n_coldeg)
{
  int i;
  
  for(i=0; i<n_coldeg; i++) {
    coldeg_q[i] = p3d_random(-1.0,1.0);
  }
}

/**********************************************************************/
static int bio_compute_and_validate_coldeg_localpath(p3d_graph *G, p3d_node *Nnear, 
						     configPt qrand, double *coldeg_qrand, int n_coldeg,
						     configPt *qnewPt, double **coldeg_qnewPt)
{
  p3d_localpath *localpathPt;
  p3d_rob *robPt = G->rob;
  //double l_step_test;
  int n_steps;
  int count_steps;
/*   configPt q; */
  configPt qinter;
  double *coldeg_qinter;
  double u, du, umax; /* parameters along the local path */

  p3d_copy_config_into(robPt,Nnear->q,qnewPt);
  bio_copy_coldeg_config(Nnear->coldeg_q,*coldeg_qnewPt,n_coldeg); 

  /* WARNING :
     The collision test step is computed only considering the deplacement produced by 
     the collective degrees (deplacement of bkb atoms).
     This can produce big steps over the local-path for the side-chains !!!
  */
  bio_compute_coldeg_coll_test_nsteps(Nnear->coldeg_q,coldeg_qrand,n_coldeg,&n_steps);

  // compute the linear interpolation path
  if (!(localpathPt = p3d_local_planner(robPt, Nnear->q, qrand)))
    {
      PrintInfo(("impossible de planifier\n"));
      return 0;
    }
  (G->nb_local_call)++;

  umax = localpathPt->range_param;
  du = umax/((double)n_steps);

  u = 0;
  coldeg_qinter = bio_alloc_coldeg_config(n_coldeg);
  qinter = p3d_alloc_config(robPt);
  for(count_steps=0; count_steps<n_steps; count_steps++) {
    u+=du;
    if (u > umax - EPS6) {
      u = umax;
    }
    bio_compute_intermediate_coldeg_config(Nnear->coldeg_q,coldeg_qrand,coldeg_qinter,n_coldeg,
					   n_steps,count_steps+1);
    bio_infer_q_from_coldeg_conf(robPt,&qinter,coldeg_qinter,n_coldeg);
    // SIDE_CHAINS DO NOT MOVE (TEMPORARY MODIF.)
/*     q = localpathPt->config_at_param(robPt,localpathPt,u); */
/*     bio_copy_sch_conf(robPt,q,qinter); */
/*     p3d_destroy_config(robPt,q);      */
    if(!p3d_set_and_update_this_robot_conf(robPt,qinter)) {
      bio_destroy_coldeg_config(coldeg_qinter,n_coldeg);
      p3d_destroy_config(robPt,qinter);
      localpathPt->destroy(robPt,localpathPt);
      return 0;
    }
    if(bio_all_molecules_col() > 0) {
      // NEW
      //if(!bio_is_collfree_conf_with_possible_sch_perturbation(robPt,qinter)) {
      if(!bio_deform_schs_avoiding_collision(robPt,qinter,0)) {
	bio_destroy_coldeg_config(coldeg_qinter,n_coldeg);
	p3d_destroy_config(robPt,qinter);
	localpathPt->destroy(robPt,localpathPt);
	return 0;
      }
    }
    p3d_copy_config_into(robPt,qinter,qnewPt);
    bio_copy_coldeg_config(coldeg_qinter,*coldeg_qnewPt,n_coldeg); 
  }
  bio_destroy_coldeg_config(coldeg_qinter,n_coldeg);
  p3d_destroy_config(robPt,qinter);
  localpathPt->destroy(robPt,localpathPt);
  return 1;
}

/**********************************************************************/

static p3d_node *bio_coldeg_extend_on_surface(p3d_graph *G, p3d_node *Nnear, 
					      configPt qrand, double *coldeg_qrand, int n_coldeg)
{
  p3d_node *NewNode = NULL;
  configPt qnew;
  double *coldeg_qnew;
  double dist_coldeg = 0;
  int *iksol = NULL;
  double dmax = p3d_get_env_dmax();
  p3d_rob *robPt = G->rob;
  //  p3d_jnt *triade_jntPt[3];
  //  int triade_conftype;
  p3d_vector3 *goal_jntcoordsPt;
  //double *goal_jntvPt;
  p3d_jnt **pairs_jntPt;
  int nump;
  int conftype;
  //  int fstnode=1;

  if (p3d_equal_config(robPt, qrand, Nnear->q))
    {
      if((robPt->cntrt_manager == NULL)||(robPt->cntrt_manager->cntrts == NULL)) /* no display if constraints */ 
	PrintInfo(("qrand invalide\n"));
      return NULL;
    }
  (G->nb_q_closed)++;

  qnew = p3d_alloc_config(robPt);
  coldeg_qnew = bio_alloc_coldeg_config(n_coldeg);

  bio_compute_and_validate_coldeg_localpath(G,Nnear,qrand,coldeg_qrand,n_coldeg,&qnew,&coldeg_qnew);

  dist_coldeg = bio_dist_coldeg_config(Nnear->coldeg_q,coldeg_qnew,n_coldeg);
    
  if (EQ(dist_coldeg,0.0)) {
    bio_destroy_coldeg_config(coldeg_qnew,n_coldeg);
    p3d_destroy_config(robPt,qnew);
    return NULL;
  }
  
  // for displaying
  p3d_set_and_update_this_robot_conf_without_cntrt(robPt,qnew);
  //p3d_set_and_update_robot_conf(qnew);
  p3d_get_iksol_vector(robPt->cntrt_manager,iksol);
  NewNode = p3d_APInode_make_multisol(G, qnew, iksol);
  bio_copy_coldeg_q_in_N(NewNode,coldeg_qnew,n_coldeg);
  bio_destroy_coldeg_config(coldeg_qnew,n_coldeg);

  // for deformation analysis
  // NewNode->weight = ffo_dist_from_root_pos(NewNode->q);
  /* WARNING : THE WEIGHT IS DECIDED DEPENDING ON ONE ONLY TRIADE !!! */
  //  if(bio_get_triade(0,triade_jntPt,&triade_conftype)) {
  //    NewNode->weight = bio_measure_triade_surface_area(triade_jntPt,NewNode->q);
/*     if(fstnode) { */
/*       refweight = NewNode->weight; */
/*       fstnode = 0; */
/*     } */
  //    printf("Triade area for node %d = %f\n",NewNode->num,NewNode->weight);

  if(bio_get_goal_jnt_coordinates(&goal_jntcoordsPt)) {
    //if(bio_get_goal_jnt_coordinates(&goal_jntvPt)) {
    NewNode->weight = bio_rmsd_to_goal_jnt_coords(robPt,goal_jntcoordsPt,NewNode->q);
    //NewNode->weight = bio_rmsd_to_goal_jnt_coords(robPt,goal_jntvPt,NewNode->q);
    printf("node %d : RMSD to goal (for all CA) = %f  ",NewNode->num,NewNode->weight);
    //printf("RMSD to goal (for bkb jnt values): node %d = %f  ",NewNode->num,NewNode->weight);
    if(NewNode->weight < coldeg_best_weight) {
      coldeg_best_weight = NewNode->weight;
      printf("- decreased RMSD");
      coldeg_n_expan_without_improve = 0;
    }
    else {
      coldeg_n_expan_without_improve++;
    }
    printf("\n");
  }
  else if(bio_get_pairs_for_dist(&nump,&pairs_jntPt,&conftype)) {
    NewNode->weight = bio_measure_distance_between_atom_pairs(nump,pairs_jntPt,NewNode->q);
    printf("Pairs distance for node %d = %f  ",NewNode->num,NewNode->weight);
    if(conftype == 1) {
      if(NewNode->weight > coldeg_best_weight) {
	coldeg_best_weight = NewNode->weight;
	printf("- increased pairs distance");
	coldeg_n_expan_without_improve = 0;
      }
      else {
	coldeg_n_expan_without_improve++;
      }
    }
    else{
      if(NewNode->weight < coldeg_best_weight) {
	coldeg_best_weight = NewNode->weight;
	printf("- decreased pairs distance");
	coldeg_n_expan_without_improve = 0;
      }
      else {
	coldeg_n_expan_without_improve++;
      }
    }
    printf("\n");
  }

  NewNode->type = LINKING;
  NewNode->radius = p3d_GetLambda()*dmax;
  NewNode->boundary = FALSE;
  
  p3d_insert_node(G,NewNode);
  p3d_create_edges(G,Nnear,NewNode,dist_coldeg);
  p3d_add_node_compco(NewNode,Nnear->comp);

  return NewNode;
}

/**********************************************************************/

// internal function making 1 expansion step of the coldeg-RRT 
static int bio_expand_one_coldeg_rrt(p3d_graph *G, p3d_compco **CompPt, configPt q, 
				     double *coldeg_q, int n_coldeg)
{
  int Added = FALSE;
  p3d_node *Nnear, *NewNode;
  //  p3d_jnt *triade_jntPt[3];
  //  int triade_conftype;
  p3d_vector3 *goal_jntcoordsPt;
  //double *goal_jntvPt;
  p3d_jnt **pairs_jntPt;
  int nump;
  int conftype = 0;
 
  if(bio_get_goal_jnt_coordinates(&goal_jntcoordsPt)) {
    //if(bio_get_goal_jnt_coordinates(&goal_jntvPt)) {
    Nnear = bio_coldeg_weighted_nearest_neighbor(coldeg_q,n_coldeg,*CompPt,-1);
  }
  //  if(bio_get_triade(0,triade_jntPt,&triade_conftype)) {
  else if(bio_get_pairs_for_dist(&nump,&pairs_jntPt,&conftype)) {
    /* WARNING : THE DIRECTION (INCREMENT/DECREMENT) FOR THE WEIGHT 
               IS DECIDED DEPENDING ON ONE ONLY TRIADE !!! */
    Nnear = bio_coldeg_weighted_nearest_neighbor(coldeg_q,n_coldeg,*CompPt,conftype);
    //Nnear = bio_coldeg_nearest_neighbor(coldeg_q,n_coldeg,*CompPt);
  } 
  else {
    Nnear = bio_coldeg_nearest_neighbor(coldeg_q,n_coldeg,*CompPt);
  }

  G->n_call_nearest++;
  p3d_SetCurrentNearNode(Nnear);

  NewNode = bio_coldeg_extend_on_surface(G,Nnear,q,coldeg_q,n_coldeg);

  if (NewNode)
    {
      // NOTE : SUPPOSE ONLY ONE COMPONENT !!!
/*       *CompPt = NewNode->comp; */
/*       CompConnect = G->comp;   */
/*       for(jcomp=0;(jcomp<G->ncomp)&&(CompConnect->num<=G->ncomp);jcomp++) */
/* 	{ */
/* 	  NextCompConnect = CompConnect->suiv; */
/* 	  if (CompConnect->num!=(*CompPt)->num) */
/* 	      { */
/* 		Nnear = hrm_nearest_neighbor(rob,NewNode->q,CompConnect); */
/* 		if (p3d_APInode_linked(G,NewNode,Nnear,&dist)) */
/* 		  { */
/* 		    PrintInfo(("RRT linking\n")); */
/* 		    if((*CompPt)->num<CompConnect->num) */
/* 		      p3d_merge_comp(G,*CompPt,&CompConnect); */
/* 		    else */
/* 		      { */
/* 			p3d_merge_comp(G,CompConnect,CompPt); */
/* 			*CompPt = CompConnect; */
/* 		      } */
		    
/* 		    p3d_create_edges(G,Nnear,NewNode,dist); */
/* 		  } */
/* 	      } */
/* 	  CompConnect = NextCompConnect; */
/* 	} */
      Added = TRUE;
      Nnear->n_fail_extend = 0;

      // STOP CONDITION BASED ON WEIGHT
/*       if(conftype != 0) { */
	
      // TO DO !!!

/*       } */

    }
  else 
    {
      Nnear->n_fail_extend++;
    }
  
  return Added;
}


// external function making 1 expansion step of the coldeg-RRT 
int bio_expand_coldeg_rrt(p3d_graph *G,int (*fct_stop)(void))
{
/*   configPt q; */
  configPt q_adapt;
  double *coldeg_q;
  int nnodemax = p3d_get_COMP_NODES();
  int Added = FALSE;
  int Stop = FALSE;
/*   int actRLG; */
  int n_coldeg;
  //  p3d_jnt *triade_jntPt[3];
  //  int triade_conftype;
  //p3d_vector3 *goal_jntcoordsPt;
  //double *goal_jntvPt;
  //p3d_jnt **pairs_jntPt;
  //int nump;
  //int conftype;

  n_coldeg = bio_get_num_collective_degrees();

/*   q = p3d_alloc_config(G->rob); */
  coldeg_q = bio_alloc_coldeg_config(n_coldeg);
  q_adapt = p3d_alloc_config(G->rob);
  while(!Stop)
    {
      if (fct_stop)
	if (!(*fct_stop)())
	  {
	    PrintInfo(("coldeg-RRT building canceled\n"));
/* 	    p3d_destroy_config(G->rob, q); */
	    bio_destroy_coldeg_config(coldeg_q,n_coldeg);
	    return FALSE;
	  }
      Added = FALSE;
      Stop = TRUE;

      // SIDE_CHAINS DO NOT MOVE (TEMPORARY MODIF.)
/*       actRLG = p3d_get_RLG(); */
/*       p3d_set_RLG(0); */
/*       p3d_shoot(G->rob,q,1); */
/*       p3d_set_RLG(actRLG);  */

      // randomly sampling a point in the space of the collective degrees
      // suppose that each collective degree variates in [-1.0,1.0]
      bio_coldeg_shoot(coldeg_q,n_coldeg);

      // generate the random conformation :
      // - adapt the bkb conformation from coldeg_q 
      // - pick the sch conformation from q 
      bio_infer_q_from_coldeg_conf(G->rob,&q_adapt,coldeg_q,n_coldeg);
      // SIDE_CHAINS DO NOT MOVE (TEMPORARY MODIF.)
/*       bio_copy_sch_conf(G->rob,q,q_adapt);  */

      // stop-condition by number of nodes
      if (G->search_start->comp->nnode < nnodemax) {
	// NEW stop-condition 
	/* NOTE : this condition is very simple
	   it could be better to consider the average amount of the improvement 
	   over a given number of expansion */
	if((!coldeg_weighted_RRT) ||
	   ((coldeg_weighted_RRT) && (coldeg_n_expan_without_improve < MAX_COLDEG_N_EXPAN_WITHOUT_IMPROVE))) {
	  Added = bio_expand_one_coldeg_rrt(G, &(G->search_start->comp), q_adapt, coldeg_q, n_coldeg);
	  Stop = Added;
	}
      }
    }
  PrintInfo(("  %4d   \r",G->search_start->comp->nnode));
/*   p3d_destroy_config(G->rob, q); */
  p3d_destroy_config(G->rob, q_adapt);
  bio_destroy_coldeg_config(coldeg_q,n_coldeg);
  return Added;
}


