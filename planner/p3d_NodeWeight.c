#include "Planner-pkg.h"
#include "Bio-pkg.h"
#include "Collision-pkg.h"
#include "Move3d-pkg.h"
#include "P3d-pkg.h"

#ifdef ENERGY
#include "../bio/BioEnergy/include/Energy-pkg.h"
#endif

#ifdef ENERGY
extern  void bio_coldeg_planner_init_weight(double weight);
#endif
static double p3d_get_best_weight(void);
static double p3d_get_node_weight_factor(p3d_node *N);
static int p3d_set_node_weight(p3d_rob *robPt, p3d_node *N);
static void p3d_set_best_weight(double w);
static void p3d_set_best_weight(double w);
static double p3d_get_ref_weight(void);
static void p3d_set_ref_weight(double w);
static int p3d_get_n_expan_without_w_improve(void);
static void p3d_set_n_expan_without_w_improve(int n);
static void p3d_inc_n_expan_without_w_improve(void);
static double p3d_get_norm_rate_n_expan_without_w_improve(void);


/*
 * FLAG to know if nodes ared weighted.
 * Weight of nodes can be used in the choice
 * of the best node in an expansion process
 */
static int IS_WEIGHTED_CHOICE = FALSE;

/*
 * Note: the integer values of the different 
 * NODE_WEIGHT_STRATEGIEs are defined in the
 * p3d_type.h file 
 */

static int NODE_WEIGHT_STRATEGY = RMSD_DISANCE_WEIGHT;
//static int NODE_WEIGHT_STRATEGY = ENERGY_WEIGHT;


/**
 * p3d_SetIsWeightedChoice
 * Set if the nodes are weighted or not
 * param[In]: IsWeightedChoice: should be TRUE if
 * the nodes are weighted.
 */
void p3d_SetIsWeightedChoice(int IsWeightedChoice)
{
  IS_WEIGHTED_CHOICE = IsWeightedChoice;
}

/**
 * p3d_GetIsWeightedChoice
 * Get if the nodes are weighted or not
 * @return: TRUE if the nodes are weighted.
 */
int p3d_GetIsWeightedChoice(void)
{
  return(IS_WEIGHTED_CHOICE);
}

/**
 * p3d_GetNodeWeightStrategy
 * Get the strategy used to weight a node
 * @return:  the strategy used to weight a node.
 */
int p3d_GetNodeWeightStrategy(void) {
  return NODE_WEIGHT_STRATEGY;
}

/**
 * p3d_SetNodeWeightStrategy
 * Set the strategy used to weight a node
 * @param[In]:  the strategy used to weight a node.
 */
void p3d_SetNodeWeightStrategy(int NodeWeightStrat) {
  NODE_WEIGHT_STRATEGY = NodeWeightStrat;
}

/**
 * p3d_GetNodeWeight
 * Get the weight of a given node depending
 * of the weighting strategy
 * @param[In]: NodePt: the given node
 * @return: the weight of the node depending of the 
 * weighting strategy. 
 * Note: currently the returned value is different than the
 * NodePt->weight field. Should be modified to remove ambiguity
 */
double p3d_GetNodeWeight(p3d_node* NodePt) {
  double NodeWeight = -1.;
  switch(p3d_GetNodeWeightStrategy()) {

  case RMSD_DISANCE_WEIGHT:
    /*Previously implemented version of tp get a node weight factor */
    NodeWeight = p3d_get_node_weight_factor(NodePt);
    break;
  case ENERGY_WEIGHT:
    //todo
    //    NodeWeight = p3d_GetEnergyWeight(NodePt);
    break;
  default:
    NodeWeight =  p3d_get_node_weight_factor(NodePt);
  }
  return NodeWeight;
}

/**
 * p3d_SetNodeWeight
 * Set the weight of a given node depending
 * of the weighting strategy. The function modify
 * the  field NodePt->weight.
 * @param[In]: GraphPt: the robot graph
 * @param[In]: NodePt: the given node
 */
void p3d_SetNodeWeight(p3d_graph* GraphPt, p3d_node* NodePt) {

  switch(p3d_GetNodeWeightStrategy()) {
  case RMSD_DISANCE_WEIGHT:
    p3d_set_node_weight(GraphPt->rob,NodePt);
    break;
  case ENERGY_WEIGHT:
   //todo
    //    p3d_SetEnergyWeight(GraphPt->rob, NodePt);
    break;
  default:
    p3d_set_node_weight(GraphPt->rob, NodePt);
  }
}

/*********************************************************/
/* functions to handle stop condition based on weight    */
/*********************************************************/

static int IS_WEIGHT_STOP_COND = FALSE;
static double STOP_WEIGHT;
static int SIGN_STOP_WEIGHT = 0;
static int IS_DIFFU_STOPPED_BY_WEIGHT = FALSE;

void p3d_SetStopWeightAndSign(double stop_weight, int sign_stop_weight)
{
  STOP_WEIGHT = stop_weight;
  SIGN_STOP_WEIGHT = sign_stop_weight;
  p3d_SetIsWeightStopCondition(TRUE);
}

void p3d_GetStopWeightAndSign(double *stop_weightPt, int *sign_stop_weightPt)
{
  *stop_weightPt = STOP_WEIGHT;
  *sign_stop_weightPt = SIGN_STOP_WEIGHT;
}

void p3d_SetDiffuStoppedByWeight(double stopped_by_weight)
{
  IS_DIFFU_STOPPED_BY_WEIGHT = stopped_by_weight;
}

int p3d_GetDiffuStoppedByWeight(void)
{
  return (IS_DIFFU_STOPPED_BY_WEIGHT);
}


int p3d_GetIsWeightStopCondition(void)
{
  return (IS_WEIGHT_STOP_COND);
}


void p3d_SetIsWeightStopCondition(int IsWeightStopCondition) {
  IS_WEIGHT_STOP_COND = IsWeightStopCondition;
}


/*********************************************************/
/* functions to handle node weight                       */
/*********************************************************/

static double best_weight = 0.0;
static double refweight = 0.0;
#define MAX_N_EXPAN_WITHOUT_W_IMPROVE 100
static int n_expan_without_w_improve = 0;

static double p3d_get_best_weight(void)
{
  return (best_weight);
}

static void p3d_set_best_weight(double w)
{
  best_weight = w;
}

static double p3d_get_ref_weight(void)
{
  return (refweight);
}

static void p3d_set_ref_weight(double w)
{
  refweight = w;
}

static int p3d_get_n_expan_without_w_improve(void)
{
  return (n_expan_without_w_improve);
}

static void p3d_set_n_expan_without_w_improve(int n)
{
  n_expan_without_w_improve = n;
}

static void p3d_inc_n_expan_without_w_improve(void)
{
  n_expan_without_w_improve++;
}

double p3d_get_rate_n_expan_without_w_improve(void)
{
  return (((double)MAX_N_EXPAN_WITHOUT_W_IMPROVE)/((double)n_expan_without_w_improve + 1.0));
}

static double p3d_get_norm_rate_n_expan_without_w_improve(void)
{
  return (((double)MAX_N_EXPAN_WITHOUT_W_IMPROVE-(double)n_expan_without_w_improve)/((double)MAX_N_EXPAN_WITHOUT_W_IMPROVE));
}

/*********************************************************/

void p3d_init_root_weight(p3d_graph *G)
{

  p3d_vector3 *goal_jntcoordsPt;
  //double *goal_jntvPt;
  p3d_jnt **pairs_jntPt;
  int nump;
  int conftype;

  // is defined goal conf
  #ifdef BIO
  if(bio_get_goal_jnt_coordinates(&goal_jntcoordsPt)) {
    if(G->search_start->weight == 0.0) {
      p3d_set_and_update_this_robot_conf_without_cntrt(G->rob,G->search_start->q);
      G->search_start->weight = bio_rmsd_to_goal_jnt_coords(G->rob,goal_jntcoordsPt,G->search_start->q);
      p3d_set_ref_weight(G->search_start->weight);
      p3d_set_best_weight(G->search_start->weight);
      printf("root node weight = %f\n",G->search_start->weight);
    }
  }
  // compute triade surface for root node
  //  if(bio_get_triade(0,triade_jntPt,&triade_conftype)) {
  else if(bio_get_pairs_for_dist(&nump,&pairs_jntPt,&conftype)) {
    if(G->search_start->weight == 0.0) {
      p3d_set_and_update_this_robot_conf_without_cntrt(G->rob,G->search_start->q);
      //G->search_start->weight = bio_measure_triade_surface_area(triade_jntPt,G->search_start->q);
      G->search_start->weight = bio_measure_distance_between_atom_pairs(nump,pairs_jntPt,G->search_start->q);
      p3d_set_ref_weight(G->search_start->weight);
      p3d_set_best_weight(G->search_start->weight);
      printf("root node weight = %f\n",G->search_start->weight);
    }
  }
  else {
    p3d_set_ref_weight(0.0);
    p3d_set_best_weight(0.0);
  }
  #endif

#ifdef ENERGY     
  if(p3d_get_MOTION_PLANNER() == BIO_COLDEG_RRT)
    bio_coldeg_planner_init_weight(G->search_start->weight);
#endif

}
/*********************************************************/

int p3d_get_w_inc_dir(void) 
{
  p3d_vector3 *goal_jntcoordsPt;
  //double *goal_jntvPt;
  p3d_jnt **pairs_jntPt;
  int nump;
  int conftype;
#ifdef BIO
  if(bio_get_goal_jnt_coordinates(&goal_jntcoordsPt)) {
    return (-1);
  }
  else if(bio_get_pairs_for_dist(&nump,&pairs_jntPt,&conftype)) {
    return (conftype);
  }
  else{
    // NOTE: suppose that, by default, thw weight contains the distance 
    //       of the "reference" frame from the root position 
    return (1);
  }
#endif
}

/*********************************************************/

static double p3d_get_node_weight_factor(p3d_node *N)
{
  int wsign;
  double refw;
  double wfact = 1.0;
  double HIGHEST_W = 1E5;  // WARNING : PROBLEM WHEN WEIGHT = 0 !!!

  // NOTE : the implementation of this function is only one among other possible choices
  //        we should also try a factor of the type : (bestW - W) / bestW 

  refw = p3d_get_ref_weight();
  wsign = p3d_get_w_inc_dir();
  if(wsign == 1) {
    if(refw != 0.0)
      wfact =  refw/N->weight;
    else {
      if(N->weight != 0.0)
	wfact =  1.0/N->weight;
      else 
	wfact = HIGHEST_W;
    }
  }
  else if(wsign == -1) {
    if(refw != 0.0)
      wfact =  N->weight/refw;
    else {
      if(N->weight != 0.0)
	wfact =  N->weight;
      else 
	wfact = HIGHEST_W;      
    }  
  }  

  return (wfact);
}
/*********************************************************/



static int p3d_set_node_weight(p3d_rob *robPt, p3d_node *N)
{
  p3d_vector3 *goal_jntcoordsPt;
  //double *goal_jntvPt;
  p3d_jnt **pairs_jntPt;
  int nump;
  int conftype;
  #ifdef BIO
  if(bio_get_goal_jnt_coordinates(&goal_jntcoordsPt)) {
    N->weight = bio_rmsd_to_goal_jnt_coords(robPt,goal_jntcoordsPt,N->q);
    printf("node %d : RMSD to goal (for all CA) = %f  ",N->num,N->weight);
    if(N->weight < p3d_get_best_weight()) {
      p3d_set_best_weight(N->weight);
      printf("- decreased RMSD\n");
      p3d_set_n_expan_without_w_improve(0);
    }
    else {
      p3d_inc_n_expan_without_w_improve();
      printf("\r");
    }
  }
  else if(bio_get_pairs_for_dist(&nump,&pairs_jntPt,&conftype)) {
    N->weight = bio_measure_distance_between_atom_pairs(nump,pairs_jntPt,N->q);
    printf("Pairs distance for node %d = %f  ",N->num,N->weight);
    if(conftype == 1) {
      if(N->weight > p3d_get_best_weight()) {
	p3d_set_best_weight(N->weight);
	printf("- increased pairs distance\n");
	p3d_set_n_expan_without_w_improve(0);
      }
      else {
	p3d_inc_n_expan_without_w_improve();
	printf("\r");
      }
    }
    else{
      if(N->weight < p3d_get_best_weight()) {
	p3d_set_best_weight(N->weight);
	printf("- decreased pairs distance\n");
	p3d_set_n_expan_without_w_improve(0);
      }
      else {
	p3d_inc_n_expan_without_w_improve();
	printf("\r");
      }
    }
  }
  else{
    //    if(1) {
#ifdef WITH_XFORMS
    if((p3d_col_get_mode() == p3d_col_mode_bio) ||
       (p3d_get_user_drawnjnt() != -1)) {
      N->weight = ffo_dist_from_root_pos(N->q);
      printf("Ref.frame distance from root for node %d = %f  ",N->num,N->weight);
      if(N->weight > p3d_get_best_weight()) {
	p3d_set_best_weight(N->weight);
	printf("- increased distance\n");
	p3d_set_n_expan_without_w_improve(0);
      }
      else {
	p3d_inc_n_expan_without_w_improve();
	printf("\r");
      }
    }
    else {
      printf("Node number = %d\r",N->num);
    }
#endif
  }

  return 1;
  #endif
}
/****************************************************************/

double ffo_dist_from_root_pos(configPt q)
{
  p3d_jnt *refjnt;
  configPt rootconf;
  double dist=0.0;
  int indexjnt=0;

  // WARNING : root pos is ROBOT_POS
  rootconf = XYZ_ROBOT->ROBOT_POS;

  //  if((XYZ_ROBOT->cntrt_manager != NULL)&&(XYZ_ROBOT->cntrt_manager->cntrts != NULL)) { // LOOP
#ifdef WITH_XFORMS
  indexjnt = p3d_get_user_drawnjnt();
#endif
  if(indexjnt != -1) {
    refjnt = XYZ_ROBOT->joints[indexjnt];
    dist = sqrt(  SQR(refjnt->abs_pos[0][3] - refjnt->pos0[0][3])
		  + SQR(refjnt->abs_pos[1][3] - refjnt->pos0[1][3])
		  + SQR(refjnt->abs_pos[2][3] - refjnt->pos0[2][3]));
  }
  else {  
    // NO LOOP (WARNING: EVEN IF THERE IS A LIGAND !!!!) 
    // WARNING : the joint we choice to mesure distance is :
    //           - the last jnt in list next_jnt of jnt0
    //           - a freeflying
    refjnt = XYZ_ROBOT->joints[0]->next_jnt[XYZ_ROBOT->joints[0]->n_next_jnt - 1];
    //if(p3d_col_get_mode() == p3d_col_mode_bio) {
    if(refjnt->type == P3D_FREEFLYER) {
      dist = sqrt(  SQR(q[refjnt->index_dof]-rootconf[refjnt->index_dof])
		    + SQR(q[refjnt->index_dof+1]-rootconf[refjnt->index_dof+1])
		    + SQR(q[refjnt->index_dof+2]-rootconf[refjnt->index_dof+2]));
    }
    else {
      dist = 0.0;
    }
  }
  return(dist);
}
