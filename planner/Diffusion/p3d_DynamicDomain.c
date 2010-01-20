#include "Planner-pkg.h"
#include "P3d-pkg.h"
#include "Collision-pkg.h"
#include "Bio-pkg.h"

/*
 * The lambda parameter is used in two cases: 
 * it defines the size of the  dynamic boxes inside 
 * which configurations are sampled and define the radii 
 * of the nodes if we set a maximal distance in the selection
 * of the node to expand.  
 */
static double LAMBDA = 5.;

/**
 * p3d_InitSpaceCostParam
 * @param[In] GraphPt: the robot graph 
 * Initialize some parameters for dealing 
 * with dynamic domain methods
 * @param[In] Ns: Initial node of the graph 
 * @param[In] Ng: Goal node of the graph 
 */
void p3d_InitDynDomainParam(p3d_graph* GraphPt, p3d_node* Ns, p3d_node* Ng) {
  double dmax = p3d_get_env_dmax();
 
  if(Ns != NULL) {
    Ns->radius= p3d_GetLambda()*dmax;
    Ns->boundary = FALSE;  
    if(p3d_GetExpansionDirectionMethod() == SUBREGION_CS_EXP) {
      p3d_ResizeDynDomain(GraphPt->rob, Ns);
    }
  }
  if((ENV.getBool(Env::expandToGoal)== true) && (Ng != NULL)) {
    Ng->radius= p3d_GetLambda()*dmax;
    Ng->boundary = FALSE;
    if(p3d_GetExpansionDirectionMethod() == SUBREGION_CS_EXP) {
      p3d_ResizeDynDomain(GraphPt->rob, Ns);
    }
  }
}

/**
 * p3d_GetLambda(double L)
 * Get the value of the lambda parameter used to
 * define the size of the dynamic sampling domain
 * @return: the value of the lambda parameter 
 */
double p3d_GetLambda(void)
{
  return(LAMBDA);
}

/**
 * p3d_SetLambda(double L)
 * Set the value of the lambda parameter used to
 * define the size of the dynamic sampling domain
 * @param[In]: L:  the Lambda value
 */
void p3d_SetLambda(double Lambda)
{
  LAMBDA = Lambda;
}

/**
 * p3d_resize_rrt_box_ORIGINAL
 * Old version of p3d_ResizeDynDomain
 * No more used
 */
int p3d_resize_rrt_box_ORIGINAL(p3d_rob* robotPt, configPt box_env[],
				p3d_node* NewNode, int koef) {
  int njnt = robotPt->njoints,i, j, k;
  p3d_jnt * jntPt;
  int change = FALSE;
  double dmax =  p3d_get_env_dmax();
  double alpha = 1;
  double weight = 0.0;
  double inf, sup;

  for(i=0; i<=njnt; i++) {
    jntPt = robotPt->joints[i];
    for(j=0; j<jntPt->dof_equiv_nbr; j++) {
      k = jntPt->index_dof + j;
      if (p3d_jnt_get_dof_is_user(jntPt, j)) {
	if (p3d_jnt_is_dof_angular(jntPt, j)) {
	  weight = koef*p3d_GetLambda()*dmax/(jntPt->dist*alpha);
	  //weight = koef*(NewNode->radius)*2./(3.*(jntPt->dist)*alpha);
	  /* seting of the min value */
	  inf = NewNode->q[k]-weight;	   
	  sup = NewNode->q[k]+weight;	   
	  if (box_env[0][k]<= box_env[1][k]){
	    /*for box_env[0][k] */
	    if(inf<box_env[1][k] -2*M_PI) {
	      box_env[0][k] = -M_PI;
	      box_env[1][k] = M_PI;
	    }
	    else if (inf<-M_PI) {
	      box_env[0][k] = inf+2*M_PI;
	    }
	    else if (inf<box_env[0][k]) {
	      box_env[0][k] = inf;
	    }
	  }
	  else{
	    if((NewNode->q[k]<box_env[1][k])&&(NewNode->q[k]>-M_PI))
	      {/*2nd condition should not be needed*/
		if(inf< box_env[1][k]-2*M_PI) {
		  box_env[0][k] = -M_PI;
		  box_env[1][k] = M_PI;
		}
		else if (inf<box_env[0][k]-2*M_PI) {
		  box_env[0][k] = inf+2*M_PI;
		}
	      }
	    else
	      {
		if(inf< box_env[1][k]) {
		  box_env[0][k] = -M_PI;
		  box_env[1][k] = M_PI;
		}
		else if (inf<box_env[0][k]) {
		  box_env[0][k] = inf;
		}
	      }
	  }
	  if (box_env[0][k]<= box_env[1][k]){
	    /*for box_env[1][k] */
	    if(sup>box_env[0][k] +2*M_PI) {
	      box_env[0][k] = -M_PI;
	      box_env[1][k] = M_PI;
	    }
	    else if (sup>M_PI) {
	      box_env[1][k] = sup-2*M_PI;
	    }
	    else if (sup>box_env[1][k]) {
	      box_env[1][k] = sup;
	    }
	  }
	  else {
	    if((NewNode->q[k]>box_env[0][k])&&(NewNode->q[k]<M_PI)){
	      if(sup> box_env[0][k]+2*M_PI) {
		box_env[0][k] = -M_PI;
		box_env[1][k] = M_PI;
	      }
	      else if (sup>box_env[1][k]+2*M_PI) {
		box_env[1][k] = sup-2*M_PI;
	      }
	    }
	    else 
	      {
		if(sup< box_env[0][k]) {
		  box_env[0][k] = -M_PI;
		  box_env[1][k] = M_PI;
		}
		else if (sup>box_env[1][k]) {
		  box_env[1][k] = sup;
		}
	      }
	  }
	}  
	
	else {
	  if (NewNode->q[k]- koef*p3d_GetLambda()*dmax <  box_env[0][k] )
	    //if (NewNode->q[k]- koef*(NewNode->radius)*2./3. <  box_env[0][k] )
	    
	    {
	      
	      box_env[0][k] = NewNode->q[k]- koef*p3d_GetLambda()*dmax  ;
	      //box_env[0][k] = NewNode->q[k]- koef*(NewNode->radius)*2./3. ;
	      change = TRUE;
	    }
	  if (NewNode->q[k] + koef*p3d_GetLambda()*dmax >  box_env[1][k]) {
	    //if (NewNode->q[k] + koef*(NewNode->radius)*2./3. >  box_env[1][k]) {
	    box_env[1][k] = NewNode->q[k] + koef*p3d_GetLambda()*dmax ;
	    ///box_env[1][k] = NewNode->q[k] + koef*(NewNode->radius)*2./3.;
	      change = TRUE;
	  }
	}
      }
    }
 }
 return change;
}

/**
 * p3d_ResizeDynDomain
 * Resize the Dynamic Domain of a componant when a new 
 * node is added. The Dynamic Domain is then used as a restricted 
 * region of sampling to balance the greediness of the expansion 
 * process with more refinement of the tree.
 * The field box_env_small of the componected componant of the node
 *  is modified.
 * @param[In]: robotPt: A pointer to the robot
 * @param[In]: NewNodePt : Pointer for the New added node
 * return: TRUE if the process of resizing succeeded
 * Note: the node should be already inserted in its connected componant
 */
int p3d_ResizeDynDomain(p3d_rob* robotPt, p3d_node* NewNodePt) {
  p3d_compco* NodeCompPt;
  double dmax;

  int njnt = robotPt->njoints,i, j, k;
  p3d_jnt * jntPt;
  double NodeBoxMin_k, NodeBoxMax_k;
  double Vmin, Vmax;
  double WeightRota = p3d_GetWeightRotations();
  double DeltaK = -1.;

  if((NewNodePt == NULL)|| (NewNodePt->comp == NULL)) {
    PrintInfo(("Warning: try to resize dynamic domain with NULL node\
 or node without componant"));
    return FALSE;
  }
  /* Initialisation if no node has been created until now */
  NodeCompPt = NewNodePt->comp;
  if(NodeCompPt->box_env_small[0] == NULL) {
    NodeCompPt->box_env_small[0] = p3d_copy_config(robotPt, NewNodePt->q);
    NodeCompPt->box_env_small[1] = p3d_copy_config(robotPt, NewNodePt->q);
  }

  /* Get dmax value */
  if(p3d_col_get_mode() == p3d_col_mode_bio) {
    bio_col_get_step_deplacement(&dmax);
  }
  else {
    dmax =  p3d_get_env_dmax();
  }

  for(i=0; i<=njnt; i++) {
    jntPt = robotPt->joints[i];
    for(j=0; j<jntPt->dof_equiv_nbr; j++) {
      k = jntPt->index_dof + j;
      p3d_jnt_get_dof_rand_bounds(jntPt, j, &Vmin, &Vmax);
      if (p3d_jnt_get_dof_is_user(jntPt, j)) {
	if (p3d_jnt_is_dof_angular(jntPt, j)) {
	  DeltaK = p3d_GetLambda()*dmax/(WeightRota*jntPt->dist);
	} else {
	  DeltaK = p3d_GetLambda()*dmax;
	}
	NodeBoxMin_k = NewNodePt->q[k] - DeltaK;
	NodeBoxMax_k = NewNodePt->q[k] + DeltaK;
	if(NodeBoxMin_k < NodeCompPt->box_env_small[0][k]) {
	  NodeCompPt->box_env_small[0][k] = MAX(NodeBoxMin_k, Vmin);
	}
	if(NodeBoxMax_k > NodeCompPt->box_env_small[1][k]) {
	  NodeCompPt->box_env_small[1][k] = MIN(NodeBoxMax_k, Vmax);
	}
      }
    }
  }
  return TRUE;
}

