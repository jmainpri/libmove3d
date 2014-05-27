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
#include "Planner-pkg.h"
#include "Bio-pkg.h"
#include "Collision-pkg.h"

#define DEBUG 0

/** 
 * IS_MANHATAN_EXPANSION
 * This Flag is TRUE if the expansion
 * is a Manhattan like expansion:
 * In a first step only the active parameters are expanded
 * then we try to expand the passive parameter by recursivly
 * expanding  only the passive parameters which were in collision
 * during the previsous expansion
 */
int IS_MANHATTAN_EXPANSION = FALSE;

/** 
 * PASS_EXT_ONLY_WHEN_ACTIVE_EXTENSION
 * This flag is TRUE if we allow the expansion
 * of the passive parameters only when the expension of
 * the active ones succeeded
*/
int PASS_EXT_WHEN_ACT_FAILED = FALSE;


/**
 * p3d_ExpanBlockedByColl
 * Todo
 */
int p3d_ExpanBlockedByColl(p3d_rob *robotPt, configPt *qinvPt) {
  // with BIO module
  if(p3d_col_get_mode() == p3d_col_mode_bio) {
#ifdef BIO
    if(!bio_get_current_q_inv(robotPt,*qinvPt)) {      
      return 0;
    }
#endif
    return 1;
  }
  // without BIO module
  if(!p3d_get_current_q_inv(robotPt,*qinvPt)) {      
    return 0;
  }
  return 1;
}

/**
* p3d_perturb_and_check_passive_params_involved_in_collision
* Todo. function no more used
*/
int p3d_perturb_and_check_passive_params_involved_in_collision(p3d_rob *robotPt, 
							       configPt qinv)
{
  int processOK = FALSE;

  // with BIO module
  if(p3d_col_get_mode() == p3d_col_mode_bio) {
    // next function possibly changes q_inv
#ifdef BIO
    processOK = bio_perturb_and_check_passive_params_involved_in_collision(robotPt,
									   qinv);
#endif
  }
  // without BIO module
  else {
    printf("WARNING: p3d_perturb_and_check_passive_params_involved_in_collision\
 : only works with BIO module\n");
  }

  return(processOK);
}

/**
 * p3d_SelectNewJntInList
 * todo
 */
// NOTE : this function allocates memory that must be freed later
int p3d_SelectNewJntInList(p3d_rob *robotPt, int nJ, p3d_jnt **Jlist,
					 int *old_nJPt, p3d_jnt ***old_JlistPt, 
					 int *new_nJPt, p3d_jnt ***new_JlistPt)
{
  int i,j;
  int there_are_new_jnts = FALSE;
  int in_list;
  
  if(*old_nJPt == 0) {
    *old_nJPt = nJ;
    *new_nJPt = nJ;
    *old_JlistPt = MY_ALLOC(p3d_jnt *,nJ);
    *new_JlistPt = MY_ALLOC(p3d_jnt *,nJ);
    for(i=0; i<nJ; i++) {
      (*old_JlistPt)[i] = Jlist[i];
      (*new_JlistPt)[i] = Jlist[i];
    }  
    there_are_new_jnts = TRUE;
  }
  else{
    for(i=0; i<nJ; i++) {
      in_list = FALSE;
      for(j=0; (j<(*old_nJPt)) && !in_list; j++) {
	if(Jlist[i] == (*old_JlistPt)[j])
	  in_list = TRUE;
      }
      if(!in_list) {
	if(*new_nJPt == 0) {
	  *new_JlistPt = MY_ALLOC(p3d_jnt *,1);
	}
	else {
	  *new_JlistPt = MY_REALLOC((*new_JlistPt),p3d_jnt *,
				    (*new_nJPt),(*new_nJPt)+1);
	}
	(*new_JlistPt)[*new_nJPt] = Jlist[i];
	(*new_nJPt)++;
      }
    }
    if(*new_nJPt != 0) {
      there_are_new_jnts = TRUE;
      // update old_list
      *old_JlistPt = MY_REALLOC((*old_JlistPt),p3d_jnt *,(*old_nJPt),
				(*old_nJPt)+(*new_nJPt));
      for(i=0; i<(*new_nJPt); i++) {
	(*old_JlistPt)[(*old_nJPt)+i] = (*new_JlistPt)[i];      
      }
      (*old_nJPt) += (*new_nJPt);
    }
  }

  return(there_are_new_jnts);
}

/**
 * p3d_GetCollidingtPassiveJntList
 * Todo
 * NOTE : this function allocates memory that must be freed later
 */
int p3d_GetCollidingtPassiveJntList(p3d_rob *robotPt, configPt qinv, 
				    int *npassJPt, p3d_jnt ***passJlistPt)
{
  int pass_jnts_in_coll = FALSE;
  p3d_poly *p1,*p2;
  //configPt qsaved;

  // with BIO module
  if(p3d_col_get_mode() == p3d_col_mode_bio) {
#ifdef BIO
    pass_jnts_in_coll = bio_get_list_of_passive_joints_involved_in_collision(robotPt,
									     qinv,
									     npassJPt,
									     passJlistPt);
#endif
  }
  // without BIO module
  else {
    //printf("WARNING: p3d_GetCollidingtPassiveJntList: only works with BIO module\n");

    //qsaved = p3d_get_robot_config(robotPt);
    p3d_set_and_update_this_robot_conf_without_cntrt(robotPt,qinv);
    if(p3d_col_test() <= 0) {
      PrintInfo(("No collision detected \n \n"));
      return FALSE;
    }
    
    // NOTE: KCD only retuns the first collision pair !!!
    // NOTE: ONLY THE PASSIVE JOINT INVOLVED IN THE COLLISION IS RETURNED
    //       BUT ALL THE PARENT JOINTS SHOULD BE ALSO CONSIDERED ???
    p3d_col_get_report(0,&p1,&p2);
    if(p1->p3d_objPt->jnt != NULL) {
      if(!p3d_jnt_get_is_active_for_planner(p1->p3d_objPt->jnt)) {
	*passJlistPt = MY_ALLOC(p3d_jnt *,1);
	(*passJlistPt)[0] = p1->p3d_objPt->jnt;
	*npassJPt = 1;
	pass_jnts_in_coll = TRUE;	  	
      }
    }
    if(p2->p3d_objPt->jnt != NULL) {
      if(!p3d_jnt_get_is_active_for_planner(p2->p3d_objPt->jnt)) {
	if(pass_jnts_in_coll == FALSE) {
	  *passJlistPt = MY_ALLOC(p3d_jnt *,1);
	  (*passJlistPt)[0] = p2->p3d_objPt->jnt;
	  *npassJPt = 1;
	  pass_jnts_in_coll = TRUE;
	}
	else {
	  *passJlistPt = MY_REALLOC((*passJlistPt),p3d_jnt *,(*npassJPt),
				    (*npassJPt)+1);
	  (*passJlistPt)[*npassJPt] = p2->p3d_objPt->jnt;
	  (*npassJPt)++;
	}
      }
    }    
  }

  return(pass_jnts_in_coll);
}


/**
 * p3d_shoot_jnt_list_and_copy_into_conf
 * Todo. If general function, should probably be
 * moved to p3d_sample.c
 */
void p3d_shoot_jnt_list_and_copy_into_conf(p3d_rob *robotPt, configPt qref, configPt qrand, 
						  int npassJ, p3d_jnt **passJlist)
{
  int i,j,k;
  p3d_jnt *jntPt;
  double vmin,vmax;
  double midrange,val,rval;
  double perturb = 0.1; // NOTE: THIS SHOULD BE A PARAMETER

  // NOTE : the random shoot should be centered at q_inv !!!
  //        (doesn't matter in the case of "circular" joints)
  
  for(i=0; i<npassJ; i++) {
    jntPt = passJlist[i];
    for(j=0; j<jntPt->dof_equiv_nbr; j++) {
      p3d_jnt_get_dof_rand_bounds(jntPt, j, &vmin, &vmax);
      k = jntPt->index_dof + j;
      if(!p3d_jnt_is_dof_circular(jntPt,j)) {
	val = p3d_random(vmin,vmax);
      }
      else {
	midrange = (vmax-vmin)/2.0;
	// perturbation factor 
	midrange *= perturb;
	rval = p3d_random(-midrange,midrange);
	val = qrand[k] + rval;
	if(val > vmax)
	  val = vmax;
	else if(val < vmin)
	  val = vmin;
      }
      qrand[k] = val;
    }
  }
}

/**
 * p3d_free_list_of_joints
 * Todo. If general function, should  be
 * moved to
 */
void p3d_free_list_of_joints(p3d_rob *robotPt, int *nJ, p3d_jnt ***Jlist)
{
  if((*nJ) != 0) 
    MY_FREE((*Jlist),p3d_jnt *,*nJ);  
  *Jlist = NULL;
  *nJ = 0;
}

/**
 * p3d_copy_passive_config_into
 * Todo
 */
void p3d_copy_passive_config_into(p3d_rob *robotPt, configPt qsrc, configPt qdst)
{
  int njnt = robotPt->njoints, i, j, k; 
  p3d_jnt * jntPt;
  
  for(i=0; i<=njnt; i++) {
    jntPt = robotPt->joints[i];
    for(j=0; j<jntPt->dof_equiv_nbr; j++) {
      k = jntPt->index_dof + j;
      if ( ! p3d_jnt_get_dof_is_active_for_planner(jntPt,j)) {
	qdst[k] = qsrc[k];
      }
    }
  }  
}


/**
 * p3d_PassivExpandProcess
 * Expand the passivee configuration parameters after
 * an expansion of active parameters
 * @param[In] GraphPt: Pointer to the robot graph
 * @param[In] ExpansionNodePt: Node from which the active params have been extended.
 * @param[In] NbActiveNodesCreated: Number of nodes created during 
 * the expansion of the active parameters
 * @return: The number of nodes created during the expansion of passive dofs. 
 * Note:  the random directions choosen to expand the passive 
 * dofs are independant of the ExpansionDirectionMethod. It directly uses
 * the p3d_shoot_jnt_list_and_copy_into_conf function.
 */ 
int p3d_PassivExpandProcess(p3d_graph* GraphPt, p3d_node*  ExpansionNodePt, 
			    int NbActiveNodesCreated) {
  configPt InvalConf = NULL;
  configPt NewRandConf = NULL;
  int Reiterate = TRUE;
  //  p3d_node* NewNodePt = NULL;
  p3d_jnt** PasJntListPt = NULL, ** OldPasJntListPt = NULL, **NewPasJntListPt = NULL;
  int NbPasJnt = 0, OldNbPasJnt = 0, NewNbPasJnt = 0;
  p3d_node* LastCreatedNodePt;
  int i,  IsPassExpanded = FALSE;
  int NbNodeCreated;
  int NbPasExp = 0;
  //  p3d_node*  lastGNode = GraphPt->last_node->N;
  int firstPass = TRUE;

  if(( ENV.getBool(Env::isPasExtWhenAct) == false)  && (NbActiveNodesCreated == 0)){
    /* The passive dof expansion only works if the 
       active dofs have been expanded */
    return 0;
  }
  InvalConf = p3d_alloc_config(GraphPt->rob);
  NewRandConf = p3d_alloc_config(GraphPt->rob);
  /*Warning: I don't anderstand the function of the Reiterate parameter */
  while ((Reiterate == TRUE) && 
	 (p3d_ExpanBlockedByColl(GraphPt->rob, &InvalConf))) {
    //NewNodePt = NULL;
    NbNodeCreated =0;
    IsPassExpanded = FALSE;
    if(p3d_GetCollidingtPassiveJntList(GraphPt->rob, InvalConf, &NbPasJnt, &PasJntListPt)) {
   // select only the passive parameters that have not been expanded yet
      if(p3d_SelectNewJntInList(GraphPt->rob,NbPasJnt,PasJntListPt, &OldNbPasJnt, 
				&OldPasJntListPt, &NewNbPasJnt, &NewPasJntListPt)) {   
	//	PrintInfo(("OldNbPasJnt: %d, NewNbPasJnt:%d \n", OldNbPasJnt, NewNbPasJnt));
	if((NbActiveNodesCreated== 0)&& (firstPass ==TRUE)){
	  /* No node has been created during the active node expansion */
	  LastCreatedNodePt = ExpansionNodePt;
	} else {
	  LastCreatedNodePt = GraphPt->last_node->N;
	}
	p3d_copy_config_into(GraphPt->rob,LastCreatedNodePt->q,&NewRandConf);
	i = 0;
	while(i < ENV.getInt(Env::MaxPassiveExpand) && !IsPassExpanded) {
	  i++;
	  // NOTE: next function modifies NewRandConf
	  p3d_shoot_jnt_list_and_copy_into_conf(GraphPt->rob, InvalConf, 
						NewRandConf,NewNbPasJnt,NewPasJntListPt);
	  NbNodeCreated = ExpandProcess(GraphPt, LastCreatedNodePt, NewRandConf);
	  if(NbNodeCreated !=0) {
	    IsPassExpanded = TRUE;
	    NbPasExp += NbNodeCreated;
	    if(DEBUG)
	      printf("Expanded passive parameters at try %d\n",i+1);
	  }
	}
	p3d_free_list_of_joints(GraphPt->rob,&NbPasJnt,&PasJntListPt);
	p3d_free_list_of_joints(GraphPt->rob,&NewNbPasJnt,&NewPasJntListPt);
      }
    }/*  else { */
/*       NbNodeCreated = 0; */
/*     } */
    if(NbNodeCreated == 0)
      Reiterate = FALSE;
  } 
  p3d_destroy_config(GraphPt->rob, InvalConf);    
  p3d_destroy_config(GraphPt->rob, NewRandConf);    
  p3d_free_list_of_joints(GraphPt->rob, &OldNbPasJnt,&OldPasJntListPt);

  return NbPasExp;
}
