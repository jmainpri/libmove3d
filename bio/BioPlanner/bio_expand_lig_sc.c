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
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Bio-pkg.h"

#define DEBUG_EXP_LIG_SC 0

p3d_node* bio_expandnode_ligand(p3d_graph* G,p3d_node* Nnear, configPt qexp_direction, 
				int_tab** sc_ligand_col_list, int* is_ligand_autocolPt, 
		      int* is_ligand_bb_colPt);
int bio_expand_side_chain(p3d_graph*G, p3d_node* last_node, int_tab* sc_lignand_col_list); 
int bio_expand_ligand(p3d_graph*G, configPt q_lig, int_tab** sc_ligand_col_list, int*  is_ligand_autocolPt, 
		      int* is_ligand_bb_colPt);
int bio_start_lig_sc_rrt(p3d_graph *G,int (*fct_stop)(void));


int bio_start_lig_sc_rrt(p3d_graph *G,int (*fct_stop)(void)) {
  int is_ligand_autocol = 0;
  int is_ligand_bb_col = 0;
  int Added = FALSE, RLG_state;
  int Stop = FALSE;
  int nnodemax = p3d_get_COMP_NODES();
  int_tab *sc_ligand_col_list = NULL;
  int_tab* sc_autocol_list =NULL,* scsc_col_list = NULL, * sc_bb_col_list = NULL;
  configPt q_lig;

  if(is_ligand_in_robot(G->rob)== FALSE) {
    //if(DEBUG_EXP_LIG_SC) 
      PrintInfo(("Planification impossible: there is no ligand detected\n"));
    return FALSE;
  }

  while(!Stop) {
   if (fct_stop) {
     if (!(*fct_stop)()) {
       if(DEBUG_EXP_LIG_SC) 
	 PrintInfo(("RRT building canceled\n"));
       return FALSE;
     }
   }
   Added = FALSE;
   Stop = TRUE;
   if (G->search_start->comp->nnode < nnodemax) {

     q_lig = p3d_alloc_config(G->rob);
     //RLG constraints are inactive for the shoot because
     // q gives only a direction of expansion
     RLG_state = p3d_get_RLG();
     p3d_set_RLG(0);


     if(p3d_GetExpansionDirectionMethod() == SUBREGION_CS_EXP) {
       bio_ligand_inbox_shoot(G->rob,G->search_start->comp->box_env_small, q_lig);
     }
     else {
       bio_ligand_shoot(G->rob,q_lig);
     }
     p3d_set_RLG(RLG_state);

     Added = bio_expand_ligand(G, q_lig, &sc_ligand_col_list, &is_ligand_autocol, &is_ligand_bb_col );
     Stop = Added;
   }
 }
 if(Added ==TRUE) {
   //PrintInfo((" nnodes : %d , w = %f\n",G->search_start->comp->nnode,G->last_node->N->weight));

   if(is_ligand_autocol == TRUE) {
     if(DEBUG_EXP_LIG_SC) 
       PrintInfo(("ligand autocollision, nothing to do\n"));
   }
   else if (is_ligand_bb_col == TRUE ){
     if(DEBUG_EXP_LIG_SC) 
       PrintInfo(("ligand backkbone collison, nothing to do\n"));
   }
   else if(sc_ligand_col_list ==NULL) {
     if(DEBUG_EXP_LIG_SC) 
       PrintInfo(("no collision with side chains, nothing to do\n"));
   }
   else {
     if(DEBUG_EXP_LIG_SC) 
       PrintInfo(("collision ligand  side chains !!\n"));
     bio_expand_side_chain(G, G->last_node->N, sc_ligand_col_list); 
     }
   }
 bio_free_multimol_collision_report(& sc_autocol_list, &scsc_col_list , &sc_bb_col_list ,&sc_ligand_col_list);
 return Added;
}


 
int bio_expand_side_chain(p3d_graph*G, p3d_node* last_node, int_tab* sc_lignand_col_list) {
  int i, numAA = -1;
  Joint_tablespt *jnt_table = NULL;
  int no_collision = TRUE;
  p3d_node* NewNode;
  configPt q_new;
  int max_try = 10; //max try to find a valid sc..
  int no_col_sidechain_i= FALSE;
  int ntry;
  double dist;

  if(sc_lignand_col_list == NULL) {
    if(DEBUG_EXP_LIG_SC) 
      PrintInfo(("Error: no side chain in collision stored\n"));
    return FALSE;
  }

  q_new = p3d_alloc_config(G->rob);
  if(!bio_get_current_q_inv(G->rob, q_new)) {      
    if(DEBUG_EXP_LIG_SC) 
      PrintInfo(("Error: no q_inv stored\n"));
    return FALSE;
  }
  // OLD (LEO VERSION) :
  //q_new = p3d_copy_config(G->rob, last_node->q);

  for( i = 0; i<sc_lignand_col_list->size ;i++) {
    numAA =sc_lignand_col_list->tab[i]; 
    jnt_table =  give_joint_tables(num_subrobot_AA_from_AAnumber(G->rob,numAA));
    no_col_sidechain_i = FALSE;
    ntry = 0;
    while ( (ntry< max_try) &&( no_col_sidechain_i == FALSE) ) {
      no_col_sidechain_i= ((no_col_sidechain_i) || 
			   //bio_generate_one_sch_conf_and_checkcoll(G->rob, numAA, jnt_table,q_new));
			   bio_perturb_one_sch_conf_and_checkcoll(G->rob, numAA, jnt_table,q_new,((double)ntry+1.0)/(double)max_try));
      ntry++;
    }
    no_collision = (no_collision) && ( no_col_sidechain_i);
  }
  if( no_collision == TRUE) {
    if(DEBUG_EXP_LIG_SC) 
      PrintInfo(("Phase 1 succeded : valid side chains found\n"));
    NewNode=  p3d_APInode_make (G, q_new);
    if(NewNode != NULL) {
      p3d_insert_node(G,NewNode);
      p3d_create_compco(G,NewNode);
      if(p3d_APInode_linked(G,NewNode,last_node,&dist))
	{
	  if(DEBUG_EXP_LIG_SC) 
	  PrintInfo(("Phase 2 succeed: New node connected \n"));
	  p3d_create_edges(G,last_node,NewNode,dist);
	  //  check_if_node_connects_connected_components(G, NewNode);
	  p3d_merge_comp(G, last_node->comp, &(NewNode->comp));
	}
      return TRUE;
    }
    else {
      PrintInfo(("Error : failed to create a new node\n"));
    }
  }
  else {
    if(DEBUG_EXP_LIG_SC) 
      PrintInfo(("Phase 1 failed : valid side chains not found\n"));
  }
  p3d_destroy_config(G->rob,q_new);
  return FALSE;
}

/****************************************************************/
/*!
* \param G: the graph
* \retval sc_ligand_col_list: the list of side chains in collision with the 
*         ligand when the 1st collision occurs along the
          direction of expansion.
* \return -1 if no node has been inserted, 0 if a node has been
*         inserted but a collision occured, 1 if the graph has 
*         been extended until qexp_direction.
*/
/****************************************************************/
int bio_expand_ligand(p3d_graph* G,   configPt q_lig, int_tab** sc_ligand_col_listPt, 
		      int* is_ligand_autocolPt, 
		      int* is_ligand_bb_colPt) {
  int Added = FALSE, result_test; 
  p3d_rob *robotPt = G->rob;
  p3d_node *NewNode, *Nnear;
  configPt qexp_direction;
  double dist = 0.; 
  int no_nearest = TRUE;
  int Nnearloop = 0;
  int   maxNnearloop = p3d_get_nbtry_DD(); 
  int RLG_state;

  if(p3d_GetExpansionDirectionMethod() == SUBREGION_CS_EXP) {
    while(no_nearest == TRUE) {

      Nnear = bio_ligand_nearest_neighbor(robotPt, q_lig, G->search_start->comp, &dist);
      if(Nnear ==NULL) {
	PrintInfo(("Error: failed to find a nearest node\n"));
	//We return 1 to make the planning process stop and to avoid infinite loop.
	return 1;
      }
      if( (Nnear->boundary==FALSE)|| (dist < Nnear->radius)
	  || (maxNnearloop == Nnearloop) ) {
	no_nearest = FALSE;
	//	if(maxNnearloop ==  Nnearloop) {
	//  PrintInfo(("qlig is not in the Dynamic Domain \n"));
	//	}
	//	else {
	//  PrintInfo(("qlig IS in the Dynamic Domain \n"));
	//	}
      }
      else {
	RLG_state = p3d_get_RLG();
	p3d_set_RLG(0);
	bio_ligand_inbox_shoot(G->rob,G->search_start->comp->box_env_small, q_lig);    
	p3d_set_RLG(RLG_state);
	Nnearloop++;
      }
    }  
  } else {
    Nnear = bio_ligand_nearest_neighbor(robotPt, q_lig,G->search_start->comp , &dist);
  }


  if(Nnear ==NULL) {
    PrintInfo(("Error: failed to find a nearest node\n"));
    //We return 1 to make the planning process stop and to avoid infinite loop.
    p3d_destroy_config(G->rob, q_lig);
    return 1;
  }
  //  necessary to check the validity along a local path
  p3d_SetCurrentNearNode(Nnear);

  qexp_direction = p3d_copy_config(robotPt, Nnear->q);
  result_test = bio_copy_ligand_conf(robotPt, q_lig, qexp_direction);
  G->nb_q = G->nb_q + 1; 
  if(result_test == 0) {
    PrintInfo(("Error: failed to copy ligand dofs\n"));
    //We return 1 to make the planning process stop and to avoid infinite loop.
    p3d_destroy_config(G->rob, q_lig);
    p3d_destroy_config(robotPt, qexp_direction);
    return 1;
  }
  NewNode = bio_expandnode_ligand(G,Nnear,qexp_direction, sc_ligand_col_listPt, is_ligand_autocolPt, is_ligand_bb_colPt);
  if(NewNode!= NULL) {
    if(!p3d_GetIsWeightStopCondition()) {
      //    check_if_node_connects_connected_components(G, NewNode);//to change
      if(p3d_APInode_linked(G,NewNode,G->search_goal,&dist)) {
	if(DEBUG_EXP_LIG_SC) 
	  PrintInfo(("expand lig_sc linked\n"));
	p3d_create_edges(G,NewNode,G->search_goal,dist);
	//  check_if_node_connects_connected_components(G, NewNode);
	p3d_merge_comp(G,NewNode->comp , &(G->search_goal->comp));
      }
    }
    if(p3d_GetExpansionDirectionMethod() == SUBREGION_CS_EXP) {
      //  p3d_resize_rrt_box(robotPt, G->search_goal->comp->box_env_small, NewNode, 1);
      p3d_ResizeDynDomain(robotPt, NewNode);

    }
    
    Added =TRUE;
    Nnear->n_fail_extend = 0;
    
  }
  else {
    Nnear->n_fail_extend++;
    Nnear->boundary = TRUE;
    G->nboundary++;
  }
  p3d_destroy_config(robotPt, qexp_direction);
  p3d_destroy_config(G->rob, q_lig);
  return Added;
}

p3d_node* bio_expandnode_ligand(p3d_graph* G,p3d_node* Nnear,
				configPt qexp_direction, int_tab** sc_ligand_col_listPt , int* is_ligand_autocolPt, int *is_ligand_bb_colPt ) {
  p3d_rob* robotPt = G->rob;
  p3d_localpath *path = NULL;
  configPt qnew, qinvalid = NULL;
  double Kpath = 0, dist = 0;
  int **iksol = NULL;  
  p3d_node* NewNode;
  int_tab* sc_autocol_list = NULL, *scsc_col_list = NULL;
  int_tab *sc_bb_col_list = NULL; // *sc_ligand_col_list = NULL;
  int  is_bb_autocol;
  double stop_weight;
  int sign_stop_weight;

  if (p3d_equal_config(robotPt, qexp_direction, Nnear->q)) {
    if((robotPt->cntrt_manager == NULL)||(robotPt->cntrt_manager->cntrts == NULL)) /* no display if constraints */ 
      PrintInfo(("qnew invalide\n"));
    (*is_ligand_autocolPt) = 0;
    (*sc_ligand_col_listPt) = NULL;
    return NULL;
  }
  (G->nb_q_closed)++;
  if (!(path = p3d_local_planner(robotPt, Nnear->q, qexp_direction))) {
    PrintInfo(("Planification impossible: problem with local path creation\n"));
    (*is_ligand_autocolPt) = 0;
    (*sc_ligand_col_listPt) = NULL;
    return NULL;
  }
  (G->nb_local_call)++;
  qnew = p3d_alloc_config(robotPt);
  p3d_unvalid_localpath_classic_test(robotPt, path, &(G->nb_test_coll), &Kpath, &qnew);
  if (Kpath==0) {
    path->destroy(robotPt,path);
    p3d_destroy_config(robotPt,qnew);
    (*is_ligand_autocolPt) = 0;
    (*sc_ligand_col_listPt)  = NULL;
    return NULL;
  }

  if(Kpath != 1.) {//test de double pouvant �tre transform� en int
    qinvalid = p3d_alloc_config(robotPt);
    if(bio_get_current_q_inv(robotPt, qinvalid)) {      
      //traitement de qinvalid pour recup sc_list 
      // et is_bb_col � base de  bio_deform_schs_avoiding_collision
      // NOTE : next fuction updates the robot configuration
      bio_multimol_collision_report(robotPt, qinvalid, &sc_autocol_list,
				    &scsc_col_list , &sc_bb_col_list, sc_ligand_col_listPt,
				    is_ligand_bb_colPt, is_ligand_autocolPt, & is_bb_autocol);
    }
    p3d_destroy_config(robotPt, qinvalid);
  }

  dist = Kpath * path->length_lp;
  path->destroy(robotPt,path);
  p3d_set_and_update_this_robot_conf_without_cntrt(robotPt,qnew);
  //we get the index of the solution generated by the cntrt
  p3d_get_iksol_vector(robotPt->cntrt_manager,&iksol);
  // WARNING: iksol does not work, a new vector containing the 1st solution of each constraint must be passed as argument
  NewNode = p3d_APInode_make_multisol(G, qnew, NULL);
  if(NewNode == NULL) {
    PrintInfo(("Error in the process of new node creation\n"));
  }
  NewNode->type = LINKING; 
  p3d_SetNodeWeight(G, NewNode);
  p3d_GetStopWeightAndSign(&stop_weight,&sign_stop_weight);  
  if(sign_stop_weight == 1) {
    if(NewNode->weight >= stop_weight) {
      p3d_SetStopValue(TRUE);
      p3d_SetDiffuStoppedByWeight(1);
    }
  }
  else if(sign_stop_weight == -1) {
    if(NewNode->weight <= stop_weight) {
      p3d_SetStopValue(TRUE);
      p3d_SetDiffuStoppedByWeight(1);
    }
  } 

  p3d_insert_node(G, NewNode);  
  p3d_create_edges(G,Nnear,NewNode,dist);
  p3d_add_node_compco(NewNode, Nnear->comp, TRUE);
  return NewNode;
}
