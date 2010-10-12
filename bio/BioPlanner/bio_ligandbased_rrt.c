#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Bio-pkg.h"

void check_if_node_connects_connected_components(p3d_graph *G, p3d_node *NewNode );
int bio_start_ligandbased_rrt(p3d_graph *G,int (*fct_stop)(void));
int bio_expandcomp_ligandbased_rrt(p3d_graph *G, p3d_compco *CompPt, configPt q);
p3d_node* bio_expandnode_ligandbased_rrt(p3d_graph* G,p3d_node* Nnear, configPt qexp_direction);


/***************************************************************/
/*!
 * \brief  This method loops through the connected components of
 *         the given graph and checks whether they can be connected to the
 *         one containing NewNode through the latter.
 * \param G: The graph containing all the connected components
 * \param NewNode: All connected components will attempt to connect to this node. 
 * \note: If a connected component succeeds in connecting,
 *        the two components will be merged
 */
/***************************************************************/
void check_if_node_connects_connected_components(p3d_graph *G, p3d_node *NewNode )
{
  p3d_compco *CompConnect, *NextCompConnect, *CompPt = NewNode->comp; 
  int jcomp;
  double dist = 0.;
  p3d_node *Nnear;
  
  //If a new node was inserted into the connected component
  // Then check whether this new node does not inter-connect two connected components
  CompConnect = G->comp;
  //Loop through all the existing connected components 
  //  (actually only two, one for the start conf. and one for the end conf. )
  for(jcomp=0;(jcomp < G->ncomp)&&(CompConnect->num <= G->ncomp);jcomp++)
    {
      NextCompConnect = CompConnect->suiv;
      if (CompConnect->num!=CompPt->num)
	{
	  //Get the nearest node from this connected component.
	  // Warning: in this version we consider only the dofs of the ligand
	  Nnear = bio_ligand_nearest_neighbor(G->rob,NewNode->q,CompConnect, &dist);
	  
	  // And see if we can connect it to the new node
	  if (p3d_APInode_linked(G,NewNode,Nnear,&dist))
	    {
	      //If so, then merge the two connected components
	      PrintInfo(("RRT linking\n"));
	      if(CompPt->num < CompConnect->num) {
		p3d_merge_comp(G,CompPt,&CompConnect);
		CompConnect = CompPt;
	      }
	      else
		{
		  p3d_merge_comp(G,CompConnect,&CompPt); 
		  CompPt = CompConnect;
		}
	      p3d_create_edges(G,Nnear,NewNode,dist);
	      
	      Nnear->n_fail_extend = 0;
	    }
	  else
	    {
	      Nnear->n_fail_extend ++;
	    }
	}
      //Move on to the next connected component.
      // There may be several connected components connected by a single node
      CompConnect = NextCompConnect;
    }
}


/****************************************************************/
/*!
 * \brief expand the connected component of the start configuration
 *        with the ligand based RRT algorithm
 * 
 * \param G:  the graph
 * \param int (*fct_stop)(void): the function used as stop criterion 
 * 
 * \return FALSE if one of the stop criterions has been riched or if
 *         the method  failed to extend a new node 
 */
/****************************************************************/
int bio_start_ligandbased_rrt(p3d_graph *G,int (*fct_stop)(void)) {
  configPt q_lig;
  int Added = FALSE;
  int Stop = FALSE;
  int nnodemax = p3d_get_COMP_NODES();
  int RLG_state;

 q_lig = p3d_alloc_config(G->rob);

 if(is_ligand_in_robot(G->rob)== FALSE) {
   PrintInfo(("Planification impossible: there is no ligand detected\n"));
   return FALSE;
 }
 

 while(!Stop) {
   if (fct_stop) {
     if (!(*fct_stop)()) {
       PrintInfo(("RRT building canceled\n"));
       p3d_destroy_config(G->rob, q_lig);
       return FALSE;
     }
   }
   Added = FALSE;
   Stop = TRUE;
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
   if (G->search_start->comp->nnode < nnodemax) {
     Added = bio_expandcomp_ligandbased_rrt(G, G->search_start->comp, q_lig);
     Stop = Added;
   }
 }
 //PrintInfo((" nnodes : %d , w = %f\n",G->search_start->comp->nnode,G->last_node->N->weight));
 //PrintInfo((" nnodes : %d\n",G->search_start->comp->nnode));
 p3d_destroy_config(G->rob, q_lig);
 return Added;
}


/****************************************************************/
/*!
* \brief extend a connected component with the RRT ligand method 
*        in the direction q, taking into account only the ligand dofs.
* \param G: the graph
* \param comptPt: pointer to the connected component 
* \param q:  the configuration to connect. Only the ligand dofs are useful. 
*            The other dofs are set to 0. 
* \return TRUE if a node has been added FALSE otherwise
*/
/****************************************************************/
int bio_expandcomp_ligandbased_rrt(p3d_graph *G, p3d_compco *CompPt, configPt q_lig) {
  p3d_node *Nnear;
  p3d_rob *robotPt = G->rob;
  configPt qsaved = p3d_alloc_config(robotPt);
  configPt qexp_direction;
  int result_test;
  p3d_node *NewNode;
  int Added = FALSE;
  double dist =0.;
  int no_nearest = TRUE;
  int Nnearloop = 0;
  int   maxNnearloop = p3d_get_nbtry_DD(); 
  int RLG_state;
 
  if(p3d_GetExpansionDirectionMethod() == SUBREGION_CS_EXP) {
    while(no_nearest == TRUE) {

      Nnear = bio_ligand_nearest_neighbor(robotPt, q_lig, CompPt, &dist);
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
	//	  PrintInfo(("qlig IS in the Dynamic Domain \n"));
	//}
      }
      else {
	RLG_state = p3d_get_RLG();
	p3d_set_RLG(0);
	bio_ligand_inbox_shoot(G->rob,CompPt->box_env_small, q_lig);    
	p3d_set_RLG(RLG_state);
	Nnearloop++;
      }
    }  
  } else {
    Nnear = bio_ligand_nearest_neighbor(robotPt, q_lig, CompPt, &dist);
  }
  //  necessary to check the validity along a local path
  p3d_SetCurrentNearNode(Nnear);
  G->n_call_nearest++;
  //we initialize the direction with Nnear->q
  qexp_direction = p3d_copy_config(robotPt, Nnear->q);
  if(Nnear->is_dist_sc_ligand_checked == FALSE) {
    p3d_get_robot_pos(qsaved);
    p3d_set_robot_pos(Nnear->q);
    update_all_sidechain_BBoxes(robotPt);
    update_ligand_bounding_box(robotPt);
    // dofs of sides chains closed to Nnear->q are sampled
    result_test = bio_restrict_sc_shoot(robotPt,Nnear,qexp_direction);
    p3d_set_robot_pos(qsaved);
    p3d_destroy_config(robotPt, qsaved);
    Nnear->is_dist_sc_ligand_checked = TRUE;
  }
  else {
    result_test = bio_restrict_sc_shoot_with_NnearInfo(robotPt,
 						       Nnear,qexp_direction);
    p3d_destroy_config(robotPt, qsaved);
  }
  if(result_test == 0) {
    PrintInfo(("Error: failed to shoot side chain dofs\n"));
    //We return 1 to make the planning process stop and to avoid infinite loop.
    p3d_destroy_config(robotPt, qexp_direction);
    return 1;
  }
  result_test = bio_copy_ligand_conf(robotPt, q_lig, qexp_direction);
  G->nb_q = G->nb_q + 1; 
  if(result_test == 0) {
    PrintInfo(("Error: failed to copy ligand dofs\n"));
    //We return 1 to make the planning process stop and to avoid infinite loop.
    p3d_destroy_config(robotPt, qexp_direction);
    return 1;
  }
  NewNode = bio_expandnode_ligandbased_rrt(G,Nnear,qexp_direction);
  p3d_destroy_config(robotPt, qexp_direction);
  if(NewNode!= NULL) {
    if(!p3d_GetIsWeightStopCondition()) {
      check_if_node_connects_connected_components(G, NewNode);
    }
    if(p3d_GetExpansionDirectionMethod() == SUBREGION_CS_EXP) {
      //      p3d_resize_rrt_box(robotPt, CompPt->box_env_small, NewNode, 1);
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
  return Added;
}

//****************************************************************/
/*!
* \brief extend a given node with the RRT ligand method 
*        in the direction qexp_direction, which is composed of:
*        - q_ligand (randomly shoot): for the ligand dofs
*        - q_Near: for the side chains of the ligand far from the ligand 
*                  for the node Nnear. Thuse these ddls are kept unchanged
*                  along the local path Nnear q_exp_direction
*        - q_rand:  for the side chains dofs closed to theligand 
*                   for the node Nnear.
* \param G: the graph
* \param comptPt: pointer to the connected component 
* \param q:  the configuration to connect. Only the ligand dofs are useful. 
*            The other dofs are set to 0. 
* \return TRUE if a node has been added FALSE otherwise
*/
/****************************************************************/
p3d_node* bio_expandnode_ligandbased_rrt(p3d_graph* G,p3d_node* Nnear,
					 configPt qexp_direction) {
  p3d_rob* robotPt = G->rob;
  p3d_localpath *path;
  configPt qnew;
  double Kpath = 0, dist = 0;
  int **iksol = NULL;  
  p3d_node* NewNode;
  double stop_weight;
  int sign_stop_weight; 

  if (p3d_equal_config(robotPt, qexp_direction, Nnear->q)) {
    if((robotPt->cntrt_manager == NULL)||(robotPt->cntrt_manager->cntrts == NULL)) /* no display if constraints */ 
      PrintInfo(("qnew invalide\n"));
    return NULL;
  }
  (G->nb_q_closed)++;
  if (!(path = p3d_local_planner(robotPt, Nnear->q, qexp_direction))) {
    PrintInfo(("Planification impossible: problem with local path creation\n"));
    return NULL;
  }
  (G->nb_local_call)++;
  qnew = p3d_alloc_config(robotPt);
  p3d_unvalid_localpath_classic_test(robotPt, path, &(G->nb_test_coll), &Kpath, &qnew);
  if (Kpath==0) {
    path->destroy(robotPt,path);
    p3d_destroy_config(robotPt,qnew);
    return NULL;
  }
  dist = Kpath * path->length_lp;
  path->destroy(robotPt,path);
  p3d_set_and_update_this_robot_conf_without_cntrt(robotPt,qnew);
  //we get the index of the solution generated by the cntrt
  // WARNING: iksol does not work, a new vector containing the 1st solution of each constraint must be passed as argument
  p3d_get_iksol_vector(robotPt->cntrt_manager,&iksol);
  NewNode = p3d_APInode_make_multisol(G, qnew, NULL);
  if(NewNode == NULL) {
    PrintInfo(("Error in the process of new node creation\n"));
  }
  NewNode->type = LINKING; 
  p3d_SetNodeWeight(G,NewNode);
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

