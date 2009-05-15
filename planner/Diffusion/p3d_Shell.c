#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Collision-pkg.h"

#define MAXN_CONSEC_PB_LEVEL 500
#define NMEM_RETRO_PB_LEVEL 5

static double last_retro_pb_level[NMEM_RETRO_PB_LEVEL];
static double BIO_Rr_shell = 0.1;
static int N_GOOD = 0;

#define DEBUGRRT  0
/**
 * p3d_init_pb_variables
 * Init the graph variables needed to  use the shells
 * @param[In] GraphPt: pointer to the robot graph 
 */
void p3d_init_pb_variables(p3d_graph* GraphPt){
  int i;
  if((!GraphPt) ||
     (!GraphPt->search_start) ) {
    PrintInfo(("Error in shell initialisation: \
NULL parameters\n"));
    return;
  }
  p3d_SetNGood(0);
  if(GraphPt->search_start->comp->nnode == 1) {
    GraphPt->CurPbLevel = 0.0;
    
    for(i=0; i<NMEM_RETRO_PB_LEVEL; i++)
      last_retro_pb_level[i] = 0.0;
    GraphPt->critic_cur_pb_level = 0.0;
    GraphPt->n_consec_fail_pb_level = 0;
    GraphPt->n_consec_pb_level = 0;
  }
}


/**
 * hrm_selected_pb_node
 * return the node selected thinks to the shell method
 * @param[In] GraphPt: pointer to the robot graph
 * @param[In] q: the configuration used to get the direction of 
 * expansion
 * @param[In comp: the connected componant to extend
 * @return: the selected node of the componant 
 */
p3d_node* hrm_selected_pb_node(p3d_graph* GraphPt, configPt q, p3d_compco *comp)
{
  p3d_list_node *ListNode;
  p3d_node **array_nodes_in_p_shell;
  p3d_node *selected_N = NULL;
  int rand_nnode;
  int i;
  double Rr_shell;
  int nnodes_in_p_shell = 0, max_nnodes_in_p_shell = 300;
  //p3d_poly **listpol1, **listpol2;
  //int col_number;
  int j;
  //int indexligand;
  int nlower_retro_pb_level = 0;

  array_nodes_in_p_shell = MY_ALLOC(p3d_node *,max_nnodes_in_p_shell);

  //#ifdef BIO
  if(p3d_col_get_mode() == p3d_col_mode_bio)
    Rr_shell = BIO_Rr_shell;
  //#else
  else
    //Rr_shell = p3d_get_env_dmax();
    Rr_shell = 1.0;
  //#endif

  while(nnodes_in_p_shell == 0) {
    ListNode  = comp->dist_nodes;
    while(ListNode != NULL) {
      if((ListNode->N->weight > (GraphPt->CurPbLevel - Rr_shell)) && 
	 (ListNode->N->weight <= GraphPt->CurPbLevel)) {
	// nodes that cannot be extended MAXN_FAILS consecutive times are disregarded (modif Juan)
	if((ListNode->N->weight == 0.0)||(ListNode->N->n_fail_extend < p3d_GetMaxExpandNodeFail())) {
 	  if(nnodes_in_p_shell > max_nnodes_in_p_shell) {
 	    array_nodes_in_p_shell = MY_REALLOC(array_nodes_in_p_shell,p3d_node *,
 						max_nnodes_in_p_shell,max_nnodes_in_p_shell*2.0);
 	    max_nnodes_in_p_shell *= 2.0;
 	  }
	  
	  if((GraphPt->n_consec_pb_level < MAXN_CONSEC_PB_LEVEL) &&
	     (GraphPt->n_consec_fail_pb_level < MAXN_CONSEC_PB_LEVEL/10)) {
	    array_nodes_in_p_shell[nnodes_in_p_shell] = ListNode->N;
	    nnodes_in_p_shell ++;
	  }
	  else {
	    for(i=0; i<nnodes_in_p_shell; i++) {
	      if(array_nodes_in_p_shell[i]->n_fail_extend > 0) {
		array_nodes_in_p_shell[i]->n_fail_extend = p3d_GetMaxExpandNodeFail();
		update_parent_nfails(array_nodes_in_p_shell[i]);
	      }	
	    }
	    nnodes_in_p_shell = 0;
	    GraphPt->n_consec_pb_level = 0;
	    GraphPt->n_consec_fail_pb_level = 0;
	    if(DEBUGRRT) printf("erasing shell !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
	    break;
	  }
	}
      }
      ListNode = ListNode->next;
    }
    if(nnodes_in_p_shell == 0) {
      GraphPt->CurPbLevel -= Rr_shell;
      nlower_retro_pb_level = 0;
      for(j=0; j<NMEM_RETRO_PB_LEVEL; j++) {
	if(GraphPt->CurPbLevel< (last_retro_pb_level[j] + Rr_shell/2.0)) {
	  //if(p3d_GetCurPbLevel() < last_retro_pb_level[j]) {
	  nlower_retro_pb_level++;
	}
      }
      //GraphPt->CurPbLevel =p3d_GetCurPbLevel() - 2*Rr_shell;
      if(nlower_retro_pb_level>0) {
	printf("multi-retro * %d\n",nlower_retro_pb_level);
	if(nlower_retro_pb_level > ((int)ceil((double)NMEM_RETRO_PB_LEVEL/2.0)))
	  GraphPt->CurPbLevel =0.0;
	else
	  GraphPt->CurPbLevel = GraphPt->CurPbLevel - Rr_shell * (double)nlower_retro_pb_level ;
	// update mem retro
	//for(j=0; j<NMEM_RETRO_PB_LEVEL; j++)
	//  last_retro_pb_level[j] = p3d_GetCurPbLevel();
	// AND ERASE PREVIOUS SHELL ????????
	ListNode  = comp->dist_nodes;
	while(ListNode != NULL) {
	  if(ListNode->N->weight > GraphPt->CurPbLevel) {
	    //if(ListNode->N->n_fail_extend > 0) {
	    ListNode->N->n_fail_extend = p3d_GetMaxExpandNodeFail();
	    update_parent_nfails(ListNode->N);
	    //}	    
	  }
	  ListNode = ListNode->next;
	}
      }      
      // IDEA : ELEGIR EL MEJOR EN FUNACION DE LA INFORMACION ??? (POCOS NODOS CERCANOS SATURADOS)

/*       if(GraphPt->critic_cur_pb_level < p3d_GetCurPbLevel()) { */
/* 	GraphPt->CurPbLevel = GraphPt->critic_cur_pb_level; */
/* 	ListNode  = comp->dist_nodes; */
/* 	while(ListNode != NULL) { */
/* 	  if(ListNode->N->weight > p3d_GetCurPbLevel()) */
/* 	    ListNode->N->n_fail_extend = MAXN_FAILS; */
/* 	  ListNode = ListNode->next; */
/* 	} */
/*       } */
      if(GraphPt->CurPbLevel< 0.0)
	GraphPt->CurPbLevel =0.0;
      //if(DEBUGRRT) 
      printf("retrograded cur_pb_level = %f\n",GraphPt->CurPbLevel);
      for(j=0; j<NMEM_RETRO_PB_LEVEL-1; j++) {
	last_retro_pb_level[j] = last_retro_pb_level[j+1];
      }
      last_retro_pb_level[NMEM_RETRO_PB_LEVEL-1] = GraphPt->CurPbLevel;
     
#ifdef BIO
/*       // identify and print blocking atom pairs */
/*       bio_set_col_mode(SURFACE_DISTANCE_BIOCOL_MODE); */
/*       //dist_surf =  bio_get_surface_d(); */
/*       //bio_set_surface_d(0.5);  // PROBAR OTROS VALORES !!!??? */
/*       // WARNING : suppose that the current robot conf. corresponds to the last computed node   */
/*       if(num_subrobot_ligand(rob)>0) { // if ligand */
/* 	indexligand = get_number_of_bcd_robots() - 1; */
/* 	bio_molecule_col_no_autocol(indexligand); */
/*       } */
/*       else {    // if no ligand */
/* 	bio_all_molecules_col(); */
/*       } */
/*       biocol_report(&col_number,&listpol1,&listpol2); */
/*       printf("### Num. heavy atoms in contact %d ###\n",col_number); */
/*       for(j=0;j<col_number;j++) { */
/* 	  printf("%s  -  %s\n",listpol1[j]->poly->name,listpol2[j]->poly->name); */
/*       }     */
/*       bio_set_col_mode(NORMAL_BIOCOL_MODE); */
#endif
    }
  }
  // random node in array
  if(DEBUGRRT) printf("nnodes_in_p_shell = %d\n",nnodes_in_p_shell);
  rand_nnode = (int) floor(p3d_random(0.0, (double)nnodes_in_p_shell - EPS6));
  selected_N = array_nodes_in_p_shell[rand_nnode];
  MY_FREE(array_nodes_in_p_shell,p3d_node *,max_nnodes_in_p_shell);  
  return selected_N;
}


/**
 * update_parent_nfails
 * update the n_fail_extend value of the parent node 
 * of a given node
 * @param[In]: The given node
 * @return: TRUE if the n_fail_extend value of the parent node
 * has been set to rrt_maxn_fails.
 */
int update_parent_nfails(p3d_node *N)
{
  p3d_node *pN;
  p3d_list_node *ListNode;
  int keep = 0;

  pN = N->parent;
  if(pN != NULL) {
    if((pN->n_fail_extend < p3d_GetMaxExpandNodeFail()) && (pN->weight > 0.0)) {
      if(pN->nneighb > 2) { // at least one child
	ListNode  = pN->neighb;
	while((ListNode != NULL) && !keep) {
	  if(ListNode->N->num > pN->num) {  // is a child
	    if(ListNode->N->n_fail_extend < p3d_GetMaxExpandNodeFail())
	      keep = 1;
	  }
	  ListNode = ListNode->next;
	}
	if(!keep) {
	  pN->n_fail_extend = p3d_GetMaxExpandNodeFail();
	  // If we want to discard the parent node
	  pN->IsDiscarded = TRUE;
	  if(DEBUGRRT)
	    printf("updated parent node : %d\n",pN->num);
	}
      } 
    }
  }
  return(!keep);
}


/**
 *p3d_GetNGood
 * Get The number of time a new node 
 * has been created without fail
 * @return: The  number of time a 
 * new node has been created without fail
 */
int p3d_GetNGood(void) {
  return N_GOOD;
}

/**
 * p3d_SetNGood
 * Set The number of time a new node 
 * has been created without fail
 * @param[In]: The  number of time a 
 * new node has been created without fail
 */
void p3d_SetNGood(int NGood) {
  N_GOOD = NGood;
}
