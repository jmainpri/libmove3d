/*******************************
General purpose Astar Algo
Written by T. Simeon 
nic@grasp.cis.upenn.edu
U.P.E.N.N july 89
*******************************/

#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"

#define DEBUG(WHAT)

extern int GlobalOrdering;
/**********************************************************************/
static void  InitializeSearch(p3d_graph *graph);
static void  RecordSolution(p3d_node *goal,int n,p3d_graph *graph);
static int   End_of_Search(p3d_node *node,p3d_graph *graph,
			   int (*fct_end)(p3d_node *, p3d_node *));
static double ComputeHeurist(p3d_node *node,
			     double (*fct_heurist)(p3d_node *,p3d_node *),p3d_graph *graph);
/***************************************************************************/

/**
 * p3d_graph_search
 * Main function of the A* algorithm (astar), 
 * extracting a solution path from a graph structure.
 *  The algorithm is based on Equilibrated Binary Trees
 * (EBT) in order to increase the efficiency (see ebt.c) 
 * @param[In]: (*fct_heurist): the heuristic function evaluating 
 * the cost to reach the goal from the given node (can take for exemple
 * the distance to the goal)
 * @param[In]:(*fct_valid): a function testing if an edge/arc is allowed.
 * Currently this function doesn't make any collision tes (ie. assume that the existing
 * edges are valid).
 * @param[In]: (*fct_end): Function testing if node extended reached the goal.
 * @return: TRUE if a solution path has been found and FALSE otherwise. 
 */

int p3d_graph_search(p3d_graph *graph, 
		     double (*fct_heurist)(p3d_node *, p3d_node *),
                     int (*fct_valid)(p3d_node *, p3d_edge *, p3d_graph *),
		     int (*fct_end)(p3d_node *, p3d_node *)) {
  char         *open    = NULL;
  int          num_dev  = 0;
  p3d_node     *bestNodePt, *V;
  p3d_list_edge *listEdgePt;
  double hCurrent, gCurrent;
/*   double edgeCost; */
  if((graph->search_start == NULL) || 
     (graph->search_goal == NULL)) {
    fprintf(stderr,"Warning: start/goal undefined\n");
    return(FALSE);
  }
  InitializeSearch(graph);
  EBT_INSERT(graph->search_start,&open);

  while(TRUE) {
    if(EBT_EMPTY(open)) {
      /* The path doesn't exist */
      RecordSolution(NULL,num_dev,graph);
      return(FALSE);
    }

    EBT_GET_BEST(bestNodePt,&open);

    if(End_of_Search(bestNodePt,graph,fct_end) == TRUE) {
      /* A path has been found */
      RecordSolution(bestNodePt,num_dev,graph);

      /*deasallocation of the memory*/
      while(!EBT_EMPTY(open)) {
        EBT_GET_BEST(V,&open);
      }
      return(TRUE);
    }

    bestNodePt->closed = TRUE;  
    num_dev++;
    /* No solution found yet, bestNode
       has to be developped */
   
    listEdgePt = bestNodePt->edges;
    /*Propagation along all the outcoming nodes*/
    while(listEdgePt != NULL) {
      V = listEdgePt->E->Nf;
      if(fct_valid != NULL) {
	if(!(*fct_valid)(V,listEdgePt->E,graph)) {
	  /*We have a validation funcion which invalidate the edge*/
	  /*the arc is not considered*/
	  listEdgePt = listEdgePt->next;
	  continue;
	}
      }
      if(IsNodeInPath(bestNodePt, V) == TRUE) {
	listEdgePt = listEdgePt->next;
	continue;
      }
	gCurrent = bestNodePt->g + listEdgePt->E->cost;

	hCurrent =  ComputeHeurist(V,fct_heurist,graph);
	hCurrent = 0.;
      if((EBT_OPENED(V) == FALSE) && (EBT_CLOSED(V) == FALSE)) {
	V->search_from = bestNodePt;
	V->edge_from = listEdgePt->E;
	V->g = gCurrent;
	V->h = hCurrent;
	V->f =	V->g + V->h;
	//	listEdgePt->E->cost = edgeCost;
	EBT_INSERT(V,&open);
      } else {
	if(V->g > gCurrent) {
	  V->search_from = bestNodePt;
	  V->edge_from = listEdgePt->E;
	
	  V->g = gCurrent;
	  V->h = hCurrent;
	  V->f = V->g + V->h;
	  //listEdgePt->E->cost = edgeCost;
	  if(EBT_CLOSED(V) == FALSE) {
	    V->closed = FALSE;
	    EBT_INSERT(V,&open);
	  }
	} else {
	}
      }
      listEdgePt = listEdgePt->next;
    }
  }
}

/**
 * Graph search function that find the 
 * path such that the highest cost is minimised, 
 * and all the successive lower cost in case of equality 
 * between two paths. 
 * @param[In]: (*fct_heurist): the heuristic function evaluating 
 * the cost to reach the goal from the given node (can take for exemple
 * the distance to the goal)
 * @param[In]:(*fct_valid): a function testing if an edge/arc is allowed.
 * Currently this function doesn't make any collision tes (ie. assume that the existing
 * edges are valid).
 * @param[In]: (*fct_end): Function testing if node extended reached the goal.
 * @return: TRUE if a solution path has been found and FALSE otherwise. 
 *
 */
int p3d_OrderingGraphSearch(p3d_graph *graph, 
		     double (*fct_heurist)(p3d_node *, p3d_node *),
                     int (*fct_valid)(p3d_node *, p3d_edge *, p3d_graph *),
		     int (*fct_end)(p3d_node *, p3d_node *)) {
  char         *open    = NULL;
  int          num_dev  = 0, res;
  p3d_node     *bestNodePt, *V;
  p3d_list_edge *listEdgePt;
  dbl_list* tmpOrderCostListEdge;

  if((graph->search_start == NULL) || 
     (graph->search_goal == NULL)) {
    fprintf(stderr,"Warning: start/goal undefined\n");
    return(FALSE);
  }
  InitializeSearch(graph);
  EBT_INSERT(graph->search_start,&open);
  graph->search_start->orderCostListEdge = dbl_list_pointer_init();
  while(TRUE) {
    if(EBT_EMPTY(open)) {
      /* The path doesn't exist */
      RecordSolution(NULL,num_dev,graph);
      return(FALSE);
    }

    EBT_GET_BEST(bestNodePt,&open);

    if(End_of_Search(bestNodePt,graph,fct_end) == TRUE) {
      /* A path has been found */
      RecordSolution(bestNodePt,num_dev,graph);

      /*deasallocation of the memory*/
      while(!EBT_EMPTY(open)) {
        EBT_GET_BEST(V,&open);
      }
      return(TRUE);
    }

    bestNodePt->closed = TRUE;  
    num_dev++;
    /* No solution found yet, bestNode
       has to be developped */
   
    listEdgePt = bestNodePt->edges;
    /*Propagation along all the outcoming nodes*/
    while(listEdgePt != NULL) {
      V = listEdgePt->E->Nf;
      if(fct_valid != NULL) {
	if(!(*fct_valid)(V,listEdgePt->E,graph)) {
	  /*We have a validation funcion which invalidate the edge*/
	  /*te arc is not considered*/
	  listEdgePt = listEdgePt->next;
	  continue;
	}
      }
      if((EBT_OPENED(V) == FALSE) && (EBT_CLOSED(V) == FALSE)) {
	V->search_from = bestNodePt;
	V->edge_from = listEdgePt->E;
	V->orderCostListEdge = dbl_list_copy(bestNodePt->orderCostListEdge);

	dbl_list_insert_sorted_link(V->orderCostListEdge,  listEdgePt->E, sortCostBestEdge);
	EBT_INSERT(V,&open);
      } else {
	tmpOrderCostListEdge =  dbl_list_copy(bestNodePt->orderCostListEdge);
	dbl_list_insert_sorted_link(tmpOrderCostListEdge, listEdgePt->E , sortCostBestEdge);
	res = dbl_list_test_equal(tmpOrderCostListEdge,
				  V->orderCostListEdge, costBestEdge);
	if(res<0) {
	  dbl_list_destroy(V->orderCostListEdge);
	  V->orderCostListEdge = tmpOrderCostListEdge;
	  V->search_from = bestNodePt;
	  V->edge_from = listEdgePt->E;
	  
	  if(EBT_CLOSED(V) == FALSE) {
	    V->closed = FALSE;
	    EBT_INSERT(V,&open);
	  }
	} else {
	  dbl_list_destroy(tmpOrderCostListEdge);
	}
      }
      listEdgePt = listEdgePt->next;
    }
  }
}
/**********************************************************************/

static void
InitializeSearch(p3d_graph *graph) {
  int i;
  p3d_list_node  *lnodes = graph->nodes;
  p3d_node   *node;


  for(i=1;i<=graph->nnode;i++) {
    node = lnodes->N;
    node->opened = 0;
    node->closed = 0;
    lnodes = lnodes->next;
  }

  node              = graph->search_start;
  node->search_from = NULL;
  node->search_to   = NULL;
  node->f           = .0;
  node->g           = .0;

  graph->search_goal->search_from = NULL;
  graph->search_goal->search_to = NULL;
}

/**********************************************************************/

static void
RecordSolution(p3d_node *goal,int n,p3d_graph *graph) {
  p3d_node *node = goal;
  int        l    = 1;

  graph->search_done    = TRUE;
  graph->search_numdev  = n;

  if(goal == NULL) {
    graph->search_path      = FALSE;
    graph->search_goal_sol  = NULL;
    graph->search_cost      = .0;
  }
  else {
    graph->search_path      = TRUE;
    graph->search_goal_sol  = goal;
    graph->search_cost      = goal->f;

    while(node->search_from != NULL) {
      l++;
      node->search_from->search_to = node;
      node = node->search_from;
    }
    graph->search_path_length = l;
  }
}

/**********************************************************************
  Some internal functions....
  **********************************************************************/

static double
ComputeHeurist(p3d_node *node,double (*fct_heurist)(p3d_node *, p3d_node *),p3d_graph *graph) {
  double h;

  if(fct_heurist != NULL) {
    h = (*fct_heurist)(node,graph->search_goal);
    return(h);
  } else
    return(.0);
}

/**********************************************************************/


static int
End_of_Search(p3d_node *node,p3d_graph *graph,int (*fct_end)(p3d_node *, p3d_node *)) {
  if(fct_end(graph->search_goal,node)) {return(TRUE);} else {return(FALSE);}
}


/**********************************************************************/
/* Fonctions heuristique, end et valid */
/* (temporaires)                       */
/***************************************/
int p3d_valid(p3d_node * nodePt, p3d_edge * edgePt, p3d_graph * graphPt) {
  if((graphPt!=NULL) && (edgePt!=NULL) && (nodePt!=NULL)) { return(TRUE); }
  PrintInfo(("ERREUR : dans p3d_valid\n"));
  return(FALSE);
}

int p3d_valid_multisol(p3d_node * nodePt, p3d_edge * edgePt, p3d_graph * graphPt) {
  p3d_node * Ni = NULL;
  if((graphPt!=NULL) && (edgePt!=NULL) && (nodePt!=NULL)) {
    Ni = edgePt->Ni; //the node before nodePt.
    if (Ni->search_from == NULL){//first node.
      return (TRUE);
    }
    while(Ni != NULL){
      if (!p3d_compare_iksol(graphPt->rob->cntrt_manager, Ni->iksol, nodePt->iksol)){
        return (FALSE);
      }
      if(Ni->isSingularity == 1){
        break;
      }
      Ni = Ni->search_from;
    }
    return(TRUE);
  }
  PrintInfo(("ERREUR : dans p3d_valid\n"));
  return(FALSE);
}


int p3d_end(p3d_node *n1, p3d_node *n2) {
  p3d_rob *rob = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);

  return p3d_equal_config(rob, n1->q, n2->q);
}




/**
 * p3d_heurist
 * Heuristic used in the Astar algorithm
 * should return a minorant value of the edge cost 
 * between
 * @param[In] n1: first node
 * @param[In] n2: second node (usually the goal)
 * @return: A minorant cost value. 
 */
double p3d_heurist(p3d_node *n1, p3d_node *n2) {
  p3d_rob *rob = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);

  // In case of Cost Spaces, we don't give a minorant heuristic
  // as it is non trivial. However epsilon*dist seems correct
  // Where epsilon is the small value put in the edges costs for 
  // downhillslopes.  
   if(ENV.getBool(Env::isCostSpace) == TRUE) {
     return 0.;
   }
    return p3d_dist_config(rob, n1->q, n2->q);
}



//start path deform
static int MAX_NB_TEST_PATH = 10;
static int NNODE_MAX = 10;
/**
 * ---------------TODO---------------------------------
 */
int p3d_graph_many_search(p3d_graph *graph, double (*fct_heurist)(p3d_node *, p3d_node *),
                          int (*fct_valid)(p3d_node *, p3d_edge *, p3d_graph *),int (*fct_end)(p3d_node *, p3d_node *), p3d_traj* traj) {
  char *open = NULL;
  int num_dev = 0,i;
  p3d_node *U_best,*V = NULL;
  p3d_list_edge *list_edgePt;
  int DEJAVU, nb_path = 0;
  p3d_path_nodes* nodeOfPath, *NewNodeOfPath;
  p3d_traj* current_trajPt = NULL;

  if((graph->search_start == NULL) || (graph->search_goal == NULL)) {
    fprintf(stderr,"Warning: start/goal undefined\n");
    return(FALSE);
  }

  //list_path_nodes = dbl_list_pointer_init();
  InitializeSearch(graph);

  // insertion the first node
  nodeOfPath = p3d_allocinit_path_nodes();
  nodeOfPath->list_node = dbl_list_pointer_init();
  dbl_list_concat_link(nodeOfPath->list_node, graph->search_start);
  nodeOfPath->N = graph->search_start;
  nodeOfPath->nnode = 1;
  nodeOfPath->h = ComputeHeurist(nodeOfPath->N,fct_heurist,graph);//distance entre graph->search_start et graph->search_goal
  nodeOfPath->g = 0.0;
  nodeOfPath->f = nodeOfPath->h + nodeOfPath->g;

  EBT_INSERT_PATH(nodeOfPath, &open);//mise dans l'arbre binaire

  while(TRUE) {
    if(EBT_EMPTY(open)) {//si l'arbre est vide
      return(FALSE);
    }
    if (nb_path > MAX_NB_TEST_PATH){//si le nombre limit de tests a ete atteint
      while(!EBT_EMPTY(open)) {//vider l'arbre et desalouer les noeuds
        EBT_GET_BEST_PATH(nodeOfPath, &open);
        p3d_destroy_path_nodes(nodeOfPath);
      }
      return(FALSE);
    }
    EBT_GET_BEST_PATH(nodeOfPath, &open);//remplissage de nodeOfPath et supression du meilleur element de open
    U_best = nodeOfPath->N;//meilleur noeud

    if(End_of_Search(U_best,graph,fct_end) == TRUE) {// si U_best == graph->search_goal
      graph->search_goal = U_best;//ne sert a rien vu que U_best == graph->search_goal
      nb_path++;
      current_trajPt = p3d_create_traj_from_list_nodes(graph, nodeOfPath->list_node);//create trajectory
      p3d_set_nretract(0);//TODO verifier si remet nretract a l'etat
      if(p3d_test_projection(graph->rob, current_trajPt, traj,p3d_get_Nstep()) == TRUE) {
        /*the cycle is reductible */
        p3d_del_traj(current_trajPt);//mokhtar
        while(!EBT_EMPTY(open)) {
          EBT_GET_BEST_PATH(nodeOfPath, &open);
          p3d_destroy_path_nodes(nodeOfPath);
        }
        return(TRUE);
      }
      p3d_del_traj(current_trajPt);//mokhtar
    } else {
      /* U_best has to be developped */
      if(nodeOfPath->nnode < NNODE_MAX) {
        num_dev++;
        /* consider all the edges starting from or ending at U_best */
        list_edgePt = U_best->edges;
        for(i=1;i<=U_best->nedge;i++) {
          DEJAVU = 0;
          /* consider all the neigbors of U_best */
          if((list_edgePt->E->Ni == U_best) &&
              (!dbl_list_find_by_data(nodeOfPath->list_node,list_edgePt->E->Nf,NULL))) { //Nf appartient a path_nodes->list_node
            V = list_edgePt->E->Nf;
          } else {
            DEJAVU = 1;
          }

          /* Check if this arc has to be considered */
          if((DEJAVU == 0) && (fct_valid != NULL)) {
            if((*fct_valid)(V,list_edgePt->E,graph) == 0) {//if V!=null && list_edgePt->E!=Null && graph != null
              DEJAVU = 1;
            }
          }
          if (DEJAVU == 0) {
            NewNodeOfPath = p3d_allocinit_path_nodes();
            NewNodeOfPath->list_node = dbl_list_copy(nodeOfPath->list_node);
            dbl_list_concat_link(NewNodeOfPath->list_node, V);
            NewNodeOfPath->nnode = nodeOfPath->nnode+1;
            NewNodeOfPath->N = V;
            NewNodeOfPath->h = ComputeHeurist(V,fct_heurist,graph);
            NewNodeOfPath->g = nodeOfPath->g  + list_edgePt->E->longueur;
            NewNodeOfPath->f = NewNodeOfPath->h + NewNodeOfPath->g;
	    NewNodeOfPath->N->search_from = U_best;
            if(NewNodeOfPath->opened == FALSE) {
              EBT_INSERT_PATH(NewNodeOfPath,&open);
            } else {
              PrintInfo(("Warning : path_node already in open list \n"));
              p3d_destroy_path_nodes(NewNodeOfPath);
            }
          }
          /* Possibilite de faire la desallocation a verifier*/
          list_edgePt = list_edgePt->next;
        }
      }
    }
    p3d_destroy_path_nodes(nodeOfPath);
    nodeOfPath  = NULL;
  }
}


p3d_path_nodes*  p3d_allocinit_path_nodes(void) {
  p3d_path_nodes* path_nodes;
  path_nodes = MY_ALLOC(p3d_path_nodes,1);

  path_nodes->f = 0.;
  path_nodes->g = 0.;
  path_nodes->h = 0.;
  path_nodes->opened =0;
  path_nodes->closed =0;
  path_nodes->nnode = 0;

  return path_nodes;
}
void p3d_destroy_path_nodes(p3d_path_nodes* path_nodes) {
  dbl_list_destroy(path_nodes->list_node);
  path_nodes->list_node = NULL;
  MY_FREE(path_nodes,p3d_path_nodes,1);
}

int IsNodeInPath(p3d_node* bestNodePt, p3d_node* N) {
  p3d_node* parentNode = bestNodePt;
  while(parentNode != NULL) {
    if(parentNode == N) {
      return TRUE;
    }
    parentNode= parentNode->search_from;
  }
  return FALSE;
}
