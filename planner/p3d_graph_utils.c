#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Bio-pkg.h"
#include "Move3d-pkg.h"
#include "Graphic-pkg.h"
#ifdef ENERGY
#include "../bio/BioEnergy/include/Energy-pkg.h"
#endif

extern double ZminEnv;
extern double ZmaxEnv;
extern double InitCostThreshold;
extern void* GroundCostObj;
static int nbAnaSuccesSamples = 100;
/* FONCTIONS DE CREATION DE STRUCTURES */

/***********************************************/
/* Fonction initialisant et allouant un graphe */
/* In :                                        */
/* Out : le graphe                             */
/***********************************************/
p3d_graph * p3d_allocinit_graph(void) {
  p3d_graph *G;

  G = MY_ALLOC(p3d_graph, 1);
  if (G == NULL) {
    PrintError(("Not enough memory !!!\n"));
    return NULL;
  }
  memset(G, 0, sizeof(p3d_graph));

  G->oriented = p3d_get_ORIENTED(); /* modif Fabien */
  return G;
}

/*--------------------------------------------------------------------------*/
/*! \brief Create a new component without nodes
 *
 *  This function create a new component without nodes and add it in
 *  the list of compco in the graph.
 *
 *  \param  G: the graph in witch the compco is added
 *
 *  \return the new compco (NULL if errors)
 *
 *  \note Most of the program assume that there is almost one node in each
 *        component, so use this function carefully.
 *  \note Use p3d_create_compco() for creating a compco for a specific node
 *  \note Some parameters in the data structure G are modified : number of components
 *  included in the graph, and the component's list.
 */
p3d_compco * p3d_create_void_compco(p3d_graph * G) {
  p3d_compco * C;

  /* Structure's allocation */
  C = MY_ALLOC(p3d_compco, 1);
  if (C == NULL) {
    PrintError(("Not enough memory !!!\n"));
    return NULL;
  }
  /* Structure intitialization */
  memset(C, 0, sizeof(p3d_compco));

  /* Increase the number of components for the graph */
  G->ncomp = G->ncomp + 1;

  /* Declaration of the new component in graph component list */
  C->num   = G->ncomp;
  C->prec  = G->last_comp;

  if (G->comp != NULL) {
    G->last_comp->suiv = C;
  } else {
    G->comp = C;
  }

  G->last_comp = C;
  C->AnaSuccessTab = MY_ALLOC(int,nbAnaSuccesSamples);
  return C;
}

/*--------------------------------------------------------------------------*/
/*! \brief Allocates and Initializes a new Node
 *
 *  This function allocates a node's structure and initializes the data
 *  No other data modified
 *
 *  \return pointer to the new node (NULL if errors)
 */
p3d_node * p3d_allocinit_node(void) {

  p3d_node * N;

  /* Allocation of the structure of the node */
  N = MY_ALLOC(p3d_node, 1);
  if (N == NULL) {
    PrintError(("Not enough memory !!!\n"));
    return NULL;
  }

  /* Initialization of the data */
  memset(N, 0, sizeof(p3d_node));

  N->type = 2; /* Carl: default value, no meaning */
  N->numcomp = -1; /* Value -1 means the node is not included in a compco */
  N->iksol = NULL;           // <- modified Juan
  N->n_fail_extend = 0;   // modif Juan (test)
  N->n_extend = 0;        // modif Juan (test)
  N->weight = 0.0;        // modif Juan (test)
  N->E = 0.0;             // modif Noureddine (test)
  N->avr_dist = 0.0;      // modif Noureddine (test)
  N->min_dist = 0.0;      // modif Noureddine (test)
  N->parent = NULL;       // modif Juan (test)
  N->pinpointed = 0;
  N->list_closed_flex_sc =NULL; // modif ljaillet (")
  N->IsDiscarded = FALSE;
  N->nbFailedTemp = 0;
  //  N->NbDown = 0;
  return N;
}

/*--------------------------------------------------------------------------*/
/*! \brief Create Compco for a Node

 *
 *  This function allocates a compco structure, and insert the node given
 *  in parameter as the only one node contained in the compco
 *
 *  Data modified in the graph structure
 *
 *  \param G: the graph in which the compco is declared
 *  \param N: the node that will be contained in the new compco
 *
 *  \note The node is supposed to be not contained in another comcpo
 *        (N->comp == NULL || N->numcomp == -1)
 */
void p3d_create_compco(p3d_graph *G,
                       p3d_node *N) {
  p3d_compco * C;   /* Pointer to the new component */
  p3d_list_node *NL;            /* Pointer to a list of nodes */

  /* Prepare a new empty component */
  C = p3d_create_void_compco(G);

  C->nnode = 1;

  /* Update information in the node structure  */
  N->comp    = C;
  N->numcomp = C->num;

  /* Prepare the chained list element */
  NL       = MY_ALLOC(p3d_list_node, 1);
  NL->N    = N;
  NL->next = NULL;
  NL->prev = NULL;

  /* Insert element in the graph list */
  //start path deform
  C->nodes = NL;
  //end path deform
  C->dist_nodes = NL;
  C->last_node  = NL;

  C->minCost= P3D_HUGE;
  C->maxCost= 0.;
  C->maxUrmsonCost= 0.;
  C->AnaSuccessTab = MY_ALLOC(int,nbAnaSuccesSamples);
  C->nbTests = 0;
}


/*--------------------------------------------------------------------------*/
/*! \brief Destroy the Graph

 *
 *  This function destroys the the graph given in parameter, the included
 *  comcpo, edges and nodes are also deleted.
 *
 *  \param G: the graph to be destroyed
 *  \return TRUE if oh, FALSE if a problem is encountered
 *
 *  \note : Data modified in the robot structure (reference to the graph)
 */

int p3d_del_graph(p3d_graph *G) {
  p3d_list_node *NLS, *NLD;  /* Node List Scanner, Node List pointer for Deletion */
  p3d_list_node *NeLS, *NeLD; /* Neighbour List Scanner, Neighbour List pointer for Deletion */
  p3d_list_edge *ELS, *ELD;  /* Edge List Scanner, Edge pointer for Deletion */
  p3d_compco *CS, *CD;      /* Compco Scanner, Compco pointer for Deletion */

  /* verification of the existence of the graph */
  if (!G)
    return (FALSE);

  /* compco desallocation*/
  CS = G->comp;
  while (CS) {
    CD = CS;
    CS = CS->suiv;
    p3d_remove_compco(G, CD);
  }

  /*nodes desallocation*/
  NLS = G->nodes;
  while (NLS) {
    NLD = NLS;
    NLS = NLS->next;

    /* neighbour list desallocation */
    NeLS = NLD->N->neighb;
    while (NeLS) {
      NeLD = NeLS;
      NeLS = NeLS->next;
      MY_FREE(NeLD, p3d_list_node, 1);
    }
    /* node's edge list desallocation */
    ELS = NLD->N->edges;
    while (ELS) {
      ELD = ELS;
      ELS = ELS->next;
      MY_FREE(ELD, p3d_list_edge, 1);
    }

    /* node structure desallocation */
    p3d_APInode_desalloc(G, NLD->N);
    MY_FREE(NLD, p3d_list_node, 1);
  }
  G->nnode = 0;

  /* edges destruction */
  ELS = G->edges;
  while (ELS) {
    ELD = ELS;
    ELS = ELS->next;
//     if(ELD->E->path != NULL){
//       ELD->E->path->destroy(G->rob, ELD->E->path);
//     }
    MY_FREE(ELD->E, p3d_edge, 1);
    MY_FREE(ELD, p3d_list_edge, 1);
  }
  G->nedge = 0;

  if((G->stat != NULL)){
    destroyStat(&(G->stat));
  }
  /* Delete references to the graph */
  if (G->rob != NULL) {
    G->rob->GRAPH = NULL;
    if (XYZ_GRAPH == G) {
      XYZ_GRAPH = NULL;
    }
  }

  MY_FREE(G, p3d_graph, 1);

  return (TRUE);
}

void p3d_remove_compco(p3d_graph * G, p3d_compco * CDel) {
  p3d_compco * CS;              /* Compco Scanner */
  p3d_list_compco * CLS, * CLD; /* Compco List Scanner, Compco List for Deletion */
  p3d_list_node   * NLS, * NLD; /* Node List Scanner, Node List pointer for Deletion */

  CS = G->comp;
  /* Read the graph compco list until the one to del is reached  */
  while (CS != NULL && CS != CDel) {
    CS = CS->suiv;
  }

  /* Update the compco list (delete the reference to the compco to delete) */
  if (CS != NULL) {
    /* General case */
    if (CS->prec != NULL) {
      CS->prec->suiv = CS->suiv;
    }
    /* Particular case : The compco to delete is the first in the graph compco list */
    else {
      G->comp = CS->suiv;
    }
    /* General case */
    if (CS->suiv != NULL) {
      CS->suiv->prec = CS->prec;
    }
    /* Particular case : The compco is the last in the graph compco list */
    else {
      G->last_comp = CS->prec;
    }

    /* Delete dist_nodes list */
    NLS = CDel->dist_nodes;
    while (NLS != NULL) {
      NLD = NLS;
      NLS = NLS->next;
      MY_FREE(NLD, p3d_list_node, 1);
    }
    /* ORIENTED CASE */
    if (G->oriented) {
      /* DELETE REFERENCE IN OTHER COMPCO REACHABLE LISTS */
      CS = G->comp;
      while (CS != NULL) {
        if (p3d_compco_linked_to_compco(CS, CDel)) {
          p3d_del_compco_from_reachable_list(CS, CDel);
        }
        CS = CS->suiv;
      }
    }
    CLS = CDel->canreach;

    while (CLS != NULL) {
      CLD = CLS;
      CLS = CLS->next;
      MY_FREE(CLD, p3d_list_compco, 1);
    }

    MY_FREE(CDel->AnaSuccessTab, int, nbAnaSuccesSamples);
    /* STRUCTURE DESALLOCATION */
    MY_FREE(CDel, p3d_compco, 1);
  }

  /* UPDATE GRAPH COMPCO NUMEROTATION */
  CS = G->comp;
  while (CS != NULL) {
    if (CS->prec != NULL) {
      if (CS->num != (CS->prec->num + 1)) {
        CS->num = CS->prec->num + 1;
        NLS = CS->dist_nodes;
        while (NLS != NULL) {
          NLS->N->numcomp = CS->num;
          NLS = NLS->next;
        }
      }
    }
    CS = CS->suiv;
  }
  G->ncomp --;
}

/* LIST MANIPULATION */

/*--------------------------------------------------------------------------*/
/*! \brief Repport the insertion of a node in a graph
 *
 *  \param  G: The graph description.
 *  \param  nodePt:  The node inserted.
 *  \note Graph data structure modified.
 */

void p3d_insert_node_in_graph(p3d_graph *G, p3d_node *nodePt) {
  p3d_list_node *NL;

  /* Allocation and Initialization */
  NL = MY_ALLOC(p3d_list_node, 1);
  NL->N = nodePt;

  /* Declaration of the new node in the graph node list */
  if (G->nodes == NULL) {
    NL->next = NULL;
    NL->prev = NULL;
    G->nodes = NL;
  } else {
    NL->next = NULL;
    G->last_node->next = NL;
    NL->prev = G->last_node;
  }

  G->last_node = NL;
  G->nnode = G->nnode + 1;
  NL->N->num = G->nnode;
}

/*--------------------------------------------------------------------------*/
/*! \brief Add a node in a node list
 *
 *  \param  N: The node to add.
 *  \param  TargetList:  The list to modify.
 *  \return the modified list.
 */
p3d_list_node * p3d_add_node_to_list(p3d_node * N,
                                     p3d_list_node * TargetList) {
  p3d_list_node *NewList = NULL;
  /* Allocation of a node list */
  NewList = MY_ALLOC(p3d_list_node, 1);

  /* Initialization and insertion in the target list */
  NewList->N = N;
  NewList->next = TargetList;
  NewList->prev = NULL;

  if (TargetList != NULL) {
    TargetList->prev = NewList;
  }
  return NewList;
}

/*--------------------------------------------------------------------------*/
/*! \brief Add a node to a compco
 *
 *  \param  N: The node to add.
 *  \param  C: The compco to modify.
 *  \param  reorder: Reorder the compco or not.
 */
void p3d_add_node_compco(p3d_node * N, p3d_compco * C, int reorder) {
  N->comp = C;
  N->numcomp = C->num;
  C->dist_nodes = p3d_add_node_to_list(N, C->dist_nodes);
  //start path deform
  //C->nodes = p3d_add_node_to_list(N, C->nodes);
  //end path deform
  C->nnode ++;
  
  if (reorder && p3d_get_SORTING() == P3D_NB_CONNECT)
    p3d_order_node_list(C->dist_nodes);
}

/*! \brief Remove a node from a compco
 *
 *  \param  N: The node to remove.
 *  \param  C: The compco to modify.
 *  \param  reorder: Reorder the compco or not.
 */
void p3d_remove_node_compco(p3d_node * node, p3d_compco * compco, int reorder) {
  for(p3d_list_node* nodes = compco->nodes; nodes; nodes = nodes->next){
    if(nodes->N == node){//remove the node from the list Temporaly
      if(nodes->prev){
        nodes->prev->next = nodes->next;
      }
      if(nodes->next){
        nodes->next->prev = nodes->prev;
      }else{
        compco->nodes = nodes->prev;
      }
      MY_FREE(nodes, p3d_list_node, 1);
      break;
    }
  }
  for(p3d_list_node* nodes = compco->dist_nodes; nodes; nodes = nodes->next){
    if(nodes->N == node){//remove the node from the list Temporaly
      if(nodes->prev){
        nodes->prev->next = nodes->next;
      }else{
        compco->dist_nodes = nodes->next;
      }
      if(nodes->next){
        nodes->next->prev = nodes->prev;
      }
      MY_FREE(nodes, p3d_list_node, 1);
      break;
    }
  }
  compco->nnode--;
  if (reorder && p3d_get_SORTING() == P3D_NB_CONNECT)
    p3d_order_node_list(compco->dist_nodes);
}

/*--------------------------------------------------------------------------*/
/*! \brief Test if a Compco is forwardly linked to another Compco
 *
 *  \param  Source: The starting compco
 *  \param  Target: The tested comcpo.
 *  \return TRUE if the tested compco is linked to the tested compco
 *  (i.e. a path exists from the starting comcpo to the tested compco),
 *  FALSE otherwise.
 */
int p3d_compco_linked_to_compco(p3d_compco * Source, p3d_compco * Target) {
  p3d_list_compco * CLS;

  CLS = Source->canreach;
  while (CLS != NULL) {
    if (CLS->comp == Target) {
      /* The Target Compco is declared in the reachable compco list */
      return TRUE;
    }
    CLS = CLS->next;
  }
  return FALSE;
}

/*--------------------------------------------------------------------------*/
/*! \brief Add a compco to a list of comcpo
 *
 *  \param  Target: The compco list to modify
 *  \param  Add: The comcpo to add in the reachable list
 *  \return The modified reachable compco list
 */
p3d_list_compco * p3d_add_list_compco(p3d_list_compco * CLTarget, p3d_compco * CAdd) {
  p3d_list_compco * CLN = NULL; /* The new element in the comcpo list */

  /* Allocation */
  CLN = MY_ALLOC(p3d_list_compco, 1);

  /* Initialization and Declaration in the list */
  CLN->comp = CAdd;
  CLN->next = CLTarget;
  CLN->prev = NULL;
  if (CLTarget != NULL) {
    CLTarget->prev = CLN;
  }
  return CLN;
}

/*--------------------------------------------------------------------------*/
/*! \brief Add a compco to a reachable compco list
 *
 *  \param  Target: The compco whose reachable list will be modified
 *  \param  Add: The comcpo to add in the reachable list
 */
void p3d_add_compco_to_reachable_list(p3d_compco * Target, p3d_compco * Add) {
  if (Target != Add) {
    if (p3d_compco_linked_to_compco(Target, Add) == FALSE) {
      Target->canreach = p3d_add_list_compco(Target->canreach, Add);
      Target->ncanreach++;
    }
  }
}

/*--------------------------------------------------------------------------*/
/*! \brief Add a compco to a reachable compco list and also to all the compco
  which can already reach the Target comcpo
 *
 *  \param G: the current graph (only for scanning all the declared compco)
 *  \param  CLTarget: The compco list to modify
 *  \param  CAdd: The comcpo to add in the reachable list
 */
void p3d_add_compco_to_reachable_list_and_update_predecessors(p3d_graph * G, p3d_compco * CTarget, p3d_compco * CAdd) {
  p3d_compco * CS;
  p3d_add_compco_to_reachable_list(CTarget, CAdd);
  CS = G->comp;
  while (CS != NULL) {
    if (p3d_compco_linked_to_compco(CS, CTarget)) {
      p3d_add_compco_to_reachable_list(CS, CAdd);
    }
    CS = CS->suiv;
  }
}

/*--------------------------------------------------------------------------*/
/*! \brief Delete a reference to a compco in a reachable comcpo list
 *
 *  \param  CTarget: The compco list to modify
 *  \param  Add: The comcpo to add in the reachable list
 */
void p3d_del_compco_from_reachable_list(p3d_compco * CTarget, p3d_compco * CDel) {
  p3d_list_compco * CLS;
  p3d_list_compco * CLD;
  int ListReached;

  CLS = CTarget->canreach;
  if (CTarget->canreach->comp == CDel && CTarget->canreach->next == NULL) {
    MY_FREE(CTarget->canreach, p3d_list_compco, 1);
    CTarget->canreach = NULL;
    CTarget->last_canreach = NULL;
    CTarget->ncanreach --;
  } else
    if (CTarget->canreach->comp == CDel) {
      CLD = CTarget->canreach;
      CTarget->canreach = CTarget->canreach->next;
      MY_FREE(CLD, p3d_list_compco, 1);
      CTarget->ncanreach --;
    } else {
      ListReached = FALSE;
      while (CLS->next != NULL && ListReached == FALSE) {
        if (CLS->next->comp == CDel)
          ListReached = TRUE;
        else
          CLS = CLS->next;
      }
      if (ListReached == TRUE) {
        CLD = CLS->next;
        CLS->next = CLS->next->next;
        if (CLS->next == NULL)
          CTarget->last_canreach = CLS;
        MY_FREE(CLD, p3d_list_compco, 1);
        CTarget->ncanreach --;
      }
    }
}

/*--------------------------------------------------------------------------*/
/*! \brief Add a node to a node list, respecting the order given by the
 *  number of edges connected to each node in the list
 *
 *  \param  Add: The node toadd in the list
 *  \param  Target: The node list to modify
 */
p3d_list_node * p3d_add_node_to_list_node_nof_connex_order(p3d_node * NAdd, p3d_list_node * NLTarget) {
  p3d_list_node * New;
  p3d_list_node * Scanner;

  New = MY_ALLOC(p3d_list_node, 1);
  New->N = NAdd;

  /* The node list is empty */
  if (NLTarget == NULL) {
    NLTarget = New;
    NLTarget->next = NULL;
    NLTarget->prev = NULL;
    return NLTarget;
  }

  /* Scan the node list until the right number of connections is encountered */
  Scanner = NLTarget;
  while (Scanner->next != NULL) {
    if (Scanner->next->N->nedge < NAdd->nedge) {
      /* The node must be inserted here */
      New->next = Scanner->next;
      New->prev = Scanner;
      Scanner->next->prev = New;
      Scanner->next = New;
      return NLTarget;
    }
    Scanner = Scanner->next;
  }
  /* The end of the list has been reached. The node must be inserted at the end of the list */
  Scanner->next = New;
  New->next = NULL;
  New->prev = Scanner;
  return NLTarget;
}

/***************************************/
/* Fonction incluant un noeud dans les */
/* voisins d'un autre                  */
/* Fonction appele par p3d_create_edge */
/* In : le graphe, les noeuds          */
/* Out :                               */
/***************************************/

void p3d_add_node_neighbour(p3d_graph *G, p3d_node *N1, p3d_node *N2) {
  p3d_list_node *neighb1;


  neighb1 = MY_ALLOC(p3d_list_node, 1);

  neighb1->N = N2;
  neighb1->next = NULL;
  neighb1->prev = NULL;

  if (N1->nneighb == 0) {
    N1->neighb = neighb1;
    N1->last_neighb = neighb1;
  } else {
    N1->last_neighb->next = neighb1;
    neighb1->prev = N1->last_neighb;
    N1->last_neighb = N1->last_neighb->next;
  }
  N1->nneighb = N1->nneighb + 1;
}


/*****************************************************/
/* Fonction creant une seule arete (N1->N2)          */
/* In : le robot, le graphe, les noeuds, la longueur */
/* du chemin                                         */
/* Out :                                             */
/*****************************************************/
void p3d_create_one_edge(p3d_graph *G, p3d_node *Ni, p3d_node *Nf, double dist) {
  p3d_edge *e;
  p3d_list_edge *list_edge;
  p3d_list_compco * ListCompcoScan;
  int *ikSol = NULL;

  p3d_add_node_neighbour(G, Ni, Nf);

  if (G->oriented) {
    if (Nf->comp && Ni->comp) {
      p3d_add_compco_to_reachable_list_and_update_predecessors(G, Ni->comp, Nf->comp);

      ListCompcoScan = Nf->comp->canreach;
      while (ListCompcoScan != NULL) {
        p3d_add_compco_to_reachable_list_and_update_predecessors(G, Ni->comp, ListCompcoScan->comp);
        ListCompcoScan = ListCompcoScan->next;
      }
    }
  }

  Ni->nedge = Ni->nedge + 1;
  G->nedge = G->nedge + 1;

  e = MY_ALLOC(p3d_edge, 1);
  e->Ni = Ni;
  e->Nf = Nf;
  e->planner = p3d_local_get_planner();
  e->sens_edge = 1;
  if(p3d_get_ik_choice() != IK_NORMAL){
    p3d_get_non_sing_iksol(G->rob->cntrt_manager, Ni->iksol, Nf->iksol, &ikSol);
  }
  e->path = p3d_local_planner_multisol(G->rob, Ni->q, Nf->q, ikSol);
  if(dist < 0){
    e->longueur = e->path->length_lp;
  }else{
    e->longueur = dist;
  }
  //start path deform
  e->unvalid = 0;
  e->for_cycle = FALSE;
  e->unvalid = FALSE;
  //end path deform
  p3d_SetEdgeCost(e);

  if(p3d_get_costComputation()){
    e->cost = p3d_GetLpHriDistCost(G->rob, e->path);
  }
  //e->cost = fabs((Nf->cost - Ni->cost)*dist);
  list_edge = MY_ALLOC(p3d_list_edge,1);
  list_edge->E = e;
  list_edge->next = NULL;
  list_edge->prev = NULL;
  if (Ni->nedge == 1) {
    Ni->edges = list_edge;
    Ni->last_edge = list_edge;
  } else {
    Ni->last_edge->next = list_edge;
    list_edge->prev = Ni->last_edge;
    Ni->last_edge = Ni->last_edge->next;
  }

  list_edge = MY_ALLOC(p3d_list_edge, 1);
  list_edge->E = e;
  list_edge->next = NULL;
  list_edge->prev = NULL;
  if (G->nedge == 1) {
    G->edges = list_edge;
    G->last_edge = list_edge;
  } else {
    G->last_edge->next = list_edge;
    list_edge->prev = G->last_edge;
    G->last_edge = G->last_edge->next;
  }
}





/*****************************************************/
/* Fonction creant deux aretes (N1->N2) et (N2->N1)  */
/* In : le robot, le graphe, les noeuds, la longueur */
/* du chemin                                         */
/* Out :                                             */
/*****************************************************/
void p3d_create_edges(p3d_graph *G,
                      p3d_node *N1,
                      p3d_node *N2,
                      double dist) {
  p3d_create_one_edge(G, N1, N2, dist);
  p3d_create_one_edge(G, N2, N1, dist);
  N2->parent = N1;  // modif Juan (WARNING : suppose that N1 is always the parent node)
  if(p3d_get_costComputation()){
    N1->cost += N1->last_edge->E->cost;
    N2->cost += N2->last_edge->E->cost;
  }
}

p3d_node** create_node_tab(p3d_list_node *node_list, size_t nnode)
  {
    p3d_node** node_tab = MY_ALLOC(p3d_node*, nnode);
    int i = 0;
    for (; node_list != NULL; node_list = node_list->next, i++)
    { node_tab[i] = node_list->N; }
    return node_tab;
  }

void replace_node_list(p3d_node **node_tab, p3d_list_node *node_list, int nnode) {
  int i = 0;
  for (; node_list != NULL; node_list = node_list->next, i++)
  { node_list->N = node_tab[i]; }
}

/*! \brief Get the number of nodes in \a NL
 */
size_t p3d_list_node_size(p3d_list_node *NL) {
  size_t i = 0;
  for (; NL != NULL; i++, NL = NL->next);
  return i;
}

/*! \brief Replace the nodes in \a NL by the nodes in \a node_tab
 */
void array_to_p3d_list_node(p3d_node **node_array, p3d_list_node *NL) {
  for (int i = 0; NL != NULL; NL = NL->next, i++)
  { NL->N = node_array[i]; }
}

/*! \brief Create an array of nodes from the nodes in \a NL
 */
p3d_node** p3d_list_node_to_array(p3d_list_node *NL) {
  p3d_node** node_array = MY_ALLOC(p3d_node*, p3d_list_node_size(NL));
  for (int i = 0; NL != NULL; NL = NL->next, i++)
  { node_array[i] = NL->N; }
  return node_array;
}

/*! \brief Wrapper to the node comparison function, to be used in qsort
 */
int qsort_BestNode(const void* a, const void* b) {
  return BestNode(*(p3d_node**) a,*(p3d_node**) b);
}

/*! \brief Order a node list
 *
 *  The list is ordered according to the comparison function \a BestNode,
 *  whose ordering criteria is determined by the global variable \a SORTING.
 *  The sort algorithm is quicksort, complexity in O( n.log(n) ).
 *
 *  \param NL The node list to order
 *  \note The order of the p3d_list_node pointers is not modified. Thus
 *        the entry/exit points of the list don't change.
 */
void p3d_order_node_list(p3d_list_node *NL) {
  p3d_node **node_array = p3d_list_node_to_array(NL);
  qsort(node_array, p3d_list_node_size(NL), sizeof(p3d_node*), qsort_BestNode);
  array_to_p3d_list_node(node_array, NL);
  free(node_array);

  /* When retrieving statistics; */
  if(getStatStatus()) {
    XYZ_GRAPH->stat->planNeigCall++;
  }
}

// methode bulle O(n^2) (obsolete)
/* void p3d_order_node_list(p3d_list_node * NodeList) { */
/*   p3d_node * Current; */
/*   p3d_node * Next; */
/*   int NofExchange = 1; */
/*   p3d_list_node * ListStart; */

/*   /\* When retrieving statistics; */
/*      Commit Jim; date: 01/10/2008 *\/ */
/*   if(getStatStatus()){ */
/*     XYZ_GRAPH->stat->planNeigCall++; */
/*   } */

/*   ListStart = NodeList; */
/*   while (NofExchange > 0) { */
/*     NofExchange = 0; */
/*     NodeList = ListStart; */
/*     while (NodeList->next != NULL) { */
/*       Current = NodeList->N; */
/*       Next    = NodeList->next->N; */
/* //       If next node is better than current node : swap */
/*       if (BestNode(Current, Next) == 1) { */
/*         NodeList->N = Next; */
/*         NodeList->next->N = Current; */
/*         NofExchange ++; */
/*       } */
/*       NodeList = NodeList->next; */
/*     } */
/*   } */
/* } */



// /*--------------------------------------------------------------------------*/
// /*! \brief Order a list node with a given criteria (global variable)
//  *
//  *  \param  NodeList : The node list to order
//  *  \note The order of the pointers is not modified. It avoids to
//  *        modify the entry point of the list.
//  */
// 
// //methode bulle O(n^2)
// void p3d_order_node_list(p3d_list_node * NodeList) {
//   p3d_node * Current;
//   p3d_node * Next;
//   int NofExchange = 1;
//   p3d_list_node * ListStart;
// 
// /* When retrieving statistics; Commit Jim; date: 01/10/2008 */
//   if(getStatStatus()){
//     XYZ_GRAPH->stat->planNeigCall++;
//   }  ListStart = NodeList;
//   while (NofExchange > 0) {
//     NofExchange = 0;
//     NodeList = ListStart;
//     while (NodeList->next != NULL) {
//       Current = NodeList->N;
//       Next    = NodeList->next->N;
// //       If next node is better than current node : swap
//       if (BestNode(Current, Next) == 1) {
//         NodeList->N = Next;
//         NodeList->next->N = Current;
//         NofExchange ++;
//       }
//       NodeList = NodeList->next;
//     }
//   }
// }

//start path deform
void p3d_clear_comp(p3d_graph *G, p3d_compco* comp) {
  p3d_list_node *NLS, *NLD;  /* Node List Scanner, Node List pointer for Deletion */
  p3d_list_node *NeLS, *NeLD; /* Neighbour List Scanner, Neighbour List pointer for Deletion */
  p3d_list_edge *ELS, *ELD;  /* Edge List Scanner, Edge pointer for Deletion */

  NLS = G->nodes;
  while (NLS) {
    NLD = NLS;
    NLS = NLS->next;
    if (NLD->N->comp == comp) {
      if (NLD->prev != NULL) {
        NLD->prev->next = NLD->next;
      } else {
        G->nodes = NLD->next;
      }
      if (NLD->next != NULL) {
        NLD->next->prev = NLD->prev;
      }
      G->nnode--;
      if (G->last_node == NLD) {
        G->last_node = G->last_node->prev;
        if (G->last_node != NULL)
          G->last_node->next = NULL;
      }
    }
  }
  ELS = G->edges;
  while (ELS) {
    ELD = ELS;
    ELS = ELS->next;
    if (ELD->E->Nf->comp == comp) {
      if (ELD->prev != NULL) {
        ELD->prev->next = ELD->next;
      } else {
        G->edges = ELD->next;
      }
      if (ELD->next != NULL) {
        ELD->next->prev = ELD->prev;
      }
      G->nedge--;
      if (G->last_edge == ELD) {
        G->last_edge = G->last_edge->prev;
        if (G->last_edge != NULL)
          G->last_edge->next = NULL;
      }
    }
  }
  NLS = comp->nodes;
  while (NLS) {
    NLD = NLS;
    NLS = NLS->next;
    /* neighbour list desallocation */
    NeLS = NLD->N->neighb;
    while (NeLS) {
      NeLD = NeLS;
      NeLS = NeLS->next;
      MY_FREE(NeLD, p3d_list_node, 1);
      NeLD = NULL;
    }
    MY_FREE(NLD, p3d_list_node, 1);
    NLD = NULL;
  }
  comp->dist_nodes = NULL;
  comp->last_node = NULL;
  comp->nodes = NULL;
  comp->nnode = 0;
}

void p3d_compute_2D_Cspace_proj(p3d_rob* robotPt, int dof1, int dof2, int nstep) {
  p3d_jnt * jntPt1, *jntPt2;
  int j1, j2, i, j;
  double  vmin1, vmax1, vmin2, vmax2;
  double step1, step2, param1, param2;
  configPt qsave, q_test;
  FILE *oFile;

  jntPt1 = p3d_robot_dof_to_jnt(robotPt, dof1,  &j1);
  jntPt2 = p3d_robot_dof_to_jnt(robotPt, dof2,  &j2);
  p3d_jnt_get_dof_rand_bounds(jntPt1, j1, &vmin1, &vmax1);
  p3d_jnt_get_dof_rand_bounds(jntPt2, j2, &vmin2, &vmax2);
  step1 = (vmax1 - vmin1) / (double) nstep;
  step2 = (vmax2 - vmin2) / (double) nstep;
  qsave = p3d_get_robot_config(robotPt);
  q_test = p3d_copy_config(robotPt, qsave);
  oFile = open_file_to_save_plot(1);
  for (i = 0;i < nstep + 1;i++) {
    for (j = 0; j < nstep + 1; j++) {
      param1 = ((double) i) * step1;
      param2 = ((double) j) * step2;
      q_test[dof1] = vmin1 + param1;
      q_test[dof2] = vmin2 + param2;
      p3d_set_and_update_this_robot_conf(robotPt, q_test);
      if (!p3d_col_test()) {
        fprintf(oFile, "%f %f\n", param1, param2);
      }
    }
  }
  close_file_to_save_plot(oFile);
  p3d_destroy_config(robotPt, q_test);
  p3d_set_and_update_this_robot_conf(robotPt, qsave);
}

int p3d_is_node_useful_for_cycle(p3d_node* nodePt,
                                 p3d_node** node_to_connect1Pt,
                                 p3d_node** node_to_connect2Pt,
                                 p3d_graph *graphPt) {
  p3d_compco *current_compPt;
  dbl_list *list_vis_nodesPt, *list_connect_nodesPt, *list_unconnect_nodesPt;
  int found = FALSE;
  p3d_node* current_nodePt;

  current_compPt = graphPt->comp;
  while (current_compPt != NULL) {
    //trouver la liste des noeuds visibles a partir de la configuration de nodePt et ayant le meme ikSol.
    list_vis_nodesPt = p3d_list_nodes_visible_from_conf(graphPt, current_compPt->dist_nodes, nodePt->q);
    list_connect_nodesPt = dbl_list_pointer_init();

    if (p3d_is_visible_connectivity(graphPt, list_vis_nodesPt, nodePt, list_connect_nodesPt)) {
      dbl_list_destroy(list_vis_nodesPt);
      list_vis_nodesPt = NULL;
      dbl_list_destroy(list_connect_nodesPt);
      list_connect_nodesPt = NULL;
    } else {
      while (found == FALSE) {
        (*node_to_connect1Pt) = find_nearest_node(graphPt, nodePt, list_connect_nodesPt);
        if (((*node_to_connect1Pt) != NULL) && ((*node_to_connect1Pt)->only_for_cycle == TRUE)) {
          dbl_list_remove_data(list_connect_nodesPt, *node_to_connect1Pt);
          (*node_to_connect1Pt) = NULL;
        } else {
          found = TRUE;
        }
      }
      if ((*node_to_connect1Pt) != NULL) {
        list_unconnect_nodesPt = dbl_list_pointer_init();
        dbl_list_push(list_vis_nodesPt);
        dbl_list_goto_first(list_vis_nodesPt);
        while (dbl_list_more(list_vis_nodesPt)) {
          current_nodePt = DBL_LIST_DATA(p3d_node, list_vis_nodesPt);
          if ((!dbl_list_find_by_data(list_connect_nodesPt,
                                      current_nodePt, NULL)) && (current_nodePt->only_for_cycle == FALSE)) {
            dbl_list_add_link(list_unconnect_nodesPt, current_nodePt);
          }
          dbl_list_next(list_vis_nodesPt);
        }
        dbl_list_pop(list_vis_nodesPt);
        (*node_to_connect2Pt) =  find_nearest_node(graphPt, nodePt, list_unconnect_nodesPt);
        dbl_list_destroy(list_vis_nodesPt);
        dbl_list_destroy(list_connect_nodesPt);
        dbl_list_destroy(list_unconnect_nodesPt);
        if ((*node_to_connect2Pt) != NULL) {
          return TRUE;
        }

      } else {
        dbl_list_destroy(list_vis_nodesPt);
        dbl_list_destroy(list_connect_nodesPt);
      }
    }
    current_compPt =  current_compPt->suiv;
  }
  return FALSE;
}

int  p3d_test_reductibility(p3d_graph *G, p3d_node *N,
                            p3d_node * node1Pt, p3d_node * node2Pt) {
  p3d_traj * traj;
  p3d_localpath *localpath1Pt, *localpath2Pt;
  int res;

  localpath1Pt = p3d_local_planner(G->rob, node1Pt->q, N->q);
  p3d_set_localpath_ikSol(localpath1Pt, G->rob, node1Pt->iksol, node1Pt->isSingularity, N->iksol, N->isSingularity);
  localpath2Pt = p3d_local_planner(G->rob, N->q, node2Pt->q);
  p3d_set_localpath_ikSol(localpath2Pt, G->rob, N->iksol, N->isSingularity, node2Pt->iksol, node2Pt->isSingularity);
  localpath1Pt = concat_liste_localpath(localpath1Pt, localpath2Pt);
  traj = p3d_create_empty_trajectory(G->rob);
  traj->courbePt = localpath1Pt;
  traj->nlp = 2;
  traj->rob = G->rob;
  traj->range_param = p3d_compute_traj_rangeparam(traj);
  G->search_start = node1Pt;
  G->search_goal = node2Pt;
  res =  p3d_graph_many_search(G, p3d_heurist, p3d_valid, p3d_end, traj, DEFAULTGRAPH);
  p3d_del_traj(traj);   //mokhtar
  return res;
}
/*--------------------------------------------------------------------------*/
/*! \brief Find the nearest nodes from a specified node
 *
 *  \param graphPt : the graph which contain the
 *                        type of distance used
 *  \param nodePt     : the node from which the distances are computed
 *  \param list_nodes : the nodes tested for the distance computation
 *
 * \return The nearest node
 */
int p3d_is_node_in_list(p3d_list_node* list_nodes, p3d_node* nodePt) {
  p3d_list_node* cur_list_nodes;

  cur_list_nodes = list_nodes;
  while (cur_list_nodes) {
    if (cur_list_nodes->N->num == nodePt->num) {
      //PrintInfo(("node already in list detected\n"));
      return TRUE;

    }
    cur_list_nodes = cur_list_nodes->next;
  }
  return FALSE;
}

/**
 * p3d_CreateExtremalNodeFromConf
 * Create an extremal node from an extremal configuration
 * Function used to initialize a motion planning problem.
 * This function is only called when a node with the same conf
 * than the extremal conf has not been found in the graph.
 * Otherwise there is no nead to create a new node.
 * @param[In] GraphPt: Pointer on the Robot graph
 * @param[In] Config: The configuration from wich we want to
 * create a node
 * @return: the new created node
 */
p3d_node* p3d_CreateExtremalNodeFromConf(p3d_graph* GraphPt,
           configPt Config) {
  configPt CopyConf = p3d_copy_config(GraphPt->rob, Config);
  p3d_node* NewNodePt = NULL;

  p3d_set_robot_config(GraphPt->rob,Config);
  p3d_update_this_robot_pos_without_cntrt_and_obj(GraphPt->rob);
  NewNodePt  = p3d_APInode_make_multisol(GraphPt,CopyConf,NULL);
  p3d_insert_node(GraphPt,NewNodePt);
  p3d_create_compco(GraphPt,NewNodePt);
  NewNodePt->type = ISOLATED;
  return NewNodePt;
}



/*! \brief Find all visible nodes in the list node nodesPt from a specific configuration.
 *
 *  \param graphPt : the graph which contain nodes
 *  \param nodesPt : the nodes tested for the distance computation
 *  \param q       : the configuration
 *
 * \return The list of visible nodes.
 */
dbl_list* p3d_list_nodes_visible_from_conf(p3d_graph* graphPt, p3d_list_node* nodesPt, configPt q) {
  dbl_list *list_visiblePt = NULL;
  p3d_list_node * nearestNodes = nodesPt;
  p3d_node *current_nodePt = NULL;
  p3d_localpath *path = NULL;
  int unvalid = 0;

  if (nodesPt == NULL) {
    return NULL;
  }
  //initialisation de la liste de noeuds visibles.
  list_visiblePt = dbl_list_pointer_init();

  while (nearestNodes != NULL) {//tant qu'on a pas parcouru tout la liste.
    current_nodePt = nearestNodes->N;
    current_nodePt->visible = FALSE;

    //si c'est la configuration q on l'ajoute dans la liste visible.
    if(p3d_equal_config(graphPt->rob, q,current_nodePt->q )) {
      dbl_list_add_link(list_visiblePt,current_nodePt);
      current_nodePt->visible = TRUE;
    }else{
      //Verifier si la config courante est visible a partir de q.
      path = p3d_local_planner(graphPt->rob, current_nodePt->q, q);
      if (path == NULL) {
        PrintInfo(("Error : impossible de planifier\n"));
        return NULL;
      }
      unvalid = p3d_unvalid_localpath_test(graphPt->rob, path, &(graphPt->nb_test_coll));
      //si la config courante est visible, ajouter le noeud dans la liste des noeuds visibles.
      if (!unvalid) {
        dbl_list_add_link(list_visiblePt,current_nodePt);
        current_nodePt->visible = TRUE;
      }
    }
    nearestNodes = nearestNodes->next;
    path->destroy(graphPt->rob, path);
  }
//   if(path != NULL){
//     path->destroy(graphPt->rob, path);
//   }

  return list_visiblePt;
}

/*! \brief
 *
 *  \param graphPt : the graph which contain nodes.
 *  \param list_nodes : list of visible nodes from the nodePt's config.
 *  \param nodePt : pointer on the node.
 *  \param list_connect_nodes : list of connected nodes to nodePt
 *
 * \return
 */
int p3d_is_visible_connectivity(p3d_graph* graphPt, dbl_list* list_nodes,
                                p3d_node* nodePt, dbl_list* list_connect_nodes) {
  int test;

  //si list_connect_nodes n'a pas ete allouee.
  if (!list_connect_nodes) {
    PrintInfo(("Allocation Error for list_connect_nodes\n"));
    return FALSE;
  }

  test = p3d_test_visible_connectivity(graphPt, list_nodes, nodePt, list_connect_nodes, FALSE);
  if (test) {
    dbl_list_clear(list_connect_nodes);
    test = p3d_test_visible_connectivity(graphPt, list_nodes, nodePt, list_connect_nodes, TRUE);
  }
  return test;
}

/*--------------------------------------------------------------------------*/
/*! \brief Find the nearest nodes from a specified node
 *
 *  \param graphPt : the graph which contain the
 *                        type of distance used
 *  \param nodePt     : the node from which the distances are computed
 *  \param list_nodes : the nodes tested for the distance computation
 *
 * \return The nearest node
 */
p3d_node* find_nearest_node(p3d_graph* graphPt,
                            p3d_node* nodePt,
                            dbl_list* list_nodes) {

  p3d_node* node1Pt;
  double dist_max = P3D_HUGE;
  p3d_node* nearest_node = NULL;


  if (dbl_list_empty(list_nodes)) {
    return NULL;
  }

  dbl_list_push(list_nodes);
  dbl_list_goto_first(list_nodes);
  while (dbl_list_more(list_nodes)) {
    node1Pt =  DBL_LIST_DATA(p3d_node, list_nodes);
    if ((node1Pt != nodePt) && (p3d_compare_iksol(graphPt->rob->cntrt_manager, nodePt->iksol, node1Pt->iksol))) {
      node1Pt->dist_Nnew =  p3d_dist_config(graphPt->rob, nodePt->q,
                                            node1Pt->q);
      if (node1Pt->dist_Nnew < dist_max) {
        nearest_node =  node1Pt;
        dist_max = node1Pt->dist_Nnew;
      }
    }
    dbl_list_next(list_nodes);
  }
  dbl_list_pop(list_nodes);
  return nearest_node;
}

/**
 * p3d_TestConfInGraph
 * try to find in the graph if there
 * is a node with the same configuration
 * return the node if it exists
 * @param[In]: GraphPt: pointer to the robot graph.
 * @param[In]: Config: configuration for which we try to
 * find a corresponding node
 * @return: The corresponding node if it exists, NULL otherwise.
 */

p3d_node* p3d_TestConfInGraph(p3d_graph* GraphPt, configPt Config) {

  p3d_list_node * GraphNodeListPt = NULL;
  configPt TestConfig;

  if(GraphPt == NULL) {
    PrintInfo(("Warning: TestConfIngraph of a NULL Graph\n"));
    return NULL;
  }
  GraphNodeListPt = GraphPt->nodes;
  while(GraphNodeListPt != NULL) {
    TestConfig = GraphNodeListPt->N->q;
    if(p3d_equal_config(GraphPt->rob, Config , TestConfig)) {
      return GraphNodeListPt->N;
    }
    GraphNodeListPt = GraphNodeListPt->next;
  }
  return NULL;
}


/**
 * p3d_InitRun
 * Init some variables for a motion planning problem
 * (diffusion and PRM technics)
 * @param[In] GraphPt: Pointer on th robot graph
 * @param[In] Ns: The starting node of the problem
 * @param[In] Ng: The final node of the problem
 */
void p3d_InitRun(p3d_graph* GraphPt, p3d_node* Ns, p3d_node* Ng) {

#ifdef ENERGY
  int n_coldeg,icoldeg;
  double *coldeg_qs;
#endif
  GraphPt->search_start = Ns;
  if(ENV.getBool(Env::expandToGoal)== true) {
  GraphPt->search_goal = Ng;
  }
  GraphPt->search_done = FALSE;
  p3d_InitDynDomainParam(GraphPt,Ns,Ng);

  if(ENV.getBool(Env::isCostSpace) == true) {
    p3d_InitSpaceCostParam(GraphPt, Ns,Ng);
  }
  if (p3d_GetIsWeightedChoice()== TRUE) {
    p3d_init_root_weight(GraphPt);
  }
  if(ENV.getInt(Env::ExpansionNodeMethod) == RANDOM_IN_SHELL_METH ) {
    p3d_init_pb_variables(GraphPt);
  }
  if(Ns != NULL) {
    Ns->rankFromRoot = 1;
    Ns->comp->nbTests = 0;
  }
  if(Ng != NULL) {
    Ng->rankFromRoot = 1;
    Ng->comp->nbTests = 0;
  }
#ifdef ENERGY
  if(p3d_get_MOTION_PLANNER() ==  BIO_COLDEG_RRT) {
    n_coldeg = bio_get_num_collective_degrees();
    // init coldeg_q in Ns
    coldeg_qs = bio_alloc_coldeg_config(n_coldeg);
    for(icoldeg=0; icoldeg<n_coldeg; icoldeg++) {
      coldeg_qs[icoldeg] = 0.0;
    }
    bio_copy_coldeg_q_in_N(Ns,coldeg_qs,n_coldeg);
    bio_destroy_coldeg_config(coldeg_qs,n_coldeg);
    // WARNING : currently Ng is not considered !!!
  }
#endif
}


/*********************************************************/
/* functions to identify exhausted nodes                 */
/*********************************************************/

static int n_exhausted_nodes = 0;
static p3d_node **array_exhausted_nodes=NULL;

void p3d_reinit_array_exhausted_nodes(void)
{
  if(n_exhausted_nodes != 0) {
    MY_FREE(array_exhausted_nodes,p3d_node *,n_exhausted_nodes);
    n_exhausted_nodes = 0;
  }
  array_exhausted_nodes = NULL;
}



// This function writes array_exhausted_nodes.
// Exhausted nodes are clustered with a given resolution
void  p3d_identify_exhausted_nodes(void)
{
  p3d_rob *robPt;
  p3d_graph *GPt;
  p3d_compco* compPt;
  p3d_list_node *ListNode;
  p3d_node *N;
  int rrtmaxnfails;
  int i;
  int incluster;
  double d, dclust = 1.0; // WARNING : constant distance value for clustering
  int totnexhausted = 0;

  robPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  GPt = robPt->GRAPH;
  // WARNING: THIS FUNCTION ONLY WORKS WITH MONO-DIRECTIONAL SEARCH !!!
  compPt = GPt->search_start->comp;
  ListNode = compPt->dist_nodes;
  rrtmaxnfails = ENV.getInt(Env::MaxExpandNodeFail);

  while(ListNode != NULL) {
    N = ListNode->N;
    if(N->n_fail_extend == rrtmaxnfails) {
      totnexhausted++;
      // check if the node can be clustered with nodes in array_exhausted_nodes
      incluster = 0;
      for(i = 0; (i < n_exhausted_nodes) && (incluster == 0); i++) {
  // nodes are considered to be in the same cluster if the configurations distance is less than dclust
  d = p3d_dist_config(robPt, N->q,array_exhausted_nodes[i]->q);
  if(d < dclust)
    incluster = 1;
      }
      if(!incluster) {
  // alloc or realloc
  if(n_exhausted_nodes == 0)
    array_exhausted_nodes = MY_ALLOC(p3d_node *,1);
  else
    array_exhausted_nodes = MY_REALLOC(array_exhausted_nodes,p3d_node *,n_exhausted_nodes,n_exhausted_nodes+1);
  array_exhausted_nodes[n_exhausted_nodes] = N;
  n_exhausted_nodes++;
      }
    }
    ListNode = ListNode->next;
  }

  // print array_exhausted_nodes
  if(n_exhausted_nodes == 0) {
    printf("\n## No exhausted nodes in the RRT ##\n");
  }
  else {
    printf("\n## Exhausted nodes in the RRT : %d nodes in %d clusters ##\n",totnexhausted, n_exhausted_nodes);
    for(i = 0; i < n_exhausted_nodes; i++) {
      printf("%d\n",array_exhausted_nodes[i]->num);
    }
  }
}


/*****************************************************************************/
/* functions to identify farther nodes                                       */
/* NOTE : next functions should be moved to another file (p3d_planner_utils?)*/
/*****************************************************************************/

static int n_farther_nodes = 0;
static p3d_node **array_farther_nodes=NULL;
static int n_farther_node_clusters = 0;
static p3d_node **array_farther_node_clusters=NULL;

void p3d_reinit_array_farther_nodes(void)
{
  if(n_farther_nodes != 0) {
    MY_FREE(array_farther_nodes,p3d_node *,n_farther_nodes);
    n_farther_nodes = 0;
  }
  array_farther_nodes = NULL;

  if(n_farther_node_clusters != 0) {
    MY_FREE(array_farther_node_clusters,p3d_node *,n_farther_node_clusters);
    n_farther_node_clusters = 0;
  }
  array_farther_node_clusters = NULL;
}

// This function writes array_farther_nodes.
// Farther nodes are "clustered" with a given resolution
void  p3d_identify_farther_nodes(void)
{
  p3d_rob *robPt;
  p3d_graph *GPt;
  p3d_compco* compPt;
  p3d_list_node *ListNode;
  p3d_node *N=NULL, *refN;
  int i,j;
  int incluster;
  double d, dclust;
  int use_simplified_frame_dist = FALSE;
  int use_rmsd_from_init = FALSE;
  p3d_matrix4 *RefFramePt=NULL, *MobFramePt=NULL;
  int MAX_NUM_FARTHER_NODES;
  double *dist_array_farther_nodes;
  p3d_vector3 *init_jntcoordsPt;

  robPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  GPt = robPt->GRAPH;
  // WARNING: THIS FUNCTION ONLY WORKS WITH MONO-DIRECTIONAL SEARCH !!!
  compPt = GPt->search_start->comp;
  ListNode = compPt->dist_nodes;
  refN = GPt->search_start;

  // init parameter
  MAX_NUM_FARTHER_NODES = (int) ceil(((double)compPt->nnode)/10.0);

  //  if(p3d_get_frames_for_metric(robPt,&RefFramePt,&MobFramePt)) {
  if(p3d_GetRefAndMobFrames(robPt ,&RefFramePt, &MobFramePt) ) {
    use_simplified_frame_dist = TRUE;
    // WARNING : ONLY CONSIDERING FRAME POSITION WITHIN THE METRIC
    // p3d_set_weight_for_rotation_distance_metric(NULL);
    p3d_SetWeightRotaFrameMetric(NULL);
  }

  if(bio_get_init_jnt_coordinates(&init_jntcoordsPt)) {
    use_rmsd_from_init = TRUE;
  }

  // memory allocation
  array_farther_nodes = MY_ALLOC(p3d_node *,MAX_NUM_FARTHER_NODES);
  dist_array_farther_nodes = MY_ALLOC(double, MAX_NUM_FARTHER_NODES);
  for(i=0; i<MAX_NUM_FARTHER_NODES; i++) {
    array_farther_nodes[i] = NULL;
    dist_array_farther_nodes[i] = 0.0;
  }
  n_farther_nodes = MAX_NUM_FARTHER_NODES;

  // fill array_farther_nodes
  while(ListNode != NULL) {
    N = ListNode->N;
    if(use_simplified_frame_dist) {
      d = p3d_GetSe3DistanceFrames(robPt,refN->RelMobFrame,N->RelMobFrame);
    }
    else if(use_rmsd_from_init) {
      // set and update the "init" conf
      //printf("using RMSD in tree analysis\n");
      p3d_set_robot_config(robPt,N->q);
      p3d_update_this_robot_pos_without_cntrt_and_obj(robPt);
      d = bio_rmsd_to_init_jnt_coords(robPt,init_jntcoordsPt,N->q);
    }
    else {
/*       if(bio_ligand_mode) */
/*  //if(1) */
/*  d = bio_compute_ligand_dist(robPt,refN->q,N->q); */
/*       else  */
  d = p3d_dist_config(robPt,refN->q,N->q);
    }

    if (d > dist_array_farther_nodes[MAX_NUM_FARTHER_NODES-1]) {
      i = 0;
      while(d < dist_array_farther_nodes[i])
  i++;
      for(j=MAX_NUM_FARTHER_NODES-1; j>i; j--) {
  dist_array_farther_nodes[j] = dist_array_farther_nodes[j-1];
  array_farther_nodes[j] = array_farther_nodes[j-1];
      }
      dist_array_farther_nodes[i] = d;
      array_farther_nodes[i] = ListNode->N;
    }
    ListNode = ListNode->next;
  }

  // print array_farther_nodes
  if(n_farther_nodes == 0) {
    printf("\n## No farther nodes in the RRT ##\n");
  }
  else {
    printf("\n## Farther nodes in the RRT : %d nodes ##\n",
     n_farther_nodes);
    for(i = 0; i < n_farther_nodes; i++) {
      printf("%d\n",array_farther_nodes[i]->num);
    }
  }

  // "clustering" farther nodes
  // NOTE : a very simple "clustering" method is currently used
  //        it should be replaced by a real clustering !!!

  array_farther_node_clusters = MY_ALLOC(p3d_node *,1);
  array_farther_node_clusters[0] = array_farther_nodes[0];
  n_farther_node_clusters = 1;

  // init parameter
  dclust = dist_array_farther_nodes[0];

  for(i = 1; (i < n_farther_nodes) && (array_farther_nodes[i] != NULL); i++) {
    incluster = 0;
    for(j = 0; (j < n_farther_node_clusters) && (incluster == 0); j++) {
      // check if the node can be clustered with nodes in array_farther_node_clusters
      if(use_simplified_frame_dist) {
  d = p3d_GetSe3DistanceFrames(robPt,
             array_farther_node_clusters[j]->RelMobFrame,
             array_farther_nodes[i]->RelMobFrame);
      }
      else if(use_rmsd_from_init) {
  d = bio_rmsd_between_confs(robPt,
           array_farther_node_clusters[j]->q,
           array_farther_nodes[i]->q);
      }
      else {
/*  if(bio_ligand_mode) */
/*    //if(1) */
/*    d = bio_compute_ligand_dist(robPt, */
/*              array_farther_node_clusters[j]->q, */
/*              array_farther_nodes[i]->q); */
/*  else  */
    d = p3d_dist_config(robPt,
            array_farther_node_clusters[j]->q,
            array_farther_nodes[i]->q);
      }
      if(d < dclust)
  incluster = 1;
    }
    if(!incluster) {
      // alloc or realloc
      array_farther_node_clusters = MY_REALLOC(array_farther_node_clusters,
                 p3d_node *,
                 n_farther_node_clusters,
                 n_farther_node_clusters+1);
      array_farther_node_clusters[n_farther_node_clusters] = array_farther_nodes[i];
      n_farther_node_clusters++;
    }
  }

  // print array_farther_nodes
  if(n_farther_node_clusters == 0) {
    printf("\n## No farther nodes in the RRT ##\n");
  }
  else {
    printf("\n## Farther nodes in the RRT : %d nodes in %d clusters ##\n",
     n_farther_nodes, n_farther_node_clusters);
    for(i = 0; i < n_farther_node_clusters; i++) {
      printf("%d\n",array_farther_node_clusters[i]->num);
    }
  }
}

/*! \brief
 *
 *  \param graphPt : the graph which contain nodes.
 *  \param list_nodes : list of visible nodes from the nodePt's config.
 *  \param nodePt : pointer on the node.
 *  \param list_connect_nodes : list of connected node to nodePt.
 *  \param test_faces : flag to compute a face test or not.
 *
 * \return true if visible, false otherwise
 */
int p3d_test_visible_connectivity(p3d_graph* graphPt, dbl_list* list_nodes,
                                  p3d_node* nodePt, dbl_list* list_connect_nodes, int test_faces) {
  int is_visible = 0;
  p3d_node *current_nodePt = NULL;
  p3d_edge*  current_edge = NULL;
  dbl_list* list_propag_nodes = NULL, *next_list_propag_nodes = NULL;
  p3d_list_edge *edges = NULL;

  //rendre tous les edges du graph invisible.
  edges = graphPt->edges;
  while (edges != NULL) {
    current_edge = edges->E;
    current_edge->visible = FALSE;
    edges = edges->next;
  }
  //si la liste des noeuds visible est vide ou null.
  if ((list_nodes == NULL) || (dbl_list_empty(list_nodes))) {
    return TRUE; //Mokhtar pourquoi TRUE ?? si la liste de noeuds est nulle pas la peine de rerentrer dans la fonction
  }

  next_list_propag_nodes = dbl_list_pointer_init();

  // Add nodes connected by edges to the node
  // for which Visibility is tested
  edges = nodePt->edges;
  while (edges != NULL) {
    //si le noeud de l'autre coté de l'edge est dans la liste des noeuds
    //visible par le noeud nodePt
    if (dbl_list_find_by_data(list_nodes, edges->E->Nf, NULL)) {
      //ajouter le noeud courant a la liste des noeuds relie a nodePt
      dbl_list_add_link(list_connect_nodes, edges->E->Nf);
      dbl_list_add_link(next_list_propag_nodes, edges->E->Nf);
    }
    edges = edges->next;
  }

  //recupere le premier noeud de la liste des noeuds visibles.
  current_nodePt =  DBL_LIST_FIRST(p3d_node, list_nodes);
  dbl_list_add_link(list_connect_nodes, current_nodePt);
  dbl_list_add_link(next_list_propag_nodes, current_nodePt);
  while (!dbl_list_empty(next_list_propag_nodes)) {
    //dbl_list_clear(list_propag_nodes);
    list_propag_nodes = dbl_list_copy(next_list_propag_nodes);
    dbl_list_clear(next_list_propag_nodes);
    dbl_list_goto_first(list_propag_nodes);
    //parcourir toute la liste.
    while (dbl_list_more(list_propag_nodes)) {
      current_nodePt = DBL_LIST_DATA(p3d_node, list_propag_nodes);
      edges =  current_nodePt->edges;
      while (edges != NULL) {
        if (dbl_list_find_by_data(list_nodes, edges->E->Nf, NULL) &&
            (!dbl_list_find_by_data(list_connect_nodes, edges->E->Nf, NULL))) {
          if (!test_faces) {
            edges->E->visible = TRUE;
            dbl_list_add_link(next_list_propag_nodes, edges->E->Nf);
            dbl_list_add_link(list_connect_nodes, edges->E->Nf);
          } else {
            if (p3d_get_is_visibility_discreet() == FALSE) {
              if (p3d_test_visibility_edge(graphPt->rob, nodePt->q, edges->E->Ni->q, edges->E->Nf->q)) {
                edges->E->visible = TRUE;
                dbl_list_add_link(next_list_propag_nodes, edges->E->Nf);
                dbl_list_add_link(list_connect_nodes, edges->E->Nf);
              }
            } else {
              if (p3d_test_discreet_visibility_edge(graphPt, nodePt->q, edges->E->Ni->q, edges->E->Nf->q)) {
                edges->E->visible = TRUE;
                dbl_list_add_link(next_list_propag_nodes, edges->E->Nf);
                dbl_list_add_link(list_connect_nodes, edges->E->Nf);
              }
            }
          }
        }
        edges = edges->next;
      }
      dbl_list_next(list_propag_nodes);
    }
    dbl_list_destroy(list_propag_nodes);
  }
  dbl_list_destroy(next_list_propag_nodes);
  is_visible  = (dbl_list_count(list_connect_nodes)
                 == dbl_list_count(list_nodes));
  return is_visible;
}

/****************************************************************************/
/** \brief Discreet visibility test in path deform (We see if an edge is
visible from the current configuration).
 \param G the current graph
 \param q0 current robot config
 \param q_edge1 initial config of the edge tested
 \param q_edge2 final config of the edge tested
 \return True if the edge is visible from q0 False otherwise.
 */
/****************************************************************************/
int p3d_test_discreet_visibility_edge(p3d_graph* G, configPt q0,
                                      configPt q_edge1, configPt q_edge2) {
  int res;
  p3d_traj* traj1Pt, *traj2Pt;
  p3d_localpath *current_lp, * current_lp2;
  double p1, p2;

  //creation de la premiere trajectoire representant l'arete
  traj1Pt = p3d_create_empty_trajectory(G->rob);
  current_lp = p3d_local_planner(G->rob, q_edge1, q_edge2);
  traj1Pt->courbePt = current_lp;
  traj1Pt->nlp = p3d_compute_traj_nloc(traj1Pt);
  traj1Pt->rob = G->rob;
  traj1Pt->range_param = p3d_compute_traj_rangeparam(traj1Pt);

  //creation de la seconde trajectoire representant
  traj2Pt = p3d_create_empty_trajectory(G->rob);
  current_lp = p3d_local_planner(G->rob, q_edge1, q0);
  current_lp2 = p3d_local_planner(G->rob, q0, q_edge2);
  current_lp = concat_liste_localpath(current_lp, current_lp2);
  traj2Pt->courbePt = current_lp;
  traj2Pt->nlp = p3d_compute_traj_nloc(traj2Pt);
  traj2Pt->rob = G->rob;
  traj2Pt->range_param = p3d_compute_traj_rangeparam(traj2Pt);

  res =  p3d_is_projectable(G->rob, traj1Pt, traj2Pt, p3d_get_Nstep(), &p1, &p2);
  p3d_del_traj(traj1Pt);   //mokhtar
  p3d_del_traj(traj2Pt);   //mokhtar

  return res;
}
//end path deform

extern FILE* DifficultyFile;

/**
 * p3d_EvaluateExpandDiffic
 * Function attempting to evaluate the difficulty
 * to success one process (for exemple node expansion)
 * during a graph construction
 * @param[In]: CompPt: connect componant for which the
 * difficulty is considered
 * @param[In]: IsSuccess: The new result of the test.
 * Note: write the information in a plotable file. Start
 * after nbTests have been done and make an average over
 * the last nbTests results
 */
void p3d_EvaluateExpandDiffic(p3d_compco* CompPt, int IsSuccess) {
  int i;
  int nbSuccess = 0;
  double Difficulty;
  int nbSamples = nbAnaSuccesSamples;
  //if(IsSuccess == 1) {
    //    PrintInfo(("We need break !\n"));
    // }
  //  PrintInfo(("IsSuccess: %d\n", IsSuccess));
  if(CompPt->nbTests < nbSamples) {
    CompPt->AnaSuccessTab[CompPt->nbTests] = IsSuccess;
    (CompPt->nbTests)++;
  } else {
    for(i=0; i<(nbSamples-1); i++) {
      CompPt->AnaSuccessTab[i] = CompPt->AnaSuccessTab[i+1];
      nbSuccess += CompPt->AnaSuccessTab[i];
    }
    CompPt->AnaSuccessTab[nbSamples-1] = IsSuccess;
    nbSuccess +=  IsSuccess;
    Difficulty = 100*(1-((double)nbSuccess)/((double)nbSamples));
    //   PrintInfo(("Difficulty: %f\r", Difficulty));
    if(DifficultyFile){
      fprintf(DifficultyFile, "%f\n",Difficulty );
    }
  }
}

/**
 * p3d_ExtractBestTraj
 * Extract the best trajectory from the graph
 * that link the query nodes. Creates a trajectory
 * and print its main cost characteristics
 * @param[In] graphPt: the robot graph
 */
void p3d_ExtractBestTraj(p3d_graph* graphPt) {
  configPt configStart, configGoal;
  p3d_node  *Ns=NULL,*Ng=NULL;
  p3d_compco*  mainCompPt = NULL;
  //  p3d_graph* graphPt = NULL;
  double    tu,ts;
  int ConnectRes;
  p3d_traj* trajPt;
  p3d_rob* robotPt;
  ChronoOn();


  if(graphPt == NULL) {
    PrintInfo(("Warning: cannot extract the best path\
 as there is no current graph\n"));
    return;
  }
  robotPt = graphPt->rob;
 // Dense Roadmap creation
  //  graphPt = XYZ_GRAPH;

  // start and goal config creation
 configStart = p3d_copy_config(robotPt, robotPt->ROBOT_POS);
 configGoal = p3d_copy_config(robotPt, robotPt->ROBOT_GOTO);
 p3d_set_and_update_robot_conf(configGoal);
 if(p3d_col_test()) {
   (graphPt->nb_test_coll)++;
   PrintInfo(("Computation of approximated optimal cost stopped: \
Goal configuration in collision\n"));
   p3d_destroy_config(robotPt, configStart);
   p3d_destroy_config(robotPt, configGoal);
   ChronoOff();
   return;
 }

 // start and goal nodes creation and initialisation
 Ns = p3d_TestConfInGraph(graphPt, configStart);
 if(Ns == NULL) {
   Ns = p3d_CreateExtremalNodeFromConf(graphPt, configStart);
 } else {
   p3d_destroy_config(robotPt, configStart);
   configStart = NULL;
 }
 Ng = p3d_TestConfInGraph(graphPt, configGoal);
 if(Ng == NULL) {
   Ng = p3d_CreateExtremalNodeFromConf(graphPt,configGoal);
 } else {
   p3d_destroy_config(robotPt, configGoal);
   configGoal = NULL;
 }
 p3d_InitRun(graphPt,Ns, Ng);
 p3d_set_and_update_robot_conf(Ns->q);

 //search of the main constructed connected compoant
 mainCompPt = graphPt->comp;
 while(mainCompPt->nnode < 2) {
   mainCompPt = mainCompPt->suiv;
 }

 //connection of the the extremal nodes to the main componant
 ConnectRes = p3d_ConnectNodeToComp(graphPt,  Ns, mainCompPt);
 ConnectRes = (ConnectRes && p3d_ConnectNodeToComp(graphPt,  Ng, mainCompPt)) ;

 if(!ConnectRes) {
    PrintInfo(("No solution path found within the 2D grid\\n"));
  }
  else {
    // on construit la trajectoire entre les points etapes
   (graphPt->nb_test_coll)++;
   PrintInfo(("Connection of the extremal nodes succeeded\n"));
   PrintInfo(("%d\n",graphPt->ncomp));
   trajPt = p3d_graph_to_traj(robotPt);
    if(trajPt != NULL) {
      g3d_add_traj((char*)"Globalsearch",p3d_get_desc_number(P3D_TRAJ));
    } else {
       printf("Failed to extract a trajectory\n");
      //g3d_draw_allwin_active();
    }
//    g3d_draw_allwin_active();
  }

 //time info
 ChronoPrint("");
 ChronoTimes(&tu,&ts);
 graphPt->time = graphPt->time + tu;
 ChronoOff();

 //PrintInfo(("ConnectRes: %d\n",ConnectRes));
 if(ConnectRes == TRUE) {
   p3d_PrintTrajCost(graphPt, trajPt);
 }
}

/**
 * p3d_CreateDenseRoadmap
 * Create a (2^D)*(2^D) grid based graph.
 * It is the possible to find an optimal path
 * inside this grid graph.
 * @param[In] robotPt: the robot
 * @return: the created graph
 * Note that if a graph already exists
 * it does not return anything.
 */
int DichotomicFactor = 4;
p3d_graph*  p3d_CreateDenseRoadmap(p3d_rob *robotPt) {
  p3d_jnt * jntPt;
  int nbPart = pow(2,DichotomicFactor), i, j, k, count = 0;
  double vMinDof1, vMaxDof1, vMinDof2, vMaxDof2, currentCost;
  configPt config;
  int** IndexConstrSoluPt = NULL;
  p3d_graph* graphPt;
  p3d_node* newNodePt, *prevJNode = NULL;
  int indPrev;
  p3d_list_node* listCompNodePt;


  if(XYZ_GRAPH != NULL) {
    PrintInfo(("Warning: can not create a Compact \
Graph: a graph already exist.\n"));
    return NULL;
  }

  graphPt = p3d_create_graph();
  jntPt = robotPt->joints[1];
  p3d_jnt_get_dof_rand_bounds(jntPt, 0, &vMinDof1, &vMaxDof1);
  p3d_jnt_get_dof_rand_bounds(jntPt, 1, &vMinDof2, &vMaxDof2);
  if((ENV.getBool(Env::isCostSpace) == true) &&
     (GroundCostObj == NULL)){
    ZminEnv = P3D_HUGE;
    ZmaxEnv = 0.;

 }
  for(i = 0; i < nbPart; i++) {
    prevJNode = NULL;
    for(j = 0; j < nbPart; j++) {
      config  = p3d_alloc_config(graphPt->rob);
      p3d_shoot(robotPt, config, TRUE);
      config[6] = vMinDof1 + i*(vMaxDof1 - vMinDof1)/(nbPart-1) ;
      config[7] = vMinDof2 + j*(vMaxDof2 - vMinDof2)/(nbPart-1) ;
      p3d_get_iksol_vector(graphPt->rob->cntrt_manager,&IndexConstrSoluPt);

      //Node creation
      // WARNING: iksol does not work, a new vector containing the 1st solution of each constraint must be passed as argument
      newNodePt = p3d_APInode_make_multisol(graphPt, config, NULL);

      if(newNodePt == NULL) {
  PrintInfo(("Error: Failed to create a new node \
for an optimal cost search \n"));
  p3d_destroy_config(graphPt->rob,config );
  return NULL;
      }
      p3d_insert_node(graphPt, newNodePt);
      p3d_create_compco(graphPt,newNodePt);
      currentCost = p3d_GetConfigCost(graphPt->rob, newNodePt->q);
      p3d_SetNodeCost(graphPt, newNodePt, currentCost);

      if(ENV.getBool(Env::isCostSpace) == true) {
		  if(GroundCostObj == NULL) {
			ZminEnv = MIN(ZminEnv, currentCost);
			ZmaxEnv = MAX(ZminEnv, currentCost);
		  }
      }
      if(prevJNode == NULL) {
  prevJNode = newNodePt;
      } else {
  // edge linking previous node in the j direction
  p3d_LinkNodesMergeComp(graphPt, prevJNode,newNodePt);
  prevJNode = newNodePt;
      }
      // count is the current counter of the node and
      // indPrev is the indice of the previous node in the i direction
      indPrev = count - nbPart;

  // edge linking previous node - 1 (diagonal) in the i direction
      if( ((indPrev-1) >= 0)&& ((indPrev-1)%nbPart !=(nbPart -1))) {
  listCompNodePt =  graphPt->nodes;
  for(k = 0; k< (indPrev-1); k++) {
    listCompNodePt = listCompNodePt->next;
  }
  p3d_LinkNodesMergeComp(graphPt,listCompNodePt->N ,newNodePt);
      }
  // edge linking previous node in the i direction
      if(indPrev >= 0) {
  listCompNodePt =  graphPt->nodes;
  for(k = 0; k< indPrev; k++) {
    listCompNodePt = listCompNodePt->next;
  }
  p3d_LinkNodesMergeComp(graphPt,listCompNodePt->N ,newNodePt);
      }

  // edge linking previous node +1 in the i direction
      if( ((indPrev+1) >= 0) && (((indPrev+1)%nbPart !=0))) {
  listCompNodePt =  graphPt->nodes;
  for(k = 0; k< (indPrev +1); k++) {
    listCompNodePt = listCompNodePt->next;
  }
  p3d_LinkNodesMergeComp(graphPt,listCompNodePt->N ,newNodePt);
      }
      count++;
    }
  }
//  g3d_draw_allwin_active();
  PrintInfo(("Build of dense roadmap done\n"));
  return graphPt;
}

/*
 * IsSmallDistNodeInGraph
 * TODO
 * @return: TRUE if the there is a commun parent in the graph between
 * N1 and N2 at a maximal distance of maxLevel for both nodes
 */

int p3d_IsSmallDistInGraph(p3d_graph* G, p3d_node* N1, p3d_node* N2,
         int maxLevel, double Step) {
  p3d_node* Ns, *Ng;
  p3d_traj* trajPt;
  int  savedParam;
  if((N1== NULL) ||(N2== NULL) ) {
    PrintInfo(("Error in DistInGraph test: one of the nodes is NULL\n"));
    return FALSE;
  }
  if(N1 == N2) return TRUE;

  Ns = G->search_start;
  Ng = G->search_goal;

  savedParam = ENV.getBool(Env::expandToGoal);
  ENV.setBool(Env::expandToGoal,true);
  p3d_InitRun(G,N1, N2);

  trajPt = p3d_graph_to_traj(G->rob);
  if(trajPt == NULL) {
    PrintInfo(("Warning: failed to extract a traj linking the nodes\n"));
    p3d_InitRun(G,Ns, Ng);
    ENV.setBool(Env::expandToGoal,savedParam);
    return TRUE;
  }
  p3d_destroy_traj(G->rob, trajPt);
  ENV.setBool(Env::expandToGoal,savedParam);
  p3d_InitRun(G,Ns, Ng);

  if ( ENV.getBool(Env::isCostSpace) &&
      (p3d_GetCostMethodChoice() == TRANSITION_RRT_CYCLE)) {
    return 0;
  }
  return (trajPt->range_param < maxLevel*Step);
}

p3d_node * p3d_getNodeInGraphByNum(p3d_graph* graph, int nodeId){
  p3d_list_node * listNode = graph->nodes;
  for(; listNode; listNode = listNode->next){
    if(listNode->N->num == nodeId){
      return listNode->N;
    }
  }
  return NULL;
}

/**
 * @brief Separate the graph into part due to a unvalid edges. Mode than two connected componants can be generated.
 * @param graph The graph
 */
void p3d_separate_graph_for_unvalid_edges(p3d_graph* graph){
  int checkedNodes[graph->nnode];
  for(int i = 0; i < graph->nnode; i++){
    checkedNodes[i] = 0;
  }
//find all unvalid edges
  for(p3d_list_edge* lEdge = graph->edges; lEdge; lEdge = lEdge->next){
    //if the edge is unvalid separate the graph
    if(lEdge->E->unvalid == TRUE){
      p3d_node* startNode = lEdge->E->Ni;
      p3d_node* endNode = lEdge->E->Nf;
      if(checkedNodes[startNode->num - 1] == 1 && checkedNodes[endNode->num - 1] == 1){
        //the reverse edge
        continue;
      }else if(checkedNodes[startNode->num - 1] == 0 && checkedNodes[endNode->num - 1] == 1){
        //the end node is already checked keep it in its compco
        startNode = lEdge->E->Nf;
        endNode = lEdge->E->Ni;
      }else if(checkedNodes[startNode->num - 1] == 0 && checkedNodes[endNode->num - 1] == 0 && startNode->numcomp > endNode->numcomp){
        //keep the smallest compcoNum
        startNode = lEdge->E->Nf;
        endNode = lEdge->E->Ni;
      }
      p3d_remove_node_compco(endNode, startNode->comp, FALSE);
      checkedNodes[endNode->num - 1] = 1;
      p3d_create_compco(graph, endNode);
      p3d_compco* newComp = endNode->comp;
      DfsDefaultGraph dfs;
      p3d_list_node* nodes = (p3d_list_node*)dfs.p3d_dfs(graph, endNode);
      //for all nodes in the subGraph except edge->Ni, put them in the new compco.
      for(;nodes ; nodes = nodes->next){
        if(nodes->N->numcomp != newComp->num){
          p3d_add_node_compco(nodes->N, newComp, FALSE);
          p3d_remove_node_compco(nodes->N, startNode->comp, FALSE);
          checkedNodes[nodes->N->num - 1] = 1;
        }
      }
      checkedNodes[startNode->num - 1] = 1;
      newComp->last_node = newComp->dist_nodes;
      startNode->comp->last_node = startNode->comp->dist_nodes;
      //reorder the two comp
      if (p3d_get_SORTING() == P3D_NB_CONNECT){
        p3d_order_node_list(newComp->dist_nodes);
        p3d_order_node_list(startNode->comp->dist_nodes);
      }
    }
  }
}

/**
 * @brief This function set the unvalid flag in the edge structure to true.
    This function will be generally used for dynamic planning when a edge is detected in collision
    The unvalid flag is set to true.
 * @param graph The graph
 * @param edge The edge to unvalid
 */
void p3d_unvalid_edge(p3d_graph* graph, p3d_edge* edge){
  edge->unvalid = TRUE;
  if(!graph->oriented){//unvalid the other edge too
    for(p3d_list_edge* lEdge = edge->Nf->edges; lEdge; lEdge = lEdge->next){
      if(lEdge->E->Nf == edge->Ni){
        lEdge->E->unvalid = TRUE;
        break;
      }
    }
  }
}

/**
 * @brief This function set the unvalid flag in the edge structure to false.
    This function will be generally used for dynamic planning
 * @param graph The graph
 * @param edge The edge to set valid
 */
void p3d_valid_edge(p3d_graph* graph, p3d_edge* edge){
  edge->unvalid = FALSE;
  if(!graph->oriented){//unvalid the other edge too
    for(p3d_list_edge* lEdge = edge->Nf->edges; lEdge; lEdge = lEdge->next){
      if(lEdge->E->Nf == edge->Ni){
        lEdge->E->unvalid = FALSE;
        break;
      }
    }
  }
}
