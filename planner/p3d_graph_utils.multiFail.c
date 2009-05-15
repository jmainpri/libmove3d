#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"

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
  N->list_closed_flex_sc = NULL; // modif ljaillet (")
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
    if(ELD->E->path != NULL){
      ELD->E->path->destroy(G->rob, ELD->E->path);
    }
    MY_FREE(ELD->E, p3d_edge, 1);
    MY_FREE(ELD, p3d_list_edge, 1);
  }
  G->nedge = 0;

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
 */
void p3d_add_node_compco(p3d_node * N, p3d_compco * C) {
  N->comp = C;
  N->numcomp = C->num;
  C->dist_nodes = p3d_add_node_to_list(N, C->dist_nodes);
  //start path deform
  //C->nodes = p3d_add_node_to_list(N, C->nodes);
  //end path deform
  C->nnode ++;
  if (p3d_get_SORTING() == P3D_NB_CONNECT)
    p3d_order_node_list(C->dist_nodes);
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
  e->longueur = dist;
  e->sens_edge = 1;
  p3d_get_non_sing_iksol(G->rob->cntrt_manager, Ni->iksol, Nf->iksol, &ikSol);
  e->path = p3d_local_planner_multisol(G->rob, Ni->q, Nf->q, ikSol);
  //start path deform
  e->unvalid = 0;
  e->for_cycle = FALSE;
  e->unvalid = FALSE;
  //end path deform

  list_edge = MY_ALLOC(p3d_list_edge, 1);
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
/* TODO : parameter robotPt seems to be useless      */
/*****************************************************/
void p3d_create_edges(p3d_graph *G,
                      p3d_node *N1,
                      p3d_node *N2,
                      double dist) {
  p3d_create_one_edge(G, N1, N2, dist);
  p3d_create_one_edge(G, N2, N1, dist);
  N2->parent = N1;  // modif Juan (WARNING : suppose that N1 is always the parent node)
}

/*--------------------------------------------------------------------------*/
/*! \brief Order a list node with a given criteria (global variable)
 *
 *  \param  NodeList : The node list to order
 *  \note The order of the pointers is not modified. It avoids to
 *        modify the entry point of the list.
 */

//methode bulle O(n^2)
void p3d_order_node_list(p3d_list_node * NodeList) {
  p3d_node * Current;
  p3d_node * Next;
  int NofExchange = 1;
  p3d_list_node * ListStart;

  ListStart = NodeList;
  while (NofExchange > 0) {
    NofExchange = 0;
    NodeList = ListStart;
    while (NodeList->next != NULL) {
      Current = NodeList->N;
      Next    = NodeList->next->N;
//       If next node is better than current node : swap
      if (BestNode(Current, Next) == 1) {
        NodeList->N = Next;
        NodeList->next->N = Current;
        NofExchange ++;
      }
      NodeList = NodeList->next;
    }
  }
}

// void p3d_order_node_list_multisol(p3d_graph *G, p3d_list_node * NodeList, p3d_node * N) {
//   p3d_node * Current;
//   p3d_node * Next;
//   int NofExchange = 1;
//   p3d_list_node * ListStart;
// 
//   ListStart = NodeList;
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
//methode tri rapide O(n*log(n));
// static p3d_list_node * p3d_part(p3d_list_node * nodeList, p3d_list_node * min, p3d_list_node * max) {
//         p3d_node * pivot = min->N;
//         p3d_list_node *i = min->next, *j = max;
//         p3d_list_node *temp;
//         while(1) {
//           while(j != NULL && BestNode(j->N, pivot) == 1){//J->N > pivot
//             j = j->prev;
//           }
//           /*do
//                   j--;
//           while(tableau[j] > pivot);*/
//           while (i!= NULL &&  BestNode(i->N, pivot) == -1){
//             i = i->next;
//           }
//           /*do
//                   i++;
//           while(tableau[i] < pivot);*/
//           if(BestNode(j->N,i->N) == 1) {//j>i
//             temp->next = i->next;
//             temp->prev = i->prev;
//             i->next = j->next;
//             i->prev = j->prev;
//             j->next = temp->next;
//             j->prev = temp->prev;
//           }
//           else return j;
//         }
//         return j;
// }
// void p3d_order_node_list2(p3d_list_node * nodeList, p3d_list_node * min/*p*/, p3d_list_node * max/*r*/) {
//         p3d_list_node * mid = NULL;
//         if(BestNode(min,max) == 1) {
//                 mid = p3d_part(nodeList, min, max);
//                 p3d_order_node_list2(nodeList, min, mid);
//                 p3d_order_node_list2(nodeList, mid->next, max);
//         }
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
    list_vis_nodesPt = p3d_list_nodes_visible(graphPt, current_compPt->dist_nodes, nodePt);
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
  traj->nloc = 2;
  traj->nlp = 2;
  traj->rob = G->rob;
  traj->range_param = p3d_compute_traj_rangeparam(traj);
  G->search_start = node1Pt;
  G->search_goal = node2Pt;
  res =  p3d_graph_many_search(G, p3d_heurist, p3d_valid, p3d_end, traj);
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

/*! \brief Find all visible nodes in the list node nodesPt from a specific Node.
 *
 *  \param graphPt : the graph which contain nodes
 *  \param nodesPt : the nodes tested for the distance computation
 *  \param N       : the node
 *
 * \return The list of visible nodes.
 */
dbl_list* p3d_list_nodes_visible(p3d_graph* graphPt, p3d_list_node* nodesPt, p3d_node* N) {
  dbl_list *list_visiblePt = NULL;
  p3d_list_node * nearestNodes = nodesPt;
  p3d_node *current_nodePt = NULL;
  p3d_localpath *path = NULL;
  configPt q;
  int unvalid = 0;

  q = p3d_copy_config(graphPt->rob, N->q);
  if (nodesPt == NULL) {
    return NULL;
  }
  //initialisation de la liste de noeuds visibles.
  list_visiblePt = dbl_list_pointer_init();

  while (nearestNodes != NULL) {  //tant qu'on a pas parcouru tout la liste.
    current_nodePt = nearestNodes->N;
    current_nodePt->visible = FALSE;

    //si c'est la configuration q on l'ajoute dans la liste visible.
    if (p3d_equal_config(graphPt->rob, q, current_nodePt->q)) {
      dbl_list_add_link(list_visiblePt, current_nodePt);
      current_nodePt->visible = TRUE;
    } else {
      //si les noeuds ont le meme ikSol
      if(p3d_compare_iksol(graphPt->rob->cntrt_manager, current_nodePt->iksol, N->iksol)){
        //Verifier si la config courante est visible a partir de q.
        path = p3d_local_planner(graphPt->rob, current_nodePt->q, q);
        p3d_set_localpath_ikSol(path, graphPt->rob, current_nodePt->iksol, current_nodePt->isSingularity, N->iksol, N->isSingularity);
        if (path == NULL) {
          PrintInfo(("Error : impossible de planifier\n"));
          return NULL;
        }
        unvalid = p3d_unvalid_localpath_test(graphPt->rob, path, & (graphPt->nb_test_coll));
        //si la config courante est visible, ajouter le noeud dans la liste des noeuds visibles.
        if (!unvalid) {
          dbl_list_add_link(list_visiblePt, current_nodePt);
          current_nodePt->visible = TRUE;
        }
      }
    }
    nearestNodes = nearestNodes->next;
    path->destroy(graphPt->rob, path);
  }
  p3d_destroy_config(graphPt->rob, q);
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
  traj1Pt->nloc = p3d_compute_traj_nloc(traj1Pt);
  traj1Pt->nlp  = p3d_compute_traj_nloc(traj1Pt);
  traj1Pt->rob = G->rob;
  traj1Pt->range_param = p3d_compute_traj_rangeparam(traj1Pt);

  //creation de la seconde trajectoire representant
  traj2Pt = p3d_create_empty_trajectory(G->rob);
  current_lp = p3d_local_planner(G->rob, q_edge1, q0);
  current_lp2 = p3d_local_planner(G->rob, q0, q_edge2);
  current_lp = concat_liste_localpath(current_lp, current_lp2);
  traj2Pt->courbePt = current_lp;
  traj2Pt->nloc = p3d_compute_traj_nloc(traj2Pt);
  traj2Pt->nlp  = p3d_compute_traj_nloc(traj2Pt);
  traj2Pt->rob = G->rob;
  traj2Pt->range_param = p3d_compute_traj_rangeparam(traj2Pt);

  res =  p3d_is_projectable(G->rob, traj1Pt, traj2Pt, p3d_get_Nstep(), &p1, &p2);
  p3d_del_traj(traj1Pt);   //mokhtar
  p3d_del_traj(traj2Pt);   //mokhtar

  return res;
}
//end path deform

int p3d_fail_stop(int *fail, int ikLayer, int init){
  static int totalFail = 0;
  static int *previousFail = NULL;
  int nbIksol = p3d_get_nb_ikSol(XYZ_ROBOT->cntrt_manager);
  int maxFail = p3d_get_NB_TRY() * nbIksol, i = 0;

  switch(init){//init
    case 1:{
      totalFail = 0;
      MY_FREE(previousFail, int, nbIksol);
      previousFail = MY_ALLOC(int, nbIksol);
      for(i = 0; i < nbIksol; i++){
        previousFail[i] = fail[i];
        totalFail += fail[i];
      }
      break;
    }
    case -1:{//destroy
      MY_FREE(previousFail, int, nbIksol);
      break;
    }
    default:{
      totalFail += (fail[ikLayer] - previousFail[ikLayer]);
      previousFail[ikLayer] = fail[ikLayer];
    }
  }
  if (totalFail >= maxFail){
    return TRUE;
  }

  return FALSE;
}
