#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Move3d-pkg.h"
#include "Bio-pkg.h"
#ifdef ENERGY
#include "../bio/BioEnergy/include/Energy-pkg.h"
#endif
#define DEBUG(x) x

pp3d_graph XYZ_GRAPH = NULL;

static void save_infos_in_file(p3d_graph *G, int sol);
static void p3d_specificPrintAverage(double * arraytimes, int nfail, int sumnnodes, int sumnsamples, int sumncallsCD, int sumncallsLP);

extern int singularityCheck;
extern int GlobalOrdering;
extern void* GroundCostObj;

/***********************************************/
/* Fonction initialisant et allouant le graphe */
/* courant                                     */
/* In :                                        */
/* Out : le graphe                             */
/***********************************************/
p3d_graph * p3d_create_graph(void) {
  p3d_graph * Graph;
  p3d_rob   * Robot;

  Robot = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);

  Graph      = p3d_allocinit_graph();
  Graph->env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  Graph->rob = Robot;

  if (Robot != NULL) {
    if (Robot->GRAPH != NULL) {
      p3d_del_graph(Robot->GRAPH);
    }
    Robot->GRAPH = Graph;
    XYZ_GRAPH    = Graph;
  }
#ifdef MULTIGRAPH
  Graph->mgTime = 0.0;
#endif
#ifdef DPG
  Graph->grid = NULL;
#endif

  if (STAT) {
    Graph->stat = createStat();
  } else {
    Graph->stat = NULL;
  }

  if (STAT) {
    Graph->stat = createStat();
  } else {
    Graph->stat = NULL;
  }

  return Graph;
}

/*******************************************/
/* Fonction de creation d'un noeud du graphe */
/* In : le graphe,                         */
/* Out : le noeud                          */
/*******************************************/
p3d_node * p3d_create_node(p3d_graph * G) {
  p3d_node * nodePt;
  double dmax = p3d_get_env_dmax();
  nodePt = p3d_allocinit_node();
//   nodePt->num = G->nnode + 1;
  nodePt->radius = p3d_GetLambda() * dmax;
  nodePt->boundary = FALSE;
  nodePt->pinpointed = FALSE;
  nodePt->isSingularity = FALSE;
#ifdef MULTIGRAPH
  nodePt->mergeState = 0; //The merge state of the node: 0 None, 1 trajectory, 2 All
  nodePt->needMgCycle = FALSE;
#endif
//  nodePt->IsDiscarded = FALSE;
//  nodePt->localEdgeGrad = NULL;
//nodePt->localGrad = 0.;
  return(nodePt);
}


/*--------------------------------------------------------------------------*/
/*! \brief Repport the insertion of a node in a graph an repport
*         in the linked graph
*
*  \param  G: The graph description.
*  \param  nodePt:  The node inserted.
*/
void p3d_insert_node(p3d_graph *G, p3d_node *nodePt) {
  p3d_insert_node_in_graph(G, nodePt);
}

/*--------------------------------------------------------------------------*/
/*! \brief Detect the need of merging some compcos.
*
*  \param  G: The graph description.
*  \note This function might be recursive
*/
void p3d_merge_check(p3d_graph * G) {
  p3d_compco * CompScan, *CompScan2;
  int NEED_MERGE = FALSE;
  p3d_compco * c1, * c2;
  /* Scan all the compco */
  CompScan = G->comp;
  while (CompScan != NULL && NEED_MERGE == FALSE) {
    /* For each other compco of th egraph */
    CompScan2 = G->comp;
    while (CompScan2 != NULL && NEED_MERGE == FALSE) {
      if (CompScan != CompScan2) {
        /* If a forward and a backward path exists between the scanned compcos */
        if (p3d_compco_linked_to_compco(CompScan, CompScan2) == TRUE) {
          if (p3d_compco_linked_to_compco(CompScan2, CompScan) == TRUE) {
            /* A merge should be done */
            c1 = CompScan;
            c2 = CompScan2;
            NEED_MERGE = TRUE;
            break;
          }
        }
      }
      CompScan2 = CompScan2->suiv;
    }
    CompScan = CompScan->suiv;
  }
  /* Do the merge if necessary */
  if (NEED_MERGE == TRUE) {
    if (c1->num < c2->num)
      p3d_merge_comp(G, c1, &c2);
    else
      p3d_merge_comp(G, c2, &c1);
  }
}

/****************************************************/
/* Fonction fusionnant deux composantes connexes et */
/* les mettant a jour dans le graphe                */
/* In : le graphe, les deux composantes             */
/* Out :                                            */
/****************************************************/
void p3d_merge_comp(p3d_graph *G,
                    p3d_compco *c1,
                    p3d_compco **c2Pt) {
//int nnode1,nnode2;
  p3d_list_node *list_node;
  p3d_compco *c2 = *c2Pt;
  p3d_list_compco * ListCompcoScan;

//nnode1 = c1->nnode;
//nnode2 = c2->nnode;

  /* The nodes of C2 are now in C1 */
  list_node = c2->dist_nodes;
  while (list_node) {
    p3d_add_node_compco(list_node->N, c1);
    list_node = list_node->next;
  }
  /* All the compcos that can reach C2 can now reach C1 */
  if (G->oriented) {
    ListCompcoScan = c2->canreach;
    while (ListCompcoScan != NULL) {
      p3d_add_compco_to_reachable_list_and_update_predecessors(G, c1, ListCompcoScan->comp);
      ListCompcoScan = ListCompcoScan->next;
    }
  }
  /* C2 is deleted from the graph */
  p3d_remove_compco(G, c2);
  if (G->oriented) {
    p3d_merge_check(G);
  }
}

/*********************************************/
/* Fonction qui essaye de connecter un noeud */
/* a une composante connexe                  */
/* In : le graphe, le noeud, la composante   */
/* connexe                                   */
/* Out : relies ou non ?                     */
/*********************************************/
int p3d_link_node_comp(p3d_graph *G, p3d_node *N, p3d_compco **compPt) {
  p3d_compco * TargetComp = *compPt;
  double dist = 0.;
  p3d_node * Nc = NULL;
  p3d_list_node *list_node;
  int ValidForward, ValidBackward;

  /* If the criteria for choosing the best node in the target compco is */
  /* the distance, then node lists must be ordered */
  if (p3d_get_SORTING() == P3D_DIST_NODE) {
    list_node = TargetComp->dist_nodes;
    while (list_node != NULL) {
      list_node->N->dist_Nnew = p3d_APInode_dist(G, N, list_node->N);
      list_node = list_node->next;
    }
    p3d_order_node_list(TargetComp->dist_nodes);
  }

  /* Test the existence of a valid forward and backward path */
  list_node = TargetComp->dist_nodes;

  ValidBackward = ValidForward = FALSE;

  while (list_node != NULL) {

    Nc = list_node->N;

    if (p3d_get_SORTING() == P3D_DIST_NODE) {
      if ((Nc->dist_Nnew > p3d_get_DMAX()) && (p3d_get_DMAX() > 0.)) {
        return (FALSE);
      }
    }
    /* Oriented case, forward and backward paths must be separately tested */
    if (G->oriented) {
      if (ValidForward == FALSE) {
        if (p3d_APInode_linked(G, N, Nc, &dist)) {
          /* A forward path is found */
          p3d_create_one_edge(G, N, Nc, dist);
          ValidForward = TRUE;
        }
      }

      if (ValidBackward == FALSE) {
        if (p3d_APInode_linked(G, Nc, N, &dist)) {
          /* A bacward path is found */
          p3d_create_one_edge(G, Nc, N, dist);
          ValidBackward = TRUE;
        }
      }

      if (ValidBackward && ValidForward) {
        if (!N->comp) {
          /* A valid forward and backward path exist, and the node is still in none compco */
          /* so the tested compco will now include the new node */
          p3d_add_node_compco(N, TargetComp);
        } else {
          /* A valid forward and backward path exist, and the node is already included in a compco */
          /* so the tested compco and the compco of the new node must merge */
          if (TargetComp->num > N->numcomp) {
            p3d_merge_comp(G, N->comp, compPt);
            *compPt = NULL;
          } else {
            p3d_merge_comp(G, TargetComp, &(N->comp));
          }
        }
        return TRUE;
      }
    }
    /* Non - oriented case, If the forward path is valid, the backward one is also valid. */
    else {
      if (p3d_APInode_linked(G, N, Nc, &dist)) {
        p3d_create_edges(G, N, Nc, dist);
        /* If the node is still not included in a compco, it will be absorbed in the tested compco*/
        if (N->comp == NULL) {
          p3d_add_node_compco(N, TargetComp);
        }
        /* Otherwise compcos merge */
        else {
          if (TargetComp->num > N->numcomp) {
            p3d_merge_comp(G, N->comp, compPt);
            *compPt = NULL;
          } else {
            p3d_merge_comp(G, TargetComp, &(N->comp));
          }
        }
        return(TRUE);
      }
    }
    list_node = list_node->next;
  }
  /* Non connexion has been found (in oriented case, arcs may have been created) */
  return(FALSE);
}

/*********************************************/
/* Fonction qui essaye de connecter un noeud */
/* a une composante connexe                  */
/* In : le graphe, le noeud, la composante   */
/* connexe                                   */
/* Out : relies ou non ?                     */
/*********************************************/
int p3d_link_node_comp_multisol(p3d_graph *G, p3d_node *N, p3d_compco **compPt) {
  p3d_compco * TargetComp = *compPt;
  double dist = 0.;
  p3d_node * Nc = NULL;
  p3d_list_node *list_node;
  int ValidForward, ValidBackward;

  /* If the criteria for choosing the best node in the target compco is */
  /* the distance, then node lists must be ordered */
  if (p3d_get_SORTING() == P3D_DIST_NODE) {
    list_node = TargetComp->dist_nodes;
    while (list_node != NULL) {
      list_node->N->dist_Nnew = p3d_APInode_dist_multisol(G, N, list_node->N);
      list_node = list_node->next;
    }
    p3d_order_node_list(TargetComp->dist_nodes);
  }

  /* Test the existence of a valid forward and backward path */
  list_node = TargetComp->dist_nodes;

  ValidBackward = ValidForward = FALSE;

  while (list_node != NULL) {

    Nc = list_node->N;

    if (p3d_get_SORTING() == P3D_DIST_NODE) {
      if ((Nc->dist_Nnew > p3d_get_DMAX()) && (p3d_get_DMAX() > 0.)) {
        return (FALSE);
      }
    }
    /* Oriented case, forward and backward paths must be separately tested */
    if (G->oriented) {
      if (ValidForward == FALSE) {
        if (p3d_APInode_linked_multisol(G, N, Nc, &dist)) {
          /* A forward path is found */
          p3d_create_one_edge(G, N, Nc, dist);
          ValidForward = TRUE;
        }
      }

      if (ValidBackward == FALSE) {
        if (p3d_APInode_linked_multisol(G, Nc, N, &dist)) {
          /* A bacward path is found */
          p3d_create_one_edge(G, Nc, N, dist);
          ValidBackward = TRUE;
        }
      }

      if (ValidBackward && ValidForward) {
        if (!N->comp) {
          /* A valid forward and backward path exist, and the node is still in none compco */
          /* so the tested compco will now include the new node */
          p3d_add_node_compco(N, TargetComp);
        } else {
          /* A valid forward and backward path exist, and the node is already included in a compco */
          /* so the tested compco and the compco of the new node must merge */
          if (TargetComp->num > N->numcomp) {
            p3d_merge_comp(G, N->comp, compPt);
            *compPt = NULL;
          } else {
            p3d_merge_comp(G, TargetComp, &(N->comp));
          }
        }
        return TRUE;
      }
    }
    /* Non - oriented case, If the forward path is valid, the backward one is also valid. */
    else {
      if (p3d_APInode_linked_multisol(G, N, Nc, &dist)) {
        p3d_create_edges(G, N, Nc, dist);
        /* If the node is still not included in a compco, it will be absorbed in the tested compco*/
        if (N->comp == NULL) {
          p3d_add_node_compco(N, TargetComp);
        }
        /* Otherwise compcos merge */
        else {
          if (TargetComp->num > N->numcomp) {
            p3d_merge_comp(G, N->comp, compPt);
            *compPt = NULL;
          } else {
            p3d_merge_comp(G, TargetComp, &(N->comp));
          }
        }
        return(TRUE);
      }
    }
    list_node = list_node->next;
  }
  /* Non connexion has been found (in oriented case, arcs may have been created) */
  return(FALSE);
}


/***************************************************************/
/*!\fn int p3d_link_node_graph(p3d_node* Node, p3d_graph* Graph)
* \brief try to link a node to the other connected component
*
* \param Node  the node to link
* \param Graph the graph
* \return number of linked components
*/
/***************************************************************/

int p3d_link_node_graph(p3d_node* Node, p3d_graph* Graph) {
  p3d_compco * Comp = Graph->comp;
  int nof_link = 0;
  /* For each compco of the graph */
  while (Comp) {
    if (Node->numcomp != Comp->num) {
      /* Try to connect the new node to the already existing compcos */
      if (p3d_link_node_comp(Graph, Node, &Comp)) {
        nof_link++;
      }
    }
    if (Comp == NULL) Comp = Graph->comp;
    Comp = Comp->suiv;
  }
  return nof_link;
}

/***************************************************************/
/*!\fn int p3d_link_node_graph_multisol(p3d_node* Node, p3d_graph* Graph)
* \brief try to link a node to the other connected component
*
* \param Node  the node to link
* \param Graph the graph
* \return number of linked components
*/
/***************************************************************/

int p3d_link_node_graph_multisol(p3d_node* Node, p3d_graph* Graph) {
  p3d_compco * Comp = Graph->comp;
  int nof_link = 0;
  /* For each compco of the graph */
  while (Comp) {
    if (Node->numcomp != Comp->num) {
      /* Try to connect the new node to the already existing compcos */
      if (p3d_link_node_comp_multisol(Graph, Node, &Comp)) {
        nof_link++;
      }
    }
    if (Comp == NULL) Comp = Graph->comp;
    Comp = Comp->suiv;
  }
  return nof_link;
}

// modif Juan
/***********************************************************/
/* Fonction generant des configurations aleatoires pour    */
/* un mecanisme dans un environement donne                 */
/* In : le nombre de configurations                        */
/* Out :                                                   */
/***********************************************************/

FILE* EnergyRandConfFile;
void p3d_randconfs(int NMAX, int (*fct_stop)(void), void (*fct_draw)(void)) {
  p3d_graph *G;
  int inode, ADDED;
  double tu, ts;
  int fail = 1;
  p3d_node  *Ns = NULL;
  configPt  q_s = NULL;
  int       *iksols_s = NULL;
  p3d_rob   *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  char str[512];
  FILE *fp = NULL;
  char EnergyRandConfFileName[] = "SampledOutput.txt";

  ChronoOn();

  p3d_del_graph(XYZ_GRAPH);
  G = p3d_create_graph();
  XYZ_GRAPH = G;

  inode = 0;

#ifdef BIO
  /*   if(bio_get_flag_pdb_nodes() == 1) { */
  /*     fp = fopen("nodes.pdb","w"); */
  /*     fprintf(fp,"# MODEL CRISTALLO\n"); */
  /*     fprintf(fp,"MODEL        %d\n",inode); */
  /*     translate_conf_to_pdb(robotPt, fp); */
  /*   } */
#endif

// for tests
//p3d_set_NB_NODES(5000);

// Create compco containing q_init
  q_s = p3d_copy_config(robotPt, robotPt->ROBOT_POS);
  p3d_copy_iksol(robotPt->cntrt_manager, NULL, &iksols_s);
  Ns  = p3d_APInode_make_multisol(G, q_s, iksols_s);
  p3d_insert_node(G, Ns);
  p3d_create_compco(G, Ns);
  Ns->type = ISOLATED;

  if (GroundCostObj != NULL) {
    EnergyRandConfFile = fopen(EnergyRandConfFileName, "w");
  }


  fprintf(EnergyRandConfFile, "SMRoadmap\n1\n2\n%d\n", p3d_get_NB_NODES());

  while (inode < p3d_get_NB_NODES()) {
    // ######### CASES #########
    ADDED = p3d_generate_random_free_conf(G, inode, fct_stop, &fail);
    //ADDED = p3d_generate_random_conf(G,fct_stop,&fail);
    //ADDED = p3d_generate_random_free_conf_multisol(G,fct_stop,&fail);

    if (ADDED) {
      inode = inode + ADDED;  // ADDED is the number of valid nodes
#ifdef BIO
      /*       if(bio_get_flag_pdb_nodes() == 1) { */
      /*         fprintf(fp,"MODEL        %d\n",inode); */
      /*         translate_conf_to_pdb(robotPt, fp); */
      /*       } */
      if (bio_get_flag_pdb_nodes() == 1) {
        sprintf(str, "sample%d.pdb", inode);
        fp = fopen(str, "w");
        translate_conf_to_pdb(robotPt, fp);
        fclose(fp);
      }
#endif
      if (fct_draw)(*fct_draw)();
    } else {
      PrintInfo(("Random conf. generation in not possible\n"));
      break;
    }

    if (fct_stop) {
      if (!(*fct_stop)()) {
        PrintInfo(("Random confs. generation canceled\n"));
        break;
      }
    }
  }
  if (GroundCostObj != NULL) {
    fclose(EnergyRandConfFile);
  }

#ifdef BIO
  /*   if(bio_get_flag_pdb_nodes() == 1) { */
  /*     fclose(fp); */
  /*   } */
#endif

  PrintInfo(("For the generation of %d configurations : ", inode));
  ChronoTimes(&tu, &ts);
  G->time = G->time + tu;
  ChronoPrint("");
  ChronoOff();
  p3d_print_info_graph(G);
}


int p3d_specific_search(char* filePrefix){
  double *arraytimes = MY_ALLOC(double, p3d_get_NB_specific());
  int sumnnodes = 0, sumnsamples = 0, sumncallsCD = 0, sumncallsLP = 0, nfail;
  p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  configPt qs = NULL, qg = NULL;

  qs = p3d_copy_config(robotPt, robotPt->ROBOT_POS);
  if (p3d_GetIsExpansionToGoal() == TRUE) {
    qg = p3d_copy_config(robotPt, robotPt->ROBOT_GOTO);
  }

  MY_ALLOC_INFO("Avant la creation du graphe");
  for (int i = 0; i < p3d_get_NB_specific(); i++) {
    printf("\n#### START OF TEST NUM.%d ####\n\n", i + 1);
    p3d_SetDiffuStoppedByWeight(0);
    p3d_SetStopValue(FALSE);

    p3d_loopSpecificLearn(robotPt, qs, qg, filePrefix, i, arraytimes, &nfail);

    sumnnodes += robotPt->GRAPH->nnode;
    sumnsamples += robotPt->GRAPH->nb_q;
    sumncallsCD += robotPt->GRAPH->nb_test_coll;
    sumncallsLP += robotPt->GRAPH->nb_local_call;

    if (i < (p3d_get_NB_specific() - 1)) {
      CB_del_param_obj(NULL, 0);//reset graphs
      if (p3d_get_RANDOM_CHOICE() == P3D_HALTON_SAMPLING) {
        p3d_init_random_seed(i);
      }
    }
    printf("\n#### END OF TEST NUM.%d ####\n\n", i + 1);
  }
  if (p3d_get_NB_specific() == 1){
    if (p3d_graph_to_traj(robotPt)) {
      g3d_add_traj("Globalsearch", p3d_get_desc_number(P3D_TRAJ));
    } else {
      printf("Problem during trajectory extraction\n");
      MY_FREE(arraytimes, double, p3d_get_NB_specific());
      printf("\n#### SPECIFIC SEARCH FAILED ####\n");
      return FALSE;
    }
  }else{
    p3d_specificPrintAverage(arraytimes, nfail, sumnnodes, sumnsamples, sumncallsCD, sumncallsLP);
  }
  MY_FREE(arraytimes, double, p3d_get_NB_specific());
  printf("\n#### SPECIFIC SEARCH COMPLETE ####\n");
  return TRUE;
}

extern int G3D_SAVE_MULT;
void p3d_loopSpecificLearn(p3d_rob *robotPt, configPt qs, configPt qg, char* filePrefix, int loopNb, double * arraytimes, int *nfail){
  int res = 0, *iksols = NULL, *iksolg = NULL;
  /* on construit un graph ou qs et qg seront dans la meme composante connexe */
  res = p3d_specific_learn(qs, qg, iksols, iksolg, fct_stop, fct_draw);
  if (!res) {
    if (p3d_GetDiffuStoppedByWeight()) {
      arraytimes[loopNb] = robotPt->GRAPH->time;
    } else {
      nfail++;
      arraytimes[loopNb] = - (robotPt->GRAPH->time);
    }
  } else {
    arraytimes[loopNb] = robotPt->GRAPH->time;
  }
  if (G3D_SAVE_MULT) {
    if (p3d_GetDiffuStoppedByWeight()){
      bio_search_max_weight_in_curr_rrt();
      res = p3d_specific_learn(qs, qg, iksols, iksolg, fct_stop, fct_draw);
      if (!res) {
        printf("p3d_specific_planner : ECHEC : il n'existe pas de chemin\n");
      } else {
        /* on construit la trajectoire entre les points etapes */
        if (!p3d_graph_to_traj(robotPt)) {
          printf("Problem during trajectory extraction\n");
        }
      }
    }
    int it = p3d_get_desc_number(P3D_TRAJ);
    if (p3d_get_desc_number(P3D_TRAJ) > it) {
      p3d_printTrajGraphContactPdbFiles(filePrefix, loopNb, robotPt);
    }
  }
}

static void p3d_specificPrintAverage(double * arraytimes, int nfail, int sumnnodes, int sumnsamples, int sumncallsCD, int sumncallsLP){
  int nbSpecific = p3d_get_NB_specific(), imin = 0, imax = 0, count_cor = 0;
  double mintime = 0.0, maxtime = 0.0, ttime = 0.0, avtime = 0.0, add_diff = 0.0;
  double std_dev = 0.0, ttime2 = 0.0, coravtime = 0.0, corstd_dev = 0.0;

  printf("\n## Computing time of %d tests ##\n", nbSpecific);
  mintime = P3D_HUGE;
  maxtime = -P3D_HUGE;
  for (int j = 0; j < nbSpecific; j++) {
    if (fabs(arraytimes[j]) < mintime) {
      imin = j;
      mintime = fabs(arraytimes[j]);
    }
    if (fabs(arraytimes[j]) > maxtime) {
      imax = j;
      maxtime = fabs(arraytimes[j]);
    }
    if (arraytimes[j] > 0)
      printf("Time %d = %f s.\n", j + 1, arraytimes[j]);
    else
      printf("Time %d = %f s. --  FAILED\n", j + 1, -arraytimes[j]);
    ttime += fabs(arraytimes[j]);
  }
  avtime = ttime / ((double) nbSpecific);
  for (int j = 0; j < nbSpecific; j++) {
    add_diff += SQR(fabs(arraytimes[j]) - avtime);
  }
  std_dev = sqrt((1.0 / ((double)nbSpecific)) * add_diff);
  // corrected average
  for (int j = 0; j < nbSpecific; j++) {
    if ((fabs(arraytimes[j]) > (avtime - std_dev)) && (fabs(arraytimes[j]) < (avtime + std_dev))) {
      ttime2 += fabs(arraytimes[j]);
      count_cor++;
    }
  }
  coravtime = ttime2 / ((double)(count_cor));
  for (int j = 0; j < nbSpecific; j++) {
    if ((fabs(arraytimes[j]) > (avtime - std_dev)) && (fabs(arraytimes[j]) < (avtime + std_dev))) {
      add_diff += SQR(fabs(arraytimes[j]) - coravtime);
    }
  }
  corstd_dev = sqrt((1.0 / ((double)nbSpecific)) * add_diff);

  printf("\nAverage time      = %f s.\n", avtime);
  printf("Minimum time      = %f s.\n", mintime);
  printf("Maximum time      = %f s.\n", maxtime);
  printf("Std. deviation    = %f\n", std_dev);
  printf("Corrected Average = %f s.\n", coravtime);
  printf("Corrected Std.Dev = %f\n", corstd_dev);
  printf("Num. fails        = %d\n", nfail);
  printf("Average Nnodes    = %f\n", (double)sumnnodes / (double)nbSpecific);
  printf("Average Nsamples  = %f\n", (double)sumnsamples / (double)nbSpecific);
  printf("Average NcallsCD  = %f\n", (double)sumncallsCD / (double)nbSpecific);
  printf("Average NcallsLP  = %f\n", (double)sumncallsLP / (double)nbSpecific);
}

// fmodif Juan

/***********************************************************/
/* Fonction creant un roadmap sur l'environnement courant, */
/* dans le graphe courant avec un nombre de noeuds fixe    */
/* In : le nombre de noeuds                                */
/* Out :                                                   */
/***********************************************************/

void p3d_learn(int NMAX, int (*fct_stop)(void), void (*fct_draw)(void)) {
  p3d_graph *G;
#ifdef MULTIGRAPH
  p3d_graph *final = NULL;
  int *addedTab = NULL;
  int mgNum = -1, totAdded = 0;
  double fsgTime = 0.0;
  configPt qs, qg;
#endif
  int inode, ADDED = TRUE;
  double tu, ts;
  int fail = 1;

  p3d_SetStopValue(FALSE);
  ChronoOn();

  if (!XYZ_GRAPH) G = p3d_create_graph();
  else           G = XYZ_GRAPH;
#ifdef MULTIGRAPH
  qs = p3d_copy_config(G->rob, G->rob->ROBOT_POS);
  if (p3d_GetIsExpansionToGoal() == TRUE) {
    qg = p3d_copy_config(G->rob, G->rob->ROBOT_GOTO);
  }
  final = G;
  addedTab = MY_ALLOC(int, G->rob->mg->nbGraphs);
  for(int i = 0; i < G->rob->mg->nbGraphs; i++){
    addedTab[i] = 1;
  }
  totAdded = G->rob->mg->nbGraphs;
#endif

  inode = 0;
  p3d_set_planning_type(P3D_GLOBAL);
  while (inode < NMAX && ADDED) {
#ifdef MULTIGRAPH
  if (p3d_get_multiGraph()) {
    if(p3d_doIncrementalConstruction(-1) && mgNum != -1){
      ChronoOn();
      p3d_setAllDofActive(final->rob);
      p3d_fillFlatMultiGraph(final->rob, NULL, NULL, mgNum, 2);
      p3d_setAllDofActive(final->rob);
      p3d_del_graph(final);
      final = p3d_create_graph();
      p3d_convertFsgToGraph(final, final->rob->mg->fsg);
      ChronoTimes(&tu, &ts);
      ChronoOff();
      fsgTime += tu;
      XYZ_GRAPH = final;
      final->rob->GRAPH = final;
      p3d_addStartAndGoalNodeToGraph(final->rob->ROBOT_POS, final->rob->ROBOT_GOTO, NULL, NULL, final, final->rob);
      if(p3d_graph_to_traj(final->rob)){
        ADDED = FALSE;
        final->mgTime += fsgTime;
        continue;
      }
    }
    do{
      G = p3d_setRandomMultiGraphAndActiveDof(final->rob, &mgNum);
    }while((!p3d_doIncrementalConstruction(-1) && addedTab[mgNum] >= p3d_get_NB_TRY())
          || (p3d_doIncrementalConstruction(-1) && !p3d_graph_to_traj(final->rob)));
     printf("Graph seclectionne : %d size : %d\n", mgNum, G->nnode);
    fail = addedTab[mgNum];
  }
#endif
    /* Call basic PRM or Visibility method */
    switch (p3d_get_MOTION_PLANNER()) {
      case P3D_BASIC:{
        ADDED = p3d_add_basic_node(G, fct_stop, &fail);
        break;
      }
      case P3D_ISOLATE_LINKING:{
        ADDED = p3d_add_isolate_or_linking_node(G, fct_stop, fct_draw,
                                                &fail, P3D_ISOLATE_LINKING);
        break;
      }
      case P3D_ALL_PRM:{
        ADDED = p3d_add_all_prm_node(G, fct_stop);
        break;
      }
      default:{
        PrintInfo(("p3d_learn : ERREUR : pas de planificateur global...\n"));
        return;
      }
    }
    if (ADDED) {
      inode += ADDED;
      if (fct_draw){
#ifdef MULTIGRAPH
        if(p3d_get_multiGraph()){
          p3d_set_user_drawnjnt(G->rob->mg->mgJoints[mgNum]->joints[G->rob->mg->mgJoints[mgNum]->nbJoints - 1]);
        }
#endif
        (*fct_draw)();
      }
    } else {
#ifdef MULTIGRAPH
      if (p3d_get_multiGraph()) {
        addedTab[mgNum] = fail;
        if(addedTab[mgNum] >= p3d_get_NB_TRY() || (fct_stop && !(*fct_stop)())){
          totAdded--;
          printf("Graph %d complete, Nodes %d, rest %d\n", mgNum, G->nnode, totAdded);
        }
        if(totAdded == 0){
          ADDED = FALSE;
        }else{
          ADDED = TRUE;
          continue;
        }
      }
#endif
      PrintInfo(("p3d_learn : ECHEC a l'insertion d'un noeud\n"));
      ADDED = FALSE;
    }
    if (fct_stop) {
      if (!(*fct_stop)()) {
        PrintInfo(("basic PRM building canceled\n"));
        ADDED = FALSE;
      }
    }
  }
#ifdef MULTIGRAPH
  if (p3d_get_multiGraph()) {
    if(!p3d_doIncrementalConstruction(-1)){
      p3d_print_info_graph(G);
      ChronoOn();
      p3d_setAllDofActive(final->rob);
      p3d_flatMultiGraph(final->rob, 0);
      p3d_setAllDofActive(final->rob);
      p3d_convertFsgToGraph(final, final->rob->mg->fsg);
      ChronoTimes(&tu, &ts);
      ChronoOff();
      final->mgTime += tu;
    }
    XYZ_GRAPH = final;
    final->rob->GRAPH = final;
    G = final;
    p3d_set_user_drawnjnt(-1);
    MY_FREE(addedTab, int, G->rob->mg->nbGraphs);
  }
#endif
  p3d_set_planning_type(P3D_NONE);
  PrintInfo(("Pour la creation de %d noeuds : ", inode));

  ChronoTimes(&tu, &ts);
  G->time = G->time + tu;
  /* When retrieving statistics;Commit Jim; date: 01/10/2008 */
  if (getStatStatus()) {
    if(p3d_get_cycles() == TRUE){
      G->stat->cyclingTime += tu;
    }else{
      G->stat->preTime += tu;
    }
  }
  ChronoPrint("");
  ChronoOff();

  MY_ALLOC_INFO("After p3d_learn");
  p3d_print_info_graph(G);
}

/***********************************************************/
/* Fonction expansant les noeuds de toutes les composantes */
/* connexes du graphe contenant moins de n% des noeuds     */
/* In : le nombre de noeuds a rajouter pour chaque noeur,  */
/* le graphe                                               */
/* Out :                                                   */
/***********************************************************/
void p3d_expand_graph(p3d_graph *G, double frac, int (*fct_stop)(void),
                      void (*fct_draw)(void)) {
  PrintWarning(("p3d_expand_graph: Not yet implemented\n"));
}

p3d_node ** p3d_addStartAndGoalNodeToGraph(configPt qs, configPt qg, int *iksols, int *iksolg, p3d_graph *G, p3d_rob *robotPt){
  p3d_node *Ns = NULL, *Ng = NULL, **nodetab;
  configPt q_s = NULL, q_g = NULL;
  int *iksols_s = NULL, *iksolg_s = NULL;

  Ns = p3d_TestConfInGraph(G, qs);
  if (p3d_GetIsExpansionToGoal() == TRUE) {
    Ng = p3d_TestConfInGraph(G, qg);
  }
  /* If not, create them */
  if (Ns == NULL) {
    q_s = p3d_copy_config(robotPt, qs);
    p3d_set_robot_config(robotPt, qs);
    p3d_update_this_robot_pos_without_cntrt_and_obj(robotPt);
    p3d_copy_iksol(robotPt->cntrt_manager, robotPt->ikSolPos, &iksols_s);
    Ns  = p3d_APInode_make_multisol(G, q_s, iksols_s);
    p3d_insert_node(G, Ns);
    G->dist_nodes = p3d_add_node_to_list(Ns, G->dist_nodes);
    p3d_create_compco(G, Ns);
    Ns->type = ISOLATED;
    // initialization for the functions computing simplified metrics mased on frames distance
    //    p3d_set_mob_frame_0(Ns->rel_mob_frame);
    p3d_SetMobFrame0(Ns->RelMobFrame);

    if (p3d_link_node_graph_multisol(Ns, G))
      PrintInfo(("qs reliee au graphe\n"));
    else
      p3d_APInode_expand(G, Ns, fct_stop, fct_draw);
  }
  if ((p3d_GetIsExpansionToGoal() == TRUE) && (Ng == NULL)) {
    q_g = p3d_copy_config(robotPt, qg);
    p3d_set_robot_config(robotPt, qg);
    p3d_update_this_robot_pos_without_cntrt_and_obj(robotPt);
    p3d_copy_iksol(robotPt->cntrt_manager, robotPt->ikSolGoto, &iksolg_s);
    Ng  = p3d_APInode_make_multisol(G, q_g, iksolg_s);
    p3d_insert_node(G, Ng);
    G->dist_nodes = p3d_add_node_to_list(Ng, G->dist_nodes);
    p3d_create_compco(G, Ng);
    Ng->type = ISOLATED;

    if (p3d_link_node_graph_multisol(Ng, G))
      PrintInfo(("qg reliee au graphe\n"));
    else
      p3d_APInode_expand(G, Ng, fct_stop, fct_draw);


  }
  p3d_set_robot_config(robotPt, Ns->q);
  p3d_update_this_robot_pos_without_cntrt_and_obj(robotPt);


  /* Initialize some data in the graph for the A* graph search */
  G->search_start = Ns;
  if (p3d_GetIsExpansionToGoal() == TRUE) {
    G->search_goal = Ng;
  }
  G->search_done = FALSE;

  if (p3d_GetIsWeightedChoice() == TRUE) {
    p3d_init_root_weight(G);
  }
  if (p3d_GetExpansionNodeMethod() == RANDOM_IN_SHELL_METH) {
    p3d_init_pb_variables(G);

#ifdef ENERGY
    if (p3d_get_MOTION_PLANNER() ==  BIO_COLDEG_RRT) {
      n_coldeg = bio_get_num_collective_degrees();
      // init coldeg_q in Ns
      coldeg_qs = bio_alloc_coldeg_config(n_coldeg);
      for (icoldeg = 0; icoldeg < n_coldeg; icoldeg++) {
        coldeg_qs[icoldeg] = 0.0;
      }
      bio_copy_coldeg_q_in_N(Ns, coldeg_qs, n_coldeg);
      bio_destroy_coldeg_config(coldeg_qs, n_coldeg);
      // WARNING : currently Ng is not considered !!!
    }
#endif
  }
  nodetab = MY_ALLOC(p3d_node*, 2);
  nodetab[0] = Ns;
  nodetab[1] = Ng;
  return nodetab;
}

/**********************************************************/
/* Fonction creant une roadmap reliant deux configurations */
/* donnees                                                */
/* In : les deux configurations                           */
/* Out : si les configurations sont reliees ou pas        */
/**********************************************************/
int p3d_specific_learn(double *qs, double *qg, int *iksols, int *iksolg, int (*fct_stop)(void), void (*fct_draw)(void)) {
  p3d_rob   *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  p3d_graph *G;
  p3d_node  *Ns = NULL, *Ng = NULL;

  int       inode = 0, fail = 1, ADDED = TRUE;
  double    tu, ts;
  int i, nb_dof;
  int nbInitGraphNodes, nbGraphNodes;
#ifdef ENERGY
  int n_coldeg, icoldeg;
  double *coldeg_qs;
#endif

  /* Avoid some stupid errors */
  if (qs == NULL) {
    PrintInfo(("p3d_specific_learn : ERREUR : pas de configuration initiale\n"));
    return(FALSE);
  }
  if ((qg == NULL) && (p3d_GetIsExpansionToGoal() == TRUE)) {
    PrintInfo(("p3d_specific_learn : ERREUR : pas de configuration finale\n"));
    return(FALSE);
  }

  if ((p3d_GetIsExpansionToGoal() == TRUE) && p3d_equal_config(robotPt, qs, qg)) {
    // tmp. modif  Juan
    nb_dof = p3d_get_robot_ndof();
    for (i = 0;i < nb_dof;i++)
      qg[i] = 0.0;
    /*     PrintInfo(("p3d_specific_search : ERREUR : qs = qg\n")); */
    /*     return(FALSE); */
    // Fmodif Juan
  }
  p3d_set_planning_type(P3D_SPECIFIC);
  ChronoOn();
  if (!XYZ_GRAPH)  G = p3d_create_graph();
  else            G = XYZ_GRAPH;
  /* Nodes QS and QG exist ?*/
  p3d_node ** nodetab = p3d_addStartAndGoalNodeToGraph(qs, qg, iksols, iksolg, G, robotPt);
  Ns = nodetab[0];
  Ng = nodetab[1];
  MY_FREE(nodetab, p3d_node *, 2);

  ADDED = TRUE;
  if (p3d_get_MOTION_PLANNER() != P3D_DIFFUSION) {
    /* While solution does not exists, insert new nodes with basic PRM or Visibility or RRT */
    while ((Ns->numcomp != Ng->numcomp) && !p3d_compco_linked_to_compco(Ns->comp, Ng->comp) && ADDED) {
      switch (p3d_get_MOTION_PLANNER()) {
        case P3D_BASIC:
          ADDED = p3d_add_basic_node(G, fct_stop, &fail);
          break;
        case P3D_ISOLATE_LINKING:
          ADDED = p3d_add_isolate_or_linking_node(G, fct_stop, fct_draw, &fail, P3D_ISOLATE_LINKING);
          break;
        case P3D_ALL_PRM:
          ADDED = p3d_add_all_prm_node(G, fct_stop);
          break;
#ifdef ENERGY
        case BIO_COLDEG_RRT:
          ADDED = bio_expand_coldeg_rrt(G, fct_stop);
          break;
#endif
        default:
          PrintInfo(("p3d_specific_learn : ERREUR : pas de planificateur global...\n"));
          return(FALSE);
      }

      if (ADDED) {
        inode += ADDED;
        if (fct_draw){
          (*fct_draw)();
        }
        if (fct_stop) {
          if (!(*fct_stop)()){
            ADDED = FALSE;
          }
        }
      } else {
        PrintInfo(("p3d_specific_learn : ECHEC a l'insertion d'un noeud\n"));
        ADDED = FALSE;
      }
    }
  } else {
    nbInitGraphNodes = G->nnode;
    ADDED = p3d_RunDiffusion(G, fct_stop, fct_draw);
    nbGraphNodes = G->nnode;
    inode  = nbGraphNodes - nbInitGraphNodes;
  }
  p3d_set_planning_type(P3D_NONE);
  PrintInfo(("Pour la creation de %d noeuds : ", inode));
  ChronoPrint("");

  ChronoTimes(&tu, &ts);
  G->time = G->time + tu;
  if(getStatStatus()){
    G->stat->planTime += tu;
  }
  ChronoOff();

  p3d_print_info_graph(G);
  MY_ALLOC_INFO("After p3d_specific_learn");
  if (p3d_get_saveInfoInFile()) {
    save_infos_in_file(G, ADDED);
  }

  PrintInfo(("\n"));
  return(ADDED);
}

/*************************************************************/
/* Fonction creant un ensemble de noeuds "non reliables" sur */
/* l'environnement courant                                   */
/* In : le nombre de noeuds a creer                          */
/* Out :                                                     */
/*************************************************************/
void p3d_create_orphans(int NMAX, int (*fct_stop)(void), void (*fct_draw)(void)) {
  p3d_graph *G;
  int inode;
  double tu, ts;
  int fail = 1;

  ChronoOn();

  if (!XYZ_GRAPH) G = p3d_create_graph();
  else           G = XYZ_GRAPH;

  inode = 0;

  while (inode < NMAX) {
    if (p3d_add_isolate_or_linking_node(G, fct_stop, fct_draw, &fail, ISOLATED)) {
      inode = inode + 1;
      if (fct_draw)(*fct_draw)();
    } else {
      PrintInfo(("p3d_learn : ECHEC a l'insertion d'un noeud\n"));
      break;
    }
  }

  PrintInfo(("Pour la creation de %d noeuds linking: ", inode));
  ChronoTimes(&tu, &ts);
  G->time = G->time + tu;
  ChronoPrint("");
  ChronoOff();
  MY_ALLOC_INFO("After p3d_create_linking");
  p3d_print_info_graph(G);
}

/***************************************************************/
/* Fonction creant un ensemble de noeuds reliant au moins deux */
/* composantes connexes sur l'environnement courant            */
/* In : le nombre de noeuds a creer                            */
/* Out :                                                       */
/***********************************************************  **/
void p3d_create_linking(int NMAX, int (*fct_stop)(void), void (*fct_draw)(void)) {
  p3d_graph *G;
  int inode;
  double tu, ts;
  int fail = 1;

  ChronoOn();

  if (!XYZ_GRAPH) G = p3d_create_graph();
  else           G = XYZ_GRAPH;

  inode = 0;

  while (inode < NMAX) {
    if (p3d_add_isolate_or_linking_node(G, fct_stop, fct_draw, &fail, LINKING)) {
      inode = inode + 1;
      if (fct_draw)(*fct_draw)();
    } else {
      PrintInfo(("p3d_learn : ECHEC a l'insertion d'un noeud\n"));
      break;
    }
  }

  PrintInfo(("Pour la creation de %d noeuds linking: ", inode));
  ChronoTimes(&tu, &ts);
  G->time = G->time + tu;
  ChronoPrint("");
  ChronoOff();
  MY_ALLOC_INFO("After p3d_create_linking");
  p3d_print_info_graph(G);
}

// modif Juan
/*****************************************************
  * Fonction de generation d'une conf. aleatoire libre
  *****************************************************/

int p3d_generate_random_free_conf(p3d_graph *G, int inode, int (*fct_stop)(void), int * fail) {
  p3d_node *N = NULL;
  double cost;
  if (fct_stop) {
    if (!(*fct_stop)()) {
      PrintInfo(("basic PRM building canceled\n"));
      return FALSE;
    }
  }
  /* Create a node */
  if (p3d_col_get_mode() == p3d_col_mode_bio) {
    if (p3d_get_RLG() &&
        ((strcmp(G->rob->cntrt_manager->cntrts[0]->namecntrt, "p3d_6R_bio_ik_nopep") == 0) ||
         (strcmp(G->rob->cntrt_manager->cntrts[0]->namecntrt, "p3d_6R_bio_ik_nopep_new") == 0))) {
      // WARNING : generates the conformation of ONE ONLY loop
      //N = bio_shoot_loop_OLD(G);
      N = bio_shoot_loop(G);
    } else {
      N = bio_shoot_free_conf(G);
    }
  } else {
    N = p3d_APInode_shoot(G);
  }

  if (N == NULL) {
    if (p3d_col_get_mode() == p3d_col_mode_bio) {
      p3d_generate_random_free_conf(G, inode, fct_stop, fail);
    } else
      return(FALSE);
  } else {
    p3d_insert_node(G, N);

    /* Don't try connections with others compcos */
    N->numcomp = -1;
    p3d_create_compco(G, N);
    if (GroundCostObj != NULL) {
      cost = p3d_GetConfigCost(G->rob, N->q);
      fprintf(EnergyRandConfFile, "%d\t 2\t %f\t %f\t  %f\t 1\n", inode, N->q[6], N->q[7], cost);
    }
  }
  return(TRUE);
}

/*****************************************************************************************
  * Fonction de generation plusieurs (toutes les possibles par IK) conf. aleatoires libres
  *****************************************************************************************/
int p3d_generate_random_free_conf_multisol(p3d_graph *G, int (*fct_stop)(void), int * fail) {
  p3d_node ** N;
  int i, nsol;

  if (fct_stop) {
    if (!(*fct_stop)()) {
      PrintInfo(("basic PRM building canceled\n"));
      return FALSE;
    }
  }
  /* Create a node */
  N = p3d_APInode_shoot_multisol(G, &nsol);
  for (i = 0; i < nsol; i++) {
    p3d_insert_node(G, N[i]);
    /* Don't try connections with others compcos */
    N[i]->numcomp = -1;
    p3d_create_compco(G, N[i]);
  }
  MY_FREE(N, p3d_node*, ((int)(nsol / 10))*10 + 10);
  return(nsol);
}


/***********************************************
  * Fonction de generation d'une conf. aleatoire
  ***********************************************/
int p3d_generate_random_conf(p3d_graph *G, int (*fct_stop)(void), int * fail) {
  p3d_node *N = NULL;

  if (fct_stop) {
    if (!(*fct_stop)()) {
      PrintInfo(("basic PRM building canceled\n"));
      return FALSE;
    }
  }
  /* Create a node */
  N = p3d_APInode_shoot_nocolltest(G);
  //p3d_insert_node(G,N);

  /* Don't try connections with others compcos */
  // DON"T CREATE COMPCO WHEN TESTING SPEED !!!
  //N->numcomp = -1;
  //p3d_create_compco(G,N);

  return(TRUE);
}

// fmodif Juan


/**************************************************
  * Fonction de generation d'un noeud de la roadmap
  **************************************************/

int p3d_add_basic_node(p3d_graph *G, int (*fct_stop)(void), int * fail) {
  p3d_node **N = NULL;
  p3d_list_edge * EdgeScan;
  p3d_list_compco * CompcoScan;
  int nbNodes = 0, i = 0, singularity = 0, ADDED = 0;

  if (fct_stop) {
    if (!(*fct_stop)()) {
      PrintInfo(("basic PRM building canceled\n"));
      return FALSE;
    }
  }
  //We shoot a singularity or not?
  if ((G->rob->cntrt_manager->ncntrts != 0) && p3d_get_ik_choice() != IK_NORMAL) {
    singularity = (int)p3d_random(0, 20);
    if (singularity == 0 && G->ncomp > 1) {// 1/20 to shoot a singularity
      singularityCheck = 1;
    }
  }
  /* Create a node */
  N = p3d_APInode_shoot_multisol(G, &nbNodes);

  if (singularity == 0) {// reinit the flag for singularity
    singularityCheck = 0;
  }

  for (i = 0; i < nbNodes; i++) {
    p3d_insert_node(G, N[i]);
    /* Try connections with others compcos */
    p3d_link_node_graph_multisol(N[i], G);

    if (N[i]->numcomp == -1 && !N[i]->isSingularity) {
      /* Node have not been included in a compco, create one for it */
      p3d_create_compco(G, N[i]);
      if (G->oriented) {
        /* In the oriented case, some arcs may have been created, so update the lists of successors */
        EdgeScan = G->edges;
        while (EdgeScan) {
          if (EdgeScan->E->Ni == N[i]) {
            p3d_add_compco_to_reachable_list_and_update_predecessors(G, N[i]->comp, EdgeScan->E->Nf->comp);
            CompcoScan = EdgeScan->E->Nf->comp->canreach;
            while (CompcoScan != NULL) {
              p3d_add_compco_to_reachable_list_and_update_predecessors(G, N[i]->comp, CompcoScan->comp);
              CompcoScan = CompcoScan->next;
            }
          }
          if (EdgeScan->E->Nf == N[i]) {
            p3d_add_compco_to_reachable_list_and_update_predecessors(G, EdgeScan->E->Ni->comp, N[i]->comp);
            CompcoScan = N[i]->comp->canreach;
            while (CompcoScan != NULL) {
              p3d_add_compco_to_reachable_list_and_update_predecessors(G, EdgeScan->E->Ni->comp, CompcoScan->comp);
              CompcoScan = CompcoScan->next;
            }
          }
          EdgeScan = EdgeScan->next;
        }
      }
      p3d_merge_check(G);
    }
    ADDED++;
  }
  return(ADDED);
}

/*******************************************************************/
/* Fonction generant un noeud liant ou gardien                     */
/* un gardien n est gardien que si il n est vu par aucun autre     */
/* noeud, et liant si il voit au moins deux noeuds de deux         */
/* composantes connexes differentes                                */
/*******************************************************************/

int p3d_add_isolate_or_linking_node(p3d_graph *G, int (*fct_stop)(void),
                                    void (*fc_tdraw)(void),
                                    int *fail, int type) {
  p3d_node **N = NULL, *Nc = NULL;
  p3d_list_node *node = NULL, *linked_nodes = NULL, *destr_node,
                        * forw_linked_nodes = NULL, * back_linked_nodes = NULL;
  int NB_LINK, NB_LINK_ORPH, ADDED = 0, LINK_LINK, LINK_ORPH;
  double dist = 0.0;
  p3d_compco * CompScan;
  p3d_list_node * NodeScan;
  int ValidForward, ValidBackward, nbNodes, *ikSol = NULL;
  //start path deform
  p3d_node * node1Pt, * node2Pt;
  p3d_list_edge *list_edge, *list_edge2;
  int loop = 0, test = 0, i = 0, j = 0;
  //end path deform

  while (!ADDED) {
    //start path deform
    loop = 0;
    nbNodes = 0;
    //end path deform
    //if stop conditions was reached
    if (fct_stop) {
      if (!(*fct_stop)()) {
        PrintInfo(("visi-PRM building canceled\n"));
        return FALSE;
      }
    }
    //We shoot a singularity or not?
    if ((G->rob->cntrt_manager->ncntrts != 0) && p3d_get_ik_choice() != IK_NORMAL) {
      test = (int)p3d_random(0, 5);//5
      if (test == 0 && G->ncomp > 1) {// 1/20 to shoot a singularity
        singularityCheck = 1;
      }
    }
    //create the node from admissible config for the robot
    N = p3d_APInode_shoot_multisol(G, &nbNodes);

    if (test == 0) {// reinit the flag for singularity
      singularityCheck = 0;
    }
    for (i = 0; i < nbNodes; i++) {
      /* Count Nof Links between the node and the other compcos */
      NB_LINK   = NB_LINK_ORPH = 0;
      LINK_ORPH = LINK_LINK    = FALSE;
      CompScan = G->comp; //list of existing compco in the graph
      linked_nodes = NULL; //list of linked nodes to N
      back_linked_nodes = NULL; //list of linked nodes to N backward (oriented graph)
      forw_linked_nodes = NULL; //list of linked nodes to N forward (oriented graph)

      while (CompScan) { //while all the compcos wasn't scanned
        if (p3d_get_SORTING() == P3D_DIST_NODE) {//if we sort nodes by distance
          NodeScan = CompScan->dist_nodes;//get the sorted list
          while (NodeScan != NULL) {//update all nodes in the list
            NodeScan->N->dist_Nnew = p3d_APInode_dist_multisol(G, N[i], NodeScan->N);//compute the distance between N[i] and the node in the list
            NodeScan = NodeScan->next;
          }
          p3d_order_node_list(CompScan->dist_nodes);//reorder the list
//           p3d_order_node_list_multisol(G, CompScan->dist_nodes, N[i]);//reorder the list
        }
        NodeScan = CompScan->dist_nodes;//get the first node
        /* Oriented case */
        if (G->oriented) {
          ValidBackward = ValidForward = FALSE;
          /* A forward or a backward link may exist, we must not create
            redundancy */
          node = forw_linked_nodes;
          while (node) {
            if (p3d_compco_linked_to_compco(node->N->comp, CompScan)) {
              ValidForward = TRUE;
            }
            if (p3d_compco_linked_to_compco(CompScan, node->N->comp)) {
              ValidBackward = TRUE;
            }
            node = node->next;
          }
          /* Scan all the nodes of each compco */
          while (NodeScan) {
            Nc = NodeScan->N;
            if (ValidForward == FALSE) {
              if (p3d_APInode_linked_multisol(G, N[i], Nc, &dist)) {
                /* a forward link can be created */
                ValidForward = TRUE;
                forw_linked_nodes =  p3d_add_node_to_list(Nc, forw_linked_nodes);
              }
            }
            if (ValidBackward == FALSE) {
              if (p3d_APInode_linked_multisol(G, Nc, N[i], &dist)) {
                /* A backward link can be created */
                ValidBackward = TRUE;
                back_linked_nodes = p3d_add_node_to_list(Nc, back_linked_nodes);
              }
            }
            NodeScan = NodeScan->next;
            if (ValidBackward && ValidForward) {
              NB_LINK ++;
              break;
            }
          }
        }
        /* Non-oriented case */
        else {
          p3d_node * tmpNode = NULL;
          while (NodeScan) {//for all node in the dist list
            Nc = NodeScan->N;
            if (p3d_APInode_linked_multisol(G, N[i], Nc, &dist)) {//Test if the two nodes can be linked
              if (Nc->type == ISOLATED) NB_LINK_ORPH++;//We link an orpholan node
#ifdef MULTIGRAPH
              if(p3d_get_multiGraph() && p3d_doIncrementalConstruction(-1) && (Nc->needMgCycle || tmpNode)){
                if(tmpNode){
                  NB_LINK += 2;//we have one more link
                  linked_nodes = p3d_add_node_to_list(tmpNode, linked_nodes);//add the node Nc to the new node linked list
                  linked_nodes = p3d_add_node_to_list(Nc, linked_nodes);//add the node Nc to the new node linked list
                  break;//check the other compco
                }else{
                  tmpNode = Nc;
                }
              }else{
#endif
                NB_LINK++;//we have one more link
                linked_nodes = p3d_add_node_to_list(Nc, linked_nodes);//add the node Nc to the new node linked list
                break;//check the other compco
#ifdef MULTIGRAPH
              }
#endif
            }
            NodeScan = NodeScan->next;
          }
        }
        CompScan = CompScan->suiv;
      }

      /* We know how many connection can be realised. A real insertion is done if the node
        is ISOLATED or LINKING */

      if ((NB_LINK == 0) && (type != LINKING) && !N[i]->isSingularity) {
        /* ISOLATED Case Guardians */
        p3d_insert_node(G, N[i]);//insert the node in the graph
        p3d_create_compco(G, N[i]);//create a compco containing the added node
        N[i]->type = ISOLATED;//is a isolated node
        //start path deform
        N[i]->only_for_cycle = FALSE;//is it not for cycles
        //end path deform
        if (G->oriented) {
          /* Oriented case. Even if ISOLATED, some arcs may be valid */
          node = forw_linked_nodes;
          while (node) {
            Nc = node->N;
            destr_node = node;
            p3d_create_one_edge(G, N[i], Nc, dist);
            node = destr_node->next;
            MY_FREE(destr_node, p3d_list_node, 1);
          }
          node = back_linked_nodes;
          while (node) {
            Nc = node->N;
            destr_node = node;
            p3d_create_one_edge(G, Nc, N[i], dist);
            node = node->next;
            MY_FREE(destr_node, p3d_list_node, 1);
          }
          p3d_merge_check(G);
        }
        /*print informations*/
        if (p3d_get_ik_choice() != IK_NORMAL) {
          p3d_print_iksol(G->rob->cntrt_manager, (N[i])->iksol);
        }
        PrintInfo(("isolated reset at : fail : %d\n", *fail));
        PrintInfo(("nodeCost : %f\n", (N[i])->cost));
        ChronoPrint("");
        *fail = 1;//reset the nb_try
        if (getStatStatus()) {
          G->stat->planConfGuard++;
        }
        ADDED++;
      } else {//if LINKING node and can connect to other nodes
        *fail = *fail + 1;

        if (*fail >= p3d_get_NB_TRY()) {//if we reach the max number of Ntry stop
          for (j = i; j < nbNodes; j++) {//desalloc all following nodes
            p3d_APInode_desalloc(G, N[j]);
          }
          MY_FREE(N, p3d_node*, ((int)(nbNodes / 10))*10 + 10);
          PrintInfo(("fail : %d\n", *fail));
          return(FALSE);
        }
        if (NB_LINK > 1 && type != ISOLATED) {//if is it not isolated and have more than a link Connector
          if (G->oriented) {
            p3d_insert_node(G, N[i]);
            p3d_create_compco(G, N[i]);
            N[i]->type = LINKING;
            //start path deform
            N[i]->only_for_cycle = FALSE;
            //end path deform
            node = forw_linked_nodes;
            while (node) {
              p3d_create_one_edge(G, N[i], node->N, dist);
              destr_node = node;
              node = node->next;
              MY_FREE(destr_node, p3d_list_node, 1);
            }
            node = back_linked_nodes;
            while (node) {
              p3d_create_one_edge(G, node->N, N[i], dist);
              destr_node = node;
              node = node->next;
              MY_FREE(destr_node, p3d_list_node, 1);
            }
            p3d_merge_check(G);
          } else {
            node = linked_nodes;//get the first node in the list of linked nodes
            p3d_insert_node(G, N[i]);//insert the new node in the graph
            N[i]->type = LINKING;//is a linking node
            N[i]->only_for_cycle = FALSE;//is it not for cycles

            p3d_add_node_compco(N[i], node->N->comp); // add the new node to the linked node compco
            p3d_get_non_sing_iksol(G->rob->cntrt_manager, N[i]->iksol, node->N->iksol, &ikSol);
            dist = p3d_dist_q1_q2_multisol(G->rob, N[i]->q, node->N->q, ikSol);//take the distance between the two nodes
            p3d_create_edges(G, N[i], node->N, dist);//create edge between the two nodes
            destr_node = node;
#ifdef MULTIGRAPH
            if (p3d_get_multiGraph() && p3d_doIncrementalConstruction(-1) && node->N->needMgCycle == TRUE) {
              node->N->needMgCycle = FALSE;
            }
#endif
            node = linked_nodes->next;
            MY_FREE(destr_node, p3d_list_node, 1);//free the allocated node in the function
            while (node) {//for all linked nodes to the new one
#ifdef MULTIGRAPH
              if (p3d_get_multiGraph() && p3d_doIncrementalConstruction(-1) && node->N->needMgCycle == TRUE) {
                node->N->needMgCycle = FALSE;
              }
#endif
              if (N[i]->numcomp < node->N->numcomp) {
                p3d_merge_comp(G, N[i]->comp, &(node->N->comp));// merge the two compco
                p3d_get_non_sing_iksol(G->rob->cntrt_manager, N[i]->iksol, node->N->iksol, &ikSol);
                dist = p3d_dist_q1_q2_multisol(G->rob, N[i]->q, node->N->q, ikSol);
                p3d_create_edges(G, N[i], node->N, dist);//link the two nodes
              } else if (N[i]->numcomp > node->N->numcomp){
                p3d_merge_comp(G, node->N->comp, &(N[i]->comp));// merge the two compco
                p3d_get_non_sing_iksol(G->rob->cntrt_manager, N[i]->iksol, node->N->iksol, &ikSol);
                dist = p3d_dist_q1_q2_multisol(G->rob, N[i]->q, node->N->q, ikSol);
                p3d_create_edges(G, N[i], node->N, dist);//link the two nodes
              }
#ifdef MULTIGRAPH
              else if (p3d_get_multiGraph() && p3d_doIncrementalConstruction(-1)){//the same compco
                p3d_get_non_sing_iksol(G->rob->cntrt_manager, N[i]->iksol, node->N->iksol, &ikSol);
                dist = p3d_dist_q1_q2_multisol(G->rob, N[i]->q, node->N->q, ikSol);
                p3d_create_edges(G, N[i], node->N, dist);//link the two nodes
              }
#endif
              destr_node = node;//free the allocated node in this function
              node = node->next;
              MY_FREE(destr_node, p3d_list_node, 1);
            }
          }

          ADDED++;
          if (p3d_get_ik_choice() != IK_NORMAL) {
            p3d_print_iksol(G->rob->cntrt_manager, (N[i])->iksol);
          }
          PrintInfo(("linking : fail : %d\n", *fail));
          PrintInfo(("nodeCost : %f\n", (N[i])->cost));
          ChronoPrint("");
          if (getStatStatus()) {
            G->stat->planConfConn++;
          }
//           *fail = 1;//reset the nb_try
        } else {
//           ADDED = FALSE;
          //start path deform
          if (p3d_get_cycles() == FALSE) {
            p3d_APInode_desalloc(G, N[i]);
            N[i] = NULL;
          }
          //end path deform
          /* NODE LISTS DESTRUCTION */
          if (G->oriented) {
            node = forw_linked_nodes;
            while (node) {
              destr_node = node;
              node = node->next;
              MY_FREE(destr_node, p3d_list_node, 1);
            }
            node = back_linked_nodes;
            while (node) {
              destr_node = node;
              node = node->next;
              MY_FREE(destr_node, p3d_list_node, 1);
            }
          } else {
            node = linked_nodes;
            while (node) {
              destr_node = node;
              node = node->next;
              MY_FREE(destr_node, p3d_list_node, 1);
            }
          }
        }
      }
      //start path deform
      // debut code graphs cycliques
      // si on veut trouver les cycles et le noeud n'as pas ete ajoute dans le graph
      //Le noeud une fois arrive ici peut se connecter a d'autres noeuds mais de la meme compco
      if ((p3d_get_cycles() == TRUE) && (ADDED == FALSE)) {
        test = p3d_is_node_useful_for_cycle(N[i], &node1Pt, &node2Pt, G);
        if (test == TRUE) {
          if (p3d_get_test_reductib() == TRUE) {
            test = !p3d_test_reductibility(G, N[i], node1Pt, node2Pt);
          } else {
            PrintInfo(("Pas de test de reduction\n"));
          }
        }
        if (test == TRUE) {
          loop++;
          if (loop > 1) {
            PrintInfo(("Warning : node multiply used\n"));
          }
          if (ADDED == FALSE) {//ne sert a rien car si on rentre ici ADDED == FALSE
            p3d_insert_node(G, N[i]);
            N[i]->type = LINKING;
            N[i]->only_for_cycle = TRUE;
          }
          if (!p3d_is_node_in_list(N[i]->neighb, node1Pt)) {
            if (!p3d_APInode_linked(G, N[i], node1Pt, &dist)) {
              PrintInfo(("Erreur 1 in link\n"));
              for (j = i; j < nbNodes; j++) {
                p3d_APInode_desalloc(G, N[j]);
                N[j] = NULL;
              }
              MY_FREE(N, p3d_node*, ((int)(nbNodes / 10))*10 + 10);
              *fail = *fail + 1;
              return FALSE;
            }

            if (N[i]->comp != node1Pt->comp) {
              p3d_add_node_compco(N[i], node1Pt->comp);
            }
            p3d_create_edges(G, N[i], node1Pt, dist);
          } else {
            PrintInfo(("Edge already exist!\n"));
          }
          if (!p3d_is_node_in_list(N[i]->neighb, node2Pt)) {
            if (!p3d_APInode_linked(G, N[i], node2Pt, &dist)) {
              PrintInfo(("Erreur 2 in link\n"));
              for (j = i; j < nbNodes; j++) {
                p3d_APInode_desalloc(G, N[j]);
                N[j] = NULL;
              }
              MY_FREE(N, p3d_node*, ((int)(nbNodes / 10))*10 + 10);
              *fail = *fail + 1;
              return FALSE;
            }

            if (N[i]->comp != node2Pt->comp) {
              if (N[i]->numcomp < node2Pt->numcomp) {
                p3d_merge_comp(G, N[i]->comp, &(node2Pt->comp));
                node2Pt->comp = N[i]->comp;
              } else {
                p3d_merge_comp(G, node2Pt->comp, &(N[i]->comp));
                N[i]->comp = node2Pt->comp;
              }
            }
            p3d_create_edges(G, N[i], node2Pt, dist);
          } else {
            PrintInfo(("Edge already exist!\n"));
          }
          ADDED = TRUE;
        }
        if (ADDED == FALSE) {
          p3d_APInode_desalloc(G, N[i]);
          N[i] = NULL;
          *fail = *fail + 1;
        } else {
          list_edge = N[i]->edges;
          while (list_edge != NULL) {
            list_edge->E->for_cycle = TRUE;
            list_edge2 = list_edge->E->Nf->edges;
            while (list_edge2 != NULL) {
              if (list_edge2->E->Nf == N[i]) {
                list_edge2->E->for_cycle = TRUE;
              }
              list_edge2 = list_edge2->next;
            }
            list_edge = list_edge->next;
          }
          PrintInfo(("cycling  : fail : %d\n", *fail));
          ChronoPrint("");
          *fail = 1;
        }
      }
      //end path deform
    }
    MY_FREE(N, p3d_node*, ((int)(nbNodes / 10))*10 + 10);
  }
  return(ADDED);
}


/*******************************************************/
/* Fonction de classement pour les  composantes */
/* connexes                                            */
/* In : les deux noeuds                                */
/* Out : le code du meilleur noeud                     */
/*******************************************************/
int BestNode(void *n1, void *n2) {
  p3d_node *node1, *node2;
  int nedge1, nedge2;
  double dist1, dist2;

  node1 = (p3d_node *)n1;
  node2 = (p3d_node *)n2;

  switch (p3d_get_SORTING()) {

    case P3D_NB_CONNECT:
      nedge1 = node1->nedge;
      nedge2 = node2->nedge;

      return (nedge1 < nedge2) ? 1 : -1; //if n1 >= n2 retrun -1 else 1
      /*(nedge1 > nedge2) ? -1
        : (nedge1 < nedge2) ? 1
        : -1;*/
      break;
    case P3D_DIST_NODE:
      dist1 = node1->dist_Nnew;
      dist2 = node2->dist_Nnew;
      return (dist1 > dist2) ? 1 : -1; //if n1 <= n2 return -1 else 1
      /*(dist1 < dist2) ? -1
          : (dist1 > dist2) ? 1
          : -1;*/
      break;
    default:
      PrintInfo(("BEST_NODE : ERREUR : wrong type of sorting\n"));
      return(0);
  }
}

/* int ebtBestNodeByEdge(void *n1, void *n2) { */
/*   p3d_node *node1, *node2; */

/*   int res; */
/*   node1 = (p3d_node *)n1; */
/*   node2 = (p3d_node *)n2; */
/*   CListEdge1 = node1->orderCostListEdge; */
/*   CListEdge2 = node2->orderCostListEdge; */
/*   res =dbl_list_test_equal(CListEdge1,CListEdge2, costBestEdge); */

/*   if(res >= 0) return 1;  */
/*   return -1;  */
/* } */


/* int sortCostBestNode(void *n1, void *n2) { */
/*   p3d_node *node1, *node2; */
/*   double cost1,cost2; */
/*   node1 = (p3d_node *)n1; */
/*   node2 = (p3d_node *)n2; */
/*   cost1 = node1->cost; */
/*   cost2 = node2->cost; */
/*   return (cost1 < cost2) ? 1 */
/*     : (cost1 > cost2) ? 0 */
/*     : 0; */
/* } */

int sortCostBestEdge(void *e1, void *e2) {
  p3d_edge *edge1, *edge2;
  double cost1, cost2;
  edge1 = (p3d_edge *)e1;
  edge2 = (p3d_edge *)e2;
  cost1 = edge1->cost;
  cost2 = edge2->cost;
  return (cost1 < cost2) ? 1
         : (cost1 > cost2) ? 0
         : 0;
}

/* int costBestNode(void *n1, void *n2) { */
/*   p3d_node *node1, *node2; */
/*   double cost1,cost2; */
/*   node1 = (p3d_node *)n1; */
/*   node2 = (p3d_node *)n2; */
/*   cost1 = node1->cost; */
/*   cost2 = node2->cost; */
/*   return (cost1 < cost2) ? -1 */
/*     : (cost1 > cost2) ? 1 */
/*     : 0; */
/* } */

int costBestEdge(void *e1, void *e2) {
  p3d_edge *edge1, *edge2;
  double cost1, cost2;
  edge1 = (p3d_edge *)e1;
  edge2 = (p3d_edge *)e2;
  cost1 = edge1->cost;
  cost2 = edge2->cost;
  return (cost1 < cost2) ? -1
         : (cost1 > cost2) ? 1
         : 0;
}


/*
  * For Debug... prints the connected components of the graph
  */
void p3d_print_graph_compco(p3d_graph *G) {
  int icomp, incomp;
  p3d_node *Ncomp;
  p3d_compco *comp, *comp_suiv = NULL, *comp_save = NULL;
  p3d_list_node *list_node;

  PrintInfo(("Composantes connexes :\n"));
  comp = G->comp;
  for (icomp = 1;icomp <= G->ncomp;icomp++) {
    comp_suiv = comp->suiv;
    comp_save = comp;
    PrintInfo(("%d ; nombre de noeuds  %d : ", comp->num, comp->nnode));
    list_node = comp->dist_nodes;
    for (incomp = 1;incomp <= comp->nnode;incomp++) {
      Ncomp = list_node->N;
      PrintInfo((" %d(%d) ", Ncomp->num, Ncomp->nedge));
      list_node = list_node->next;
    }
    PrintInfo(("\n"));
    if (comp_suiv != comp->suiv) {
      PrintError(("p3d_global.c: compco changed\n"));
    }
    comp = comp_suiv;
  }
}

void p3d_order_list_node_nofconnex(void) {
  p3d_graph * G;
  p3d_compco * C;
  G = XYZ_GRAPH;

  if (G != NULL) {
    C = G->comp;
    while (C) {
      p3d_order_node_list(C->dist_nodes);
      C = C->suiv;
    }
  }
}

/****************************************************/
/*       traj to graph                              */
/*!\brief convert a trajectory to a graph
  *
  * \param traj the trajectory
  * \param graph the graph
  * \param fct_stop function called for a user stop
  *
  * \return TRUE in case of error or user stop
  *
  * each vertice of a direct path is added into the graph
    as a node. If the direct path is collision free, it
    is added as an edge
  */
/****************************************************/


int p3d_convert_traj_to_graph(p3d_traj *traj, p3d_graph *graph, int (*fct_stop)(void)) {
  configPt q;
  p3d_localpath* path;
  p3d_rob *rob = traj->rob;
  p3d_node *Node0, *Node1;
  double lpathtotal = traj->range_param;
  double lpath = 0;

  PrintInfo(("\nconverting path to roadmap\n"));
  path = traj->courbePt;
  if (!path) {
    return TRUE;
  }

  ChronoOn();
  q = path->config_at_param(rob, path, 0);
  Node0 = p3d_APInode_make(graph, q);
  Node0->type = ISOLATED;
  p3d_insert_node(graph, Node0);
  while (path) {
    if (fct_stop) {
      if (!(*fct_stop)()) {
        PrintInfo(("path convertion canceled\n"));
        return TRUE;
      }
    }
    lpath += path->range_param;
    PrintInfo(("    %7.4f %%           \r", lpath*100 / lpathtotal));
    q = path->config_at_param(rob, path, path->range_param);
    p3d_set_and_update_robot_conf(q);
    if (p3d_col_test()) {
      (graph->nb_test_coll)++;  //Mokhtar
      PrintError(("\n\nERROR: node colliding !!!!\n\n"));
      p3d_destroy_config(rob, q);
      return TRUE;
    }
    Node1 = p3d_APInode_make(graph, q);
    Node1->type = ISOLATED;
    p3d_insert_node(graph, Node1);
    if (!p3d_unvalid_localpath_test(rob, path, &(graph->nb_test_coll))) {  // <-modif Juan
      if (Node0->numcomp < Node1->numcomp) {
        p3d_merge_comp(graph, Node0->comp, &(Node1->comp));
        Node1->comp = Node0->comp;
      } else {
        p3d_merge_comp(graph, Node1->comp, &(Node0->comp));
        Node0->comp = Node1->comp;
      }
      p3d_create_edges(graph, Node0, Node1, path->length_lp);
      Node1->type = LINKING;
    }
    Node0 = Node1;
    path = path->next_lp;
  }
  ChronoPrint("");
  ChronoOff();
  p3d_print_info_graph(graph);
  return FALSE;
}



int p3d_add_all_prm_node(p3d_graph *G, int (*fct_stop)(void)) {
  p3d_node *N = NULL;

  if (fct_stop) {
    if (!(*fct_stop)()) {
      PrintInfo(("basic PRM building canceled\n"));
      return FALSE;
    }
  }
  N = p3d_APInode_shoot(G);
  p3d_insert_node(G, N);
  p3d_all_link_node(N, G);
  if (N->numcomp == -1) {
    /* Node have not been included in a compco, create one for it */
    p3d_create_compco(G, N);
    p3d_merge_check(G);
  }
  return (N != NULL);
}

int p3d_all_link_node(p3d_node* N, p3d_graph* G) {
  p3d_list_node *list_node;
  p3d_node* Nc;
  double dist;
  int n_connect = 0, max_connect = p3d_get_max_connect();
  list_node = G->nodes;

  G->dist_nodes = p3d_add_node_to_list(N, G->dist_nodes);

  if (p3d_get_SORTING() == P3D_DIST_NODE) {
    while (list_node != NULL) {
      if (list_node->N == N) {
        list_node->N->dist_Nnew = P3D_HUGE;
      } else {
        list_node->N->dist_Nnew = p3d_APInode_dist(G, N, list_node->N);
      }
      list_node = list_node->next;
    }
    p3d_order_node_list(G->dist_nodes);
  }
  list_node = G->dist_nodes;

  for (n_connect = 0; (list_node != NULL) && (n_connect < max_connect); n_connect ++) {
    if (list_node->N->num != N->num) {
      Nc = list_node->N;
      if (p3d_APInode_linked(G, N, Nc, &dist)) {
        p3d_create_edges(G, N, Nc, dist);
        /* If the node is still not included in a compco, it will be absorbed in the tested compco*/
        if (N->comp == NULL) {
          p3d_add_node_compco(N, Nc->comp);
        } else {/* Otherwise compcos merge */
          if (Nc->comp->num != N->comp->num) {
            p3d_merge_comp(G, Nc->comp, &(N->comp));
          }
        }
      }
    }
    list_node = list_node->next;
  }
  return TRUE;
}


//start path deform
void del_plot_file(int index) {
  char buf[128], nomfichier[255] = "./plotinfos";
  char* recup, *env_name;

  env_name = p3d_get_desc_curname(P3D_ENV);
  strcat(nomfichier, ".");
  strcat(nomfichier, env_name);
  strcat(nomfichier, ".");
  snprintf(buf, 128, "%d", index);
  recup = strdup(buf);
  strcat(nomfichier, recup);
  unlink(nomfichier);
  free(recup);
}

FILE * open_file_to_save_plot(int index) {
  char buf[128], nomfichier[255] = "./plotinfos";
  char* recup, *env_name;

  env_name = p3d_get_desc_curname(P3D_ENV);
  strcat(nomfichier, ".");
  strcat(nomfichier, env_name);
  strcat(nomfichier, ".");
  snprintf(buf, 128, "%d", index);
  recup = strdup(buf);
  strcat(nomfichier, recup);
  free(recup);
  return fopen(nomfichier, "a");//write at the end of file
}

void close_file_to_save_plot(FILE *OFile) {
  fclose(OFile);
}
/***********************************************/
/* Fonction recuperant dans un fichier les     */
/* donnees issues d'un specific learn          */
/*                                             */
/***********************************************/
static void save_infos_in_file(p3d_graph *G, int sol) {
  char rep[255] = "./saveinfos";
  char nomfichier[255];
  FILE * OFile;
  char *env_name;
  strcpy(nomfichier, rep);
  env_name = p3d_get_desc_curname(P3D_ENV);
  strcat(nomfichier, ".");
  strcat(nomfichier, env_name);
  OFile = fopen(nomfichier, "a");
  PrintInfo(("save_infos_in_file\n"));
  if (!sol) {
    fprintf(OFile, "solution not found \n");
  } else {
    fprintf(OFile, "solution found \n");
  }
  fprintf(OFile, "DD_RRT-Move3D\n");
  fprintf(OFile, "Motion Planner : %d \n", p3d_get_MOTION_PLANNER());
  fprintf(OFile, "Lambda value : %f\n", p3d_GetLambda());
  fprintf(OFile, "Number of nodes : %d\n", G->nnode);
  fprintf(OFile, "Number of boundary nodes : %d\n", G->nboundary);
  fprintf(OFile, "Number of call to the nearest neighbour: %d\n", G->n_call_nearest);
  fprintf(OFile, "Number of collision tests : %d\n", G->nb_test_coll);
  fprintf(OFile, "Number of call to the local method : %d\n", G->nb_local_call);
  fprintf(OFile, "Number of nodes built by standart-RRT : %d\n", G->nb_standart_nodes);
  fprintf(OFile, "Number of nodes built by DD-RRT : %d\n", G->nb_DD_nodes);
  fprintf(OFile, "Number of generated configurations  : %d\n", G->nb_q);
  fprintf(OFile, "Time to build the graph : %f\n", G->time);
  fprintf(OFile, "\n\n");
  fclose(OFile);
}

/*******************************************************/
/* Fonction de classement pour les ebt des composantes */
/* connexes                                            */
/* In : les deux noeuds                                */
/* Out : le code du meilleur noeud                     */
/*******************************************************/
int BestPath(void *n1, void *n2) {
  p3d_path_nodes *node1, *node2;

  node1 = (p3d_path_nodes *)n1;
  node2 = (p3d_path_nodes *)n2;
  //  if  (node1->f <= node2->f)
  //   return -1;
  // return 1;
  return (node1->f < node2->f) ? -1
         : (node1->f > node2->f) ? 1
         : -1;
}
//end path deform
