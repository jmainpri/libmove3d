#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"


extern double  InitCostThreshold;

/**
 * Note: the integer values of the different
 * EXPANSION_CHOICEs are defined in the
 * p3d_type.h file
 */
static int EXPANSION_CHOICE = EXTEND_EXP_CHOICE;

/**
 * Value of the extension step parameter
 */
static double EXTENSION_STEP_PARAM = 3.;

/**
 * p3d_GetExtendStepParam
 * Get the value of the extension step
 * parameter. the step of expansion in the extend
 * method is then equal to  ExtendStepParam*Dmax
 * @return: the extension step parameter
 */
double p3d_GetExtendStepParam(void) {
  return EXTENSION_STEP_PARAM;
}

/**
 * p3d_SetExtendStepParam
 * Set the value of the extension step
 * parameter. the step of expansion in the extend
 * method is then equal to  ExtendStepParam*Dmax
 * @param[In] ExtendStepParam: the extension step parameter
 */
void p3d_SetExtendStepParam(double ExtendStepParam) {
  EXTENSION_STEP_PARAM = ExtendStepParam;
}

/**
 * p3d_SetExpansionChoice
 * Set the current value of the method used to
 * process the expansion of a node toward
 * a direction configuration selected as direction
 * of expansion
 * @param[In] the expansion process choice
 */
void p3d_SetExpansionChoice(int ExpansionChoice) {
  EXPANSION_CHOICE = ExpansionChoice;
}

/**
 * p3d_GetExpansionChoice
 * Get the value of the current method used to
 * process the expansion of a node toward
 * a direction configuration selected as direction
 * of expansion
 * @return: the current expansion process choice
 */
int p3d_GetExpansionChoice(void) {
  return EXPANSION_CHOICE;
}

/**
 * p3d_AddNodeUpdateGraphStruc
 * Insert the node in the graph create the edge
 *
 */
void p3d_AddNodeUpdateGraphStruc(p3d_graph* graphPt, p3d_node* newNodePt,
                                 p3d_node* expansionNodePt,
                                 double expansionDist, double currentCost) {

  p3d_rob* robotPt = graphPt->rob;
  double StopWeight;
  int SignStopWeight;

  //the New Node is added to the graph
  newNodePt->type = LINKING;
  p3d_insert_node(graphPt, newNodePt);
  p3d_create_edges(graphPt, expansionNodePt, newNodePt, expansionDist);
  p3d_add_node_compco(newNodePt, expansionNodePt->comp);
  newNodePt->rankFromRoot = expansionNodePt->rankFromRoot + 1;

  //cost updates
  if (p3d_GetIsCostFuncSpace() == TRUE) {
    p3d_SetNodeCost(graphPt, newNodePt, currentCost);
    //for adaptive variant, new temp is refreshed except if it is going down.
    if (currentCost < expansionNodePt->cost) {
      newNodePt->temp = expansionNodePt->temp;
    } else {
      newNodePt->temp = expansionNodePt->temp / 2.;
    }
  }

  //dynamic domains update
  if (p3d_GetExpansionDirectionMethod() == SUBREGION_CS_EXP) {
    p3d_ResizeDynDomain(robotPt, newNodePt);
  }

  //weight updates
  if (p3d_GetIsWeightedChoice() == TRUE) {
    p3d_SetNodeWeight(graphPt, newNodePt);
  }

  //check stop conditions
  if (p3d_GetIsWeightStopCondition() == TRUE) {
    p3d_GetStopWeightAndSign(&StopWeight, &SignStopWeight);
    if (SignStopWeight == 1) {
      if (newNodePt->weight >= StopWeight) {
        p3d_SetStopValue(TRUE);
        p3d_SetDiffuStoppedByWeight(1);
      }
    } else if (SignStopWeight == -1) {
      if (newNodePt->weight <= StopWeight) {
        p3d_SetStopValue(TRUE);
        p3d_SetDiffuStoppedByWeight(1);
      }
    }
  }

  // Graph updates for RANDOM_IN_SHELL method
  if (p3d_GetExpansionNodeMethod() == RANDOM_IN_SHELL_METH)  {
    p3d_SetNGood(p3d_GetNGood() + 1);
    if (newNodePt->weight > graphPt->CurPbLevel) {
      graphPt->CurPbLevel = newNodePt->weight;

      graphPt->n_consec_pb_level = 0;
      graphPt->n_consec_fail_pb_level = 0;
      if (p3d_GetNGood() > 2)
        graphPt->critic_cur_pb_level = graphPt->CurPbLevel;
    } else {
      graphPt->n_consec_pb_level ++;
      graphPt->n_consec_fail_pb_level = 0;
    }
  }
}

/**
 * @brief ExpandOneNodeWithConnect
 * Expand a the tree by creating a  single node toward
 * the configuration choosen as direction of expansion with
 * the connect principle (i.e. testing the local path until
 * a collision is found  or the configuration is reached)
 * @param[In] GraphPt: Pointer to the robot graph
 * @param[In] ExpansionNodePt: the node to expand
 * @param[In] DirectionConfig: the direction of expansion
 * return: TRUE if a new node has been created FALSE otherwise
 */
static int ExpandOneNodeWithConnect(p3d_graph *GraphPt,
                                    p3d_node *ExpansionNodePt,
                                    configPt DirectionConfig) {

  p3d_rob* robotPt = GraphPt->rob;
  p3d_node* NewNodePt = NULL;
  p3d_localpath *LocalPathPt = NULL;
  configPt NewConfig;
  double DeltaPath = -1.;
  double ExpansionDist = -1.;
  int** IndexConstrSoluPt = NULL;
  double PreviousCost, CurrentCost;
  int IsRaisingCost;
  double distNodes;
  p3d_list_node* destroyListNodePt, * savedListDistNodePt, *listDistNodePt;
  double step, DMax, ExtendStepParam;
  double  kFactor = 3.;

//if the sampled direction is the goal, try to directly connect
  //the expansion node to the goal (avoid the supperposition of
  // newNode and Goal
  if (GraphPt->IsCurrentNodeBias == TRUE) {
    if (p3d_LinkNodesMergeComp(GraphPt, ExpansionNodePt,
                               GraphPt->NodeBiasPt)) {
      return FALSE;
    }
  }

  LocalPathPt = p3d_local_planner(robotPt,
                                  ExpansionNodePt->q, DirectionConfig);
  if (LocalPathPt == NULL) {
    PrintInfo(("Failed to create a localpath during the expansion process\n"));
    return FALSE;
  }

  (GraphPt->nb_local_call)++;

  NewConfig = p3d_alloc_config(robotPt);
  p3d_unvalid_localpath_classic_test(robotPt, LocalPathPt,
                                     &(GraphPt->nb_test_coll), &DeltaPath,
                                     &NewConfig);
  if (DeltaPath == 0) {
    if (p3d_GetExpansionNodeMethod() == RANDOM_IN_SHELL_METH)  {
      p3d_SetNGood(0);
    }
    LocalPathPt->destroy(robotPt, LocalPathPt);
    p3d_destroy_config(robotPt, NewConfig);
    ExpansionNodePt->n_fail_extend++;
    if ((p3d_GetIsMaxExpandNodeFail() == TRUE) &&
        (ExpansionNodePt != GraphPt->search_start) &&
        (ExpansionNodePt != GraphPt->search_goal) &&
        (ExpansionNodePt->n_fail_extend > p3d_GetMaxExpandNodeFail())) {
      ExpansionNodePt->IsDiscarded = TRUE;
      update_parent_nfails(ExpansionNodePt);
      GraphPt->n_consec_pb_level ++;
      GraphPt->n_consec_fail_pb_level ++;
    }
    if (ExpansionNodePt->boundary == FALSE) {
      ExpansionNodePt->boundary = TRUE;
      GraphPt->nboundary++;
    }
    return FALSE;
  }
  GraphPt->nb_q_free = GraphPt->nb_q_free + 1;
  (GraphPt->nb_local_call)++;

  //additional test for cost spaces
  if (p3d_GetIsCostFuncSpace() == TRUE) {
    PreviousCost = ExpansionNodePt->cost;
    CurrentCost = p3d_GetConfigCost(GraphPt->rob, NewConfig);

    // IsRaisingCost is FALSE when we are expanding the InitComp:
    // Downhill slopes have better chances to be accepted. If
    // we are expanding the GoalComp tests are inversed: Uphill
    //slopes have better chances to be accepted
    if (ExpansionNodePt->numcomp == 1) {
      IsRaisingCost = FALSE;
    } else {
      IsRaisingCost = TRUE;
    }
    // Main transition test used in CostSpaces
    if (CostTestSucceeded(GraphPt, ExpansionNodePt,
                          NewConfig, PreviousCost,
                          CurrentCost, IsRaisingCost) == FALSE) {
      if (p3d_GetExpansionNodeMethod() == RANDOM_IN_SHELL_METH)  {
        p3d_SetNGood(0);
      }
      p3d_destroy_config(robotPt, NewConfig);
      ExpansionNodePt->n_fail_extend++;
      if ((p3d_GetIsMaxExpandNodeFail() == TRUE) &&
          (ExpansionNodePt != GraphPt->search_start) &&
          (ExpansionNodePt != GraphPt->search_goal) &&
          (ExpansionNodePt->n_fail_extend > p3d_GetMaxExpandNodeFail())) {
        ExpansionNodePt->IsDiscarded = TRUE;
        update_parent_nfails(ExpansionNodePt);
        GraphPt->n_consec_pb_level ++;
        GraphPt->n_consec_fail_pb_level ++;
      }
      return FALSE;
    }
  }

  ExpansionDist = DeltaPath * LocalPathPt->length_lp;
  LocalPathPt->destroy(robotPt, LocalPathPt);

  p3d_get_iksol_vector(robotPt->cntrt_manager,&IndexConstrSoluPt);
  // WARNING: iksol does not work, a new vector containing the 1st solution of each constraint must be passed as argument
  NewNodePt = p3d_APInode_make_multisol(GraphPt, NewConfig, NULL);

  if (NewNodePt == NULL) {
    if (p3d_GetExpansionNodeMethod() == RANDOM_IN_SHELL_METH)  {
      p3d_SetNGood(0);
    }
    p3d_destroy_config(robotPt, NewConfig);
    ExpansionNodePt->n_fail_extend++;
    if ((p3d_GetIsMaxExpandNodeFail() == TRUE) &&
        (ExpansionNodePt != GraphPt->search_start) &&
        (ExpansionNodePt != GraphPt->search_goal) &&
        (ExpansionNodePt->n_fail_extend > p3d_GetMaxExpandNodeFail())) {
      ExpansionNodePt->IsDiscarded = TRUE;
      update_parent_nfails(ExpansionNodePt);
      GraphPt->n_consec_pb_level ++;
      GraphPt->n_consec_fail_pb_level ++;
    }
    return FALSE;
  }

  p3d_AddNodeUpdateGraphStruc(GraphPt, NewNodePt, ExpansionNodePt,
                              ExpansionDist, CurrentCost);

  //Additional cycles through edges addition if the flag is active
  if (p3d_GetIsCycles() == TRUE) {
    DMax =  p3d_get_env_dmax();
    ExtendStepParam = p3d_GetExtendStepParam();
    step = DMax * ExtendStepParam;
    listDistNodePt = p3d_listNodeInDist(GraphPt->rob,
                                        NewNodePt->comp, NewNodePt, kFactor * step);

    savedListDistNodePt = listDistNodePt;
    while (listDistNodePt != NULL) {
      if (!p3d_IsSmallDistInGraph(GraphPt, NewNodePt, listDistNodePt->N, p3d_GetNbFailOptimCostMax(), step)) {
        //should add a cost test
        distNodes = p3d_dist_q1_q2(GraphPt->rob, NewNodePt->q, listDistNodePt->N->q);
        PrintInfo(("create a cycle edge\n"));
        p3d_create_edges(GraphPt, NewNodePt, listDistNodePt->N, distNodes);
        NewNodePt->edges->E->for_cycle = TRUE;
      }
      listDistNodePt = listDistNodePt->next;
    }
    while (savedListDistNodePt != NULL) {
      destroyListNodePt = savedListDistNodePt;
      savedListDistNodePt = savedListDistNodePt->next;
      MY_FREE(destroyListNodePt, p3d_list_node, 1);
    }
  }

  return TRUE;
}

/**
 * ExpandOneNodeWithExtend
 * Expand a the tree by creating a  single node toward
 * the configuration choosen as direction of expansion with
 * the extend principle (i.e. testing a local path of a given step)
 * @param[In]: GraphPt: Pointer to the robot graph
 * @param[In]: ExpansionNodePt: the node to expand
 * @param[In]: DirectionConfig: the direction of expansion
 * @param[In]: Step: the step of expension  in DirectionConfig direction
 * return: TRUE if a new node has been created FALSE otherwise
 */

static int ExpandOneNodeWithExtend(p3d_graph *GraphPt,
                                   p3d_node *ExpansionNodePt,
                                   configPt DirectionConfig, double Step, int* IsReachedPt) {

  configPt newConfig;
  p3d_localpath* LocalPathPt;
  int i, j, k;
  double DistPath;
  p3d_node* NewNodePt;
  int** IndexConstrSoluPt = NULL;
  double DeltaPath;
  double PreviousCost, CurrentCost;
  int IsRaisingCost;
  int IsTooMuchRefin;
  p3d_jnt* jntPt;
  p3d_rob* robotPt = GraphPt->rob;
  double distNodes;
  p3d_list_node* destroyListNodePt, * savedListDistNodePt, *listDistNodePt;
  double longStep, kFactor = 3;


  //if the sampled direction is the goal, try to directly connect
  //the expansion node to the goal (avoid the supperposition of
  // newNode and Goal
  if ((GraphPt->IsCurrentNodeBias == TRUE) &&
      (p3d_GetIsCostFuncSpace() == FALSE)) {
    if (p3d_LinkNodesMergeComp(GraphPt, ExpansionNodePt,
                               GraphPt->NodeBiasPt)) {
      return FALSE;
    }
  }

  LocalPathPt =  p3d_local_planner(GraphPt->rob, ExpansionNodePt->q, DirectionConfig);
  
  // computation of the path expansion step
//   newConfig  = p3d_alloc_config(GraphPt->rob);
  DeltaPath = Step / LocalPathPt->length_lp;
  if (DeltaPath < 1.) {
    *IsReachedPt = FALSE;
  } else {
    /* We limit  deltaPath to [0,1] i.e.:
       the expansion can not go further than DirectionConfig */
    DeltaPath = 1.;
    *IsReachedPt = TRUE;
  }


  //in cost spaces the number of refinement nodes (when the expansion
  //reach the dir config) is limited to 50%
  if (((*IsReachedPt) == TRUE) && (p3d_GetIsExpandControl() == TRUE)) {
    IsTooMuchRefin = ((double)ExpansionNodePt->comp->nbRefinNodes * 2 >
                      ((double)ExpansionNodePt->comp->nnode));
    if (IsTooMuchRefin == TRUE) {
//       p3d_destroy_config(GraphPt->rob, newConfig);
      return FALSE;
    }
  }

  // in cost spaces in case of expansion to the goal,
  // the attempt of goal connection is only done if the
  // expansion step reach the goal. It avoids unvalid
  // long edges to the goal
  if (((*IsReachedPt) == TRUE) && (GraphPt->IsCurrentNodeBias == TRUE)) {
    if (p3d_LinkNodesMergeComp(GraphPt, ExpansionNodePt,
                               GraphPt->NodeBiasPt)) {
      return FALSE;
    }
  }

  //Computation of the new configuration by linear interpolation
  // between ExpansionNodePt and DirectionConfig
  /*warning assumes that the local method is linear ! */
  newConfig = LocalPathPt->config_at_param(robotPt, LocalPathPt, DeltaPath*LocalPathPt->length_lp);
//   for (i = 0; i <= robotPt->njoints; i++) {
//     jntPt = robotPt->joints[i];
//     for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
//       k = jntPt->index_dof + j;
//       if (p3d_jnt_get_dof_is_user(jntPt, j)) {
//         if (p3d_jnt_is_dof_circular(jntPt, j)) {
//           newConfig[k] = ExpansionNodePt->q[k] +
//                          DeltaPath * diff_angle(ExpansionNodePt->q[k], DirectionConfig[k]);
//         } else {
//           newConfig[k] = ExpansionNodePt->q[k] +
//                          DeltaPath * (DirectionConfig[k] - ExpansionNodePt->q[k]);
//         }
//       } else {
//         newConfig[k] = ExpansionNodePt->q[k];
//       }
//     }
//   }
LocalPathPt->destroy(robotPt, LocalPathPt);
  // newConf kinematic validity test
  GraphPt->nb_q = GraphPt->nb_q + 1;
  if (!p3d_set_and_update_robot_conf(newConfig)) {
    PrintInfo(("update Failed during extend step in \
               an expansion comp process\n"));
    p3d_destroy_config(GraphPt->rob, newConfig);
    return FALSE;
  }

  // newConf collision test
  (GraphPt->nb_test_coll)++;
  if (p3d_col_test()) {
    p3d_destroy_config(GraphPt->rob, newConfig);
    return FALSE;
  }
  GraphPt->nb_q_free = GraphPt->nb_q_free + 1;

  // in cost spaces additionnal transition test based on
  // costs
  if (p3d_GetIsCostFuncSpace() == TRUE) {
    PreviousCost = ExpansionNodePt->cost;
    CurrentCost = p3d_GetConfigCost(GraphPt->rob, newConfig);
    if (ExpansionNodePt->numcomp == 1) {
      IsRaisingCost = FALSE;
    } else {
      IsRaisingCost = TRUE;
    }
    if (CostTestSucceeded(GraphPt, ExpansionNodePt, newConfig,
                          PreviousCost, CurrentCost,
                          IsRaisingCost)  == FALSE) {
      p3d_destroy_config(GraphPt->rob, newConfig);
      return FALSE;
    }
  }

  // construction of the local path
  (GraphPt->nb_local_call)++;
  LocalPathPt =  p3d_local_planner(GraphPt->rob, ExpansionNodePt->q, newConfig);
  if (LocalPathPt == NULL) {
    PrintInfo(("Error: failed to create a localpath\n"));
    p3d_SetStopValue(TRUE);
    return FALSE;
  }

  // local path collision validity test
  if (p3d_unvalid_localpath_test(GraphPt->rob, LocalPathPt,
                                 &(GraphPt->nb_test_coll))) {

    //in case of failure, must update some graph structures
    if (p3d_GetExpansionNodeMethod() == RANDOM_IN_SHELL_METH)  {
      p3d_SetNGood(0);
    }
    LocalPathPt->destroy(GraphPt->rob, LocalPathPt);
    p3d_destroy_config(GraphPt->rob, newConfig);
    ExpansionNodePt->n_fail_extend++;
    if ((p3d_GetIsMaxExpandNodeFail() == TRUE) &&
        (ExpansionNodePt != GraphPt->search_start) &&
        (ExpansionNodePt != GraphPt->search_goal) &&
        (ExpansionNodePt->n_fail_extend > p3d_GetMaxExpandNodeFail())) {
      ExpansionNodePt->IsDiscarded = TRUE;
      update_parent_nfails(ExpansionNodePt);
      GraphPt->n_consec_pb_level ++;
      GraphPt->n_consec_fail_pb_level ++;
    }
    if (ExpansionNodePt->boundary == FALSE) {
      ExpansionNodePt->boundary = TRUE;
      GraphPt->nboundary++;
    }
    return FALSE;
  }
  DistPath = LocalPathPt->length_lp;
  LocalPathPt->destroy(GraphPt->rob, LocalPathPt);
  p3d_get_iksol_vector(GraphPt->rob->cntrt_manager,&IndexConstrSoluPt);

  //attempt the new node creation
  // WARNING: iksol does not work, a new vector containing the 1st solution of each constraint must be passed as argument
  NewNodePt = p3d_APInode_make_multisol(GraphPt, newConfig, NULL);
  if(NewNodePt == NULL) {  
    if(p3d_GetExpansionNodeMethod() == RANDOM_IN_SHELL_METH)  { 
      p3d_SetNGood(0);
    }
    p3d_destroy_config(GraphPt->rob, newConfig);
    ExpansionNodePt->n_fail_extend++;
    if ((p3d_GetIsMaxExpandNodeFail() == TRUE) &&
        (ExpansionNodePt != GraphPt->search_start) &&
        (ExpansionNodePt != GraphPt->search_goal) &&
        (ExpansionNodePt->n_fail_extend > p3d_GetMaxExpandNodeFail())) {
      ExpansionNodePt->IsDiscarded = TRUE;
      update_parent_nfails(ExpansionNodePt);
      GraphPt->n_consec_pb_level ++;
      GraphPt->n_consec_fail_pb_level ++;
    }
    return FALSE;
  }

  p3d_AddNodeUpdateGraphStruc(GraphPt, NewNodePt, ExpansionNodePt,
                              DeltaPath, CurrentCost);

  //Additional cycles through edges addition if the flag is active
  if (p3d_GetIsCycles() == TRUE) {
    longStep = kFactor * Step;
    listDistNodePt = p3d_listNodeInDist(GraphPt->rob,
                                        NewNodePt->comp, NewNodePt, longStep);

    savedListDistNodePt = listDistNodePt;
    while (listDistNodePt != NULL) {
      if (!p3d_IsSmallDistInGraph(GraphPt, NewNodePt, listDistNodePt->N, p3d_GetNbFailOptimCostMax(), Step)) {
        //should add a cost test
        distNodes = p3d_dist_q1_q2(GraphPt->rob, NewNodePt->q, listDistNodePt->N->q);
        PrintInfo(("create a cycle edge\n"));
        p3d_create_edges(GraphPt, NewNodePt, listDistNodePt->N, distNodes);
        NewNodePt->edges->E->for_cycle = TRUE;
      }
      listDistNodePt = listDistNodePt->next;
    }
    while (savedListDistNodePt != NULL) {
      destroyListNodePt = savedListDistNodePt;
      savedListDistNodePt = savedListDistNodePt->next;
      MY_FREE(destroyListNodePt, p3d_list_node, 1);
    }
  }
  return TRUE;
}

/**
 * ExpandProcess
 *  General function expanding a node toward a direction
 * of expansion. The mode of expansion depends of the
 * expansion choice selected.
 * @param[In] GraphPt: Pointer to the robot graph
 * @param[In] ExpansionNodePt: the node to expand
 * @param[In] DirectionConfig: the direction of expansion
 * @return: the number of nodes created during the expansion
 * process (several nodes can be created during one expansion)
 */
int ExpandProcess(p3d_graph *GraphPt, p3d_node* ExpansionNodePt,
                  configPt DirectionConfig) {
  int NbCreatedNodes = 0;
  int IsReached = FALSE;
  double DMax = -1.;
  double ExtendStepParam = -1.;
  int IsNewNode = TRUE;
  p3d_node* CurExpanNodePt = NULL;

  if (ExpansionNodePt == NULL) {
    PrintInfo(("Warning: try to expand a NULL node\n"));
    return FALSE;
  }
  switch (p3d_GetExpansionChoice()) {
    case ONE_NODE_CONNECT_EXP_CHOICE:
      NbCreatedNodes  = ExpandOneNodeWithConnect(GraphPt, ExpansionNodePt,
                        DirectionConfig);
      //p3d_EvaluateExpandDiffic(ExpansionNodePt->comp, NbCreatedNodes);
      break;
    case N_NODES_EXTEND_EXP_CHOICE:
      DMax =  p3d_get_env_dmax();
      ExtendStepParam = p3d_GetExtendStepParam();
      CurExpanNodePt = ExpansionNodePt;
      while ((IsNewNode == TRUE) && (IsReached == FALSE)) {
        IsNewNode = ExpandOneNodeWithExtend(GraphPt, CurExpanNodePt,
                                            DirectionConfig, DMax * ExtendStepParam, &IsReached);
        //p3d_EvaluateExpandDiffic(ExpansionNodePt->comp, NbCreatedNodes);
        if (IsNewNode == TRUE) {
          CurExpanNodePt = GraphPt->last_node->N;
          NbCreatedNodes++;
        } else {
          GraphPt->last_node->N->boundary = TRUE;
        }
      }
      break;
    case EXTEND_EXP_CHOICE:
      DMax =  p3d_get_env_dmax();
      ExtendStepParam = p3d_GetExtendStepParam();
      NbCreatedNodes  = ExpandOneNodeWithExtend(GraphPt, ExpansionNodePt,
                        DirectionConfig, DMax * ExtendStepParam, &IsReached);

      p3d_EvaluateExpandDiffic(ExpansionNodePt->comp, NbCreatedNodes);

      if (NbCreatedNodes == 0) {
        ExpansionNodePt->boundary = TRUE;
      } else {
        if (IsReached == TRUE) {
          (ExpansionNodePt->comp->nbRefinNodes)++;
        } else {
        }
      }
      break;
    default:
      /* by default expand one node with connect */
      NbCreatedNodes  = ExpandOneNodeWithConnect(GraphPt, ExpansionNodePt,
                        DirectionConfig);
      //p3d_EvaluateExpandDiffic(ExpansionNodePt->comp, NbCreatedNodes);
      if (NbCreatedNodes == 0) {
        ExpansionNodePt->boundary = TRUE;
      }
  }

  if (p3d_GetIsCostFuncSpace() && (p3d_GetCostMethodChoice() == MAXIMAL_THRESHOLD)) {
    p3d_updateCostThreshold();
  }

  return NbCreatedNodes;
}

