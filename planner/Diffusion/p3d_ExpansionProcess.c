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
#include "Localpath-pkg.h"
#include "Collision-pkg.h"


extern double  InitCostThreshold;

/**
 * Value of the extension step parameter
 */
static double EXTENSION_STEP_PARAM = 3.;
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
  p3d_add_node_compco(newNodePt, expansionNodePt->comp, TRUE);
  newNodePt->rankFromRoot = expansionNodePt->rankFromRoot + 1;

  //cost updates
  if (ENV.getBool(Env::isCostSpace) == TRUE) {
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
  if (ENV.getInt(Env::ExpansionNodeMethod) == RANDOM_IN_SHELL_METH)  {
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
  //ikSol Treatement
  if (p3d_get_ik_choice() == IK_UNIQUE) {
    p3d_AddIkSolInArray(graphPt->rob->cntrt_manager, newNodePt->iksol, graphPt->usedIkSols, &graphPt->nbUsedIkSols);
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
  //int** IndexConstrSoluPt = NULL;
  int * ikSol = NULL, *ikSolTmp = NULL;
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

  if (p3d_get_ik_choice() == IK_UNIQUE) {
    p3d_copy_iksol(robotPt->cntrt_manager, ExpansionNodePt->iksol, &ikSol);
    if(ExpansionNodePt->isSingularity){//choose an iksol different from neigbourg ikSols
      for (int i = 0; i < robotPt->cntrt_manager->ncntrts; i++) {
        if (ikSol[i] < 0) {
          //check witch class we have to explore.
          int validSol = TRUE;
          do{
            ikSol[i] = p3d_get_random_ikSol(robotPt->cntrt_manager, i);
            for (p3d_list_node * nl = ExpansionNodePt->neighb ; nl; nl = nl->next) {
              if(nl->N->iksol[i] == ikSol[i]){
                validSol = FALSE;
                break;
              }else {
                validSol = TRUE;
              }
            }
            if(!validSol && robotPt->cntrt_manager->cntrts[i]->nSingularities){
              int differentIkSols = 0;
              int ** iksolsArray = MY_ALLOC(int*, ExpansionNodePt->nneighb);
              for (p3d_list_node * nl = ExpansionNodePt->neighb ; nl; nl = nl->next){
                for (p3d_list_node * nl2 = nl->next ; nl2; nl2 = nl2->next){
                  int allreadyCompared = false;
                  for (int i = 0; i < differentIkSols; i++) {
                    if(p3d_compare_iksol(robotPt->cntrt_manager, iksolsArray[i], nl2->N->iksol)){
                      allreadyCompared = true;
                    }
                  }
                  if(!allreadyCompared && !p3d_compare_iksol(robotPt->cntrt_manager, nl->N->iksol, nl2->N->iksol)){
                    iksolsArray[differentIkSols] = nl2->N->iksol;
                    differentIkSols++; 
                  }
                }
              }
              if (robotPt->cntrt_manager->cntrts[i]->nSingularities <= differentIkSols) {
                return FALSE;
              }
            }
          }while(!validSol);          
        }
      }
    }
    p3d_copy_iksol(robotPt->cntrt_manager, ikSol, &ikSolTmp);
  }
  
  LocalPathPt = p3d_local_planner_multisol(robotPt,
                                  ExpansionNodePt->q, DirectionConfig, ikSolTmp);
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
    if (ENV.getInt(Env::ExpansionNodeMethod) == RANDOM_IN_SHELL_METH)  {
      p3d_SetNGood(0);
    }
    LocalPathPt->destroy(robotPt, LocalPathPt);
    p3d_destroy_config(robotPt, NewConfig);
    ExpansionNodePt->n_fail_extend++;
    if ((ENV.getBool(Env::discardNodes) == true) &&
        (ExpansionNodePt != GraphPt->search_start) &&
        (ExpansionNodePt != GraphPt->search_goal) &&
        (ExpansionNodePt->n_fail_extend > ENV.getInt(Env::MaxExpandNodeFail))) {
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
  if (ENV.getBool(Env::isCostSpace) == true) {
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
      if (ENV.getInt(Env::ExpansionNodeMethod) == RANDOM_IN_SHELL_METH)  {
        p3d_SetNGood(0);
      }
      p3d_destroy_config(robotPt, NewConfig);
      ExpansionNodePt->n_fail_extend++;
      if ((ENV.getBool(Env::discardNodes) == true) &&
          (ExpansionNodePt != GraphPt->search_start) &&
          (ExpansionNodePt != GraphPt->search_goal) &&
          (ExpansionNodePt->n_fail_extend > ENV.getInt(Env::MaxExpandNodeFail))) {
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
  ikSolTmp = NULL;

  NewNodePt = p3d_APInode_make_multisol(GraphPt, NewConfig, ikSol);
  p3d_node * singularNode = NULL;
  int singularDist = -1;
  if(p3d_get_ik_choice() == IK_UNIQUE){
    
    p3d_set_and_update_robot_conf_multisol(NewNodePt->q, ikSol);
    configPt tmpConfig = p3d_get_robot_config(robotPt);
    p3d_destroy_config(robotPt, NewNodePt->q);
    NewNodePt->q = tmpConfig;
    p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, NewNodePt->q);
//    g3d_draw_allwin_active();
    
    int singNum = 0;
    int cntrt = p3d_isCloseToSingularityConfig(robotPt, robotPt->cntrt_manager, NewNodePt->q, &singNum);
    if (cntrt != -1){
      p3d_isCloseToSingularityConfig(robotPt, robotPt->cntrt_manager, NewNodePt->q, &singNum);
      //Transform to singular configuration
      configPt singConfig = p3d_alloc_config(robotPt);
      do{
        p3d_APInode_shoot_singularity(robotPt, &singConfig, &singNum, &cntrt, NewNodePt->q, NewNodePt->iksol);
        p3d_unmark_for_singularity(robotPt->cntrt_manager, cntrt);
      }while (p3d_col_test());
      
      ikSol[cntrt] = -(singNum + 1);
      //test connexion between ExpansionNodePt and the singular config
      LocalPathPt = p3d_local_planner_multisol(robotPt, NewNodePt->q, singConfig, ikSol);
      if (LocalPathPt == NULL) {
        PrintInfo(("Failed to create a localpath during singularity creation\n"));
      }else{
        (GraphPt->nb_local_call)++;
        int ntest;
        if(!p3d_unvalid_localpath_test(robotPt, LocalPathPt, &ntest)){
          //if collision free create singular node
          singularNode = p3d_APInode_make_multisol(GraphPt, singConfig, ikSol);
          singularNode->isSingularity = TRUE;
          singularDist = LocalPathPt->length_lp;
          LocalPathPt->destroy(robotPt, LocalPathPt);
          ikSol = NULL;
          printf("singular configuration added to the tree\n");
        }else {//else test connexion between NewNodePt and singular config
          LocalPathPt->destroy(robotPt, LocalPathPt);
        }
      }
    }
  }
  
  if (NewNodePt == NULL) {
    if (ENV.getInt(Env::ExpansionNodeMethod) == RANDOM_IN_SHELL_METH)  {
      p3d_SetNGood(0);
    }
    p3d_destroy_config(robotPt, NewConfig);
    ExpansionNodePt->n_fail_extend++;
    if ((ENV.getBool(Env::discardNodes) == TRUE) &&
        (ExpansionNodePt != GraphPt->search_start) &&
        (ExpansionNodePt != GraphPt->search_goal) &&
        (ExpansionNodePt->n_fail_extend > ENV.getInt(Env::MaxExpandNodeFail))) {
      ExpansionNodePt->IsDiscarded = TRUE;
      update_parent_nfails(ExpansionNodePt);
      GraphPt->n_consec_pb_level ++;
      GraphPt->n_consec_fail_pb_level ++;
    }
    return FALSE;
  }

  p3d_AddNodeUpdateGraphStruc(GraphPt, NewNodePt, ExpansionNodePt,
                              ExpansionDist, CurrentCost);
  if (p3d_get_ik_choice() == IK_UNIQUE) {
    if (singularNode) {
      CurrentCost = p3d_GetConfigCost(GraphPt->rob, singularNode->q);
      p3d_AddNodeUpdateGraphStruc(GraphPt, singularNode, NewNodePt,
                                  singularDist, CurrentCost);
    }
  }
  //Additional cycles through edges addition if the flag is active
  if (ENV.getBool(Env::addCycles) == TRUE) {
    DMax =  p3d_get_env_dmax();
    ExtendStepParam = ENV.getDouble(Env::extensionStep);
    step = DMax * ExtendStepParam;
    listDistNodePt = p3d_listNodeInDist(GraphPt->rob,
                                        NewNodePt->comp, NewNodePt, kFactor * step);

    savedListDistNodePt = listDistNodePt;
    while (listDistNodePt != NULL) {
      if (!p3d_IsSmallDistInGraph(GraphPt, NewNodePt, listDistNodePt->N, ENV.getInt(Env::maxCostOptimFailures), step)) {
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
  double DistPath;
  p3d_node* NewNodePt;
  //int** IndexConstrSoluPt = NULL;
  double DeltaPath;
  double PreviousCost, CurrentCost;
  int IsRaisingCost;
  int IsTooMuchRefin;
  p3d_rob* robotPt = GraphPt->rob;
  double distNodes;
  p3d_list_node* destroyListNodePt, * savedListDistNodePt, *listDistNodePt;
  double longStep, kFactor = 3;


  //if the sampled direction is the goal, try to directly connect
  //the expansion node to the goal (avoid the supperposition of
  // newNode and Goal
  if ((GraphPt->IsCurrentNodeBias == TRUE) &&
      (ENV.getBool(Env::isCostSpace) == false)) {
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
  if (((*IsReachedPt) == TRUE) && (ENV.getBool(Env::expandControl) == true)) {
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
    p3d_destroy_config(GraphPt->rob, newConfig);
    return FALSE;
  }

  // newConf collision test
  (GraphPt->nb_test_coll)++;
  if (p3d_col_test()) {
    p3d_destroy_config(GraphPt->rob, newConfig);
    return FALSE;
  }
  //take into account the constraints
  p3d_destroy_config(GraphPt->rob, newConfig);
  newConfig = p3d_get_robot_config(GraphPt->rob);

  GraphPt->nb_q_free = GraphPt->nb_q_free + 1;

  // in cost spaces additionnal transition test based on
  // costs
  if (ENV.getBool(Env::isCostSpace) == true) {
    PreviousCost = ExpansionNodePt->cost;
    CurrentCost = p3d_GetConfigCost(GraphPt->rob, newConfig);
    if (ExpansionNodePt->numcomp == 1) {
      IsRaisingCost = FALSE;
    } else {
      IsRaisingCost = TRUE;
    }
    if (ENV.getBool(Env::findLowCostConf)) {
      if (CurrentCost < ENV.getDouble(Env::findLowCostThreshold) || (ENV.getInt(Env::tRrtNbtry) > 500)) {
        printf("EndCost = %f\n", CurrentCost);
        p3d_SetStopValue(TRUE);
      }else{
        if(ENV.getDouble(Env::bestCost) - CurrentCost > 0){
          ENV.setInt(Env::tRrtNbtry, 0);
          ENV.setDouble(Env::bestCost, CurrentCost);
        }else {
          ENV.setInt(Env::tRrtNbtry, ENV.getInt(Env::tRrtNbtry) + 1);
        }
      }
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
  if (!ENV.getBool(Env::findLowCostConf) && p3d_unvalid_localpath_test(GraphPt->rob, LocalPathPt,
                                 &(GraphPt->nb_test_coll))) {

    //in case of failure, must update some graph structures
    if (ENV.getInt(Env::ExpansionNodeMethod) == RANDOM_IN_SHELL_METH)  {
      p3d_SetNGood(0);
    }
    LocalPathPt->destroy(GraphPt->rob, LocalPathPt);
    p3d_destroy_config(GraphPt->rob, newConfig);
    ExpansionNodePt->n_fail_extend++;
    if ((ENV.getBool(Env::discardNodes) == true) &&
        (ExpansionNodePt != GraphPt->search_start) &&
        (ExpansionNodePt != GraphPt->search_goal) &&
        (ExpansionNodePt->n_fail_extend > ENV.getInt(Env::MaxExpandNodeFail))) {
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
  //p3d_get_iksol_vector(GraphPt->rob->cntrt_manager,&IndexConstrSoluPt);

  //attempt the new node creation
  // WARNING: iksol does not work, a new vector containing the 1st solution of each constraint must be passed as argument
  NewNodePt = p3d_APInode_make_multisol(GraphPt, newConfig, NULL);
  if(NewNodePt == NULL) {  
    if(ENV.getInt(Env::ExpansionNodeMethod) == RANDOM_IN_SHELL_METH)  {
      p3d_SetNGood(0);
    }
    p3d_destroy_config(GraphPt->rob, newConfig);
    ExpansionNodePt->n_fail_extend++;
    if ((ENV.getBool(Env::discardNodes) == true) &&
        (ExpansionNodePt != GraphPt->search_start) &&
        (ExpansionNodePt != GraphPt->search_goal) &&
        (ExpansionNodePt->n_fail_extend > ENV.getInt(Env::MaxExpandNodeFail))) {
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
  if (ENV.getBool(Env::addCycles) == true) {
    longStep = kFactor * Step;
    listDistNodePt = p3d_listNodeInDist(GraphPt->rob,
                                        NewNodePt->comp, NewNodePt, longStep);

    savedListDistNodePt = listDistNodePt;
    while (listDistNodePt != NULL) {
      if (!p3d_IsSmallDistInGraph(GraphPt, NewNodePt, listDistNodePt->N, ENV.getInt(Env::maxCostOptimFailures), Step)) {
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
  switch (ENV.getExpansionMethod()) {
    case Env::Connect:
      NbCreatedNodes  = ExpandOneNodeWithConnect(GraphPt, ExpansionNodePt,
                        DirectionConfig);
      //p3d_EvaluateExpandDiffic(ExpansionNodePt->comp, NbCreatedNodes);
      break;
    case Env::nExtend:
      DMax =  p3d_get_env_dmax();
      ExtendStepParam = ENV.getDouble(Env::extensionStep);
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
    case Env::Extend:
      DMax =  p3d_get_env_dmax();
      ExtendStepParam = ENV.getDouble(Env::extensionStep);
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

  if (ENV.getBool(Env::isCostSpace) && (p3d_GetCostMethodChoice() == MAXIMAL_THRESHOLD)) {
    p3d_updateCostThreshold();
  }

  return NbCreatedNodes;
}

