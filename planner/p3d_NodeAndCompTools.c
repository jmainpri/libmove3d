#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"

/**
 * Note: the integer values of the different
 * NODE_TO_COMP_STRATEGYs are defined in the
 * p3d_type.h file
 */
static int NODE_TO_COMP_STRATEGY = NEAREST_NODE_COMP;

/*
 * Flag to set if there is a maximal distance allowed when
 * finding the node to expand.
 */
static int IS_MAXIMAL_DIST_NEIGHBOR = FALSE;

/**
 * p3d_SetNodeCompStrategy
 * Set the current value of the strategy used to
 * connect a node to a connected componant
 * @param[In] NodeToCompStrategy: the selected strategy.
 */
void p3d_SetNodeCompStrategy(int NodeToCompStrategy)
{
  NODE_TO_COMP_STRATEGY = NodeToCompStrategy;
}

/**
 * p3d_GetNodeCompStrategy
 * Get the value of the current strategy used to
 * connect a node to a connected componant
 * @return: the current strategy to connect a node
 * to a componant.
 */
int p3d_GetNodeCompStrategy(void)
{
  return NODE_TO_COMP_STRATEGY;
}

/**
 * p3d_SetIsMaxDistNeighbor
 * Set if there is a maximal distance allowed when
 * finding the node to expand. Used in the orginal
 * version of DD-RRT to restrict the shooting configurations
 * inside the dynamic domain of the nodes
 * @param[In]: IsMaxDistNeighbor: TRUE if there is a filtering
 * on the maximal distance
 */
void p3d_SetIsMaxDistNeighbor(int IsMaxDistNeighbor)
{
  IS_MAXIMAL_DIST_NEIGHBOR = IsMaxDistNeighbor;
}

/**
 * p3d_GetIsMaxDistNeighbor
 * Get if there is a maximal distance allowed when
 * finding the node to expand. Used in the orginal
 * version of DD-RRT to restrict the shooting configurations
 * inside the dynamic domain of the nodes
 * @return: TRUE if there is a filtering
 * on the maximal distance
 */
int p3d_GetIsMaxDistNeighbor(void)
{
  return IS_MAXIMAL_DIST_NEIGHBOR;
}


/**
 * NearestNeighborComp
 * Select the nearest neighbor of a  componant
 * from a given configuration.
 * @param[In] GraphPt: Pointer to the robot graph
 * @param[In] CompPt: The given connected componant
 * @param[In] Config: The given configuration
 * @return: The nearest node of the componant CompPt
 * note: the distances are computed using the SelectedDistConfig
 * function.
 */
p3d_node* NearestWeightNeighbor(p3d_graph* GraphPt, p3d_compco* CompPt,
                                configPt Config)
{

  p3d_list_node* NodeOfCompListPt;
  p3d_node* BestNodePt =  NULL;
  double  BestScore  = P3D_HUGE;
  double CurrentDist = -1.;
  double CurrentScore = -1.;
  double DistOfBestNode = -1.;
  int SavedDistConfigChoice = -1;
  p3d_matrix4 *RefFramePt = NULL, *MobFramePt = NULL;
  p3d_matrix4 MobFrameRef, invT;
  if (CompPt == NULL) {
    PrintInfo(("Warning: Try to find the nearest neighbor node of a Null comp \n"));
    return NULL;
  }
  /* When retrieving statistics; Commit Jim; date: 01/10/2008 */
  if (getStatStatus()) {
    XYZ_GRAPH->stat->planNeigCall++;
  }
  NodeOfCompListPt = CompPt->dist_nodes;
  //computation of the mobFrameRef of the Config
  if (p3d_GetRefAndMobFrames(GraphPt->rob, &RefFramePt, &MobFramePt)) {
    SavedDistConfigChoice = p3d_GetDistConfigChoice();
    p3d_set_robot_config(GraphPt->rob, Config);
    p3d_update_this_robot_pos_without_cntrt_and_obj(GraphPt->rob);
    p3d_GetRefAndMobFrames(GraphPt->rob, &RefFramePt, &MobFramePt);
    if (RefFramePt == NULL) {
      p3d_mat4Copy(*MobFramePt, MobFrameRef);
    } else {
      p3d_matInvertXform(*RefFramePt, invT);
      p3d_matMultXform(invT, *MobFramePt, MobFrameRef);
    }
  }

  extern int rrtExpansionPhase;
  int * ikSol = NULL;
  if(p3d_get_ik_choice() == IK_UNIQUE && rrtExpansionPhase == true){
      int randomIkSol = (int) floor(p3d_random(0, GraphPt->nbUsedIkSols - EPS6));
      p3d_copy_iksol(GraphPt->rob->cntrt_manager, GraphPt->usedIkSols[randomIkSol], &ikSol);
  }
  
  while (NodeOfCompListPt != NULL) {
    /* We take into account only the nodes undiscarded */
    if (NodeOfCompListPt->N->IsDiscarded == FALSE) {
      if(p3d_get_ik_choice() == IK_UNIQUE){
        if (rrtExpansionPhase == false) {
          if (ikSol) {
            MY_FREE(ikSol, int, GraphPt->rob->cntrt_manager->ncntrts);//IkSol Vector is allocated in the copy
            ikSol = NULL;
          }
          p3d_copy_iksol(GraphPt->rob->cntrt_manager, NULL, &ikSol);
        }
        if(!p3d_compare_iksol(GraphPt->rob->cntrt_manager, ikSol, NodeOfCompListPt->N->iksol)){
          NodeOfCompListPt = NodeOfCompListPt->next;
          continue;
        }
        if (rrtExpansionPhase == false) {
          MY_FREE(ikSol, int, GraphPt->rob->cntrt_manager->ncntrts);
          ikSol = NULL;
        }
      }
      
      if (p3d_GetDistConfigChoice() == MOBILE_FRAME_DIST) {
        CurrentDist = p3d_GetSe3DistanceFrames(GraphPt->rob, MobFrameRef,
                                               NodeOfCompListPt->N->RelMobFrame);
      } else {
        CurrentDist = SelectedDistConfig(GraphPt->rob, Config,
                                         NodeOfCompListPt->N->q);
      }
      if (p3d_GetIsWeightedChoice() == TRUE) {
        CurrentScore = CurrentDist * p3d_GetNodeWeight(NodeOfCompListPt->N);
      } else {
        CurrentScore = CurrentDist;
      }
      if (CurrentScore < BestScore) {
        BestScore = CurrentScore;
        BestNodePt = NodeOfCompListPt->N;
        DistOfBestNode = CurrentDist;
      }
    }
    NodeOfCompListPt = NodeOfCompListPt->next;
  }
  if(p3d_get_ik_choice() == IK_UNIQUE && rrtExpansionPhase == true){
    MY_FREE(ikSol, int, GraphPt->rob->cntrt_manager->ncntrts);
  }
  
  if (SavedDistConfigChoice != -1) {
    ENV.setInt(Env::DistConfigChoice, SavedDistConfigChoice);
  }
  if ((p3d_GetIsMaxDistNeighbor() == TRUE) &&
      (BestNodePt->boundary == TRUE) &&
      (BestNodePt->radius < DistOfBestNode)) {
    /* There is a maximal distance allowed to get a node as neighbor */
    return NULL;
  }
  return BestNodePt;
}


/**
 * KNearestNeighborComp
 * Select randomly a node among the Knearest neighbor
 * of a  componant from a given configuration.
 * @param[In] GraphPt: Pointer to the robot graph
 * @param[In] CompPt: The given connected componant
 * @param[In] Config: The given configuration
 * @param[In] KNearest: the number of nodes considered
 * @return: A  node choosed randomly among the K
 * nearest nodes of the componant CompPt
 * note: the distances are computed using the SelectedDistConfig
 * function.
 */
p3d_node* KNearestWeightNeighbor(p3d_graph* GraphPt, p3d_compco* CompPt,
                                 configPt Config, int KNearest)
{

  p3d_list_node* NodeOfCompListPt;
  p3d_node* BestNodePt =  NULL;
  double CurrentDist = -1.;
  double CurrentScore = -1.;
  double DistOfBestNode = -1.;
  int SavedDistConfigChoice = -1;
  p3d_matrix4 *RefFramePt = NULL, *MobFramePt = NULL;
  p3d_matrix4 MobFrameRef, invT;
  int i, j, NbNearNodes, RandNodeNum;
  p3d_node **ArrayNearNodes = NULL;
  double *ArrayScorNearNodes = NULL;
  double*ArrayDistNearNodes = NULL;
  if ((CompPt == NULL) || (KNearest < 1)) {
    PrintInfo(("Warning :  Try to find the nearest neighbor node of \
               a NULL comp  or Knearest == 0 \n"));
    return NULL;
  }
  if (getStatStatus()) {
    XYZ_GRAPH->stat->planNeigCall++;
  }
  NodeOfCompListPt = CompPt->dist_nodes;
  //computation of the mobFrameRef of the Config
  if (p3d_GetRefAndMobFrames(GraphPt->rob, &RefFramePt, &MobFramePt)) {
    SavedDistConfigChoice = p3d_GetDistConfigChoice();
    p3d_set_robot_config(GraphPt->rob, Config);
    p3d_update_this_robot_pos_without_cntrt_and_obj(GraphPt->rob);
    p3d_GetRefAndMobFrames(GraphPt->rob, &RefFramePt, &MobFramePt);
    if (RefFramePt == NULL) {
      p3d_mat4Copy(*MobFramePt, MobFrameRef);
    } else {
      p3d_matInvertXform(*RefFramePt, invT);
      p3d_matMultXform(invT, *MobFramePt, MobFrameRef);
    }
  }

  ArrayNearNodes = MY_ALLOC(p3d_node*, KNearest);
  ArrayScorNearNodes = MY_ALLOC(double, KNearest);
  ArrayDistNearNodes = MY_ALLOC(double, KNearest);
  for (i = 0; i < KNearest; i++) {
    ArrayNearNodes[i] = NULL;
    ArrayScorNearNodes[i] = P3D_HUGE;
    ArrayDistNearNodes[i] = P3D_HUGE;
  }
  while (NodeOfCompListPt != NULL) {
    /* We take into account only the nodes undiscarded */
    if (NodeOfCompListPt->N->IsDiscarded == FALSE) {
      if (p3d_GetDistConfigChoice() == MOBILE_FRAME_DIST) {
        CurrentDist = p3d_GetSe3DistanceFrames(GraphPt->rob, MobFrameRef,
                                               NodeOfCompListPt->N->RelMobFrame);
      } else {
        CurrentDist = SelectedDistConfig(GraphPt->rob, Config,
                                         NodeOfCompListPt->N->q);
      }
      if (p3d_GetIsWeightedChoice() == TRUE) {
        CurrentScore = CurrentDist * p3d_GetNodeWeight(NodeOfCompListPt->N);
      } else {
        CurrentScore = CurrentDist;
      }
      if (CurrentScore < ArrayScorNearNodes[KNearest-1]) {
        i = 0;
        while (CurrentScore > ArrayScorNearNodes[i])
          i++;
        for (j = KNearest - 1; j > i; j--) {
          ArrayNearNodes[j] = ArrayNearNodes[j-1];
          ArrayScorNearNodes[j] = ArrayScorNearNodes[j-1];
          ArrayDistNearNodes[j] = ArrayDistNearNodes[j-1];
        }
        ArrayNearNodes[i] = NodeOfCompListPt->N;
        ArrayScorNearNodes[i] = CurrentScore;
        ArrayDistNearNodes[i] = CurrentDist;
      }
    }
    NodeOfCompListPt = NodeOfCompListPt->next;
  }
  NbNearNodes = 0;
  while ((NbNearNodes < KNearest) &&
         (ArrayNearNodes[NbNearNodes] != NULL)) {
    NbNearNodes++;
  }
  if (NbNearNodes > 0) {
    RandNodeNum = (int) floor(p3d_random(0.0, (double)(NbNearNodes - EPS6)));
    BestNodePt = ArrayNearNodes[RandNodeNum];
    DistOfBestNode =  ArrayDistNearNodes[RandNodeNum];
  } else {
    PrintInfo(("Warning: failed to find an expansion node in the %d best \
               nodes select randomly among all the componant \n", KNearest));
    RandNodeNum = (int) floor(p3d_random(0.0, (double)(CompPt->nnode - EPS6)));
    NodeOfCompListPt = CompPt->dist_nodes;
    for (i = 0; i < RandNodeNum; i++) {
      NodeOfCompListPt = NodeOfCompListPt->next;
    }
    BestNodePt = NodeOfCompListPt->N;
    if (p3d_GetDistConfigChoice() == MOBILE_FRAME_DIST) {
      DistOfBestNode  = p3d_GetSe3DistanceFrames(GraphPt->rob, MobFrameRef,
                        BestNodePt->RelMobFrame);
    } else {
      DistOfBestNode  = SelectedDistConfig(GraphPt->rob, Config,
                                           BestNodePt->q);
    }
  }
  MY_FREE(ArrayNearNodes, p3d_node *, KNearest);
  MY_FREE(ArrayScorNearNodes, double, KNearest);
  MY_FREE(ArrayDistNearNodes, double, KNearest);
  if (SavedDistConfigChoice != -1) {
    ENV.setInt(Env::DistConfigChoice, SavedDistConfigChoice);
  }
  if ((p3d_GetIsMaxDistNeighbor() == TRUE) &&
      (BestNodePt->boundary == TRUE) &&
      (BestNodePt->radius < DistOfBestNode)) {
    /* There is a maximal distance allowed to get a node as neighbor */
    return NULL;
  }
  return BestNodePt;
}

/**
 * p3d_LinkNodesMergeComp
 * Try to link two nodes by testing the local path
 * from Node1Pt to Node2Pt. If the two nodes are linkable,
 * add a double edge Node1Pt->Node2Pt and Node2Pt>Node1Pt
 * and merge the two componants.
 * @param[In] GraphPt: Pointer to the robot graph
 * @param[In] Node1Pt: The first node
 * @param[In] Node2Pt: The second node
 * @return: TRUE if the two nodes have been linked and the
 * componants merged
 * Warning; this function is based on the p3d_APInodelinked
 * function which doesn't test the reverse local path
 * Node2Pt>Node1Pt. Thus this function can currently
 * be used only for reversible local paths
 */
int p3d_LinkNodesMergeComp(p3d_graph* GraphPt,
                           p3d_node* Node1Pt,
                           p3d_node* Node2Pt)
{
  int  IsLinkedAndMerged = FALSE;
  double DistNodes = 0.;
  p3d_compco* CompNode1Pt = NULL;
  p3d_compco* CompNode2Pt = NULL;

  if ((Node1Pt == NULL) || (Node2Pt == NULL) || (GraphPt == NULL)) {
    PrintInfo(("Warning: Try to link two nodes with NULL structures \n"));
    return FALSE;
  }
  CompNode1Pt = Node1Pt->comp;
  CompNode2Pt = Node2Pt->comp;

  if (p3d_APInode_linked(GraphPt, Node1Pt, Node2Pt, &DistNodes)) {
    if (CompNode1Pt->num < CompNode2Pt->num)
      p3d_merge_comp(GraphPt, CompNode1Pt, &CompNode2Pt);
    else if (CompNode1Pt->num > CompNode2Pt->num) {
      p3d_merge_comp(GraphPt, CompNode2Pt, &CompNode1Pt);
    }
    p3d_create_edges(GraphPt, Node1Pt, Node2Pt, DistNodes);
    //    PrintInfo(("dist: %f\n",DistNodes));
    IsLinkedAndMerged = TRUE;
  }
  return IsLinkedAndMerged;
}

/**
 * p3d_ConnectNodeToComp
 * Try to connect a given node to a given connected componant
 * using the current connection strategy of a node to a componant
 * @param[In] GraphPt: Pointer to the robot graph
 * @param[In] Node1ToConnectPt: The node to connect to the componant
 * @param[In] CompToConnectPt: The componant we want to connect
 * to the node
 * @return: TRUE if connection of the node to the componant succeeded
 */
int p3d_ConnectNodeToComp(p3d_graph* GraphPt,
                          p3d_node* Node1ToConnectPt,
                          p3d_compco* CompToConnectPt)
{
  int SavedIsMaxDis = FALSE;
  int SavedIsWeightChoice = FALSE;
  int IsConnectSuccess = FALSE;
  p3d_node* Node2ToConnectPt = NULL;
  //  double ratio = 1./5.;

  if ((GraphPt == NULL) ||
      (Node1ToConnectPt == NULL) || (CompToConnectPt == NULL)) {
    PrintInfo(("Warning: Try to connect a node to a comp \
               with NULL structures\n"));
    return FALSE;
  }
  if (Node1ToConnectPt->comp->num == CompToConnectPt->num) {
    PrintInfo(("Warning: Try to connect a Node to its own componant \n"));
    return TRUE;
  }
  switch (p3d_GetNodeCompStrategy()) {
    case NEAREST_NODE_COMP:
      /* Connect the node to the nearest node of the comp */

      SavedIsMaxDis =  p3d_GetIsMaxDistNeighbor();
      SavedIsWeightChoice = p3d_GetIsWeightedChoice();
      p3d_SetIsMaxDistNeighbor(FALSE);
      p3d_SetIsWeightedChoice(FALSE);
      Node2ToConnectPt =  NearestWeightNeighbor(GraphPt,
                          CompToConnectPt,
                          Node1ToConnectPt->q);

      p3d_SetIsMaxDistNeighbor(SavedIsMaxDis);
      p3d_SetIsWeightedChoice(SavedIsWeightChoice);

      if (Node2ToConnectPt == NULL) {
        if (p3d_get_ik_choice() == IK_NORMAL) {
          PrintInfo(("Warning: Failed to find a nearest node in the Componant to connect\n"));
        }
        return FALSE;
      }

      /*     if(SelectedDistConfig(GraphPt->rob, Node1ToConnectPt->q, Node2ToConnectPt->q) >  */
      /*        ratio*SelectedDistConfig(GraphPt->rob, GraphPt->search_start->q, GraphPt->search_goal->q)){ */
      /*       return FALSE; */
      /*     } */
      IsConnectSuccess = p3d_LinkNodesMergeComp(GraphPt,
                         Node2ToConnectPt,
                         Node1ToConnectPt);
      break;
    case K_NEAREST_NODE_COMP:
      /*Connect randomly to one of the k nearest
       nodes of the componant */
      /*todo*/
      break;
    default:
      /* By default  try to
         connect to the node to the nearest node of the componant */
      SavedIsMaxDis =  p3d_GetIsMaxDistNeighbor();
      SavedIsWeightChoice = p3d_GetIsWeightedChoice();
      p3d_SetIsMaxDistNeighbor(FALSE);
      p3d_SetIsWeightedChoice(FALSE);
      Node2ToConnectPt =  NearestWeightNeighbor(GraphPt,
                          CompToConnectPt,
                          Node1ToConnectPt->q);
      p3d_SetIsMaxDistNeighbor(SavedIsMaxDis);
      p3d_SetIsWeightedChoice(SavedIsWeightChoice);

      if (Node2ToConnectPt == NULL) {
        PrintInfo(("Warning: Failed to find a nearest node in the Componant to connect\n"));
        return FALSE;
      }
      /*    if(SelectedDistConfig(GraphPt->rob, Node1ToConnectPt->q, Node2ToConnectPt->q) >  */
      /*        ratio*SelectedDistConfig(GraphPt->rob, GraphPt->search_start->q, GraphPt->search_goal->q)){ */
      /*       return FALSE; */
      /*    } */
      IsConnectSuccess = p3d_LinkNodesMergeComp(GraphPt,
                         Node2ToConnectPt,
                         Node1ToConnectPt);
  }
  return IsConnectSuccess;

}

/**
 * p3d_RandomNodeFromComp
 * Select randomly a node inside a connect componant
 * @param[In]: a pointer to the given connect componant
 * @return: The selected node
 */
p3d_node* p3d_RandomNodeFromComp(p3d_compco* CompPt)
{
  int RandNodeNum = -1;
  p3d_list_node* ListNode = NULL;
  int i;

  if (CompPt == NULL) {
    PrintInfo(("Warning: Try to choose a randomly a\
               node from a NULL comp"));
    return NULL;
  }
  ListNode = CompPt->dist_nodes;
  RandNodeNum = (int)floor(p3d_random(0.0, (double)CompPt->nnode - EPS6));
  for (i = 0; i < RandNodeNum; i++) {
    ListNode = ListNode->next;
  }
  return  ListNode->N;
}

/**
 * p3d_listNodeInDist
 * TODO
 */
p3d_list_node* p3d_listNodeInDist(p3d_rob* robotPt, p3d_compco* compPt, p3d_node* nodePt, double dist)
{
  p3d_list_node* listCompNodePt = compPt->nodes, *listDistNodePt = NULL;
  while (listCompNodePt != NULL) {
    if ((listCompNodePt->N != nodePt) &&
        (SelectedDistConfig(robotPt, listCompNodePt->N->q, nodePt->q) < dist)) {
      listDistNodePt =  p3d_add_node_to_list(listCompNodePt->N, listDistNodePt);
    }
    listCompNodePt = listCompNodePt->prev;
  }
  return listDistNodePt;

}
