#include "Planner-pkg.h"
#include "Collision-pkg.h"
#include "P3d-pkg.h"
#include "Localpath-pkg.h"
#include "Bio-pkg.h"
#include "Util-pkg.h"
#include "GroundHeight-pkg.h"

void* GroundCostObj;
double InitCostThreshold = -1.;
int ThresholdDown = 3;
double AverQsQgCost;


/**
 * Param that can be used for several things.
 * Currently used to weight distance and 
 * configuration costs in the dist+cost method
 * and for bio applications, to weigth the cost of 
 *  rotations  from a reference config with the cost of 
 * the ligand protein distance.   
 */
double Alpha = 0.5;

extern double ZminEnv;
extern double ZmaxEnv;

/* static param
 * correspond to the current cost method
 *  used for exploration in cost spaces.
 */
static int CostMethodChoice = TRANSITION_RRT;


/* Flag to set the temperature (by analogy with
 * simulated annealing methods) witch is used to set 
 * the threshold to the CostTestSucceeded function.
 */  
//static double Temperature = 10.e-6;

static double Temperature = 0.000001;

/*  value of the number of consecutive 
 * failures of the transition test. Used to
 * automatically tune the temperature parameter
 */
static int GlobalNbFailTemp =0;

/* Number of consecutive times the extension
 * is made toward downhill slopes
 * Warning: Currently ths parameter is no
 * more used
 */
//static  int GlobalNbDown = 0;

/* Value of the CostThreshold.
 * Used in the RRT_Obst literature algorithm.
 */
static double CostThreshold = HUGE_VAL;

/* IsLocalCostAdapt
 * static param set to TRUE if the adatpation 
 * of the success/failed extension is local
 */
static int IsLocalCostAdapt = FALSE;

/* DelatCostChoice
 * Give the value of the current method
 * to set motion cost (ie. portion of path cost)
 * in function of the cost of the extremal configurations
 * if 0: mechanical Work of the motion  
 * if 1: average cost between the 2 extremal configs 
 */
int DeltaCostChoice = MECHANICAL_WORK;


/**
 * p3d_GetCostMethodChoice
 * Get the method used for exploration in cost spaces.
 * @return: the method used for exploration in cost spaces.
 */
int p3d_GetCostMethodChoice(void) {
  return CostMethodChoice;
}

/**
 * p3d_GetAlphaValue
 * @return: the alpha value
 */
double p3d_GetAlphaValue(void) {
  return Alpha;
}

/**
 * p3d_SetAlphaValue
 * @param[In] alpha: the alpha value to set
 */
void p3d_SetAlphaValue(double alpha) {
  Alpha = alpha;
}

/**
 * p3d_GetIsLocalCostAdapt
 * Get if the adatpation of the success/failed extension 
 * is local or global  
 * @return: TRUE if the adaptation is local.
 */
int p3d_GetIsLocalCostAdapt(void) {
  return IsLocalCostAdapt;
}

/**
 * p3d_SetIsLocalCostAdapt
 * Set if the adatpation of the success/failed extension 
 * is local or global  
 * @param[In] isLocalCostAdapt: TRUE if the adaptation 
 * is local.
 */
void p3d_SetIsLocalCostAdapt(int isLocalCostAdapt) {
  IsLocalCostAdapt = isLocalCostAdapt;
}

/**
 * p3d_GetTemperatureParam
 * Get the value of the temperature parameter
 * this parameter is used to set the threshold in the
 * CostTestSucceeded function
 * @return: the value of the temperature parameter
 */
double p3d_GetTemperatureParam(void) {
  return Temperature;
}

/**
 * p3d_SetTemperatureParam
 * Set the value of the temperature parameter
 * this parameter is used to set the threshold in the
 * CostTestSucceeded function
 * @param[In] temperature: the value of the temperature parameter
 */
void p3d_SetTemperatureParam(double temperature) {
  Temperature = temperature;
}

/**
 * p3d_InitSpaceCostParam
 * Initialize some parameters for the planner
 * for space cost environments
 * @param[In] GraphPt: the robot graph
 * @param[In] Ns: Initial node of the graph
 * @param[In] Ng: Goal node of the graph
 */
void p3d_InitSpaceCostParam(p3d_graph* GraphPt, p3d_node* Ns, p3d_node* Ng) {
  Ns->temp = ENV.getDouble(Env::initialTemperature);
  Ns->comp->temperature = ENV.getDouble(Env::initialTemperature);
  GlobalNbFailTemp = 0;
  Ns->nbFailedTemp = 0;
  //  GlobalNbDown = 0;
  //  Ns->NbDown = 0;
  p3d_SetNodeCost(GraphPt, Ns, p3d_GetConfigCost(GraphPt->rob, Ns->q));
  p3d_SetCostThreshold(Ns->cost);
  InitCostThreshold = p3d_GetNodeCost(Ns);
  if(ENV.getBool(Env::expandToGoal) && (Ng != NULL)) {
    Ng->temp = Ns->temp = ENV.getDouble(Env::initialTemperature);
    Ng->comp->temperature = ENV.getDouble(Env::initialTemperature);
    Ng->nbFailedTemp = 0;
    //    Ng->NbDown = 0;
    p3d_SetNodeCost(GraphPt, Ng, p3d_GetConfigCost(GraphPt->rob, Ng->q));
    p3d_SetCostThreshold(MAX(p3d_GetNodeCost(Ns), p3d_GetNodeCost(Ng) ));
    InitCostThreshold = MAX(p3d_GetNodeCost(Ns), p3d_GetNodeCost(Ng) );
    AverQsQgCost = (GraphPt->rob->GRAPH->search_start->cost +
		    GraphPt->rob->GRAPH->search_goal->cost)/2.;
  } else {
    p3d_SetCostThreshold(Ns->cost);
    InitCostThreshold = Ns->cost;
    AverQsQgCost =  GraphPt->rob->GRAPH->search_start->cost;
  }
}


/**
 * p3d_GetCostThreshold
 * Get the value of the CostThreshold.
 * Used in the RRT_Obst literature algorithm.
 * @return: the value of the CostThreshold.
 */
double p3d_GetCostThreshold(void) {
  return CostThreshold;
}

/**
 * p3d_SetCostThreshold
 * Set the value of the CostThreshold.
 * Used in the RRT_Obst literature algorithm.
 * @param[In] costThreshold: the value of the CostThreshold.
 */
void p3d_SetCostThreshold(double costThreshold) {
  CostThreshold = costThreshold;
}

/*
 * p3d_updateCostThreshold
 * increases the cost threshold by a given amount
 * in the MAXIMAL_THRESHOLD (Ettlin/Bleuer) variant
 */
void p3d_updateCostThreshold(void) {
  p3d_SetCostThreshold(p3d_GetCostThreshold() + 
		       InitCostThreshold/ENV.getInt(Env::maxCostOptimFailures));
}

/**
 * p3d_GetNodeCost
 * Get the cost of a node 
 * given by the Space cost function  
 * @param[In] NodePt:  the considered node
 * @return: the cost of the node
 */
double p3d_GetNodeCost(p3d_node* NodePt) {
  return NodePt->cost;
}

/**
 * p3d_SetNodeCost
 * Set the cost of a node   
 * @param[In] G: the robot Graph 
 * @param[In] NodePt: pointer to th given node
 * @param[In] Cost: the cost to set to the node
 */
void p3d_SetNodeCost(p3d_graph* G, p3d_node* NodePt, double cost) {

  NodePt->cost = cost;  
  if(NodePt->parent != NULL) {
       NodePt->sumCost = NodePt->parent->sumCost+cost;
  } else {
     NodePt->sumCost = cost; 
  }
  NodePt->comp->minCost = MIN(cost,NodePt->comp->minCost );
  NodePt->comp->maxCost = MAX(cost,NodePt->comp->maxCost );
  if(p3d_GetCostMethodChoice() ==URMSON_TRANSITION ) {
    NodePt->comp->maxUrmsonCost = MAX(NodePt->sumCost + 
				      p3d_ComputeUrmsonCostToGoal(G,NodePt) ,NodePt->comp->maxUrmsonCost);
  }
  //  PrintInfo(("Mincost : %f\n",NodePt->comp->minCost ));
}

/**
 * p3d_GetConfigCost
 * Get the cost of a configuration 
 * given by the Space cost function  
 * @param[In] robotPt: the current robotPt
 * @param[In] ConfPt: the considered configuration
 * @return: the cost of the configuration
 */
double p3d_GetConfigCost(p3d_rob* robotPt, configPt ConfPt) {
  configPt QSaved;
  double Cost, cost1, cost2, alpha;
  int val;
  if(GroundCostObj) {
    val= GHintersectionVerticalLineWithGround(GroundCostObj, 
					      ConfPt[6], 
					      ConfPt[7], &Cost);
  } else if (p3d_col_get_mode() != p3d_col_mode_bio) {
    QSaved = p3d_get_robot_config(robotPt);
    p3d_set_and_update_robot_conf(ConfPt);    
     Cost = p3d_GetMinDistCost(robotPt);
     //Cost = p3d_GetAverageDistCost(robotPt);
    p3d_set_and_update_robot_conf(QSaved);
    p3d_destroy_config(robotPt, QSaved);
    //  PrintInfo(("cost = %f\n\n",Cost));
  }
  else if(is_ligand_in_robot(robotPt) == TRUE) { 
    QSaved = p3d_get_robot_config(robotPt);
    p3d_set_and_update_robot_conf(ConfPt);    
    //    
    if(1) {
      Cost = bioGetInvMinLigProtDist();
    } else {
      alpha = p3d_GetAlphaValue();
      cost1 = bioGetInvMinLigProtDist();
      cost2 = bioGetLigRotaMotionFromInit(robotPt, ConfPt);
      Cost = alpha*cost1 +(1-alpha)*cost2;
    }
    p3d_set_and_update_robot_conf(QSaved);
    p3d_destroy_config(robotPt, QSaved);
  } else {
    PrintInfo(("Case not yet implemented\n"));
    Cost = 0.;
  }
  return Cost;
}

/**
 * p3d_CostConnectNodeToComp
 * Try to connect a node to a given component 
 * taking into account the fact that the space 
 * is a cost space
 * @param[In] GraphPt: the robot graph
 * @param[In] NodePt: the node to connect
 * @param[In] CompPt: the componant to connect with
 * @return: TRUE if the node and the componant have 
 * been connected.
 */
int p3d_CostConnectNodeToComp(p3d_graph* GraphPt, p3d_node* NodePt, 
			      p3d_compco* CompPt) {
  int SavedIsMaxDis = FALSE;
  int SavedIsWeightChoice = FALSE;
  p3d_node* Node2Pt = NULL;
  double Dist;
  int SavedExpansionChoice;
  int SavedIsNodeBias;
  p3d_node* SavedNodeBias;
  int IsCompConnectToNode = FALSE;
  double ratio = 1./10.;

  if((GraphPt == NULL) ||
     (NodePt == NULL) || (CompPt == NULL)) {
    PrintInfo (("Warning: Try to connect a node to a comp \
with NULL structures\n"));
    return FALSE;
  }
  if(NodePt->comp->num == CompPt->num) {
    PrintInfo (("Warning: Try to connect a Node to its own componant \n"));
    return TRUE;
  }
  switch(p3d_GetNodeCompStrategy()) {
  case NEAREST_NODE_COMP: 
    
    SavedIsMaxDis =  p3d_GetIsMaxDistNeighbor();
    SavedIsWeightChoice = p3d_GetIsWeightedChoice();
    p3d_SetIsMaxDistNeighbor(FALSE);
    p3d_SetIsWeightedChoice(FALSE);
    Node2Pt =  NearestWeightNeighbor(GraphPt, 
				     CompPt, 
				     NodePt->q);
    
    p3d_SetIsMaxDistNeighbor(SavedIsMaxDis);
    p3d_SetIsWeightedChoice(SavedIsWeightChoice);
    
    if(Node2Pt == NULL) {
      PrintInfo (("Warning: Failed to find a nearest node in \
the Componant to connect\n"));
      return FALSE;
    }
     if(SelectedDistConfig(GraphPt->rob, NodePt->q, Node2Pt->q) >  
        ratio*SelectedDistConfig(GraphPt->rob, GraphPt->search_start->q, GraphPt->search_goal->q)){ 
       return FALSE; 
     } 
    if(p3d_APInode_linked(GraphPt, NodePt, Node2Pt, &Dist)) {
      SavedExpansionChoice = ENV.getExpansionMethod();
      ENV.setExpansionMethod(Env::nExtend);
      
      SavedIsNodeBias = GraphPt->IsCurrentNodeBias;
      GraphPt->IsCurrentNodeBias = TRUE;
      
      SavedNodeBias = GraphPt->NodeBiasPt;
      GraphPt->NodeBiasPt = NodePt;
      ExpandProcess(GraphPt, Node2Pt, NodePt->q);
	    
      ENV.setExpansionMethod(Env::nExtend);
      GraphPt->IsCurrentNodeBias = SavedIsNodeBias;
      GraphPt->NodeBiasPt = SavedNodeBias;
    }
    if(NodePt->comp->num == CompPt->num) {
      IsCompConnectToNode = TRUE;
    } else {
      IsCompConnectToNode = FALSE;
    }
    break;
  case K_NEAREST_NODE_COMP:
    /*Connect randomly to one of the k nearest
      nodes of the componant */
    /*todo*/
    break;
  default:
    /* By default idem than  NEAREST_NODE_COMP*/
    SavedIsMaxDis =  p3d_GetIsMaxDistNeighbor();
    SavedIsWeightChoice = p3d_GetIsWeightedChoice();
    p3d_SetIsMaxDistNeighbor(FALSE);
    p3d_SetIsWeightedChoice(FALSE);
    Node2Pt =  NearestWeightNeighbor(GraphPt, 
				     CompPt, 
				     NodePt->q);
    
    p3d_SetIsMaxDistNeighbor(SavedIsMaxDis);
    p3d_SetIsWeightedChoice(SavedIsWeightChoice);
    
    if(Node2Pt == NULL) {
      PrintInfo (("Warning: Failed to find a nearest node in \
the Componant to connect\n"));
      return FALSE;
    }
    if(SelectedDistConfig(GraphPt->rob, NodePt->q, Node2Pt->q) > 
       ratio*SelectedDistConfig(GraphPt->rob, GraphPt->search_start->q, GraphPt->search_goal->q)){
      return FALSE;
    }
    if(p3d_APInode_linked(GraphPt, NodePt, Node2Pt, &Dist)) {
      Env::expansionMethod SavedExpansionChoice = ENV.getExpansionMethod();
      
      SavedIsNodeBias = GraphPt->IsCurrentNodeBias;
      GraphPt->IsCurrentNodeBias = TRUE;
      
      SavedNodeBias = GraphPt->NodeBiasPt;
      GraphPt->NodeBiasPt = NodePt;
      ExpandProcess(GraphPt, Node2Pt, NodePt->q);
	    
      ENV.setExpansionMethod(SavedExpansionChoice);
      //p3d_SetExpansionChoice(SavedExpansionChoice);
      GraphPt->IsCurrentNodeBias = SavedIsNodeBias;
      GraphPt->NodeBiasPt = SavedNodeBias;
    }
    if(NodePt->comp->num == CompPt->num) {
      IsCompConnectToNode = TRUE;
    } else {
      IsCompConnectToNode = FALSE;
    }
  }
  return IsCompConnectToNode;
}



/**
 * p3d_ComputeNodeSolutionCost
 * Compute an approximation of the cost of the solution path
 * using only the nodes of this path
 * the cost is normalised by dividing by the total number of nodes
 * @param[In] GraphPt: pointer the robot graph 
 * @return: the cost of the solution path, -1 if the path doesn't exist.
 */ 
double p3d_ComputeNodeSolutionCost(p3d_graph* GraphPt) {
  int NbNodes = 0;
  double CurrentCost, TotalCost;
  p3d_node *bckw_path;
  configPt qCurrent; 
  if(GraphPt == NULL) {  
    PrintInfo(("Warning: Failed to compute the cost of a solution path: \
as no graph has been found\n"));
    return -1.;
  } 
  qCurrent = p3d_get_robot_config(GraphPt->rob);
  CurrentCost= p3d_GetConfigCost(GraphPt->rob, qCurrent);
    PrintInfo(("Current config cost : %f\n", CurrentCost));
    p3d_destroy_config(GraphPt->rob, qCurrent);

  if (p3d_graph_search(GraphPt, p3d_heurist, p3d_valid, p3d_end, DEFAULTGRAPH) == FALSE) {
    PrintInfo(("Warning: Failed to compute the cost of a solution path: \
as no soltion solution path has been found\n"));
    return -1.;
  }
  bckw_path = GraphPt->search_goal;
  CurrentCost= p3d_GetConfigCost(GraphPt->rob, bckw_path->q);
  TotalCost = CurrentCost;
  while(bckw_path != GraphPt->search_start) {
    bckw_path = bckw_path->search_from;
    NbNodes++;
    CurrentCost= p3d_GetConfigCost(GraphPt->rob, bckw_path->q);   
    TotalCost = TotalCost + CurrentCost;
  }
  return  TotalCost/((double)(NbNodes+1));
}

/**
 * p3d_PrintNodeSolutionCost
 * Print the cost of  the solution path nodes and print an approximation 
 * of the global cost based on these nodes.
 * The global cost is normalised by dividing by the total number of nodes
 * @param[In] GraphPt: pointer the robot graph 
 */ 
void p3d_PrintNodeSolutionCost(p3d_graph* GraphPt) {
  int NbNodes = 0;
  double CurrentCost, TotalCost;
  p3d_node *bckw_path;
  configPt qCurrent; 

  if(GraphPt == NULL) {  
    PrintInfo(("Warning: Failed to compute the cost of a solution path: \
as no graph has been found\n"));
    return;
  }

  qCurrent = p3d_get_robot_config(GraphPt->rob);
  CurrentCost= p3d_GetConfigCost(GraphPt->rob, qCurrent);
  PrintInfo(("Current config cost : %f\n", CurrentCost));
  p3d_destroy_config(GraphPt->rob, qCurrent);

  if (p3d_graph_search(GraphPt, p3d_heurist, p3d_valid, p3d_end, DEFAULTGRAPH) == FALSE) {
    PrintInfo(("Warning: Failed to compute the cost of a solution path: \
as no soltion solution path has been found\n"));
    return;
  }
  bckw_path = GraphPt->search_goal;
  PrintInfo(("Nodes solution costs:\n"));
  CurrentCost= p3d_GetConfigCost(GraphPt->rob, bckw_path->q);
  TotalCost = CurrentCost;
  PrintInfo(("Cost Node[%d]: %f\n",NbNodes,CurrentCost));
  while(bckw_path != GraphPt->search_start) {
    bckw_path = bckw_path->search_from;
    NbNodes++;
    CurrentCost= p3d_GetConfigCost(GraphPt->rob, bckw_path->q);   
    TotalCost = TotalCost + CurrentCost;
    PrintInfo(("Cost Node[%d]: %f\n",NbNodes,CurrentCost));
  }
  PrintInfo(("Average cost: %f\n", TotalCost/((double)(NbNodes+1))));   
  PrintInfo(("\n"));
}



double p3d_getTrajCost(p3d_graph* graphPt, p3d_traj* trajPt, 
		       double* minCostPt, double* maxCostPt, 
		       double* totalCostPt, int* nbConfigPt) {
  configPt config;
  double currentCost;
  double currentParam = 0.;
  double dMax = p3d_get_env_dmax();
  double prevCost;
  double Wsum = 0.;
  
  if (trajPt == NULL) {
    PrintInfo(("Warning: Failed to compute the cost of a solution path: \
as no soltion solution path has been found\n"));
    return 0.;
  }
  config = p3d_config_at_param_along_traj(trajPt,currentParam);
  currentCost= p3d_GetConfigCost(graphPt->rob, config);
  *totalCostPt = currentCost;
  *maxCostPt = currentCost;
  *minCostPt = currentCost;
  
  while(currentParam <= trajPt->range_param) {
    currentParam += dMax;
    prevCost = currentCost;
    p3d_destroy_config(graphPt->rob, config);
    config = p3d_config_at_param_along_traj(trajPt,currentParam);
    currentCost= p3d_GetConfigCost(graphPt->rob, config);
    
    //computation of the cost associated to a mechanical work W
    Wsum += p3d_ComputeDeltaStepCost(prevCost, currentCost, dMax);
    *totalCostPt = (*totalCostPt) + currentCost;
    *minCostPt = MIN((*minCostPt),currentCost);
    *maxCostPt = MAX((*maxCostPt),currentCost);
    (*nbConfigPt)++;
  }
  return Wsum;
}

/**
 * p3d_PrintTrajCost
 * Print several trajectory cost informations 
 * @param[In] graphPt: pointer to the robot graph 
 * @param[In] trajPt: the given trajectory
 */ 
void p3d_PrintTrajCost(p3d_graph* graphPt, p3d_traj* trajPt) {
  int nbConfig = 0;
  double  totalCost = 0, minCost = 0, maxCost = 0;
  double Wsum = 0.;

   Wsum =  p3d_getTrajCost(graphPt, trajPt, &minCost, &maxCost, &totalCost, 
			   &nbConfig);

  // Print relevant information
  PrintInfo(("\n"));
  PrintInfo(("Minimal cost: %f\n", minCost)); 
  PrintInfo(("Maximal cost: %f\n", maxCost)); 
  PrintInfo(("Average cost: %f\n", totalCost/((double)(nbConfig+1)))); 
  PrintInfo(("Average cost * dist: %f\n", (trajPt->range_param)*totalCost/((double)(nbConfig+1))));
  PrintInfo(("W sum: %f\n", Wsum)); 
  PrintInfo(("Length: %f\n", trajPt->range_param));
  PrintInfo(("\n"));
}


p3d_matrix4 Transfo;
double ZminEnv = HUGE_VAL;
double ZmaxEnv = 0;

/**
 * AddSpaceScaledVertex
 * For 2D cost spaces. Add the vertices in 
 * the GroundCostObj. It allow to create a cost 
 * function depending of the shape of the ground. 
 * It relies on the C++ groundHeigh library.
 * It also saves ZminEnv and  ZmaxEnv that are 
 * notably used to color the environment in function of the cost. 
 * param[In] GroundCostObj: the object for the C++ interfacage
 * param[In] x: x coordinate of the vertex
 * param[In] y: y coordinate of the vertex
 * param[In] z: z coordinate of the vertex
 */
void AddSpaceScaledVertex (void* GroundCostObj, double x, double y , double z) {
  p3d_vector4 xyz1, newXyz1;  
  xyz1[0] = x;
  xyz1[1] = y;
  xyz1[2] = z;
  xyz1[3] = 1;

  //  p3d_mat4Pos;
  p3d_matvec4Mult(Transfo,xyz1,newXyz1);
  ZminEnv = MAX(0, MIN(ZminEnv, newXyz1[2]));
  ZmaxEnv = MAX(ZmaxEnv, newXyz1[2]);

   GHaddVertex(GroundCostObj,newXyz1[0] ,newXyz1[1] ,newXyz1[2]);
}

/**
 * p3d_getEdgeCost
 * Get the cost value of an edge
 * @param[In] edgePt: the edge
 */
double p3d_getEdgeCost(p3d_edge* edgePt){
  return edgePt->cost;
}

/**
 * p3d_GetDeltaCostChoice
 * Get the choice of the cost  function used 
 * to compute Delta motion costs in function of 
 * the cost of the extremal configurations.
 * @return: the current DelatCostChoice. 
 */
int p3d_GetDeltaCostChoice(){
  return DeltaCostChoice;
}

/**
 * p3d_SetDeltaCostChoice
 * Set the choice of the cost function used 
 * to compute Delta motion costs in function of 
 * the cost of the extremal configurations.
 * @param[In]: the DelatCostChoice that is set. 
 */
void p3d_SetDeltaCostChoice(int deltaCostChoice){
  DeltaCostChoice = deltaCostChoice;
}

/**
 * p3d_SetEdgeCost
 * Set the  cost of an edge based on
 * its extremal nodes and its length
 * @param[In] edgePt: pointer to the edge 
 */
void p3d_SetEdgeCost (p3d_edge* edgePt) {
  double cost1, cost2, length;
  if(edgePt == NULL) return;

  cost1 = edgePt->Ni->cost;
  cost2 = edgePt->Nf->cost;
  length = edgePt->longueur;
  edgePt->cost = p3d_ComputeDeltaStepCost(cost1,cost2,length);
}

/**
 * p3d_ComputeDeltaStepCost
 * Compute the cost of a portion of path 
 * of length length, between two config with 
 * respective cost  cost1 and cost2. 
 */
double p3d_ComputeDeltaStepCost(double cost1, double cost2, 
				double length) {
  double epsilon =0.002;
  double alpha;
  double kb = 0.00831, temp = 310.15;
  if(ENV.getBool(Env::isCostSpace) == TRUE) {
    switch(p3d_GetDeltaCostChoice()) {
    case MECHANICAL_WORK:
      if(cost2 > cost1) {
	return length*(epsilon+ cost2 - cost1);
      }
      return epsilon*length;
    case AVERAGE_CONFIG_COST:
      return (cost1+cost2)/2.;
    case CONFIG_COST_AND_DIST:
      alpha = p3d_GetAlphaValue();
      return alpha*(cost1+cost2)/2. + (1.-alpha)*length;
    case BOLTZMANN_COST:
      if(cost2 >cost1 )
	return 1;
      return 1/exp(ENV.getInt(Env::maxCostOptimFailures)*(cost2-cost1)/(kb*temp));
    }
  }
  //no cost function
  return length;
}

/**
 * void p3d_UpdateEdgeGraphCost
 * Update the cost values of all the graph edges
 * Used in case of cost function change
 * @param[In] graphPt: The robot graph
 */
 void p3d_UpdateEdgeGraphCost(p3d_graph* graphPt) {
  p3d_list_edge* listEdgePt = graphPt->edges;
  while(listEdgePt != NULL) {
    p3d_SetEdgeCost(listEdgePt->E);
    listEdgePt = listEdgePt->next;
  }
}

/**
 * CostTestSucceeded
 * Transition Test function to validate the feasability of the motion
 * from the current config with the current cost in function
 * of the previous config and cost
 * this test is currently based on the Metropolis Criterion
 * also referred as the Boltzmann probability when applied to
 * statistical physics or molecular modeling
 * The temperature parameter is adaptively tuned  in function of the
 * failures and successes during the search process.
 * This adaptation can be local to each node or applied globaly to
 * the entire graph.
 * @param[In] G: pointer to the robot graph
 * @param[In] previousNodePt: pointer to the previous node
 * @param[In] currentConfig: current confguration
 * @param[In] PreviousCost: Previous cost (i.e. cost
 * of the previous config)
 * @param[In] CurrentCost: Current node cost
 * @param[In] IsRaisingCost: Give the direction of the
 * cost. TRUE if its easy to succeed test for increasing costs
 * (i.e from goal to init) FALSE otherwise.
 * @return: TRUE if the test succeeded, FALSE otherwise
 */
int CostTestSucceeded(p3d_graph* G, p3d_node* previousNodePt, configPt currentConfig,
		      double PreviousCost, double CurrentCost,
		      int IsRaisingCost) {
  double RandomVal;
  double ThresholdVal;
  double TempCost;
  double dist;
  int isSucceeded = FALSE;
  configPt previousConfig = previousNodePt->q;
  double temperature, cVertex,cOpt,cMax ;
  double coefTempMultip = 2.;
  p3d_rob* robotPt = G->rob;
  double minThreshold = 0.05;
  double step = p3d_get_env_dmax()* ENV.getDouble(Env::extensionStep);

  switch(p3d_GetCostMethodChoice()) {
  case MAXIMAL_THRESHOLD:
    // Literature method:
    // the condition test is only based on
    // an absolute cost which increase continuously.
    if(p3d_GetConfigCost(robotPt, currentConfig) < p3d_GetCostThreshold()) {
      isSucceeded = TRUE;
    } else {
      isSucceeded = FALSE;
    }
    break;
  case URMSON_TRANSITION:
    //TODO !
    cVertex = p3d_ComputeUrmsonNodeCost(G, previousNodePt);
    cOpt = (G->search_start->comp->minCost)*(SelectedDistConfig(G->rob, G->search_start->q,G->search_goal->q)+1)/(2.*step);
    cMax = G->search_start->comp->maxUrmsonCost;
    ThresholdVal = 1-(cVertex - cOpt)/(cMax - cOpt);
    ThresholdVal = MAX(ThresholdVal, minThreshold);
    //    PrintInfo(("Threshold value : %f,cVertex:%f, cOpt:%f, cMax:%f \n ",ThresholdVal,
    //       cVertex, cOpt, cMax));
    RandomVal = p3d_random(0.,1.);
    isSucceeded = (RandomVal < ThresholdVal);
    //PrintInfo(("isSucceeded: %d\n",isSucceeded));
    break;
    //the same part is used for TRANSITION_RRT and
    //MONTE_CARLO_SEARCH
  case TRANSITION_RRT:
  case TRANSITION_RRT_CYCLE:
  case MONTE_CARLO_SEARCH:

    // IsRaisingCost is FALSE when we are expanding the InitComp:
    // Downhill slopes have better chances to be accepted. If
    // we are expanding the GoalComp tests are inversed: Uphill
    //slopes have better chances to be accepted
    if(IsRaisingCost == TRUE) {
      TempCost = PreviousCost;
      PreviousCost = CurrentCost;
      CurrentCost = TempCost;
    }

    // Old implementation in case of
    //down hill slopes including a maximal
    // ratio between uphill and downhill slopes
    // this test ratio has been removed
    /*   if(CurrentCost <= PreviousCost) { */
    /*     GlobalNbDown++; */
    /*     (previousNodePt->NbDown)++; */
    /*     if(p3d_GetIsLocalCostAdapt()) { */
    /*       if( (p3d_GetThresholdDown() != 0 )&&  */
    /* 	  (previousNodePt->NbDown > p3d_GetThresholdDown())) { */
    /* 	previousNodePt->NbDown =0; */
    /* 	return FALSE; */
    /*       } else { */
    /* 	return TRUE; */
    /*       } */
    /*     } else { */
    /*       if( (p3d_GetThresholdDown() != 0 )&&  */
    /* 	  (GlobalNbDown > p3d_GetThresholdDown())) { */
    /* 	GlobalNbDown = 0; */
    /* 	return FALSE; */
    /*       } else { */
    /* 	return TRUE; */
    /*       } */
    /*     } */
    /*   } */

    //new simplified test for down hill slopes
    if(CurrentCost <= PreviousCost) {
      return TRUE;
    }

    //  GlobalNbDown =0;
    // previousNodePt->NbDown =0;

    // In principle, the distance are not necessarly
    // reversible for non-holonomic robots
    if(IsRaisingCost == TRUE) {
      dist =p3d_dist_q1_q2(robotPt,currentConfig, previousConfig);
    } else {
      dist =p3d_dist_q1_q2(robotPt, previousConfig, currentConfig);
    }
    // get the value of the auto adaptive temperature.
    if(p3d_GetIsLocalCostAdapt()) {
      temperature = previousNodePt->temp;
    } else {
      temperature = p3d_GetTemperatureParam();
    }

    if(p3d_GetCostMethodChoice() == MONTE_CARLO_SEARCH) {
      temperature = 0.001*p3d_GetAlphaValue()*ENV.getInt(Env::maxCostOptimFailures);
    }
    /*Main function to test if the next config
      will be selected as a new node.
      The TemperatureParam adjust itself automatically
      in function of the fails and successes of extensions*/


    //Metropolis criterion (ie Boltzman probability)
    ThresholdVal = exp((PreviousCost-CurrentCost)/(temperature*dist*AverQsQgCost));

    //Main function with a sigmoid shape
    /*   ThresholdVal = 1/(1+exp(-1.*(10e6)*(PreviousCost-CurrentCost)/
	 (p3d_GetTemperatureParam()*dist*(AverageCost)))); */

    //    PrintInfo(("Temperature: %f, threshold: %f\n",p3d_GetTemperatureParam(), ThresholdVal));


    RandomVal = p3d_random(0.,1.);
    isSucceeded = (RandomVal < ThresholdVal);
    if(p3d_GetCostMethodChoice() == MONTE_CARLO_SEARCH) {
      break;
    }
    if(isSucceeded == TRUE) {

      //nbFail update
      GlobalNbFailTemp =0;
      previousNodePt->nbFailedTemp =0;

      //temperature update
      p3d_SetTemperatureParam(p3d_GetTemperatureParam()/(coefTempMultip));
      previousNodePt->temp= previousNodePt->temp/(coefTempMultip);

    } else {


      //nbFail update
      GlobalNbFailTemp++;
      (previousNodePt->nbFailedTemp)++;

      //True if the adaptation is local
      if(p3d_GetIsLocalCostAdapt()){
	if (previousNodePt->nbFailedTemp > pow(10,ENV.getDouble(Env::temperatureRate))) {
	  //temperature update
	  p3d_SetTemperatureParam(p3d_GetTemperatureParam()*(coefTempMultip));
	  previousNodePt->temp= previousNodePt->temp*(coefTempMultip);

	  //nbFail update
	  GlobalNbFailTemp =0;
	  previousNodePt->nbFailedTemp = 0;
	}

      } else {
	if (GlobalNbFailTemp > pow(10,ENV.getDouble(Env::temperatureRate))) {
	  //temperature update
	  p3d_SetTemperatureParam(p3d_GetTemperatureParam()*(coefTempMultip));
	  previousNodePt->temp= previousNodePt->temp*(coefTempMultip);

	  //nbFail update
	  GlobalNbFailTemp =0;
	  previousNodePt->nbFailedTemp = 0;
	}
      }
    }
    //  p3d_EvaluateExpandDiffic(previousNodePt->comp, isSucceeded);
  }
  return isSucceeded;
}

/**
 * getTrajPortionCost
 * get the cost of the trajectory portion
 * between parameters pathParam1 and pathParam2
 * @param[In] graphPt: the robot graph
 * @param[In] trajPt: the trajectory
 * @param[In] pathParam1: the first trajectory param from 
 * which the trajectory cost is computed.
 *  @param[In] pathParam2: the second trajectory param until
 * which the trajectory cost is computed.
 * return: the cost of the trajectory portion 
 */
double  getTrajPortionCost(p3d_graph* graphPt, p3d_traj* trajPt, 
				double pathParam1, double pathParam2) {
  configPt config;
  double currentCost, currentParam = pathParam1;
  double prevCost;
  double dMax = p3d_get_env_dmax();
  double Wsum = 0.; 

  config = p3d_config_at_param_along_traj(trajPt, currentParam);
  currentCost= p3d_GetConfigCost(graphPt->rob, config);

  while(currentParam < pathParam2) {
    currentParam += MIN(dMax, pathParam2-currentParam);
    prevCost = currentCost;
    p3d_destroy_config(graphPt->rob, config);
    config = p3d_config_at_param_along_traj(trajPt,currentParam);
    currentCost= p3d_GetConfigCost(graphPt->rob, config);

    //computation of the cost associated to the path portion
    Wsum += p3d_ComputeDeltaStepCost(prevCost, currentCost, dMax);
  }

  return Wsum;
}

/**
 * p3d_OneLoopOptimizeCostTraj
 * Compute one loop of cost optmization
 * @param[In] G: the robot graph
 * @trajPt[In] trajPt: the current trajectory 
 * to optimize
 * return: TRUE if the trajectory has been successfully 
 * optimized.
 */
int p3d_OneLoopOptimizeCostTraj(p3d_graph* G, p3d_traj* trajPt) {
  int isFailReplaced;
  configPt qPrev = NULL, qCurrent = NULL, qNext=NULL, qNew = NULL;
  double lPrev =0., lCurrent=0., lNext=0.;
  double cCurrent, cNew;
  p3d_traj*  newTrajPart = NULL;
  int isOptimSuccess = FALSE;

  p3d_GetRandSuccesConfAlongTraj(trajPt, &qPrev, &qCurrent, &qNext, 
				 &lPrev, &lCurrent, &lNext);
  qNew = p3d_DiffuseOneConf(G->rob, qCurrent);
  cCurrent = getTrajPortionCost(G,  trajPt, 
				    lPrev, lNext);
  newTrajPart = p3d_createThreeConfTraj(G->rob, qPrev, qNew, qNext);
  cNew = getTrajPortionCost(G,  newTrajPart, 0., newTrajPart->range_param);
  if(cNew < cCurrent) {
    //warning: p3d_replace_traj return FALSE in case of success
    isFailReplaced = p3d_replace_traj(trajPt, lPrev, lNext, newTrajPart);
    if(isFailReplaced == TRUE) {
      PrintInfo(("Warning: failed to replace the trajectory parts\n"));
    } else {
      isOptimSuccess = TRUE;
    }
  }
  p3d_destroy_traj(G->rob, newTrajPart);
  
  p3d_destroy_config(G->rob, qPrev);
  p3d_destroy_config(G->rob, qCurrent);
  p3d_destroy_config(G->rob, qNext);
  p3d_destroy_config(G->rob, qNew);
  return isOptimSuccess;
}

/**
 * p3d_ComputeUrmsonNodeCost
 * compute the gloabal cost of a node according to 
 * the Urmson and Simmons paper. The cost is equal 
 * to the sum cost plus the estimation
 * of the cost to the goal. 
 * @param[In] G: the robot graph
 * @param[In] nodePt: the node for which we compute the cost
 * @return: the Urmson node cost 
 */
double p3d_ComputeUrmsonNodeCost(p3d_graph* G, p3d_node* nodePt) {
  double costToGoal;
  costToGoal =  p3d_ComputeUrmsonCostToGoal(G,nodePt);
  return nodePt->sumCost +costToGoal;
}

/**
 * p3d_ComputeUrmsonCostToGoal
 * Estimation of the sum cost to the goal
 * @param[In] G: the robot graph
 * @param[In] nodePt: the node for which we compute the cost
 * @return: the estimation of the sum of the Ursom cost to the Goal  
 */
double p3d_ComputeUrmsonCostToGoal(p3d_graph* G, p3d_node* nodePt) {
  double step =  p3d_get_env_dmax()* ENV.getDouble(Env::extensionStep);
  if((G ==NULL) || (G->search_goal == NULL)) {
    PrintInfo(("Error: In ComputeUrmsonNodeCost, no goal to estimate\n"));
    return 0.;
  }
  return (nodePt->cost+G->search_goal->cost)*
    (SelectedDistConfig(G->rob,nodePt->q,G->search_goal->q)+1)/(2.*step);

}
