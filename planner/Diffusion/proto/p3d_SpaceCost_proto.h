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
#ifndef __CEXTRACT__

int p3d_GetCostMethodChoice(void);

/**
 * p3d_GetIsLocalCostAdapt
 * Get if the adatpation of the success/failed extension 
 * is local or global  
 * @return: TRUE if the adaptation is local.
 */
int p3d_GetIsLocalCostAdapt(void);

/**
 * p3d_SetIsLocalCostAdapt
 * Set if the adatpation of the success/failed extension 
 * is local or global  
 * @param[In] isLocalCostAdapt: TRUE if the adaptation 
 * is local.
 */
void p3d_SetIsLocalCostAdapt(int isLocalCostAdapt);

/**
 * p3d_GetThresholdDown
 * Get the number of consecutive time the 
 * planner or node (local) is allowed to create 
 * a new node with a lower cost until the 
 * node is rejected  
 * @return: the number of decreasing 
 * costs allowed.
 */
/* int p3d_GetThresholdDown(void); */

/**
 * p3d_SetThresholdDown
 * Set the number of consecutive time the 
 * planner or node (local) is allowed to create 
 * a new node with a lower cost until the 
 * node is rejected  
 * @param[In] thresholdDow: the number of decreasing 
 * costs allowed.
 */
/* void p3d_SetThresholdDown(int thresholdDown); */

/**
 * p3d_GetTemperatureParam
 * Get the value of the temperature parameter
 * this parameter is used to set the threshold in the
 * CostTestSucceeded function
 * @return: the value of the temperature parameter
 */
double p3d_GetTemperatureParam(void);

/**
 * p3d_SetTemperatureParam
 * Set the value of the temperature parameter
 * this parameter is used to set the threshold in the
 * CostTestSucceeded function
 * @param[In] temperature: the value of the temperature parameter
 */
void p3d_SetTemperatureParam(double temperatur);


/**
 * p3d_GetAverQsQgCost
 */
int p3d_GetAverQsQgCost(void);

/**
 * p3d_SetAverQsQgCost
 * @param[In] the Number of failed transition
 */
void p3d_SetAverQsQgCost(double averQsQgCost);

/**
 * p3d_SetGlobalNumberOfFail
 * @param[In] the Number of failed transition
 */
void p3d_SetGlobalNumberOfFail(double nFail);

/**
 * p3d_SetInitCostThreshold
 * @param[In] the Number of failed transition
 */
void p3d_SetInitCostThreshold(double threshold);

/**
 * p3d_GetInitCostThreshold
 */
double p3d_GetInitCostThreshold(void);

/**
 * p3d_GetGlobalNumberOfFail
 */
void p3d_SetGlobalNumberOfFail(int);

/**
 * p3d_GetGlobalNumberOfFail
 */
int p3d_GetGlobalNumberOfFail(void);

/**
 * p3d_InitSpaceCostParam
 * Initialize some parameters for the planner
 * for space cost environments 
 * @param[In] GraphPt: the robot graph 
 * @param[In] Ns: Initial node of the graph 
 * @param[In] Ng: Goal node of the graph 
 */
void p3d_InitSpaceCostParam(p3d_graph* GraphPt, p3d_node* Ns, p3d_node* Ng);
/**
 * p3d_GetCostThreshold
 * Get the value of the CostThreshold.
 * Used in the RRT_Obst literature algorithm.
 * @return: the value of the CostThreshold.
 */
double p3d_GetCostThreshold(void);

/**
 * p3d_SetCostThreshold
 * Set the value of the CostThreshold.
 * Used in the RRT_Obst literature algorithm.
 * @param[In] costThreshold: the value of the CostThreshold.
 */
void p3d_SetCostThreshold(double costThreshold);

/*
 * p3d_updateCostThreshold
 * increases the cost threshold by a given amount
 * in the MAXIMAL_THRESHOLD (Ettlin/Bleuer) variant
 */
void p3d_updateCostThreshold(void);

/**
 * p3d_GetNodeCost
 * Get the cost of a node 
 *given by the Space cost function  
 * @param[In] NodePt:  the considered node
 * @return: the cost of the node
 */
double p3d_GetNodeCost(p3d_node* NodePt);


/**
 * p3d_SetNodeCost
 * Set the cost of a node   
 * @param[In] G: the robot Graph 
 * @param[In] NodePt: pointer to th given node
 * @param[In] Cost: the cost to set to the node
 */

void p3d_SetNodeCost(p3d_graph* G, p3d_node* NodePt, double Cost);

/**
 * p3d_GetConfigCost
 * Get the cost of a configuration 
 * given by the Space cost function 
 * @param[In] robotPt: the current robotPt
 * @param[In] ConfPt: the considered configuration 
 * @return: the cost of the configuration
 */
double p3d_GetConfigCost(p3d_rob* RobotPt, configPt ConfPt);


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
			      p3d_compco* CompPt);

/**
 * p3d_ComputeNodeSolutionCost
 * Compute an approximation of the cost of the solution path
 * using only the nodes of this path
 * the cost is normalised by dividing by the total number of nodes
 * @param[In] GraphPt: pointer the robot graph 
 * @return: the cost of the solution path, -1 if the path doesn't exist.
 */ 
double p3d_ComputeNodeSolutionCost(p3d_graph* GraphPt);

/**
 * p3d_PrintNodeSolutionCost
 * Print the cost of  the solution path nodes and print an approximation 
 * of the global cost based on these nodes.
 * The global cost is normalised by dividing by the total number of nodes
 * @param[In] GraphPt: pointer the robot graph 
 */ 
void p3d_PrintNodeSolutionCost(p3d_graph* GraphPt);

/**
 * p3d_PrintNodeSolutionCost
 * Print the cost of configuration along the solution path (for each dmax) 
 * and print an good evluation  of the global cost based on these configuration. 
 * The global cost is normalised by dividing by the total number of nodes
 * @param[In] graphPt: pointer to the robot graph 
 */ 
void p3d_PrintTrajCost(p3d_graph* graphPt, p3d_traj* trajPt);

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
void  AddSpaceScaledVertex (void* GroudCostObj, double x, double y , double z) ;

/**
 * p3d_SetEdgeCost
 * Set the cost the cost of an edge based on
 * its extremal nodes and its length
 * @param[In] edgePt: pointer to the edge 
 */
extern void p3d_SetEdgeCost (p3d_rob* robot, p3d_edge* edgePt);

/**
 * p3d_ComputeDeltaStepCost
 * Compute the cost of a portion of path 
 * of length length, between two config with 
 * respective cost  cost1 and cost2. 
 */
double p3d_ComputeDeltaStepCost(double cost1, double cost2, 
				double length);

/**
 * p3d_getEdgeCost
 * Get the cost value of an edge
 * @param[In] edgePt: the edge
 */
extern double p3d_getEdgeCost(p3d_edge* edgePt);

/**
 * void p3d_UpdateEdgeGraphCost
 * Update the cost values of all the graph edges
 * Used in case of cost function change
 * @param[In] graphPt: The robot graph
 */
extern void p3d_UpdateEdgeGraphCost(p3d_graph* graphPt);

/**
 * p3d_GetAlphaValue
 * @return: the alpha value
 */
extern double p3d_GetAlphaValue(void);

/**
 * p3d_SetAlphaValue
 * @param[In] alpha: the alpha value to set
 */
extern void p3d_SetAlphaValue(double alhpa);

/**
 * p3d_GetIsMonteCarloSearch
 * @return: TRUE if it is a Monte Carlo search
 */
double p3d_GetIsMonteCarloSearch(void);

/**
 * p3d_SetIsMonteCarloSearch
 * @param[In] isMonteCarloSearch: TRUE if it
 * is a Monte Carlo search
 */
void p3d_SetIsMonteCarloSearch(double isMonteCarloSearch);

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
		      int IsRaisingCost);

/**
 * p3d_GetDeltaCostChoice
 * Get the choice of the cost  function used 
 * to compute Delta motion costs in function of 
 * the cost of the extremal configurations.
 * @return: the current DelatCostChoice. 
 */
int p3d_GetDeltaCostChoice();

/**
 * p3d_SetDeltaCostChoice
 * Set the choice of the cost function used 
 * to compute Delta motion costs in function of 
 * the cost of the extremal configurations.
 * @param[In]: the DelatCostChoice that is set. 
 */
void p3d_SetDeltaCostChoice(int deltaCostChoice);

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
				   double pathParam1, double pathParam2);

/**
 * p3d_OneLoopOptimizeCostTraj
 * Compute one loop of cost optmization
 * @param[In] G: the robot graph
 * @trajPt[In] trajPt: the current trajectory 
 * to optimize
 * return: TRUE if the trajectory has been successfully 
 * optimized.
 */
int p3d_OneLoopOptimizeCostTraj(p3d_graph* G, p3d_traj* trajPt);


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
double p3d_ComputeUrmsonNodeCost(p3d_graph* G, p3d_node* nodePt);

/**
 * p3d_ComputeUrmsonCostToGoal
 * Estimation of the sum cost to the goal
 * @param[In] G: the robot graph
 * @param[In] nodePt: the node for which we compute the cost
 * @return: the estimation of the sum of the Ursom cost to the Goal  
 */
double p3d_ComputeUrmsonCostToGoal(p3d_graph* G, p3d_node* nodePt);

#endif
