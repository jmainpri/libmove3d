#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"

/**
 * Note: the integer values of the different 
 * EXPANSION_NODE_METHODs are defined in the
 * p3d_type.h file 
 */
static int EXPANSION_NODE_METHOD = NEAREST_EXP_NODE_METH;

/**
 * Flag  checking if there is 
 * or not a  maximum number of failures allowed  
 * for a node until it is discarded for selection
 *Should be TRUE if there is a maximal number of failures
 */
static int IS_MAX_EXPAND_NODE_FAILURE = FALSE;

/**
 * Number of failures allowed  
 * for a node until it is discarded if the flag 
 *  IsMaximalNumberOfFail is TRUE
 * Can be modified with the p3d_SetMaxExpandNodeFail
 * function
 */
static int MAX_EXPAND_NODE_FAILURE = 10;

/**
 * the percentage  of K-nearest neighbors in a compoant, 
 * inside which a node is selected for expansion
 * if the appropriate expansion method is selected.
 */
static int K_NEAREST_PERCENT = 10;

/**
 * p3d_GetNearestExpandPercent
 * Get for a componant the percent of K-nearest neighbors
 * inside which a node is selected for expansion
 * if the appropriate expansion method is selected.
 * @return:  the percentage of K-nearest neighbor 
 * in the componant.
 */
int p3d_GetNearestExpandPercent(void) {
  return K_NEAREST_PERCENT;
}

/**
 * SetNbNearestExpand
 * Set for a componant the percent of K-nearest neighbors
 * inside which a node is selected for expansion
 * if the appropriate expansion method is selected.
 * @param[In]: KNearestPercent: the percentage of K-nearest neighbor 
 * in the componant.
 */
void p3d_SetNearestExpandPercent(int KNearestPercent) {
  K_NEAREST_PERCENT = KNearestPercent;
}

/**
 * p3d_SetExpansionNodeMethod
 * Set the current value of the method used to 
 * choose the node to expand
 * @param[in] ExpNodeMethod: the expansion Node method
 */
 void p3d_SetExpansionNodeMethod(int ExpNodeMethod)
{
  EXPANSION_NODE_METHOD = ExpNodeMethod;
}

/**
 * p3d_GetExpansionNodeMethod
 * Get the value of the current method used to 
 * choose the node to expand
 * @return: the current expansion Node method
 */
 int p3d_GetExpansionNodeMethod(void)
{
  return EXPANSION_NODE_METHOD;
}

/**
 * p3d_SetIsMaxExpandNodeFail
 * Function to get the flag checking if there is 
 * or not a  maximum number of failures allowed  
 * for a node until it is discarded for selection
 * @param[in]: IsMaxExpandNodeFailure the value of the flag
 * Should be true if there is a maximal number of failures.
 */
 void p3d_SetIsMaxExpandNodeFail(int IsMaxExpandNodeFailure)
{
  IS_MAX_EXPAND_NODE_FAILURE = IsMaxExpandNodeFailure;
}

/**
 * p3d_GetIsMaxExpandNodeFail
 * Function to get the flag checking if there is 
 * or not a  maximum number of failures allowed  
 * for a node until it is discarded for selection
 * @return:  the value of the flag
 * Should be true if there is a maximal number of failures.
 */
 int p3d_GetIsMaxExpandNodeFail(void)
{
  return IS_MAX_EXPAND_NODE_FAILURE;
}

/**
 * p3d_SetMaxExpandNodeFail
 * Set the maximum number of failures allowed  
 * for a node until it is discarded if the flag 
 *  IsMaximalNumberOfFail is TRUE
 * @param[in]: MaxExpandNodeFailure: 
 * the maximum number of failures allowed
 */
void p3d_SetMaxExpandNodeFail(int MaxExpandNodeFailure)
{
  MAX_EXPAND_NODE_FAILURE = MaxExpandNodeFailure;
}

/**
 * p3d_GetMaxExpandNodeFail
 * Get the maximum number of failures allowed  
 * for a node until it is discarded if the flag 
 *  IsMaximalNumberOfFail is TRUE
 * @return: the maximum number of failures allowed
 */
int p3d_GetMaxExpandNodeFail(void)
{
  return MAX_EXPAND_NODE_FAILURE;
}

/**
 * SelectExpansionNode
 * Main function selecting a node to expand
 * for a connected componant
 * @param[in] GraphPt: Pointer to the robot graph
 * @param[in] CompToExpandPt: the connected componant to expand
 * @param[in] DirectionConfig the configuration previously 
 * selected as  direction of expansion
 * @return: The node of the connected componant selected
 * or NULL if no node has been found
 */
p3d_node* SelectExpansionNode(p3d_graph* GraphPt, p3d_compco* CompToExpandPt, 
			      configPt DirectionConfig) {
  p3d_node* ExpansionNodePt =NULL;
  int KNearest = -1;
  int NearestPercent;


 if((GraphPt == NULL) || (CompToExpandPt == NULL)) {
   PrintInfo(("Warning : Try to Select an expansion node with wrong graph structures \n"));
    return NULL;
  }

 switch(p3d_GetExpansionNodeMethod()) {

 case NEAREST_EXP_NODE_METH:
   /* Choose the nearest node of the componant*/
   ExpansionNodePt = NearestWeightNeighbor(GraphPt, CompToExpandPt, DirectionConfig);
   break;

 case K_NEAREST_EXP_NODE_METH:
   /* Select randomly among the K nearest nodes of a componant */
   NearestPercent = p3d_GetNearestExpandPercent();
   KNearest = MAX(1,(int)((NearestPercent*(CompToExpandPt->nnode))/100.));
   ExpansionNodePt = KNearestWeightNeighbor(GraphPt, CompToExpandPt, DirectionConfig,
					    KNearest);
   break;
 case BEST_SCORE_EXP_METH:
   /* Select the node which has the best score: weight*dist */
   ExpansionNodePt = NearestWeightNeighbor(GraphPt, CompToExpandPt, DirectionConfig);
   break;
 case K_BEST_SCORE_EXP_METH:
   NearestPercent = p3d_GetNearestExpandPercent();
   KNearest = MAX(1,(int)((NearestPercent*(CompToExpandPt->nnode))/100.));
   ExpansionNodePt = KNearestWeightNeighbor(GraphPt, CompToExpandPt, DirectionConfig, KNearest);
   break;
 case RANDOM_IN_SHELL_METH:
   /* Select randomly among all the nodes inside a given portion of shell */
   ExpansionNodePt = hrm_selected_pb_node(GraphPt,
					  DirectionConfig,
					  CompToExpandPt);
   break;
 case RANDOM_NODE_METH:
   ExpansionNodePt = p3d_RandomNodeFromComp(CompToExpandPt);
   break;
 default:
   /* By default return the nearest node of the componant */
   ExpansionNodePt = NearestWeightNeighbor(GraphPt, CompToExpandPt, DirectionConfig);
  }
 return ExpansionNodePt;
}




