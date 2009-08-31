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
Node* RRT::selectExpansionNode(Node* compNode,
				     shared_ptr<Configuration> direction,
				     int distance)
{
  int KNearest = -1;
  int NearestPercent;

 switch(distance) {

 case NEAREST_EXP_NODE_METH:
   /* Choose the nearest node of the componant*/
   return(mG->nearestWeightNeighbour(compNode, direction, p3d_GetIsWeightedChoice(), distance));
 case K_NEAREST_EXP_NODE_METH:
   /* Select randomly among the K nearest nodes of a componant */
   NearestPercent = p3d_GetNearestExpandPercent();
   KNearest = MAX(1,(int)((NearestPercent*(compNode->mN->comp->nnode))/100.));
   // TODO : fix
   //   ExpansionNodePt = KNearestWeightNeighbor(mG, compNode->mN->comp, direction->mQ,
   // KNearest);
   return(mG->nearestWeightNeighbour(compNode, direction, p3d_GetIsWeightedChoice(), distance));
 case BEST_SCORE_EXP_METH:
   /* Select the node which has the best score: weight*dist */
   return(mG->nearestWeightNeighbour(compNode, direction, p3d_GetIsWeightedChoice(), distance));
 case K_BEST_SCORE_EXP_METH:
   NearestPercent = p3d_GetNearestExpandPercent();
   KNearest = MAX(1,(int)((NearestPercent*(compNode->mN->comp->nnode))/100.));
   // TODO : fix
   // ExpansionNodePt = KNearestWeightNeighbor(mG, compNode->mN->comp, direction->mQ, KNearest);
   return(mG->nearestWeightNeighbour(compNode, direction, p3d_GetIsWeightedChoice(), distance));
 case RANDOM_IN_SHELL_METH:
   /* Select randomly among all the nodes inside a given portion of shell */
   return(mG->getNode(hrm_selected_pb_node(mG->getGraph(),
					   direction->getP3dConfigPt(),
					   compNode->mN->comp)));
 case RANDOM_NODE_METH:
   return(mG->getNode(p3d_RandomNodeFromComp(compNode->mN->comp)));
 default:
   /* By default return the nearest node of the componant */
   return(mG->nearestWeightNeighbour(compNode, direction, p3d_GetIsWeightedChoice(), distance));
  }
}
