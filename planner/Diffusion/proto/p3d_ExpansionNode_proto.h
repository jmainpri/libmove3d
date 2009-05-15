#ifndef __CEXTRACT__

/**
 * p3d_GetNearestExpandPercent
 * Get for a componant the percent of K-nearest neighbors
 * inside which a node is selected for expansion
 * if the appropriate expansion method is selected.
 * @return:  the percentage of K-nearest neighbor 
 * in the componant.
 */
int p3d_GetNearestExpandPercent(void);

/**
 * SetNbNearestExpand
 * Set for a componant the percent of K-nearest neighbors
 * inside which a node is selected for expansion
 * if the appropriate expansion method is selected.
 * @param[In]: KNearestPercent: the percentage of K-nearest neighbor 
 * in the componant.
 */
void p3d_SetNearestExpandPercent(int KNearestPercent);

/**
 * p3d_SetExpansionNodeMethod
 * Set the current value of the method used to 
 * choose the node to expand
 * @param[in] ExpNodeMethod: the expansion Node method
 */
void p3d_SetExpansionNodeMethod(int ExpNodeMethod);

/**
 * p3d_GetExpansionNodeMethod
 * Get the value of the current method used to 
 * choose the node to expand
 * @return: the current expansion Node method
 */
int p3d_GetExpansionNodeMethod(void);

/**
 * p3d_SetIsMaxExpandNodeFail
 * Function to get the flag checking if there is 
 * or not a  maximum number of failures allowed  
 * for a node until it is discarded for selection
 * @param[in]: IsMaxExpandNodeFailure the value of the flag
 * Should be true if there is a maximal number of failures.
 */
void p3d_SetIsMaxExpandNodeFail(int IsMaxExpandNodeFailure);

/**
 * p3d_GetIsMaxExpandNodeFail
 * Function to get the flag checking if there is 
 * or not a  maximum number of failures allowed  
 * for a node until it is discarded for selection
 * @return:  the value of the flag
 * Should be true if there is a maximal number of failures.
 */
int p3d_GetIsMaxExpandNodeFail(void);

/**
 * p3d_SetMaxExpandNodeFail
 * Set the maximum number of failures allowed  
 * for a node until it is discarded if the flag 
 *  IsMaximalNumberOfFail is TRUE
 * @param[in]: MaxExpandNodeFailure: 
 * the maximum number of failures allowed
 */
void p3d_SetMaxExpandNodeFail(int MaxExpandNodeFailure);

/**
 * p3d_GetMaxExpandNodeFail
 * Get the maximum number of failures allowed  
 * for a node until it is discarded if the flag 
 *  IsMaximalNumberOfFail is TRUE
 * @return: the maximum number of failures allowed
 */
int p3d_GetMaxExpandNodeFail(void);

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
				     configPt DirectionConfig);

#endif
