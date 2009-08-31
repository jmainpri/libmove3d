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

#endif
