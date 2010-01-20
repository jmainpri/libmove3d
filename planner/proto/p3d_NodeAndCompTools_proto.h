#ifndef __CEXTRACT__

/**
 * p3d_SetNodeCompStrategy 
 * Set the current value of the strategy used to
 * connect a node to a connected componant
 * @param[In] NodeToCompStrategy: the selected strategy.
 */
void p3d_SetNodeCompStrategy(int NodeToCompStrategy);

/**
 * p3d_GetNodeCompStrategy 
 * Get the value of the current strategy used to
 * connect a node to a connected componant
 * @return: the current strategy to connect a node
 * to a componant.
 */
int p3d_GetNodeCompStrategy(void);

/**
 * p3d_SetIsMaxDistNeighbor
 * Set if there is a maximal distance allowed when 
 * finding the node to expand. Used in the orginal
 * version of DD-RRT to restrict the shooting configurations
 * inside the dynamic domain of the nodes
 * @param[In]: IsMaxDistNeighbor: TRUE if there is a filtering
 * on the maximal distance
 */ 
void p3d_SetIsMaxDistNeighbor(int IsMaxDistNeighbor);

/**
 * p3d_GetIsMaxDistNeighbor
 * Get if there is a maximal distance allowed when 
 * finding the node to expand. Used in the orginal
 * version of DD-RRT to restrict the shooting configurations
 * inside the dynamic domain of the nodes
 * @return: TRUE if there is a filtering
 * on the maximal distance
 */ 
int p3d_GetIsMaxDistNeighbor(void);

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
p3d_node* NearestWeightNeighbor(p3d_graph* GraphPt,
				       p3d_compco* CompToExpandPt, 
				       configPt Config);

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
				 configPt Config, int KNearest);


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
int p3d_LinkNodesMergeComp(p3d_graph* graphPt, 
			   p3d_node* Node1Pt,
			   p3d_node* Node2Pt);

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
			  p3d_compco* CompToConnectPt);


/**
 * p3d_RandomNodeFromComp
 * Select randomly a node inside a connect componant
 * @param[In]: a pointer to the given connect componant
 * @return: The selected node
 */
p3d_node* p3d_RandomNodeFromComp(p3d_compco* CompPt);

/**
 * p3d_listNodeInDist
 * TODO
 */
p3d_list_node* p3d_listNodeInDist(p3d_rob* robotPt, p3d_compco* compPt, 
				  p3d_node* nodePt, double dist);
#endif
