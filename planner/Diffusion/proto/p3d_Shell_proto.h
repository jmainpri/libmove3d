#ifndef __CEXTRACT__

/**
 * p3d_init_pb_variables
 * Init the graph variables needed to  use the shells
 * @param[In] GraphPt: the robot graph 
 */
void p3d_init_pb_variables(p3d_graph* GraphPt);

/**
 * hrm_selected_pb_node
 * return the node selected thinks to the shell method
 * @param[In] GraphPt: pointer to the robot graph
 * @param[In] q: the configuration used to get the direction of 
 * expansion
 * @param[In comp: the connected componant to extend
 * @return: the selected node of the componant 
 */
p3d_node* hrm_selected_pb_node(p3d_graph* GraphPt, 
			       configPt q, p3d_compco *comp);
/*
 * update_parent_nfails
 * update the n_fail_extend value of the parent node 
 * of a given node
 * @param[In]: The given node
 * @return: TRUE if the n_fail_extend value of the parent node
 * has been set to rrt_maxn_fails.
 */
int update_parent_nfails(p3d_node *N);

/**
 * Get The number of time a new node 
 * has been created without fail
 * @return: The  number of time a 
 * new node has been created without fail
 */
int p3d_GetNGood(void);

/**
 * p3d_SetNGood
 * Set The number of time a new node 
 * has been created without fail
 * @param[In]: The  number of time a 
 * new node has been created without fail
 */
void p3d_SetNGood(int NGood);



#endif

