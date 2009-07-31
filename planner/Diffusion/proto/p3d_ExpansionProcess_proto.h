#ifndef __CEXTRACT__

/**
 * ExpandProcess
 *  General function expanding a node toward a direction
 * of expansion. The mode of expansion depends of the 
 * expansion choice selected.
 * @param[In] GraphPt: Pointer to the robot graph
 * @param[In] ExpansionNodePt: the node to expand
 * @param[In] DirectionConfig: the direction of expansion
 * return: the number of nodes created during the expansion 
 * process (several nodes can be created during one expansion) 
 */
int ExpandProcess(p3d_graph *GraphPt, p3d_node* ExpansionNodePt, 
			 configPt DirectionConfig);
#endif
