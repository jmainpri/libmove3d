#ifndef __CEXTRACT__

/**
 * p3d_GetExtendStepParam
 * Get the value of the extension step
 * parameter. the step of expansion in the extend
 * method is then equal to  ExtendStepParam*Dmax
 * @return: the extension step parameter
 */
double p3d_GetExtendStepParam(void);

/**
 * p3d_SetExtendStepParam
 * Set the value of the extension step
 * parameter. the step of expansion in the extend
 * method is then equal to  ExtendStepParam*Dmax 
 * @param[In] ExtendStepParam: the extension step parameter
 */
void p3d_SetExtendStepParam(double ExtendStepParam);

/**
 * p3d_SetExpansionChoice
 * Set the current value of the method used to 
 * process the expansion of a node toward 
 * a direction configuration selected as direction 
 * of expansion
 * @param[In] the expansion process choice  
 */
void p3d_SetExpansionChoice(int ExpansionChoice);

/**
 * p3d_GetExpansionChoice
 * Get the value of the current method used to 
 * process the expansion of a node toward 
 * a direction configuration selected as direction 
 * of expansion
 * @return: the current expansion process choice   
 */
int p3d_GetExpansionChoice(void);

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
