#ifndef __CEXTRACT__

/**
 * p3d_SetIsBidirectDiffu
 * Set if the diffusion method is mono or bidirectionnal
 * @param[In]: TRUE if the diffusion method is birectionnal
 */
void p3d_SetIsBidirectDiffu(int IsBidirectDiffu);

/**
 * p3d_GetIsBidirectDiffu
 * Get if the diffusion method is  mono or bidirectionnal
 * @return: TRUE if the diffusion method is birectionnal
 */
int p3d_GetIsBidirectDiffu(void);

/**
 * p3d_SetIsExpansionToGoal
 * Set if the diffusion method expand toward 
 * a given goal or not
 * @param[In]: TRUE if diffusion method expand toward 
 * a given goal
 */
void p3d_SetIsExpansionToGoal(int IsExpansionToGoal);

/**
 * p3d_GetIsExpansionToGoal
 * Get if the diffusion method expand toward 
 * a given goal or not
 * @return: TRUE if diffusion method expand toward 
 * a given goal
 */
int p3d_GetIsExpansionToGoal(void);

/**
 * p3d_GetIsBalancedExpansion
 * Get if the expansion process must keep the
 * componant balanced.
 * @return: TRUE if the expansion is balance
 */
int p3d_GetIsBalancedExpansion(void);

/**
* p3d_SetIsBalancedExpansion
 * Set if the expansion process must keep the
 * componant balanced.
 * @param[In] IsBalancedExpansion: TRUE if the expansion 
 * is balance
 */
void p3d_SetIsBalancedExpansion(int IsBalancedExpansion);

/**
 * p3d_ExpandCompWithoutGoal 
 * Expand a connect componant without any goal to reach
 * stop when a given amount of nodes has been created or
 * when the user push the stop button 
 * @param[In] GraphPt: Pointer to the robot graph
 * @param[In] CompPt: Pointer to the componant to expand
 * @param[In] StopFunction: a pointer to the function used to 
 * stop the application (through the stop button)
 * @param[In] DrawFunction: a pointer to the function used to 
 * draw the graph
 * @return: The total number of created nodes
*/
int p3d_ExpandCompWithoutGoal(p3d_graph* GraphPt, 
			      p3d_compco* CompPt, 
			      int (*StopFunction)(void), 
			      void (*DrawFunction)(void));

/**
 * p3d_ExpandCompToGoal 
 * Expand a connect componant with a goal to reach 
 * @param[In] GraphPt: Pointer to the robot graph
 * @param[In] CompPt: Pointer to the componant to expand
 * @param[In] StopFunction: a pointer to the function used to 
 * stop the application (through the stop button)
 * @param[In] DrawFunction: a pointer to the function used to 
 * draw the graph
 * @return: the number of created nodes
*/
int  p3d_ExpandCompToGoal(p3d_graph* GraphPt, p3d_compco* CompPt, 
			  p3d_node* GoalNodePt, 
			  int (*StopFunction)(void), 
			  void (*DrawFunction)(void));

/**
 * p3d_ExpandCompToGoal 
 * Expand the initial componant (ie. including 
 * the search start node) and try to connect it to the goal 
 * configuration (search_goal) 
 * @param[In] GraphPt: Pointer to the robot graph
 * @param[In] StopFunction: a pointer to the function used to 
 * stop the application (through the stop button)
 * @param[In] DrawFunction: a pointer to the function used to 
 * draw the graph
 * @return: the number of created nodes
*/
int p3d_ExpandInitCompToGoal(p3d_graph* GraphPt, 
			     int (*StopFunction)(void),
			     void (*DrawFunction)(void));

/**
 * p3d_BiExpandInitGoalComp 
 * Expand the initial and Goal componants and try to connect 
 * them. 
 * @param[In] GraphPt: Pointer to the robot graph
 * @param[In] StopFunction: a pointer to the function used to 
 * stop the application (through the stop button)
 * @param[In] DrawFunction: a pointer to the function used to 
 * draw the graph
 * @return: if the extremal configurations are linked
*/
int p3d_BiExpandInitGoalComp(p3d_graph* GraphPt, 
			     int (*StopFunction)(void),
			     void (*DrawFunction)(void));

/**
 * p3d_RunDiffusion
 * Main function to run a diffusion process based 
 * on the current value of the setting parameter
 * @param[In] GraphPt: Pointeer on the Robot Graph
 * @param[In] fct_stop: stop function used to stop the process.
 * Used in articular when the user push the stop button
 * @param[In] fct_draw: function used to draw the current graph.
 * @return: TRUE if the extremal positions are linked.
 */
int p3d_RunDiffusion(p3d_graph* GraphPt, int (*fct_stop)(void),
		     void (*fct_draw)(void), configPt ConfigStart, configPt ConfigGoal);

/**
 * p3d_DiffuseOneConf
 * Create a new configuration diffused from a first one
 * by sampling a rand configuration in the space.
 * The diffusion step id equal to dMax*extendStepParam
 * (similarly to the one used in the expend version of the 
 * diffusion methods)
 * @param[In] robotPt: the current robot.
 * @param[In] qcurrent: the configuration from which the 
 * diffusion occure 
 * @return: the diffused new configuration 
 */
configPt p3d_DiffuseOneConf(p3d_rob* robotPt, configPt qCurrent);
#endif

