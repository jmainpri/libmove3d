#ifndef __CEXTRACT__

/**
 * Set the value of the bias toward the goal componant
 * if the expansion process is biased.
 * @param[In]: the value of the bias toward the goal componant
 */
void p3d_SetGoalBiasValue(double GoalBias);

/**
 * Get the value of the bias toward the goal componant
 * if the expansion process is biased.
 * @return: the value of the bias toward the goal componant
 */
double p3d_GetGoalBiasValue(void);

/** 
 * p3d_GetIsGoalBias
 * Get if the expansion is biased toward the goal
 * return: TRUE if there is a bias toward the goal
 */ 
int p3d_GetIsGoalBias(void);

/** 
 * p3d_SetIsGoalBias
 * Set if the expansion is biased toward the goal
 * @param[In] IsGoalBias: TRUE if there is a bias toward the goal
 */ 
void p3d_SetIsGoalBias(int IsGoalBias);

/**
 * p3d_GetIsDirSampleWithRlg
 * Get if the  sampling direction must
 * validate the Rlg constraints or not 
 * @return: TRUE if if the  sampling direction 
 * must validate the Rlg constraints
 */
int p3d_GetIsDirSampleWithRlg(void);

/**
 * p3d_GetIsDirSampleWithRlg
 * Set if the  sampling direction must
 * validate the Rlg constraints or not 
 * @param[In] isDirSampleWithRlg: TRUE 
 * if if the  sampling direction 
 * must validate the Rlg constraints
 */
void p3d_SetIsDirSampleWithRlg(int isDirSampleWithRlg);

/**
 * p3d_SetExpansionDirectionMethod
 * Set the current value of the method used to 
 * choose the direction of expansion
 * @param[in] the expansion direction method
*/
void p3d_SetExpansionDirectionMethod(int ExpDirMethod);

/**
 * p3d_GetExpansionDirectionMethod
 * Get the  value of the current method used to 
 * choose the direction of expansion
 * @return: the current expansion direction method
*/
int p3d_GetExpansionDirectionMethod(void);

/**
 * SelectExpansionDirection
 * Main function selecting a direction of expansion 
 * for a connect componant to expand
 * @param[In] GraphPt: Pointer to the robot graph
 * @param[In] CompToExpandPt: the connected componant to expand
 * @param[In] GoalCompPt: the goal componant that we want to reach. Can
 * be used to bias th expansion. Can be set to NULL if we don't wawnt any bias.
 * @param[Out]: DirectionConfig: the configuration selected as 
 * a direction of expansion
 * @return: TRUE if a direction of expansion has been found 
 * FALSE  otherwise.
 */
int SelectExpansionDirection(p3d_graph* GraphPt,  p3d_compco* CompToExpandPt, 
			     p3d_compco* GoalCompPt, configPt DirectionConfig, 
			     int ArePassiveDofsSampled);

#endif
