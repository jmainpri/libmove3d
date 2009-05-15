#ifndef __CEXTRACT__

/**
 * Get the maximal number of try to expand
 * the passive nodes during a Manhattan 
 * like expansion.
 * @return: The maximal number of try to expand
 * the passive nodes
 */ 
int p3d_GetMaxPassiveExpand(void);

/**
 * Set the maximal number of try to expand
 * the passive nodes during a Manhattan 
 * like expansion.
 * @param[In] MaxPassiveExpand: Maximal number of try to expand
 * the passive nodes
 */ 
void p3d_SetMaxPassiveExpand(int MaxPassiveExpand);

/**
 * p3d_GetIsPasExtWhenAct
 * Function get if we allow or not the expansion
 * of the passive parameters only when the expension of
 * the active ones succeeded
 * return:TRUE if  we allow 
 * the expansion of the passive parameters only when 
 * the expension of the active ones succeeded
 */
int p3d_GetIsPasExtWhenAct(void);

/** 
 * p3d_GetIsPasExtWhenAct
 * Function get if we allow or not the expansion
 * of the passive parameters only when the expension of
 * the active ones succeeded
 * param[In] IsPasExtWhenAct: TRUE if  we allow 
 * the expansion of the passive parameters only when 
 * the expension of the active ones succeeded
 */
void p3d_SetIsPasExtWhenAct(int IsPasExtWhenAct);

/** 
 * p3d_GetIsManhatExpansion
 * This function return  TRUE if the expansion
 * is a Manhattan like expansion:
 * In a first step only the active parameters are expanded
 * then we try to expand the passive parameter by recursivly
 * expanding  only the passive parameters which were in collision
 * during the previsous expansion.
 * @return:  TRUE if the expansion  is a Manhattan like expansion
 */
int p3d_GetIsManhatExpansion(void);

/** 
 * p3d_GetIsManhatExpansion
 * This function set   if the expansion is a Manhattan like expansion:
 * In a first step only the active parameters are expanded
 * then we try to expand the passive parameter by recursivly
 * expanding  only the passive parameters which were in collision
 * during the previsous expansion.
 * @param[In]: IsManhattanExp  TRUE if the expansion is a Manhattan 
 * like expansion
 */
void p3d_SetIsManhatExpansion(int IsManhattanExp);

/**
 * p3d_GetManhattanRatio
 * Get the ratio giving the amount of Manhattan
 * expansion against classical expansions when
 * the flag IS_MANHATTAN_EXPANSION is TRUE
 */
double p3d_GetManhattanRatio(void);

/**
 * p3d_SetManhattanRatio
 * Set the ratio giving the amount of Manhattan
 * expansion against classical expansions when
 * the flag IS_MANHATTAN_EXPANSION is TRUE
 */
void p3d_SetManhattanRatio(double ManhatExpanRatio);


/**
 * p3d_ExpanBlockedByColl
 * Todo
 */
int p3d_ExpanBlockedByColl(p3d_rob *robotPt, configPt *qinvPt);

/**
* p3d_perturb_and_check_passive_params_involved_in_collision
* Todo. function no more used
*/
int p3d_perturb_and_check_passive_params_involved_in_collision(p3d_rob *robotPt,
							       configPt qinv);

/**
 * p3d_SelectNewJntInList
 * todo
 */
// NOTE : this function allocates memory that must be freed later
int p3d_SelectNewJntInList(p3d_rob *robotPt, int nJ, p3d_jnt **Jlist,
			   int *old_nJPt, p3d_jnt ***old_JlistPt, 
			   int *new_nJPt, p3d_jnt ***new_JlistPt);

/**
 * p3d_GetCollidingtPassiveJntList
 * Todo
 * NOTE : this function allocates memory that must be freed later
 */
int p3d_GetCollidingtPassiveJntList(p3d_rob *robotPt, configPt qinv, 
				     int *npassJPt, p3d_jnt ***passJlistPt);

/**
 * p3d_shoot_jnt_list_and_copy_into_conf
 * Todo. If general function, should probably be
 * moved to p3d_sample.c
 */
void p3d_shoot_jnt_list_and_copy_into_conf(p3d_rob *robotPt, configPt qref, configPt qrand, 
					   int npassJ, p3d_jnt **passJlist);


/**
 * p3d_free_list_of_joints
 * Todo. If general function, should  be
 * moved to p3d/p3d_joint.c
 */
void p3d_free_list_of_joints(p3d_rob *robotPt, int *nJ, p3d_jnt ***Jlist);


/**
 * p3d_copy_passive_config_into
 * Todo
 */
void p3d_copy_passive_config_into(p3d_rob *robotPt, configPt qsrc, configPt qdst);

/**
 * p3d_PassivExpandProcess
 * Expand the passivee configuration parameters after
 * an expansion of active parameters
 * @param[In] GraphPt: Pointer to the robot graph
 * @param[In] ExpansionNodePt: Node from which the active params have been extended.
 * @param[In] NbActiveNodesCreated: Number of nodes created during 
 * the expansion of the active parameters
 * @return: The number of nodes created during the expansion of passive dofs. 
 * Note:  the random directions choosen to expand the passive 
 * dofs are independant of the ExpansionDirectionMethod. It directly uses
 * the p3d_shoot_jnt_list_and_copy_into_conf function.
 */ 
int p3d_PassivExpandProcess(p3d_graph* GraphPt, 
			    p3d_node*  ExpansionNodePt, int NbActiveNodesCreated);

#endif
