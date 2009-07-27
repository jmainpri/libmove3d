#ifndef __CEXTRACT__

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
