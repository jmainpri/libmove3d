#ifndef __CEXTRACT__

#define Manha_DEBUG 0

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

int getCollidingPassiveJntList(p3d_rob *robotPt, configPt qinv,
			       std::vector<p3d_jnt*>& joints);

int selectNewJntInList(p3d_rob *robotPt, std::vector<p3d_jnt*>& joints,
			   std::vector<p3d_jnt*>& oldJoints, std::vector<p3d_jnt*>& newJoints);

void shoot_jnt_list_and_copy_into_conf(p3d_rob *robotPt, configPt qrand,
				       std::vector<p3d_jnt*>& joints);

#endif
