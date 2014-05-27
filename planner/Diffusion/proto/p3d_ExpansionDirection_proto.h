/*
 * Copyright (c) 2001-2014 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 */
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
