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
 * p3d_SetDistConfigChoice 
 * Set the current choice of metric used to compute 
 * the distance between
 * two configurations   
 * @param[In] DistConfChoice: the selected choice of 
 * metric to compute the distance.
 */
void p3d_SetDistConfigChoice(int DistConfChoice);

/**
 * p3d_GetDisConfigChoice 
 * Get the current metric used to compute 
 * the distance between two configurations   
 * @return:  the current metric used to compute 
 * the distance.
 */
int p3d_GetDistConfigChoice(void);

/**
 * SelectedDistConfig
 * Compute the distance between two robot configurations using the
 * current metric defined by  p3d_SetDisConfigChoice
 * @param[in] robotPt: a pointer to the current robot
 * @param[in] Config1: the first configuration
 * @param[in] Config2: the second configuration
 * @return: the distance between the two configurations,
 * and -1. if the computation of the ditances failed
 */
double SelectedDistConfig(p3d_rob* robotPt, configPt Config1,
			  configPt Config2);

#endif
