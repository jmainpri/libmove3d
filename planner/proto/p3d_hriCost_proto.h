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
extern void p3d_SetIsHriFuncSpace(int IsHriFuncSpace);
extern int p3d_GetIsHriFuncSpace();
extern void p3d_SetHriZoneSize(double size);
extern void p3d_AddHriZone();
extern double p3d_GetHriZoneSize();
extern double* p3d_GetVectJim();
extern void p3d_DeactivateAllButHri(int disp);
extern void p3d_ActivateAll(int disp);
extern double p3d_GetHriDistCost(p3d_rob* robotPt, int disp);
extern void p3d_SaveGPlotCostTraj(void);
extern void p3d_SaveGPlotCostNodeChro(void);
extern double p3d_GetLpHriDistCost(p3d_rob* robotPt, p3d_localpath * lp);
// void p3d_SetCostToTab(p3d_rob *robotPt,  conf_cost * tab, int nbPart);
// void p3d_WriteConfCostToCSV(FILE *fd, conf_cost *tab, int size);
// void p3d_WriteConfCostToOBPlane(FILE *fd, conf_cost *tab, int size);
