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
#ifndef DEFS_H
#define DEFS_H

/**********************************************************************/
// DEFINITIONS

#define SQR(x) ((x)*(x))
#define PI 3.14159265358979323846
#define FUZZ  0.00001
#define ABS(x)  (((x) >= 0.0) ? (x) : -(x))
#define EQ(x,y) (ABS((x)-(y)) < FUZZ)

/////////////////////////////////
#define OMEGAMAX 190.0*(PI/180.0)
#define OMEGAMIN 170.0*(PI/180.0)
/////////////////////////////////
#define C_VDWR 1.53
#define N_VDWR 1.38
#define O_VDWR 1.35
#define S_VDWR 1.79
#define H_VDWR 1.10

#define Br_VDWR 1.94
#define Cl_VDWR 1.74
#define F_VDWR 1.29

/* #define C_VDWR 1.7 */
/* #define N_VDWR 1.55 */
/* #define O_VDWR 1.52 */
/* #define S_VDWR 1.8 */
/* #define H_VDWR 1.2 */
/////////////////////////////////
#define N_COLOR "Blue"
//#define CA_COLOR "DGreen"
#define C_COLOR "Green"
#define O_COLOR "Red"
#define S_COLOR "Yellow"
#define H_COLOR "White"
/////////////////////////////////

// ATOM NAME STANDARDS
#define INSIGHT_NAMES 0  // default
#define AMBER_NAMES   1

#endif
