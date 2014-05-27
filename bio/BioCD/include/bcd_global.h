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

/****************************************************************************/
/*!		bcd_global.h
 *
 *   bio structures
 *
 ****************************************************************************/

 
#if !defined (BCD_GLOBAL)
#define BCD_GLOBAL
#include "P3d-pkg.h"
#include "Bio-pkg.h"

extern int number_of_bcd_robots; /* number of robots in the bio collision detector */

/* array of robots in the bio collision detector */
extern Robot_structure *bcd_robots[]; 

/* table with the number of robots in a p3d-rob */
extern int num_subrobots[]; /* Stores the number of subrobots of
each p3d_rob */
extern int robot_p3d[]; /* robot_p3d[N] indicates the first
	subrobot corresponding to p3d_rob number N */
  
extern int HYDROGEN_ENVIRONEMENT;



/**********************************************************************************
TYPES AND VARIABLES FOR THE HIERARCHIES
**********************************************************************************/
/**********************************************************************************/

extern check_pair_function check_pair;
extern check_all_function check_all;
 
extern void (*box_collision)(Bboxpt, Bboxpt);
/*trash*/
int giveme_mininhierarchy();

/*************************************************************************************/

extern int charlatan;
#endif





	













