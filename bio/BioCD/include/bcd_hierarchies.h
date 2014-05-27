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
/*!			bcd_hierarchies.h
 *
 *  
 *
 ****************************************************************************/
 
#if !defined (BCD_HIERARCHIES)
#define BCD_HIERARCHIES

/* user-level functions : BEGIN +++++++++++++++++++ */
void bio_set_surface_d(double distance);
double min_dist_report(p3d_poly **poly1, p3d_poly **poly2);
void biocol_report(int *col_number, p3d_poly ***list1, p3d_poly ***list2);
void bio_set_col_mode(int mode);
void set_required_collisions(int rc);
void set_required_collisions_to_max(void);
/*user-level functions : END+++++++++++++++++++++++++++++++++++++++++++++*/


int bio_get_col_mode(void);
void set_minimum_Sdistance_to_max(void);
void set_n_collisions_to_zero(void);
int get_n_collisions(void);
int too_much_collisions(void);
boxnodept create_rigid_root(Rigid_structure *rigido);
void bio_create_molecule_root(Robot_structure *robot);
void sup_collision(supnodept node1, supnodept node2);
void box_coll_withinProt(Bboxpt b1, Bboxpt b2);
void box_coll_general(Bboxpt b1, Bboxpt b2);
void freeboxhierarchy(Robot_structure *robot, int i);
void freeboxhierarchy2(Rigid_structure *r);
void freesuphierarchy(Robot_structure *robot);
void my_robot_autocollision(Robot_structure *robot);


#endif
