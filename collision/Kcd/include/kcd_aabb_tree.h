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
#ifndef KCD_AABB_HIER_H
#define KCD_AABB_HIER_H

/* #include "kcd.h" */

typedef struct axis_list_el{
  double val;                  /* key on which this element is sorted */
  int the_bb;                  /* AABB from which val originates */
}axis_list_el,*axis_list_elp,**axis_list_elpp;

typedef struct aabb_hier_el{
  int the_bb;                      /* AABB around the group of AABBs in lists, nr in all_bbs */
  int this_hier_level;
  int this_place_at_level;
  axis_list_elp *aabb_xlist;
  int aabb_xlist_nofels;
  axis_list_elp *aabb_ylist;
  int aabb_ylist_nofels;
  axis_list_elp *aabb_zlist;
  int aabb_zlist_nofels;
  /* children of type aabb_hier_el */
  int group_below_nr;                    /* nr of children */
  int *group_below_aabb_hier_el;         /* indices on lower level */
}aabb_hier_el;

typedef struct hash_on_aabb_axis_list
{
  double            hashval;
  axis_list_el     *hashed_el;
}hash_on_aabb_axis_list;

typedef aabb_hier_el     *kcd_aabb_list ;
typedef kcd_aabb_list    *kcd_aabb_lists;


#endif

