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

#define UNSET  0                     /* for AABB and OBB */
#define MOVED_BACKWARD   1           /* for AABB */
#define MOVED_FORWARD   2            /* for AABB */
#define UNCHANGED  3                 /* for AABB and OBB */
#define MOVED  4                     /* for OBB */

#define WORK  1                      /* OBB defined w.r.t. workframe */
#define LOCAL  2                     /* OBB defined w.r.t. local frame */

/* what is inside */
#define WAS_POLYHEDRON  1            /* (O/AA)BB sits around polyhedron */
#define WAS_SOLID  2                 /* (O/AA)BB sits around a solid */
#define WAS_BBS   3                  /* (O/AA)BB sits around at least 1 other BB */


#define GO_DOWN_ROBOT_TREE     1
#define GO_DOWN_BOTH_TREES     2
#define GO_DOWN_2_ROBOT_TREES  3


/* entity attributes */
/* Begin modif Pepijn june 2001 */
/* 
 * KCD_SMALL_VOLUME_BOX is used as an entity type attribute
 * and not as the whats_inside attribute
 */
#define KCD_SMALL_VOLUME_BOX 8
#define KCD_OBB 9 /* used for exact distance between OBB's*/

#define KCD_AABB 10


/* end modif Pepijn june 2001 */

#define AABB_BOXTYPE  1              /* box type is AABB */
#define OBB_BOXTYPE  2               /* box type is OBB  */

#define OBB_FLAT  0
#define OBB_SPHERICAL  1
#define OBB_TALL  2



