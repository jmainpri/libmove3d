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
/* 
 *  This file includes the protoypes of the functions defined in the
 *  directory collision and the files defining the structure appearing in 
 *  these prototypes.
 */

#ifndef COLLISION_PKG_H
#define COLLISION_PKG_H

#ifdef UNIX
#endif


/* struct */
#include "p3d_sys.h"
#include "p3d_type.h"
#include "p3d_matrix.h"
#include "polyhedre.h"
#include "p3d_poly.h"
#include "p3d.h"
#include "collision_context.h"

#include "traj.h"
#include "localpath.h"
#include "device.h"

#ifdef VCOLLIDE_ACT
#include "../collision/Vcollide/src/VCol.h"
#endif

#include "../collision/Kcd/include/kcd_api.h"
#include "../collision/Kcd/include/kcd_type.h"
#include "../collision/Kcd/include/kcd.h"
#include "../collision/Kcd/include/kcd_aabb_tree.h"



/* globals */

extern int COLLISION_BY_OBJECT;

#ifdef VCOLLIDE_ACT
extern void *vc_hand;			/* the VCOLLIDE collision detection engine */
extern VCReportType *vc_colrep; /* the VCOLLIDE collision report */
#endif

#include "../collision/Kcd/include/kcd_global.h"  /* Kcd */
#include "../collision/Kcd/include/kcd_aabb_tree_global.h"  /* Kcd */


/* proto */

#include "triangles.h"
#include "../collision/proto/collision_proto.h"

#include "../collision/Vcollide/src/VCol.h"
#include "../collision/Kcd/proto/kcd_proto.h"

#ifdef UNIX
#endif

#endif /* #ifndef COLLISION_PKG_H */
