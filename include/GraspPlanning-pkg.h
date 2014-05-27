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
 *  This file includes the prototypes of the functions defined in the
 *  directory graspPlanning and the files defining the structure appearing in
 *  these prototypes.
 */
#ifdef GRASP_PLANNING
#ifndef GRASP_PLANNING_PKG_H
#define GRASP_PLANNING_PKG_H

#include "p3d_sys.h"
#include "p3d_type.h"
#include "p3d_matrix.h"
#include "p3d.h"


#include "../graspPlanning/include/graspPlanning.h"
#include "../graspPlanning/include/gpContact.h"
#include "../graspPlanning/include/gpGrasp.h"
#include "../graspPlanning/include/gp_volInt.h"
#include "../graspPlanning/include/gpKdTree.h"
#include "../graspPlanning/include/gpConvexHull.h"
#include "../graspPlanning/include/gp_grasp_io.h"
#include "../graspPlanning/include/gpWorkspace.h"
#include "../graspPlanning/include/gpPlacement.h"

#include "../graspPlanning/proto/gp_grasping_utils_proto.h"
#include "../graspPlanning/proto/gp_grasp_generation_proto.h"
#include "../graspPlanning/proto/gp_extensionsM3D_proto.h"
#include "../graspPlanning/proto/gp_force_closure_proto.h"
#include "../graspPlanning/proto/gp_geometry_proto.h"
#include "../graspPlanning/proto/gp_inertia_axes_proto.h"
#include "../graspPlanning/proto/gp_volInt_proto.h"
#include "../graspPlanning/proto/FormgraspPlanning_proto.h"



#endif
#endif
