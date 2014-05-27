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
 *  directory localpath and the files defining the structure appearing in 
 *  these prototypes.
 */

#ifndef LOCALPATH_PKG_H
#define LOCALPATH_PKG_H

#ifdef UNIX
#endif

#define SOFT_MOTION_PRINT_DATA 0

/* struct */
#include "p3d_sys.h"
#include "p3d_type.h"
#include "p3d_matrix.h"
#include "polyhedre.h"
#include "p3d_poly.h"
#include "p3d.h"

#include "traj.h"
#include "localpath.h"
#include "device.h"

#include "rs.h"
#include "../localpath/flat/include/struct_flat.h"


/* Arm models */
#include "../include/arm_models/pa10Const.h"

/* globals */


/* proto */
#include "../localpath/flat/include/init_flat_proto.h"
#include "../localpath/flat/include/general_flat_proto.h"
#include "../localpath/flat/include/kinematic_flat_proto.h"

// gbM  XB
#if defined(USE_GBM)
extern "C"{
#include "gbM/Proto_gb.h"
#include "gbM/Proto_gbModeles.h"
#include "gbM/gbStruct.h"
}
#endif
#if defined(MULTILOCALPATH)
#include "softMotion/softMotionStruct.h"
#include "softMotion/softMotion.h"

#endif

#include "../localpath/proto/localpath_proto.h"


#ifdef UNIX
#endif

#endif /* #ifndef LOCALPATH_PKG_H */
