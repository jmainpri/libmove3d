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
